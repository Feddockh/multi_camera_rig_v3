/*
Run with:
    ros2 run firefly-ros2-wrapper-bringup foundation_stereo_points_node --ros-args \
    -p engine_path:=/tmp/test.plan \
    -p baseline:=0.06 \
    -p stride:=2 \
    -p max_range_m:=10.0
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

static inline void checkCuda(cudaError_t e, const char *msg)
{
    if (e != cudaSuccess)
    {
        throw std::runtime_error(std::string(msg) + ": " + cudaGetErrorString(e));
    }
}

class TrtLogger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
        {
            std::cerr << "[TRT] " << msg << std::endl;
        }
    }
};

static std::vector<char> readFile(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("Failed to open engine: " + path);
    f.seekg(0, std::ios::end);
    size_t n = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> data(n);
    f.read(data.data(), n);
    return data;
}

static size_t dtypeBytes(nvinfer1::DataType t)
{
    switch (t)
    {
    case nvinfer1::DataType::kFLOAT:
        return 4;
    case nvinfer1::DataType::kHALF:
        return 2;
    case nvinfer1::DataType::kINT8:
        return 1;
    case nvinfer1::DataType::kINT32:
        return 4;
    case nvinfer1::DataType::kBOOL:
        return 1;
    default:
        return 0;
    }
}

static size_t volume(const nvinfer1::Dims &d)
{
    size_t v = 1;
    for (int i = 0; i < d.nbDims; ++i)
        v *= static_cast<size_t>(d.d[i]);
    return v;
}

struct IoTensor
{
    std::string name;
    nvinfer1::Dims shape{};
    nvinfer1::DataType dtype{};
    void *dptr{nullptr};
    size_t bytes{0};
};

class TrtRunner
{
public:
    explicit TrtRunner(const std::string &engine_path)
    {
        logger_ = std::make_unique<TrtLogger>();

        auto blob = readFile(engine_path);
        runtime_.reset(nvinfer1::createInferRuntime(*logger_));
        if (!runtime_)
            throw std::runtime_error("createInferRuntime failed");

        engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
        if (!engine_)
            throw std::runtime_error("deserializeCudaEngine failed");

        context_.reset(engine_->createExecutionContext());
        if (!context_)
            throw std::runtime_error("createExecutionContext failed");

        checkCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");

        // Allocate for all IO tensors
        int n = engine_->getNbIOTensors();
        for (int i = 0; i < n; ++i)
        {
            const char *tn = engine_->getIOTensorName(i);
            IoTensor t;
            t.name = tn;
            t.shape = engine_->getTensorShape(tn);
            t.dtype = engine_->getTensorDataType(tn);
            t.bytes = volume(t.shape) * dtypeBytes(t.dtype);
            checkCuda(cudaMalloc(&t.dptr, t.bytes), ("cudaMalloc " + t.name).c_str());
            idx_[t.name] = tensors_.size();
            tensors_.push_back(t);
        }

        // Sanity check your known names
        requireTensor("left");
        requireTensor("right");
        requireTensor("disp");
    }

    ~TrtRunner()
    {
        for (auto &t : tensors_)
        {
            if (t.dptr)
                cudaFree(t.dptr);
        }
        if (stream_)
            cudaStreamDestroy(stream_);
    }

    nvinfer1::Dims shapeOf(const std::string &name) const
    {
        return tensors_.at(idx_.at(name)).shape;
    }

    void run(const float *h_left_nchw, const float *h_right_nchw, float *h_disp,
             size_t disp_floats)
    {
        auto &left = tensors_.at(idx_.at("left"));
        auto &right = tensors_.at(idx_.at("right"));
        auto &disp = tensors_.at(idx_.at("disp"));

        // H2D
        checkCuda(cudaMemcpyAsync(left.dptr, h_left_nchw, left.bytes,
                                  cudaMemcpyHostToDevice, stream_),
                  "H2D left");
        checkCuda(cudaMemcpyAsync(right.dptr, h_right_nchw, right.bytes,
                                  cudaMemcpyHostToDevice, stream_),
                  "H2D right");

        // Bind
        int n = engine_->getNbIOTensors();
        for (int i = 0; i < n; ++i)
        {
            const char *tn = engine_->getIOTensorName(i);
            auto &t = tensors_.at(idx_.at(tn));
            context_->setTensorAddress(tn, t.dptr);
        }

        if (!context_->enqueueV3(stream_))
        {
            throw std::runtime_error("enqueueV3 failed");
        }

        // D2H output
        const size_t out_bytes = disp_floats * sizeof(float);
        if (out_bytes > disp.bytes)
        {
            throw std::runtime_error("disp output buffer too small");
        }
        checkCuda(cudaMemcpyAsync(h_disp, disp.dptr, out_bytes,
                                  cudaMemcpyDeviceToHost, stream_),
                  "D2H disp");
        checkCuda(cudaStreamSynchronize(stream_), "stream sync");
    }

private:
    void requireTensor(const std::string &name) const
    {
        if (!idx_.count(name))
            throw std::runtime_error("Engine missing tensor: " + name);
    }

    struct TRTDeleter
    {
        template <typename T>
        void operator()(T *p) const noexcept
        {
            delete p; // TensorRT 10+ uses standard delete (destroy() removed)
        }
    };

    std::unique_ptr<TrtLogger> logger_;
    std::unique_ptr<nvinfer1::IRuntime, TRTDeleter> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine, TRTDeleter> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext, TRTDeleter> context_;
    cudaStream_t stream_{};

    std::vector<IoTensor> tensors_;
    std::unordered_map<std::string, size_t> idx_;
};

static std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });
    return s;
}

static rmw_qos_reliability_policy_t parseReliability(const std::string &s)
{
    const auto v = toLower(s);
    if (v == "reliable")
        return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    if (v == "best_effort" || v == "besteffort" || v == "best-effort")
        return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    if (v == "system_default" || v == "default")
        return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    throw std::runtime_error("Invalid reliability: " + s);
}

static rmw_qos_durability_policy_t parseDurability(const std::string &s)
{
    const auto v = toLower(s);
    if (v == "volatile")
        return RMW_QOS_POLICY_DURABILITY_VOLATILE;
    if (v == "transient_local" || v == "transientlocal" || v == "transient-local")
        return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    if (v == "system_default" || v == "default")
        return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    throw std::runtime_error("Invalid durability: " + s);
}

static rmw_qos_history_policy_t parseHistory(const std::string &s)
{
    const auto v = toLower(s);
    if (v == "keep_last" || v == "keeplast")
        return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    if (v == "keep_all" || v == "keepall")
        return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    if (v == "system_default" || v == "default")
        return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    throw std::runtime_error("Invalid history: " + s);
}

// Build an rclcpp::QoS from user strings (Humble API)
static rclcpp::QoS makeQos(
    const std::string &reliability,
    const std::string &durability,
    const std::string &history,
    int depth)
{
    auto hist = parseHistory(history);

    // rclcpp::QoS requires an initialization; use KeepLast(depth) for KEEP_LAST,
    // and KeepAll() for KEEP_ALL.
    rclcpp::QoS qos =
        (hist == RMW_QOS_POLICY_HISTORY_KEEP_ALL)
            ? rclcpp::QoS(rclcpp::KeepAll())
            : rclcpp::QoS(rclcpp::KeepLast(std::max(1, depth)));

    // Apply reliability
    switch (parseReliability(reliability))
    {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        qos.reliable();
        break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
        qos.best_effort();
        break;
    default: /* SYSTEM_DEFAULT */
        break;
    }

    // Apply durability
    switch (parseDurability(durability))
    {
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
        qos.durability_volatile();
        break;
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
        qos.transient_local();
        break;
    default: /* SYSTEM_DEFAULT */
        break;
    }

    return qos;
}

class FoundationStereoMatcherNode : public rclcpp::Node
{
public:
    FoundationStereoMatcherNode() : Node("foundation_stereo_matcher_node")
    {
        engine_path_ = declare_parameter<std::string>("engine_path", "");
        baseline_ = declare_parameter<double>("baseline", 0.06);

        left_topic_ = declare_parameter<std::string>("left_image_topic", "/firefly_left/image_rect");
        right_topic_ = declare_parameter<std::string>("right_image_topic", "/firefly_right/image_rect");
        info_topic_ = declare_parameter<std::string>("left_info_topic", "/firefly_left/camera_info");

        // Point cloud density control (very useful for CPU)
        stride_ = declare_parameter<int>("stride", 2); // 1=full model res, 2=quarter points, 4=1/16 points
        max_range_m_ = declare_parameter<double>("max_range_m", 10.0);
        use_background_ = declare_parameter<bool>("use_background", false);

        // Outputs + topics
        publish_cloud_ = declare_parameter<bool>("publish_cloud", true);
        publish_depth_ = declare_parameter<bool>("publish_depth", true);
        publish_disparity_ = declare_parameter<bool>("publish_disparity", false);
        cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/stereo/points");
        depth_topic_ = declare_parameter<std::string>("depth_topic", "/stereo/depth");
        disparity_topic_ = declare_parameter<std::string>("disparity_topic", "/stereo/disparity");

        // QoS params for pubs/subs
        sub_rel_ = declare_parameter<std::string>("sub_qos.reliability", "reliable");
        sub_dur_ = declare_parameter<std::string>("sub_qos.durability", "volatile");
        sub_hist_ = declare_parameter<std::string>("sub_qos.history", "keep_last");
        sub_depth_ = declare_parameter<int>("sub_qos.depth", 5);
        pub_rel_ = declare_parameter<std::string>("pub_qos.reliability", "best_effort");
        pub_dur_ = declare_parameter<std::string>("pub_qos.durability", "volatile");
        pub_hist_ = declare_parameter<std::string>("pub_qos.history", "keep_last");
        pub_depth_ = declare_parameter<int>("pub_qos.depth", 5);

        // Disparity filtering options
        disp_filter_mode_ = declare_parameter<std::string>("disp_filter.mode", "none");
        // modes: none | median | bilateral | speckle | edge_flying_kill
        // median
        disp_median_ksize_ = declare_parameter<int>("disp_filter.median_ksize", 5); // odd
        // bilateral
        disp_bilat_d_ = declare_parameter<int>("disp_filter.bilateral_d", 7);
        disp_bilat_sc_ = declare_parameter<double>("disp_filter.bilateral_sigma_color", 3.0);
        disp_bilat_ss_ = declare_parameter<double>("disp_filter.bilateral_sigma_space", 7.0);
        // speckle
        disp_speckle_max_size_ = declare_parameter<int>("disp_filter.speckle_max_size", 120);
        disp_speckle_range_ = declare_parameter<double>("disp_filter.speckle_range", 1.0);  // in disparity pixels
        disp_speckle_scale_ = declare_parameter<double>("disp_filter.speckle_scale", 16.0); // float->fixed
        // edge flying pixel removal
        disp_edge_ksize_ = declare_parameter<int>("disp_filter.edge_ksize", 5);   // odd neighborhood
        disp_edge_tau_ = declare_parameter<double>("disp_filter.edge_tau", 0.20); // 20% outlier from local median
        disp_edge_min_neigh_ = declare_parameter<int>("disp_filter.edge_min_neighbors", 6);

        // Depth filtering options
        depth_filter_mode_ = declare_parameter<std::string>("depth_filter.mode", "none");
        // modes: none | flying_pixel | median
        // flying pixel
        depth_flying_ksize_ = declare_parameter<int>("depth_filter.flying_ksize", 5);   // odd
        depth_flying_tau_ = declare_parameter<double>("depth_filter.flying_tau", 0.25); // relative deviation
        depth_flying_min_neigh_ = declare_parameter<int>("depth_filter.flying_min_neighbors", 6);
        // median
        depth_median_ksize_ = declare_parameter<int>("depth_filter.median_ksize", 5); // odd

        // Point cloud filtering options
        pc_filter_mode_ = declare_parameter<std::string>("pc_filter.mode", "none");
        // modes: none | grid_outlier | knn_outlier
        // grid outlier
        pc_grid_ksize_ = declare_parameter<int>("pc_filter.grid_ksize", 5);
        pc_grid_tau_ = declare_parameter<double>("pc_filter.grid_tau", 0.25);
        pc_grid_min_neigh_ = declare_parameter<int>("pc_filter.grid_min_neighbors", 4);
        // knn outlier
        pc_knn_k_ = declare_parameter<int>("pc_filter.knn_k", 20);
        pc_knn_stddev_mul_ = declare_parameter<double>("pc_filter.knn_stddev_multiplier", 2.0);

        // Create the subscribers
        auto sub_qos = makeQos(sub_rel_, sub_dur_, sub_hist_, sub_depth_);
        auto pub_qos = makeQos(pub_rel_, pub_dur_, pub_hist_, pub_depth_);

        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, sub_qos, std::bind(&FoundationStereoMatcherNode::onInfo, this, std::placeholders::_1));
        left_sub_.subscribe(this, left_topic_, sub_qos.get_rmw_qos_profile());
        right_sub_.subscribe(this, right_topic_, sub_qos.get_rmw_qos_profile());

        // Create the publishers (if enabled)
        if (publish_cloud_)
            cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, pub_qos);
        if (publish_depth_)
            depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, pub_qos);
        if (publish_disparity_)
            disp_pub_ = create_publisher<sensor_msgs::msg::Image>(disparity_topic_, pub_qos);

        // using Policy = message_filters::sync_policies::ApproximateTime<
        //     sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        using Policy = message_filters::sync_policies::ExactTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(5), left_sub_, right_sub_);
        sync_->registerCallback(std::bind(&FoundationStereoMatcherNode::onStereo, this,
                                          std::placeholders::_1, std::placeholders::_2));

        if (engine_path_.empty())
            throw std::runtime_error("engine_path param is empty");
        runner_ = std::make_unique<TrtRunner>(engine_path_);

        const auto in = runner_->shapeOf("left"); // ex (1,3,672,896)
        in_h_ = in.d[2];
        in_w_ = in.d[3];

        const auto out = runner_->shapeOf("disp"); // ex (1,1,672,896)
        out_h_ = out.d[2];
        out_w_ = out.d[3];

        // host buffers (optionally switch to pinned with cudaMallocHost for better H2D speed)
        left_nchw_.resize(3 * in_h_ * in_w_);
        right_nchw_.resize(3 * in_h_ * in_w_);
        disp_.resize(static_cast<size_t>(out_h_ * out_w_));

        RCLCPP_INFO(get_logger(), "Engine IO: left/right (1,3,%d,%d) disp (1,1,%d,%d)",
                    in_h_, in_w_, out_h_, out_w_);
    }

private:
    void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        cam_w_ = msg->width;
        cam_h_ = msg->height;
        have_info_ = true;

        // Check that msg->width/height matches engine input
        if ((int)msg->width != in_w_ || (int)msg->height != in_h_)
        {
            RCLCPP_WARN(get_logger(),
                        "CameraInfo size (%dx%d) does not match engine input (%dx%d)",
                        msg->width, msg->height, in_w_, in_h_);
        }
    }

    static inline void bgrToNCHWFloat(const cv::Mat &bgr, int H, int W, std::vector<float> &out)
    {
        // out: C*H*W contiguous floats
        const int HW = H * W;
        out.resize(3 * HW);

        // Faster than split+convertTo: manual loop (still CPU, but avoids extra Mats)
        // bgr is CV_8UC3
        const uint8_t *p = bgr.ptr<uint8_t>(0);
        float *outB = out.data() + 0 * HW;
        float *outG = out.data() + 1 * HW;
        float *outR = out.data() + 2 * HW;

        for (int i = 0; i < HW; ++i)
        {
            outB[i] = static_cast<float>(p[3 * i + 0]);
            outG[i] = static_cast<float>(p[3 * i + 1]);
            outR[i] = static_cast<float>(p[3 * i + 2]);
        }
    }

    void onStereo(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        if (!have_info_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for CameraInfo...");
            return;
        }

        const auto t0 = now();

        cv_bridge::CvImageConstPtr left_cv, right_cv;
        try
        {
            left_cv = cv_bridge::toCvShare(left_msg, "bgr8");
            right_cv = cv_bridge::toCvShare(right_msg, "bgr8");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge failed: %s", e.what());
            return;
        }

        // Resize to engine input (and we will sample RGB from left_rs)
        cv::Mat left_rs, right_rs;
        // Expect inputs already match the engine input resolution.
        if ((int)left_cv->image.cols != in_w_ || (int)left_cv->image.rows != in_h_ ||
            (int)right_cv->image.cols != in_w_ || (int)right_cv->image.rows != in_h_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Input image size (%dx%d) does not match engine (%dx%d). "
                                 "Use stereo_rectify_scale_node or adjust output size.",
                                 left_cv->image.cols, left_cv->image.rows, in_w_, in_h_);
            return;
        }

        // Run inference
        bgrToNCHWFloat(left_cv->image, in_h_, in_w_, left_nchw_);
        bgrToNCHWFloat(right_cv->image, in_h_, in_w_, right_nchw_);
        const auto t1 = now();
        runner_->run(left_nchw_.data(), right_nchw_.data(), disp_.data(), disp_.size());
        const auto t2 = now();

        // ------------------ DISPARITY FILTER BLOCK ------------------
        auto mode = toLower(disp_filter_mode_);
        cv::Mat disp_mat(out_h_, out_w_, CV_32FC1, disp_.data());

        // Make a working copy so we don't mutate disp_ unless we choose to:
        cv::Mat disp_filt = disp_mat.clone();

        // Normalize invalids: treat <=0 as invalid
        // (keeps your existing logic consistent)
        for (int v = 0; v < disp_filt.rows; ++v)
        {
            float *row = disp_filt.ptr<float>(v);
            for (int u = 0; u < disp_filt.cols; ++u)
            {
                if (!(row[u] > 0.0f))
                    row[u] = 0.0f;
            }
        }

        if (mode == "median")
        {
            int k = std::max(3, disp_median_ksize_ | 1); // force odd
            RCLCPP_INFO_ONCE(get_logger(), "[Disparity Filter] Using median filter with ksize=%d", k);
            cv::medianBlur(disp_filt, disp_filt, k);
            // NOTE: medianBlur will spread zeros a bit; that's often OK for flying pixels.
        }
        else if (mode == "bilateral")
        {
            // Bilateral works best when disparity is in a reasonable numeric range.
            // If disp values are large, consider scaling temporarily.
            RCLCPP_INFO_ONCE(get_logger(), "[Disparity Filter] Using bilateral filter with d=%d, sigma_color=%.1f, sigma_space=%.1f",
                             disp_bilat_d_, disp_bilat_sc_, disp_bilat_ss_);
            cv::Mat tmp;
            disp_filt.convertTo(tmp, CV_32FC1);
            cv::bilateralFilter(tmp, disp_filt, disp_bilat_d_, disp_bilat_sc_, disp_bilat_ss_);
        }
        else if (mode == "speckle")
        {
            // OpenCV's filterSpeckles expects fixed-point disparity (typically CV_16S).
            // We'll convert float disparity -> fixed-point, filter speckles, convert back.
            const double S = std::max(1.0, disp_speckle_scale_);
            RCLCPP_INFO_ONCE(get_logger(), "[Disparity Filter] Using speckle filter with max_size=%d, range=%.1f, scale=%.1f",
                             disp_speckle_max_size_, disp_speckle_range_, S);
            cv::Mat disp_16s(disp_filt.size(), CV_16S);

            for (int v = 0; v < disp_filt.rows; ++v)
            {
                const float *src = disp_filt.ptr<float>(v);
                int16_t *dst = disp_16s.ptr<int16_t>(v);
                for (int u = 0; u < disp_filt.cols; ++u)
                {
                    const float d = src[u];
                    dst[u] = (d > 0.0f) ? (int16_t)std::lround(d * S) : (int16_t)0;
                }
            }

            // Remove small connected components with similar disparity values
            cv::filterSpeckles(
                disp_16s,
                0,                                        // newVal for removed speckles
                disp_speckle_max_size_,                   // max speckle size
                (int)std::lround(disp_speckle_range_ * S) // max disparity variation within speckle
            );

            // Convert back to float disparity
            for (int v = 0; v < disp_filt.rows; ++v)
            {
                float *dst = disp_filt.ptr<float>(v);
                const int16_t *src = disp_16s.ptr<int16_t>(v);
                for (int u = 0; u < disp_filt.cols; ++u)
                {
                    const int16_t q = src[u];
                    dst[u] = (q > 0) ? (float)((double)q / S) : 0.0f;
                }
            }
        }
        else if (mode == "edge_flying_kill")
        {
            // Target: invalidate disparity pixels that are strong outliers compared to local neighborhood.
            // This is very effective for "edge points shoot backward" artifacts.

            int k = std::max(3, disp_edge_ksize_ | 1);
            int r = k / 2;
            const double tau = std::max(0.0, disp_edge_tau_);
            RCLCPP_INFO_ONCE(get_logger(), "[Disparity Filter] Using edge_flying_kill filter with ksize=%d, tau=%.2f, min_neighbors=%d",
                             k, tau, disp_edge_min_neigh_);

            // Precompute a median-smoothed reference (cheap robust local estimate)
            cv::Mat med;
            cv::medianBlur(disp_filt, med, k);

            cv::Mat out = disp_filt.clone();

            for (int v = r; v < out.rows - r; ++v)
            {
                float *o = out.ptr<float>(v);
                const float *drow = disp_filt.ptr<float>(v);
                const float *mrow = med.ptr<float>(v);

                for (int u = r; u < out.cols - r; ++u)
                {
                    const float d = drow[u];
                    if (!(d > 0.0f))
                        continue;

                    const float m = mrow[u];
                    if (!(m > 0.0f))
                        continue;

                    // Count valid neighbors and compute local deviation
                    int valid_n = 0;
                    for (int yy = v - r; yy <= v + r; ++yy)
                    {
                        const float *rr = disp_filt.ptr<float>(yy);
                        for (int xx = u - r; xx <= u + r; ++xx)
                        {
                            if (rr[xx] > 0.0f)
                                valid_n++;
                        }
                    }
                    if (valid_n < disp_edge_min_neigh_)
                        continue;

                    // If disparity deviates too far from local median, invalidate it.
                    // (High disparity => close; Low disparity => far. "Backward points" often come from too-small disparity.)
                    const double rel = std::abs((double)d - (double)m) / (double)m;
                    if (rel > tau)
                    {
                        o[u] = 0.0f; // mark invalid
                    }
                }
            }

            disp_filt = out;
        }

        // Commit filtered disparity back to disp_ if enabled
        if (mode != "none")
        {
            std::memcpy(disp_.data(), disp_filt.ptr<float>(0), (size_t)out_h_ * out_w_ * sizeof(float));
        }
        // ---------------- END DISPARITY FILTER BLOCK ----------------

        // Publish disparity image if needed
        if (publish_disparity_)
        {
            sensor_msgs::msg::Image disp_msg;
            disp_msg.header = left_msg->header;
            disp_msg.height = out_h_;
            disp_msg.width = out_w_;
            disp_msg.encoding = "32FC1";
            disp_msg.is_bigendian = false;
            disp_msg.step = out_w_ * sizeof(float);
            disp_msg.data.resize((size_t)out_h_ * disp_msg.step);
            std::memcpy(disp_msg.data.data(), disp_.data(), (size_t)out_h_ * disp_msg.step);
            disp_pub_->publish(disp_msg);
        }

        // Grab the camera intrinsics
        const double fx = fx_;
        const double fy = fy_;
        const double cx = cx_;
        const double cy = cy_;
        const double B = baseline_;
        const double maxR = max_range_m_;
        const int s = std::max(1, stride_);

        // Publish depth image if needed
        if (publish_depth_)
        {
            std::vector<float> depth((size_t)out_h_ * out_w_, std::numeric_limits<float>::quiet_NaN());

            for (int v = 0; v < out_h_; ++v)
            {
                const float *row = disp_.data() + v * out_w_;
                float *drow = depth.data() + v * out_w_;
                for (int u = 0; u < out_w_; ++u)
                {
                    const float disp = row[u];
                    if (disp <= 0.0f)
                        continue;
                    const double Z = fx * B / (double)disp;
                    if (Z <= 0.0 || Z > maxR)
                        continue;
                    drow[u] = (float)Z;
                }
            }

            // ------------------ DEPTH FILTER BLOCK ------------------
            auto dmode = toLower(depth_filter_mode_);

            if (dmode == "median")
            {
                // Median on depth (ignores NaN poorly). We'll do a simple "NaN -> 0" trick, then restore NaN where invalid.
                RCLCPP_INFO_ONCE(get_logger(), "[Depth Filter] Using median filter with ksize=%d", depth_median_ksize_);
                cv::Mat dmat(out_h_, out_w_, CV_32FC1, depth.data());
                cv::Mat valid = (dmat == dmat) & (dmat > 0.0f); // NaN check: x==x
                cv::Mat tmp = dmat.clone();
                tmp.setTo(0.0f, ~valid);

                int k = std::max(3, depth_median_ksize_ | 1);
                cv::medianBlur(tmp, tmp, k);

                // Restore invalids
                tmp.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
                std::memcpy(depth.data(), tmp.ptr<float>(0), depth.size() * sizeof(float));
            }
            else if (dmode == "flying_pixel")
            {
                // Invalidate depth pixels that are strong outliers compared to local median neighborhood.
                // This is the classic "flying pixel" filter used for RGB-D.
                cv::Mat dmat(out_h_, out_w_, CV_32FC1, depth.data());

                int k = std::max(3, depth_flying_ksize_ | 1);
                int r = k / 2;
                const double tau = std::max(0.0, depth_flying_tau_);
                RCLCPP_INFO_ONCE(get_logger(), "[Depth Filter] Using flying_pixel filter with ksize=%d, tau=%.2f, min_neighbors=%d",
                                 k, tau, depth_flying_min_neigh_);

                // Build a median-smoothed reference
                cv::Mat tmp = dmat.clone();
                // Replace NaNs with 0 for median blur; we'll treat 0 as invalid
                for (int v = 0; v < tmp.rows; ++v)
                {
                    float *row = tmp.ptr<float>(v);
                    for (int u = 0; u < tmp.cols; ++u)
                    {
                        const float z = row[u];
                        if (!(z == z) || z <= 0.0f)
                            row[u] = 0.0f;
                    }
                }

                cv::Mat med;
                cv::medianBlur(tmp, med, k);

                for (int v = r; v < dmat.rows - r; ++v)
                {
                    float *zrow = dmat.ptr<float>(v);
                    const float *mrow = med.ptr<float>(v);

                    for (int u = r; u < dmat.cols - r; ++u)
                    {
                        const float z = zrow[u];
                        if (!(z == z) || z <= 0.0f)
                            continue; // invalid already

                        const float m = mrow[u];
                        if (!(m > 0.0f))
                            continue;

                        int valid_n = 0;
                        for (int yy = v - r; yy <= v + r; ++yy)
                        {
                            const float *rr = dmat.ptr<float>(yy);
                            for (int xx = u - r; xx <= u + r; ++xx)
                            {
                                const float zz = rr[xx];
                                if ((zz == zz) && zz > 0.0f)
                                    valid_n++;
                            }
                        }
                        if (valid_n < depth_flying_min_neigh_)
                            continue;

                        const double rel = std::abs((double)z - (double)m) / (double)m;
                        if (rel > tau)
                        {
                            zrow[u] = std::numeric_limits<float>::quiet_NaN();
                        }
                    }
                }
            }
            // ---------------- END DEPTH FILTER BLOCK ------------------

            sensor_msgs::msg::Image depth_msg;
            depth_msg.header = left_msg->header;
            depth_msg.height = out_h_;
            depth_msg.width = out_w_;
            depth_msg.encoding = "32FC1";
            depth_msg.is_bigendian = false;
            depth_msg.step = out_w_ * sizeof(float);
            depth_msg.data.resize(depth.size() * sizeof(float));
            std::memcpy(depth_msg.data.data(), depth.data(), depth.size() * sizeof(float));
            depth_pub_->publish(depth_msg);
        }

        // Publish point cloud if needed
        if (publish_cloud_)
        {

            // Helper: pack RGB into a float (PCL convention used by RViz "RGB8")
            auto packRGBFloat = [](uint8_t r, uint8_t g, uint8_t b) -> float
            {
                const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                                     (static_cast<uint32_t>(g) << 8) |
                                     (static_cast<uint32_t>(b));
                float f;
                std::memcpy(&f, &rgb, sizeof(float));
                return f;
            };

            // ------------------ POINTCLOUD FILTER BLOCK (GRID & KNN) ------------------
            // Produces a boolean keep-mask for sampled pixels (u,v) based on local depth consistency.
            std::vector<uint8_t> keep_mask; // 1=keep, 0=drop, sized out_h_*out_w_
            keep_mask.assign((size_t)out_h_ * out_w_, 1);

            auto pcmode = toLower(pc_filter_mode_);
            if (pcmode == "grid_outlier")
            {
                int k = std::max(3, pc_grid_ksize_ | 1);
                int r = k / 2;
                const double tau = std::max(0.0, pc_grid_tau_);
                RCLCPP_INFO_ONCE(get_logger(), "[PointCloud Filter] Using grid_outlier filter with ksize=%d, tau=%.2f, min_neighbors=%d",
                                 k, tau, pc_grid_min_neigh_);

                // Build depth image from disparity (single pass)
                std::vector<float> zimg((size_t)out_h_ * out_w_, std::numeric_limits<float>::quiet_NaN());
                for (int v = 0; v < out_h_; ++v)
                {
                    const float *row = disp_.data() + v * out_w_;
                    float *zrow = zimg.data() + v * out_w_;
                    for (int u = 0; u < out_w_; ++u)
                    {
                        const float d = row[u];
                        if (d <= 0.0f)
                            continue;
                        const double Z = fx * B / (double)d;
                        if (Z <= 0.0 || Z > maxR)
                            continue;
                        zrow[u] = (float)Z;
                    }
                }

                // Median reference (robust)
                cv::Mat zmat(out_h_, out_w_, CV_32FC1, zimg.data());
                cv::Mat tmp = zmat.clone();
                // NaN->0 for median blur
                for (int v = 0; v < out_h_; ++v)
                {
                    float *row = tmp.ptr<float>(v);
                    for (int u = 0; u < out_w_; ++u)
                    {
                        const float z = row[u];
                        if (!(z == z) || z <= 0.0f)
                            row[u] = 0.0f;
                    }
                }
                cv::Mat med;
                cv::medianBlur(tmp, med, k);

                for (int v = r; v < out_h_ - r; ++v)
                {
                    for (int u = r; u < out_w_ - r; ++u)
                    {
                        const float z = zimg[(size_t)v * out_w_ + u];
                        if (!(z == z) || z <= 0.0f)
                            continue;

                        const float m = med.at<float>(v, u);
                        if (!(m > 0.0f))
                            continue;

                        int valid_n = 0;
                        for (int yy = v - r; yy <= v + r; ++yy)
                        {
                            for (int xx = u - r; xx <= u + r; ++xx)
                            {
                                const float zz = zimg[(size_t)yy * out_w_ + xx];
                                if ((zz == zz) && zz > 0.0f)
                                    valid_n++;
                            }
                        }
                        if (valid_n < pc_grid_min_neigh_)
                            continue;

                        const double rel = std::abs((double)z - (double)m) / (double)m;
                        if (rel > tau)
                        {
                            keep_mask[(size_t)v * out_w_ + u] = 0;
                        }
                    }
                }
            }
            else if (pcmode == "knn_outlier")
            {
                // K-Nearest Neighbor outlier removal in 3D space
                // First, build a list of all valid 3D points (with their grid indices)
                struct Point3D
                {
                    float x, y, z;
                    size_t idx; // linear index in keep_mask
                };
                std::vector<Point3D> points;
                points.reserve((size_t)out_h_ * out_w_ / 4); // rough estimate

                for (int v = 0; v < out_h_; v += s)
                {
                    const float *row = disp_.data() + v * out_w_;
                    for (int u = 0; u < out_w_; u += s)
                    {
                        const float d = row[u];
                        if (d <= 0.0f)
                            continue;

                        const double Z = fx * B / static_cast<double>(d);
                        if (Z <= 0.0 || Z > maxR)
                            continue;

                        const float X = static_cast<float>((static_cast<double>(u) - cx) * Z / fx);
                        const float Y = static_cast<float>((static_cast<double>(v) - cy) * Z / fy);
                        const float Zf = static_cast<float>(Z);

                        Point3D pt;
                        pt.x = X;
                        pt.y = Y;
                        pt.z = Zf;
                        pt.idx = (size_t)v * out_w_ + u;
                        points.push_back(pt);
                    }
                }

                if (points.size() > (size_t)pc_knn_k_)
                {
                    RCLCPP_INFO_ONCE(get_logger(), "[PointCloud Filter] Using knn_outlier filter with k=%d, stddev_multiplier=%.1f on %zu points",
                                     pc_knn_k_, pc_knn_stddev_mul_, points.size());
                    // For each point, find k nearest neighbors and compute mean distance
                    std::vector<float> mean_dists(points.size());

                    for (size_t i = 0; i < points.size(); ++i)
                    {
                        const Point3D &p = points[i];

                        // Simple brute-force k-NN (for production, consider using a KD-tree)
                        std::vector<float> dists;
                        dists.reserve(points.size());

                        for (size_t j = 0; j < points.size(); ++j)
                        {
                            if (i == j)
                                continue;
                            const Point3D &q = points[j];
                            const float dx = p.x - q.x;
                            const float dy = p.y - q.y;
                            const float dz = p.z - q.z;
                            const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                            dists.push_back(dist);
                        }

                        // Sort to find k nearest
                        std::partial_sort(dists.begin(), dists.begin() + pc_knn_k_, dists.end());

                        // Compute mean of k nearest
                        float sum = 0.0f;
                        for (int k = 0; k < pc_knn_k_; ++k)
                        {
                            sum += dists[k];
                        }
                        mean_dists[i] = sum / (float)pc_knn_k_;
                    }

                    // Compute mean and stddev of all mean distances
                    float sum = 0.0f;
                    for (float md : mean_dists)
                    {
                        sum += md;
                    }
                    const float global_mean = sum / (float)mean_dists.size();

                    float var_sum = 0.0f;
                    for (float md : mean_dists)
                    {
                        const float diff = md - global_mean;
                        var_sum += diff * diff;
                    }
                    const float stddev = std::sqrt(var_sum / (float)mean_dists.size());

                    // Mark outliers
                    const float threshold = global_mean + (float)pc_knn_stddev_mul_ * stddev;
                    for (size_t i = 0; i < points.size(); ++i)
                    {
                        if (mean_dists[i] > threshold)
                        {
                            keep_mask[points[i].idx] = 0;
                        }
                    }
                }
            }
            // ---------------- END POINTCLOUD FILTER BLOCK (GRID & KNN) ------------------

            // Count points using SAME filters as fill (important!)
            size_t n_valid = 0;
            for (int v = 0; v < out_h_; v += s)
            {
                const float *row = disp_.data() + v * out_w_;
                for (int u = 0; u < out_w_; u += s)
                {
                    if (keep_mask[(size_t)v * out_w_ + u] == 0)
                        continue;

                    const float d = row[u];
                    if (d <= 0.0f)
                    {
                        if (use_background_)
                        {
                            n_valid++;
                        }
                        continue;
                    }

                    const double Z = fx * B / static_cast<double>(d);
                    if (Z <= 0.0 || Z > maxR)
                    {
                        if (use_background_)
                        {
                            n_valid++;
                        }
                        continue;
                    }

                    n_valid++;
                }
            }

            sensor_msgs::msg::PointCloud2 cloud;
            cloud.header.stamp = left_msg->header.stamp;
            cloud.header.frame_id = left_msg->header.frame_id; // or force optical frame if you prefer
            cloud.height = 1;
            cloud.width = static_cast<uint32_t>(n_valid);
            cloud.is_bigendian = false;
            cloud.is_dense = false;

            // Define xyz + rgb fields explicitly
            cloud.fields.resize(4);

            cloud.fields[0].name = "x";
            cloud.fields[0].offset = 0;
            cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[0].count = 1;

            cloud.fields[1].name = "y";
            cloud.fields[1].offset = 4;
            cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[1].count = 1;

            cloud.fields[2].name = "z";
            cloud.fields[2].offset = 8;
            cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[2].count = 1;

            cloud.fields[3].name = "rgb";
            cloud.fields[3].offset = 12;
            cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32; // packed RGB in float
            cloud.fields[3].count = 1;

            cloud.point_step = 16; // 4 floats: x,y,z,rgb
            cloud.row_step = cloud.point_step * cloud.width;
            cloud.data.resize(cloud.row_step);

            // Fill packed data
            size_t idx = 0;
            for (int v = 0; v < out_h_; v += s)
            {
                const float *row = disp_.data() + v * out_w_;
                for (int u = 0; u < out_w_; u += s)
                {
                    if (keep_mask[(size_t)v * out_w_ + u] == 0)
                        continue;

                    const float d = row[u];
                    bool use_max_range = false;

                    if (d <= 0.0f)
                    {
                        if (use_background_)
                        {
                            use_max_range = true;
                        }
                        else
                        {
                            continue;
                        }
                    }

                    double Z = 0.0;
                    if (!use_max_range)
                    {
                        Z = fx * B / static_cast<double>(d);
                        if (Z <= 0.0)
                        {
                            if (use_background_)
                            {
                                use_max_range = true;
                            }
                            else
                            {
                                continue;
                            }
                        }
                        else if (Z > maxR)
                        {
                            if (use_background_)
                            {
                                // Clamp Z to maxR (the X,Y will scale proportionally below)
                                Z = maxR;
                            }
                            else
                            {
                                continue;
                            }
                        }
                    }
                    else // Set to max range if invalid disparity
                    {
                        Z = maxR;
                    }

                    const float X = static_cast<float>((static_cast<double>(u) - cx) * Z / fx);
                    const float Y = static_cast<float>((static_cast<double>(v) - cy) * Z / fy);
                    const float Zf = static_cast<float>(Z);

                    // Sample color from resized left image at the same pixel (u,v)
                    const cv::Vec3b bgr = left_cv->image.at<cv::Vec3b>(v, u);
                    const float rgb_f = packRGBFloat(bgr[2], bgr[1], bgr[0]); // r,g,b

                    uint8_t *ptr = cloud.data.data() + idx * cloud.point_step;
                    std::memcpy(ptr + 0, &X, sizeof(float));
                    std::memcpy(ptr + 4, &Y, sizeof(float));
                    std::memcpy(ptr + 8, &Zf, sizeof(float));
                    std::memcpy(ptr + 12, &rgb_f, sizeof(float));

                    idx++;
                }
            }

            cloud_pub_->publish(cloud);
        }
        const auto t3 = now();

        const double prep_ms = (t1 - t0).seconds() * 1e3;
        const double infer_ms = (t2 - t1).seconds() * 1e3;
        const double post_ms = (t3 - t2).seconds() * 1e3;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "prep %.1f ms | infer %.1f ms | post %.1f ms",
                             prep_ms, infer_ms, post_ms);
    }

private:
    // params
    std::string engine_path_, left_topic_, right_topic_, info_topic_;
    double baseline_{0.06};
    int stride_{2};
    double max_range_m_{10.0};
    bool use_background_{false};

    // intrinsics
    bool have_info_{false};
    double fx_{0}, fy_{0}, cx_{0}, cy_{0};
    uint32_t cam_w_{0}, cam_h_{0};

    // TRT runner and buffers
    std::unique_ptr<TrtRunner> runner_;
    int in_w_{0}, in_h_{0}, out_w_{0}, out_h_{0};
    std::vector<float> left_nchw_, right_nchw_, disp_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    // std::shared_ptr<message_filters::Synchronizer<
    //     message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
    //     sync_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
        sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disp_pub_;

    // Output options
    bool publish_cloud_, publish_depth_, publish_disparity_;
    std::string cloud_topic_, depth_topic_, disparity_topic_;

    // QoS params
    std::string sub_rel_, sub_dur_, sub_hist_;
    int sub_depth_{5};
    std::string pub_rel_, pub_dur_, pub_hist_;
    int pub_depth_{5};

    // Disparity filter params
    std::string disp_filter_mode_;
    int disp_median_ksize_{5};
    int disp_bilat_d_{7};
    double disp_bilat_sc_{3.0}, disp_bilat_ss_{7.0};
    int disp_speckle_max_size_{120};
    double disp_speckle_range_{1.0}, disp_speckle_scale_{16.0};
    int disp_edge_ksize_{5};
    double disp_edge_tau_{0.20};
    int disp_edge_min_neigh_{6};

    // Depth filter params
    std::string depth_filter_mode_;
    int depth_flying_ksize_{5};
    double depth_flying_tau_{0.25};
    int depth_flying_min_neigh_{6};
    int depth_median_ksize_{5};

    // Point cloud filter params
    std::string pc_filter_mode_;
    int pc_grid_ksize_{5};
    double pc_grid_tau_{0.25};
    int pc_grid_min_neigh_{4};
    int pc_knn_k_{20};
    double pc_knn_stddev_mul_{2.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoundationStereoMatcherNode>());
    rclcpp::shutdown();
    return 0;
}
