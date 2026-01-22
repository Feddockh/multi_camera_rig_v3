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

        bgrToNCHWFloat(left_cv->image, in_h_, in_w_, left_nchw_);
        bgrToNCHWFloat(right_cv->image, in_h_, in_w_, right_nchw_);
        const auto t1 = now();
        runner_->run(left_nchw_.data(), right_nchw_.data(), disp_.data(), disp_.size());
        const auto t2 = now();

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

        // Count points using SAME filters as fill (important!)
        size_t n_valid = 0;
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
                const float d = row[u];
                if (d <= 0.0f)
                    continue;

                const double Z = fx * B / static_cast<double>(d);
                if (Z <= 0.0 || Z > maxR)
                    continue;

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
        const auto t3 = now();

        const double prep_ms = (t1 - t0).seconds() * 1e3;
        const double infer_ms = (t2 - t1).seconds() * 1e3;
        const double pc_ms = (t3 - t2).seconds() * 1e3;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "prep %.1f ms | infer %.1f ms | post %.1f ms | points %u (stride %d)",
                             prep_ms, infer_ms, pc_ms, cloud.width, s);
    }

private:
    // params
    std::string engine_path_, left_topic_, right_topic_, info_topic_;
    double baseline_{0.06};
    int stride_{2};
    double max_range_m_{10.0};

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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoundationStereoMatcherNode>());
    rclcpp::shutdown();
    return 0;
}
