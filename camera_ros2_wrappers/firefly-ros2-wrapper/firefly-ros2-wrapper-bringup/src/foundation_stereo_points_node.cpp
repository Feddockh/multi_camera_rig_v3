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

class FoundationStereoPointsNode : public rclcpp::Node
{
public:
    FoundationStereoPointsNode() : Node("foundation_stereo_points_node")
    {
        engine_path_ = declare_parameter<std::string>("engine_path", "");
        baseline_ = declare_parameter<double>("baseline", 0.06);

        left_topic_ = declare_parameter<std::string>("left_image_topic", "/firefly_left/image_rect");
        right_topic_ = declare_parameter<std::string>("right_image_topic", "/firefly_right/image_rect");
        info_topic_ = declare_parameter<std::string>("left_info_topic", "/firefly_left/camera_info");

        // Point cloud density control (very useful for CPU)
        stride_ = declare_parameter<int>("stride", 2); // 1=full model res, 2=quarter points, 4=1/16 points
        max_range_m_ = declare_parameter<double>("max_range_m", 10.0);

        auto qos = rclcpp::SensorDataQoS();

        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, qos, std::bind(&FoundationStereoPointsNode::onInfo, this, std::placeholders::_1));

        left_sub_.subscribe(this, left_topic_, rmw_qos_profile_sensor_data);
        right_sub_.subscribe(this, right_topic_, rmw_qos_profile_sensor_data);

        // using Policy = message_filters::sync_policies::ApproximateTime<
        //     sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        using Policy = message_filters::sync_policies::ExactTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(5), left_sub_, right_sub_);
        // using ExactPolicy = message_filters::sync_policies::ExactTime<Image, Image>;
        // std::shared_ptr<message_filters::Synchronizer<ExactPolicy>> sync_;
        sync_->registerCallback(std::bind(&FoundationStereoPointsNode::onStereo, this,
                                          std::placeholders::_1, std::placeholders::_2));

        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/firefly/points_fs", qos);

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
        cv::resize(left_cv->image, left_rs, cv::Size(in_w_, in_h_), 0, 0, cv::INTER_LINEAR);
        cv::resize(right_cv->image, right_rs, cv::Size(in_w_, in_h_), 0, 0, cv::INTER_LINEAR);

        // NCHW float32
        bgrToNCHWFloat(left_rs, in_h_, in_w_, left_nchw_);
        bgrToNCHWFloat(right_rs, in_h_, in_w_, right_nchw_);

        const auto t1 = now();
        runner_->run(left_nchw_.data(), right_nchw_.data(), disp_.data(), disp_.size());
        const auto t2 = now();

        // Scale intrinsics to model resolution (rectified)
        const double sw = static_cast<double>(in_w_) / static_cast<double>(cam_w_);
        const double sh = static_cast<double>(in_h_) / static_cast<double>(cam_h_);

        const double fx = fx_ * sw;
        const double fy = fy_ * sh;
        const double cx = cx_ * sw;
        const double cy = cy_ * sh;

        const double B = baseline_;
        const double maxR = max_range_m_;
        const int s = std::max(1, stride_);

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
                const cv::Vec3b bgr = left_rs.at<cv::Vec3b>(v, u);
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
                             "prep %.1f ms | infer %.1f ms | cloud %.1f ms | points %u (stride %d)",
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoundationStereoPointsNode>());
    rclcpp::shutdown();
    return 0;
}
