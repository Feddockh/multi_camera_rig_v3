#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <mutex>
#include <string>
#include <vector>

static rclcpp::ReliabilityPolicy parseReliability(const std::string &s)
{
    if (s == "reliable")
        return rclcpp::ReliabilityPolicy::Reliable;
    if (s == "best_effort")
        return rclcpp::ReliabilityPolicy::BestEffort;
    throw std::runtime_error("Invalid reliability: " + s + " (use reliable|best_effort)");
}

static int parseInterpolation(const std::string &s)
{
    if (s == "linear")
        return cv::INTER_LINEAR;
    if (s == "nearest")
        return cv::INTER_NEAREST;
    throw std::runtime_error("Invalid interpolation: " + s + " (use linear|nearest)");
}

class StereoRectifyScaleNode : public rclcpp::Node
{
public:
    StereoRectifyScaleNode() : Node("stereo_rectify_scale_node")
    {
        in_image_topic_ = declare_parameter<std::string>("in_image_topic", "image_raw");
        in_info_topic_ = declare_parameter<std::string>("in_info_topic", "camera_info");
        out_image_topic_ = declare_parameter<std::string>("out_image_topic", "image_rect_scaled");
        out_info_topic_ = declare_parameter<std::string>("out_info_topic", "camera_info_scaled");

        out_w_ = declare_parameter<int>("output_width", 896);
        out_h_ = declare_parameter<int>("output_height", 672);

        interpolation_ = parseInterpolation(declare_parameter<std::string>("interpolation", "linear"));

        sub_rel_ = parseReliability(declare_parameter<std::string>("sub_qos.reliability", "reliable"));
        pub_rel_ = parseReliability(declare_parameter<std::string>("pub_qos.reliability", "best_effort"));
        sub_depth_ = declare_parameter<int>("sub_qos.depth", 5);
        pub_depth_ = declare_parameter<int>("pub_qos.depth", 5);

        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(sub_depth_)).reliability(sub_rel_);
        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(pub_depth_)).reliability(pub_rel_);

        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            in_info_topic_, sub_qos,
            std::bind(&StereoRectifyScaleNode::onInfo, this, std::placeholders::_1));

        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            in_image_topic_, sub_qos,
            std::bind(&StereoRectifyScaleNode::onImage, this, std::placeholders::_1));

        img_pub_ = create_publisher<sensor_msgs::msg::Image>(out_image_topic_, pub_qos);
        info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(out_info_topic_, pub_qos);

        RCLCPP_INFO(get_logger(), "Rectify+Scale: %s + %s -> %s + %s (out %dx%d)",
                    in_image_topic_.c_str(), in_info_topic_.c_str(),
                    out_image_topic_.c_str(), out_info_topic_.c_str(),
                    out_w_, out_h_);
    }

private:
    void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_info_ = *msg;
        have_info_ = true;
        maps_valid_ = false; // new calibration => recompute maps
    }

    void ensureMapsLocked(int in_w, int in_h)
    {
        if (!have_info_)
            return;
        if (maps_valid_ && in_w == last_in_w_ && in_h == last_in_h_)
            return;

        // Build OpenCV matrices from CameraInfo
        cv::Mat K(3, 3, CV_64F, (void *)last_info_.k.data());
        cv::Mat D((int)last_info_.d.size(), 1, CV_64F, (void *)last_info_.d.data());
        cv::Mat R(3, 3, CV_64F, (void *)last_info_.r.data());
        cv::Mat P(3, 4, CV_64F, (void *)last_info_.p.data());

        // We want rectified image in the input resolution first:
        cv::Size in_size(in_w, in_h);

        // OpenCV expects 3x3 "new camera matrix". For rectified images, use the left 3x3 of P.
        cv::Mat P3 = P(cv::Rect(0, 0, 3, 3)).clone();

        cv::initUndistortRectifyMap(
            K, D, R, P3, in_size, CV_32FC1,
            map1_, map2_);

        last_in_w_ = in_w;
        last_in_h_ = in_h;
        maps_valid_ = true;
    }

    static inline void scaleIntrinsics(sensor_msgs::msg::CameraInfo &ci, double sx, double sy)
    {
        // K (row-major)
        ci.k[0] *= sx; // fx
        ci.k[2] *= sx; // cx
        ci.k[4] *= sy; // fy
        ci.k[5] *= sy; // cy

        // P (row-major 3x4): scale fx, fy, cx, cy, and also Tx if present
        ci.p[0] *= sx; // fx
        ci.p[2] *= sx; // cx
        ci.p[3] *= sx; // Tx (baseline term in pixels)
        ci.p[5] *= sy; // fy
        ci.p[6] *= sy; // cy
    }

    void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        sensor_msgs::msg::CameraInfo info_copy;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!have_info_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for CameraInfo...");
                return;
            }
            info_copy = last_info_;
            // recompute maps based on THIS image resolution
            ensureMapsLocked((int)msg->width, (int)msg->height);
            if (!maps_valid_)
                return;
        }

        cv_bridge::CvImageConstPtr cv_in;
        try
        {
            cv_in = cv_bridge::toCvShare(msg, msg->encoding);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge failed: %s", e.what());
            return;
        }

        // Rectify
        cv::Mat rectified;
        cv::remap(cv_in->image, rectified, map1_, map2_, interpolation_);

        // Scale
        cv::Mat scaled;
        cv::resize(rectified, scaled, cv::Size(out_w_, out_h_), 0, 0, interpolation_);

        // Publish scaled image
        cv_bridge::CvImage out;
        out.header = msg->header;
        out.encoding = msg->encoding;
        out.image = scaled;
        img_pub_->publish(*out.toImageMsg());

        // Publish scaled CameraInfo (rectified)
        // Make a rectified camera info:
        // - D should be zeros (rectified image has no distortion)
        // - R = Identity
        // - K and P derived from incoming P/K but scaled to out resolution
        // We’ll start from input info_copy and "rectified-ize" it.
        sensor_msgs::msg::CameraInfo out_info = info_copy;
        out_info.header = msg->header;
        out_info.width = out_w_;
        out_info.height = out_h_;

        // For rectified images, D is usually zeroed and R is identity
        out_info.d.assign(out_info.d.size(), 0.0);
        out_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

        // IMPORTANT: scale intrinsics from input image size -> output size
        const double sx = static_cast<double>(out_w_) / static_cast<double>(msg->width);
        const double sy = static_cast<double>(out_h_) / static_cast<double>(msg->height);
        scaleIntrinsics(out_info, sx, sy);

        info_pub_->publish(out_info);
    }

private:
    std::string in_image_topic_, in_info_topic_;
    std::string out_image_topic_, out_info_topic_;
    int out_w_{896}, out_h_{672};
    int interpolation_{cv::INTER_LINEAR};

    rclcpp::ReliabilityPolicy sub_rel_{rclcpp::ReliabilityPolicy::Reliable};
    rclcpp::ReliabilityPolicy pub_rel_{rclcpp::ReliabilityPolicy::BestEffort};
    int sub_depth_{5}, pub_depth_{5};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

    std::mutex mtx_;
    bool have_info_{false};
    bool maps_valid_{false};
    int last_in_w_{-1}, last_in_h_{-1};
    sensor_msgs::msg::CameraInfo last_info_;

    cv::Mat map1_, map2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoRectifyScaleNode>());
    rclcpp::shutdown();
    return 0;
}
