/*
Foundation Stereo Matcher Node
ROS2 wrapper for TensorRT-accelerated stereo matching with optional speckle filtering.

Run with:
    ros2 run firefly-ros2-wrapper-reconstruction foundation_stereo_matcher_node --ros-args \
    -p engine_path:=/path/to/model.plan \
    -p disp_filter.mode:=speckle
*/

#include "multi_camera_rig_reconstruction/foundation_stereo_matcher.hpp"
#include "multi_camera_rig_common/qos_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <memory>

using namespace multi_camera_rig_reconstruction;

class FoundationStereoMatcherNode : public rclcpp::Node
{
public:
    FoundationStereoMatcherNode() : Node("foundation_stereo_matcher_node")
    {
        // Declare parameters
        auto engine_path = declare_parameter<std::string>("engine_path", "");
        auto left_topic = declare_parameter<std::string>("left_image_topic", "/firefly_left/image_rect");
        auto right_topic = declare_parameter<std::string>("right_image_topic", "/firefly_right/image_rect");
        auto info_topic = declare_parameter<std::string>("left_info_topic", "/firefly_left/camera_info");
        auto disparity_topic = declare_parameter<std::string>("disparity_topic", "/stereo/disparity");

        // QoS params
        auto sub_rel = declare_parameter<std::string>("sub_qos.reliability", "reliable");
        auto sub_dur = declare_parameter<std::string>("sub_qos.durability", "volatile");
        auto sub_hist = declare_parameter<std::string>("sub_qos.history", "keep_last");
        auto sub_depth = declare_parameter<int>("sub_qos.depth", 5);
        auto pub_rel = declare_parameter<std::string>("pub_qos.reliability", "best_effort");
        auto pub_dur = declare_parameter<std::string>("pub_qos.durability", "volatile");
        auto pub_hist = declare_parameter<std::string>("pub_qos.history", "keep_last");
        auto pub_depth = declare_parameter<int>("pub_qos.depth", 5);

        // Disparity filter params
        FoundationStereoMatcherConfig config;
        config.engine_path = engine_path;
        config.disp_filter_mode = declare_parameter<std::string>("disp_filter.mode", "none");
        config.speckle_max_size = declare_parameter<int>("disp_filter.speckle_max_size", 120);
        config.speckle_range = declare_parameter<double>("disp_filter.speckle_range", 1.0);
        config.speckle_scale = declare_parameter<double>("disp_filter.speckle_scale", 16.0);

        // Create processor
        try
        {
            matcher_ = std::make_unique<FoundationStereoMatcher>(config);
            RCLCPP_INFO(get_logger(), "Engine IO: left/right (1,3,%d,%d) disp (1,1,%d,%d)",
                        matcher_->inputHeight(), matcher_->inputWidth(),
                        matcher_->outputHeight(), matcher_->outputWidth());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize matcher: %s", e.what());
            throw;
        }

        // Setup QoS
        auto sub_qos = multi_camera_rig_common::makeQos(sub_rel, sub_dur, sub_hist, sub_depth);
        auto pub_qos = multi_camera_rig_common::makeQos(pub_rel, pub_dur, pub_hist, pub_depth);

        // Create subscribers
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic, sub_qos,
            std::bind(&FoundationStereoMatcherNode::onInfo, this, std::placeholders::_1));
        
        left_sub_.subscribe(this, left_topic, sub_qos.get_rmw_qos_profile());
        right_sub_.subscribe(this, right_topic, sub_qos.get_rmw_qos_profile());

        // Create synchronizer
        using Policy = message_filters::sync_policies::ExactTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
            Policy(5), left_sub_, right_sub_);
        sync_->registerCallback(std::bind(&FoundationStereoMatcherNode::onStereo, this,
                                          std::placeholders::_1, std::placeholders::_2));

        // Create publisher
        disp_pub_ = create_publisher<sensor_msgs::msg::Image>(disparity_topic, pub_qos);

        RCLCPP_INFO(get_logger(), "Foundation Stereo Matcher Node initialized");
    }

private:
    void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if ((int)msg->width != matcher_->inputWidth() || 
            (int)msg->height != matcher_->inputHeight())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                        "CameraInfo size (%dx%d) does not match engine input (%dx%d)",
                        msg->width, msg->height, 
                        matcher_->inputWidth(), matcher_->inputHeight());
        }
    }

    void onStereo(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        const auto t0 = now();

        // Convert to OpenCV
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

        // Check size
        if (left_cv->image.cols != matcher_->inputWidth() || 
            left_cv->image.rows != matcher_->inputHeight())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Input size mismatch. Expected %dx%d, got %dx%d",
                                 matcher_->inputWidth(), matcher_->inputHeight(),
                                 left_cv->image.cols, left_cv->image.rows);
            return;
        }

        const auto t1 = now();

        // Process
        cv::Mat disparity;
        try
        {
            matcher_->process(left_cv->image, right_cv->image, disparity);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Processing failed: %s", e.what());
            return;
        }

        const auto t2 = now();

        // Publish
        sensor_msgs::msg::Image disp_msg;
        disp_msg.header = left_msg->header;
        disp_msg.height = disparity.rows;
        disp_msg.width = disparity.cols;
        disp_msg.encoding = "32FC1";
        disp_msg.is_bigendian = false;
        disp_msg.step = disparity.cols * sizeof(float);
        disp_msg.data.resize(disparity.rows * disp_msg.step);
        std::memcpy(disp_msg.data.data(), disparity.ptr<float>(0), disp_msg.data.size());
        disp_pub_->publish(disp_msg);

        const auto t3 = now();

        const double prep_ms = (t1 - t0).seconds() * 1e3;
        const double proc_ms = (t2 - t1).seconds() * 1e3;
        const double pub_ms = (t3 - t2).seconds() * 1e3;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "prep %.1f ms | process %.1f ms | publish %.1f ms",
                             prep_ms, proc_ms, pub_ms);
    }

    std::unique_ptr<FoundationStereoMatcher> matcher_;
    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
        sync_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disp_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoundationStereoMatcherNode>());
    rclcpp::shutdown();
    return 0;
}
