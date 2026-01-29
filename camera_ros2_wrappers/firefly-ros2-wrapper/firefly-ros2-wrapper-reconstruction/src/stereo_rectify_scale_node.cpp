/*
Stereo Rectify and Scale Node
ROS2 wrapper for combined rectification and scaling operations.

Run with:
    ros2 run firefly-ros2-wrapper-reconstruction stereo_rectify_scale_node --ros-args \
    -p output_width:=896 \
    -p output_height:=672
*/

#include "firefly_reconstruction/stereo_rectify_scale.hpp"
#include "firefly_reconstruction/qos_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <memory>
#include <mutex>

using namespace firefly_reconstruction;

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
        // Declare parameters
        auto in_image_topic = declare_parameter<std::string>("in_image_topic", "image_raw");
        auto in_info_topic = declare_parameter<std::string>("in_info_topic", "camera_info");
        auto out_image_topic = declare_parameter<std::string>("out_image_topic", "image_rect_scaled");
        auto out_info_topic = declare_parameter<std::string>("out_info_topic", "camera_info_scaled");

        // Processor config
        StereoRectifyScaleConfig config;
        config.output_width = declare_parameter<int>("output_width", 896);
        config.output_height = declare_parameter<int>("output_height", 672);
        config.interpolation = parseInterpolation(
            declare_parameter<std::string>("interpolation", "linear"));

        // QoS params (now consistent with other nodes)
        auto sub_rel = declare_parameter<std::string>("sub_qos.reliability", "reliable");
        auto sub_dur = declare_parameter<std::string>("sub_qos.durability", "volatile");
        auto sub_hist = declare_parameter<std::string>("sub_qos.history", "keep_last");
        auto sub_depth = declare_parameter<int>("sub_qos.depth", 5);
        auto pub_rel = declare_parameter<std::string>("pub_qos.reliability", "best_effort");
        auto pub_dur = declare_parameter<std::string>("pub_qos.durability", "volatile");
        auto pub_hist = declare_parameter<std::string>("pub_qos.history", "keep_last");
        auto pub_depth = declare_parameter<int>("pub_qos.depth", 5);

        // Create processor
        processor_ = std::make_unique<StereoRectifyScale>(config);

        // Setup QoS
        auto sub_qos = makeQos(sub_rel, sub_dur, sub_hist, sub_depth);
        auto pub_qos = makeQos(pub_rel, pub_dur, pub_hist, pub_depth);

        // Create synchronized subscribers using message_filters
        img_sub_.subscribe(this, in_image_topic, sub_qos.get_rmw_qos_profile());
        info_sub_.subscribe(this, in_info_topic, sub_qos.get_rmw_qos_profile());

        // Create synchronizer for exact time matching
        using Policy = message_filters::sync_policies::ExactTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
        sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
            Policy(sub_depth), img_sub_, info_sub_);
        sync_->registerCallback(std::bind(&StereoRectifyScaleNode::onSync, this,
                                          std::placeholders::_1, std::placeholders::_2));

        // Create publishers
        img_pub_ = create_publisher<sensor_msgs::msg::Image>(out_image_topic, pub_qos);
        info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(out_info_topic, pub_qos);

        RCLCPP_INFO(get_logger(), "Rectify+Scale: %s + %s -> %s + %s (out %dx%d)",
                    in_image_topic.c_str(), in_info_topic.c_str(),
                    out_image_topic.c_str(), out_info_topic.c_str(),
                    config.output_width, config.output_height);
    }

private:
    void onSync(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        
        // Update camera info
        processor_->updateCameraInfo(*info_msg);

        // Convert to OpenCV
        cv_bridge::CvImageConstPtr cv_in;
        try
        {
            cv_in = cv_bridge::toCvShare(img_msg, img_msg->encoding);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge failed: %s", e.what());
            return;
        }

        // Process
        cv::Mat output;
        sensor_msgs::msg::CameraInfo output_info;
        if (!processor_->process(cv_in->image, output, output_info))
        {
            RCLCPP_ERROR(get_logger(), "Processing failed");
            return;
        }

        // Publish image
        cv_bridge::CvImage out_msg;
        out_msg.header = img_msg->header;
        out_msg.encoding = img_msg->encoding;
        out_msg.image = output;
        img_pub_->publish(*out_msg.toImageMsg());

        // Publish camera info
        output_info.header = img_msg->header;
        info_pub_->publish(output_info);
    }

    std::unique_ptr<StereoRectifyScale> processor_;
    std::mutex mtx_;

    message_filters::Subscriber<sensor_msgs::msg::Image> img_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
    using SyncPolicy = message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoRectifyScaleNode>());
    rclcpp::shutdown();
    return 0;
}
