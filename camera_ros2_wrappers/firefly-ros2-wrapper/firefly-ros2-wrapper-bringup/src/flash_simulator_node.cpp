/**
 * @file flash_simulator_node.cpp
 * @brief Fast flash camera effect simulator for RGB-D images
 * 
 * Subscribes to color and depth images, applies flash simulation, and republishes.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class FlashSimulatorNode : public rclcpp::Node
{
public:
    FlashSimulatorNode() : Node("flash_simulator")
    {
        // Declare parameters with defaults
        this->declare_parameter("flash_intensity", 2.5);
        this->declare_parameter("shutter_speed", 0.1);
        this->declare_parameter("max_flash_distance", 1.5);
        this->declare_parameter("near_plane", 0.01);
        this->declare_parameter("far_plane", 20.0);
        this->declare_parameter("queue_size", 10);
        this->declare_parameter("color_topic", "/firefly_left/image_raw");
        this->declare_parameter("depth_topic", "/firefly_left/depth/image");
        this->declare_parameter("output_topic", "/firefly_left/image");

        // Get parameters
        flash_intensity_ = this->get_parameter("flash_intensity").as_double();
        shutter_speed_ = this->get_parameter("shutter_speed").as_double();
        max_flash_distance_ = this->get_parameter("max_flash_distance").as_double();
        near_plane_ = this->get_parameter("near_plane").as_double();
        far_plane_ = this->get_parameter("far_plane").as_double();
        int queue_size = this->get_parameter("queue_size").as_int();
        std::string color_topic = this->get_parameter("color_topic").as_string();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        // Create synchronized subscribers
        color_sub_.subscribe(this, color_topic);
        depth_sub_.subscribe(this, depth_topic);

        // Create synchronizer with approximate time policy
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(queue_size), color_sub_, depth_sub_);
        sync_->registerCallback(
            std::bind(&FlashSimulatorNode::imageCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Publisher
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Flash Simulator Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Color input: %s", color_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Depth input: %s", depth_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Flash intensity: %.2f", flash_intensity_);
        RCLCPP_INFO(this->get_logger(), "  Shutter speed: %.2f", shutter_speed_);
        RCLCPP_INFO(this->get_logger(), "  Max flash distance: %.2f m", max_flash_distance_);
    }

private:
    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        try {
            // Convert ROS messages to OpenCV
            cv_bridge::CvImageConstPtr color_ptr = cv_bridge::toCvShare(
                color_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(
                depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

            cv::Mat color_img = color_ptr->image;
            cv::Mat depth_img = depth_ptr->image;

            // Apply flash simulation
            cv::Mat flash_img = applyFlashEffect(color_img, depth_img);

            // Publish result
            cv_bridge::CvImage out_msg;
            out_msg.header = color_msg->header;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = flash_img;

            image_pub_->publish(*out_msg.toImageMsg());

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    cv::Mat applyFlashEffect(const cv::Mat& color_img, const cv::Mat& depth_img)
    {
        // Ensure images are same size
        if (color_img.rows != depth_img.rows || color_img.cols != depth_img.cols) {
            RCLCPP_ERROR(this->get_logger(), 
                "Image size mismatch: color %dx%d, depth %dx%d",
                color_img.cols, color_img.rows, depth_img.cols, depth_img.rows);
            return color_img.clone();
        }

        // Linearize depth buffer (convert from normalized depth to meters)
        // For Gazebo depth: depth_value is actual distance in meters
        // No linearization needed if already linear, but we'll normalize it
        cv::Mat linear_depth;
        if (depth_img.type() == CV_32FC1) {
            linear_depth = depth_img.clone();
        } else {
            depth_img.convertTo(linear_depth, CV_32FC1);
        }

        // Normalize depth to [0, 1] range based on max_flash_distance
        cv::Mat depth_normalized = linear_depth / max_flash_distance_;
        cv::threshold(depth_normalized, depth_normalized, 1.0, 1.0, cv::THRESH_TRUNC);
        cv::max(depth_normalized, 0.0, depth_normalized);

        // Calculate flash falloff: exp(-6 * depth^2)
        cv::Mat depth_squared;
        cv::multiply(depth_normalized, depth_normalized, depth_squared);
        cv::Mat flash_falloff;
        cv::exp(-6.0 * depth_squared, flash_falloff);

        // Calculate brightness factor: shutter_speed + flash_intensity * falloff
        cv::Mat flash_contribution = flash_intensity_ * flash_falloff;
        cv::Mat brightness_factor = shutter_speed_ + flash_contribution;

        // Apply brightness factor to each channel
        cv::Mat flash_img = color_img.clone();
        flash_img.convertTo(flash_img, CV_32FC3);

        std::vector<cv::Mat> channels(3);
        cv::split(flash_img, channels);

        for (int i = 0; i < 3; ++i) {
            cv::multiply(channels[i], brightness_factor, channels[i]);
        }

        cv::merge(channels, flash_img);

        // Clip to valid range [0, 255] and convert back to uint8
        cv::threshold(flash_img, flash_img, 255.0, 255.0, cv::THRESH_TRUNC);
        cv::max(flash_img, 0.0, flash_img);
        flash_img.convertTo(flash_img, CV_8UC3);

        return flash_img;
    }

    // Subscribers and publisher
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // Parameters
    double flash_intensity_;
    double shutter_speed_;
    double max_flash_distance_;
    double near_plane_;
    double far_plane_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlashSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
