/**
 * @file image_saver_node.cpp
 * @brief ROS2 node for saving images from a topic to disk with timestamps
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode() : Node("image_saver_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("save_directory", "/tmp/saved_images");
        this->declare_parameter<std::string>("image_prefix", "image");
        this->declare_parameter<std::string>("image_format", "png");
        this->declare_parameter<int>("qos_depth", 10);
        this->declare_parameter<std::string>("qos_reliability", "reliable");
        this->declare_parameter<std::string>("qos_durability", "volatile");
        this->declare_parameter<std::string>("qos_history", "keep_last");

        // Get parameters
        image_topic_ = this->get_parameter("image_topic").as_string();
        save_directory_ = this->get_parameter("save_directory").as_string();
        image_prefix_ = this->get_parameter("image_prefix").as_string();
        image_format_ = this->get_parameter("image_format").as_string();

        // Create save directory if it doesn't exist
        try
        {
            if (!fs::exists(save_directory_))
            {
                fs::create_directories(save_directory_);
                RCLCPP_INFO(this->get_logger(), "Created save directory: %s", save_directory_.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Using existing save directory: %s", save_directory_.c_str());
            }

            // Test write permissions by attempting to create a test file
            std::string test_file = save_directory_ + "/.write_test";
            std::ofstream test_stream(test_file);
            if (!test_stream.good())
            {
                RCLCPP_ERROR(this->get_logger(),
                             "No write permission for directory '%s'. Please check permissions or use a different directory.",
                             save_directory_.c_str());
                throw std::runtime_error("No write permission for save directory");
            }
            test_stream.close();
            fs::remove(test_file);
        }
        catch (const fs::filesystem_error &e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to create directory '%s': %s\nPlease use a directory you have write access to (e.g., /home/hayden/saved_images)",
                         save_directory_.c_str(), e.what());
            throw;
        }

        // Setup QoS profile
        auto qos_depth = this->get_parameter("qos_depth").as_int();
        auto qos_reliability = this->get_parameter("qos_reliability").as_string();
        auto qos_durability = this->get_parameter("qos_durability").as_string();
        auto qos_history = this->get_parameter("qos_history").as_string();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));

        if (qos_reliability == "reliable")
        {
            qos.reliable();
        }
        else
        {
            qos.best_effort();
        }

        if (qos_durability == "transient_local")
        {
            qos.transient_local();
        }
        else
        {
            qos.durability_volatile();
        }

        // Create subscriber
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, qos,
            std::bind(&ImageSaverNode::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Image Saver Node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Saving images to: %s", save_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Image format: %s", image_format_.c_str());

        image_count_ = 0;
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS image to OpenCV image (BGR8 for color, MONO8 for grayscale)
            cv_bridge::CvImagePtr cv_ptr;
            if (msg->encoding == sensor_msgs::image_encodings::MONO8 ||
                msg->encoding == sensor_msgs::image_encodings::MONO16)
            {
                cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            }
            else
            {
                // Convert to BGR8 for OpenCV's native color format
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }

            // Generate filename with timestamp
            auto now = std::chrono::system_clock::now();
            auto now_time_t = std::chrono::system_clock::to_time_t(now);
            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch()) %
                          1000;

            std::ostringstream filename;
            filename << save_directory_ << "/"
                     << image_prefix_ << "_"
                     << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S")
                     << "_" << std::setfill('0') << std::setw(3) << now_ms.count()
                     << "." << image_format_;

            // Save image
            cv::imwrite(filename.str(), cv_ptr->image);

            image_count_++;

            if (image_count_ % 10 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Saved %zu images (latest: %s)",
                            image_count_, filename.str().c_str());
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::string image_topic_;
    std::string save_directory_;
    std::string image_prefix_;
    std::string image_format_;
    size_t image_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<ImageSaverNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("image_saver_node"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
