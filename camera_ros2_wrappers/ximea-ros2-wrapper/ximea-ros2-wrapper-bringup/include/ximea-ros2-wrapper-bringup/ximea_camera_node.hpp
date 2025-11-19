#ifndef XIMEA_ROS2_WRAPPER_BRINGUP_XIMEA_CAMERA_NODE_HPP_
#define XIMEA_ROS2_WRAPPER_BRINGUP_XIMEA_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <m3api/xiApi.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <filesystem>
#include <cstdlib>
#include <functional>


class XimeaCameraNode : public rclcpp::Node
{
public:
    explicit XimeaCameraNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~XimeaCameraNode();

private:
    void capture_thread_func();
    bool initialize_camera();
    bool init_software_ffc();
    cv::Mat apply_software_ffc(cv::Mat &raw);
    cv::Mat capture_raw_image();
    cv::Mat capture_calibrated_image();
    void publish_image(cv::Mat &image);
    rcl_interfaces::msg::SetParametersResult on_parameters_set(
        const std::vector<rclcpp::Parameter> &params);

    // ROS 2 components
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Capture thread
    std::thread capture_thread_;
    std::atomic<bool> running_;

    // Parameters
    std::string camera_name_;
    std::string frame_id_;
    float gain_;
    int exposure_time_;
    bool enable_ffc_;
    int device_id_;
    int trigger_timeout_ms_;

    // XIMEA Camera Handle
    HANDLE xi_handle_;
    bool camera_initialized_;

    // FFC calibration
    std::string ffc_dir_;
    std::string mid_file_;
    std::string dark_file_;
    cv::Mat dark_;
    cv::Mat mid_;
    cv::Mat mid_dark_;
    cv::Mat FFC_;
    float mid_dark_mean_;
};

#endif // XIMEA_ROS2_WRAPPER_BRINGUP_XIMEA_CAMERA_NODE_HPP_
