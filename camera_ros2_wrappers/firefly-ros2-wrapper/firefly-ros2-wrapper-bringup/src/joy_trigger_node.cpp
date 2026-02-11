#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"

// Xbox mapping (typical on Linux joy_node; verify with `ros2 topic echo /joy`)
enum Axis
{
    LEFT_STICK_X = 0,
    LEFT_STICK_Y = 1,
    LEFT_TRIGGER = 2, // usually 1.0 at rest, -1.0 when fully pressed (driver dependent)
    RIGHT_STICK_X = 3,
    RIGHT_STICK_Y = 4,
    RIGHT_TRIGGER = 5, // usually 1.0 at rest, -1.0 when fully pressed (driver dependent)
    D_PAD_X = 6,
    D_PAD_Y = 7
};

enum Button
{
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LEFT_BUMPER = 4,
    RIGHT_BUMPER = 5,
    CHANGE_VIEW = 6,
    MENU = 7,
    HOME = 8,
    LEFT_STICK_CLICK = 9,
    RIGHT_STICK_CLICK = 10
};

class JoyTriggerNode : public rclcpp::Node
{
public:
    JoyTriggerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("joy_trigger_node", options)
    {
        joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_, rclcpp::SystemDefaultsQoS(),
            std::bind(&JoyTriggerNode::joyCallback, this, std::placeholders::_1));

        trigger_video_client_ = this->create_client<std_srvs::srv::Trigger>("/trigger/start_video");
        stop_video_client_ = this->create_client<std_srvs::srv::Trigger>("/trigger/stop_video");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
    {
        // Check if the A button is pressed
        if (msg->buttons.size() > Button::A && msg->buttons[Button::A] == 1)
        {
            // If the video is not already on, trigger the video start
            if (!video_on_)
            {
                video_on_ = true;
                RCLCPP_INFO(this->get_logger(), "A button pressed - triggering video start");
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                trigger_video_client_->async_send_request(request);
            }
        }
        else if (msg->buttons.size() > Button::A && msg->buttons[Button::A] == 0)
        {
            // If the A button is released and video is on, trigger the video stop
            if (video_on_)
            {
                video_on_ = false;
                RCLCPP_INFO(this->get_logger(), "A button released - triggering video stop");
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                stop_video_client_->async_send_request(request);
            }
        }
    }

    bool video_on_{false};
    std::string joy_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_video_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_video_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<JoyTriggerNode>();
        RCLCPP_INFO(node->get_logger(), "Joy Trigger Node started. Press A button to start/stop video.");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("joy_trigger_node"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
