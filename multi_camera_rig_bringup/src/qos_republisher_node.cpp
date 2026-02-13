]633;E;git commit -m "Added controller trigger node to the trigger package.";b94f454c-51cc-4814-a680-5feb2a77d27e]633;C/*
Examples
Best effort --> Reliable
ros2 run firefly-ros2-wrapper-bringup qos_republisher_node --ros-args \
  -p type:=sensor_msgs/msg/Image \
  -p in_topic:=/firefly_left/image_raw \
  -p out_topic:=/firefly_left/image_raw_be \
  -p sub_qos.reliability:=reliable \
  -p sub_qos.depth:=5 \
  -p pub_qos.reliability:=best_effort \
  -p pub_qos.depth:=5

ros2 run firefly-ros2-wrapper-bringup qos_republisher_node --ros-args \
  -p type:=sensor_msgs/msg/CameraInfo \
  -p in_topic:=/firefly_left/camera_info \
  -p out_topic:=/firefly_left/camera_info_be \
  -p sub_qos.reliability:=reliable \
  -p pub_qos.reliability:=best_effort
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

#include <rmw/types.h>
#include <rmw/qos_profiles.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <memory>
#include <stdexcept>
#include "multi_camera_rig_common/qos_utils.hpp"


class QosRepublisherNode : public rclcpp::Node
{
public:
  QosRepublisherNode() : Node("qos_republisher_node")
  {
    in_topic_  = declare_parameter<std::string>("in_topic",  "/in");
    out_topic_ = declare_parameter<std::string>("out_topic", "/out");
    type_      = declare_parameter<std::string>("type", "");

    if (type_.empty()) {
      throw std::runtime_error("Parameter 'type' must be set (e.g. sensor_msgs/msg/Image)");
    }

    sub_rel_  = declare_parameter<std::string>("sub_qos.reliability", "reliable");
    sub_dur_  = declare_parameter<std::string>("sub_qos.durability",  "volatile");
    sub_hist_ = declare_parameter<std::string>("sub_qos.history",     "keep_last");
    sub_depth_= declare_parameter<int>("sub_qos.depth", 5);

    pub_rel_  = declare_parameter<std::string>("pub_qos.reliability", "best_effort");
    pub_dur_  = declare_parameter<std::string>("pub_qos.durability",  "volatile");
    pub_hist_ = declare_parameter<std::string>("pub_qos.history",     "keep_last");
    pub_depth_= declare_parameter<int>("pub_qos.depth", 5);

    auto sub_qos = multi_camera_rig_common::makeQos(sub_rel_, sub_dur_, sub_hist_, sub_depth_);
    auto pub_qos = multi_camera_rig_common::makeQos(pub_rel_, pub_dur_, pub_hist_, pub_depth_);

    // Create generic publisher/subscription with serialized passthrough
    rclcpp::PublisherOptions pub_opts;
    pub_ = this->create_generic_publisher(out_topic_, type_, pub_qos, pub_opts);

    rclcpp::SubscriptionOptions sub_opts;
    sub_ = this->create_generic_subscription(
      in_topic_, type_, sub_qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        pub_->publish(*msg);
      },
      sub_opts);

    RCLCPP_INFO(get_logger(),
      "QoS republisher:\n"
      "  type: %s\n"
      "  in:   %s\n"
      "  out:  %s\n"
      "  sub_qos: reliability=%s durability=%s history=%s depth=%d\n"
      "  pub_qos: reliability=%s durability=%s history=%s depth=%d",
      type_.c_str(), in_topic_.c_str(), out_topic_.c_str(),
      sub_rel_.c_str(), sub_dur_.c_str(), sub_hist_.c_str(), sub_depth_,
      pub_rel_.c_str(), pub_dur_.c_str(), pub_hist_.c_str(), pub_depth_);
  }

private:
  std::string in_topic_, out_topic_, type_;
  std::string sub_rel_, sub_dur_, sub_hist_;
  std::string pub_rel_, pub_dur_, pub_hist_;
  int sub_depth_{5}, pub_depth_{5};

  rclcpp::GenericPublisher::SharedPtr pub_;
  rclcpp::GenericSubscription::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<QosRepublisherNode>());
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
