/*
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
#include <string>

static std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

static rmw_qos_reliability_policy_t parseReliability(const std::string& s)
{
  const auto v = toLower(s);
  if (v == "reliable") return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  if (v == "best_effort" || v == "besteffort" || v == "best-effort")
    return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  if (v == "system_default" || v == "default")
    return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  throw std::runtime_error("Invalid reliability: " + s);
}

static rmw_qos_durability_policy_t parseDurability(const std::string& s)
{
  const auto v = toLower(s);
  if (v == "volatile") return RMW_QOS_POLICY_DURABILITY_VOLATILE;
  if (v == "transient_local" || v == "transientlocal" || v == "transient-local")
    return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  if (v == "system_default" || v == "default")
    return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  throw std::runtime_error("Invalid durability: " + s);
}

static rmw_qos_history_policy_t parseHistory(const std::string& s)
{
  const auto v = toLower(s);
  if (v == "keep_last" || v == "keeplast") return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  if (v == "keep_all"  || v == "keepall")  return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  if (v == "system_default" || v == "default")
    return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
  throw std::runtime_error("Invalid history: " + s);
}

// Build an rclcpp::QoS from user strings (Humble API)
static rclcpp::QoS makeQos(
  const std::string& reliability,
  const std::string& durability,
  const std::string& history,
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
  switch (parseReliability(reliability)) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE: qos.reliable(); break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT: qos.best_effort(); break;
    default: /* SYSTEM_DEFAULT */ break;
  }

  // Apply durability
  switch (parseDurability(durability)) {
    case RMW_QOS_POLICY_DURABILITY_VOLATILE: qos.durability_volatile(); break;
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL: qos.transient_local(); break;
    default: /* SYSTEM_DEFAULT */ break;
  }

  return qos;
}

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

    auto sub_qos = makeQos(sub_rel_, sub_dur_, sub_hist_, sub_depth_);
    auto pub_qos = makeQos(pub_rel_, pub_dur_, pub_hist_, pub_depth_);

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
