#ifndef MULTI_CAMERA_RIG_COMMON_QOS_UTILS_HPP
#define MULTI_CAMERA_RIG_COMMON_QOS_UTILS_HPP

#include <rclcpp/qos.hpp>
#include <algorithm>
#include <stdexcept>
#include <string>

namespace multi_camera_rig_common
{

/**
 * @brief Convert string to lowercase
 */
inline std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

/**
 * @brief Parse reliability policy from string
 * @param s String value: "reliable", "best_effort", "system_default"
 * @return RMW reliability policy
 */
inline rmw_qos_reliability_policy_t parseReliability(const std::string &s)
{
    auto lower = toLower(s);
    if (lower == "reliable") return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    if (lower == "best_effort") return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    if (lower == "system_default") return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    throw std::invalid_argument("Invalid reliability: " + s);
}

/**
 * @brief Parse durability policy from string
 * @param s String value: "volatile", "transient_local", "system_default"
 * @return RMW durability policy
 */
inline rmw_qos_durability_policy_t parseDurability(const std::string &s)
{
    auto lower = toLower(s);
    if (lower == "volatile") return RMW_QOS_POLICY_DURABILITY_VOLATILE;
    if (lower == "transient_local") return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    if (lower == "system_default") return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    throw std::invalid_argument("Invalid durability: " + s);
}

/**
 * @brief Parse history policy from string
 * @param s String value: "keep_last", "keep_all", "system_default"
 * @return RMW history policy
 */
inline rmw_qos_history_policy_t parseHistory(const std::string &s)
{
    auto lower = toLower(s);
    if (lower == "keep_last") return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    if (lower == "keep_all") return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    if (lower == "system_default") return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    throw std::invalid_argument("Invalid history: " + s);
}

/**
 * @brief Build rclcpp::QoS from string parameters
 * @param reliability Reliability policy string
 * @param durability Durability policy string
 * @param history History policy string
 * @param depth Queue depth for keep_last policy
 * @return Configured QoS object
 */
inline rclcpp::QoS makeQos(
    const std::string &reliability,
    const std::string &durability,
    const std::string &history,
    int depth)
{
    // Parse policies
    auto rel_policy = parseReliability(reliability);
    auto dur_policy = parseDurability(durability);
    auto hist_policy = parseHistory(history);

    // Start with system default profile
    rmw_qos_profile_t profile = rmw_qos_profile_system_default;

    // Override with user settings
    profile.reliability = rel_policy;
    profile.durability = dur_policy;
    profile.history = hist_policy;

    // Set depth (only relevant for keep_last)
    if (hist_policy == RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    {
        profile.depth = static_cast<size_t>(depth);
    }

    // Construct rclcpp::QoS from profile
    // Use KeepLast or KeepAll based on history policy
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(profile));

    // Apply the full profile
    qos.reliability(rel_policy);
    qos.durability(dur_policy);
    qos.history(hist_policy);

    if (hist_policy == RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    {
        qos.keep_last(static_cast<size_t>(depth));
    }
    else if (hist_policy == RMW_QOS_POLICY_HISTORY_KEEP_ALL)
    {
        qos.keep_all();
    }

    return qos;
}

} // namespace multi_camera_rig_common

#endif // MULTI_CAMERA_RIG_COMMON_QOS_UTILS_HPP
