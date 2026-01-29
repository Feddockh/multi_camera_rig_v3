#ifndef FIREFLY_RECONSTRUCTION_QOS_UTILS_HPP
#define FIREFLY_RECONSTRUCTION_QOS_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <stdexcept>
#include <string>

namespace firefly_reconstruction
{

/**
 * @brief Convert string to lowercase
 */
inline std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });
    return s;
}

/**
 * @brief Parse reliability policy from string
 * @param s String value: "reliable", "best_effort", "system_default"
 * @return RMW reliability policy
 */
inline rmw_qos_reliability_policy_t parseReliability(const std::string &s)
{
    const auto v = toLower(s);
    if (v == "reliable")
        return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    if (v == "best_effort" || v == "besteffort" || v == "best-effort")
        return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    if (v == "system_default" || v == "default")
        return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    throw std::runtime_error("Invalid reliability: " + s);
}

/**
 * @brief Parse durability policy from string
 * @param s String value: "volatile", "transient_local", "system_default"
 * @return RMW durability policy
 */
inline rmw_qos_durability_policy_t parseDurability(const std::string &s)
{
    const auto v = toLower(s);
    if (v == "volatile")
        return RMW_QOS_POLICY_DURABILITY_VOLATILE;
    if (v == "transient_local" || v == "transientlocal" || v == "transient-local")
        return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    if (v == "system_default" || v == "default")
        return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    throw std::runtime_error("Invalid durability: " + s);
}

/**
 * @brief Parse history policy from string
 * @param s String value: "keep_last", "keep_all", "system_default"
 * @return RMW history policy
 */
inline rmw_qos_history_policy_t parseHistory(const std::string &s)
{
    const auto v = toLower(s);
    if (v == "keep_last" || v == "keeplast")
        return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    if (v == "keep_all" || v == "keepall")
        return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    if (v == "system_default" || v == "default")
        return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    throw std::runtime_error("Invalid history: " + s);
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
    auto hist = parseHistory(history);

    rclcpp::QoS qos =
        (hist == RMW_QOS_POLICY_HISTORY_KEEP_ALL)
            ? rclcpp::QoS(rclcpp::KeepAll())
            : rclcpp::QoS(rclcpp::KeepLast(std::max(1, depth)));

    switch (parseReliability(reliability))
    {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        qos.reliable();
        break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
        qos.best_effort();
        break;
    default:
        break;
    }

    switch (parseDurability(durability))
    {
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
        qos.durability_volatile();
        break;
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
        qos.transient_local();
        break;
    default:
        break;
    }

    return qos;
}

} // namespace firefly_reconstruction

#endif // FIREFLY_RECONSTRUCTION_QOS_UTILS_HPP
