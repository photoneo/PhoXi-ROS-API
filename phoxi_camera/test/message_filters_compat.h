#pragma once

#include "message_filters/subscriber.hpp"
#include "rclcpp/version.h"

// message_filters::Subscriber::subscribe() API changed in rclcpp 29 (Kilted):
// - rclcpp < 29  (Humble, Jazzy): takes rmw_qos_profile_t
// - rclcpp >= 29 (Kilted, Lyrical, Rolling): takes rclcpp::QoS directly
//   (the rmw_qos_profile_t overload is deprecated in 29 and removed in 30+)
template <typename MsgT>
inline void mfSubscribe(message_filters::Subscriber<MsgT>& sub, const std::shared_ptr<rclcpp::Node>& node, const std::string& topic, const rclcpp::QoS& qos) {
#if RCLCPP_VERSION_MAJOR >= 29
    sub.subscribe(node, topic, qos);
#else
    sub.subscribe(node, topic, qos.get_rmw_qos_profile());
#endif
}
