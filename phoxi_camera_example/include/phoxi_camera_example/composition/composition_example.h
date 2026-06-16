/**
 * @file composition_example.h
 * @brief Composable node demonstrating maximum-rate acquisition with the phoxi_camera driver.
 */
#ifndef PHOXI_CAMERA_EXAMPLE_COMPOSITION_COMPOSITION_EXAMPLE_H
#define PHOXI_CAMERA_EXAMPLE_COMPOSITION_COMPOSITION_EXAMPLE_H

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace phoxi_camera::composition_example {

/**
 * @brief Composable node demonstrating maximum-rate acquisition with the phoxi_camera driver.
 *
 * Drives a back-to-back trigger loop: each arriving `point_cloud` frame immediately
 * re-triggers the next one, achieving the highest sustainable frame rate.
 *
 * **Workflow**
 *
 * 1. Call `~/start` — the node sends the first trigger via the phoxi_camera `trigger_frame` service.
 * 2. phoxi_camera publishes `PointCloud2` on `point_cloud`.
 * 3. `on_point_cloud()` counts non-zero points, logs FPS and timing, then triggers the next frame.
 * 4. Call `~/stop` to break the loop.
 *
 * **Parameters**
 *
 * - `trigger_service_name` (string, default `"/phoxi_camera/trigger_frame"`):
 *   Full service name of the trigger-frame service on the camera driver node.
 *
 * **Subscribed topics**
 *
 * - `point_cloud` (`sensor_msgs/PointCloud2`): Incoming point clouds from phoxi_camera.
 * - `color_camera_image` (`sensor_msgs/Image`): Color camera images (logged when received).
 *
 * **Services**
 *
 * - `~/start` (`std_srvs/Trigger`): Start the acquisition loop.
 * - `~/stop` (`std_srvs/Trigger`): Stop the acquisition loop.
 */
class CompositionExample : public rclcpp::Node {
public:
    /**
     * @brief Construct the node, declare parameters, and set up all ROS interfaces.
     * @param options ROS2 node options (passed by the component container).
     */
    explicit CompositionExample(const rclcpp::NodeOptions& options);

private:
    /** @brief Count non-zero points, log FPS/timing, and trigger the next frame if running. */
    void on_point_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    /** @brief Log receipt of a color camera image. */
    void on_color_camera_image(sensor_msgs::msg::Image::ConstSharedPtr msg);

    /** @brief Start the acquisition loop; triggers the first frame. */
    void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /** @brief Stop the acquisition loop. */
    void on_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /** @brief Send an asynchronous trigger_frame request to phoxi_camera. */
    void trigger_next_frame();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_camera_image_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

    // Separate MutuallyExclusive callback groups prevent deadlock when the subscription
    // callback calls the trigger service while start/stop service callbacks are pending.
    rclcpp::CallbackGroup::SharedPtr cb_group_sub_;
    rclcpp::CallbackGroup::SharedPtr cb_group_srv_;
    rclcpp::CallbackGroup::SharedPtr cb_group_client_;

    std::atomic<bool> running_{false};       ///< True while the acquisition loop is active.
    uint64_t frame_count_{0};                ///< Number of frames received since the loop started.
    std::chrono::steady_clock::time_point loop_start_time_;  ///< Timestamp when the loop was started.
    std::chrono::steady_clock::time_point last_frame_time_;  ///< Timestamp of the previous frame arrival.
    bool timing_initialized_{false};         ///< True once the first frame has been received.
    std::string trigger_service_name_;       ///< Resolved name of the phoxi_camera trigger service.
};

}  // namespace phoxi_camera::composition_example

#endif
