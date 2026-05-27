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
 * CompositionExample – composable node that drives a maximum-rate
 * acquisition loop together with the phoxi_camera driver.
 *
 * Workflow
 * --------
 *   1. Call the ~/start service → the node triggers the first frame via
 *      phoxi_camera's trigger_frame service.
 *   2. phoxi_camera publishes a PointCloud2 on /point_cloud.
 *   3. on_point_cloud() counts non-zero 3D points, logs timing / FPS, then
 *      calls trigger_frame again (if still running).
 *   4. Call ~/stop to break the loop.
 *
 * Parameters
 * ----------
 *   trigger_service_name (string, default "/phoxi_camera/trigger_frame")
 *     Full name of the trigger-frame service on the camera driver node.
 */
class CompositionExample : public rclcpp::Node {
public:
    explicit CompositionExample(const rclcpp::NodeOptions& options);

private:
    // ------------------------------------------------------------------ //
    //  Callbacks                                                          //
    // ------------------------------------------------------------------ //
    void on_point_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    void on_color_camera_image(sensor_msgs::msg::Image::ConstSharedPtr msg);

    void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void on_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // ------------------------------------------------------------------ //
    //  Helpers                                                            //
    // ------------------------------------------------------------------ //
    void trigger_next_frame();

    // ------------------------------------------------------------------ //
    //  ROS interfaces                                                     //
    // ------------------------------------------------------------------ //
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_camera_image_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

    // Each group is MutuallyExclusive so the multi-threaded container can
    // run the subscription callback and the start/stop service callbacks
    // concurrently without risking a deadlock on the trigger service call.
    rclcpp::CallbackGroup::SharedPtr cb_group_sub_;
    rclcpp::CallbackGroup::SharedPtr cb_group_srv_;
    rclcpp::CallbackGroup::SharedPtr cb_group_client_;

    // ------------------------------------------------------------------ //
    //  State                                                              //
    // ------------------------------------------------------------------ //
    std::atomic<bool> running_{false};

    uint64_t frame_count_{0};
    std::chrono::steady_clock::time_point loop_start_time_;
    std::chrono::steady_clock::time_point last_frame_time_;
    bool timing_initialized_{false};

    std::string trigger_service_name_;
};

}  // namespace phoxi_camera::composition_example

#endif
