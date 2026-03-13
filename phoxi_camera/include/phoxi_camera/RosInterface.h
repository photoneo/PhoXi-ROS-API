#ifndef PHOXI_CAMERA_ROSINTERFACE_H
#define PHOXI_CAMERA_ROSINTERFACE_H

#include <memory>
#include <string>

#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera_msgs/srv/connect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace phoxi_camera
{
class RosInterface : public rclcpp_lifecycle::LifecycleNode
{
  public:
    explicit RosInterface(const rclcpp::NodeOptions& options);
    virtual ~RosInterface();

  protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    std::unique_ptr<PhoXiInterface> phoxi_interface_;

  private:
    void connect_cb(const std::shared_ptr<phoxi_camera_msgs::srv::Connect::Request> request,
                    std::shared_ptr<phoxi_camera_msgs::srv::Connect::Response> response);

    void disconnect_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void trigger_frame_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    rclcpp::Service<phoxi_camera_msgs::srv::Connect>::SharedPtr connect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_frame_service_;

    std::string sensor_sn_;
    std::string frame_id_;
};

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROSINTERFACE_H
