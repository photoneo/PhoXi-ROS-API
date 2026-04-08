#include "phoxi_camera/RosInterface.h"

#include <chrono>

#include "lifecycle_msgs/msg/state.hpp"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/RosConversions.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace phoxi_camera
{

RosInterface::RosInterface(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("phoxi_camera", options) {
    RCLCPP_INFO(this->get_logger(), "Creating PhoXi Camera node.");
    this->declare_parameter<std::string>("sensor_sn", "");
    this->declare_parameter<std::string>("frame_id", "phoxi_camera_sensor");
}

RosInterface::~RosInterface() {
    if (phoxi_interface_) {
        phoxi_interface_->disconnectCamera();
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RosInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Configuring PhoXi Camera node...");

    this->get_parameter("sensor_sn", sensor_sn_);
    this->get_parameter("frame_id", frame_id_);

    if (!phoxi_interface_) {
        phoxi_interface_ = std::make_unique<PhoXiInterface>();
    }

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", rclcpp::SystemDefaultsQoS());

    connect_service_ = this->create_service<phoxi_camera_msgs::srv::Connect>(
        "~/connect",
        std::bind(&RosInterface::connect_cb, this, std::placeholders::_1, std::placeholders::_2));
    disconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/disconnect", std::bind(&RosInterface::disconnect_cb, this, std::placeholders::_1,
                                  std::placeholders::_2));
    trigger_frame_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/trigger_frame", std::bind(&RosInterface::trigger_frame_cb, this, std::placeholders::_1,
                                     std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Configuration complete.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RosInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Activating PhoXi Camera node...");

    point_cloud_pub_->on_activate();

    RCLCPP_INFO(this->get_logger(), "Activation complete.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RosInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Deactivating PhoXi Camera node...");

    point_cloud_pub_->on_deactivate();

    if (phoxi_interface_) {
        phoxi_interface_->disconnectCamera();
    }

    RCLCPP_INFO(this->get_logger(), "Deactivation complete.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RosInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Cleaning up PhoXi Camera node...");

    phoxi_interface_.reset();
    point_cloud_pub_.reset();
    connect_service_.reset();
    disconnect_service_.reset();
    trigger_frame_service_.reset();

    RCLCPP_INFO(this->get_logger(), "Cleanup complete.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RosInterface::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Shutting down PhoXi Camera node...");

    phoxi_interface_.reset();
    point_cloud_pub_.reset();
    connect_service_.reset();
    disconnect_service_.reset();
    trigger_frame_service_.reset();

    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void RosInterface::connect_cb(
    const std::shared_ptr<phoxi_camera_msgs::srv::Connect::Request> request,
    std::shared_ptr<phoxi_camera_msgs::srv::Connect::Response> response) {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(this->get_logger(), "Node is not active. Cannot connect.");
        response->success = false;
        response->message = "Node is not in the active state.";
        return;
    }

    std::string sn_to_use = request->sn.empty() ? sensor_sn_ : request->sn;

    if (sn_to_use.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Connection failed: No serial number provided either as a parameter or in the "
                     "service request.");
        response->success = false;
        response->message = "No serial number provided.";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Service call to connect to camera with SN: %s",
                sn_to_use.c_str());

    auto getFrameCallback = [this](pho::api::PFrame pframe) {
        try {
            std::unique_ptr<sensor_msgs::msg::PointCloud2> msg = phoXiFrameToRosMsg(pframe);

            msg->header.frame_id = frame_id_;
            std::chrono::nanoseconds stamp{0};

            if (pframe->Info.FrameStartTime.IsValid()) {
                stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    pframe->Info.FrameStartTime.Time.time_since_epoch());
            } else if (pframe->Info.FrameTimestamp > 0.) {
                stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double, std::milli>(pframe->Info.FrameTimestamp));
            }
            if (stamp.count() > 0) {
                msg->header.stamp.sec = stamp.count() / 1000000000;
                msg->header.stamp.nanosec = stamp.count() % 1000000000;
            } else {
                msg->header.stamp = this->get_clock()->now();
            }

            if (point_cloud_pub_->is_activated()) {
                point_cloud_pub_->publish(std::move(msg));
            }

            RCLCPP_INFO(this->get_logger(), "Frame published successfully.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to trigger and publish frame: %s", e.what());
        }
    };

    try {
        phoxi_interface_->connectCamera(sn_to_use, getFrameCallback);
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", e.what());
        response->success = false;
        response->message = e.what();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully connected to the PhoXi device.");
    response->success = true;
}

void RosInterface::disconnect_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(this->get_logger(), "Node is not active. Cannot disconnect.");
        response->success = false;
        response->message = "Node is not in the active state.";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Service call to disconnect from camera.");
    try {
        phoxi_interface_->disconnectCamera();
    } catch (PhoXiInterfaceException& e) {
        // device is already disconnected on exception
    }
    RCLCPP_INFO(this->get_logger(), "Successfully disconnected the PhoXi device.");
    response->success = true;
}

void RosInterface::trigger_frame_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(this->get_logger(), "Node is not active. Cannot trigger frame.");
        response->success = false;
        response->message = "Node is not in the active state.";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Service call to trigger a frame.");
    try {
        phoxi_interface_->triggerFrame();
        RCLCPP_INFO(this->get_logger(), "Frame triggered successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to trigger frame: %s", e.what());
        response->success = false;
        response->message = e.what();
        return;
    }
    response->success = true;
    return;
}

}  // namespace phoxi_camera

RCLCPP_COMPONENTS_REGISTER_NODE(phoxi_camera::RosInterface)
