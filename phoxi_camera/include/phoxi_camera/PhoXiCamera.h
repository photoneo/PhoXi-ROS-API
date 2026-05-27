#ifndef PHOXI_CAMERA_ROSINTERFACE_H
#define PHOXI_CAMERA_ROSINTERFACE_H

#include <memory>
#include <mutex>
#include <string>

#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera_msgs/srv/connect.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace phoxi_camera
{
class PhoXiCamera : public rclcpp_lifecycle::LifecycleNode
{
  public:
    explicit PhoXiCamera(const std::string& deviceId, const rclcpp::NodeOptions& options);
    explicit PhoXiCamera(const rclcpp::NodeOptions& options);
    ~PhoXiCamera() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  protected:
    std::unique_ptr<PhoXiInterface> mPhoXiInterface;

  private:
    void drainFrameCallback();
    void connect_cb(const std::shared_ptr<const phoxi_camera_msgs::srv::Connect::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::Connect::Response>& response);
    void disconnect_cb(const std::shared_ptr<const std_srvs::srv::Trigger::Request>& request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) const;
    void trigger_frame_cb(const std::shared_ptr<const phoxi_camera_msgs::srv::TriggerFrame::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::TriggerFrame::Response>& response) const;
    void on_frame_cb(const pho::api::PFrame& frame);

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointsPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mNormalMapPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mDepthMapPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mConfidenceMapPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mEventMapPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mTexturePub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mTextureRgbPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mColorCameraImagePub;
    rclcpp::Service<phoxi_camera_msgs::srv::Connect>::SharedPtr mConnectService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mDisconnectService;
    rclcpp::Service<phoxi_camera_msgs::srv::TriggerFrame>::SharedPtr mTriggerFrameService;
    std::mutex mFrameMutex;
    std::string mDeviceId;
    std::string mFrameId;
};

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROSINTERFACE_H
