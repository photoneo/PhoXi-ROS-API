#ifndef PHOXI_CAMERA_ROSINTERFACE_H
#define PHOXI_CAMERA_ROSINTERFACE_H

#include <memory>
#include <mutex>
#include <string>

#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera_msgs/srv/connect.hpp"
#include "phoxi_camera_msgs/srv/create_profile.hpp"
#include "phoxi_camera_msgs/srv/delete_profile.hpp"
#include "phoxi_camera_msgs/srv/export_profile.hpp"
#include "phoxi_camera_msgs/srv/get_active_profile.hpp"
#include "phoxi_camera_msgs/srv/get_profile_list.hpp"
#include "phoxi_camera_msgs/srv/get_startup_profile.hpp"
#include "phoxi_camera_msgs/srv/import_profile.hpp"
#include "phoxi_camera_msgs/srv/set_active_profile.hpp"
#include "phoxi_camera_msgs/srv/set_startup_profile.hpp"
#include "phoxi_camera_msgs/srv/update_profile.hpp"
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
    explicit PhoXiCamera(std::string deviceId, const rclcpp::NodeOptions& options);
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
    void declareParameters();
    void activatePublishers();
    void deactivatePublishers();
    void cleanupResources();

    void onFrameCallback(const PhoXiFrame& frame);
    rcl_interfaces::msg::SetParametersResult onParametersChanged(
        const std::vector<rclcpp::Parameter>& params);

    void connectCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::Connect::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::Connect::Response>& response);
    void disconnectCallback(const std::shared_ptr<const std_srvs::srv::Trigger::Request>& request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);
    void triggerFrameCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::TriggerFrame::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::TriggerFrame::Response>& response);
    void getProfileListCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::GetProfileList::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::GetProfileList::Response>& response);
    void getActiveProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::GetActiveProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::GetActiveProfile::Response>& response);
    void setActiveProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::SetActiveProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::SetActiveProfile::Response>& response);
    void createProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::CreateProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::CreateProfile::Response>& response);
    void deleteProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::DeleteProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::DeleteProfile::Response>& response);
    void updateProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::UpdateProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::UpdateProfile::Response>& response);
    void getStartupProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::GetStartupProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::GetStartupProfile::Response>& response);
    void setStartupProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::SetStartupProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::SetStartupProfile::Response>& response);
    void exportProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::ExportProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::ExportProfile::Response>& response);
    void importProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::ImportProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::ImportProfile::Response>& response);
    void resetActiveProfileCallback(const std::shared_ptr<const std_srvs::srv::Trigger::Request>& request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);

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
    rclcpp::Service<phoxi_camera_msgs::srv::GetProfileList>::SharedPtr mGetProfileListService;
    rclcpp::Service<phoxi_camera_msgs::srv::GetActiveProfile>::SharedPtr mGetActiveProfileService;
    rclcpp::Service<phoxi_camera_msgs::srv::SetActiveProfile>::SharedPtr mSetActiveProfileService;
    rclcpp::Service<phoxi_camera_msgs::srv::CreateProfile>::SharedPtr mCreateProfileService;
    rclcpp::Service<phoxi_camera_msgs::srv::DeleteProfile>::SharedPtr mDeleteProfileService;
    rclcpp::Service<phoxi_camera_msgs::srv::UpdateProfile>::SharedPtr mUpdateProfileService;
    rclcpp::Service<phoxi_camera_msgs::srv::GetStartupProfile>::SharedPtr mGetStartupProfileService;
    rclcpp::Service<phoxi_camera_msgs::srv::SetStartupProfile>::SharedPtr mSetStartupProfileService;
    rclcpp::Service<phoxi_camera_msgs::srv::ExportProfile>::SharedPtr mExportProfileService;
    rclcpp::Service<phoxi_camera_msgs::srv::ImportProfile>::SharedPtr mImportProfileService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mResetActiveProfileService;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr mParamCallbackHandle;
    std::mutex mFrameMutex;
    std::string mDeviceId;
    std::string mFrameId;
    bool mPublishCombined = false;
};

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROSINTERFACE_H
