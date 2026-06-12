cmake_minimum_required(VERSION 3.22)
project(phoxi_camera_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/FrameInfo.msg"
        "srv/Connect.srv"
        "srv/TriggerFrame.srv"
        "srv/GetProfileList.srv"
        "srv/GetActiveProfile.srv"
        "srv/SetActiveProfile.srv"
        "srv/CreateProfile.srv"
        "srv/DeleteProfile.srv"
        "srv/UpdateProfile.srv"
        "srv/GetStartupProfile.srv"
        "srv/SetStartupProfile.srv"
        "srv/ExportProfile.srv"
        "srv/ImportProfile.srv"
        DEPENDENCIES std_msgs
)

ament_package()
