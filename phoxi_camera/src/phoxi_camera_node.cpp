#include "phoxi_camera/PhoXiCamera.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto lcNode = std::make_shared<phoxi_camera::PhoXiCamera>(options);
    executor.add_node(lcNode->get_node_base_interface());

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
