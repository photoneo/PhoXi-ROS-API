#include "phoxi_camera/RosInterface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto lc_node = std::make_shared<phoxi_camera::RosInterface>(options);
    executor.add_node(lc_node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
