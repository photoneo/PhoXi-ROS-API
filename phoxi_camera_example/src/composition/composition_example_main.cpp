#include "phoxi_camera_example/composition/composition_example.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto node = std::make_shared<phoxi_camera::composition_example::CompositionExample>(options);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
