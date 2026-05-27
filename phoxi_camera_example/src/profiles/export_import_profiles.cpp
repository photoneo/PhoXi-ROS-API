#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "phoxi_camera_msgs/srv/export_profile.hpp"
#include "phoxi_camera_msgs/srv/import_profile.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

template<typename SrvT>
typename SrvT::Response::SharedPtr call_service(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& service_name,
    typename SrvT::Request::SharedPtr request = std::make_shared<typename SrvT::Request>()) {
    auto client = node->create_client<SrvT>(service_name);
    if (!client->wait_for_service(5s)) {
        RCLCPP_ERROR(node->get_logger(), "Service '%s' not available.", service_name.c_str());
        return nullptr;
    }
    return client->async_send_request(request).get();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("export_import_profiles");

    if (argc < 3) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage:\n"
                     "  export <output.bin>\n"
                     "  import <input.bin> <profile_name>");
        rclcpp::shutdown();
        return 1;
    }

    const std::string mode = argv[1];
    const std::string path = argv[2];

    if (mode == "export") {
        auto resp = call_service<phoxi_camera_msgs::srv::ExportProfile>(
            node, "/phoxi_camera/profiles/export");
        if (!resp || !resp->success) {
            RCLCPP_ERROR(node->get_logger(), "Export failed: %s",
                         resp ? resp->message.c_str() : "timeout");
            rclcpp::shutdown();
            return 1;
        }
        std::ofstream file(path, std::ios::binary);
        file.write(reinterpret_cast<const char*>(resp->content.data()),
                   static_cast<std::streamsize>(resp->content.size()));
        RCLCPP_INFO(node->get_logger(), "Exported profile '%s' → %s (%zu bytes)",
                    resp->name.c_str(), path.c_str(), resp->content.size());

    } else if (mode == "import") {
        if (argc < 4) {
            RCLCPP_ERROR(node->get_logger(), "Usage: import <input.bin> <profile_name>");
            rclcpp::shutdown();
            return 1;
        }
        const std::string profile_name = argv[3];

        std::ifstream file(path, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(node->get_logger(), "Cannot open '%s'", path.c_str());
            rclcpp::shutdown();
            return 1;
        }
        const std::vector<uint8_t> data{std::istreambuf_iterator<char>(file),
                                        std::istreambuf_iterator<char>()};

        auto req = std::make_shared<phoxi_camera_msgs::srv::ImportProfile::Request>();
        req->name = profile_name;
        req->content.assign(data.begin(), data.end());

        auto resp = call_service<phoxi_camera_msgs::srv::ImportProfile>(
            node, "/phoxi_camera/profiles/import", req);
        if (!resp || !resp->success) {
            RCLCPP_ERROR(node->get_logger(), "Import failed: %s",
                         resp ? resp->message.c_str() : "timeout");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Imported profile '%s' from %s",
                    profile_name.c_str(), path.c_str());

    } else {
        RCLCPP_ERROR(node->get_logger(), "Unknown mode '%s'. Use 'export' or 'import'.",
                     mode.c_str());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
