#include "phoxi_camera_example/composition/composition_example.h"

#include "rclcpp/version.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace phoxi_camera::composition_example {

CompositionExample::CompositionExample(const rclcpp::NodeOptions& options) : rclcpp::Node("phoxi_camera_example_client", options) {
    this->declare_parameter<std::string>("trigger_service_name", "/phoxi_camera/trigger_frame");
    trigger_service_name_ = this->get_parameter("trigger_service_name").as_string();

    // Separate, mutually-exclusive callback groups prevent a deadlock when
    // the subscription callback fires an async service request to a node that
    // lives in the same multi-threaded component container.
    cb_group_sub_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_srv_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_client_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_sub_;

    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", rclcpp::SystemDefaultsQoS(), std::bind(&CompositionExample::on_point_cloud, this, std::placeholders::_1), sub_opts);

    color_camera_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/color_camera_image", rclcpp::SystemDefaultsQoS(), std::bind(&CompositionExample::on_color_camera_image, this, std::placeholders::_1), sub_opts);

#if defined(RCLCPP_VERSION_MAJOR) && (RCLCPP_VERSION_MAJOR < 21)
    const auto service_qos = rclcpp::ServicesQoS().get_rmw_qos_profile();
#else
    const auto service_qos = rclcpp::ServicesQoS();
#endif

    trigger_client_ = create_client<std_srvs::srv::Trigger>(trigger_service_name_, service_qos, cb_group_client_);

    start_service_ = create_service<std_srvs::srv::Trigger>(
            "~/start", std::bind(&CompositionExample::on_start, this, std::placeholders::_1, std::placeholders::_2), service_qos, cb_group_srv_);

    stop_service_ = create_service<std_srvs::srv::Trigger>(
            "~/stop", std::bind(&CompositionExample::on_stop, this, std::placeholders::_1, std::placeholders::_2), service_qos, cb_group_srv_);

    RCLCPP_INFO(get_logger(), "PhoXiCameraExampleClient ready.");
    RCLCPP_INFO(get_logger(), "  Trigger service : %s", trigger_service_name_.c_str());
    RCLCPP_INFO(get_logger(), "  Call ~/start to begin the maximum-rate acquisition loop.");
    RCLCPP_INFO(get_logger(), "  Call ~/stop  to halt the loop.");
}

void CompositionExample::on_point_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();

    // Count non-zero 3-D points.
    uint32_t non_zero = 0;
    try {
        sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

        for (; ix != ix.end(); ++ix, ++iy, ++iz) {
            if (*ix != 0.0f || *iy != 0.0f || *iz != 0.0f) {
                ++non_zero;
            }
        }
    } catch (const std::runtime_error& e) {
        RCLCPP_WARN_ONCE(get_logger(), "PointCloud2 does not have x/y/z fields (%s); skipping point count.", e.what());
    }

    // Log FPS and timing.
    if (!timing_initialized_) {
        loop_start_time_ = now;
        last_frame_time_ = now;
        timing_initialized_ = true;
        frame_count_ = 1;

        RCLCPP_INFO(get_logger(), "[Frame %4lu]  Non-zero points: %6u  |  FPS: --- (first frame)", frame_count_, non_zero);
    } else {
        const double dt = std::chrono::duration<double>(now - last_frame_time_).count();
        const double instant_fps = (dt > 0.0) ? (1.0 / dt) : 0.0;

        const double total_s = std::chrono::duration<double>(now - loop_start_time_).count();
        ++frame_count_;
        const double avg_fps = (total_s > 0.0) ? (static_cast<double>(frame_count_) / total_s) : 0.0;

        RCLCPP_INFO(get_logger(), "[Frame %4lu]  Non-zero points: %6u  |  FPS: %6.2f  (avg %6.2f)  |  Elapsed: %.3f s", frame_count_, non_zero, instant_fps, avg_fps, total_s);

        last_frame_time_ = now;
    }

    if (running_) {
        trigger_next_frame();
    }
}

void CompositionExample::on_color_camera_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    RCLCPP_INFO(get_logger(), "[color_camera_image]  %ux%u  encoding=%s  stamp=%d.%09u", msg->width, msg->height, msg->encoding.c_str(), msg->header.stamp.sec,
            msg->header.stamp.nanosec);
}

void CompositionExample::trigger_next_frame() {
    if (!trigger_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "Trigger service '%s' is not ready – stopping the loop.", trigger_service_name_.c_str());
        running_ = false;
        return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    trigger_client_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
        const auto resp = fut.get();
        if (!resp->success) {
            RCLCPP_WARN(get_logger(), "trigger_frame call failed: %s – stopping the loop.", resp->message.c_str());
            running_ = false;
        }
    });
}

void CompositionExample::on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (running_) {
        res->success = false;
        res->message = "Acquisition loop is already running.";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
    }

    if (!trigger_client_->service_is_ready()) {
        res->success = false;
        res->message = "Trigger service '" + trigger_service_name_ + "' is not available.";
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(), "Starting maximum-rate acquisition loop.");
    timing_initialized_ = false;
    frame_count_ = 0;
    running_ = true;

    trigger_next_frame();

    res->success = true;
    res->message = "Acquisition loop started.";
}

void CompositionExample::on_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!running_) {
        res->success = false;
        res->message = "Acquisition loop is not running.";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(), "Stopping acquisition loop after %lu frames.", frame_count_);
    running_ = false;

    res->success = true;
    res->message = "Acquisition loop stopped.";
}

}  // namespace phoxi_camera::composition_example

RCLCPP_COMPONENTS_REGISTER_NODE(phoxi_camera::composition_example::CompositionExample)
