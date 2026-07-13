#include "hno_vio/ros/HnoVioNode.h"

#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("run_hno_vio");

    std::string config_path = node->declare_parameter<std::string>("config_path", "");
    if (config_path.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Parameter config_path is required.");
        rclcpp::shutdown();
        return 2;
    }

    try {
        auto application = std::make_shared<hno_vio::ros::HnoVioNode>(node, config_path);
        application->launchSubscribers();

        RCLCPP_INFO(node->get_logger(), "HNO Node Started (ROS2).");
        rclcpp::spin(node);
        application.reset();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(node->get_logger(), "Unhandled HNO-VIO error: %s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
}
