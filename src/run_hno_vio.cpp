#include "hno_vio/HNOManager.h"

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

    auto manager = std::make_shared<hno_vio::HNOManager>(node, config_path);
    manager->launch_subscribers();

    RCLCPP_INFO(node->get_logger(), "HNO Node Started (ROS2).");
    rclcpp::spin(node);
    manager.reset();
    rclcpp::shutdown();
    return 0;
}
