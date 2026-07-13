#ifndef HNO_VIO_ROS_ROS_PUBLISHER_H
#define HNO_VIO_ROS_ROS_PUBLISHER_H

#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "track/TrackKLT.h"
#include "utils/sensor_data.h"

#include "hno_vio/State.h"

namespace hno_vio::ros {

class RosPublisher {
public:
    RosPublisher(const rclcpp::Node::SharedPtr& node,
                 std::string odom_frame,
                 std::string base_frame);

    void publishPrediction(double timestamp, const State& state);
    void publishCommittedPath(double timestamp, const State& state);
    void publishFrontend(double timestamp,
                         const ov_core::CameraData& camera,
                         const std::map<size_t, Eigen::Vector3d>& active_landmarks,
                         const std::shared_ptr<ov_core::TrackKLT>& tracker);

private:
    static rclcpp::Time toRosTime(double timestamp);

    rclcpp::Node::SharedPtr node_;
    std::string odom_frame_;
    std::string base_frame_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr feature_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Path path_;
    std::map<int64_t, geometry_msgs::msg::PoseStamped> committed_path_poses_;
};

} // namespace hno_vio::ros

#endif
