#ifndef HNO_VIO_ROS_GT_MAPPING_H
#define HNO_VIO_ROS_GT_MAPPING_H

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "hno_vio/State.h"

namespace hno_vio::ros {

class GTMapping {
public:
    GTMapping(const rclcpp::Node::SharedPtr& node,
              const std::string& trajectory_path,
              std::string odom_frame,
              std::string gt_base_frame = "gt_base_link");

    std::optional<Pose> getPose(double timestamp) const;
    std::optional<Pose> publish(double timestamp, const Pose& estimated_pose);
    bool loaded() const { return !poses_.empty(); }

private:
    static rclcpp::Time toRosTime(double timestamp);
    void load(const std::string& trajectory_path);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Path path_;
    std::map<int64_t, geometry_msgs::msg::PoseStamped> path_poses_;
    std::map<double, Pose> poses_;
    std::string odom_frame_;
    std::string gt_base_frame_;
    bool has_visual_alignment_ = false;
    Eigen::Matrix3d R_visual_gt_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_visual_gt_ = Eigen::Vector3d::Zero();
};

} // namespace hno_vio::ros

#endif
