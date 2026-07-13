#include "hno_vio/ros/GTMapping.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <utility>
#include <vector>

namespace hno_vio::ros {

GTMapping::GTMapping(const rclcpp::Node::SharedPtr& node,
                     const std::string& trajectory_path,
                     std::string odom_frame,
                     std::string gt_base_frame)
    : node_(node),
      odom_frame_(std::move(odom_frame)),
      gt_base_frame_(std::move(gt_base_frame)) {
    path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/hno_vio/path_gt", 100);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    path_.header.frame_id = odom_frame_;
    load(trajectory_path);
}

rclcpp::Time GTMapping::toRosTime(double timestamp) {
    return rclcpp::Time(static_cast<int64_t>(std::llround(timestamp * 1e9)), RCL_ROS_TIME);
}

void GTMapping::load(const std::string& trajectory_path) {
    if (trajectory_path.empty()) return;
    std::ifstream stream(trajectory_path);
    if (!stream.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open GT file: %s", trajectory_path.c_str());
        return;
    }
    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty() || line.front() == '#') continue;
        std::stringstream parser(line);
        std::vector<double> values;
        double value = 0.0;
        while (parser >> value) values.push_back(value);
        if (values.size() < 8) continue;
        double timestamp = values[0];
        if (timestamp > 1e10) timestamp *= 1e-9;
        Eigen::Quaterniond q(values[7], values[4], values[5], values[6]);
        q.normalize();
        poses_[timestamp] = Pose{q.toRotationMatrix(),
                                 Eigen::Vector3d(values[1], values[2], values[3])};
    }
    RCLCPP_INFO(node_->get_logger(), "Loaded %zu GT poses", poses_.size());
}

std::optional<Pose> GTMapping::getPose(double timestamp) const {
    if (poses_.empty()) return std::nullopt;
    auto next = poses_.lower_bound(timestamp);
    if (next == poses_.begin() || next == poses_.end()) return std::nullopt;
    const auto previous = std::prev(next);
    const double alpha = (timestamp - previous->first) / (next->first - previous->first);
    const Eigen::Vector3d p = (1.0 - alpha) * previous->second.p + alpha * next->second.p;
    const Eigen::Quaterniond q_previous(previous->second.R);
    const Eigen::Quaterniond q_next(next->second.R);
    return Pose{q_previous.slerp(alpha, q_next).toRotationMatrix(), p};
}

void GTMapping::publish(double timestamp, const Pose& estimated_pose) {
    const auto raw = getPose(timestamp);
    if (!raw) return;
    if (!has_visual_alignment_) {
        R_visual_gt_ = estimated_pose.R * raw->R.transpose();
        t_visual_gt_ = estimated_pose.p - R_visual_gt_ * raw->p;
        has_visual_alignment_ = true;
    }

    const Pose aligned{R_visual_gt_ * raw->R,
                       R_visual_gt_ * raw->p + t_visual_gt_};
    const Eigen::Quaterniond q(aligned.R);
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.stamp = toRosTime(timestamp);
    stamped.header.frame_id = odom_frame_;
    stamped.pose.position.x = aligned.p.x();
    stamped.pose.position.y = aligned.p.y();
    stamped.pose.position.z = aligned.p.z();
    stamped.pose.orientation.x = q.x();
    stamped.pose.orientation.y = q.y();
    stamped.pose.orientation.z = q.z();
    stamped.pose.orientation.w = q.w();
    const int64_t timestamp_ns =
        static_cast<int64_t>(stamped.header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(stamped.header.stamp.nanosec);
    path_poses_[timestamp_ns] = stamped;
    path_.header = stamped.header;
    path_.poses.clear();
    path_.poses.reserve(path_poses_.size());
    for (const auto& [timestamp_ns, pose] : path_poses_) {
        (void)timestamp_ns;
        path_.poses.push_back(pose);
    }
    path_publisher_->publish(path_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = stamped.header;
    transform.child_frame_id = gt_base_frame_;
    transform.transform.translation.x = aligned.p.x();
    transform.transform.translation.y = aligned.p.y();
    transform.transform.translation.z = aligned.p.z();
    transform.transform.rotation = stamped.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
}

} // namespace hno_vio::ros
