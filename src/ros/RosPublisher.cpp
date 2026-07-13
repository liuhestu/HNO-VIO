#include "hno_vio/ros/RosPublisher.h"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <utility>

namespace hno_vio::ros {

RosPublisher::RosPublisher(const rclcpp::Node::SharedPtr& node,
                           std::string odom_frame,
                           std::string base_frame)
    : node_(node),
      odom_frame_(std::move(odom_frame)),
      base_frame_(std::move(base_frame)) {
    pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/hno_vio/pose", 100);
    odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/hno_vio/odom", 100);
    path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/hno_vio/path", 100);
    feature_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("/hno_vio/features_3d", 100);
    image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("/hno_vio/image_track", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    path_.header.frame_id = odom_frame_;
}

rclcpp::Time RosPublisher::toRosTime(double timestamp) {
    return rclcpp::Time(static_cast<int64_t>(std::llround(timestamp * 1e9)), RCL_ROS_TIME);
}

void RosPublisher::publishPrediction(double timestamp, const State& state) {
    const rclcpp::Time stamp = toRosTime(timestamp);
    Eigen::Quaterniond q(state.R_hat_B2I);
    q.normalize();

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = odom_frame_;
    pose.pose.pose.position.x = state.p_hat.x();
    pose.pose.pose.position.y = state.p_hat.y();
    pose.pose.pose.position.z = state.p_hat.z();
    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();
    pose.pose.pose.orientation.w = q.w();
    pose_publisher_->publish(pose);

    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = pose.pose.pose;
    odom.twist.twist.linear.x = state.v_hat.x();
    odom.twist.twist.linear.y = state.v_hat.y();
    odom.twist.twist.linear.z = state.v_hat.z();
    odom_publisher_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = state.p_hat.x();
    transform.transform.translation.y = state.p_hat.y();
    transform.transform.translation.z = state.p_hat.z();
    transform.transform.rotation = pose.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
}

void RosPublisher::publishCommittedPath(double timestamp, const State& state) {
    const rclcpp::Time stamp = toRosTime(timestamp);
    Eigen::Quaterniond q(state.R_hat_B2I);
    q.normalize();

    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header.stamp = stamp;
    path_pose.header.frame_id = odom_frame_;
    path_pose.pose.position.x = state.p_hat.x();
    path_pose.pose.position.y = state.p_hat.y();
    path_pose.pose.position.z = state.p_hat.z();
    path_pose.pose.orientation.x = q.x();
    path_pose.pose.orientation.y = q.y();
    path_pose.pose.orientation.z = q.z();
    path_pose.pose.orientation.w = q.w();

    committed_path_poses_[stamp.nanoseconds()] = path_pose;
    path_.header = path_pose.header;
    path_.poses.clear();
    path_.poses.reserve(committed_path_poses_.size());
    for (const auto& [timestamp_ns, committed_pose] : committed_path_poses_) {
        (void)timestamp_ns;
        path_.poses.push_back(committed_pose);
    }
    path_publisher_->publish(path_);
}

void RosPublisher::publishFrontend(
    double timestamp,
    const ov_core::CameraData& camera,
    const std::map<size_t, Eigen::Vector3d>& active_landmarks,
    const std::shared_ptr<ov_core::TrackKLT>& tracker) {
    const rclcpp::Time stamp = toRosTime(timestamp);
    sensor_msgs::msg::PointCloud cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = odom_frame_;
    for (const auto& [id, landmark] : active_landmarks) {
        (void)id;
        geometry_msgs::msg::Point32 point;
        point.x = landmark.x();
        point.y = landmark.y();
        point.z = landmark.z();
        cloud.points.push_back(point);
    }
    feature_publisher_->publish(cloud);

    if (!tracker || image_publisher_->get_subscription_count() == 0) return;
    std::unordered_set<size_t> allowed;
    for (const auto& [id, landmark] : active_landmarks) {
        (void)landmark;
        allowed.insert(id);
    }
    cv::Mat visualization;
    tracker->display_history(visualization, 0, 255, 0, 255, 0, 0, {}, "HNO Tracker", &allowed);
    if (visualization.empty()) return;

    const auto observations = tracker->get_last_obs();
    const auto ids = tracker->get_last_ids();
    if (!camera.images.empty()) {
        const int width = camera.images[0].cols;
        const int height = camera.images[0].rows;
        if (visualization.cols >= width && visualization.rows >= height &&
            observations.count(0) && ids.count(0)) {
            cv::Mat left = visualization(cv::Rect(0, 0, width, height));
            for (size_t i = 0; i < ids.at(0).size(); ++i) {
                const size_t id = ids.at(0)[i];
                const cv::Scalar color = active_landmarks.count(id)
                    ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 255);
                cv::circle(left, observations.at(0)[i].pt, 6, color,
                           active_landmarks.count(id) ? 2 : 1);
            }
        }
        if (camera.images.size() > 1 && visualization.cols >= 2 * width &&
            observations.count(1) && ids.count(1)) {
            cv::Mat right = visualization(cv::Rect(width, 0, width, height));
            for (size_t i = 0; i < ids.at(1).size(); ++i) {
                const size_t id = ids.at(1)[i];
                const cv::Scalar color = active_landmarks.count(id)
                    ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 255);
                cv::circle(right, observations.at(1)[i].pt, 6, color,
                           active_landmarks.count(id) ? 2 : 1);
            }
        }
    }
    auto image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", visualization).toImageMsg();
    image->header.stamp = stamp;
    image->header.frame_id = base_frame_;
    image_publisher_->publish(*image);
}

} // namespace hno_vio::ros
