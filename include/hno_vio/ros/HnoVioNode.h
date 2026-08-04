#ifndef HNO_VIO_ROS_HNO_VIO_NODE_H
#define HNO_VIO_ROS_HNO_VIO_NODE_H

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "cam/CamBase.h"

#include "hno_vio/Diagnostics.h"
#include "hno_vio/pipeline/OdomExport.h"
#include "hno_vio/pipeline/VioPipeline.h"
#include "hno_vio/ros/GTMapping.h"
#include "hno_vio/ros/RosPublisher.h"

namespace hno_vio::ros {

class HnoVioNode {
public:
    HnoVioNode(const rclcpp::Node::SharedPtr& node, const std::string& config_path);
    void launchSubscribers();

private:
    void loadParameters(const std::string& config_path);
    void pushImu(const ov_core::ImuData& imu);
    void pushCamera(const ov_core::CameraData& camera);
    std::optional<pipeline::PipelineResult> tryProcessReadyCameras();
    void publishLatestPrediction(const pipeline::PipelineResult& result);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message);
    void monoCallback(const sensor_msgs::msg::Image::SharedPtr message);
    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr left,
                        const sensor_msgs::msg::Image::ConstSharedPtr right);

    rclcpp::Node::SharedPtr node_;
    std::mutex data_mutex_;

    std::unique_ptr<pipeline::VioPipeline> pipeline_;
    std::unique_ptr<GTMapping> gt_mapping_;
    std::unique_ptr<pipeline::OdomExport> odom_export_;
    std::unique_ptr<Diagnostics> diagnostics_;
    std::unique_ptr<RosPublisher> ros_publisher_;
    std::map<double, ov_core::CameraData> pending_cameras_;

    std::vector<std::shared_ptr<ov_core::CamBase>> cameras_;
    std::vector<Eigen::Matrix4d> camera_to_body_;
    pipeline::VioPipelineOptions pipeline_options_;

    std::string config_path_;
    std::string path_gt_;
    std::string dataset_;
    std::string bag_path_;
    std::string raw_bag_;
    std::string config_name_;
    std::string camera_config_;
    std::string odom_output_path_;
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
    std::string topic_imu_ = "/imu0";
    std::string topic_cam0_ = "/cam0/image_raw";
    std::string topic_cam1_ = "/cam1/image_raw";
    int num_cams_ = 2;
    int experiment_max_frames_ = 0;
    bool use_gt_mapping_ = false;
    bool export_odom_ = false;
    bool experiment_fix_e_hat_ = false;
    bool experiment_force_sigma_r_zero_ = false;
    bool experiment_complete_ = false;
    bool frontend_print_ = false;
    bool essential_print_ = true;
    bool updater_print_ = false;
    bool zupt_print_ = false;
    bool pipeline_print_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_subscription_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_subscription_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_subscription_;
    using SyncPolicy = message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    std::unique_ptr<Synchronizer> synchronizer_;
};

} // namespace hno_vio::ros

#endif
