#ifndef HNO_MANAGER_H
#define HNO_MANAGER_H

#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include "utils/sensor_data.h"

#include "hno_vio/HNOFeature.h"
#include "hno_vio/HNOInitializer.h"
#include "hno_vio/HNOPropagator.h"
#include "hno_vio/HNOState.h"
#include "hno_vio/HNOUpdater.h"

namespace hno_vio {

class HNOManager {
public:
    HNOManager(const rclcpp::Node::SharedPtr& node, const std::string& config_path);
    ~HNOManager();

    void launch_subscribers();
    void feed_measurement(const ov_core::ImuData& msg);
    void feed_measurement(const ov_core::CameraData& msg);

private:
    void load_parameters(const std::string& config_path);
    void load_gt_data();
    void process_camera_data(const ov_core::CameraData& msg);
    bool get_interpolated_gt(double timestamp, Eigen::Vector3d& p_gt, Eigen::Matrix3d& R_gt);

    void publish_state(double timestamp, const std::shared_ptr<HNOState>& state);
    void export_odom_state(double timestamp, const std::shared_ptr<HNOState>& state_to_export);
    std::string make_run_id_beijing_time() const;
    std::string infer_dataset_from_bag_path() const;
    std::string json_escape(const std::string& value) const;
    void write_run_context(const std::string& run_dir) const;
    void publish_visualization(double timestamp, const ov_core::CameraData& msg);
    void compute_and_print_error(double timestamp, const Eigen::Vector3d& p_est, int num_feats, int num_obs);

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void mono_callback(const sensor_msgs::msg::Image::SharedPtr msg0);
    void stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg0,
                         const sensor_msgs::msg::Image::ConstSharedPtr msg1);

    rclcpp::Time time_from_sec(double timestamp) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_gt;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feat;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam0_mono;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_cam0;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_cam1;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::unique_ptr<Sync> sync;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    nav_msgs::msg::Path path_msg;
    nav_msgs::msg::Path path_gt_msg;

    std::shared_ptr<HNOState> state;
    std::shared_ptr<HNOPropagator> propagator;
    std::shared_ptr<HNOUpdater> updater;
    std::shared_ptr<HNOFeature> feature_handler;
    std::shared_ptr<HNOInitializer> initializer;

    std::mutex data_mutex;
    std::vector<ov_core::ImuData> imu_data_buffer;

    bool is_initialized = false;
    double current_time = -1;
    double first_timestamp = -1;
    double last_published_time = -1;
    double last_exported_odom_time = -1;

    std::vector<std::shared_ptr<ov_core::CamBase>> cams;
    std::vector<Eigen::Matrix4d> cams_T_C2B;
    HNOFeature::Options feature_options;
    HNOUpdater::Options updater_options;

    std::string path_gt;
    std::string dataset;
    std::string bag_path;
    std::string raw_bag;
    std::string config_name;
    std::string config_path_;
    std::string camera_config;
    std::map<double, Eigen::VectorXd> gt_states;
    bool has_align = false;
    Eigen::Vector3d t_align = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();
    bool has_gt_viz_align = false;
    Eigen::Vector3d t_gt_viz_align = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_gt_viz_align = Eigen::Matrix3d::Identity();

    bool use_gt_mapping = false;

    bool export_odom = false;
    std::string odom_output_path;
    std::string odom_txt_output_path;
    std::ofstream odom_output_file;
    std::ofstream odom_txt_output_file;

    std::string odom_frame = "odom";
    std::string base_frame = "base_link";
    std::string world_frame = "odom";
    std::string topic_imu = "/imu0";
    std::string topic_cam0 = "/cam0/image_raw";
    std::string topic_cam1 = "/cam1/image_raw";
    int num_cams = 2;
};

} // namespace hno_vio

#endif // HNO_MANAGER_H
