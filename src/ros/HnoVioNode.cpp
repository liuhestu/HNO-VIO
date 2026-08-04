#include "hno_vio/ros/HnoVioNode.h"

#include <cam/CamRadtan.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <utils/opencv_yaml_parse.h>

#include <algorithm>
#include <functional>
#include <stdexcept>

namespace hno_vio::ros {
namespace {

template <typename T>
T declareOrGet(const rclcpp::Node::SharedPtr& node,
               const std::string& name,
               const T& default_value) {
    if (!node->has_parameter(name)) {
        return node->declare_parameter<T>(name, default_value);
    }
    return node->get_parameter(name).get_value<T>();
}

double stampToSeconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

} // namespace

HnoVioNode::HnoVioNode(const rclcpp::Node::SharedPtr& node,
                       const std::string& config_path)
    : node_(node), config_path_(config_path) {
    loadParameters(config_path);
    pipeline_ = std::make_unique<pipeline::VioPipeline>(
        cameras_, camera_to_body_, pipeline_options_);
    gt_mapping_ = std::make_unique<GTMapping>(node_, path_gt_, odom_frame_);
    ros_publisher_ = std::make_unique<RosPublisher>(node_, odom_frame_, base_frame_);
    odom_export_ = std::make_unique<pipeline::OdomExport>();
    DiagnosticsOptions diagnostics_options;
    diagnostics_options.frontend_print = frontend_print_;
    diagnostics_options.essential_print = essential_print_;
    diagnostics_options.updater_print = updater_print_;
    diagnostics_options.zupt_print = zupt_print_;
    diagnostics_options.pipeline_print = pipeline_print_;
    diagnostics_ = std::make_unique<Diagnostics>(diagnostics_options);

    if (export_odom_) {
        pipeline::RunContext context;
        context.dataset = dataset_;
        context.raw_bag = raw_bag_;
        context.config = config_name_;
        context.config_path = config_path_;
        context.camera_config = camera_config_;
        context.ground_truth = path_gt_;
        context.odom_frame = odom_frame_;
        context.base_frame = base_frame_;
        context.num_cams = num_cams_;
        context.use_gt_mapping = use_gt_mapping_;
        context.update_enforce_structure =
            pipeline_options_.updater.enforce_structure_after_update;
        context.experiment_fix_e_hat = experiment_fix_e_hat_;
        context.experiment_force_sigma_r_zero =
            experiment_force_sigma_r_zero_;
        context.experiment_max_frames = experiment_max_frames_;
        if (!odom_export_->open(odom_output_path_, context)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open odom output: %s",
                         odom_output_path_.c_str());
            export_odom_ = false;
        }
    }
}

void HnoVioNode::loadParameters(const std::string& config_path) {
    ov_core::YamlParser parser(config_path);
    int max_cameras = 2;
    parser.parse_config("max_cameras", max_cameras);
    num_cams_ = declareOrGet<int>(node_, "num_cams", max_cameras);
    num_cams_ = std::max(1, std::min(num_cams_, max_cameras));

    for (int i = 0; i < num_cams_; ++i) {
        const std::string camera_name = "cam" + std::to_string(i);
        std::vector<double> intrinsics;
        std::vector<double> distortion;
        std::vector<int> resolution;
        parser.parse_external("relative_config_imucam", camera_name, "intrinsics", intrinsics);
        parser.parse_external("relative_config_imucam", camera_name, "distortion_coeffs", distortion);
        parser.parse_external("relative_config_imucam", camera_name, "resolution", resolution);
        auto camera = std::make_shared<ov_core::CamRadtan>(resolution[0], resolution[1]);
        Eigen::Matrix<double, 8, 1> calibration;
        calibration << intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
                       distortion[0], distortion[1], distortion[2], distortion[3];
        camera->set_value(calibration);
        cameras_.push_back(camera);
        Eigen::Matrix4d extrinsic;
        parser.parse_external("relative_config_imucam", camera_name, "T_imu_cam", extrinsic);
        camera_to_body_.push_back(extrinsic);
    }

    pipeline_options_.noise.noise_acc = 0.01;
    pipeline_options_.noise.noise_gyro = 0.001;
    parser.parse_config("accelerometer_noise_density", pipeline_options_.noise.noise_acc);
    parser.parse_config("gyroscope_noise_density", pipeline_options_.noise.noise_gyro);

    int initializer_window = 250;
    parser.parse_config("init_window_size", initializer_window, false);
    parser.parse_config("init_imu_thresh", pipeline_options_.initializer_acc_variance, false);
    parser.parse_config("init_gyro_thresh", pipeline_options_.initializer_gyro_variance, false);
    initializer_window = declareOrGet<int>(node_, "init_window_size", initializer_window);
    pipeline_options_.initializer_window_size =
        static_cast<size_t>(std::max(10, initializer_window));
    pipeline_options_.initializer_acc_variance = declareOrGet<double>(
        node_, "init_imu_thresh", pipeline_options_.initializer_acc_variance);
    pipeline_options_.initializer_gyro_variance = declareOrGet<double>(
        node_, "init_gyro_thresh", pipeline_options_.initializer_gyro_variance);

    auto& frontend = pipeline_options_.frontend;
    parser.parse_config("feature_tracker_num_pts", frontend.tracker_num_pts, false);
    parser.parse_config("feature_tracker_fast_threshold", frontend.tracker_fast_threshold, false);
    parser.parse_config("feature_tracker_grid_x", frontend.tracker_grid_x, false);
    parser.parse_config("feature_tracker_grid_y", frontend.tracker_grid_y, false);
    parser.parse_config("feature_tracker_min_px_dist", frontend.tracker_min_px_dist, false);
    parser.parse_config("feature_min_stereo_depth", frontend.min_stereo_depth, false);
    parser.parse_config("feature_max_stereo_depth", frontend.max_stereo_depth, false);
    parser.parse_config("feature_stereo_reproj_thresh", frontend.stereo_reproj_thresh, false);
    parser.parse_config("feature_reproj_thresh", frontend.reproj_thresh, false);
    parser.parse_config("feature_reproj_thresh_low", frontend.reproj_thresh_low, false);
    parser.parse_config("feature_low_feature_pts", frontend.low_feature_pts, false);
    parser.parse_config("feature_low_feature_db", frontend.low_feature_db, false);
    parser.parse_config("feature_mature_thresh", frontend.mature_thresh, false);
    parser.parse_config("feature_mature_thresh_low", frontend.mature_thresh_low, false);
    parser.parse_config("feature_map_jump_thresh", frontend.map_jump_thresh, false);
    parser.parse_config("feature_active_mature_thresh", frontend.active_mature_thresh, false);
    parser.parse_config("feature_health_min_stable", frontend.health_min_stable, false);
    parser.parse_config("feature_health_min_db", frontend.health_min_db, false);
    parser.parse_config("feature_health_hold_frames", frontend.health_hold_frames, false);
    parser.parse_config("feature_health_start_frame", frontend.health_start_frame, false);

#define HNO_FRONTEND_PARAM(type, name, field) \
    frontend.field = declareOrGet<type>(node_, name, frontend.field)
    HNO_FRONTEND_PARAM(int, "feature_tracker_num_pts", tracker_num_pts);
    HNO_FRONTEND_PARAM(int, "feature_tracker_fast_threshold", tracker_fast_threshold);
    HNO_FRONTEND_PARAM(int, "feature_tracker_grid_x", tracker_grid_x);
    HNO_FRONTEND_PARAM(int, "feature_tracker_grid_y", tracker_grid_y);
    HNO_FRONTEND_PARAM(int, "feature_tracker_min_px_dist", tracker_min_px_dist);
    HNO_FRONTEND_PARAM(double, "feature_min_stereo_depth", min_stereo_depth);
    HNO_FRONTEND_PARAM(double, "feature_max_stereo_depth", max_stereo_depth);
    HNO_FRONTEND_PARAM(double, "feature_stereo_reproj_thresh", stereo_reproj_thresh);
    HNO_FRONTEND_PARAM(double, "feature_reproj_thresh", reproj_thresh);
    HNO_FRONTEND_PARAM(double, "feature_reproj_thresh_low", reproj_thresh_low);
    HNO_FRONTEND_PARAM(int, "feature_low_feature_pts", low_feature_pts);
    HNO_FRONTEND_PARAM(int, "feature_low_feature_db", low_feature_db);
    HNO_FRONTEND_PARAM(int, "feature_mature_thresh", mature_thresh);
    HNO_FRONTEND_PARAM(int, "feature_mature_thresh_low", mature_thresh_low);
    HNO_FRONTEND_PARAM(double, "feature_map_jump_thresh", map_jump_thresh);
    HNO_FRONTEND_PARAM(int, "feature_active_mature_thresh", active_mature_thresh);
    HNO_FRONTEND_PARAM(int, "feature_health_min_stable", health_min_stable);
    HNO_FRONTEND_PARAM(int, "feature_health_min_db", health_min_db);
    HNO_FRONTEND_PARAM(int, "feature_health_hold_frames", health_hold_frames);
    HNO_FRONTEND_PARAM(int, "feature_health_start_frame", health_start_frame);
#undef HNO_FRONTEND_PARAM

    auto& updater = pipeline_options_.updater;
    parser.parse_config("update_pixel_noise", updater.pixel_noise, false);
    parser.parse_config("update_focal_length", updater.focal_length, false);
    parser.parse_config("update_chi2_gate", updater.chi2_gate, false);
    parser.parse_config("update_max_delta_p", updater.max_delta_p, false);
    parser.parse_config("update_max_delta_r", updater.max_delta_r, false);
    parser.parse_config("update_min_observations", updater.min_observations, false);
    parser.parse_config("update_low_observation_hold_frames", updater.low_observation_hold_frames, false);
    parser.parse_config("update_warn_delta_ratio", updater.warn_delta_ratio, false);
    parser.parse_config("zupt_velocity_noise", updater.zupt_velocity_noise, false);
    updater.pixel_noise = declareOrGet<double>(node_, "update_pixel_noise", updater.pixel_noise);
    updater.focal_length = declareOrGet<double>(node_, "update_focal_length", updater.focal_length);
    updater.chi2_gate = declareOrGet<double>(node_, "update_chi2_gate", updater.chi2_gate);
    updater.max_delta_p = declareOrGet<double>(node_, "update_max_delta_p", updater.max_delta_p);
    updater.max_delta_r = declareOrGet<double>(node_, "update_max_delta_r", updater.max_delta_r);
    updater.min_observations = declareOrGet<int>(node_, "update_min_observations", updater.min_observations);
    updater.low_observation_hold_frames = declareOrGet<int>(node_, "update_low_observation_hold_frames", updater.low_observation_hold_frames);
    updater.warn_delta_ratio = declareOrGet<double>(node_, "update_warn_delta_ratio", updater.warn_delta_ratio);
    updater.enforce_structure_after_update = declareOrGet<bool>(node_, "update_enforce_structure", updater.enforce_structure_after_update);
    updater.zupt_velocity_noise = declareOrGet<double>(node_, "zupt_velocity_noise", updater.zupt_velocity_noise);

    auto& zupt = pipeline_options_.zupt;
    parser.parse_config("zupt_max_disparity", zupt.max_disparity, false);
    parser.parse_config("zupt_imu_window_size", zupt.imu_window_size, false);
    parser.parse_config("zupt_min_tracks", zupt.min_tracks, false);
    parser.parse_config("zupt_hold_frames", zupt.hold_frames, false);
    parser.parse_config("zupt_acc_var_threshold", zupt.acc_variance_threshold, false);
    parser.parse_config("zupt_gyro_var_threshold", zupt.gyro_variance_threshold, false);
    parser.parse_config("zupt_activation_speed", zupt.activation_speed, false);
    zupt.enabled = declareOrGet<bool>(node_, "try_zupt", zupt.enabled);
    zupt.max_disparity = declareOrGet<double>(node_, "zupt_max_disparity", zupt.max_disparity);
    zupt.imu_window_size = declareOrGet<int>(node_, "zupt_imu_window_size", zupt.imu_window_size);
    zupt.min_tracks = declareOrGet<int>(node_, "zupt_min_tracks", zupt.min_tracks);
    zupt.hold_frames = declareOrGet<int>(node_, "zupt_hold_frames", zupt.hold_frames);
    zupt.acc_variance_threshold = declareOrGet<double>(node_, "zupt_acc_var_threshold", zupt.acc_variance_threshold);
    zupt.gyro_variance_threshold = declareOrGet<double>(node_, "zupt_gyro_var_threshold", zupt.gyro_variance_threshold);
    zupt.activation_speed = declareOrGet<double>(node_, "zupt_activation_speed", zupt.activation_speed);

    path_gt_ = declareOrGet<std::string>(node_, "path_gt", "");
    experiment_fix_e_hat_ =
        declareOrGet<bool>(node_, "experiment_fix_e_hat", false);
    experiment_force_sigma_r_zero_ =
        declareOrGet<bool>(node_, "experiment_force_sigma_r_zero", false);
    experiment_max_frames_ =
        std::max(0, declareOrGet<int>(node_, "experiment_max_frames", 0));
    if (experiment_fix_e_hat_ && experiment_force_sigma_r_zero_) {
        throw std::invalid_argument(
            "experiment_fix_e_hat and experiment_force_sigma_r_zero "
            "must not be enabled together");
    }
    pipeline_options_.experiment_fix_e_hat = experiment_fix_e_hat_;
    pipeline_options_.experiment_force_sigma_r_zero =
        experiment_force_sigma_r_zero_;
    use_gt_mapping_ = declareOrGet<bool>(node_, "use_gt_mapping", false);
    export_odom_ = declareOrGet<bool>(node_, "export_odom", false);
    frontend_print_ = declareOrGet<bool>(node_, "frontend_print", false);
    essential_print_ = declareOrGet<bool>(node_, "essential_print", true);
    updater_print_ = declareOrGet<bool>(node_, "updater_print", false);
    zupt_print_ = declareOrGet<bool>(node_, "ZUPT_print", false);
    pipeline_print_ = declareOrGet<bool>(node_, "pipeline_print", false);
    odom_output_path_ = declareOrGet<std::string>(node_, "odom_output_path", "");
    dataset_ = declareOrGet<std::string>(node_, "dataset", "");
    bag_path_ = declareOrGet<std::string>(node_, "bag_path", "");
    raw_bag_ = declareOrGet<std::string>(node_, "raw_bag", bag_path_);
    config_name_ = declareOrGet<std::string>(node_, "config", "");
    camera_config_ = declareOrGet<std::string>(node_, "camera_config", "");
    odom_frame_ = declareOrGet<std::string>(node_, "odom_frame", "odom");
    base_frame_ = declareOrGet<std::string>(node_, "base_frame", "base_link");
    topic_imu_ = declareOrGet<std::string>(node_, "topic_imu", "/imu0");
    topic_cam0_ = declareOrGet<std::string>(node_, "topic_cam0", "/cam0/image_raw");
    topic_cam1_ = declareOrGet<std::string>(node_, "topic_cam1", "/cam1/image_raw");
}

void HnoVioNode::launchSubscribers() {
    const auto qos = rclcpp::SensorDataQoS();
    imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        topic_imu_, qos, std::bind(&HnoVioNode::imuCallback, this, std::placeholders::_1));
    if (num_cams_ == 2) {
        left_subscription_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
        right_subscription_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
        left_subscription_->subscribe(node_.get(), topic_cam0_, qos.get_rmw_qos_profile());
        right_subscription_->subscribe(node_.get(), topic_cam1_, qos.get_rmw_qos_profile());
        synchronizer_ = std::make_unique<Synchronizer>(SyncPolicy(10), *left_subscription_, *right_subscription_);
        synchronizer_->registerCallback(std::bind(&HnoVioNode::stereoCallback, this,
                                                  std::placeholders::_1, std::placeholders::_2));
    } else {
        mono_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            topic_cam0_, qos, std::bind(&HnoVioNode::monoCallback, this, std::placeholders::_1));
    }
}

void HnoVioNode::publishLatestPrediction(const pipeline::PipelineResult& result) {
    if (result.latest_prediction && result.latest_prediction_timestamp >= 0.0) {
        ros_publisher_->publishPrediction(result.latest_prediction_timestamp,
                                          *result.latest_prediction);
        return;
    }
    ros_publisher_->publishPrediction(result.timestamp, result.state);
}

std::optional<pipeline::PipelineResult> HnoVioNode::tryProcessReadyCameras() {
    std::optional<pipeline::PipelineResult> latest_result;
    if (experiment_complete_) return latest_result;
    while (!pending_cameras_.empty() && pipeline_->initialized()) {
        auto next = pending_cameras_.begin();
        if (next->first <= pipeline_->committedTime() + 1e-9) {
            // Frames preceding successful inertial initialization cannot be
            // applied to an initialized observer state.
            pending_cameras_.erase(next);
            continue;
        }
        if (!pipeline_->canProcessStereo(next->first)) break;

        const ov_core::CameraData camera = next->second;
        std::optional<Pose> gt_pose;
        if (use_gt_mapping_) gt_pose = gt_mapping_->getPose(camera.timestamp);
        const auto result = pipeline_->processStereo(camera, gt_pose);
        if (!result) break;
        pending_cameras_.erase(next);

        ros_publisher_->publishCommittedPath(result->timestamp, result->state);
        ros_publisher_->publishFrontend(result->timestamp, camera,
                                        result->active_landmarks, result->tracker);
        const Pose estimated_pose = Pose::FromState(result->state);
        const std::optional<Pose> aligned_ground_truth =
            gt_mapping_->publish(result->timestamp, estimated_pose);
        if (export_odom_) {
            odom_export_->write(result->timestamp, result->state,
                                result->diagnostics);
        }
        diagnostics_->report(result->diagnostics, estimated_pose, aligned_ground_truth);
        latest_result = result;
        if (experiment_max_frames_ > 0 &&
            result->diagnostics.frame_index >= experiment_max_frames_) {
            experiment_complete_ = true;
            pending_cameras_.clear();
            RCLCPP_INFO(node_->get_logger(),
                        "Reached experiment_max_frames=%d; shutting down.",
                        experiment_max_frames_);
            rclcpp::shutdown();
            break;
        }
    }
    return latest_result;
}

void HnoVioNode::pushImu(const ov_core::ImuData& imu) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto prediction = pipeline_->processImu(imu);
    const auto corrected = tryProcessReadyCameras();
    if (corrected) {
        publishLatestPrediction(*corrected);
    } else if (prediction) {
        publishLatestPrediction(*prediction);
    }
}

void HnoVioNode::pushCamera(const ov_core::CameraData& camera) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pending_cameras_[camera.timestamp] = camera;
    const auto corrected = tryProcessReadyCameras();
    if (corrected) publishLatestPrediction(*corrected);
}

void HnoVioNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr message) {
    ov_core::ImuData imu;
    imu.timestamp = stampToSeconds(message->header.stamp);
    imu.wm << message->angular_velocity.x, message->angular_velocity.y,
             message->angular_velocity.z;
    imu.am << message->linear_acceleration.x, message->linear_acceleration.y,
             message->linear_acceleration.z;
    pushImu(imu);
}

void HnoVioNode::monoCallback(const sensor_msgs::msg::Image::SharedPtr message) {
    ov_core::CameraData camera;
    camera.timestamp = stampToSeconds(message->header.stamp);
    camera.sensor_ids = {0};
    try {
        camera.images.push_back(cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::MONO8)->image);
        camera.masks.push_back(cv::Mat::zeros(camera.images[0].rows, camera.images[0].cols, CV_8UC1));
    } catch (const std::exception& exception) {
        RCLCPP_WARN(node_->get_logger(), "Failed to convert image: %s", exception.what());
        return;
    }
    pushCamera(camera);
}

void HnoVioNode::stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr left,
    const sensor_msgs::msg::Image::ConstSharedPtr right) {
    ov_core::CameraData camera;
    camera.timestamp = stampToSeconds(left->header.stamp);
    camera.sensor_ids = {0, 1};
    try {
        camera.images.push_back(cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::MONO8)->image);
        camera.images.push_back(cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::MONO8)->image);
        camera.masks.push_back(cv::Mat::zeros(camera.images[0].rows, camera.images[0].cols, CV_8UC1));
        camera.masks.push_back(cv::Mat::zeros(camera.images[1].rows, camera.images[1].cols, CV_8UC1));
    } catch (const std::exception& exception) {
        RCLCPP_WARN(node_->get_logger(), "Failed to convert stereo images: %s", exception.what());
        return;
    }
    pushCamera(camera);
}

} // namespace hno_vio::ros
