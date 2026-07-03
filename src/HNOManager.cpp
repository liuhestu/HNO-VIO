#include "hno_vio/HNOManager.h"

#include <boost/filesystem.hpp>
#include <cam/CamRadtan.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <utils/opencv_yaml_parse.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unordered_set>

using namespace hno_vio;

namespace {

template <typename T>
T declare_or_get(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value) {
    if (!node->has_parameter(name)) {
        return node->declare_parameter<T>(name, default_value);
    }
    return node->get_parameter(name).get_value<T>();
}

double stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

} // namespace

HNOManager::HNOManager(const rclcpp::Node::SharedPtr& node, const std::string& config_path)
    : node_(node), config_path_(config_path) {
    state = std::make_shared<HNOState>();
    propagator = std::make_shared<HNOPropagator>();
    initializer = std::make_shared<HNOInitializer>();
    updater = std::make_shared<HNOUpdater>();

    load_parameters(config_path);

    feature_handler = std::make_shared<HNOFeature>(cams, cams_T_C2B, feature_options);

    std::map<size_t, Eigen::Matrix4d> extrinsics_map;
    for (size_t i = 0; i < cams_T_C2B.size(); i++) {
        extrinsics_map[i] = cams_T_C2B[i];
    }
    updater->setExtrinsics(extrinsics_map);
    updater->setOptions(updater_options);

    pub_pose = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/hno_vio/pose", 100);
    pub_odom = node_->create_publisher<nav_msgs::msg::Odometry>("/hno_vio/odom", 100);
    pub_path = node_->create_publisher<nav_msgs::msg::Path>("/hno_vio/path", 100);
    pub_path_gt = node_->create_publisher<nav_msgs::msg::Path>("/hno_vio/path_gt", 100);
    pub_feat = node_->create_publisher<sensor_msgs::msg::PointCloud>("/hno_vio/features_3d", 100);
    pub_img = node_->create_publisher<sensor_msgs::msg::Image>("/hno_vio/image_track", 10);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    path_msg.header.frame_id = odom_frame;
    path_gt_msg.header.frame_id = odom_frame;

    RCLCPP_INFO(node_->get_logger(), "[HNOManager] Initialized successfully.");
}

HNOManager::~HNOManager() {
    if (odom_output_file.is_open()) {
        odom_output_file.flush();
        odom_output_file.close();
    }
    if (odom_txt_output_file.is_open()) {
        odom_txt_output_file.flush();
        odom_txt_output_file.close();
    }
}

rclcpp::Time HNOManager::time_from_sec(double timestamp) const {
    const int64_t ns = static_cast<int64_t>(std::llround(timestamp * 1e9));
    return rclcpp::Time(ns, RCL_ROS_TIME);
}

std::string HNOManager::make_run_id_beijing_time() const {
    const std::time_t now = std::time(nullptr);
    const std::time_t beijing_now = now + 8 * 60 * 60;
    std::tm tm_now;
    gmtime_r(&beijing_now, &tm_now);

    char run_id[32];
    std::strftime(run_id, sizeof(run_id), "run_%Y%m%dT%H%M%S", &tm_now);
    return std::string(run_id);
}

std::string HNOManager::infer_dataset_from_bag_path() const {
    const std::string source = !raw_bag.empty() ? raw_bag : bag_path;
    if (source.empty()) {
        return "";
    }
    boost::filesystem::path path(source);
    return path.stem().string();
}

std::string HNOManager::json_escape(const std::string& value) const {
    std::ostringstream escaped;
    for (unsigned char c : value) {
        switch (c) {
            case '"':
                escaped << "\\\"";
                break;
            case '\\':
                escaped << "\\\\";
                break;
            case '\b':
                escaped << "\\b";
                break;
            case '\f':
                escaped << "\\f";
                break;
            case '\n':
                escaped << "\\n";
                break;
            case '\r':
                escaped << "\\r";
                break;
            case '\t':
                escaped << "\\t";
                break;
            default:
                if (c < 0x20) {
                    escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<unsigned int>(c) << std::dec << std::setfill(' ');
                } else {
                    escaped << c;
                }
                break;
        }
    }
    return escaped.str();
}

void HNOManager::write_run_context(const std::string& run_dir) const {
    if (run_dir.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[HNO] Cannot write run_context.json because run directory is empty.");
        return;
    }

    const std::string context_dataset = dataset.empty() ? infer_dataset_from_bag_path() : dataset;
    const boost::filesystem::path context_path = boost::filesystem::path(run_dir) / "run_context.json";
    std::ofstream context_file(context_path.string(), std::ios::out | std::ios::trunc);
    if (!context_file.is_open()) {
        RCLCPP_WARN(node_->get_logger(), "[HNO] Failed to open run context file: %s", context_path.string().c_str());
        return;
    }

    context_file << "{\n";
    context_file << "  \"dataset\": \"" << json_escape(context_dataset) << "\",\n";
    context_file << "  \"raw_bag\": \"" << json_escape(raw_bag.empty() ? bag_path : raw_bag) << "\",\n";
    context_file << "  \"config\": \"" << json_escape(config_name) << "\",\n";
    context_file << "  \"config_path\": \"" << json_escape(config_path_) << "\",\n";
    context_file << "  \"camera_config\": \"" << json_escape(camera_config) << "\",\n";
    context_file << "  \"ground_truth_tum\": \"" << json_escape(path_gt) << "\",\n";
    context_file << "  \"odom_csv\": \"vio_results/odom_raw.csv\",\n";
    context_file << "  \"odom_tum\": \"vio_results/odom_raw.txt\",\n";
    context_file << "  \"rtabmap_input_bag\": \"vio_results/rtabmap_input_db3\",\n";
    context_file << "  \"odom_frame\": \"" << json_escape(odom_frame) << "\",\n";
    context_file << "  \"base_frame\": \"" << json_escape(base_frame) << "\",\n";
    context_file << "  \"camera_left_frame\": \"cam0_rect\",\n";
    context_file << "  \"camera_right_frame\": \"cam1_rect\",\n";
    context_file << "  \"odom_semantic\": \"T_odom_base\",\n";
    context_file << "  \"num_cams\": " << num_cams << ",\n";
    context_file << "  \"use_gt_mapping\": " << (use_gt_mapping ? "true" : "false") << "\n";
    context_file << "}\n";

    if (!context_file.good()) {
        RCLCPP_WARN(node_->get_logger(), "[HNO] Failed while writing run context file: %s", context_path.string().c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "[HNO] Wrote run context to %s", context_path.string().c_str());
}

void HNOManager::load_parameters(const std::string& config_path) {
    ov_core::YamlParser parser(config_path);

    int max_cameras = 2;
    parser.parse_config("max_cameras", max_cameras);
    num_cams = declare_or_get<int>(node_, "num_cams", max_cameras);
    num_cams = std::max(1, std::min(num_cams, max_cameras));

    for (int i = 0; i < num_cams; i++) {
        std::string cam_str = "cam" + std::to_string(i);

        std::vector<double> intrinsics, distortion_coeffs;
        std::vector<int> resolution;

        parser.parse_external("relative_config_imucam", cam_str, "intrinsics", intrinsics);
        parser.parse_external("relative_config_imucam", cam_str, "distortion_coeffs", distortion_coeffs);
        parser.parse_external("relative_config_imucam", cam_str, "resolution", resolution);

        auto cam = std::make_shared<ov_core::CamRadtan>(resolution[0], resolution[1]);
        Eigen::Matrix<double, 8, 1> calib;
        calib << intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
                 distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2], distortion_coeffs[3];
        cam->set_value(calib);
        cams.push_back(cam);

        Eigen::Matrix4d T_in_file;
        parser.parse_external("relative_config_imucam", cam_str, "T_imu_cam", T_in_file);
        cams_T_C2B.push_back(T_in_file);
    }

    double acc_sigma = 0.01;
    double gyro_sigma = 0.001;
    parser.parse_config("accelerometer_noise_density", acc_sigma);
    parser.parse_config("gyroscope_noise_density", gyro_sigma);

    HNOPropagator::NoiseParams noise_params;
    noise_params.noise_acc = acc_sigma;
    noise_params.noise_gyro = gyro_sigma;
    propagator->setNoiseParams(noise_params);

    double init_imu_thresh = 1.5;
    double init_gyro_thresh = 0.01;
    int init_window_size = 250;
    parser.parse_config("init_imu_thresh", init_imu_thresh, false);
    parser.parse_config("init_gyro_thresh", init_gyro_thresh, false);
    parser.parse_config("init_window_size", init_window_size, false);
    init_imu_thresh = declare_or_get<double>(node_, "init_imu_thresh", init_imu_thresh);
    init_gyro_thresh = declare_or_get<double>(node_, "init_gyro_thresh", init_gyro_thresh);
    init_window_size = declare_or_get<int>(node_, "init_window_size", init_window_size);
    initializer->setOptions(static_cast<size_t>(std::max(10, init_window_size)), init_imu_thresh, init_gyro_thresh);
    RCLCPP_INFO(node_->get_logger(), "[HNOInit] thresholds: acc_var=%.6f gyro_var=%.6f window=%d",
                init_imu_thresh, init_gyro_thresh, init_window_size);

    parser.parse_config("feature_tracker_num_pts", feature_options.tracker_num_pts, false);
    parser.parse_config("feature_tracker_fast_threshold", feature_options.tracker_fast_threshold, false);
    parser.parse_config("feature_tracker_grid_x", feature_options.tracker_grid_x, false);
    parser.parse_config("feature_tracker_grid_y", feature_options.tracker_grid_y, false);
    parser.parse_config("feature_tracker_min_px_dist", feature_options.tracker_min_px_dist, false);
    parser.parse_config("feature_min_stereo_depth", feature_options.min_stereo_depth, false);
    parser.parse_config("feature_max_stereo_depth", feature_options.max_stereo_depth, false);
    parser.parse_config("feature_stereo_reproj_thresh", feature_options.stereo_reproj_thresh, false);
    parser.parse_config("feature_reproj_thresh", feature_options.reproj_thresh, false);
    parser.parse_config("feature_reproj_thresh_low", feature_options.reproj_thresh_low, false);
    parser.parse_config("feature_low_feature_pts", feature_options.low_feature_pts, false);
    parser.parse_config("feature_low_feature_db", feature_options.low_feature_db, false);
    parser.parse_config("feature_mature_thresh", feature_options.mature_thresh, false);
    parser.parse_config("feature_mature_thresh_low", feature_options.mature_thresh_low, false);
    parser.parse_config("feature_fail_limit", feature_options.fail_limit, false);
    parser.parse_config("feature_fail_limit_low", feature_options.fail_limit_low, false);
    parser.parse_config("feature_map_jump_thresh", feature_options.map_jump_thresh, false);
    parser.parse_config("feature_active_mature_thresh", feature_options.active_mature_thresh, false);
    parser.parse_config("feature_health_min_stable", feature_options.health_min_stable, false);
    parser.parse_config("feature_health_min_db", feature_options.health_min_db, false);
    parser.parse_config("feature_health_hold_frames", feature_options.health_hold_frames, false);
    parser.parse_config("feature_health_start_frame", feature_options.health_start_frame, false);

    parser.parse_config("update_pixel_noise", updater_options.pixel_noise, false);
    parser.parse_config("update_focal_length", updater_options.focal_length, false);
    parser.parse_config("update_chi2_gate", updater_options.chi2_gate, false);
    parser.parse_config("update_max_delta_p", updater_options.max_delta_p, false);
    parser.parse_config("update_max_delta_r", updater_options.max_delta_r, false);
    parser.parse_config("update_min_observations", updater_options.min_observations, false);
    parser.parse_config("update_low_observation_hold_frames", updater_options.low_observation_hold_frames, false);
    parser.parse_config("update_warn_delta_ratio", updater_options.warn_delta_ratio, false);
    parser.parse_config("update_enforce_structure", updater_options.enforce_structure_after_update, false);

    feature_options.tracker_num_pts = declare_or_get<int>(node_, "feature_tracker_num_pts", feature_options.tracker_num_pts);
    feature_options.tracker_fast_threshold = declare_or_get<int>(node_, "feature_tracker_fast_threshold", feature_options.tracker_fast_threshold);
    feature_options.tracker_grid_x = declare_or_get<int>(node_, "feature_tracker_grid_x", feature_options.tracker_grid_x);
    feature_options.tracker_grid_y = declare_or_get<int>(node_, "feature_tracker_grid_y", feature_options.tracker_grid_y);
    feature_options.tracker_min_px_dist = declare_or_get<int>(node_, "feature_tracker_min_px_dist", feature_options.tracker_min_px_dist);
    feature_options.min_stereo_depth = declare_or_get<double>(node_, "feature_min_stereo_depth", feature_options.min_stereo_depth);
    feature_options.max_stereo_depth = declare_or_get<double>(node_, "feature_max_stereo_depth", feature_options.max_stereo_depth);
    feature_options.stereo_reproj_thresh = declare_or_get<double>(node_, "feature_stereo_reproj_thresh", feature_options.stereo_reproj_thresh);
    feature_options.reproj_thresh = declare_or_get<double>(node_, "feature_reproj_thresh", feature_options.reproj_thresh);
    feature_options.reproj_thresh_low = declare_or_get<double>(node_, "feature_reproj_thresh_low", feature_options.reproj_thresh_low);
    feature_options.low_feature_pts = declare_or_get<int>(node_, "feature_low_feature_pts", feature_options.low_feature_pts);
    feature_options.low_feature_db = declare_or_get<int>(node_, "feature_low_feature_db", feature_options.low_feature_db);
    feature_options.mature_thresh = declare_or_get<int>(node_, "feature_mature_thresh", feature_options.mature_thresh);
    feature_options.mature_thresh_low = declare_or_get<int>(node_, "feature_mature_thresh_low", feature_options.mature_thresh_low);
    feature_options.fail_limit = declare_or_get<int>(node_, "feature_fail_limit", feature_options.fail_limit);
    feature_options.fail_limit_low = declare_or_get<int>(node_, "feature_fail_limit_low", feature_options.fail_limit_low);
    feature_options.map_jump_thresh = declare_or_get<double>(node_, "feature_map_jump_thresh", feature_options.map_jump_thresh);
    feature_options.active_mature_thresh = declare_or_get<int>(node_, "feature_active_mature_thresh", feature_options.active_mature_thresh);
    feature_options.health_min_stable = declare_or_get<int>(node_, "feature_health_min_stable", feature_options.health_min_stable);
    feature_options.health_min_db = declare_or_get<int>(node_, "feature_health_min_db", feature_options.health_min_db);
    feature_options.health_hold_frames = declare_or_get<int>(node_, "feature_health_hold_frames", feature_options.health_hold_frames);
    feature_options.health_start_frame = declare_or_get<int>(node_, "feature_health_start_frame", feature_options.health_start_frame);

    updater_options.pixel_noise = declare_or_get<double>(node_, "update_pixel_noise", updater_options.pixel_noise);
    updater_options.focal_length = declare_or_get<double>(node_, "update_focal_length", updater_options.focal_length);
    updater_options.chi2_gate = declare_or_get<double>(node_, "update_chi2_gate", updater_options.chi2_gate);
    updater_options.max_delta_p = declare_or_get<double>(node_, "update_max_delta_p", updater_options.max_delta_p);
    updater_options.max_delta_r = declare_or_get<double>(node_, "update_max_delta_r", updater_options.max_delta_r);
    updater_options.min_observations = declare_or_get<int>(node_, "update_min_observations", updater_options.min_observations);
    updater_options.low_observation_hold_frames =
        declare_or_get<int>(node_, "update_low_observation_hold_frames", updater_options.low_observation_hold_frames);
    updater_options.warn_delta_ratio = declare_or_get<double>(node_, "update_warn_delta_ratio", updater_options.warn_delta_ratio);
    updater_options.enforce_structure_after_update =
        declare_or_get<bool>(node_, "update_enforce_structure", updater_options.enforce_structure_after_update);

    path_gt = declare_or_get<std::string>(node_, "path_gt", "");
    if (!path_gt.empty()) {
        load_gt_data();
    } else {
        RCLCPP_WARN(node_->get_logger(), "No GT path provided in params.");
    }

    use_gt_mapping = declare_or_get<bool>(node_, "use_gt_mapping", false);
    export_odom = declare_or_get<bool>(node_, "export_odom", false);
    odom_output_path = declare_or_get<std::string>(node_, "odom_output_path", "");
    dataset = declare_or_get<std::string>(node_, "dataset", "");
    bag_path = declare_or_get<std::string>(node_, "bag_path", "");
    raw_bag = declare_or_get<std::string>(node_, "raw_bag", bag_path);
    config_name = declare_or_get<std::string>(node_, "config", "");
    camera_config = declare_or_get<std::string>(node_, "camera_config", "");
    odom_frame = declare_or_get<std::string>(node_, "odom_frame", "odom");
    base_frame = declare_or_get<std::string>(node_, "base_frame", "base_link");
    world_frame = odom_frame;
    topic_imu = declare_or_get<std::string>(node_, "topic_imu", "/imu0");
    topic_cam0 = declare_or_get<std::string>(node_, "topic_cam0", "/cam0/image_raw");
    topic_cam1 = declare_or_get<std::string>(node_, "topic_cam1", "/cam1/image_raw");

    RCLCPP_INFO(node_->get_logger(), "[HNO] Switches: use_gt_mapping=%s export_odom=%s",
                use_gt_mapping ? "true" : "false", export_odom ? "true" : "false");
    if (use_gt_mapping) {
        RCLCPP_WARN(node_->get_logger(), "[HNO] RUNNING IN WANG 2022 GT MAPPING MODE!");
    } else {
        RCLCPP_INFO(node_->get_logger(), "[HNO] RUNNING IN REAL MAPPING MODE.");
    }

    if (export_odom) {
        if (odom_output_path.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[HNO] export_odom is true but odom_output_path is empty. Disabling odom export.");
            export_odom = false;
        } else {
            const std::string run_id_token = "{run_id}";
            size_t token_pos = odom_output_path.find(run_id_token);
            if (token_pos != std::string::npos) {
                const std::string run_id = make_run_id_beijing_time();
                odom_output_path.replace(token_pos, run_id_token.size(), run_id);
            }

            boost::filesystem::path out_path(odom_output_path);
            boost::filesystem::path parent = out_path.parent_path();
            if (!parent.empty()) {
                boost::filesystem::create_directories(parent);
            }

            odom_output_file.open(odom_output_path, std::ios::out | std::ios::trunc);
            if (!odom_output_file.is_open()) {
                RCLCPP_ERROR(node_->get_logger(), "[HNO] Failed to open odom output file: %s", odom_output_path.c_str());
                export_odom = false;
            } else {
                boost::filesystem::path txt_path = out_path;
                txt_path.replace_extension(".txt");
                odom_txt_output_path = txt_path.string();
                odom_txt_output_file.open(odom_txt_output_path, std::ios::out | std::ios::trunc);
                if (!odom_txt_output_file.is_open()) {
                    RCLCPP_ERROR(node_->get_logger(), "[HNO] Failed to open odom TXT output file: %s", odom_txt_output_path.c_str());
                    odom_output_file.close();
                    export_odom = false;
                    return;
                }

                odom_output_file << "timestamp_ns,tx,ty,tz,qx,qy,qz,qw\n";
                odom_output_file << std::fixed << std::setprecision(12);
                odom_txt_output_file << std::fixed << std::setprecision(9);
                boost::filesystem::path run_dir = parent.filename().string() == "vio_results" ? parent.parent_path() : parent;
                write_run_context(run_dir.string());
                RCLCPP_INFO(node_->get_logger(), "[HNO] Exporting camera-frame odom to %s and %s",
                            odom_output_path.c_str(), odom_txt_output_path.c_str());
            }
        }
    }
}

void HNOManager::launch_subscribers() {
    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_imu = node_->create_subscription<sensor_msgs::msg::Imu>(
        topic_imu, sensor_qos, std::bind(&HNOManager::imu_callback, this, std::placeholders::_1));

    if (num_cams == 2) {
        sub_cam0 = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
        sub_cam1 = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
        sub_cam0->subscribe(node_.get(), topic_cam0, sensor_qos.get_rmw_qos_profile());
        sub_cam1->subscribe(node_.get(), topic_cam1, sensor_qos.get_rmw_qos_profile());

        sync = std::make_unique<Sync>(MySyncPolicy(10), *sub_cam0, *sub_cam1);
        sync->registerCallback(std::bind(&HNOManager::stereo_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(node_->get_logger(), "Subscribed to Stereo: %s, %s", topic_cam0.c_str(), topic_cam1.c_str());
    } else if (num_cams == 1) {
        sub_cam0_mono = node_->create_subscription<sensor_msgs::msg::Image>(
            topic_cam0, sensor_qos, std::bind(&HNOManager::mono_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Subscribed to Mono: %s", topic_cam0.c_str());
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Unsupported num_cams=%d; expected 1 or 2.", num_cams);
        rclcpp::shutdown();
    }
}

void HNOManager::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ov_core::ImuData data;
    data.timestamp = stamp_to_sec(msg->header.stamp);
    data.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    data.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    feed_measurement(data);
}

void HNOManager::mono_callback(const sensor_msgs::msg::Image::SharedPtr msg0) {
    ov_core::CameraData data;
    data.timestamp = stamp_to_sec(msg0->header.stamp);
    data.sensor_ids = {0};
    try {
        data.images.push_back(cv_bridge::toCvCopy(msg0, sensor_msgs::image_encodings::MONO8)->image);
        data.masks.push_back(cv::Mat::zeros(data.images[0].rows, data.images[0].cols, CV_8UC1));
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to convert mono image: %s", e.what());
        return;
    }
    feed_measurement(data);
}

void HNOManager::stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg0,
                                 const sensor_msgs::msg::Image::ConstSharedPtr msg1) {
    ov_core::CameraData data;
    data.timestamp = stamp_to_sec(msg0->header.stamp);
    data.sensor_ids = {0, 1};
    try {
        data.images.push_back(cv_bridge::toCvCopy(msg0, sensor_msgs::image_encodings::MONO8)->image);
        data.images.push_back(cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::MONO8)->image);
        data.masks.push_back(cv::Mat::zeros(data.images[0].rows, data.images[0].cols, CV_8UC1));
        data.masks.push_back(cv::Mat::zeros(data.images[1].rows, data.images[1].cols, CV_8UC1));
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to convert stereo image: %s", e.what());
        return;
    }
    feed_measurement(data);
}

void HNOManager::feed_measurement(const ov_core::ImuData& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);

    imu_data_buffer.push_back(msg);

    if (!is_initialized) {
        initializer->feedImuData(msg);
        if (initializer->initialize(state, current_time)) {
            is_initialized = true;
            RCLCPP_INFO(node_->get_logger(), "Statical Initialization Done at %.3f", current_time);
        }
        return;
    }

    auto state_viz = std::make_shared<HNOState>(*state);
    double sim_time = current_time;

    for (const auto& imu : imu_data_buffer) {
        if (imu.timestamp > sim_time) {
            double dt = imu.timestamp - sim_time;
            if (dt > 1e-6) {
                propagator->propagate(state_viz, imu.wm, imu.am, dt);
                sim_time = imu.timestamp;
            }
        }
    }

    publish_state(sim_time, state_viz);
}

void HNOManager::feed_measurement(const ov_core::CameraData& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (is_initialized) {
        process_camera_data(msg);
    }
}

bool HNOManager::get_interpolated_gt(double timestamp, Eigen::Vector3d& p_gt, Eigen::Matrix3d& R_gt) {
    if (gt_states.empty()) return false;

    auto it = gt_states.lower_bound(timestamp);
    if (it == gt_states.end() || it == gt_states.begin()) return false;

    auto it_prev = std::prev(it);
    double t1 = it_prev->first;
    double t2 = it->first;
    double alpha = (timestamp - t1) / (t2 - t1);

    Eigen::Vector3d p1 = it_prev->second.head<3>();
    Eigen::Vector3d p2 = it->second.head<3>();
    p_gt = (1.0 - alpha) * p1 + alpha * p2;

    Eigen::Quaterniond q1(it_prev->second(6), it_prev->second(3), it_prev->second(4), it_prev->second(5));
    Eigen::Quaterniond q2(it->second(6), it->second(3), it->second(4), it->second(5));
    R_gt = q1.slerp(alpha, q2).toRotationMatrix();

    return true;
}

void HNOManager::process_camera_data(const ov_core::CameraData& msg) {
    if (msg.timestamp <= current_time) {
        RCLCPP_WARN(node_->get_logger(), "Camera message skipped (old params): %.3f <= %.3f", msg.timestamp, current_time);
        return;
    }

    for (const auto& imu : imu_data_buffer) {
        if (imu.timestamp > current_time && imu.timestamp <= msg.timestamp) {
            double dt = imu.timestamp - current_time;
            if (dt > 1e-6) propagator->propagate(state, imu.wm, imu.am, dt);
            current_time = imu.timestamp;
        }
    }
    if (msg.timestamp > current_time && !imu_data_buffer.empty()) {
        auto& last_imu = imu_data_buffer.back();
        double dt = msg.timestamp - current_time;
        if (dt > 1e-6) propagator->propagate(state, last_imu.wm, last_imu.am, dt);
        current_time = msg.timestamp;
    }

    auto it = imu_data_buffer.begin();
    while (it != imu_data_buffer.end() && it->timestamp <= current_time) {
        it = imu_data_buffer.erase(it);
    }

    std::vector<HNOObservation> valid_measurements;
    Eigen::Vector3d p_gt;
    Eigen::Matrix3d R_gt;
    bool has_gt = get_interpolated_gt(msg.timestamp, p_gt, R_gt);

    if (first_timestamp < 0) first_timestamp = msg.timestamp;
    if (!is_initialized) return;

    bool enable_cheat = use_gt_mapping && has_gt;
    if (enable_cheat) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[Wang 2022] Using GT for Mapping");
        feature_handler->feed_measurement(msg, state->R_hat_B2I, state->p_hat, R_gt, p_gt, valid_measurements);
    } else {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[HNO] Using estimate for Mapping");
        feature_handler->feed_measurement(msg, state->R_hat_B2I, state->p_hat, state->R_hat_B2I, state->p_hat, valid_measurements);
    }

    int num_obs = valid_measurements.size();
    if (!valid_measurements.empty()) {
        updater->update(state, valid_measurements);
    }

    int map_size = feature_handler->get_active_map().size();
    compute_and_print_error(msg.timestamp, state->p_hat, map_size, num_obs);
    export_odom_state(msg.timestamp, state);
    publish_visualization(msg.timestamp, msg);
}

void HNOManager::publish_state(double timestamp, const std::shared_ptr<HNOState>& state_viz) {
    rclcpp::Time rtime = time_from_sec(timestamp);
    if (std::abs(timestamp - last_published_time) < 1e-9) return;
    last_published_time = timestamp;

    geometry_msgs::msg::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.stamp = rtime;
    msg_pose.header.frame_id = odom_frame;
    msg_pose.pose.pose.position.x = state_viz->p_hat.x();
    msg_pose.pose.pose.position.y = state_viz->p_hat.y();
    msg_pose.pose.pose.position.z = state_viz->p_hat.z();
    Eigen::Quaterniond q(state_viz->R_hat_B2I);
    q.normalize();
    msg_pose.pose.pose.orientation.w = q.w();
    msg_pose.pose.pose.orientation.x = q.x();
    msg_pose.pose.pose.orientation.y = q.y();
    msg_pose.pose.pose.orientation.z = q.z();
    pub_pose->publish(msg_pose);

    nav_msgs::msg::Odometry msg_odom;
    msg_odom.header = msg_pose.header;
    msg_odom.child_frame_id = base_frame;
    msg_odom.pose.pose = msg_pose.pose.pose;
    msg_odom.twist.twist.linear.x = state_viz->v_hat.x();
    msg_odom.twist.twist.linear.y = state_viz->v_hat.y();
    msg_odom.twist.twist.linear.z = state_viz->v_hat.z();
    pub_odom->publish(msg_odom);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg_pose.header;
    ps.pose = msg_pose.pose.pose;
    path_msg.header.stamp = rtime;
    path_msg.header.frame_id = odom_frame;
    path_msg.poses.push_back(ps);
    pub_path->publish(path_msg);

    Eigen::Vector3d p_gt;
    Eigen::Matrix3d R_gt;
    if (get_interpolated_gt(timestamp, p_gt, R_gt)) {
        if (!has_gt_viz_align) {
            R_gt_viz_align = state_viz->R_hat_B2I * R_gt.transpose();
            t_gt_viz_align = state_viz->p_hat - R_gt_viz_align * p_gt;
            has_gt_viz_align = true;
        }

        Eigen::Vector3d p_gt_aligned = R_gt_viz_align * p_gt + t_gt_viz_align;
        Eigen::Matrix3d R_gt_aligned = R_gt_viz_align * R_gt;
        Eigen::Quaterniond q_gt_aligned(R_gt_aligned);

        geometry_msgs::msg::PoseStamped ps_gt;
        ps_gt.header = msg_pose.header;
        ps_gt.pose.position.x = p_gt_aligned.x();
        ps_gt.pose.position.y = p_gt_aligned.y();
        ps_gt.pose.position.z = p_gt_aligned.z();
        ps_gt.pose.orientation.w = q_gt_aligned.w();
        ps_gt.pose.orientation.x = q_gt_aligned.x();
        ps_gt.pose.orientation.y = q_gt_aligned.y();
        ps_gt.pose.orientation.z = q_gt_aligned.z();

        path_gt_msg.header.stamp = rtime;
        path_gt_msg.header.frame_id = odom_frame;
        path_gt_msg.poses.push_back(ps_gt);
        pub_path_gt->publish(path_gt_msg);

        geometry_msgs::msg::TransformStamped transform_gt;
        transform_gt.header.stamp = rtime;
        transform_gt.header.frame_id = odom_frame;
        transform_gt.child_frame_id = gt_base_frame;
        transform_gt.transform.translation.x = p_gt_aligned.x();
        transform_gt.transform.translation.y = p_gt_aligned.y();
        transform_gt.transform.translation.z = p_gt_aligned.z();
        transform_gt.transform.rotation.x = q_gt_aligned.x();
        transform_gt.transform.rotation.y = q_gt_aligned.y();
        transform_gt.transform.rotation.z = q_gt_aligned.z();
        transform_gt.transform.rotation.w = q_gt_aligned.w();
        tf_broadcaster->sendTransform(transform_gt);
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rtime;
    transform.header.frame_id = odom_frame;
    transform.child_frame_id = base_frame;
    transform.transform.translation.x = state_viz->p_hat.x();
    transform.transform.translation.y = state_viz->p_hat.y();
    transform.transform.translation.z = state_viz->p_hat.z();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(transform);
}

void HNOManager::export_odom_state(double timestamp, const std::shared_ptr<HNOState>& state_to_export) {
    if (!export_odom || !odom_output_file.is_open() || !state_to_export) return;
    if (last_exported_odom_time > 0.0 && timestamp <= last_exported_odom_time) return;

    Eigen::Quaterniond q(state_to_export->R_hat_B2I);
    q.normalize();
    const int64_t timestamp_ns = static_cast<int64_t>(std::llround(timestamp * 1e9));

    odom_output_file << timestamp_ns << ","
                     << state_to_export->p_hat.x() << ","
                     << state_to_export->p_hat.y() << ","
                     << state_to_export->p_hat.z() << ","
                     << q.x() << ","
                     << q.y() << ","
                     << q.z() << ","
                     << q.w() << "\n";
    if (odom_txt_output_file.is_open()) {
        odom_txt_output_file << timestamp << " "
                             << state_to_export->p_hat.x() << " "
                             << state_to_export->p_hat.y() << " "
                             << state_to_export->p_hat.z() << " "
                             << q.x() << " "
                             << q.y() << " "
                             << q.z() << " "
                             << q.w() << "\n";
    }
    last_exported_odom_time = timestamp;
}

void HNOManager::publish_visualization(double timestamp, const ov_core::CameraData& msg) {
    rclcpp::Time rtime = time_from_sec(timestamp);

    sensor_msgs::msg::PointCloud msg_pc;
    msg_pc.header.stamp = rtime;
    msg_pc.header.frame_id = odom_frame;

    const auto& map = feature_handler->get_active_map();
    for (auto& pair : map) {
        geometry_msgs::msg::Point32 p;
        p.x = pair.second.x();
        p.y = pair.second.y();
        p.z = pair.second.z();
        msg_pc.points.push_back(p);
    }
    pub_feat->publish(msg_pc);

    if (pub_img->get_subscription_count() > 0) {
        const auto& active_map = feature_handler->get_active_map();
        auto obs = feature_handler->get_tracker()->get_last_obs();
        auto ids = feature_handler->get_tracker()->get_last_ids();

        std::unordered_set<size_t> allowed_trails;
        for (const auto& kv : active_map) allowed_trails.insert(kv.first);

        cv::Mat img_viz;
        feature_handler->get_tracker()->display_history(img_viz, 0, 255, 0, 255, 0, 0, {}, "HNO Tracker", &allowed_trails);
        if (img_viz.empty()) return;

        int width = msg.images[0].cols;
        int height = msg.images[0].rows;
        bool has_right_img = (msg.images.size() > 1 && !msg.images[1].empty());
        if (img_viz.cols >= width && img_viz.rows >= height) {
            cv::Mat left_roi = img_viz(cv::Rect(0, 0, width, height));
            cv::Mat right_roi;
            if (has_right_img && img_viz.cols >= 2 * width) {
                right_roi = img_viz(cv::Rect(width, 0, width, height));
            }

            if (obs.count(0) && ids.count(0)) {
                size_t num = ids[0].size();
                for (size_t i = 0; i < num; ++i) {
                    size_t id = ids[0][i];
                    cv::Point2f pt = obs[0][i].pt;
                    if (active_map.count(id)) {
                        cv::circle(left_roi, pt, 6, cv::Scalar(0, 0, 255), 2);
                    } else {
                        cv::circle(left_roi, pt, 6, cv::Scalar(0, 255, 255), 1);
                    }
                }
            }

            if (has_right_img && !right_roi.empty() && obs.count(1) && ids.count(1)) {
                for (size_t k = 0; k < ids[1].size(); ++k) {
                    size_t id = ids[1][k];
                    cv::Point2f pt = obs[1][k].pt;
                    if (active_map.count(id)) {
                        cv::circle(right_roi, pt, 6, cv::Scalar(0, 0, 255), 2);
                    } else {
                        cv::circle(right_roi, pt, 6, cv::Scalar(0, 255, 255), 1);
                    }
                }
            }
        }

        auto msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_viz).toImageMsg();
        msg_img->header.stamp = rtime;
        msg_img->header.frame_id = base_frame;
        pub_img->publish(*msg_img);
    }
}

void HNOManager::load_gt_data() {
    std::ifstream f(path_gt);
    if (!f.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open GT file: %s", path_gt.c_str());
        return;
    }

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::vector<double> val;
        double d;
        while (ss >> d) val.push_back(d);

        if (val.size() >= 8) {
            double ts = val[0];
            if (ts > 1e10) ts *= 1e-9;

            Eigen::VectorXd pose(7);
            pose << val[1], val[2], val[3], val[4], val[5], val[6], val[7];
            gt_states[ts] = pose;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Loaded %zu GT poses.", gt_states.size());
}

void HNOManager::compute_and_print_error(double timestamp, const Eigen::Vector3d& p_est, int num_feats, int num_obs) {
    std::string err_str = "";

    if (!gt_states.empty()) {
        auto it = gt_states.lower_bound(timestamp);
        bool found = false;
        Eigen::VectorXd best_gt;
        double min_dt = 100.0;

        if (it != gt_states.end()) {
            double dt = std::abs(it->first - timestamp);
            if (dt < min_dt) {
                min_dt = dt;
                best_gt = it->second;
                found = true;
            }
        }
        if (it != gt_states.begin()) {
            auto it_prev = std::prev(it);
            double dt = std::abs(it_prev->first - timestamp);
            if (dt < min_dt) {
                min_dt = dt;
                best_gt = it_prev->second;
                found = true;
            }
        }

        if (found && min_dt < 0.05) {
            Eigen::Vector3d p_gt = best_gt.head<3>();
            if (!has_align) {
                Eigen::Quaterniond q_gt(best_gt(6), best_gt(3), best_gt(4), best_gt(5));
                Eigen::Matrix3d R_gt = q_gt.toRotationMatrix();
                R_align = R_gt * state->R_hat_B2I.transpose();
                t_align = p_gt - R_align * p_est;
                has_align = true;
            }

            Eigen::Vector3d p_aligned = R_align * p_est + t_align;
            Eigen::Vector3d err = p_aligned - p_gt;
            char buff[128];
            snprintf(buff, sizeof(buff), "Err:%.3f (xyz: %.2f %.2f %.2f)", err.norm(), err.x(), err.y(), err.z());
            err_str = std::string(buff);
        }
    }

    Eigen::Vector3d v = state->v_hat;
    Eigen::Matrix3d E;
    E.col(0) = state->e_hat[0];
    E.col(1) = state->e_hat[1];
    E.col(2) = state->e_hat[2];
    double e_orth_frob = (E.transpose() * E - Eigen::Matrix3d::Identity()).norm();

    printf("[HNO] %.3f | Feat:%d/%d | Pos:%.2f %.2f %.2f | Vel:%.2f %.2f %.2f | EOrth:%.6f | %s\n",
           timestamp, num_feats, num_obs,
           p_est.x(), p_est.y(), p_est.z(),
           v.x(), v.y(), v.z(),
           e_orth_frob,
           err_str.c_str());
}
