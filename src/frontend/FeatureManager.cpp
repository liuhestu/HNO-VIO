#include "hno_vio/frontend/FeatureManager.h"
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <unordered_set>

using namespace hno_vio;
using namespace hno_vio::frontend;

FeatureManager::FeatureManager(std::vector<std::shared_ptr<ov_core::CamBase>> cams,
                       std::vector<Eigen::Matrix4d> extrinsics,
                       const Options& options)
    : options_(options),
      cameras(cams),
      T_C_B(extrinsics),
      triangulator_(extrinsics,
                    StereoTriangulator::Constraints{options.min_stereo_depth,
                                                    options.max_stereo_depth,
                                                    options.stereo_reproj_thresh}),
      feature_health_(FeatureHealth::Constraints{options.health_start_frame,
                                                options.health_min_stable,
                                                options.health_min_db,
                                                options.health_hold_frames}) {

    std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> cam_map;
    for(size_t i=0; i<cams.size(); ++i) cam_map[i] = cams[i];

    // 稍微增加一点点特征点上限，因为我们会扔掉很多不稳定的点
    tracker = std::make_shared<ov_core::TrackKLT>(
        cam_map,
        options_.tracker_num_pts,
        0,
        true,
        ov_core::TrackBase::HistogramMethod::HISTOGRAM,
        options_.tracker_fast_threshold,
        options_.tracker_grid_x,
        options_.tracker_grid_y,
        options_.tracker_min_px_dist);
}

void FeatureManager::processStereo(const ov_core::CameraData& message,
                                   const Pose& mapping_pose,
                                   std::vector<observer::VisualObservation>& observations,
                                   FeatureDiagnostics* diagnostics) {
    observations.clear();
    last_median_disparity_ = std::numeric_limits<double>::infinity();
    last_common_track_count_ = 0;
    if (diagnostics) {
        *diagnostics = FeatureDiagnostics{};
        diagnostics->landmark_map_size = static_cast<int>(landmark_map_.size());
        diagnostics->median_disparity = last_median_disparity_;
    }

    tracker->feed_new_camera(message);

    auto obs_raw = tracker->get_last_obs();
    auto ids_raw = tracker->get_last_ids();

    if(!obs_raw.count(0)) return;
    if (diagnostics) {
        diagnostics->tracked_count = static_cast<int>(obs_raw[0].size());
    }
    if(obs_raw[0].empty()) return; // 无特征时直接退出，避免尾帧继续估计

    // --- 1. RANSAC (2D-2D) 剔除动态点 ---
    std::unordered_set<size_t> outlier_ids;
    if (!history_obs.empty()) {
        std::vector<cv::Point2f> pts_prev, pts_curr;
        std::vector<size_t> pts_ids;
        std::vector<double> disparities;
        for(size_t i=0; i<ids_raw[0].size(); ++i) {
            size_t id = ids_raw[0][i];
            if(history_obs.count(id)) {
                pts_prev.push_back(history_obs[id]);
                pts_curr.push_back(obs_raw[0][i].pt);
                pts_ids.push_back(id);
                disparities.push_back(
                    cv::norm(obs_raw[0][i].pt - history_obs[id]));
            }
        }
        last_common_track_count_ = static_cast<int>(disparities.size());
        if(!disparities.empty()) {
            const size_t mid = disparities.size() / 2;
            std::nth_element(disparities.begin(),
                             disparities.begin() + mid,
                             disparities.end());
            last_median_disparity_ = disparities[mid];
        }
        if(pts_prev.size() >= 15) {
             std::vector<uchar> status;
             cv::findFundamentalMat(pts_prev, pts_curr, cv::FM_RANSAC, 2.0, 0.99, status);
             for(size_t k=0; k<status.size(); ++k) {
                 if(!status[k]) outlier_ids.insert(pts_ids[k]);
             }
        }
    }
    if (diagnostics) {
        diagnostics->common_track_count = last_common_track_count_;
        diagnostics->median_disparity = last_median_disparity_;
    }

    // --- 准备数据 ---
    std::map<size_t, cv::Point2f> next_history_obs;

    std::map<size_t, int> right_cam_idx;
    if(obs_raw.count(1)) {
        for(size_t i=0; i<ids_raw[1].size(); ++i) right_cam_idx[ids_raw[1][i]] = i;
    }

    const Eigen::Matrix3d& R_wb = mapping_pose.R;
    const Eigen::Vector3d& p_wb = mapping_pose.p;

    // 获取内参 T_bc (Cam -> Body)
    Eigen::Matrix3d R_bc = T_C_B[0].block<3,3>(0,0);
    Eigen::Vector3d p_bc = T_C_B[0].block<3,1>(0,3);

    int count_stable = 0;
    int count_new = 0;
    int reproj_pass = 0, reproj_reject = 0;
    double reproj_err_max = 0.0, reproj_err_sum = 0.0;
    int stereo_pass = 0, stereo_reject = 0;
    double stereo_err_max = 0.0, stereo_err_sum = 0.0;

    bool low_feat_mode = false; // 自适应：少点时放宽门限并提前输出

    // --- 遍历左目所有特征点 ---
    size_t num_pts = obs_raw[0].size();
    if(num_pts < static_cast<size_t>(options_.low_feature_pts) ||
       landmark_map_.size() < static_cast<size_t>(options_.low_feature_db)) low_feat_mode = true;
    double reproj_thresh = low_feat_mode ? options_.reproj_thresh_low : options_.reproj_thresh;
    int mature_thresh = low_feat_mode ? options_.mature_thresh_low : options_.mature_thresh;
    for(size_t i=0; i<num_pts; ++i) {
        size_t id = ids_raw[0][i];
        if(outlier_ids.count(id)) continue; // 跳过 2D RANSAC 外点

        next_history_obs[id] = obs_raw[0][i].pt;

        // 计算左目归一化坐标
        Eigen::Vector2d uv_l_px(obs_raw[0][i].pt.x, obs_raw[0][i].pt.y);
        Eigen::Vector2d uv_l_norm = cameras[0]->undistort_d(uv_l_px);
        Eigen::Vector3d uv_l_vec(uv_l_norm.x(), uv_l_norm.y(), 1.0);

        // 检查右目是否存在
        bool has_right = false;
        Eigen::Vector3d uv_r_vec = Eigen::Vector3d::Zero();
        if(right_cam_idx.count(id)) {
            int idx_r = right_cam_idx[id];
            cv::Point2f pt_r = obs_raw[1][idx_r].pt;
            Eigen::Vector2d uv_r_px(pt_r.x, pt_r.y);
            Eigen::Vector2d uv_r_norm = cameras[1]->undistort_d(uv_r_px);
            uv_r_vec << uv_r_norm.x(), uv_r_norm.y(), 1.0;
            has_right = true;
        }

        // --- 逻辑分支 ---
        // 1. 老点：使用当前帧双目刷新 p_w；若无双目则重投影检查
        if(landmark_map_.contains(id)) {
            Landmark& info = landmark_map_.at(id);

            bool stereo_ok = false;
            Eigen::Vector3d p_w_new = info.p_world;
            if(has_right) {
                Eigen::Vector3d p_c_curr;
                double stereo_err = 0.0;
                if(triangulator_.triangulate(uv_l_vec, uv_r_vec, p_c_curr, &stereo_err)) {
                    double depth = p_c_curr.z();
                    if(depth > options_.min_stereo_depth && depth < options_.max_stereo_depth) {
                        stereo_pass++;
                        stereo_err_sum += stereo_err;
                        if(stereo_err > stereo_err_max) stereo_err_max = stereo_err;
                        stereo_ok = true;
                        Eigen::Vector3d p_body = R_bc * p_c_curr + p_bc;
                        p_w_new = R_wb * p_body + p_wb;

                        double dist = (info.p_world - p_w_new).norm();
                        if(dist > options_.map_jump_thresh) {
                            // Keep the world anchor while the KLT track is alive.
                            info.fail_count++;
                            continue;
                        } else {
                            double alpha = 1.0 / (info.track_count + 1.0);
                            if(alpha < 0.05) alpha = 0.05;
                            if(alpha > 0.2) alpha = 0.2;
                            info.p_world = (1.0 - alpha) * info.p_world + alpha * p_w_new;
                        }
                    } else {
                        stereo_reject++;
                    }
                }
                else {
                    stereo_reject++;
                }
            }

            Eigen::Vector3d p_w_est = stereo_ok ? p_w_new : info.p_world;
            double reproj_err = 0.0;
            bool reproj_ok = stereo_ok ? true : check_reprojection(p_w_est, R_wb, p_wb, uv_l_vec, reproj_thresh, &reproj_err);

            if(!reproj_ok) {
                info.fail_count++;
                reproj_reject++;
                continue;
            }

            reproj_pass++;
            reproj_err_sum += reproj_err;
            if(reproj_err > reproj_err_max) reproj_err_max = reproj_err;

            info.fail_count = 0;
            if(info.track_count < 15) info.track_count++;

            // 只有成熟点才输出观测；少点时提前输出
            if(info.track_count >= mature_thresh) {
                observer::VisualObservation obs;
                obs.uv_left = uv_l_vec.normalized();
                if(stereo_ok) {
                    obs.uv_right = uv_r_vec.normalized();
                    obs.has_right = true;
                } else {
                    obs.has_right = false;
                }
                obs.landmark = info.p_world;
                observations.push_back(obs);
                count_stable++;
            }

        }
        // 2. 新点：双目初始化，先建图不输出，等待成熟
        else if(has_right) {
            Eigen::Vector3d p_c_left;
            double stereo_err = 0.0;
            if(triangulator_.triangulate(uv_l_vec, uv_r_vec, p_c_left, &stereo_err)) {
                double depth = p_c_left.z();
                if(depth > options_.min_stereo_depth && depth < options_.max_stereo_depth) {
                    stereo_pass++;
                    stereo_err_sum += stereo_err;
                    if(stereo_err > stereo_err_max) stereo_err_max = stereo_err;
                    Landmark info;
                    Eigen::Vector3d p_body = R_bc * p_c_left + p_bc;
                    info.p_world = R_wb * p_body + p_wb;
                    info.track_count = 1;
                    info.fail_count = 0;
                    landmark_map_.insert(id, info);
                    count_new++;
                } else {
                    stereo_reject++;
                }
            } else {
                stereo_reject++;
            }
        }
    }

    // 4. 地图清理
    std::unordered_set<size_t> klt_alive_ids;
    if(obs_raw.count(0)) {
        for(size_t id : ids_raw[0]) klt_alive_ids.insert(id);
    }

    landmark_map_.eraseMissing(klt_alive_ids);

    history_obs = next_history_obs;

    const FeatureHealth::Status health =
        feature_health_.update(count_stable, static_cast<int>(landmark_map_.size()));

    if(!health.allow_visual_update) {
        if(!observations.empty()) {
            observations.clear();
        }
    }

    if (diagnostics) {
        diagnostics->new_count = count_new;
        diagnostics->stable_count = count_stable;
        diagnostics->landmark_map_size = static_cast<int>(landmark_map_.size());
        diagnostics->stereo_passed = stereo_pass;
        diagnostics->stereo_rejected = stereo_reject;
        diagnostics->reprojection_passed = reproj_pass;
        diagnostics->reprojection_rejected = reproj_reject;
        diagnostics->unhealthy_streak = health.unhealthy_streak;
        diagnostics->stereo_error_mean =
            stereo_pass > 0 ? stereo_err_sum / stereo_pass : 0.0;
        diagnostics->stereo_error_max = stereo_err_max;
        diagnostics->reprojection_error_mean =
            reproj_pass > 0 ? reproj_err_sum / reproj_pass : 0.0;
        diagnostics->reprojection_error_max = reproj_err_max;
        diagnostics->low_feature_mode = low_feat_mode;
        diagnostics->health_guard_active = health.guard_active;
        diagnostics->health_low_stable = health.low_stable;
        diagnostics->health_low_landmarks = health.low_landmarks;
        diagnostics->visual_update_allowed = health.allow_visual_update;
    }
}

// 检查重投影误差
// 如果返回 false，说明点有问题
bool FeatureManager::check_reprojection(const Eigen::Vector3d& p_w,
                                    const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& p_wb,
                                    const Eigen::Vector3d& uv_meas_norm,
                                    double reproj_thresh,
                                    double* reproj_err) {

    // 1. 转到 Body 系: P_b = R_wb^T * (P_w - p_wb)
    Eigen::Vector3d p_b = R_wb.transpose() * (p_w - p_wb);

    // 2. 转到 Cam 系: P_c = R_bc^T * (P_b - p_bc)
    Eigen::Matrix3d R_bc = T_C_B[0].block<3,3>(0,0);
    Eigen::Vector3d p_bc = T_C_B[0].block<3,1>(0,3);
    Eigen::Vector3d p_c = R_bc.transpose() * (p_b - p_bc);

    // 3. 深度检查 (必须在相机前方)
    if(p_c.z() < 0.2) return false;

    // 4. 投影到归一化平面
    Eigen::Vector2d uv_proj = p_c.head<2>() / p_c.z();
    Eigen::Vector2d uv_meas = uv_meas_norm.head<2>();

    // 5. 计算误差
    double err = (uv_proj - uv_meas).norm();
    if(reproj_err) *reproj_err = err;

    // 自适应阈值（正常 0.08，少点模式 0.10）
    if(err > reproj_thresh) return false;

    return true;
}

const std::map<size_t, Eigen::Vector3d> FeatureManager::get_active_map() const {
    return landmark_map_.activeMap(options_.active_mature_thresh);
}
