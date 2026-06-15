#include "HNOFeature.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unordered_set>

using namespace ov_hno;

HNOFeature::HNOFeature(std::vector<std::shared_ptr<ov_core::CamBase>> cams,
                       std::vector<Eigen::Matrix4d> extrinsics) 
    : cameras(cams), T_C_B(extrinsics) {
    
    std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> cam_map;
    for(size_t i=0; i<cams.size(); ++i) cam_map[i] = cams[i];
    
    // 增加特征点数，利用后端筛选
    tracker = std::make_shared<ov_core::TrackKLT>(cam_map, 200, 0, true, 
              ov_core::TrackBase::HistogramMethod::HISTOGRAM, 20, 5, 5, 10);
}

void HNOFeature::feed_measurement(const ov_core::CameraData& message, 
                                  Eigen::Matrix3d R_wb, Eigen::Vector3d p_wb,
                                  std::vector<HNOObservation>& observations) {
    
    tracker->feed_new_camera(message);
    
    auto obs_raw = tracker->get_last_obs();
    auto ids_raw = tracker->get_last_ids();
    
    if(!obs_raw.count(0)) return; 

    // --- 0. 视差检测 (仅用于 RANSAC 策略，不用于阻止三角化) ---
    // 计算平均光流
    double total_flow = 0.0;
    int flow_cnt = 0;
    if (!history_obs.empty()) {
        for(size_t i=0; i<ids_raw[0].size(); ++i) {
            size_t id = ids_raw[0][i];
            if(history_obs.count(id)) {
                cv::Point2f p_prev = history_obs[id];
                cv::Point2f p_curr = obs_raw[0][i].pt;
                double dx = p_prev.x - p_curr.x;
                double dy = p_prev.y - p_curr.y;
                total_flow += std::sqrt(dx*dx + dy*dy);
                flow_cnt++;
            }
        }
    }
    double avg_flow = (flow_cnt > 0) ? total_flow / flow_cnt : 0.0;
    // 如果光流很小，说明机体没怎么动
    bool is_stationary = (flow_cnt > 10 && avg_flow < 0.5);

    // --- 1. 前端几何 RANSAC (2D-2D) ---
    // 只有在运动幅度够大时才做 Fundamental Matrix RANSAC，否则计算不稳定
    std::unordered_set<size_t> outlier_ids;
    if (!history_obs.empty() && !is_stationary) {
        std::vector<cv::Point2f> pts_prev, pts_curr;
        std::vector<size_t> pts_ids;
        for(size_t i=0; i<ids_raw[0].size(); ++i) {
            size_t id = ids_raw[0][i];
            if(history_obs.count(id)) {
                pts_prev.push_back(history_obs[id]);
                pts_curr.push_back(obs_raw[0][i].pt);
                pts_ids.push_back(id);
            }
        }
        
        if(pts_prev.size() >= 15) {
             std::vector<uchar> status;
             // 阈值放宽到 2.0 像素
             cv::findFundamentalMat(pts_prev, pts_curr, cv::FM_RANSAC, 2.0, 0.99, status);
             for(size_t k=0; k<status.size(); ++k) {
                 if(!status[k]) outlier_ids.insert(pts_ids[k]);
             }
        }
    }

    // --- 2. 准备数据 ---
    observations.clear();
    std::vector<size_t> current_frame_ids;
    std::map<size_t, cv::Point2f> next_history_obs;

    std::map<size_t, int> right_cam_idx;
    if(obs_raw.count(1)) {
        for(size_t i=0; i<ids_raw[1].size(); ++i) {
            right_cam_idx[ids_raw[1][i]] = i;
        }
    }

    // 统计变量 (Debug用)
    int count_tracked = 0;
    int count_new_triangulated = 0;
    int count_tri_failed = 0;

    // --- 3. 遍历特征点 ---
    size_t num_pts = obs_raw[0].size();
    for(size_t i=0; i<num_pts; ++i) {
        size_t id = ids_raw[0][i];
        
        if(outlier_ids.count(id)) continue; // 跳过 RANSAC 外点

        current_frame_ids.push_back(id);
        
        cv::Point2f pt_l = obs_raw[0][i].pt;
        next_history_obs[id] = pt_l; 

        Eigen::Vector2d uv_l_px(pt_l.x, pt_l.y);
        Eigen::Vector2d uv_l_norm = cameras[0]->undistort_d(uv_l_px); 
        Eigen::Vector3d uv_l_vec(uv_l_norm.x(), uv_l_norm.y(), 1.0); 

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

        // ==========================================
        // 核心分支
        // ==========================================

        // Case A: 地图里的老点 -> 用于更新
        if(feature_map.count(id)) {
            HNOObservation obs;
            obs.uv_left = uv_l_vec;
            obs.uv_right = uv_r_vec;
            obs.isValidRight = has_right;
            obs.xyz = feature_map[id]; 
            observations.push_back(obs);
            count_tracked++;
        }
        // Case B: 新点 (且双目可见) -> 尝试三角化建图
        else if(has_right) {
            // 删除了 !is_stationary 的检查！
            // 双目只要有视差，静止也能建图。
            
            Eigen::Vector3d p_c_left;
            if(triangulate_stereo(uv_l_vec, uv_r_vec, p_c_left)) {
                // 转到世界系
                Eigen::Matrix3d R_bc = T_C_B[0].block<3,3>(0,0);
                Eigen::Vector3d p_bc = T_C_B[0].block<3,1>(0,3);
                
                Eigen::Vector3d p_body = R_bc * p_c_left + p_bc;
                Eigen::Vector3d p_world = R_wb * p_body + p_wb;
                
                feature_map[id] = p_world;
                count_new_triangulated++;
            } else {
                count_tri_failed++;
            }
        }
    } 

    // 4. 地图维护
    for (auto it = feature_map.begin(); it != feature_map.end(); ) {
        bool is_active = false;
        for(size_t id : current_frame_ids) {
            if(id == it->first) {
                is_active = true;
                break;
            }
        }
        if (!is_active) it = feature_map.erase(it);
        else ++it;
    }
    
    history_obs = next_history_obs;

    // [调试打印] 如果观测点很少，打印原因
    if (observations.size() < 10) {
        printf("[HNOFeat] Warn: Low Obs! Tracked:%d, NewTri:%d, FailTri:%d, TotalMap:%lu\n", 
               count_tracked, count_new_triangulated, count_tri_failed, feature_map.size());
    }
}


bool HNOFeature::triangulate_stereo(const Eigen::Vector3d& uv_left, 
                                   const Eigen::Vector3d& uv_right, 
                                   Eigen::Vector3d& p_c_left) {
    
    Eigen::Matrix4d T_Right_Left = T_C_B[1].inverse() * T_C_B[0];
    Eigen::Matrix3d R = T_Right_Left.block<3,3>(0,0);
    Eigen::Vector3d t = T_Right_Left.block<3,1>(0,3);

    Eigen::Vector3d f_l = uv_left.normalized();
    Eigen::Vector3d f_r = uv_right.normalized();

    // 简化版几何解法 (比 SVD 快且对双目足够)
    // 求解 depth 使得 || P_r - (R * P_l + t) || 最小
    
    Eigen::Matrix<double, 3, 2> J;
    J.col(0) = uv_left; 
    J.col(1) = -(R.transpose() * uv_right); 
    Eigen::Vector3d p_rel = -(R.transpose() * t); 

    Eigen::Vector2d z = J.colPivHouseholderQr().solve(p_rel);
    double depth = z(0);

    // 放宽深度下限和上限
    if (depth < 0.1 || depth > 80.0) return false; 

    p_c_left = uv_left * depth;
    
    // 重投影检查
    Eigen::Vector3d P_R = R * p_c_left + t;
    if (P_R(2) < 0.1) return false; // Z必须为正
    
    Eigen::Vector3d uv_r_proj = P_R / P_R(2);
    double err = (uv_r_proj.head<2>() - uv_right.head<2>()).norm();

    // 放宽重投影阈值
    // 0.01 约等于 4-5 像素，给噪声一点空间
    if (err > 0.02) return false; 

    return true;
}

const std::map<size_t, Eigen::Vector3d> HNOFeature::get_active_map() const {
    return feature_map;
}
