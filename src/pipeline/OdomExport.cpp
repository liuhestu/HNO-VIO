#include "hno_vio/pipeline/OdomExport.h"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>

namespace hno_vio::pipeline {

OdomExport::~OdomExport() {
    if (csv_.is_open()) csv_.flush();
    if (tum_.is_open()) tum_.flush();
    if (diagnostics_csv_.is_open()) diagnostics_csv_.flush();
}

bool OdomExport::open(const std::string& csv_path, const RunContext& context) {
    if (csv_path.empty()) return false;
    csv_path_ = csv_path;
    boost::filesystem::path output(csv_path_);
    boost::filesystem::create_directories(output.parent_path());
    boost::filesystem::path tum_path = output;
    tum_path.replace_extension(".txt");
    tum_path_ = tum_path.string();
    diagnostics_csv_path_ =
        (output.parent_path() / "e_diagnostics.csv").string();

    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    tum_.open(tum_path_, std::ios::out | std::ios::trunc);
    diagnostics_csv_.open(diagnostics_csv_path_,
                          std::ios::out | std::ios::trunc);
    if (!csv_.is_open() || !tum_.is_open() || !diagnostics_csv_.is_open()) {
        return false;
    }
    csv_ << "timestamp_ns,tx,ty,tz,qx,qy,qz,qw\n" << std::fixed << std::setprecision(12);
    tum_ << std::fixed << std::setprecision(9);
    diagnostics_csv_
        << "timestamp_ns,frame_index,visual_update_applied,"
        << "structure_projection_enabled,experiment_fix_e_hat,"
        << "experiment_force_sigma_r_zero,"
        << "px,py,pz,vx,vy,vz,qx,qy,qz,qw";
    for (const char* matrix_name :
         {"e_before", "e_raw", "e_projected", "e_committed"}) {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                diagnostics_csv_ << ',' << matrix_name << '_' << row << col;
            }
        }
    }
    diagnostics_csv_
        << ",theta_e_raw_deg,theta_e_committed_deg,epsilon_orth,"
        << "epsilon_det,projection_correction,"
        << "visual_dR_deg,visual_dp_m,visual_dv_mps,visual_dE_fro,"
        << "sigma_r_raw_x,sigma_r_raw_y,sigma_r_raw_z,sigma_r_raw_norm,"
        << "sigma_r_raw_max,sigma_r_raw_integral,"
        << "sigma_r_applied_x,sigma_r_applied_y,sigma_r_applied_z,"
        << "sigma_r_applied_norm,sigma_r_applied_max,"
        << "sigma_r_applied_integral,sigma_sample_count,state_finite\n"
        << std::fixed << std::setprecision(12);

    update_enforce_structure_ = context.update_enforce_structure;
    experiment_fix_e_hat_ = context.experiment_fix_e_hat;
    experiment_force_sigma_r_zero_ =
        context.experiment_force_sigma_r_zero;

    const boost::filesystem::path parent = output.parent_path();
    const boost::filesystem::path run_dir =
        parent.filename().string() == "vio_results" ? parent.parent_path() : parent;
    writeRunContext(context, run_dir.string());
    return true;
}

void OdomExport::write(double timestamp,
                       const State& state,
                       const PipelineDiagnostics& diagnostics) {
    if (!enabled() || (last_timestamp_ > 0.0 && timestamp <= last_timestamp_)) return;
    Eigen::Quaterniond q(state.R_hat_B2I);
    q.normalize();
    const int64_t timestamp_ns = static_cast<int64_t>(std::llround(timestamp * 1e9));
    csv_ << timestamp_ns << ',' << state.p_hat.x() << ',' << state.p_hat.y() << ','
         << state.p_hat.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ','
         << q.w() << '\n';
    tum_ << timestamp << ' ' << state.p_hat.x() << ' ' << state.p_hat.y() << ' '
         << state.p_hat.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
         << q.w() << '\n';

    const Eigen::Matrix3d E_raw = diagnostics.updater.e_raw;
    const Eigen::Matrix3d E_committed = state.eMatrix();
    const Eigen::Matrix3d R_e_raw = State::projectToSO3(E_raw);
    const Eigen::Matrix3d R_e_committed = State::projectToSO3(E_committed);
    const auto rotationAngle = [](const Eigen::Matrix3d& rotation) {
        const double cosine =
            std::clamp(0.5 * (rotation.trace() - 1.0), -1.0, 1.0);
        return std::acos(cosine);
    };
    const double radians_to_degrees = 180.0 / std::acos(-1.0);
    const double theta_e_raw = rotationAngle(R_e_raw) * radians_to_degrees;
    const double theta_e_committed =
        rotationAngle(R_e_committed) * radians_to_degrees;
    const double epsilon_orth =
        (E_raw.transpose() * E_raw - Eigen::Matrix3d::Identity()).norm();
    const double epsilon_det = std::abs(E_raw.determinant() - 1.0);
    const auto writeMatrix = [this](const Eigen::Matrix3d& matrix) {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                diagnostics_csv_ << ',' << matrix(row, col);
            }
        }
    };
    const bool state_finite =
        state.R_hat_B2I.allFinite() && state.p_hat.allFinite() &&
        state.v_hat.allFinite() && E_committed.allFinite() &&
        state.P.allFinite();
    const PropagationDiagnostics& propagation = diagnostics.propagation;
    const UpdaterDiagnostics& updater = diagnostics.updater;

    diagnostics_csv_
        << timestamp_ns << ',' << diagnostics.frame_index << ','
        << (diagnostics.visual_update_applied ? 1 : 0) << ','
        << (update_enforce_structure_ ? 1 : 0) << ','
        << (experiment_fix_e_hat_ ? 1 : 0) << ','
        << (experiment_force_sigma_r_zero_ ? 1 : 0) << ','
        << state.p_hat.x() << ',' << state.p_hat.y() << ','
        << state.p_hat.z() << ',' << state.v_hat.x() << ','
        << state.v_hat.y() << ',' << state.v_hat.z() << ','
        << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
    writeMatrix(updater.e_before);
    writeMatrix(E_raw);
    writeMatrix(updater.e_projected);
    writeMatrix(E_committed);
    diagnostics_csv_
        << ',' << theta_e_raw << ',' << theta_e_committed << ','
        << epsilon_orth << ',' << epsilon_det << ','
        << updater.projection_correction << ','
        << updater.visual_delta_r * radians_to_degrees << ','
        << updater.visual_delta_p << ',' << updater.visual_delta_v << ','
        << updater.visual_delta_e << ','
        << propagation.sigma_r_raw.x() << ','
        << propagation.sigma_r_raw.y() << ','
        << propagation.sigma_r_raw.z() << ','
        << propagation.sigma_r_raw.norm() << ','
        << propagation.sigma_r_raw_max << ','
        << propagation.sigma_r_raw_integral << ','
        << propagation.sigma_r_applied.x() << ','
        << propagation.sigma_r_applied.y() << ','
        << propagation.sigma_r_applied.z() << ','
        << propagation.sigma_r_applied.norm() << ','
        << propagation.sigma_r_applied_max << ','
        << propagation.sigma_r_applied_integral << ','
        << propagation.sample_count << ',' << (state_finite ? 1 : 0)
        << '\n';
    last_timestamp_ = timestamp;
}

std::string OdomExport::escapeJson(const std::string& value) {
    std::ostringstream escaped;
    for (char c : value) {
        if (c == '"' || c == '\\') escaped << '\\';
        escaped << c;
    }
    return escaped.str();
}

void OdomExport::writeRunContext(const RunContext& context,
                                 const std::string& run_directory) const {
    if (run_directory.empty()) return;
    boost::filesystem::create_directories(run_directory);
    std::ofstream out((boost::filesystem::path(run_directory) / "run_context.json").string(),
                      std::ios::out | std::ios::trunc);
    if (!out.is_open()) return;
    out << "{\n"
        << "  \"dataset\": \"" << escapeJson(context.dataset) << "\",\n"
        << "  \"raw_bag\": \"" << escapeJson(context.raw_bag) << "\",\n"
        << "  \"config\": \"" << escapeJson(context.config) << "\",\n"
        << "  \"config_path\": \"" << escapeJson(context.config_path) << "\",\n"
        << "  \"camera_config\": \"" << escapeJson(context.camera_config) << "\",\n"
        << "  \"ground_truth_tum\": \"" << escapeJson(context.ground_truth) << "\",\n"
        << "  \"odom_csv\": \"" << escapeJson(csv_path_) << "\",\n"
        << "  \"odom_tum\": \"" << escapeJson(tum_path_) << "\",\n"
        << "  \"e_diagnostics_csv\": \"" << escapeJson(diagnostics_csv_path_) << "\",\n"
        << "  \"odom_frame\": \"" << escapeJson(context.odom_frame) << "\",\n"
        << "  \"base_frame\": \"" << escapeJson(context.base_frame) << "\",\n"
        << "  \"num_cams\": " << context.num_cams << ",\n"
        << "  \"use_gt_mapping\": " << (context.use_gt_mapping ? "true" : "false") << ",\n"
        << "  \"update_enforce_structure\": "
        << (context.update_enforce_structure ? "true" : "false") << ",\n"
        << "  \"experiment_fix_e_hat\": "
        << (context.experiment_fix_e_hat ? "true" : "false") << ",\n"
        << "  \"experiment_force_sigma_r_zero\": "
        << (context.experiment_force_sigma_r_zero ? "true" : "false") << ",\n"
        << "  \"experiment_max_frames\": " << context.experiment_max_frames << "\n"
        << "}\n";
}

} // namespace hno_vio::pipeline
