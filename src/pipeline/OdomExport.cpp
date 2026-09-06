#include "hno_vio/pipeline/OdomExport.h"

#include <boost/filesystem.hpp>

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>

namespace hno_vio::pipeline {

OdomExport::~OdomExport() {
    if (csv_.is_open()) csv_.flush();
    if (tum_.is_open()) tum_.flush();
}

bool OdomExport::open(const std::string& csv_path, const RunContext& context) {
    if (csv_path.empty()) return false;
    csv_path_ = csv_path;
    boost::filesystem::path output(csv_path_);
    boost::filesystem::create_directories(output.parent_path());
    boost::filesystem::path tum_path = output;
    tum_path.replace_extension(".txt");
    tum_path_ = tum_path.string();

    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    tum_.open(tum_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open() || !tum_.is_open()) {
        return false;
    }
    csv_ << "timestamp_ns,tx,ty,tz,qx,qy,qz,qw\n" << std::fixed << std::setprecision(12);
    tum_ << std::fixed << std::setprecision(9);

    const boost::filesystem::path parent = output.parent_path();
    const boost::filesystem::path run_dir =
        parent.filename().string() == "vio_results" ? parent.parent_path() : parent;
    writeRunContext(context, run_dir.string());
    return true;
}

void OdomExport::write(double timestamp, const State& state) {
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
        << "  \"odom_frame\": \"" << escapeJson(context.odom_frame) << "\",\n"
        << "  \"base_frame\": \"" << escapeJson(context.base_frame) << "\",\n"
        << "  \"num_cams\": " << context.num_cams << ",\n"
        << "  \"use_gt_mapping\": " << (context.use_gt_mapping ? "true" : "false") << ",\n"
        << "  \"update_enforce_structure\": "
        << (context.update_enforce_structure ? "true" : "false") << ",\n"
        << "  \"fix_e_hat\": "
        << (context.fix_e_hat ? "true" : "false") << ",\n"
        << "  \"sigma_r_zero\": "
        << (context.sigma_r_zero ? "true" : "false") << "\n"
        << "}\n";
}

} // namespace hno_vio::pipeline
