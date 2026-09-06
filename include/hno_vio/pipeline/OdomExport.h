#ifndef HNO_VIO_PIPELINE_ODOM_EXPORT_H
#define HNO_VIO_PIPELINE_ODOM_EXPORT_H

#include <fstream>
#include <string>

#include "hno_vio/State.h"

namespace hno_vio::pipeline {

struct RunContext {
    std::string dataset;
    std::string raw_bag;
    std::string config;
    std::string config_path;
    std::string camera_config;
    std::string ground_truth;
    std::string odom_frame;
    std::string base_frame;
    int num_cams = 2;
    bool use_gt_mapping = false;
    bool update_enforce_structure = true;
    bool fix_e_hat = false;
    bool sigma_r_zero = false;
};

class OdomExport {
public:
    OdomExport() = default;
    ~OdomExport();

    bool open(const std::string& csv_path, const RunContext& context);
    void write(double timestamp, const State& state);
    bool enabled() const {
        return csv_.is_open() && tum_.is_open();
    }

private:
    void writeRunContext(const RunContext& context, const std::string& run_directory) const;
    static std::string escapeJson(const std::string& value);

    std::ofstream csv_;
    std::ofstream tum_;
    std::string csv_path_;
    std::string tum_path_;
    double last_timestamp_ = -1.0;
};

} // namespace hno_vio::pipeline

#endif
