#define JTZERO_OPTFLOW_LIBRARY
#include "../src/optical_flow_mavlink.cpp"
#include "../src/camera_csi.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

int main() {
    try {
        const std::string yaml =
            "config/ov5647_current_mount.yaml";

        CameraCalib calib = loadCameraCalib(yaml);

        g_feature_roi.x0 = 0.20;
        g_feature_roi.y0 = 0.32;
        g_feature_roi.x1 = 0.80;
        g_feature_roi.y1 = 0.90;
        g_max_features = 500;

        // Production shadow diagnostics are disabled only
        // inside this standalone process.
        g_fb_shadow_max_px = 0.0;
        g_obs_shadow_enabled = false;

        jtzero_csi::CameraCSI camera;
        camera.start();

        std::ofstream csv("/tmp/ov5647_lk_30s.csv");
        if (!csv)
            throw std::runtime_error("Cannot open CSV");

        csv << "frame,sensor_ns,callback_ns,dt_ms,"
               "frame_age_ms,valid,invalid_reason,"
               "features,tracked,inliers,inlier_ratio,"
               "t_features_ms,t_lk_ms,t_ransac_ms,"
               "t_post_ms,du_px,dv_px,du_norm,dv_norm\n";

        cv::Mat prev;
        int64_t prev_ts = 0;

        uint64_t frames = 0;
        uint64_t valid = 0;
        uint64_t invalid = 0;
        uint64_t dt_rejects = 0;
        double max_lk_ms = 0.0;
        double sum_lk_ms = 0.0;

        const int64_t start = jtzero_csi::monoNs();
        const int64_t finish = start + 30000000000LL;

        while (jtzero_csi::monoNs() < finish) {
            jtzero_csi::Frame frame;

            if (!camera.newest(frame)) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1));
                continue;
            }

            if (frame.gray.empty())
                continue;

            const int64_t now = jtzero_csi::monoNs();
            const double age_ms =
                (now - frame.sensor_ns) * 1e-6;

            if (prev.empty()) {
                prev = frame.gray;
                prev_ts = frame.sensor_ns;
                continue;
            }

            const double dt =
                (frame.sensor_ns - prev_ts) * 1e-9;

            FlowStep s = estimateRawFlow(
                prev, frame.gray, dt, calib);

            ++frames;

            if (s.valid)
                ++valid;
            else
                ++invalid;

            if (s.invalid_reason == 1)
                ++dt_rejects;

            sum_lk_ms += s.t_lk_ms;
            max_lk_ms =
                std::max(max_lk_ms, s.t_lk_ms);

            csv << frames << ','
                << frame.sensor_ns << ','
                << frame.callback_ns << ','
                << dt * 1000.0 << ','
                << age_ms << ','
                << (s.valid ? 1 : 0) << ','
                << s.invalid_reason << ','
                << s.features << ','
                << s.tracked << ','
                << s.inliers << ','
                << s.inlier_ratio << ','
                << s.t_features_ms << ','
                << s.t_lk_ms << ','
                << s.t_ransac_ms << ','
                << s.t_post_ms << ','
                << s.du_px << ','
                << s.dv_px << ','
                << s.du_norm << ','
                << s.dv_norm << '\n';

            prev = frame.gray;
            prev_ts = frame.sensor_ns;
        }

        csv.flush();
        csv.close();

        const auto stats = camera.stats();
        camera.stop();

        std::cout << "===== OV5647 PRODUCTION LK =====\n";
        std::cout << "frames=" << frames << "\n";
        std::cout << "valid=" << valid << "\n";
        std::cout << "invalid=" << invalid << "\n";
        std::cout << "valid_ratio="
                  << (frames ?
                      double(valid) / frames : 0.0)
                  << "\n";
        std::cout << "dt_rejects="
                  << dt_rejects << "\n";
        std::cout << "mean_lk_ms="
                  << (frames ?
                      sum_lk_ms / frames : 0.0)
                  << "\n";
        std::cout << "max_lk_ms="
                  << max_lk_ms << "\n";
        std::cout << "camera_captured="
                  << stats.captured << "\n";
        std::cout << "camera_skipped="
                  << stats.skipped << "\n";
        std::cout << "camera_errors="
                  << stats.errors << "\n";
        std::cout << "csv=/tmp/ov5647_lk_30s.csv\n";

        return 0;
    } catch (const std::exception &e) {
        std::cerr << "RESULT FAIL: "
                  << e.what() << "\n";
        return 1;
    }
}
