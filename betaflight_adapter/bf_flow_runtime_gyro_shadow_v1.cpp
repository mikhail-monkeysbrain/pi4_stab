#include "../src/runtime.hpp"
#include "../src/flow_roi.hpp"
#include "bf_camera_latest.hpp"
#include "bf_ipc_receiver.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cstdint>

#include <deque>
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <stdexcept>

FeatureRoi g_feature_roi{};
int g_max_features = 500;
double g_fb_shadow_max_px = 0.0;
bool g_obs_shadow_enabled = false;

#include "../src/flow_core.hpp"

namespace {

CameraCalib makeCameraCalib() {
    CameraCalib c;

    // OV7251 ChArUco calibration, 2026-11-01.
    c.fx = 615.008145;
    c.fy = 616.200754;
    c.cx = 277.197977;
    c.cy = 276.270723;

    c.K = (cv::Mat_<double>(3,3) <<
        c.fx, 0.0, c.cx,
        0.0, c.fy, c.cy,
        0.0, 0.0, 1.0);

    c.D = (cv::Mat_<double>(1,5) <<
        -0.597585376,
         0.263536183,
        -0.011099252,
         0.025182752,
        -0.091036464);

    // Camera OpenCV -> Body FLU.
    // Optical axis DOWN, image TOP -> FRONT.
    c.B_R_C = cv::Matx33d(
         0, -1,  0,
        -1,  0,  0,
         0,  0, -1
    );

    return c;
}

} // namespace


// Shadow-only comparison from existing RANSAC correspondences.
// This does not modify the production Optical Flow or IPC packet.
static bool gyroShadowFlow(
    const std::vector<cv::Point2f>& p0,
    const std::vector<cv::Point2f>& p1,
    double dt,
    const CameraCalib& calib,
    const cv::Matx33d& C1_R_C0,
    double& out_x,
    double& out_y)
{
    if (p0.size() < 20 || p0.size() != p1.size() || !(dt > 0.0))
        return false;

    std::vector<cv::Point2f> a, b;
    cv::undistortPoints(p0, a, calib.K, calib.D);
    cv::undistortPoints(p1, b, calib.K, calib.D);

    cv::Mat A(static_cast<int>(a.size()) * 2, 4, CV_64F);
    cv::Mat rhs(static_cast<int>(a.size()) * 2, 1, CV_64F);

    for (size_t k = 0; k < a.size(); ++k) {
        const cv::Vec3d q =
            C1_R_C0 * cv::Vec3d(a[k].x, a[k].y, 1.0);

        if (!std::isfinite(q[2]) || q[2] <= 0.1)
            return false;

        const double x = q[0] / q[2];
        const double y = q[1] / q[2];
        const double du = b[k].x - x;
        const double dv = b[k].y - y;
        const int r = static_cast<int>(2 * k);

        A.at<double>(r, 0) = 1.0;
        A.at<double>(r, 1) = 0.0;
        A.at<double>(r, 2) = x;
        A.at<double>(r, 3) = -y;
        rhs.at<double>(r, 0) = du;

        A.at<double>(r+1, 0) = 0.0;
        A.at<double>(r+1, 1) = 1.0;
        A.at<double>(r+1, 2) = y;
        A.at<double>(r+1, 3) = x;
        rhs.at<double>(r+1, 0) = dv;
    }

    cv::Mat sol;
    if (!cv::solve(A, rhs, sol, cv::DECOMP_SVD) || sol.rows != 4)
        return false;

    const double cam_x = sol.at<double>(1, 0) / dt;
    const double cam_y = -sol.at<double>(0, 0) / dt;

    const cv::Matx33d FLU_TO_FRD(
        1,0,0, 0,-1,0, 0,0,-1);
    const cv::Matx33d FRD_R_C = FLU_TO_FRD * calib.B_R_C;
    const cv::Vec3d body =
        FRD_R_C * cv::Vec3d(cam_x, cam_y, 0.0);

    out_x = body[0];
    out_y = body[1];

    return std::isfinite(out_x) &&
           std::isfinite(out_y) &&
           std::hypot(out_x, out_y) < 4.0;
}

int main() {
    try {
        const CameraCalib calib = makeCameraCalib();

        bf_imu::BfIpcReceiver ipc;

        const int flow_fd = socket(
            AF_UNIX,
            SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC,
            0
        );
        if (flow_fd < 0)
            throw std::runtime_error("BF flow IPC socket failed");

        sockaddr_un flow_addr{};
        flow_addr.sun_family = AF_UNIX;
        std::strncpy(
            flow_addr.sun_path,
            "/tmp/jtzero_bf_flow_raw.sock",
            sizeof(flow_addr.sun_path) - 1
        );
        bf_camera::Camera camera;
        camera.open("/dev/video0", 640, 480, 60);

        std::cout << "BF_FLOW_CAMERA_OPEN fps="
                  << camera.actualFps() << std::endl;

        bf_camera::Frame prev, curr;

        if (!camera.read(prev, 1000))
            throw std::runtime_error("First camera frame timeout");

        int frames = 0;
        int valid = 0;
        int invalid = 0;
        int missing = 0;
        int gyro_covered = 0;
        int gyro_uncovered = 0;
        int gyro_delayed_covered = 0;
        int gyro_delayed_uncovered = 0;
        int gyro_rotation_ok = 0;
        int gyro_rotation_fail = 0;
        int gyro_delayed_pending = 0;
        struct PendingGyroInterval {
            int64_t t0;
            int64_t t1;
            int64_t ready_ns;
            std::vector<cv::Point2f> p0;
            std::vector<cv::Point2f> p1;
            double dt;
            double raw_x;
            double raw_y;
            bool valid;
        };
        std::deque<PendingGyroInterval> pending_gyro;
        uint64_t shadow_compared = 0;
        uint64_t shadow_uncovered = 0;
        uint64_t shadow_invalid = 0;

        const int64_t start_ns = monoNs();

        while (true) {
            if (!camera.read(curr, 1000)) {
                missing++;
                continue;
            }

            ipc.poll();
            const int64_t gyro_check_now = monoNs();
            while (!pending_gyro.empty() &&
                   pending_gyro.front().ready_ns <= gyro_check_now) {
                const auto& item = pending_gyro.front();
                const auto delayed = ipc.gyroInterval(item.t0, item.t1);
                if (delayed.covered)
                    ++gyro_delayed_covered;
                else {
                    ++gyro_delayed_uncovered;
                    ++shadow_uncovered;
                }

                if (delayed.covered) {
                    cv::Matx33d R_gyro;
                    if (ipc.gyroRotation(
                            item.t0, item.t1,
                            0.001090830782496456, R_gyro)) {

                        // Gyro integration: active body rotation.
                        // Convert to previous-camera -> current-camera.
                        // Body FLU -> body FRD.
                        const cv::Matx33d FLU_TO_FRD(
                            1, 0, 0,
                            0,-1, 0,
                            0, 0,-1
                        );

                        // Camera -> body FRD.
                        const cv::Matx33d FRD_R_C =
                            FLU_TO_FRD * calib.B_R_C;

                        // Previous camera -> current camera.
                        const cv::Matx33d C1_R_C0 =
                            FRD_R_C.t() *
                            R_gyro.t() *
                            FRD_R_C;

                        // Shadow only: no effect on Optical Flow IPC.
                        double shadow_x = 0.0;
                        double shadow_y = 0.0;
                        if (item.valid && gyroShadowFlow(
                                item.p0, item.p1, item.dt,
                                calib, C1_R_C0,
                                shadow_x, shadow_y)) {
                            ++shadow_compared;
                            if (shadow_compared % 30 == 0) {
                                std::cout << "BF_GYRO_SHADOW"
                                          << " t_ns=" << item.t1
                                          << " raw_x=" << item.raw_x
                                          << " raw_y=" << item.raw_y
                                          << " gyro_x=" << shadow_x
                                          << " gyro_y=" << shadow_y
                                          << " delta_x=" << shadow_x-item.raw_x
                                          << " delta_y=" << shadow_y-item.raw_y
                                          << " compared=" << shadow_compared
                                          << " uncovered=" << shadow_uncovered
                                          << " invalid=" << shadow_invalid
                                          << std::endl;
                            }
                        } else {
                            ++shadow_invalid;
                        }

                        ++gyro_rotation_ok;
                    } else {
                        ++gyro_rotation_fail;
                    }
                }
                pending_gyro.pop_front();
            }
            const auto sensors = ipc.snapshot(curr.timestamp_ns);

            const double dt =
                (curr.timestamp_ns - prev.timestamp_ns) * 1e-9;

            const auto gyro_interval = ipc.gyroInterval(
                prev.timestamp_ns,
                curr.timestamp_ns
            );

            if (gyro_interval.covered)
                ++gyro_covered;
            else
                ++gyro_uncovered;
            const FlowStep flow = estimateRawFlow(
                prev.gray,
                curr.gray,
                dt,
                calib
            );

            pending_gyro.push_back({
                prev.timestamp_ns,
                curr.timestamp_ns,
                monoNs() + 50000000LL,
                flow.metric_prev_points,
                flow.metric_curr_points,
                dt,
                flow.flow_body_x,
                flow.flow_body_y,
                flow.valid
            });

            frames++;

            if (flow.valid) {
                valid++;

                // Machine-readable Core output.
                // No MSP transmission from this runtime.
                std::cout
                    << std::setprecision(12)
                    << "BF_CORE_SAMPLE"
                    << " t_ns=" << curr.timestamp_ns
                    << " dt_s=" << dt
                    << " rate_x=" << flow.flow_body_x
                    << " rate_y=" << flow.flow_body_y
                    << " inliers=" << flow.inliers
                    << std::endl;

                // IPC v3: <BQdddB, 34 bytes.
                // Timestamp, frame dt, rates, quality.
                // No MSP Optical Flow transmission.
                uint8_t packet[34]{};
                packet[0] = 3;

                const uint64_t ts =
                    static_cast<uint64_t>(curr.timestamp_ns);

                const double rx = flow.flow_body_x;
                const double ry = flow.flow_body_y;

                std::memcpy(packet + 1, &ts, 8);
                std::memcpy(packet + 9, &dt, 8);
                std::memcpy(packet + 17, &rx, 8);
                std::memcpy(packet + 25, &ry, 8);

                packet[33] = 255;

                // GYRO SHADOW V1: IPC transmission disabled.
                // Never inject experimental samples into production.
                (void)packet;
            } else {
                invalid++;
            }

            if (frames % 30 == 0) {
                std::cout
                    << "BF_FLOW"
                    << " frames=" << frames
                    << " valid=" << valid
                    << " invalid=" << invalid
                    << " missing=" << missing
                    << " dt_ms=" << dt * 1000.0
                    << " features=" << flow.features
                    << " tracked=" << flow.tracked
                    << " inliers=" << flow.inliers
                    << " reason=" << flow.invalid_reason
                    << " du_px=" << flow.du_px
                    << " dv_px=" << flow.dv_px
                    << " flow_x=" << flow.flow_body_x
                    << " flow_y=" << flow.flow_body_y
                    << " gyro_covered=" << gyro_interval.covered
                    << " gyro_samples=" << gyro_interval.samples
                    << " gyro_gap_ms=" << gyro_interval.max_gap_ms
                    << " gyro_left_ms=" << gyro_interval.left_margin_ms
                    << " gyro_right_ms=" << gyro_interval.right_margin_ms
                    << " gyro_first_offset_ms="
                    << gyro_interval.first_offset_ms
                    << " gyro_last_offset_ms="
                    << gyro_interval.last_offset_ms
                    << " gyro_history_size="
                    << ipc.gyroHistorySize()
                    << " imu_fresh=" << sensors.imu_fresh
                    << " att_fresh=" << sensors.attitude_fresh
                    << " imu_age_ms="
                    << (sensors.imu.valid
                        ? (curr.timestamp_ns - sensors.imu.recv_ns) * 1e-6
                        : -1.0)
                    << " att_age_ms="
                    << (sensors.attitude.valid
                        ? (curr.timestamp_ns - sensors.attitude.recv_ns) * 1e-6
                        : -1.0)
                    << " gyro_x=" << sensors.imu.gyro[0]
                    << " gyro_y=" << sensors.imu.gyro[1]
                    << " gyro_z=" << sensors.imu.gyro[2]
                    << " roll=" << sensors.attitude.roll
                    << " pitch=" << sensors.attitude.pitch
                    << " yaw=" << sensors.attitude.yaw
                    << " lk_ms=" << flow.t_lk_ms
                    << " ransac_ms=" << flow.t_ransac_ms
                    << std::endl;
            }

            prev = std::move(curr);
        }

        std::cout
            << "BF_FLOW_RESULT"
            << " gyro_covered=" << gyro_covered
            << " gyro_uncovered=" << gyro_uncovered
            << " gyro_delayed_covered=" << gyro_delayed_covered
            << " gyro_delayed_uncovered=" << gyro_delayed_uncovered
            << " gyro_rotation_ok=" << gyro_rotation_ok
            << " gyro_rotation_fail=" << gyro_rotation_fail
            << " gyro_delayed_pending=" << pending_gyro.size()
            << " frames=" << frames
            << " valid=" << valid
            << " invalid=" << invalid
            << " missing=" << missing
            << " imu_packets=" << ipc.imuCount()
            << " att_packets=" << ipc.attCount()
            << " ipc_invalid=" << ipc.invalidCount()
            << " ipc_errors=" << ipc.errorCount()
            << std::endl;

        close(flow_fd);
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "BF_FLOW_ERROR " << e.what() << std::endl;
        return 1;
    }
}
