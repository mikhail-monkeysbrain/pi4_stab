#include "../src/runtime.hpp"
#include "../src/flow_roi.hpp"
#include "bf_camera.hpp"
#include "bf_ipc_receiver.hpp"

#include <deque>
#include <iostream>
#include <iomanip>
#include <stdexcept>

FeatureRoi g_feature_roi{};
int g_max_features = 500;
double g_fb_shadow_max_px = 0.0;
bool g_obs_shadow_enabled = false;

#include "bf_flow_core.hpp"

namespace {

CameraCalib makeCameraCalib() {
    CameraCalib c;

    c.fx = 606.1389752628015;
    c.fy = 603.5480010781832;
    c.cx = 300.60888753736833;
    c.cy = 253.63245690081897;

    c.K = (cv::Mat_<double>(3,3) <<
        c.fx, 0.0, c.cx,
        0.0, c.fy, c.cy,
        0.0, 0.0, 1.0);

    c.D = (cv::Mat_<double>(1,5) <<
        -0.5372209631565794,
         0.373528231991675,
        -0.0013189332134377475,
         0.004816473341320975,
        -0.1459802845289353);

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

int main() {
    try {
        const CameraCalib calib = makeCameraCalib();

        bf_imu::BfIpcReceiver ipc;
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
            cv::Mat prev_gray;
            cv::Mat curr_gray;
        };

        std::deque<PendingGyroInterval> pending_gyro;

        const int64_t start_ns = monoNs();

        // Camera -> body FRD.
        const cv::Matx33d FLU_TO_FRD(
            1, 0, 0,
            0,-1, 0,
            0, 0,-1
        );

        const cv::Matx33d FRD_R_C =
            FLU_TO_FRD * calib.B_R_C;

        // Process saved pairs after the 50 ms IMU waiting period.
        auto drainPending = [&](bool flush) {
            ipc.poll();

            while (!pending_gyro.empty() &&
                   (flush ||
                    pending_gyro.front().ready_ns <= monoNs())) {

                auto item = std::move(pending_gyro.front());
                pending_gyro.pop_front();

                const auto delayed =
                    ipc.gyroInterval(item.t0, item.t1);

                if (delayed.covered)
                    ++gyro_delayed_covered;
                else
                    ++gyro_delayed_uncovered;

                cv::Matx33d rotation = cv::Matx33d::eye();
                bool rotation_valid = false;

                if (delayed.covered) {
                    cv::Matx33d R_gyro;

                    if (ipc.gyroRotation(
                            item.t0,
                            item.t1,
                            0.001090830782496456,
                            R_gyro)) {

                        const cv::Matx33d C1_R_C0 =
                            FRD_R_C.t() *
                            R_gyro.t() *
                            FRD_R_C;

                        // SHADOW ONLY.
                        // Not passed to Core until MSP
                        // axes, scale and timing are verified.
                        rotation = C1_R_C0;
                        rotation_valid = true;

                        ++gyro_rotation_ok;
                    } else {
                        ++gyro_rotation_fail;
                    }
                }

                const double dt =
                    (item.t1 - item.t0) * 1e-9;

                const auto sensors =
                    ipc.snapshot(item.t1);

                const auto gyro_interval = delayed;

                const FlowStep flow = estimateRawFlow(
                    item.prev_gray,
                    item.curr_gray,
                    dt,
                    calib
                );

                const FlowStep flow_rot =
                    rotation_valid
                    ? estimateRawFlow(
                        item.prev_gray,
                        item.curr_gray,
                        dt,
                        calib,
                        0.0,
                        0.0,
                        &rotation)
                    : flow;

                ++frames;

                if (flow.valid)
                    ++valid;
                else
                    ++invalid;

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
                        << " rot_valid=" << rotation_valid
                        << " rot_flow_x=" << flow_rot.flow_body_x
                        << " rot_flow_y=" << flow_rot.flow_body_y
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
                            ? (item.t1 - sensors.imu.recv_ns) * 1e-6
                            : -1.0)
                        << " att_age_ms="
                        << (sensors.attitude.valid
                            ? (item.t1 - sensors.attitude.recv_ns) * 1e-6
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

            }
        };

        while ((monoNs() - start_ns) < 10000000000LL) {
            if (!camera.read(curr, 1000)) {
                ++missing;
                drainPending(false);
                continue;
            }

            ipc.poll();

            const auto immediate = ipc.gyroInterval(
                prev.timestamp_ns,
                curr.timestamp_ns
            );

            if (immediate.covered)
                ++gyro_covered;
            else
                ++gyro_uncovered;

            // Save independent images for delayed processing.
            pending_gyro.push_back({
                prev.timestamp_ns,
                curr.timestamp_ns,
                monoNs() + 50000000LL,
                prev.gray.clone(),
                curr.gray.clone()
            });

            prev = std::move(curr);

            drainPending(false);
        }

        // Process remaining saved frame pairs.
        drainPending(true);

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

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "BF_FLOW_ERROR " << e.what() << std::endl;
        return 1;
    }
}
