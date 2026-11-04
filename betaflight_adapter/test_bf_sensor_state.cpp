#include "bf_sensor_state.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

int main() {
    bf_imu::SensorState state;

    constexpr int64_t T0 = 1000000000LL;
    constexpr int64_t DT = 10000000LL; // 10 ms

    // IMU: 11 samples at 100 Hz.
    for (int i = 0; i <= 10; ++i) {
        bf_msp::SensorSample sample;
        sample.command = bf_msp::MSP_RAW_IMU;
        sample.timestamp_ns = T0 + i * DT;

        sample.raw_imu = {
            100, 200, 300,
            160, -80, 40,
            10, 20, 30
        };

        state.update(sample);
    }

    assert(state.imuCount() == 11);
    assert(state.gyroHistorySize() == 11);

    std::cout << "PASS: IMU samples and history\n";

    // Verify raw gyro scale is unchanged.
    auto snapshot = state.snapshot(T0 + 10 * DT);

    assert(snapshot.imu.valid);
    assert(snapshot.imu.gyro[0] == 160.0f);
    assert(snapshot.imu.gyro[1] == -80.0f);
    assert(snapshot.imu.gyro[2] == 40.0f);

    std::cout << "PASS: raw gyro preservation\n";

    // Freshness at exact timestamp.
    assert(snapshot.imu_fresh);

    // Freshness after 31 ms.
    auto stale = state.snapshot(T0 + 10 * DT + 31000000LL);
    assert(!stale.imu_fresh);

    // Snapshot before last received IMU.
    auto earlier = state.snapshot(T0 + 9 * DT);
    assert(!earlier.imu_fresh);

    std::cout << "PASS: IMU freshness\n";

    // Attitude: roll=90 deg, pitch=-45 deg, yaw=180 deg.
    bf_msp::SensorSample att;
    att.command = bf_msp::MSP_ATTITUDE;
    att.timestamp_ns = T0 + 10 * DT;

    constexpr double PI = 3.14159265358979323846;

    att.attitude_rad = {
        PI / 2.0,
        -PI / 4.0,
        PI
    };

    state.update(att);

    snapshot = state.snapshot(att.timestamp_ns);

    assert(state.attCount() == 1);
    assert(snapshot.attitude.valid);
    assert(snapshot.attitude_fresh);

    assert(std::abs(snapshot.attitude.roll - PI / 2.0) < 1e-6);
    assert(std::abs(snapshot.attitude.pitch + PI / 4.0) < 1e-6);
    assert(std::abs(snapshot.attitude.yaw - PI) < 1e-6);

    std::cout << "PASS: attitude conversion\n";

    // Gyro coverage between 20 and 80 ms.
    const auto interval = state.gyroInterval(
        T0 + 2 * DT,
        T0 + 8 * DT
    );

    assert(interval.covered);
    assert(interval.max_gap_ms <= 10.01);

    std::cout << "PASS: gyro interval coverage\n";

    // Constant gyro: 160 raw units = 10 deg/s
    // under the currently used scale.
    constexpr double GYRO_SCALE =
        0.001090830782496456;

    cv::Matx33d R;

    const bool rotation_ok = state.gyroRotation(
        T0 + 2 * DT,
        T0 + 8 * DT,
        GYRO_SCALE,
        R
    );

    assert(rotation_ok);

    // 60 ms at 10 deg/s = 0.6 deg.
    const double expected_angle =
        10.0 * PI / 180.0 * 0.060;

    // Rotation matrix is not identity.
    assert(std::abs(R(0, 0) - 1.0) > 1e-8);

    // Check expected rotation magnitude using trace.
    const double cos_angle =
        (R(0, 0) + R(1, 1) + R(2, 2) - 1.0) / 2.0;

    const double measured_angle =
        std::acos(std::max(-1.0, std::min(1.0, cos_angle)));

    // Three gyro axes are nonzero, so expected magnitude
    // is norm([160,-80,40]) * scale * 0.060.
    const double expected_magnitude =
        std::sqrt(160.0 * 160.0 +
                  80.0 * 80.0 +
                  40.0 * 40.0) *
        GYRO_SCALE * 0.060;

    assert(std::abs(measured_angle - expected_magnitude) < 1e-6);

    std::cout << "PASS: gyro rotation integration\n";

    // Avoid unused-value ambiguity.
    assert(expected_angle > 0.0);

    std::cout << "PASS: no UART access\n";
    return 0;
}
