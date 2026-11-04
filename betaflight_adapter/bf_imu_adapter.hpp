#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace bf_imu {

struct RawImu {
    std::array<float, 3> acc{};
    std::array<float, 3> gyro{};
    std::array<float, 3> mag{};
    int64_t recv_ns = 0;
    bool valid = false;
};

struct Attitude {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    int64_t recv_ns = 0;
    bool valid = false;
};

struct Snapshot {
    RawImu imu;
    Attitude attitude;
    bool imu_fresh = false;
    bool attitude_fresh = false;
};

class Adapter {
public:
    void updateRawImu(
        const std::array<float, 9>& values,
        int64_t timestamp_ns
    ) {
        if (timestamp_ns <= 0)
            return;

        for (float value : values) {
            if (!std::isfinite(value))
                return;
        }

        if (imu_.valid && timestamp_ns <= imu_.recv_ns)
            return;

        for (int i = 0; i < 3; ++i) {
            imu_.acc[i] = values[i];
            imu_.gyro[i] = values[i + 3];
            imu_.mag[i] = values[i + 6];
        }

        imu_.recv_ns = timestamp_ns;
        imu_.valid = true;
    }

    void updateAttitude(
        const std::array<float, 3>& values,
        int64_t timestamp_ns
    ) {
        if (timestamp_ns <= 0)
            return;

        for (float value : values) {
            if (!std::isfinite(value))
                return;
        }

        if (attitude_.valid &&
            timestamp_ns <= attitude_.recv_ns)
            return;

        attitude_.roll = values[0];
        attitude_.pitch = values[1];
        attitude_.yaw = values[2];
        attitude_.recv_ns = timestamp_ns;
        attitude_.valid = true;
    }

    Snapshot snapshot(int64_t now_ns) const {
        Snapshot result;
        result.imu = imu_;
        result.attitude = attitude_;

        result.imu_fresh =
            imu_.valid &&
            now_ns >= imu_.recv_ns &&
            now_ns - imu_.recv_ns <= 30000000LL;

        result.attitude_fresh =
            attitude_.valid &&
            now_ns >= attitude_.recv_ns &&
            now_ns - attitude_.recv_ns <= 150000000LL;

        return result;
    }

private:
    RawImu imu_;
    Attitude attitude_;
};

}  // namespace bf_imu
