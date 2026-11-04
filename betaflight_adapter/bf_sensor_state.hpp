#pragma once

#include "bf_imu_adapter.hpp"
#include "bf_imu_history.hpp"
#include "bf_msp_reader.hpp"

#include <array>
#include <cstdint>

namespace bf_imu {

class SensorState {
public:
    void update(const bf_msp::SensorSample& sample) {
        if (sample.command == bf_msp::MSP_RAW_IMU) {
            std::array<float, 9> values{};

            for (int i = 0; i < 9; ++i)
                values[i] = static_cast<float>(sample.raw_imu[i]);

            adapter_.updateRawImu(values, sample.timestamp_ns);

            history_.push(
                sample.timestamp_ns,
                {values[3], values[4], values[5]}
            );

            ++imu_count_;
        }
        else if (sample.command == bf_msp::MSP_ATTITUDE) {
            std::array<float, 3> values{
                static_cast<float>(sample.attitude_rad[0]),
                static_cast<float>(sample.attitude_rad[1]),
                static_cast<float>(sample.attitude_rad[2])
            };

            adapter_.updateAttitude(values, sample.timestamp_ns);
            ++att_count_;
        }
    }

    Snapshot snapshot(int64_t now_ns) const {
        return adapter_.snapshot(now_ns);
    }

    GyroInterval gyroInterval(
        int64_t t0,
        int64_t t1
    ) const {
        return history_.inspect(t0, t1);
    }

    bool gyroRotation(
        int64_t t0,
        int64_t t1,
        double scale,
        cv::Matx33d& R
    ) const {
        return history_.integrateRotation(t0, t1, scale, R);
    }

    std::size_t gyroHistorySize() const {
        return history_.size();
    }

    uint64_t imuCount() const {
        return imu_count_;
    }

    uint64_t attCount() const {
        return att_count_;
    }

private:
    Adapter adapter_;
    GyroHistory history_;

    uint64_t imu_count_ = 0;
    uint64_t att_count_ = 0;
};

} // namespace bf_imu
