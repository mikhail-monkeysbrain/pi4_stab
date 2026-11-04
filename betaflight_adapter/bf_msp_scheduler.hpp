#pragma once

#include <cstdint>

namespace bf_msp {

struct PollDecision {
    bool imu = false;
    bool attitude = false;
};

class PollScheduler {
public:
    static constexpr int64_t IMU_PERIOD_NS = 10000000LL;
    static constexpr int64_t ATT_PERIOD_NS = 50000000LL;

    PollDecision poll(int64_t now_ns) {
        PollDecision result;

        if (now_ns <= 0)
            return result;

        if (!initialized_) {
            initialized_ = true;
            next_imu_ns_ = now_ns;
            next_att_ns_ = now_ns;
        }

        if (now_ns >= next_imu_ns_) {
            result.imu = true;
            next_imu_ns_ = advance(
                next_imu_ns_, IMU_PERIOD_NS, now_ns
            );
        }

        if (now_ns >= next_att_ns_) {
            result.attitude = true;
            next_att_ns_ = advance(
                next_att_ns_, ATT_PERIOD_NS, now_ns
            );
        }

        return result;
    }

    void reset() {
        initialized_ = false;
        next_imu_ns_ = 0;
        next_att_ns_ = 0;
    }

private:
    static int64_t advance(
        int64_t next_ns,
        int64_t period_ns,
        int64_t now_ns
    ) {
        const int64_t missed =
            (now_ns - next_ns) / period_ns + 1;

        return next_ns + missed * period_ns;
    }

    bool initialized_ = false;
    int64_t next_imu_ns_ = 0;
    int64_t next_att_ns_ = 0;
};

} // namespace bf_msp
