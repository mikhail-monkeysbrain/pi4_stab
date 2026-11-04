#pragma once

#include "bf_flow_msp_aggregator.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace bf_flow {

class MspTxScheduler {
public:
    struct Packet {
        int32_t x = 0;
        int32_t y = 0;
        uint8_t quality = 255;
        int64_t interval_ns = 0;
        uint32_t windows = 0;
    };

    void reset() {
        synchronized_ = false;
        pending_x_ = 0;
        pending_y_ = 0;
        pending_windows_ = 0;
        interval_start_ns_ = 0;
        interval_end_ns_ = 0;
    }

    bool add(const MspAggregator::Result& r) {
        if (!r.valid ||
            r.end_ns <= r.start_ns ||
            r.coverage_ns <
                MspAggregator::PERIOD_NS * 98 / 100) {
            reset();
            return false;
        }

        if (!synchronized_) {
            interval_start_ns_ = r.start_ns;
            interval_end_ns_ = r.end_ns;
            pending_x_ = r.x;
            pending_y_ = r.y;
            pending_windows_ = 1;
            synchronized_ = true;
            return true;
        }

        if (r.start_ns != interval_end_ns_) {
            reset();
            return false;
        }

        pending_x_ += r.x;
        pending_y_ += r.y;
        interval_end_ns_ = r.end_ns;
        ++pending_windows_;

        return true;
    }

    bool build(Packet& packet) {
        if (!synchronized_ || pending_windows_ == 0)
            return false;

        if (pending_x_ > std::numeric_limits<int32_t>::max() ||
            pending_x_ < std::numeric_limits<int32_t>::min() ||
            pending_y_ > std::numeric_limits<int32_t>::max() ||
            pending_y_ < std::numeric_limits<int32_t>::min()) {
            reset();
            return false;
        }

        packet.x = static_cast<int32_t>(pending_x_);
        packet.y = static_cast<int32_t>(pending_y_);
        packet.interval_ns =
            interval_end_ns_ - interval_start_ns_;
        packet.windows = pending_windows_;
        packet.quality = 255;

        pending_x_ = 0;
        pending_y_ = 0;
        pending_windows_ = 0;
        interval_start_ns_ = interval_end_ns_;

        return true;
    }

private:
    bool synchronized_ = false;

    int64_t pending_x_ = 0;
    int64_t pending_y_ = 0;

    int64_t interval_start_ns_ = 0;
    int64_t interval_end_ns_ = 0;

    uint32_t pending_windows_ = 0;
};

}
