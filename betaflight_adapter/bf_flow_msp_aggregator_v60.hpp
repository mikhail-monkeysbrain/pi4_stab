#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>

namespace bf_flow {

class MspAggregator {
public:
    static constexpr int64_t PERIOD_NS = 20000000LL;
    static constexpr int64_t MAX_GAP_NS = 8000000LL;
    static constexpr int64_t LATENCY_NS = 40000000LL;
    static constexpr double MIN_COVERAGE = 0.98;

    struct Result {
        int64_t start_ns = 0;
        int64_t end_ns = 0;
        int64_t coverage_ns = 0;
        int32_t x = 0;
        int32_t y = 0;
        bool valid = false;
    };

    void reset() {
        initialized_ = false;
        residual_x_ = 0;
        residual_y_ = 0;
        last_input_end_ns_ = 0;
        ready_.clear();
        clearWindow();
    }

    void begin(int64_t start_ns) {
        reset();
        start_ns_ = start_ns;
        end_ns_ = start_ns + PERIOD_NS;
        initialized_ = true;
    }

    bool add(double rx, double ry, double dt, int64_t ts_ns) {
        if (!initialized_ ||
            !std::isfinite(rx) ||
            !std::isfinite(ry) ||
            !std::isfinite(dt) ||
            dt <= 0 || dt > 0.1 ||
            ts_ns <= 0) {
            reset();
            return false;
        }

        const int64_t duration =
            static_cast<int64_t>(std::llround(dt * 1e9));

        const int64_t t0 = ts_ns - duration;
        const int64_t t1 = ts_ns;

        if (t1 <= t0) {
            reset();
            return false;
        }

        if (last_input_end_ns_ != 0) {
            const int64_t gap = t0 - last_input_end_ns_;

            if (gap < 0 || gap > MAX_GAP_NS) {
                reset();
                return false;
            }
        }

        last_input_end_ns_ = t1;

        int64_t cursor = t0;

        while (cursor < t1) {
            if (cursor >= end_ns_)
                advanceWindow();

            if (cursor < start_ns_) {
                cursor = std::min(t1, start_ns_);
                continue;
            }

            const int64_t segment_end =
                std::min(t1, end_ns_);

            const int64_t segment_ns =
                segment_end - cursor;

            if (segment_ns <= 0)
                break;

            angle_x_ += rx * (segment_ns * 1e-9);
            angle_y_ += ry * (segment_ns * 1e-9);
            coverage_ns_ += segment_ns;

            cursor = segment_end;
        }

        return true;
    }

    void flush(int64_t now_ns) {
        if (!initialized_)
            return;

        while (now_ns >= end_ns_ + LATENCY_NS)
            advanceWindow();
    }

    bool take(Result& result) {
        if (ready_.empty())
            return false;

        result = ready_.front();
        ready_.pop_front();
        return true;
    }

private:
    void clearWindow() {
        angle_x_ = 0;
        angle_y_ = 0;
        coverage_ns_ = 0;
    }

    void advanceWindow() {
        Result result;
        result.start_ns = start_ns_;
        result.end_ns = end_ns_;
        result.coverage_ns = coverage_ns_;

        const double coverage =
            static_cast<double>(coverage_ns_) /
            static_cast<double>(PERIOD_NS);

        if (coverage >= MIN_COVERAGE) {
            const double vx =
                angle_x_ * 200.0 + residual_x_;
            const double vy =
                angle_y_ * 200.0 + residual_y_;

            if (std::isfinite(vx) &&
                std::isfinite(vy) &&
                std::fabs(vx) < 2147483647.0 &&
                std::fabs(vy) < 2147483647.0) {

                result.x = static_cast<int32_t>(std::trunc(vx));
                result.y = static_cast<int32_t>(std::trunc(vy));
                result.valid = true;

                residual_x_ = vx - result.x;
                residual_y_ = vy - result.y;
            }
        }

        if (!result.valid) {
            residual_x_ = 0;
            residual_y_ = 0;
        }

        ready_.push_back(result);

        start_ns_ = end_ns_;
        end_ns_ += PERIOD_NS;

        clearWindow();
    }

    bool initialized_ = false;

    int64_t start_ns_ = 0;
    int64_t end_ns_ = 0;
    int64_t last_input_end_ns_ = 0;
    int64_t coverage_ns_ = 0;

    double angle_x_ = 0;
    double angle_y_ = 0;
    double residual_x_ = 0;
    double residual_y_ = 0;

    std::deque<Result> ready_;
};

}
