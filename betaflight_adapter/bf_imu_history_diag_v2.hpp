#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
#include <algorithm>
#include <opencv2/core.hpp>

namespace bf_imu {

struct GyroHistorySample {
    int64_t timestamp_ns = 0;
    std::array<float, 3> gyro{};
};

struct GyroInterval {
    bool covered = false;
    int failure_reason = 0;
    int samples = 0;
    double max_gap_ms = -1.0;
    double first_offset_ms = -1.0;
    double last_offset_ms = -1.0;
    double left_margin_ms = -1.0;
    double right_margin_ms = -1.0;
};

class GyroHistory {
public:
    struct GapStats {
        std::size_t count = 0;
        double median_ms = 0;
        double p95_ms = 0;
        double p99_ms = 0;
        double max_ms = 0;
        std::size_t gt30 = 0;
        std::size_t gt40 = 0;
        std::size_t gt50 = 0;
    };

    GapStats gapStats() const {
        GapStats s;
        if (gaps_ns_.empty())
            return s;

        std::vector<int64_t> sorted = gaps_ns_;
        std::sort(sorted.begin(), sorted.end());

        s.count = sorted.size();

        auto percentile = [&](double p) {
            const auto i = static_cast<std::size_t>(
                (sorted.size() - 1) * p
            );
            return sorted[i] * 1e-6;
        };

        s.median_ms = percentile(0.50);
        s.p95_ms = percentile(0.95);
        s.p99_ms = percentile(0.99);
        s.max_ms = sorted.back() * 1e-6;

        for (const auto gap : sorted) {
            if (gap > 30000000LL) ++s.gt30;
            if (gap > 40000000LL) ++s.gt40;
            if (gap > 50000000LL) ++s.gt50;
        }

        return s;
    }

    static constexpr int64_t HISTORY_NS = 1000000000LL;
    static constexpr int64_t MAX_GAP_NS = 30000000LL;

    bool push(int64_t timestamp_ns,
              const std::array<float, 3>& gyro) {
        if (timestamp_ns <= 0)
            return false;

        for (float v : gyro) {
            if (!std::isfinite(v))
                return false;
        }

        if (!samples_.empty() &&
            timestamp_ns <= samples_.back().timestamp_ns)
            return false;

        if (!samples_.empty()) {
            gaps_ns_.push_back(
                timestamp_ns - samples_.back().timestamp_ns
            );
        }

        samples_.push_back({timestamp_ns, gyro});

        while (samples_.size() > 1 &&
               timestamp_ns - samples_.front().timestamp_ns > HISTORY_NS)
            samples_.pop_front();

        return true;
    }

    GyroInterval inspect(int64_t t0, int64_t t1) const {
        GyroInterval result;

        if (t0 <= 0 || t1 <= t0 || samples_.size() < 2) {
            result.failure_reason = 5;
            return result;
        }

        result.first_offset_ms =
            (samples_.front().timestamp_ns - t0) * 1e-6;
        result.last_offset_ms =
            (samples_.back().timestamp_ns - t1) * 1e-6;

        const bool left_missing =
            samples_.front().timestamp_ns > t0;
        const bool right_missing =
            samples_.back().timestamp_ns < t1;

        if (left_missing || right_missing) {
            result.failure_reason =
                left_missing && right_missing ? 4 :
                left_missing ? 1 : 2;
            return result;
        }

        std::size_t index = 0;

        while (index + 1 < samples_.size() &&
               samples_[index + 1].timestamp_ns <= t0)
            ++index;

        result.left_margin_ms =
            (t0 - samples_[index].timestamp_ns) * 1e-6;

        int64_t previous_ns = samples_[index].timestamp_ns;
        int64_t max_gap_ns = 0;

        for (++index; index < samples_.size(); ++index) {
            const int64_t current_ns =
                samples_[index].timestamp_ns;

            const int64_t gap_ns = current_ns - previous_ns;

            if (gap_ns > max_gap_ns)
                max_gap_ns = gap_ns;

            previous_ns = current_ns;
            ++result.samples;

            if (current_ns >= t1)
                break;
        }

        result.max_gap_ms = max_gap_ns * 1e-6;

        if (previous_ns >= t1)
            result.right_margin_ms =
                (previous_ns - t1) * 1e-6;

        result.covered =
            previous_ns >= t1 &&
            result.samples > 0 &&
            max_gap_ns <= MAX_GAP_NS;

        if (!result.covered)
            result.failure_reason = 3;

        return result;
    }

    bool integrateRotation(
        int64_t t0,
        int64_t t1,
        double gyro_scale_rad_s,
        cv::Matx33d& R
    ) const {
        R = cv::Matx33d::eye();

        const auto interval = inspect(t0, t1);
        if (!interval.covered)
            return false;

        auto interpolate = [](
            const GyroHistorySample& a,
            const GyroHistorySample& b,
            int64_t t
        ) {
            const double alpha =
                double(t - a.timestamp_ns) /
                double(b.timestamp_ns - a.timestamp_ns);

            cv::Vec3d w;
            for (int k = 0; k < 3; ++k)
                w[k] = a.gyro[k] +
                    alpha * (b.gyro[k] - a.gyro[k]);
            return w;
        };

        auto expSO3 = [](const cv::Vec3d& v) {
            const double theta = cv::norm(v);
            const cv::Matx33d K(
                0, -v[2], v[1],
                v[2], 0, -v[0],
                -v[1], v[0], 0
            );

            if (theta < 1e-9)
                return cv::Matx33d::eye() + K;

            return cv::Matx33d::eye()
                + (std::sin(theta) / theta) * K
                + ((1 - std::cos(theta)) /
                   (theta * theta)) * (K * K);
        };

        auto angularRate = [&](int64_t t) {
            for (std::size_t i = 1; i < samples_.size(); ++i) {
                if (samples_[i].timestamp_ns >= t) {
                    return interpolate(
                        samples_[i - 1], samples_[i], t
                    ) * gyro_scale_rad_s;
                }
            }
            return cv::Vec3d(0, 0, 0);
        };

        int64_t t = t0;
        cv::Vec3d w0 = angularRate(t0);

        while (t < t1) {
            int64_t next = t1;

            for (const auto& sample : samples_) {
                if (sample.timestamp_ns > t) {
                    next = std::min(next, sample.timestamp_ns);
                    break;
                }
            }

            if (next <= t)
                return false;

            const cv::Vec3d w1 = angularRate(next);
            const double dt = (next - t) * 1e-9;
            const cv::Vec3d angle = (w0 + w1) * (0.5 * dt);

            R = R * expSO3(angle);

            t = next;
            w0 = w1;
        }

        return true;
    }

    std::size_t size() const {
        return samples_.size();
    }

private:
    std::deque<GyroHistorySample> samples_;
    std::vector<int64_t> gaps_ns_;
};

} // namespace bf_imu
