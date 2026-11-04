#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace bf_tfluna {

constexpr uint16_t MIN_CM = 5;
constexpr uint16_t MAX_CM = 800;

struct Measurement {
    double distance_m = 0.0;
    uint16_t strength = 0;
};

class Parser {
public:
    void feed(const uint8_t* data, std::size_t size) {
        if (size == 0)
            return;

        buffer_.insert(buffer_.end(), data, data + size);

        // Match Python buffer limit.
        if (buffer_.size() > 512) {
            buffer_.erase(
                buffer_.begin(),
                buffer_.end() - 512
            );
        }

        parse();
    }

    bool pop(Measurement& result) {
        if (measurements_.empty())
            return false;

        result = measurements_.front();
        measurements_.pop_front();
        return true;
    }

    void reset() {
        buffer_.clear();
        measurements_.clear();
    }

    std::size_t pending() const {
        return measurements_.size();
    }

private:
    std::vector<uint8_t> buffer_;
    std::deque<Measurement> measurements_;

    void parse() {
        while (buffer_.size() >= 9) {
            if (buffer_[0] != 0x59 ||
                buffer_[1] != 0x59) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            uint8_t checksum = 0;

            for (int i = 0; i < 8; ++i)
                checksum = static_cast<uint8_t>(
                    checksum + buffer_[i]
                );

            if (checksum != buffer_[8]) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            const uint16_t distance_cm =
                static_cast<uint16_t>(buffer_[2]) |
                (static_cast<uint16_t>(buffer_[3]) << 8);

            const uint16_t strength =
                static_cast<uint16_t>(buffer_[4]) |
                (static_cast<uint16_t>(buffer_[5]) << 8);

            buffer_.erase(
                buffer_.begin(),
                buffer_.begin() + 9
            );

            if (distance_cm < MIN_CM ||
                distance_cm > MAX_CM)
                continue;

            if (strength == 0 || strength == 65535)
                continue;

            Measurement result;
            result.distance_m = distance_cm / 100.0;
            result.strength = strength;

            measurements_.push_back(result);
        }
    }
};

} // namespace bf_tfluna
