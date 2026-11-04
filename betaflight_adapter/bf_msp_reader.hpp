#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace bf_msp {

constexpr uint8_t MSP_RAW_IMU = 102;
constexpr uint8_t MSP_ATTITUDE = 108;

struct SensorSample {
    uint8_t command = 0;
    int64_t timestamp_ns = 0;
    std::array<int16_t, 9> raw_imu{};
    std::array<double, 3> attitude_rad{};
};

inline int16_t readInt16LE(const uint8_t* p) {
    const uint16_t value =
        static_cast<uint16_t>(p[0]) |
        (static_cast<uint16_t>(p[1]) << 8);

    return static_cast<int16_t>(value);
}

inline std::array<uint8_t, 6> mspRequest(uint8_t command) {
    return {{'$', 'M', '<', 0, command, command}};
}

class MspReader {
public:
    void feed(const uint8_t* data, std::size_t size, int64_t timestamp_ns) {
        buffer_.insert(buffer_.end(), data, data + size);

        if (buffer_.size() > 4096) {
            buffer_.erase(
                buffer_.begin(),
                buffer_.end() - 4096
            );
        }

        parse(timestamp_ns);
    }

    bool pop(SensorSample& sample) {
        if (samples_.empty())
            return false;

        sample = samples_.front();
        samples_.pop_front();
        return true;
    }

    void reset() {
        buffer_.clear();
        samples_.clear();
    }

    std::size_t bufferedBytes() const {
        return buffer_.size();
    }

    std::size_t pendingSamples() const {
        return samples_.size();
    }

private:
    std::vector<uint8_t> buffer_;
    std::deque<SensorSample> samples_;

    void parse(int64_t timestamp_ns) {
        constexpr double PI = 3.14159265358979323846;

        while (buffer_.size() >= 3) {
            if (buffer_[0] != '$' ||
                buffer_[1] != 'M' ||
                buffer_[2] != '>') {
                buffer_.erase(buffer_.begin());
                continue;
            }

            if (buffer_.size() < 6)
                return;

            const uint8_t payload_size = buffer_[3];
            const uint8_t command = buffer_[4];
            const std::size_t frame_size =
                static_cast<std::size_t>(payload_size) + 6;

            if (buffer_.size() < frame_size)
                return;

            uint8_t checksum = payload_size ^ command;

            for (std::size_t i = 5; i < frame_size - 1; ++i)
                checksum ^= buffer_[i];

            if (checksum != buffer_[frame_size - 1]) {
                // Shift one byte and search for the next header.
                buffer_.erase(buffer_.begin());
                continue;
            }

            SensorSample sample;
            sample.command = command;
            sample.timestamp_ns = timestamp_ns;

            bool recognized = false;

            if (command == MSP_RAW_IMU && payload_size == 18) {
                for (int i = 0; i < 9; ++i) {
                    sample.raw_imu[i] =
                        readInt16LE(&buffer_[5 + i * 2]);
                }
                recognized = true;
            }

            if (command == MSP_ATTITUDE && payload_size == 6) {
                const int16_t roll = readInt16LE(&buffer_[5]);
                const int16_t pitch = readInt16LE(&buffer_[7]);
                const int16_t yaw = readInt16LE(&buffer_[9]);

                sample.attitude_rad[0] =
                    (roll / 10.0) * PI / 180.0;

                sample.attitude_rad[1] =
                    (pitch / 10.0) * PI / 180.0;

                sample.attitude_rad[2] =
                    yaw * PI / 180.0;

                recognized = true;
            }

            if (recognized)
                samples_.push_back(sample);

            buffer_.erase(
                buffer_.begin(),
                buffer_.begin() + frame_size
            );
        }
    }
};

} // namespace bf_msp
