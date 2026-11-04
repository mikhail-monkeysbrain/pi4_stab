#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace bf_msp {

constexpr uint16_t RANGEFINDER = 0x1F01;
constexpr uint16_t OPTICALFLOW = 0x1F02;

inline uint8_t crc8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x80)
                ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

inline void append32(std::vector<uint8_t>& out, int32_t value) {
    const uint32_t v = static_cast<uint32_t>(value);
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

inline std::vector<uint8_t> frame(
    uint16_t command,
    const std::vector<uint8_t>& payload
) {
    if (payload.size() > 65535)
        throw std::invalid_argument("MSP payload too large");

    const uint16_t size = static_cast<uint16_t>(payload.size());

    std::vector<uint8_t> body = {
        0,
        static_cast<uint8_t>(command),
        static_cast<uint8_t>(command >> 8),
        static_cast<uint8_t>(size),
        static_cast<uint8_t>(size >> 8)
    };

    body.insert(body.end(), payload.begin(), payload.end());

    std::vector<uint8_t> result = {'$', 'X', '<'};
    result.insert(result.end(), body.begin(), body.end());
    result.push_back(crc8(body));
    return result;
}

inline std::vector<uint8_t> rangefinder(
    double distance_m,
    uint8_t quality = 255
) {
    if (!std::isfinite(distance_m) ||
        distance_m < 0.0 || distance_m > 2147483.647)
        throw std::invalid_argument("Invalid range");

    const auto mm = static_cast<int32_t>(
        std::nearbyint(distance_m * 1000.0)
    );

    std::vector<uint8_t> payload = {quality};
    append32(payload, mm);
    return frame(RANGEFINDER, payload);
}

inline std::vector<uint8_t> opticalflow(
    int32_t x,
    int32_t y,
    uint8_t quality = 255
) {
    std::vector<uint8_t> payload = {quality};
    append32(payload, x);
    append32(payload, y);
    return frame(OPTICALFLOW, payload);
}

class AngularMotionQuantizer {
public:
    void reset() {
        residual_x_ = 0.0;
        residual_y_ = 0.0;
    }

    int32_t encodeAxis(double rate, double dt, double& residual) {
        if (!std::isfinite(rate) || !std::isfinite(dt) ||
            dt <= 0.0 || dt >= 1.0)
            throw std::invalid_argument("Invalid flow input");

        double value = rate * dt * 200.0 + residual;
        const double nearest = std::nearbyint(value);

        if (std::abs(value - nearest) < 1e-10)
            value = nearest;

        if (!std::isfinite(value) ||
            value < -2147483648.0 ||
            value >= 2147483648.0)
            throw std::overflow_error("Flow overflow");

        const auto result = static_cast<int32_t>(std::trunc(value));
        residual = value - result;
        return result;
    }

    void encode(double rx, double ry, double dt,
                int32_t& x, int32_t& y) {
        double next_x = residual_x_;
        double next_y = residual_y_;

        const int32_t qx = encodeAxis(rx, dt, next_x);
        const int32_t qy = encodeAxis(ry, dt, next_y);

        x = qx;
        y = qy;
        residual_x_ = next_x;
        residual_y_ = next_y;
    }

private:
    double residual_x_ = 0.0;
    double residual_y_ = 0.0;
};

} // namespace bf_msp
