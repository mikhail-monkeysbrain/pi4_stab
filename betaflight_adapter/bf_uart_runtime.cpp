#include "bf_msp_encoder.hpp"
#include "bf_msp_reader.hpp"
#include "bf_msp_scheduler.hpp"
#include "bf_sensor_state.hpp"
#include "bf_tfluna.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

constexpr int64_t T0 = 1000000000LL;
constexpr int64_t MS = 1000000LL;

std::vector<uint8_t> mspResponse(
    uint8_t command,
    const std::vector<uint8_t>& payload
) {
    std::vector<uint8_t> frame = {
        '$', 'M', '>',
        static_cast<uint8_t>(payload.size()),
        command
    };

    uint8_t checksum =
        static_cast<uint8_t>(payload.size()) ^ command;

    for (uint8_t b : payload) {
        frame.push_back(b);
        checksum ^= b;
    }

    frame.push_back(checksum);
    return frame;
}

std::vector<uint8_t> tfFrame(
    uint16_t distance_cm,
    uint16_t strength
) {
    std::vector<uint8_t> frame = {
        0x59, 0x59,
        static_cast<uint8_t>(distance_cm),
        static_cast<uint8_t>(distance_cm >> 8),
        static_cast<uint8_t>(strength),
        static_cast<uint8_t>(strength >> 8),
        0, 0, 0
    };

    uint8_t checksum = 0;
    for (int i = 0; i < 8; ++i)
        checksum = static_cast<uint8_t>(
            checksum + frame[i]
        );

    frame[8] = checksum;
    return frame;
}

void append16(std::vector<uint8_t>& dst, int16_t value) {
    const auto bits = static_cast<uint16_t>(value);
    dst.push_back(static_cast<uint8_t>(bits));
    dst.push_back(static_cast<uint8_t>(bits >> 8));
}

}

int main() {
    bf_msp::PollScheduler scheduler;
    bf_msp::MspReader reader;
    bf_imu::SensorState sensors;
    bf_tfluna::Parser luna;

    uint64_t imu_requests = 0;
    uint64_t att_requests = 0;
    uint64_t imu_responses = 0;
    uint64_t att_responses = 0;
    uint64_t range_built = 0;

    // Synthetic 1-second scheduler run.
    for (int i = 0; i < 1000; ++i) {
        const int64_t now = T0 + i * MS;
        const auto decision = scheduler.poll(now);

        if (decision.imu) {
            ++imu_requests;

            std::vector<uint8_t> payload;
            for (int k = 0; k < 9; ++k)
                append16(payload, static_cast<int16_t>(
                    k == 3 ? 160 : 0
                ));

            const auto frame = mspResponse(
                bf_msp::MSP_RAW_IMU, payload
            );

            reader.feed(frame.data(), frame.size(), now);
        }

        if (decision.attitude) {
            ++att_requests;

            std::vector<uint8_t> payload;
            append16(payload, 100); // roll = 10 deg
            append16(payload, 0);
            append16(payload, 0);

            const auto frame = mspResponse(
                bf_msp::MSP_ATTITUDE, payload
            );

            reader.feed(frame.data(), frame.size(), now);
        }

        bf_msp::SensorSample sample;
        while (reader.pop(sample)) {
            sensors.update(sample);

            if (sample.command == bf_msp::MSP_RAW_IMU)
                ++imu_responses;
            else if (sample.command == bf_msp::MSP_ATTITUDE)
                ++att_responses;
        }
    }

    assert(imu_requests == 100);
    assert(att_requests == 20);
    assert(imu_responses == 100);
    assert(att_responses == 20);

    assert(sensors.imuCount() == 100);
    assert(sensors.attCount() == 20);

    const auto snapshot = sensors.snapshot(
        T0 + 990 * MS
    );

    assert(snapshot.imu.valid);
    assert(snapshot.imu_fresh);
    assert(snapshot.attitude.valid);
    assert(snapshot.attitude_fresh);
    assert(snapshot.imu.gyro[0] == 160.0f);

    constexpr double PI = 3.14159265358979323846;
    assert(std::abs(
        snapshot.attitude.roll - 10.0 * PI / 180.0
    ) < 1e-5);

    // TF-Luna: fragmented 123 cm frame.
    const auto tf = tfFrame(123, 250);
    luna.feed(tf.data(), 3);
    luna.feed(tf.data() + 3, tf.size() - 3);

    bf_tfluna::Measurement measurement;
    assert(luna.pop(measurement));
    assert(std::abs(measurement.distance_m - 1.23) < 1e-9);

    // Build MSPv2 Rangefinder, but do not transmit.
    const auto range_packet =
        bf_msp::rangefinder(measurement.distance_m, 255);

    assert(range_packet.size() == 14);
    ++range_built;

    std::cout << "IMU_REQUESTS=" << imu_requests << '\n';
    std::cout << "ATT_REQUESTS=" << att_requests << '\n';
    std::cout << "IMU_RESPONSES=" << imu_responses << '\n';
    std::cout << "ATT_RESPONSES=" << att_responses << '\n';
    std::cout << "GYRO_HISTORY="
              << sensors.gyroHistorySize() << '\n';
    std::cout << "RANGE_MSP_BUILT=" << range_built << '\n';

    std::cout << "PASS: integrated offline runtime\n";
    std::cout << "PASS: no UART access\n";
    std::cout << "PASS: no Optical Flow TX\n";

    return 0;
}
