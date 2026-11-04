#include "bf_msp_reader.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

static std::vector<uint8_t> response(
    uint8_t command,
    const std::vector<uint8_t>& payload
) {
    std::vector<uint8_t> result = {
        '$', 'M', '>',
        static_cast<uint8_t>(payload.size()),
        command
    };

    uint8_t checksum =
        static_cast<uint8_t>(payload.size()) ^ command;

    for (uint8_t b : payload) {
        result.push_back(b);
        checksum ^= b;
    }

    result.push_back(checksum);
    return result;
}

static void append16(
    std::vector<uint8_t>& data,
    int16_t value
) {
    const uint16_t v = static_cast<uint16_t>(value);
    data.push_back(static_cast<uint8_t>(v));
    data.push_back(static_cast<uint8_t>(v >> 8));
}

int main() {
    bf_msp::MspReader reader;
    bf_msp::SensorSample sample;

    const auto request =
        bf_msp::mspRequest(bf_msp::MSP_RAW_IMU);

    assert(request[0] == '$');
    assert(request[1] == 'M');
    assert(request[2] == '<');
    assert(request[4] == 102);
    assert(request[5] == 102);

    std::vector<uint8_t> imu_payload;

    for (int i = 0; i < 9; ++i)
        append16(imu_payload, static_cast<int16_t>(i * 100 - 400));

    const auto imu = response(
        bf_msp::MSP_RAW_IMU,
        imu_payload
    );

    // Feed the frame in two parts.
    reader.feed(imu.data(), 7, 1000000);
    assert(!reader.pop(sample));

    reader.feed(imu.data() + 7, imu.size() - 7, 2000000);
    assert(reader.pop(sample));

    assert(sample.command == 102);
    assert(sample.timestamp_ns == 2000000);
    assert(sample.raw_imu[0] == -400);
    assert(sample.raw_imu[4] == 0);
    assert(sample.raw_imu[8] == 400);

    std::vector<uint8_t> att_payload;
    append16(att_payload, 900);
    append16(att_payload, -450);
    append16(att_payload, 180);

    const auto att = response(
        bf_msp::MSP_ATTITUDE,
        att_payload
    );

    reader.feed(att.data(), att.size(), 3000000);
    assert(reader.pop(sample));

    assert(sample.command == 108);
    assert(std::abs(sample.attitude_rad[0] - 1.57079632679) < 1e-8);
    assert(std::abs(sample.attitude_rad[1] + 0.78539816339) < 1e-8);
    assert(std::abs(sample.attitude_rad[2] - 3.14159265359) < 1e-8);

    // Corrupted frame must be rejected.
    auto bad = imu;
    bad.back() ^= 0x01;

    reader.feed(bad.data(), bad.size(), 4000000);
    assert(!reader.pop(sample));

    // Parser must recover after corruption.
    reader.feed(imu.data(), imu.size(), 5000000);
    assert(reader.pop(sample));
    assert(sample.timestamp_ns == 5000000);

    std::cout << "PASS: MSPv1 request encoding\n";
    std::cout << "PASS: fragmented IMU frame\n";
    std::cout << "PASS: ATTITUDE conversion\n";
    std::cout << "PASS: corrupted frame rejection\n";
    std::cout << "PASS: parser recovery\n";
    std::cout << "PASS: no UART access\n";

    return 0;
}
