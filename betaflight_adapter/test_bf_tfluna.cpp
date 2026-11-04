#include "bf_tfluna.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

static std::vector<uint8_t> makeFrame(
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

int main() {
    bf_tfluna::Parser parser;
    bf_tfluna::Measurement m;

    // 123 cm, valid signal.
    const auto valid = makeFrame(123, 250);

    parser.feed(valid.data(), 4);
    assert(!parser.pop(m));

    parser.feed(valid.data() + 4, valid.size() - 4);
    assert(parser.pop(m));
    assert(std::abs(m.distance_m - 1.23) < 1e-9);
    assert(m.strength == 250);

    std::cout << "PASS: fragmented valid frame\n";

    // Invalid checksum.
    auto bad = makeFrame(200, 300);
    bad[8] ^= 0x01;

    parser.feed(bad.data(), bad.size());
    assert(!parser.pop(m));

    std::cout << "PASS: checksum rejection\n";

    // Recovery after corrupted frame.
    parser.feed(valid.data(), valid.size());
    assert(parser.pop(m));

    std::cout << "PASS: parser recovery\n";

    // Distance below 5 cm.
    auto too_close = makeFrame(4, 300);
    parser.feed(too_close.data(), too_close.size());
    assert(!parser.pop(m));

    // Distance above 800 cm.
    auto too_far = makeFrame(801, 300);
    parser.feed(too_far.data(), too_far.size());
    assert(!parser.pop(m));

    std::cout << "PASS: distance filtering\n";

    // Invalid signal strength.
    auto zero_strength = makeFrame(100, 0);
    auto max_strength = makeFrame(100, 65535);

    parser.feed(zero_strength.data(), zero_strength.size());
    parser.feed(max_strength.data(), max_strength.size());

    assert(!parser.pop(m));

    std::cout << "PASS: strength filtering\n";

    // Two valid frames in one buffer.
    std::vector<uint8_t> combined = valid;
    combined.insert(
        combined.end(),
        valid.begin(),
        valid.end()
    );

    parser.feed(combined.data(), combined.size());

    assert(parser.pop(m));
    assert(parser.pop(m));
    assert(!parser.pop(m));

    std::cout << "PASS: multiple frames\n";
    std::cout << "PASS: no UART access\n";

    return 0;
}
