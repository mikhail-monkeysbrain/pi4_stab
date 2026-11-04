#include "bf_msp_encoder.hpp"

#include <cassert>
#include <iostream>

int main() {
    const auto rf = bf_msp::rangefinder(1.234);
    const auto of = bf_msp::opticalflow(12, -34, 200);

    assert(rf.size() == 14);
    assert(of.size() == 18);

    assert(rf[0] == '$' && rf[1] == 'X' && rf[2] == '<');
    assert(of[0] == '$' && of[1] == 'X' && of[2] == '<');

    bf_msp::AngularMotionQuantizer q;
    int64_t sum = 0;

    for (int i = 0; i < 100; ++i) {
        int32_t x = 0, y = 0;
        q.encode(0.1, 0.0, 0.02, x, y);
        sum += x;
    }

    assert(sum == 40);

    std::cout << "PASS: MSP frame lengths and quantizer\n";
    std::cout << "RANGEFINDER:";
    for (auto b : rf) std::printf(" %02x", b);
    std::cout << "\nOPTICALFLOW:";
    for (auto b : of) std::printf(" %02x", b);
    std::cout << "\n";

    return 0;
}
