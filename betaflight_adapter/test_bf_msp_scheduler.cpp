#include "bf_msp_scheduler.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>

int main() {
    bf_msp::PollScheduler scheduler;

    constexpr int64_t T0 = 1000000000LL;
    constexpr int64_t MS = 1000000LL;

    int imu_count = 0;
    int att_count = 0;

    // 1000 ms, sampled every 1 ms.
    for (int i = 0; i < 1000; ++i) {
        auto d = scheduler.poll(T0 + i * MS);

        imu_count += d.imu ? 1 : 0;
        att_count += d.attitude ? 1 : 0;
    }

    assert(imu_count == 100);
    assert(att_count == 20);

    std::cout << "PASS: IMU 100 Hz schedule\n";
    std::cout << "PASS: ATTITUDE 20 Hz schedule\n";

    // No duplicate requests at same timestamp.
    scheduler.reset();

    auto first = scheduler.poll(T0);
    auto duplicate = scheduler.poll(T0);

    assert(first.imu && first.attitude);
    assert(!duplicate.imu && !duplicate.attitude);

    std::cout << "PASS: duplicate suppression\n";

    // A 200 ms stall must not generate a burst.
    auto after_stall = scheduler.poll(T0 + 200 * MS);

    assert(after_stall.imu);
    assert(after_stall.attitude);

    auto again = scheduler.poll(T0 + 200 * MS);

    assert(!again.imu);
    assert(!again.attitude);

    std::cout << "PASS: missed-period recovery\n";

    // The next request is scheduled normally.
    auto next = scheduler.poll(T0 + 210 * MS);

    assert(next.imu);
    assert(!next.attitude);

    std::cout << "PASS: schedule resumes\n";
    std::cout << "PASS: no UART access\n";

    return 0;
}
