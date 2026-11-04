#include "bf_flow_msp_aggregator.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

using bf_flow::MspAggregator;

struct Stats {
    int valid = 0;
    int invalid = 0;
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t coverage_ns = 0;
};

static void collect(MspAggregator& agg, Stats& s) {
    MspAggregator::Result r;

    while (agg.take(r)) {
        if (r.valid) {
            ++s.valid;
            s.sum_x += r.x;
            s.sum_y += r.y;
            s.coverage_ns += r.coverage_ns;
        } else {
            ++s.invalid;
        }
    }
}

static Stats run(
    double rate_x,
    double rate_y,
    bool drop_frame
) {
    constexpr int64_t MS = 1000000LL;
    constexpr int64_t BASE = 1000000000LL;

    MspAggregator agg;
    Stats s;

    agg.begin(BASE);

    // 100 кадров по 10 мс = 1 секунда.
    // Ожидаем 20 окон по 50 мс.
    for (int i = 1; i <= 100; ++i) {
        if (drop_frame && i == 37)
            continue;

        const int64_t ts = BASE + i * 10 * MS;

        if (!agg.add(rate_x, rate_y, 0.010, ts)) {
            ++s.invalid;
            agg.begin(ts - 10 * MS);
            agg.add(rate_x, rate_y, 0.010, ts);
        }

        collect(agg, s);
    }

    agg.flush(BASE + 1100 * MS);
    collect(agg, s);

    return s;
}

static bool check(
    const char* name,
    const Stats& s,
    int expected_valid,
    int64_t expected_x,
    int64_t expected_y
) {
    const bool ok =
        s.valid == expected_valid &&
        s.sum_x == expected_x &&
        s.sum_y == expected_y;

    std::cout
        << name
        << " valid=" << s.valid
        << " invalid=" << s.invalid
        << " sum_x=" << s.sum_x
        << " sum_y=" << s.sum_y
        << " expected_valid=" << expected_valid
        << " expected_x=" << expected_x
        << " expected_y=" << expected_y
        << " RESULT=" << (ok ? "PASS" : "FAIL")
        << '\n';

    return ok;
}

int main() {
    bool ok = true;

    // 1 рад/с * 1 сек * 200 = 200.
    const auto constant = run(1.0, -0.5, false);
    ok &= check(
        "CONSTANT",
        constant,
        20,
        200,
        -100
    );

    const auto zero = run(0.0, 0.0, false);
    ok &= check(
        "ZERO",
        zero,
        20,
        0,
        0
    );

    // Интервалы по 10 мс пересекают
    // границы 50-миллисекундных окон.
    const auto boundaries = run(0.5, 0.25, false);
    ok &= check(
        "BOUNDARIES",
        boundaries,
        20,
        100,
        50
    );

    const auto dropout = run(1.0, 0.0, true);

    const bool dropout_ok =
        dropout.invalid > 0 &&
        dropout.valid < 20;

    std::cout
        << "DROPOUT"
        << " valid=" << dropout.valid
        << " invalid=" << dropout.invalid
        << " RESULT="
        << (dropout_ok ? "PASS" : "FAIL")
        << '\n';

    ok &= dropout_ok;

    std::cout
        << "BF_AGGREGATOR_TEST_RESULT="
        << (ok ? "PASS" : "FAIL")
        << '\n';

    return ok ? 0 : 1;
}
