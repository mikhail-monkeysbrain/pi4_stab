#include "bf_flow_msp_aggregator.hpp"

#include <cstdint>
#include <iostream>

using bf_flow::MspAggregator;

int main() {
    constexpr int64_t MS = 1000000LL;
    constexpr int64_t BASE = 1000000000LL;

    MspAggregator agg;
    agg.begin(BASE);

    int valid = 0;
    int invalid = 0;
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    bool coverage_ok = true;

    // 100 кадров по 10 мс.
    // Смещение 5 мс заставляет интервалы
    // пересекать границы 50-мс окон.
    for (int i = 0; i < 100; ++i) {
        const int64_t ts =
            BASE + 5 * MS + (i + 1) * 10 * MS;

        if (!agg.add(1.0, -0.5, 0.010, ts)) {
            std::cout << "ADD_FAILED i=" << i << '\n';
            return 1;
        }

        MspAggregator::Result r;
        while (agg.take(r)) {
            if (r.valid) {
                ++valid;
                sum_x += r.x;
                sum_y += r.y;
                if (r.coverage_ns != 50 * MS)
                    coverage_ok = false;
            } else {
                ++invalid;
            }
        }
    }

    // Завершаем окна с учётом задержки.
    agg.flush(BASE + 1100 * MS);

    MspAggregator::Result r;
    while (agg.take(r)) {
        if (r.valid) {
            ++valid;
            sum_x += r.x;
            sum_y += r.y;
            if (r.coverage_ns != 50 * MS)
                coverage_ok = false;
        } else {
            ++invalid;
        }
    }

    // Первое окно покрыто на 45 мс.
    // Последнее — только на 5 мс.
    // Между ними ожидаем 19 полных окон.
    const bool ok =
        valid == 19 &&
        sum_x == 190 &&
        sum_y == -95 &&
        coverage_ok;

    std::cout
        << "BOUNDARY_RESULT"
        << " valid=" << valid
        << " invalid=" << invalid
        << " sum_x=" << sum_x
        << " sum_y=" << sum_y
        << " coverage_ok=" << coverage_ok
        << " RESULT=" << (ok ? "PASS" : "FAIL")
        << '\n';

    return ok ? 0 : 1;
}
