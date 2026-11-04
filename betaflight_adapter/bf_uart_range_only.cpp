#include "bf_msp_encoder.hpp"
#include "bf_msp_reader.hpp"
#include "bf_msp_scheduler.hpp"
#include "bf_sensor_state.hpp"
#include "bf_serial_port.hpp"
#include "bf_tfluna.hpp"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <time.h>

namespace {

volatile std::sig_atomic_t stop_requested = 0;

extern "C" void onSignal(int) {
    stop_requested = 1;
}


int64_t monoNs() {
    timespec ts{};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        throw std::runtime_error("clock_gettime failed");

    return int64_t(ts.tv_sec) * 1000000000LL +
           int64_t(ts.tv_nsec);
}

struct Counters {
    uint64_t imu_req = 0;
    uint64_t att_req = 0;
    uint64_t imu_rx = 0;
    uint64_t att_rx = 0;
    uint64_t luna_rx = 0;
    uint64_t range_msp_built = 0;
    uint64_t range_msp_tx = 0;
    uint64_t range_msp_errors = 0;
    uint64_t fc_rx_bytes = 0;
    uint64_t luna_rx_bytes = 0;
    uint64_t fc_read_errors = 0;
    uint64_t luna_read_errors = 0;
};

}

int main(int argc, char** argv) {
    bool range_tx_enabled = false;

    if (argc == 2 && std::string(argv[1]) == "--range-tx") {
        range_tx_enabled = true;
    } else if (argc != 1) {
        std::cerr << "Usage: bf_uart_runtime_live [--range-tx]\n";
        return 2;
    }
    try {
        bf_serial::Port fc("/dev/ttyAMA4", B230400);
        bf_serial::Port range("/dev/serial0", B115200);

        bf_msp::PollScheduler scheduler;
        bf_msp::MspReader reader;
        bf_imu::SensorState sensors;
        bf_tfluna::Parser luna;
        Counters count;

        const int64_t start_ns = monoNs();

        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        double last_distance_m = 0.0;
        double luna_min_m = 1e9;
        double luna_max_m = 0.0;
        uint16_t last_strength = 0;
        int64_t last_luna_ns = 0;
        int64_t next_range_tx_ns = start_ns;
        constexpr int64_t RANGE_PERIOD_NS = 100000000LL;
        constexpr int64_t RANGE_MAX_AGE_NS = 750000000LL;

        std::cout << "BF_LIVE_START\n";
        std::cout << "DURATION=UNTIL_CTRL_C\n";
        std::cout << "RANGE_TX="
                  << (range_tx_enabled ? "ENABLED" : "DISABLED")
                  << '\n';
        std::cout << "FLOW_TX=DISABLED\n";

        while (!stop_requested) {
            const int64_t now = monoNs();
            const auto decision = scheduler.poll(now);

            (void)decision;
            // Diagnostic: IMU/ATTITUDE polling disabled.

            if (fc.flushTx() < 0)
                throw std::runtime_error("FC UART TX error");

            uint8_t buffer[512];

            // Read available FC bytes.
            for (int i = 0; i < 8; ++i) {
                const ssize_t n = fc.read(
                    buffer, sizeof(buffer)
                );

                if (n < 0) {
                    ++count.fc_read_errors;
                    throw std::runtime_error("FC UART RX error");
                }

                if (n == 0)
                    break;

                count.fc_rx_bytes +=
                    static_cast<uint64_t>(n);

                reader.feed(
                    buffer,
                    static_cast<std::size_t>(n),
                    monoNs()
                );
            }

            bf_msp::SensorSample sample;

            while (reader.pop(sample)) {
                sensors.update(sample);

                if (sample.command ==
                    bf_msp::MSP_RAW_IMU) {
                    ++count.imu_rx;
                } else if (sample.command ==
                           bf_msp::MSP_ATTITUDE) {
                    ++count.att_rx;
                }
            }

            // Read available TF-Luna bytes.
            for (int i = 0; i < 8; ++i) {
                const ssize_t n = range.read(
                    buffer, sizeof(buffer)
                );

                if (n < 0) {
                    ++count.luna_read_errors;
                    throw std::runtime_error(
                        "TF-Luna UART RX error"
                    );
                }

                if (n == 0)
                    break;

                count.luna_rx_bytes +=
                    static_cast<uint64_t>(n);

                luna.feed(
                    buffer,
                    static_cast<std::size_t>(n)
                );
            }

            bf_tfluna::Measurement measurement;

            while (luna.pop(measurement)) {
                ++count.luna_rx;
                last_distance_m = measurement.distance_m;
                luna_min_m = std::min(
                    luna_min_m, measurement.distance_m
                );
                luna_max_m = std::max(
                    luna_max_m, measurement.distance_m
                );
                last_strength = measurement.strength;

                last_luna_ns = monoNs();
            }

            // Independent 10 Hz Rangefinder scheduler.
            const int64_t range_now_ns = monoNs();

            if (range_now_ns >= next_range_tx_ns) {
                next_range_tx_ns =
                    range_now_ns + RANGE_PERIOD_NS;

                if (last_luna_ns != 0 &&
                    range_now_ns - last_luna_ns <= RANGE_MAX_AGE_NS) {
                    try {
                        const auto packet = bf_msp::rangefinder(
                            last_distance_m, 255
                        );

                        if (packet.size() == 14 &&
                            packet[0] == '$' &&
                            packet[1] == 'X' &&
                            packet[2] == '<') {
                            ++count.range_msp_built;

                            if (range_tx_enabled) {
                                fc.enqueue(packet.data(), packet.size());
                                ++count.range_msp_tx;
                            }
                        } else {
                            ++count.range_msp_errors;
                        }
                    } catch (const std::exception&) {
                        ++count.range_msp_errors;
                    }
                }
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(1)
            );
        }

        const auto snapshot = sensors.snapshot(monoNs());

        const double duration_s =
            double(monoNs() - start_ns) / 1e9;

        std::cout << "===== BF_LIVE_RESULT =====\n";
        std::cout << "DURATION_S="
                  << duration_s << '\n';
        std::cout << "IMU_REQ=" << count.imu_req << '\n';
        std::cout << "ATT_REQ=" << count.att_req << '\n';
        std::cout << "IMU_RX=" << count.imu_rx << '\n';
        std::cout << "ATT_RX=" << count.att_rx << '\n';
        std::cout << "LUNA_RX=" << count.luna_rx << '\n';
        std::cout << "FC_RX_BYTES="
                  << count.fc_rx_bytes << '\n';
        std::cout << "LUNA_RX_BYTES="
                  << count.luna_rx_bytes << '\n';
        std::cout << "FC_READ_ERRORS="
                  << count.fc_read_errors << '\n';
        std::cout << "LUNA_READ_ERRORS="
                  << count.luna_read_errors << '\n';
        std::cout << "GYRO_HISTORY="
                  << sensors.gyroHistorySize() << '\n';
        std::cout << "IMU_FRESH="
                  << snapshot.imu_fresh << '\n';
        std::cout << "ATT_FRESH="
                  << snapshot.attitude_fresh << '\n';
        std::cout << "LUNA_MIN_M="
                  << (count.luna_rx ? luna_min_m : 0.0)
                  << '\n';
        std::cout << "LUNA_MAX_M="
                  << luna_max_m << '\n';
        std::cout << "LAST_DISTANCE_M="
                  << last_distance_m << '\n';
        std::cout << "LAST_STRENGTH="
                  << last_strength << '\n';
        std::cout << "RANGE_MSP_BUILT="
                  << count.range_msp_built << '\n';
        std::cout << "RANGE_MSP_ERRORS="
                  << count.range_msp_errors << '\n';
        std::cout << "RANGE_MSP_TX="
                  << count.range_msp_tx << '\n';
        std::cout << "RANGE_TX="
                  << (range_tx_enabled ? 1 : 0) << '\n';
        std::cout << "FLOW_TX=0\n";

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "BF_LIVE_ERROR: "
                  << e.what() << '\n';
        return 1;
    }
}
