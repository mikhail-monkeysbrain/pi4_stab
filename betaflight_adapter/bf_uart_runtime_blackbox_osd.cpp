#include <cstdio>
#include <fstream>
// BF_UNIFIED_FLOW_TX_V3
// BF_UNIFIED_FLOW_TX_V2
#include "bf_msp_encoder.hpp"
#include "bf_flow_msp_aggregator_v60.hpp"
#include <deque>
#include <cmath>
#include "bf_msp_reader.hpp"
#include "bf_msp_warning_reader.hpp"
#include "bf_msp_scheduler.hpp"
#include "bf_sensor_state.hpp"
#include "bf_serial_port.hpp"
#include "bf_tfluna.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <array>
#include <cstring>


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
    bool flow_tx_enabled = false;
    int flow_axis_map = -1;

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);

        if (arg == "--range-tx") {
            range_tx_enabled = true;
        } else if (arg == "--flow-tx") {
            flow_tx_enabled = true;
        } else if (arg.rfind("--axis-map=", 0) == 0) {
            const std::string value = arg.substr(11);

            if (value.size() != 1 ||
                value[0] < '0' ||
                value[0] > '7') {
                std::cerr << "Invalid axis map\n";
                return 2;
            }

            flow_axis_map = value[0] - '0';
        } else {
            std::cerr << "Unknown argument: "
                      << arg << '\n';
            return 2;
        }
    }

    if (flow_tx_enabled && flow_axis_map < 0) {
        std::cerr
            << "FLOW_TX_REFUSED: axis map required\n";
        return 3;
    }

    try {
        bf_serial::Port fc("/dev/ttyAMA4", B230400);
        bf_serial::Port range("/dev/ttyAMA0", B115200);

        bf_msp::PollScheduler scheduler;
        bf_msp::MspReader reader;
        bf_msp::WarningReader warning_reader;
        bf_imu::SensorState sensors;
        bf_tfluna::Parser luna;
        Counters count;

        // BF_UNIFIED_FLOW_V1
        constexpr const char* FLOW_PATH =
            "/tmp/jtzero_bf_flow_raw.sock";

        const int flow_fd = ::socket(
            AF_UNIX,
            SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC,
            0
        );

        if (flow_fd < 0)
            throw std::runtime_error("FLOW socket failed");

        sockaddr_un flow_addr{};
        flow_addr.sun_family = AF_UNIX;

        std::strncpy(
            flow_addr.sun_path,
            FLOW_PATH,
            sizeof(flow_addr.sun_path) - 1
        );

        if (::bind(
                flow_fd,
                reinterpret_cast<sockaddr*>(&flow_addr),
                sizeof(flow_addr)) != 0) {
            ::close(flow_fd);
            throw std::runtime_error(
                "FLOW socket already in use");
        }

        bf_flow::MspAggregator flow_aggregator;
        std::deque<bf_flow::MspAggregator::Result> flow_queue;

        bool flow_started = false;

        constexpr int64_t FLOW_PERIOD_NS = 20000000LL;
        constexpr int64_t FLOW_MAX_AGE_NS = 150000000LL;

        int64_t next_flow_tx_ns = 0;
        int64_t last_sent_real_end_ns = 0;
        int64_t last_selected_end_ns = 0;
        int64_t last_flow_tx_ns = 0;

        // BF_FLOW_TX_CORRECTED
        bool flow_ready = false;
        bool flow_window_conflict = false;
        int32_t flow_ready_x = 0;
        int32_t flow_ready_y = 0;
        int64_t flow_ready_end_ns = 0;
        uint64_t flow_uart_busy = 0;

        uint64_t flow_tx_packets = 0;
        uint64_t flow_tx_invalid = 0;
        uint64_t flow_tx_valid = 0;
        uint64_t diag56_queue_empty = 0;
        uint64_t diag56_future = 0;
        int64_t diag56_next_ns = 0;
        int diag56_last_quality = -1;
        int32_t diag56_last_x = 0;
        int32_t diag56_last_y = 0;

        // JT_OSD_STATUS_V1
        int64_t last_valid_flow_tx_ns = 0;
        int64_t next_osd_tx_ns = 0;
        int64_t next_bb_osd_tx_ns = 0;
        int64_t last_quality_rx_ns = 0;
        uint8_t last_flow_quality = 0;


        uint64_t flow_ipc_rx = 0;
        uint64_t diag_agg_invalid = 0;
        uint64_t diag_timing_samples = 0;
        uint64_t diag_gap_samples = 0;
        uint64_t diag_gap_negative = 0;
        uint64_t diag_gap_zero = 0;
        uint64_t diag_gap_0_1ms = 0;
        uint64_t diag_gap_1_2ms = 0;
        uint64_t diag_gap_2_4ms = 0;
        uint64_t diag_gap_4_8ms = 0;
        uint64_t diag_gap_over_8ms = 0;
        int64_t diag_gap_sum_ns = 0;
        int64_t diag_gap_max_ns = 0;
        int64_t diag_dt_sum_ns = 0;
        int64_t diag_age_sum_ns = 0;
        int64_t diag_age_max_ns = 0;
        int64_t diag_prev_end_ns = 0;
        uint64_t diag_cov_0 = 0, diag_cov_1_25 = 0;
        uint64_t diag_cov_25_50 = 0, diag_cov_50_75 = 0;
        uint64_t diag_cov_75_98 = 0, diag_cov_98_100 = 0;
        uint64_t diag_cov_over_100 = 0;
        uint64_t diag_sched_reject = 0, diag_multi_window = 0;
        uint64_t diag_ready_conflict = 0;
        uint64_t diag_tx_irregular = 0, diag_tx_not_fresh = 0;
        uint64_t flow_ipc_rejected = 0;
        uint64_t flow_msp_built = 0;
        uint64_t flow_tx_ready_windows = 0;
        uint64_t flow_tx_dropped_windows = 0;
        int64_t flow_last_window_end_ns = 0;
        bool flow_tx_synchronized = false;
        uint64_t flow_invalid_windows = 0;
        const int ipc_fd = ::socket(
            AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (ipc_fd < 0)
            throw std::runtime_error("IMU IPC socket failed");

        sockaddr_un ipc_addr{};
        ipc_addr.sun_family = AF_UNIX;
        std::strncpy(
            ipc_addr.sun_path,
            "/tmp/jtzero_bf_imu.sock",
            sizeof(ipc_addr.sun_path) - 1);

        uint64_t ipc_sent = 0;
        uint64_t ipc_no_receiver = 0;
        uint64_t ipc_errors = 0;

        auto publishIpc = [&](const bf_msp::SensorSample& sample) {
            uint8_t packet[46]{};
            packet[0] = 1;

            std::array<float, 9> values{};

            if (sample.command == bf_msp::MSP_RAW_IMU) {
                packet[1] = 102;
                for (int i = 0; i < 9; ++i)
                    values[i] = static_cast<float>(sample.raw_imu[i]);
            } else if (sample.command == bf_msp::MSP_ATTITUDE) {
                packet[1] = 108;
                for (int i = 0; i < 3; ++i)
                    values[i] =
                        static_cast<float>(sample.attitude_rad[i]);
            } else {
                return;
            }

            const uint64_t ts =
                static_cast<uint64_t>(sample.timestamp_ns);

            for (int i = 0; i < 8; ++i)
                packet[2 + i] =
                    static_cast<uint8_t>(ts >> (8 * i));

            for (int i = 0; i < 9; ++i) {
                uint32_t bits = 0;
                std::memcpy(&bits, &values[i], sizeof(bits));
                for (int j = 0; j < 4; ++j)
                    packet[10 + i * 4 + j] =
                        static_cast<uint8_t>(bits >> (8 * j));
            }

            const ssize_t n = ::sendto(
                ipc_fd, packet, sizeof(packet), MSG_DONTWAIT,
                reinterpret_cast<const sockaddr*>(&ipc_addr),
                sizeof(ipc_addr));

            if (n == static_cast<ssize_t>(sizeof(packet)))
                ++ipc_sent;
            else if (n < 0 &&
                     (errno == ENOENT || errno == ECONNREFUSED))
                ++ipc_no_receiver;
            else
                ++ipc_errors;
        };


        const int64_t start_ns = monoNs();

        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        double last_distance_m = 0.0;
        double luna_min_m = 1e9;
        double luna_max_m = 0.0;
        uint16_t last_strength = 0;
        int64_t last_luna_ns = 0;
        int64_t next_range_tx_ns = start_ns;
        constexpr int64_t RANGE_PERIOD_NS = 10000000LL;
        constexpr int64_t RANGE_MAX_AGE_NS = 750000000LL;

        std::cout << "BF_LIVE_START\n";
        std::cout << "DURATION=UNTIL_CTRL_C\n";
        std::cout << "RANGE_TX="
                  << (range_tx_enabled ? "ENABLED" : "DISABLED")
                  << '\n';
        std::cout << "FLOW_TX=" << (flow_tx_enabled ? "ENABLED" : "DISABLED") << '\n';

        int64_t next_warning_req_ns = monoNs();

        while (!stop_requested) {
            const int64_t now = monoNs();
            const auto decision = scheduler.poll(now);

            if (decision.imu) {
                const auto request =
                    bf_msp::mspRequest(bf_msp::MSP_RAW_IMU);

                fc.enqueue(request.data(), request.size());
                ++count.imu_req;
            }

            if (decision.attitude) {
                const auto request =
                    bf_msp::mspRequest(bf_msp::MSP_ATTITUDE);

                fc.enqueue(request.data(), request.size());
                ++count.att_req;
            }

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

                warning_reader.feed(
                    buffer,
                    static_cast<std::size_t>(n)
                );
            }

            bf_msp::SensorSample sample;

            while (reader.pop(sample)) {
                sensors.update(sample);
                publishIpc(sample);

                if (sample.command ==
                    bf_msp::MSP_RAW_IMU) {
                    ++count.imu_rx;
                } else if (sample.command ==
                           bf_msp::MSP_ATTITUDE) {
                    ++count.att_rx;
                }
            }

            bf_msp::OsdWarning warning;

            while (warning_reader.pop(warning)) {
                std::cout
                    << "BF_OSD_WARNING_ATTR="
                    << int(warning.attributes)
                    << " TEXT="
                    << warning.text
                    << std::endl;
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
                                while (fc.pendingTx() != 0) {
                                    if (fc.flushTx() < 0)
                                        throw std::runtime_error(
                                            "FC UART TX error");
                                    if (fc.pendingTx() != 0)
                                        std::this_thread::sleep_for(
                                            std::chrono::milliseconds(1));
                                }
                                fc.writeDirect(
                                    packet.data(), packet.size());
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

            // RAW Optical Flow IPC v3.
            uint8_t flow_packet[128];

            for (;;) {
                const ssize_t n = ::recv(
                    flow_fd,
                    flow_packet,
                    sizeof(flow_packet),
                    MSG_DONTWAIT
                );

                if (n < 0)
                    break;

                if (n != 34 || flow_packet[0] != 3) {
                    ++flow_ipc_rejected;
                    flow_aggregator.reset();
                    flow_queue.clear();
                    flow_started = false;
                    continue;
                }

                uint64_t ts = 0;
                double dt = 0;
                double rx = 0;
                double ry = 0;

                std::memcpy(&ts, flow_packet + 1, 8);
                std::memcpy(&dt, flow_packet + 9, 8);
                std::memcpy(&rx, flow_packet + 17, 8);
                std::memcpy(&ry, flow_packet + 25, 8);

                const int64_t flow_now = monoNs();

                if (ts > static_cast<uint64_t>(flow_now) ||
                    flow_now - static_cast<int64_t>(ts) >
                        150000000LL ||
                    flow_packet[33] == 0 ||
                    !std::isfinite(dt) ||
                    !std::isfinite(rx) ||
                    !std::isfinite(ry) ||
                    dt <= 0.0 || dt > 0.1) {

                    ++flow_ipc_rejected;
                    flow_aggregator.reset();
                    flow_queue.clear();
                    flow_started = false;
                    continue;
                }

                if (!flow_started) {
                    const int64_t frame_start =
                        static_cast<int64_t>(ts) -
                        static_cast<int64_t>(
                            std::llround(dt * 1e9));

                    flow_aggregator.begin(frame_start);
                    flow_started = true;
                }

                // JT_FLOW_TIMING_SAMPLE_V1
                {
                    const int64_t t1 = static_cast<int64_t>(ts);
                    const int64_t duration =
                        static_cast<int64_t>(std::llround(dt * 1e9));
                    const int64_t t0 = t1 - duration;
                    const int64_t age = monoNs() - t1;

                    ++diag_timing_samples;
                    diag_dt_sum_ns += duration;
                    diag_age_sum_ns += age;
                    diag_age_max_ns = std::max(diag_age_max_ns, age);

                    if (diag_prev_end_ns != 0) {
                        const int64_t gap = t0 - diag_prev_end_ns;
                        ++diag_gap_samples;
                        diag_gap_sum_ns += gap;
                        diag_gap_max_ns = std::max(diag_gap_max_ns, gap);

                        if (gap < 0) ++diag_gap_negative;
                        else if (gap == 0) ++diag_gap_zero;
                        else if (gap <= 1000000LL) ++diag_gap_0_1ms;
                        else if (gap <= 2000000LL) ++diag_gap_1_2ms;
                        else if (gap <= 4000000LL) ++diag_gap_2_4ms;
                        else if (gap <= 8000000LL) ++diag_gap_4_8ms;
                        else ++diag_gap_over_8ms;
                    }

                    diag_prev_end_ns = t1;
                }

                if (!flow_aggregator.add(
                        rx, ry, dt,
                        static_cast<int64_t>(ts))) {

                    ++flow_ipc_rejected;
                    flow_queue.clear();
                    flow_started = false;
                    continue;
                }

                last_flow_quality = flow_packet[33];
                last_quality_rx_ns = flow_now;
                ++flow_ipc_rx;
            }


            bf_flow::MspAggregator::Result result;

            while (flow_aggregator.take(result)) {
                if (result.valid) {
                    flow_queue.push_back(result);
                    ++flow_tx_ready_windows;
                } else {
                    ++flow_invalid_windows;
                    ++diag_agg_invalid;
                }
            }

            // V6.0: real 20 ms optical-flow windows.
            if (flow_tx_enabled) {
                const int64_t tx_now = monoNs();
                constexpr int64_t DELIVERY_DELAY_NS = 140000000LL;

                if (next_flow_tx_ns == 0)
                    next_flow_tx_ns =
                        ((tx_now / FLOW_PERIOD_NS) + 1) *
                        FLOW_PERIOD_NS + DELIVERY_DELAY_NS;

                if (tx_now >= next_flow_tx_ns) {
                    const int64_t expected_end =
                        next_flow_tx_ns - DELIVERY_DELAY_NS;

                    bool valid = false;
                    int32_t x = 0, y = 0;

                    // V6.0 FIFO: one real window per TX slot.
                    // Discard only already-sent or stale windows.
                    while (!flow_queue.empty()) {
                        const auto &sample = flow_queue.front();

                        if (sample.end_ns > tx_now)
                            break;

                        if (sample.end_ns <= last_sent_real_end_ns ||
                            tx_now - sample.end_ns > FLOW_MAX_AGE_NS) {
                            flow_queue.pop_front();
                            ++flow_tx_dropped_windows;
                            continue;
                        }

                        x = sample.x;
                        y = sample.y;
                        last_selected_end_ns = sample.end_ns;
                        flow_queue.pop_front();
                        valid = true;
                        break;
                    }

                    if (!valid)
                        ++diag56_queue_empty;

                    if (fc.pendingTx() == 0) {
                        if (valid) {
                            if (flow_axis_map & 1)
                                std::swap(x, y);
                            if (flow_axis_map & 2)
                                x = -x;
                            if (flow_axis_map & 4)
                                y = -y;
                        }

                        const auto packet = bf_msp::opticalflow(
                            valid ? x : 0,
                            valid ? y : 0,
                            valid ? 255 : 0
                        );

                        fc.writeDirect(packet.data(), packet.size());

                        ++flow_tx_packets;
                        diag56_last_x = valid ? x : 0;
                        diag56_last_y = valid ? y : 0;
                        diag56_last_quality = valid ? 255 : 0;

                        if (valid) {
                            ++flow_tx_valid;
                            last_sent_real_end_ns = last_selected_end_ns;
                            last_valid_flow_tx_ns = monoNs();
                        } else {
                            ++flow_tx_invalid;
                        }

                        last_flow_tx_ns = monoNs();
                    } else {
                        ++flow_uart_busy;
                        ++flow_tx_invalid;
                    }

                    next_flow_tx_ns += FLOW_PERIOD_NS;

                    if (tx_now >= next_flow_tx_ns)
                        next_flow_tx_ns =
                            ((tx_now / FLOW_PERIOD_NS) + 1) *
                            FLOW_PERIOD_NS + DELIVERY_DELAY_NS;
                }
            }

            const int64_t diag_now = monoNs();
            if (diag56_next_ns == 0 || diag_now >= diag56_next_ns) {
                diag56_next_ns = diag_now + 1000000000LL;
                std::cout
                    << "BF_FLOW_DIAG_V56"
                    << " IPC_RX=" << flow_ipc_rx
                    << " IPC_REJECT=" << flow_ipc_rejected
                    << " READY=" << flow_tx_ready_windows
                    << " INVALID_WINDOWS=" << flow_invalid_windows
                    << " DROPPED=" << flow_tx_dropped_windows
                    << " QUEUE_EMPTY=" << diag56_queue_empty
                    << " FUTURE=" << diag56_future
                    << " TX_TOTAL=" << flow_tx_packets
                    << " TX_VALID=" << flow_tx_valid
                    << " TX_INVALID=" << flow_tx_invalid
                    << " UART_BUSY=" << flow_uart_busy
                    << " X=" << diag56_last_x
                    << " Y=" << diag56_last_y
                    << " QUALITY=" << diag56_last_quality
                    << std::endl;
            }

            // JT_OSD_STATUS_V2: alternate two CUSTOM_MSG fields.
            // Total OSD transmission rate <= 1 Hz.
            const int64_t osd_now = monoNs();

            if (osd_now >= next_osd_tx_ns) {
                const bool flow_fresh =
                    flow_tx_enabled &&
                    last_valid_flow_tx_ns != 0 &&
                    osd_now - last_valid_flow_tx_ns <= 300000000LL;

                const bool range_fresh =
                    range_tx_enabled &&
                    last_luna_ns != 0 &&
                    osd_now - last_luna_ns <= RANGE_MAX_AGE_NS &&
                    count.range_msp_tx > 0;

                const bool quality_fresh =
                    flow_fresh &&
                    last_quality_rx_ns != 0 &&
                    osd_now - last_quality_rx_ns <= 300000000LL;

                const char* status = nullptr;
                const uint8_t text_id = 8;
                char quality_status[32]{};

                if (!flow_fresh) {
                    status = "PH NO FLOW";
                } else if (!range_fresh) {
                    status = "PH NO RANGE";
                } else if (!quality_fresh) {
                    status = "PH FLOW INVALID";
                } else {
                    const unsigned percent =
                        (static_cast<unsigned>(last_flow_quality)
                         * 100u + 127u) / 255u;

                    std::snprintf(
                        quality_status,
                        sizeof(quality_status),
                        "OF - %u%%",
                        percent
                    );
                    status = quality_status;
                }

                const size_t n = std::strlen(status);
                std::array<uint8_t, 64> frame{};
                size_t pos = 0;

                frame[pos++] = '$';
                frame[pos++] = 'X';
                frame[pos++] = '<';
                frame[pos++] = 0;
                frame[pos++] = 0x07;
                frame[pos++] = 0x30;
                frame[pos++] = static_cast<uint8_t>(n + 2);
                frame[pos++] = 0;
                frame[pos++] = text_id;
                frame[pos++] = static_cast<uint8_t>(n);

                for (size_t i = 0; i < n; ++i)
                    frame[pos++] = static_cast<uint8_t>(status[i]);

                uint8_t crc = 0;
                for (size_t i = 3; i < pos; ++i) {
                    crc ^= frame[i];
                    for (int bit = 0; bit < 8; ++bit)
                        crc = (crc & 0x80)
                            ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                            : static_cast<uint8_t>(crc << 1);
                }
                frame[pos++] = crc;

                // OSD is low priority. Never block OF transmission.
                if (fc.pendingTx() == 0 &&
                    (!flow_tx_enabled ||
                     next_flow_tx_ns - monoNs() > 15000000LL)) {
                    fc.writeDirect(frame.data(), pos);
                    next_osd_tx_ns = monoNs() + 500000000LL;
                }
            }


            // JT_BLACKBOX_OSD_V1
            // CUSTOM_MSG3: MSP2_SET_TEXT, text ID 9.
            // Independent best-effort 2 Hz update.
            const int64_t bb_now_ns = monoNs();

            if (bb_now_ns >= next_bb_osd_tx_ns) {
                char bb_text[32] = "BB OFF";

                std::ifstream status_file(
                    "/home/vio/monkeysStab_runs/blackbox/blackbox_status.txt"
                );

                if (status_file) {
                    std::string state;
                    double published = 0.0;
                    double started = 0.0;
                    double last = 0.0;
                    unsigned long long bytes = 0;

                    if (status_file >> state >> published
                                    >> started >> last >> bytes) {

                        const double now =
                            double(bb_now_ns) / 1e9;

                        if (published > 0.0 &&
                            now >= published &&
                            now - published <= 1.5) {

                            if (state == "REC" &&
                                started > 0.0 &&
                                last > 0.0 &&
                                now - last <= 1.0 &&
                                bytes > 0) {

                                const unsigned elapsed =
                                    static_cast<unsigned>(
                                        now - started
                                    );

                                std::snprintf(
                                    bb_text,
                                    sizeof(bb_text),
                                    "BB REC %02u:%02u",
                                    elapsed / 60,
                                    elapsed % 60
                                );

                            } else if (state == "LOST" ||
                                       state == "REC") {
                                std::snprintf(
                                    bb_text,
                                    sizeof(bb_text),
                                    "BB LOST"
                                );
                            } else if (state == "STOP") {
                                std::snprintf(
                                    bb_text,
                                    sizeof(bb_text),
                                    "BB STOP"
                                );
                            }
                        } else {
                            std::snprintf(
                                bb_text,
                                sizeof(bb_text),
                                "BB OFF"
                            );
                        }
                    }
                }

                const size_t n = std::strlen(bb_text);

                std::array<uint8_t, 64> frame{};
                size_t pos = 0;

                frame[pos++] = '$';
                frame[pos++] = 'X';
                frame[pos++] = '<';
                frame[pos++] = 0;
                frame[pos++] = 0x07;
                frame[pos++] = 0x30;
                frame[pos++] = static_cast<uint8_t>(n + 2);
                frame[pos++] = 0;
                frame[pos++] = 9;
                frame[pos++] = static_cast<uint8_t>(n);

                for (size_t i = 0; i < n; ++i)
                    frame[pos++] =
                        static_cast<uint8_t>(bb_text[i]);

                uint8_t crc = 0;

                for (size_t i = 3; i < pos; ++i) {
                    crc ^= frame[i];

                    for (int bit = 0; bit < 8; ++bit) {
                        crc = (crc & 0x80)
                            ? static_cast<uint8_t>(
                                (crc << 1) ^ 0xD5
                              )
                            : static_cast<uint8_t>(
                                crc << 1
                              );
                    }
                }

                frame[pos++] = crc;

                // Optical Flow remains higher priority.
                if (fc.pendingTx() == 0 &&
                    (!flow_tx_enabled ||
                     next_flow_tx_ns - monoNs() >
                         15000000LL)) {

                    fc.writeDirect(frame.data(), pos);

                    next_bb_osd_tx_ns =
                        monoNs() + 500000000LL;
                }
            }

            // BF MSPv2 OSD warning diagnostic.
            // Low-priority request, max 1 Hz.
            const int64_t warning_now = monoNs();

            if (warning_now >= next_warning_req_ns) {
                next_warning_req_ns =
                    warning_now + 1000000000LL;

                if (fc.pendingTx() == 0 &&
                    (!flow_tx_enabled ||
                     next_flow_tx_ns - monoNs() > 15000000LL)) {

                    const auto request =
                        bf_msp::frame(0x3005, {});

                    fc.writeDirect(
                        request.data(),
                        request.size()
                    );
                }
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(1)
            );
        }

        const auto snapshot = sensors.snapshot(monoNs());

        const double duration_s =
            double(monoNs() - start_ns) / 1e9;

        std::cout << "FLOW_IPC_RX="
                  << flow_ipc_rx << '\n';
        std::cout << "FLOW_IPC_REJECTED="
                  << flow_ipc_rejected << '\n';
        std::cout << "FLOW_MSP_BUILT="
                  << flow_msp_built << '\n';
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

        std::cout << "FLOW_TX_READY_WINDOWS="
                  << flow_tx_ready_windows << '\n';
        std::cout << "FLOW_TX_DROPPED_WINDOWS="
                  << flow_tx_dropped_windows << '\n';
        std::cout << "FLOW_TX_SYNCHRONIZED="
                  << flow_tx_synchronized << '\n';
        std::cout << "FLOW_UART_BUSY=" << flow_uart_busy << '\n';
        std::cout << "FLOW_TX_PACKETS="
                  << flow_tx_packets << '\n';
        std::cout << "FLOW_TX_VALID="
                  << flow_tx_valid << '\n';
        std::cout << "FLOW_TX_INVALID="
                  << flow_tx_invalid << '\n';
        std::cout << "FLOW_TX_ENABLED="
                  << flow_tx_enabled << '\n';
        std::cout << "FLOW_AXIS_MAP="
                  << flow_axis_map << '\n';
        std::cout << "IPC_SENT=" << ipc_sent << '\n';
        std::cout << "IPC_NO_RECEIVER=" << ipc_no_receiver << '\n';
        std::cout << "IPC_ERRORS=" << ipc_errors << '\n';
        ::close(ipc_fd);


        return 0;
    } catch (const std::exception& e) {
        std::cerr << "BF_LIVE_ERROR: "
                  << e.what() << '\n';
        return 1;
    }
}
