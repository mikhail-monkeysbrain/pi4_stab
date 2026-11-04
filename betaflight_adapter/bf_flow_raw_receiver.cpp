#include "bf_flow_msp_aggregator.hpp"
#include "bf_msp_encoder.hpp"
#include "bf_flow_msp_tx_scheduler.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

static int64_t monoNs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int main() {
    constexpr const char* PATH =
        "/tmp/jtzero_bf_flow_raw.sock";

    const int fd = socket(
        AF_UNIX,
        SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC,
        0
    );

    if (fd < 0)
        throw std::runtime_error("socket failed");

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;

    std::strncpy(
        addr.sun_path,
        PATH,
        sizeof(addr.sun_path) - 1
    );

    if (bind(fd,
             reinterpret_cast<sockaddr*>(&addr),
             sizeof(addr)) != 0) {
        close(fd);
        throw std::runtime_error("RAW socket already in use");
    }

    bf_flow::MspAggregator aggregator;
    bf_flow::MspTxScheduler scheduler;

    int accepted = 0;
    int rejected = 0;
    int msp_built = 0;
    int windows_invalid = 0;

    bool started = false;

    const int64_t end_time =
        monoNs() + 10000000000LL;

    while (monoNs() < end_time) {
        uint8_t packet[128];

        for (;;) {
            const ssize_t n = recv(
                fd,
                packet,
                sizeof(packet),
                MSG_DONTWAIT
            );

            if (n < 0)
                break;

            if (n != 34 || packet[0] != 3) {
                rejected++;
                continue;
            }

            uint64_t ts = 0;
            double dt = 0;
            double rx = 0;
            double ry = 0;

            std::memcpy(&ts, packet + 1, 8);
            std::memcpy(&dt, packet + 9, 8);
            std::memcpy(&rx, packet + 17, 8);
            std::memcpy(&ry, packet + 25, 8);

            const int64_t now = monoNs();

            if (ts > static_cast<uint64_t>(now) ||
                now - static_cast<int64_t>(ts) > 150000000LL ||
                packet[33] == 0 ||
                !std::isfinite(dt) ||
                !std::isfinite(rx) ||
                !std::isfinite(ry)) {
                rejected++;
                aggregator.reset();
                scheduler.reset();
                started = false;
                continue;
            }

            if (!started) {
                const int64_t frame_start =
                    static_cast<int64_t>(ts) -
                    static_cast<int64_t>(
                        std::llround(dt * 1e9));

                aggregator.begin(frame_start);
                started = true;
            }

            if (!aggregator.add(
                    rx, ry, dt,
                    static_cast<int64_t>(ts))) {
                rejected++;
                scheduler.reset();
                started = false;
                continue;
            }

            accepted++;

            bf_flow::MspAggregator::Result result;

            while (aggregator.take(result)) {
                if (!result.valid) {
                    windows_invalid++;
                    scheduler.reset();
                    continue;
                }

                if (!scheduler.add(result)) {
                    windows_invalid++;
                    continue;
                }

                bf_flow::MspTxScheduler::Packet tx;

                if (scheduler.build(tx)) {
                    const auto frame =
                        bf_msp::opticalflow(
                            tx.x,
                            tx.y,
                            tx.quality
                        );

                    (void)frame;
                    msp_built++;
                }
            }
        }

        if (started) {
            aggregator.flush(monoNs());

            bf_flow::MspAggregator::Result result;

            while (aggregator.take(result)) {
                if (!result.valid) {
                    windows_invalid++;
                    scheduler.reset();
                    continue;
                }

                if (!scheduler.add(result)) {
                    windows_invalid++;
                    continue;
                }

                bf_flow::MspTxScheduler::Packet tx;

                if (scheduler.build(tx)) {
                    const auto frame =
                        bf_msp::opticalflow(
                            tx.x,
                            tx.y,
                            tx.quality
                        );

                    (void)frame;
                    msp_built++;
                }
            }
        }

        pollfd pfd{fd, POLLIN, 0};
        poll(&pfd, 1, 2);
    }

    close(fd);
    unlink(PATH);

    std::cout
        << "BF_RAW_IPC_RESULT"
        << " accepted=" << accepted
        << " rejected=" << rejected
        << " msp_built=" << msp_built
        << " windows_invalid=" << windows_invalid
        << " flow_tx=0"
        << std::endl;

    return 0;
}
