#include "bf_msp_encoder.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iostream>
#include <stdexcept>

static int64_t monoNs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int main() {
    constexpr const char* path = "/tmp/jtzero_bf_flow.sock";

    int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return 1;

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    // Never remove an existing socket automatically.
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr),
             sizeof(addr)) != 0) {
        std::cerr << "FLOW_IPC_BIND_FAILED errno=" << errno << '\n';
        close(fd);
        return 2;
    }

    bf_msp::AngularMotionQuantizer quantizer;
    uint64_t accepted = 0, rejected = 0, built = 0;

    std::cout << "FLOW_IPC_DRYRUN_START FLOW_TX=0\n";

    const int64_t deadline = monoNs() + 10000000000LL;

    while (monoNs() < deadline) {
        uint8_t packet[34]{};

        const ssize_t n = recv(fd, packet, sizeof(packet),
                               MSG_DONTWAIT);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(1000);
                continue;
            }
            ++rejected;
            break;
        }

        if (n != 34 || packet[0] != 2) {
            ++rejected;
            continue;
        }

        uint64_t ts = 0;
        double dt = 0, rx = 0, ry = 0;

        std::memcpy(&ts, packet + 1, 8);
        std::memcpy(&dt, packet + 9, 8);
        std::memcpy(&rx, packet + 17, 8);
        std::memcpy(&ry, packet + 25, 8);

        const int64_t age = monoNs() - static_cast<int64_t>(ts);

        if (age < 0 || age > 150000000LL ||
            !std::isfinite(dt) || dt <= 0 || dt >= 1 ||
            !std::isfinite(rx) || !std::isfinite(ry) ||
            packet[33] == 0) {
            ++rejected;
            continue;
        }

        ++accepted;

        try {
            int32_t x = 0, y = 0;
            quantizer.encode(rx, ry, dt, x, y);

            const auto msp = bf_msp::opticalflow(x, y, packet[33]);

            if (msp.size() == 18) {
                ++built;
                if (built <= 5)
                    std::cout << "FLOW_MSP_DRYRUN"
                              << " x=" << x << " y=" << y
                              << " quality=" << int(packet[33])
                              << " bytes=" << msp.size() << '\n';
            } else {
                ++rejected;
            }
        } catch (const std::exception&) {
            ++rejected;
        }
    }

    std::cout << "FLOW_IPC_DRYRUN_RESULT"
              << " accepted=" << accepted
              << " rejected=" << rejected
              << " msp_built=" << built
              << " flow_tx=0\n";

    close(fd);
    unlink(path);
}
