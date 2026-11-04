#include "bf_msp_encoder.hpp"
#include "bf_tfluna.hpp"

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

static volatile sig_atomic_t stop_requested = 0;

extern "C" void on_signal(int) {
    stop_requested = 1;
}

static double monotonic_seconds() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return double(ts.tv_sec) + double(ts.tv_nsec) / 1e9;
}

static int open_uart(const char* path, speed_t baud) {
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        throw std::runtime_error(std::strerror(errno));

    termios cfg{};
    if (tcgetattr(fd, &cfg) != 0) {
        close(fd);
        throw std::runtime_error("tcgetattr failed");
    }

    cfmakeraw(&cfg);
    cfsetispeed(&cfg, baud);
    cfsetospeed(&cfg, baud);
    cfg.c_cflag |= CLOCAL | CREAD;
    cfg.c_cflag &= ~CRTSCTS;
    cfg.c_cc[VMIN] = 0;
    cfg.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &cfg) != 0) {
        close(fd);
        throw std::runtime_error("tcsetattr failed");
    }

    return fd;
}

int main() {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    int fc = -1;
    int luna_fd = -1;

    uint64_t luna_rx = 0;
    uint64_t packets_tx = 0;
    uint64_t bytes_tx = 0;

    try {
        fc = open_uart("/dev/ttyAMA4", B230400);
        luna_fd = open_uart("/dev/serial0", B115200);

        bf_tfluna::Parser parser;
        uint8_t buffer[512];

        double last_distance = 0;
        double last_update = 0;
        double next_tx = monotonic_seconds();

        std::cout << "BF_RANGE_DIRECT_START\n";
        std::cout << "MODE=REAL_TFLUNA\n";
        std::cout << "RATE=10HZ\n";
        std::cout << "IMU_POLL=DISABLED\n";
        std::cout << "FLOW_TX=DISABLED\n";
        std::cout << "STOP=CTRL+C\n" << std::flush;

        while (!stop_requested) {
            ssize_t n = read(luna_fd, buffer, sizeof(buffer));

            if (n > 0) {
                parser.feed(buffer, static_cast<size_t>(n));
            } else if (n < 0 &&
                       errno != EAGAIN &&
                       errno != EWOULDBLOCK &&
                       errno != EINTR) {
                throw std::runtime_error("TF-Luna read failed");
            }

            bf_tfluna::Measurement m;

            while (parser.pop(m)) {
                last_distance = m.distance_m;
                last_update = monotonic_seconds();
                ++luna_rx;

                std::cout << "LUNA_CM="
                          << last_distance * 100.0
                          << '\n';
            }

            const double now = monotonic_seconds();

            if (now >= next_tx) {
                next_tx = now + 0.1;

                if (last_update > 0 &&
                    now - last_update <= 0.75) {

                    const auto packet =
                        bf_msp::rangefinder(last_distance, 255);

                    size_t offset = 0;

                    while (offset < packet.size()) {
                        ssize_t written = write(
                            fc,
                            packet.data() + offset,
                            packet.size() - offset
                        );

                        if (written > 0) {
                            offset += static_cast<size_t>(written);
                            bytes_tx += static_cast<uint64_t>(written);
                        } else if (written < 0 &&
                                   errno == EINTR) {
                            continue;
                        } else if (written < 0 &&
                                   (errno == EAGAIN ||
                                    errno == EWOULDBLOCK)) {
                            usleep(1000);
                        } else {
                            throw std::runtime_error(
                                "FC UART write failed"
                            );
                        }
                    }

                    ++packets_tx;

                    if (packets_tx % 10 == 0) {
                        std::cout << "TX_CM="
                                  << last_distance * 100.0
                                  << " PACKETS=" << packets_tx
                                  << '\n';
                    }
                }
            }

            usleep(1000);
        }

        std::cout << "===== RESULT =====\n";
        std::cout << "LUNA_RX=" << luna_rx << '\n';
        std::cout << "PACKETS_TX=" << packets_tx << '\n';
        std::cout << "BYTES_TX=" << bytes_tx << '\n';
        std::cout << "FLOW_TX=0\n";

        close(luna_fd);
        close(fc);
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        if (luna_fd >= 0) close(luna_fd);
        if (fc >= 0) close(fc);
        return 1;
    }
}
