#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <array>
#include "bf_imu_adapter.hpp"

static constexpr const char* SOCKET_PATH =
    "/tmp/jtzero_bf_imu.sock";

static constexpr size_t PACKET_SIZE = 46;

static uint64_t readU64LE(const uint8_t* p) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(p[i]) << (8 * i);
    }
    return value;
}

static float readF32LE(const uint8_t* p) {
    uint32_t bits = 0;
    for (int i = 0; i < 4; ++i) {
        bits |= static_cast<uint32_t>(p[i]) << (8 * i);
    }
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

int main() {
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);

    if (fd < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(
        addr.sun_path,
        SOCKET_PATH,
        sizeof(addr.sun_path) - 1
    );

    unlink(SOCKET_PATH);

    if (bind(fd,
             reinterpret_cast<sockaddr*>(&addr),
             sizeof(addr)) < 0) {
        perror("bind");
        close(fd);
        return 1;
    }

    bf_imu::Adapter adapter;

    uint64_t imu_count = 0;
    uint64_t att_count = 0;
    uint64_t invalid_count = 0;

    auto last_report = std::chrono::steady_clock::now();

    std::cout << "JT-Zero BF IPC receiver\n";
    std::cout << "Socket: " << SOCKET_PATH << "\n";
    std::cout << "Waiting for Python MSP runtime..."
              << std::endl;

    while (true) {
        uint8_t packet[128];

        ssize_t n = recv(
            fd,
            packet,
            sizeof(packet),
            MSG_DONTWAIT
        );

        if (n > 0) {
            if (n != static_cast<ssize_t>(PACKET_SIZE) ||
                packet[0] != 1) {
                ++invalid_count;
            } else {
                const uint8_t type = packet[1];
                const uint64_t timestamp_ns =
                    readU64LE(packet + 2);

                float values[9];

                for (int i = 0; i < 9; ++i) {
                    values[i] = readF32LE(
                        packet + 10 + i * 4
                    );
                }

                if (type == 102) {
                    std::array<float, 9> raw{};
                    for (int i = 0; i < 9; ++i)
                        raw[i] = values[i];

                    adapter.updateRawImu(raw, timestamp_ns);
                    ++imu_count;

                } else if (type == 108) {
                    adapter.updateAttitude(
                        {values[0], values[1], values[2]},
                        timestamp_ns
                    );
                    ++att_count;

                } else {
                    ++invalid_count;
                }

                (void)timestamp_ns;
                (void)values;
            }
        } else if (n < 0 &&
                   errno != EAGAIN &&
                   errno != EWOULDBLOCK &&
                   errno != EINTR) {
            perror("recv");
            break;
        }

        const auto now =
            std::chrono::steady_clock::now();

        if (now - last_report >=
            std::chrono::seconds(1)) {
            std::cout
                << "IPC_IMU=" << imu_count
                << " IPC_ATT=" << att_count
                << " IPC_INVALID=" << invalid_count
                << std::endl;

            const auto now_ns =
                std::chrono::duration_cast<
                    std::chrono::nanoseconds>(
                    now.time_since_epoch()
                ).count();

            const auto snap = adapter.snapshot(now_ns);

            constexpr double RAD_TO_DEG =
                57.29577951308232;

            if (snap.imu.valid) {
                std::cout
                    << "GYRO_RAW="
                    << snap.imu.gyro[0] << ","
                    << snap.imu.gyro[1] << ","
                    << snap.imu.gyro[2]
                    << " AGE_MS="
                    << (now_ns - snap.imu.recv_ns) / 1e6
                    << " FRESH=" << snap.imu_fresh
                    << std::endl;
            }

            if (snap.attitude.valid) {
                std::cout
                    << "ATT_DEG="
                    << snap.attitude.roll * RAD_TO_DEG << ","
                    << snap.attitude.pitch * RAD_TO_DEG << ","
                    << snap.attitude.yaw * RAD_TO_DEG
                    << " AGE_MS="
                    << (now_ns - snap.attitude.recv_ns) / 1e6
                    << " FRESH=" << snap.attitude_fresh
                    << std::endl;
            }

            last_report = now;
        }

        usleep(1000);
    }

    close(fd);
    unlink(SOCKET_PATH);
    return 0;
}
