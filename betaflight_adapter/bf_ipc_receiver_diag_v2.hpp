#pragma once

#include "bf_imu_adapter.hpp"
#include "bf_imu_history_diag_v2.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace bf_imu {

class BfIpcReceiver {
public:
    static constexpr const char* DEFAULT_PATH =
        "/tmp/jtzero_bf_imu.sock";

    explicit BfIpcReceiver(
        const std::string& path = DEFAULT_PATH
    ) : path_(path) {
        fd_ = socket(
            AF_UNIX,
            SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC,
            0
        );

        if (fd_ < 0)
            throw std::runtime_error("BF IPC socket failed");

        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;

        if (path_.size() >= sizeof(addr.sun_path)) {
            close(fd_);
            throw std::runtime_error("BF IPC path too long");
        }

        std::memcpy(
            addr.sun_path,
            path_.c_str(),
            path_.size() + 1
        );

        if (bind(
            fd_,
            reinterpret_cast<sockaddr*>(&addr),
            sizeof(addr)
        ) < 0) {
            const int err = errno;
            close(fd_);
            fd_ = -1;
            throw std::runtime_error(
                "BF IPC bind failed, errno=" +
                std::to_string(err)
            );
        }

        struct stat st{};
        if (lstat(path_.c_str(), &st) == 0) {
            socket_dev_ = st.st_dev;
            socket_ino_ = st.st_ino;
        }
    }

    ~BfIpcReceiver() {
        if (fd_ >= 0)
            close(fd_);

        struct stat st{};

        if (socket_ino_ != 0 &&
            lstat(path_.c_str(), &st) == 0 &&
            st.st_dev == socket_dev_ &&
            st.st_ino == socket_ino_ &&
            S_ISSOCK(st.st_mode)) {
            unlink(path_.c_str());
        }
    }

    BfIpcReceiver(const BfIpcReceiver&) = delete;
    BfIpcReceiver& operator=(const BfIpcReceiver&) = delete;

    void poll() {
        uint8_t packet[128];

        while (true) {
            const ssize_t n = recv(
                fd_,
                packet,
                sizeof(packet),
                MSG_DONTWAIT
            );

            if (n < 0) {
                if (errno == EINTR)
                    continue;

                if (errno == EAGAIN ||
                    errno == EWOULDBLOCK)
                    break;

                ++errors_;
                break;
            }

            if (n != 46 || packet[0] != 1) {
                ++invalid_;
                continue;
            }

            const uint8_t type = packet[1];
            const uint64_t timestamp_ns =
                readU64(packet + 2);

            std::array<float, 9> values{};

            for (int i = 0; i < 9; ++i)
                values[i] = readF32(packet + 10 + i * 4);

            if (type == 102) {
                adapter_.updateRawImu(
                    values,
                    static_cast<int64_t>(timestamp_ns)
                );
                history_.push(
                    static_cast<int64_t>(timestamp_ns),
                    {values[3], values[4], values[5]}
                );
                ++imu_count_;

            } else if (type == 108) {
                adapter_.updateAttitude(
                    {values[0], values[1], values[2]},
                    static_cast<int64_t>(timestamp_ns)
                );
                ++att_count_;

            } else {
                ++invalid_;
            }
        }
    }

    GyroInterval gyroInterval(
        int64_t t0, int64_t t1
    ) const {
        return history_.inspect(t0, t1);
    }

    GyroHistory::GapStats gyroGapStats() const {
        return history_.gapStats();
    }

    std::size_t gyroHistorySize() const {
        return history_.size();
    }

    bool gyroRotation(
        int64_t t0,
        int64_t t1,
        double gyro_scale_rad_s,
        cv::Matx33d& R
    ) const {
        return history_.integrateRotation(
            t0, t1, gyro_scale_rad_s, R
        );
    }

    Snapshot snapshot(int64_t now_ns) const {
        return adapter_.snapshot(now_ns);
    }

    uint64_t imuCount() const { return imu_count_; }
    uint64_t attCount() const { return att_count_; }
    uint64_t invalidCount() const { return invalid_; }
    uint64_t errorCount() const { return errors_; }

private:
    static uint64_t readU64(const uint8_t* p) {
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i)
            v |= uint64_t(p[i]) << (8 * i);
        return v;
    }

    static float readF32(const uint8_t* p) {
        uint32_t bits = 0;
        for (int i = 0; i < 4; ++i)
            bits |= uint32_t(p[i]) << (8 * i);

        float value;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    std::string path_;
    int fd_ = -1;
    dev_t socket_dev_ = 0;
    ino_t socket_ino_ = 0;

    Adapter adapter_;
    GyroHistory history_;

    uint64_t imu_count_ = 0;
    uint64_t att_count_ = 0;
    uint64_t invalid_ = 0;
    uint64_t errors_ = 0;
};

}  // namespace bf_imu
