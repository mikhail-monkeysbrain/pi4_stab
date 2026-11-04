#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace bf_serial {

class Port {
public:
    Port(const std::string& path, speed_t baud) {
        fd_ = ::open(
            path.c_str(),
            O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC
        );

        if (fd_ < 0)
            throw std::runtime_error("Serial open failed");

        termios tty{};

        if (tcgetattr(fd_, &tty) != 0) {
            close();
            throw std::runtime_error("tcgetattr failed");
        }

        cfmakeraw(&tty);

        if (cfsetispeed(&tty, baud) != 0 ||
            cfsetospeed(&tty, baud) != 0) {
            close();
            throw std::runtime_error("Serial baud failed");
        }

        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close();
            throw std::runtime_error("tcsetattr failed");
        }
    }

    ~Port() {
        close();
    }

    Port(const Port&) = delete;
    Port& operator=(const Port&) = delete;

    ssize_t read(uint8_t* dst, std::size_t capacity) {
        while (true) {
            const ssize_t n = ::read(fd_, dst, capacity);

            if (n < 0 && errno == EINTR)
                continue;

            if (n < 0 &&
                (errno == EAGAIN || errno == EWOULDBLOCK))
                return 0;

            return n;
        }
    }


    void writeDirect(const uint8_t* data, std::size_t size) {
        std::size_t offset = 0;
        while (offset < size) {
            const ssize_t n = ::write(
                fd_, data + offset, size - offset);

            if (n > 0) {
                offset += static_cast<std::size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR)
                continue;
            if (n < 0 &&
                (errno == EAGAIN || errno == EWOULDBLOCK)) {
                ::usleep(1000);
                continue;
            }
            throw std::runtime_error("Direct UART write failed");
        }
    }

    void enqueue(const uint8_t* data, std::size_t size) {
        if (size > 4096 - tx_.size())
            throw std::runtime_error("Serial TX queue full");

        for (std::size_t i = 0; i < size; ++i)
            tx_.push_back(data[i]);
    }

    ssize_t flushTx() {
        ssize_t total = 0;

        while (!tx_.empty()) {
            uint8_t chunk[256];
            std::size_t count = 0;

            for (auto it = tx_.begin();
                 it != tx_.end() && count < sizeof(chunk);
                 ++it) {
                chunk[count++] = *it;
            }

            const ssize_t n = ::write(fd_, chunk, count);

            if (n < 0 && errno == EINTR)
                continue;

            if (n < 0 &&
                (errno == EAGAIN || errno == EWOULDBLOCK))
                break;

            if (n <= 0)
                return n < 0 ? n : total;

            for (ssize_t i = 0; i < n; ++i)
                tx_.pop_front();

            total += n;
        }

        return total;
    }

    std::size_t pendingTx() const {
        return tx_.size();
    }

private:
    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    int fd_ = -1;
    std::deque<uint8_t> tx_;
};

} // namespace bf_serial
