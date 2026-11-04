#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <linux/videodev2.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <iostream>
#include <ctime>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace bf_camera {

inline void check(bool ok, const char* msg) {
    if (!ok)
        throw std::runtime_error(
            std::string(msg) + ": " + std::strerror(errno));
}

inline int ioctl_retry(int fd, unsigned long req, void* arg) {
    int rc;
    do {
        rc = ioctl(fd, req, arg);
    } while (rc < 0 && errno == EINTR);
    return rc;
}

struct Frame {
    cv::Mat gray;
    int64_t timestamp_ns = 0;
    uint32_t sequence = 0;
};

class Camera {
public:
    Camera() = default;
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    ~Camera() {
        close();
    }

    void open(
        const std::string& device,
        int width = 640,
        int height = 480,
        int fps = 60
    ) {
        if (fd_ >= 0)
            throw std::runtime_error("Camera already open");

        try {
            fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK);
            check(fd_ >= 0, "open camera");

            v4l2_capability cap{};
            check(ioctl_retry(fd_, VIDIOC_QUERYCAP, &cap) == 0,
                  "VIDIOC_QUERYCAP");

            const uint32_t caps =
                (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
                ? cap.device_caps : cap.capabilities;

            if (!(caps & V4L2_CAP_VIDEO_CAPTURE) ||
                !(caps & V4L2_CAP_STREAMING))
                throw std::runtime_error(
                    "Device does not support V4L2 capture/streaming");

            v4l2_format fmt{};
            fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            fmt.fmt.pix.width = width;
            fmt.fmt.pix.height = height;
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
            fmt.fmt.pix.field = V4L2_FIELD_ANY;

            check(ioctl_retry(fd_, VIDIOC_S_FMT, &fmt) == 0,
                  "VIDIOC_S_FMT");

            if (fmt.fmt.pix.width != static_cast<unsigned>(width) ||
                fmt.fmt.pix.height != static_cast<unsigned>(height) ||
                fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
                throw std::runtime_error(
                    "Camera rejected requested MJPEG format");

            v4l2_streamparm parm{};
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            parm.parm.capture.timeperframe.numerator = 1;
            parm.parm.capture.timeperframe.denominator = fps;

            check(ioctl_retry(fd_, VIDIOC_S_PARM, &parm) == 0,
                  "VIDIOC_S_PARM");

            v4l2_streamparm actual{};
            actual.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            check(ioctl_retry(fd_, VIDIOC_G_PARM, &actual) == 0,
                  "VIDIOC_G_PARM");

            const auto n =
                actual.parm.capture.timeperframe.numerator;
            const auto d =
                actual.parm.capture.timeperframe.denominator;

            actual_fps_ = n ? static_cast<double>(d) / n : 0.0;

            if (actual_fps_ <= 0.0)
                throw std::runtime_error("Invalid camera FPS");

            v4l2_requestbuffers req{};
            req.count = 8;
            req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            req.memory = V4L2_MEMORY_MMAP;

            check(ioctl_retry(fd_, VIDIOC_REQBUFS, &req) == 0,
                  "VIDIOC_REQBUFS");

            if (req.count < 2)
                throw std::runtime_error("Insufficient V4L2 buffers");

            buffers_.resize(req.count);

            for (unsigned i = 0; i < req.count; ++i) {
                v4l2_buffer b{};
                b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                b.memory = V4L2_MEMORY_MMAP;
                b.index = i;

                check(ioctl_retry(fd_, VIDIOC_QUERYBUF, &b) == 0,
                      "VIDIOC_QUERYBUF");

                void* ptr = mmap(
                    nullptr, b.length,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED, fd_, b.m.offset);

                check(ptr != MAP_FAILED, "mmap");

                buffers_[i] = {ptr, b.length};

                check(ioctl_retry(fd_, VIDIOC_QBUF, &b) == 0,
                      "VIDIOC_QBUF");
            }

            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            check(ioctl_retry(fd_, VIDIOC_STREAMON, &type) == 0,
                  "VIDIOC_STREAMON");

            streaming_ = true;
        } catch (...) {
            close();
            throw;
        }
    }


    bool read(Frame& out, int timeout_ms = 1000) {
        if (!streaming_)
            throw std::runtime_error("Camera not streaming");

        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;

        int rc;
        do {
            rc = poll(&pfd, 1, timeout_ms);
        } while (rc < 0 && errno == EINTR);

        if (rc == 0)
            return false;

        check(rc > 0, "camera poll");

        v4l2_buffer newest{};
        bool have_newest = false;
        uint32_t dropped = 0;

        for (;;) {
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;

            if (ioctl_retry(fd_, VIDIOC_DQBUF, &b) < 0) {
                if (errno == EAGAIN)
                    break;
                check(false, "VIDIOC_DQBUF");
            }

            if (b.index >= buffers_.size())
                throw std::runtime_error("Invalid V4L2 buffer index");

            if (have_newest) {
                check(ioctl_retry(fd_, VIDIOC_QBUF, &newest) == 0,
                      "VIDIOC_QBUF old");
                ++dropped;
            }

            newest = b;
            have_newest = true;
        }

        if (!have_newest)
            return false;

        const auto capture_timestamp = newest.timestamp;
        const uint32_t capture_sequence = newest.sequence;
        const uint32_t capture_flags = newest.flags;

        cv::Mat gray;

        try {
            if (newest.bytesused > 0 &&
                newest.bytesused <= buffers_[newest.index].length) {
                cv::Mat encoded(
                    1, static_cast<int>(newest.bytesused),
                    CV_8UC1, buffers_[newest.index].ptr);

                gray = cv::imdecode(
                    encoded, cv::IMREAD_GRAYSCALE);
            }
        } catch (...) {
            ioctl_retry(fd_, VIDIOC_QBUF, &newest);
            throw;
        }

        check(ioctl_retry(fd_, VIDIOC_QBUF, &newest) == 0,
              "VIDIOC_QBUF newest");

        if (gray.empty())
            return false;

        if (!(capture_flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC))
            throw std::runtime_error(
                "Camera timestamp is not CLOCK_MONOTONIC");

        out.gray = std::move(gray);
        out.timestamp_ns =
            static_cast<int64_t>(capture_timestamp.tv_sec) * 1000000000LL +
            static_cast<int64_t>(capture_timestamp.tv_usec) * 1000LL;
        out.sequence = capture_sequence;

        struct timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);

        const int64_t now_ns =
            static_cast<int64_t>(ts.tv_sec) * 1000000000LL +
            ts.tv_nsec;

        std::cout
            << "BF_CAMERA_LATEST"
            << " seq=" << capture_sequence
            << " dropped=" << dropped
            << " capture_age_ms="
            << (now_ns - out.timestamp_ns) * 1e-6
            << std::endl;

        return true;
    }

    double actualFps() const {
        return actual_fps_;
    }

    void close() noexcept {
        if (fd_ < 0)
            return;

        if (streaming_) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl_retry(fd_, VIDIOC_STREAMOFF, &type);
            streaming_ = false;
        }

        for (auto& b : buffers_) {
            if (b.ptr && b.ptr != MAP_FAILED)
                munmap(b.ptr, b.length);
        }

        buffers_.clear();
        ::close(fd_);
        fd_ = -1;
    }

private:
    struct Buffer {
        void* ptr = nullptr;
        size_t length = 0;
    };

    int fd_ = -1;
    bool streaming_ = false;
    double actual_fps_ = 0.0;
    std::vector<Buffer> buffers_;
};

} // namespace bf_camera
