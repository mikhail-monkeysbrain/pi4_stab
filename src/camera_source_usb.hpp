#pragma once

#include "camera_source.hpp"
#include "runtime.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cerrno>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace jtzero {

class CameraSourceUSB final : public CameraSource {
public:
    explicit CameraSourceUSB(std::string device)
        : device_(std::move(device)) {}

    ~CameraSourceUSB() override {
        stop();
    }

    CameraSourceUSB(const CameraSourceUSB&) = delete;
    CameraSourceUSB& operator=(const CameraSourceUSB&) = delete;

    void start() override {
        camera_.openDev(device_);
    }

    void stop() override {
        camera_.close();
    }

    bool newest(CameraFrame& frame) override {
        if (camera_.fd < 0)
            return false;

        std::vector<uint8_t> latest_jpeg;
        int64_t latest_ts = 0;
        int64_t latest_dq_ns = 0;

        for (;;) {
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;

            if (xioctl(camera_.fd, VIDIOC_DQBUF, &b) < 0) {
                if (errno == EAGAIN)
                    break;
                fail("USB VIDIOC_DQBUF");
            }

            const int64_t dq_ns = monoNs();

            const int64_t v4l2_ns =
                static_cast<int64_t>(b.timestamp.tv_sec) * 1000000000LL +
                static_cast<int64_t>(b.timestamp.tv_usec) * 1000LL;

            const bool monotonic =
                (b.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) != 0;

            const int64_t frame_ns =
                (monotonic && v4l2_ns > 0) ? v4l2_ns : dq_ns;

            if (b.index >= camera_.bufs.size() ||
                b.bytesused > camera_.bufs[b.index].n) {
                xioctl(camera_.fd, VIDIOC_QBUF, &b);
                throw std::runtime_error("Invalid USB camera buffer");
            }

            const auto* data = static_cast<const uint8_t*>(
                camera_.bufs[b.index].p
            );

            latest_jpeg.assign(data, data + b.bytesused);
            latest_ts = frame_ns;
            latest_dq_ns = dq_ns;

            if (xioctl(camera_.fd, VIDIOC_QBUF, &b) < 0)
                fail("USB VIDIOC_QBUF");
        }

        if (latest_jpeg.empty())
            return false;

        cv::Mat encoded(
            1,
            static_cast<int>(latest_jpeg.size()),
            CV_8UC1,
            latest_jpeg.data()
        );

        cv::Mat gray = cv::imdecode(
            encoded,
            cv::IMREAD_GRAYSCALE
        );

        if (gray.empty())
            return false;

        CameraFrame next;
        next.gray = std::move(gray);
        next.sensor_ns = latest_ts;
        next.callback_ns = latest_dq_ns;
        next.sequence = ++sequence_;
        next.source = CameraSourceType::USB;

        if (!next.valid())
            return false;

        frame = std::move(next);
        return true;
    }

    CameraSourceType type() const override {
        return CameraSourceType::USB;
    }

    std::string name() const override {
        return "OV9281 USB";
    }

private:
    std::string device_;
    Camera camera_;
    uint64_t sequence_ = 0;
};

} // namespace jtzero
