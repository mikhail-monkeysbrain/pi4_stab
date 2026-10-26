#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace jtzero {

enum class CameraSourceType {
    USB,
    CSI
};

struct CameraFrame {
    cv::Mat gray;

    // Timestamp in CLOCK_MONOTONIC nanoseconds.
    int64_t sensor_ns = 0;

    // libcamera callback timestamp, CLOCK_MONOTONIC ns.
    int64_t callback_ns = 0;

    // Sequential identifier of the source frame.
    uint64_t sequence = 0;

    CameraSourceType source = CameraSourceType::USB;

    bool valid() const {
        return !gray.empty() &&
               gray.type() == CV_8UC1 &&
               sensor_ns > 0;
    }
};

class CameraSource {
public:
    virtual ~CameraSource() = default;

    virtual void start() = 0;
    virtual void stop() = 0;

    // Returns true only when a new frame is available.
    virtual bool newest(CameraFrame& frame) = 0;

    virtual CameraSourceType type() const = 0;

    virtual std::string name() const = 0;
};

} // namespace jtzero
