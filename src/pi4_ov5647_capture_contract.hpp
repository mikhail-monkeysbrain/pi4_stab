#pragma once
// OV5647 libcamera capture adapter contract for Pi4.
// Diagnostic-only API: this header never publishes MAVLink or estimates height.
// The caller owns the camera lifecycle and receives copied grayscale frames.
#include <opencv2/core.hpp>
#include <cstdint>

namespace pi4_capture {
struct Frame {
    cv::Mat gray;                 // owned 8-bit 640x480 image
    int64_t sensor_timestamp_ns=0; // libcamera SensorTimestamp from SAME Request
    uint64_t sequence=0;
};
struct Stats {
    uint64_t received=0;
    uint64_t missing_timestamp=0;
    uint64_t nonmonotonic_timestamp=0;
    uint64_t cancelled=0;
};
// Contract for future adapter implementation:
// start() -> next(Frame&,timeout_ms) -> stop().
// No synthetic altitude may be presented as a measured range.
// No FC TX is permitted in the camera adapter.
}  // namespace pi4_capture
