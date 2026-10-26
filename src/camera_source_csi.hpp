#pragma once

#include "camera_source.hpp"
#include "camera_csi.hpp"

#include <cstdint>
#include <string>

namespace jtzero {

class CameraSourceCSI final : public CameraSource {
public:
    CameraSourceCSI() = default;

    ~CameraSourceCSI() override {
        stop();
    }

    CameraSourceCSI(const CameraSourceCSI&) = delete;
    CameraSourceCSI& operator=(const CameraSourceCSI&) = delete;

    void start() override {
        camera_.start();
    }

    void stop() override {
        camera_.stop();
    }

    bool newest(CameraFrame& frame) override {
        jtzero_csi::Frame raw;

        if (!camera_.newest(raw))
            return false;

        CameraFrame next;
        next.gray = raw.gray;
        next.sensor_ns = raw.sensor_ns;
        next.callback_ns = raw.callback_ns;
        next.sequence = ++sequence_;
        next.source = CameraSourceType::CSI;

        if (!next.valid())
            return false;

        frame = std::move(next);
        return true;
    }

    CameraSourceType type() const override {
        return CameraSourceType::CSI;
    }

    std::string name() const override {
        return "OV5647 CSI";
    }

private:
    jtzero_csi::CameraCSI camera_;
    uint64_t sequence_ = 0;
};

} // namespace jtzero
