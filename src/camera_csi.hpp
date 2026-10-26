#pragma once

#include <libcamera/libcamera.h>
#include <libcamera/control_ids.h>
#include <opencv2/core.hpp>

#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace jtzero_csi {

inline int64_t monoNs() {
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return int64_t(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

struct Frame {
    cv::Mat gray;
    int64_t sensor_ns = 0;
    int64_t callback_ns = 0;
    uint64_t sequence = 0;
};

struct Stats {
    uint64_t captured = 0;
    uint64_t consumed = 0;
    uint64_t skipped = 0;
    uint64_t errors = 0;
    uint64_t missing_ts = 0;
    uint64_t cancelled = 0;
    uint64_t queue_errors = 0;
};

class CameraCSI {
    struct Mapping {
        void *base = nullptr;
        size_t length = 0;
        size_t delta = 0;
        size_t available = 0;
    };

    libcamera::CameraManager manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::map<libcamera::FrameBuffer*, Mapping> mappings_;

    std::mutex mu_;
    Frame latest_;
    Stats stats_;
    uint64_t last_consumed_sequence_ = 0;
    unsigned stride_ = 0;
    bool manager_started_ = false;
    bool acquired_ = false;
    bool started_ = false;
    bool connected_ = false;
    std::atomic<bool> running_{false};

    void completed(libcamera::Request *request) {
        if (request->status() ==
            libcamera::Request::RequestCancelled) {
            std::lock_guard<std::mutex> lock(mu_);
            ++stats_.cancelled;
            return;
        }

        const int64_t callback_ns = monoNs();
        const auto sensor =
            request->metadata().get(
                libcamera::controls::SensorTimestamp);

        bool ok = false;
        cv::Mat gray;

        for (const auto &entry : request->buffers()) {
            auto it = mappings_.find(entry.second);
            if (it == mappings_.end())
                continue;

            const Mapping &m = it->second;
            const size_t required =
                size_t(stride_) * 479 + 640;

            if (m.available < required)
                continue;

            const uint8_t *src =
                static_cast<const uint8_t*>(m.base) + m.delta;

            gray.create(480, 640, CV_8UC1);

            for (int y = 0; y < 480; ++y) {
                std::memcpy(
                    gray.ptr<uint8_t>(y),
                    src + size_t(y) * stride_,
                    640
                );
            }

            ok = true;
            break;
        }

        {
            std::lock_guard<std::mutex> lock(mu_);
            if (!ok) {
                ++stats_.errors;
            } else {
                ++stats_.captured;
                if (!sensor)
                    ++stats_.missing_ts;

                latest_.gray = std::move(gray);
                latest_.sensor_ns =
                    sensor ? *sensor : callback_ns;
                latest_.callback_ns = callback_ns;
                latest_.sequence = stats_.captured;
            }
        }

        if (running_.load()) {
            request->reuse(libcamera::Request::ReuseBuffers);
            if (camera_->queueRequest(request) < 0) {
                std::lock_guard<std::mutex> lock(mu_);
                ++stats_.queue_errors;
            }
        }
    }

public:
    CameraCSI() = default;
    CameraCSI(const CameraCSI&) = delete;
    CameraCSI& operator=(const CameraCSI&) = delete;

    ~CameraCSI() { stop(); }

    void start() {
        if (started_)
            throw std::runtime_error("CSI already started");

        try {
            if (manager_.start() < 0)
                throw std::runtime_error("CameraManager start failed");
            manager_started_ = true;

            for (auto &c : manager_.cameras()) {
                if (c->id().find("ov5647") != std::string::npos) {
                    camera_ = c;
                    break;
                }
            }
            if (!camera_)
                throw std::runtime_error("OV5647 not found");

            if (camera_->acquire() < 0)
                throw std::runtime_error("Camera acquire failed");
            acquired_ = true;

            config_ = camera_->generateConfiguration(
                {libcamera::StreamRole::Viewfinder});

            if (!config_ || config_->empty())
                throw std::runtime_error("CSI config unavailable");

            auto &sc = config_->at(0);
            sc.size = libcamera::Size(640, 480);
            sc.pixelFormat = libcamera::formats::YUV420;
            sc.bufferCount = 6;

            if (config_->validate() ==
                libcamera::CameraConfiguration::Invalid)
                throw std::runtime_error("Invalid CSI config");

            if (sc.size.width != 640 ||
                sc.size.height != 480 ||
                sc.pixelFormat != libcamera::formats::YUV420 ||
                sc.stride < 640)
                throw std::runtime_error("Unexpected CSI format");

            stride_ = sc.stride;

            if (camera_->configure(config_.get()) < 0)
                throw std::runtime_error("CSI configure failed");

            allocator_ =
                std::make_unique<libcamera::FrameBufferAllocator>(
                    camera_);

            auto *stream = sc.stream();
            if (allocator_->allocate(stream) < 0)
                throw std::runtime_error("CSI allocation failed");

            const long page = sysconf(_SC_PAGESIZE);
            if (page <= 0)
                throw std::runtime_error("Invalid page size");

            for (const auto &buffer :
                 allocator_->buffers(stream)) {
                const auto &planes = buffer->planes();
                if (planes.empty())
                    throw std::runtime_error("No CSI planes");

                const auto &p = planes[0];
                const size_t aligned =
                    (size_t(p.offset) / size_t(page)) *
                    size_t(page);
                const size_t delta =
                    size_t(p.offset) - aligned;
                const size_t length =
                    size_t(p.length) + delta;

                void *base = mmap(
                    nullptr, length, PROT_READ, MAP_SHARED,
                    p.fd.get(), static_cast<off_t>(aligned));

                if (base == MAP_FAILED)
                    throw std::runtime_error("CSI mmap failed");

                mappings_[buffer.get()] = {
                    base, length, delta, size_t(p.length)
                };

                auto req = camera_->createRequest();
                if (!req ||
                    req->addBuffer(stream, buffer.get()) < 0)
                    throw std::runtime_error("CSI request failed");

                requests_.push_back(std::move(req));
            }

            camera_->requestCompleted.connect(
                this, &CameraCSI::completed);
            connected_ = true;

            libcamera::ControlList controls(camera_->controls());
            // OV5647: automatic exposure and analogue gain.
            controls.set(libcamera::controls::AeEnable, true);
            controls.set(
                libcamera::controls::FrameDurationLimits,
                std::array<int64_t, 2>{16666, 16666});

            if (camera_->start(&controls) < 0)
                throw std::runtime_error("CSI start failed");

            started_ = true;
            running_ = true;

            for (auto &req : requests_) {
                if (camera_->queueRequest(req.get()) < 0)
                    throw std::runtime_error(
                        "Initial CSI queue failed");
            }
        } catch (...) {
            stop();
            throw;
        }
    }

    bool newest(Frame &out) {
        std::lock_guard<std::mutex> lock(mu_);

        if (latest_.sequence == 0 ||
            latest_.sequence == last_consumed_sequence_)
            return false;

        if (last_consumed_sequence_ > 0 &&
            latest_.sequence > last_consumed_sequence_ + 1)
            stats_.skipped +=
                latest_.sequence - last_consumed_sequence_ - 1;

        out = latest_;
        last_consumed_sequence_ = latest_.sequence;
        ++stats_.consumed;
        return true;
    }

    Stats stats() {
        std::lock_guard<std::mutex> lock(mu_);
        return stats_;
    }

    unsigned stride() const { return stride_; }

    void stop() {
        running_ = false;

        if (started_ && camera_) {
            camera_->stop();
            started_ = false;
        }

        if (connected_ && camera_) {
            camera_->requestCompleted.disconnect(this);
            connected_ = false;
        }

        requests_.clear();

        for (auto &entry : mappings_) {
            if (entry.second.base)
                munmap(entry.second.base, entry.second.length);
        }
        mappings_.clear();

        allocator_.reset();
        config_.reset();

        if (acquired_ && camera_) {
            camera_->release();
            acquired_ = false;
        }

        camera_.reset();

        if (manager_started_) {
            manager_.stop();
            manager_started_ = false;
        }
    }
};

} // namespace jtzero_csi
