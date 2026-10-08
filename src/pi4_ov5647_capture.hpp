#pragma once
// Pi4 OV5647 libcamera Request capture, independent of WORKED5 and MAVLink.
// Caller must stop() before destruction. Single consumer; callback is invoked
// synchronously from next(), not from the libcamera completion callback.
#include "pi4_ov5647_capture_contract.hpp"
#include <libcamera/libcamera.h>
#include <opencv2/imgproc.hpp>
#include <sys/mman.h>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <array>

namespace pi4_capture {
class Ov5647Capture {
    struct Mapping {
        void *ptr=MAP_FAILED;
        size_t size=0;
        ~Mapping() { if(ptr!=MAP_FAILED) munmap(ptr,size); }
        Mapping()=default;
        Mapping(const Mapping&)=delete;
        Mapping& operator=(const Mapping&)=delete;
    };
    class Completion {
        std::mutex mu_;
        std::condition_variable cv_;
        std::deque<libcamera::Request*> queue_;
    public:
        void done(libcamera::Request *request) {
            std::lock_guard<std::mutex> lock(mu_);
            queue_.push_back(request);
            cv_.notify_one();
        }
        libcamera::Request *wait(int timeout_ms) {
            std::unique_lock<std::mutex> lock(mu_);
            if (!cv_.wait_for(lock,std::chrono::milliseconds(timeout_ms),
                              [&]{return !queue_.empty();})) return nullptr;
            auto *request=queue_.front();
            queue_.pop_front();
            return request;
        }
    };
    libcamera::CameraManager manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream *stream_=nullptr;
    std::map<libcamera::FrameBuffer*,std::unique_ptr<Mapping>> mappings_;
    Completion completion_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    bool manager_started_=false, acquired_=false, connected_=false, running_=false;
    int64_t last_timestamp_=0;
    Stats stats_;
public:
    Ov5647Capture()=default;
    Ov5647Capture(const Ov5647Capture&)=delete;
    Ov5647Capture& operator=(const Ov5647Capture&)=delete;
    ~Ov5647Capture() { stop(); }

    void start() {
        if (running_ || manager_started_) throw std::runtime_error("capture already started");
        if (manager_.start()) throw std::runtime_error("CameraManager start failed");
        manager_started_=true;
        try {
            if (manager_.cameras().empty()) throw std::runtime_error("no libcamera camera");
            camera_=manager_.cameras().front();
            if (camera_->acquire()) throw std::runtime_error("camera acquire failed");
            acquired_=true;
            config_=camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
            if (!config_ || config_->empty()) throw std::runtime_error("no viewfinder configuration");
            auto &cfg=config_->at(0);
            cfg.size=libcamera::Size(640,480);
            cfg.pixelFormat=libcamera::formats::RGB888;
            cfg.bufferCount=4;
            if (config_->validate()==libcamera::CameraConfiguration::Invalid ||
                cfg.size!=libcamera::Size(640,480) ||
                cfg.pixelFormat!=libcamera::formats::RGB888)
                throw std::runtime_error("RGB888 640x480 unavailable");
            if (camera_->configure(config_.get())) throw std::runtime_error("camera configure failed");
            stream_=cfg.stream();
            allocator_=std::make_unique<libcamera::FrameBufferAllocator>(camera_);
            if (allocator_->allocate(stream_)<0) throw std::runtime_error("buffer allocation failed");
            for (const auto &buffer:allocator_->buffers(stream_)) {
                const auto &planes=buffer->planes();
                if (planes.size()!=1) throw std::runtime_error("expected single RGB plane");
                auto mapping=std::make_unique<Mapping>();
                mapping->size=planes[0].length;
                mapping->ptr=mmap(nullptr,mapping->size,PROT_READ,MAP_SHARED,planes[0].fd.get(),0);
                if (mapping->ptr==MAP_FAILED)
                    throw std::runtime_error(std::string("mmap: ")+strerror(errno));
                mappings_.emplace(buffer.get(),std::move(mapping));
                auto request=camera_->createRequest();
                if (!request || request->addBuffer(stream_,buffer.get()))
                    throw std::runtime_error("request creation failed");
                requests_.push_back(std::move(request));
            }
            camera_->requestCompleted.connect(&completion_,&Completion::done);
            connected_=true;
            libcamera::ControlList controls(camera_->controls());
            controls.set(libcamera::controls::FrameDurationLimits,
                         std::array<int64_t,2>{16666,16666});
            if (camera_->start(&controls)) throw std::runtime_error("camera start failed");
            running_=true;
            for (auto &request:requests_)
                if (camera_->queueRequest(request.get()))
                    throw std::runtime_error("queueRequest failed");
        } catch (...) { stop(); throw; }
    }

    // Returns false on timeout or rejected frame. Stats distinguish causes.
    bool next(Frame &frame,int timeout_ms=1000) {
        if (!running_) throw std::runtime_error("capture not started");
        auto *request=completion_.wait(timeout_ms);
        if (!request) return false;
        bool valid=false;
        if (request->status()!=libcamera::Request::RequestComplete) {
            ++stats_.cancelled;
        } else {
            ++stats_.received;
            const auto timestamp=request->metadata().get(libcamera::controls::SensorTimestamp);
            if (!timestamp) ++stats_.missing_timestamp;
            else if (last_timestamp_ && *timestamp<=last_timestamp_)
                ++stats_.nonmonotonic_timestamp;
            else {
                last_timestamp_=*timestamp;
                auto it=request->buffers().find(stream_);
                if (it==request->buffers().end()) throw std::runtime_error("request missing buffer");
                const auto &mapping=*mappings_.at(it->second);
                const auto &cfg=config_->at(0);
                if (static_cast<size_t>(cfg.stride)*cfg.size.height>mapping.size)
                    throw std::runtime_error("buffer smaller than stride*height");
                cv::Mat rgb(480,640,CV_8UC3,mapping.ptr,cfg.stride);
                cv::cvtColor(rgb,frame.gray,cv::COLOR_RGB2GRAY);
                frame.sensor_timestamp_ns=*timestamp;
                frame.sequence=request->sequence();
                valid=!frame.gray.empty();
            }
        }
        // The frame has its own storage before the Request is requeued.
        request->reuse(libcamera::Request::ReuseBuffers);
        if (camera_->queueRequest(request)) throw std::runtime_error("requeue failed");
        return valid;
    }

    const Stats &stats() const { return stats_; }

    void stop() noexcept {
        if (running_) { camera_->stop(); running_=false; }
        if (connected_) {
            camera_->requestCompleted.disconnect(&completion_,&Completion::done);
            connected_=false;
        }
        requests_.clear();
        mappings_.clear();
        allocator_.reset();
        config_.reset();
        if (acquired_) { camera_->release(); acquired_=false; }
        camera_.reset();
        if (manager_started_) { manager_.stop(); manager_started_=false; }
        stream_=nullptr;
    }
};
} // namespace pi4_capture
