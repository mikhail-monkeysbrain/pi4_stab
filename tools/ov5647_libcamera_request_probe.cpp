// Standalone, read-only OV5647 libcamera Request diagnostic.
// Compile: g++ -std=c++17 -O2 -pthread tools/ov5647_libcamera_request_probe.cpp -o /tmp/ov5647_request_probe $(pkg-config --cflags --libs libcamera opencv4)
// Does not change WORKED5, FC or production capture.
#include <libcamera/libcamera.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <sys/mman.h>
#include <unistd.h>
#include <cerrno>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

using namespace libcamera;

struct Mapping {
    void *ptr = MAP_FAILED;
    size_t size = 0;
    ~Mapping() { if (ptr != MAP_FAILED) munmap(ptr, size); }
    Mapping() = default;
    Mapping(const Mapping &) = delete;
    Mapping &operator=(const Mapping &) = delete;
};

class Completed {
public:
    void done(Request *r) {
        std::lock_guard<std::mutex> lk(mu_);
        queue_.push_back(r);
        cv_.notify_one();
    }
    Request *wait(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lk(mu_);
        if (!cv_.wait_for(lk, timeout, [&]{return !queue_.empty();})) return nullptr;
        auto *r = queue_.front();
        queue_.pop_front();
        return r;
    }
private:
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Request *> queue_;
};

int main(int argc, char **argv) {
    const int seconds = argc > 1 ? std::stoi(argv[1]) : 8;
    if (seconds < 1 || seconds > 120) {
        std::cerr << "seconds must be 1..120\n";
        return 2;
    }
    try {
        CameraManager manager;
        if (manager.start()) throw std::runtime_error("CameraManager start failed");
        if (manager.cameras().empty()) throw std::runtime_error("no libcamera cameras");
        auto camera = manager.cameras().front();
        if (camera->acquire()) throw std::runtime_error("camera acquire failed");
        auto config = camera->generateConfiguration({StreamRole::Viewfinder});
        if (!config || config->empty()) throw std::runtime_error("no viewfinder config");
        auto &cfg = config->at(0);
        cfg.size = Size(640, 480);
        cfg.pixelFormat = formats::RGB888;
        cfg.bufferCount = 4;
        auto validation = config->validate();
        if (validation == CameraConfiguration::Invalid)
            throw std::runtime_error("camera config invalid");
        if (cfg.pixelFormat != formats::RGB888 || cfg.size != Size(640,480))
            throw std::runtime_error("RGB888 640x480 not supported by validated configuration");
        if (camera->configure(config.get()))
            throw std::runtime_error("camera configure failed");
        Stream *stream = cfg.stream();
        FrameBufferAllocator allocator(camera);
        if (allocator.allocate(stream) < 0)
            throw std::runtime_error("buffer allocation failed");

        // Map each plane independently. Reject multi-plane RGB layouts in this probe.
        std::map<FrameBuffer *, std::unique_ptr<Mapping>> mappings;
        for (const auto &buffer : allocator.buffers(stream)) {
            const auto &planes = buffer->planes();
            if (planes.size() != 1)
                throw std::runtime_error("probe expects one RGB plane");
            auto mapping = std::make_unique<Mapping>();
            mapping->size = planes[0].length;
            mapping->ptr = mmap(nullptr, mapping->size, PROT_READ, MAP_SHARED,
                                planes[0].fd.get(), 0);
            if (mapping->ptr == MAP_FAILED)
                throw std::runtime_error(std::string("mmap failed: ")+strerror(errno));
            mappings.emplace(buffer.get(), std::move(mapping));
        }
        Completed completed;
        camera->requestCompleted.connect(&completed, &Completed::done);
        std::vector<std::unique_ptr<Request>> requests;
        for (const auto &buffer : allocator.buffers(stream)) {
            auto request = camera->createRequest();
            if (!request || request->addBuffer(stream, buffer.get()))
                throw std::runtime_error("request creation/addBuffer failed");
            requests.push_back(std::move(request));
        }
        ControlList start_controls(camera->controls());
        start_controls.set(controls::FrameDurationLimits, std::array<int64_t, 2>{16666, 16666});
        if (camera->start(&start_controls)) throw std::runtime_error("camera start failed");
        for (auto &r : requests)
            if (camera->queueRequest(r.get())) throw std::runtime_error("queueRequest failed");

        uint64_t received=0, valid=0, missing_ts=0, nonmono=0, cancelled=0;
        int64_t last_ts=0, min_gap=INT64_MAX, max_gap=0;
        const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(seconds);
        while (std::chrono::steady_clock::now() < until) {
            Request *r=completed.wait(std::chrono::milliseconds(1000));
            if (!r) continue;
            if (r->status() != Request::RequestComplete) {
                ++cancelled;
                continue;
            }
            ++received;
            const auto sensor_ts=r->metadata().get(controls::SensorTimestamp);
            if (!sensor_ts) ++missing_ts;
            else {
                const int64_t ts=*sensor_ts;
                if (last_ts && ts<=last_ts) ++nonmono;
                if (last_ts && ts>last_ts) {
                    min_gap=std::min(min_gap,ts-last_ts);
                    max_gap=std::max(max_gap,ts-last_ts);
                }
                last_ts=ts;
            }
            auto it=r->buffers().find(stream);
            if (it == r->buffers().end()) throw std::runtime_error("request missing buffer");
            auto *buffer=it->second;
            const auto &mapping=*mappings.at(buffer);
            const size_t required=static_cast<size_t>(cfg.stride)*cfg.size.height;
            if (required > mapping.size)
                throw std::runtime_error("mapped buffer smaller than stride*height");
            cv::Mat rgb(480,640,CV_8UC3,mapping.ptr,cfg.stride);
            cv::Mat gray;
            cv::cvtColor(rgb,gray,cv::COLOR_RGB2GRAY);
            if (!gray.empty() && sensor_ts) ++valid;
            if (received<=3)
                std::cout<<"REQUEST_SAMPLE seq="<<r->sequence()
                         <<" sensor_ts_ns="<<(sensor_ts?std::to_string(*sensor_ts):"MISSING")
                         <<" stride="<<cfg.stride<<" gray="<<gray.cols<<"x"<<gray.rows<<"\n";
            r->reuse(Request::ReuseBuffers);
            if (camera->queueRequest(r)) throw std::runtime_error("requeue failed");
        }
        camera->stop();
        std::cout<<"REQUEST_FINAL received="<<received<<" valid="<<valid
                 <<" missing_ts="<<missing_ts<<" nonmonotonic_ts="<<nonmono
                 <<" cancelled="<<cancelled
                 <<" min_gap_ms="<<(min_gap==INT64_MAX?0:min_gap/1e6)
                 <<" max_gap_ms="<<max_gap/1e6
                 <<" association=SAME_LIBCAMERA_REQUEST"
                 <<" no_fc_tx=1 no_worked5_change=1\n";
        camera->requestCompleted.disconnect(&completed, &Completed::done);
        camera->release();
        // CameraManager destructor runs after camera, requests and allocator destructors.
        // Explicit stop here would remove media devices while objects are still alive.
        return received>0 && missing_ts==0 && nonmono==0 && valid==received ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr<<"REQUEST_PROBE_FAIL "<<e.what()<<"\n";
        return 2;
    }
}
