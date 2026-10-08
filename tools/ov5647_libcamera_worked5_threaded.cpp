// Standalone, read-only OV5647 libcamera Request diagnostic.
// Compile: g++ -std=c++17 -O2 -pthread tools/ov5647_libcamera_worked5_diagnostic.cpp -Isrc -o /tmp/ov5647_libcamera_worked5 $(pkg-config --cflags --libs libcamera opencv4)
// Does not change WORKED5, FC or production capture.
#include <libcamera/libcamera.h>
#include "worked5_estimator.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <thread>
#include <opencv2/core.hpp>
#include <sys/mman.h>
#include <unistd.h>
#include <cerrno>
#include <array>
#include <chrono>
#include <fstream>
#include <ctime>
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

        cv::setNumThreads(2);
        cv::Mat prev;
        int64_t prev_sensor_ts=0;
        uint64_t pairs=0, ransac_ok=0, worked5_ok=0, invalid_dt=0;
        double lk_ms_sum=0, worked5_ms_sum=0, max_pair_gap_ms=0;
        // Provisional geometry and synthetic height: no metric navigation claims.
        const cv::Mat K=(cv::Mat_<double>(3,3)<<600.,0.,320.,0.,600.,240.,0.,0.,1.);
        const cv::Mat D=cv::Mat::zeros(1,5,CV_64F);
        struct Sample { cv::Mat gray; int64_t ts; };
        std::mutex sample_mu;
        std::condition_variable sample_cv;
        std::deque<Sample> samples;
        bool producer_done=false;
        uint64_t captured=0, dropped=0;
        uint64_t received=0, valid=0, missing_ts=0, nonmono=0, cancelled=0;
        int64_t last_ts=0, min_gap=INT64_MAX, max_gap=0;
        std::thread worker([&] {
            for (;;) {
                Sample sample;
                {
                    std::unique_lock<std::mutex> lk(sample_mu);
                    sample_cv.wait(lk,[&]{return producer_done || !samples.empty();});
                    if (samples.empty()) {
                        if (producer_done) break;
                        continue;
                    }
                    sample=std::move(samples.front());
                    samples.pop_front();
                }
                const int64_t ts=sample.ts;
                if (!prev.empty() && prev_sensor_ts>0) {
                    const double dt=(ts-prev_sensor_ts)*1e-9;
                    ++pairs;
                    max_pair_gap_ms=std::max(max_pair_gap_ms,dt*1000.);
                    if (!(dt>0. && dt<0.2)) {
                        ++invalid_dt;
                    } else {
                        std::vector<cv::Point2f> pts,next;
                        cv::goodFeaturesToTrack(prev,pts,300,0.01,8);
                        if (pts.size()>=20) {
                            std::vector<uchar> ok;
                            std::vector<float> err;
                            const auto t0=std::chrono::steady_clock::now();
                            cv::calcOpticalFlowPyrLK(prev,sample.gray,pts,next,ok,err);
                            lk_ms_sum+=std::chrono::duration<double,std::milli>(
                                std::chrono::steady_clock::now()-t0).count();
                            std::vector<cv::Point2f> a,b;
                            for (size_t i=0;i<ok.size();++i) if (ok[i]) {
                                a.push_back(pts[i]); b.push_back(next[i]);
                            }
                            if (a.size()>=20) {
                                cv::Mat mask;
                                cv::estimateAffinePartial2D(a,b,mask,cv::RANSAC,3.0);
                                if (!mask.empty() && cv::countNonZero(mask)>=20) {
                                    ++ransac_ok;
                                    std::vector<cv::Point2f> in_a,in_b;
                                    for (int i=0;i<mask.rows;++i) if (mask.at<uchar>(i,0)) {
                                        in_a.push_back(a[(size_t)i]);
                                        in_b.push_back(b[(size_t)i]);
                                    }
                                    const auto t1=std::chrono::steady_clock::now();
                                    const auto result=worked5::estimate(in_a,in_b,K,1.0,D,1.0,dt);
                                    worked5_ms_sum+=std::chrono::duration<double,std::milli>(
                                        std::chrono::steady_clock::now()-t1).count();
                                    if (result.valid && std::isfinite(result.du_norm) &&
                                        std::isfinite(result.dv_norm)) ++worked5_ok;
                                }
                            }
                        }
                    }
                }
                prev=std::move(sample.gray);
                prev_sensor_ts=ts;
            }
        });
        std::ofstream timing_csv;
        if (argc > 2) {
            timing_csv.open(argv[2]);
            if (!timing_csv) throw std::runtime_error("cannot open timing CSV");
            timing_csv << "sequence,sensor_ts_ns,recv_mono_ns,recv_steady_ns,status\\n";
        }
        const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(seconds);
        while (std::chrono::steady_clock::now() < until) {
            Request *r=completed.wait(std::chrono::milliseconds(1000));
            struct timespec rx_clock {};
            if (r && clock_gettime(CLOCK_MONOTONIC, &rx_clock) != 0)
                throw std::runtime_error("clock_gettime CLOCK_MONOTONIC failed");
            const int64_t rx_mono_ns = static_cast<int64_t>(rx_clock.tv_sec)*1000000000LL+rx_clock.tv_nsec;
            const int64_t rx_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (!r) continue;
            if (r->status() != Request::RequestComplete) {
                ++cancelled;
                continue;
            }
            ++received;
            const auto sensor_ts=r->metadata().get(controls::SensorTimestamp);
            if (timing_csv)
                timing_csv << r->sequence() << "," << (sensor_ts?std::to_string(*sensor_ts):"")
                           << "," << rx_mono_ns << "," << rx_steady_ns << ",complete\\n";
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
            if (sensor_ts && !gray.empty()) {
                std::lock_guard<std::mutex> lk(sample_mu);
                ++captured;
                if (!samples.empty()) { dropped+=samples.size(); samples.clear(); }
                samples.push_back({std::move(gray), *sensor_ts});
                sample_cv.notify_one();
            }
            r->reuse(Request::ReuseBuffers);
            if (camera->queueRequest(r)) throw std::runtime_error("requeue failed");
        }
        if (timing_csv) timing_csv.flush();
        camera->stop();
        {
            std::lock_guard<std::mutex> lk(sample_mu);
            producer_done=true;
        }
        sample_cv.notify_one();
        worker.join();
        std::cout<<"LIBCAMERA_WORKED5 pairs="<<pairs
                 <<" ransac_ok="<<ransac_ok
                 <<" worked5_valid="<<worked5_ok
                 <<" invalid_dt="<<invalid_dt
                 <<" captured="<<captured<<" dropped_latest="<<dropped
                 <<" mean_lk_ms="<<(pairs?lk_ms_sum/pairs:0)
                 <<" mean_worked5_ms="<<(ransac_ok?worked5_ms_sum/ransac_ok:0)
                 <<" max_pair_gap_ms="<<max_pair_gap_ms
                 <<" dt_source=SensorTimestamp association=SAME_LIBCAMERA_REQUEST"
                 <<" processing=SEPARATE_THREAD queue=LATEST_ONLY"
                 <<" calibration=PROVISIONAL height=SYNTHETIC no_fc_tx=1\n";
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
        return received>0 && worked5_ok>0 && missing_ts==0 && nonmono==0 && valid==received ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr<<"REQUEST_PROBE_FAIL "<<e.what()<<"\n";
        return 2;
    }
}