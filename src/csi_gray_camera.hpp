#pragma once
// CSI input adapter: libcamera Y plane -> latest grayscale frame + sensor timestamp.
// Isolated from WORKED5; USB V4L2 path remains unchanged.
#include <libcamera/libcamera.h>
#include <libcamera/formats.h>
#include <libcamera/control_ids.h>
#include <opencv2/core.hpp>
#include <sys/mman.h>
#include <unistd.h>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

class CsiGrayCamera {
public:
  struct Frame { cv::Mat gray; int64_t sensor_ns=0; int64_t received_ns=0; uint64_t dropped=0; };
  CsiGrayCamera() = default;
  CsiGrayCamera(const CsiGrayCamera&) = delete;
  CsiGrayCamera& operator=(const CsiGrayCamera&) = delete;
  ~CsiGrayCamera() { close(); }
  void open(int fps=60) {
    if(manager_.start()!=0) throw std::runtime_error("CSI camera manager start");
    if(manager_.cameras().empty()) throw std::runtime_error("CSI camera missing");
    camera_=manager_.cameras().front();
    if(camera_->acquire()!=0) throw std::runtime_error("CSI acquire");
    config_=camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if(!config_ || config_->empty()) throw std::runtime_error("CSI config");
    auto &cfg=config_->at(0);
    cfg.size={640,480};
    cfg.pixelFormat=libcamera::formats::YUV420;
    if(config_->validate()==libcamera::CameraConfiguration::Invalid)
      throw std::runtime_error("CSI unsupported format");
    if(cfg.size.width!=640 || cfg.size.height!=480 || cfg.pixelFormat!=libcamera::formats::YUV420)
      throw std::runtime_error("CSI camera adjusted format unexpectedly");
    if(camera_->configure(config_.get())!=0) throw std::runtime_error("CSI configure");
    stride_=cfg.stride;
    allocator_=std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    auto *stream=cfg.stream();
    if(allocator_->allocate(stream)<0) throw std::runtime_error("CSI allocate");
    for(auto &buffer:allocator_->buffers(stream)){
      const auto &plane=buffer->planes().front();
      const size_t bytes=plane.offset+plane.length;
      void *p=mmap(nullptr,bytes,PROT_READ,MAP_SHARED,plane.fd.get(),0);
      if(p==MAP_FAILED) throw std::runtime_error("CSI mmap");
      mapped_.push_back({p,bytes});
      planes_[buffer.get()]=static_cast<const uint8_t*>(p)+plane.offset;
      auto req=camera_->createRequest();
      if(!req || req->addBuffer(stream,buffer.get())!=0)
        throw std::runtime_error("CSI request");
      requests_.push_back(std::move(req));
    }
    camera_->requestCompleted.connect(this,&CsiGrayCamera::completed);
    const int64_t us=1000000/fps;
    const std::array<int64_t,2> duration{us,us};
    libcamera::ControlList ctrl(camera_->controls());
    ctrl.set(libcamera::controls::FrameDurationLimits,libcamera::Span<const int64_t,2>(duration));
    if(camera_->start(&ctrl)!=0) throw std::runtime_error("CSI start");
    running_.store(true);
    for(auto &req:requests_)
      if(camera_->queueRequest(req.get())!=0) throw std::runtime_error("CSI queue");
  }
  bool take(Frame &out,int wait_ms=20) {
    std::unique_lock<std::mutex> lk(mu_);
    if(!cv_.wait_for(lk,std::chrono::milliseconds(wait_ms),[&]{return ready_ || !running_.load();}))
      return false;
    if(!ready_) return false;
    out.gray=std::move(latest_);
    out.sensor_ns=latest_ts_;
    out.received_ns=latest_receive_;
    out.dropped=dropped_;
    dropped_=0;
    ready_=false;
    return true;
  }
  void close() {
    running_.store(false);
    cv_.notify_all();
    if(camera_){
      camera_->stop();
      camera_->requestCompleted.disconnect(this);
    }
    requests_.clear();
    planes_.clear();
    for(const auto &m:mapped_) munmap(m.first,m.second);
    mapped_.clear();
    allocator_.reset();
    if(camera_){camera_->release();camera_.reset();}
    config_.reset();
    manager_.stop();
  }
private:
  static int64_t monotonicNs(){
    timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);
    return static_cast<int64_t>(t.tv_sec)*1000000000LL+t.tv_nsec;
  }
  void completed(libcamera::Request *req){
    if(req->status()==libcamera::Request::RequestCancelled || !running_.load()) return;
    const auto sensor=req->metadata().get(libcamera::controls::SensorTimestamp);
    auto *buffer=req->buffers().begin()->second;
    auto it=planes_.find(buffer);
    if(sensor && it!=planes_.end()){
      cv::Mat y(480,640,CV_8UC1,const_cast<uint8_t*>(it->second),stride_);
      cv::Mat copied=y.clone();
      const int64_t receive=monotonicNs();
      {
        std::lock_guard<std::mutex> lk(mu_);
        if(ready_) ++dropped_;
        latest_=std::move(copied);
        latest_ts_=*sensor;
        latest_receive_=receive;
        ready_=true;
      }
      cv_.notify_one();
    }
    req->reuse(libcamera::Request::ReuseBuffers);
    if(running_.load()) camera_->queueRequest(req);
  }
  libcamera::CameraManager manager_;
  std::shared_ptr<libcamera::Camera> camera_;
  std::unique_ptr<libcamera::CameraConfiguration> config_;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<libcamera::Request>> requests_;
  std::map<libcamera::FrameBuffer*,const uint8_t*> planes_;
  std::vector<std::pair<void*,size_t>> mapped_;
  unsigned stride_=640;
  std::atomic<bool> running_{false};
  std::mutex mu_;
  std::condition_variable cv_;
  cv::Mat latest_;
  int64_t latest_ts_=0,latest_receive_=0;
  uint64_t dropped_=0;
  bool ready_=false;
};
