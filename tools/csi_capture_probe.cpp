// CSI OV5647: isolated native libcamera capture/timestamp diagnostic.
// No MAVLink, no FC commands, no changes to WORKED5 or USB camera path.
#include <libcamera/libcamera.h>
#include <atomic>
#include <array>
#include <libcamera/formats.h>
#include <libcamera/control_ids.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
class Capture {
public:
  void run(int seconds, int fps) {
    if (manager_.start() != 0) throw std::runtime_error("CameraManager::start failed");
    if (manager_.cameras().empty()) throw std::runtime_error("no libcamera camera");
    camera_ = manager_.cameras().front();
    if (camera_->acquire() != 0) throw std::runtime_error("camera acquire failed");
    config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!config_ || config_->empty()) throw std::runtime_error("no viewfinder configuration");
    auto &cfg = config_->at(0);
    cfg.size = {640, 480};
    cfg.pixelFormat = libcamera::formats::YUV420;
    const auto validation = config_->validate();
    if (validation == libcamera::CameraConfiguration::Invalid)
      throw std::runtime_error("invalid 640x480 YUV420 configuration");
    std::cout << "CONFIG size=" << cfg.size.toString()
              << " format=" << cfg.pixelFormat.toString()
              << " validation=" << static_cast<int>(validation) << "\n";
    if (camera_->configure(config_.get()) != 0)
      throw std::runtime_error("configure failed");
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    auto *stream = config_->at(0).stream();
    if (allocator_->allocate(stream) < 0) throw std::runtime_error("buffer allocation failed");
    for (auto &buffer : allocator_->buffers(stream)) {
      auto req = camera_->createRequest();
      if (!req || req->addBuffer(stream, buffer.get()) != 0)
        throw std::runtime_error("request setup failed");
      requests_.push_back(std::move(req));
    }
    camera_->requestCompleted.connect(this, &Capture::completed);
    libcamera::ControlList controls(camera_->controls());
    const int64_t frame_us = 1000000 / fps;
    const std::array<int64_t, 2> durations{frame_us, frame_us};
    controls.set(libcamera::controls::FrameDurationLimits,
                 libcamera::Span<const int64_t, 2>(durations));
    if (camera_->start(&controls) != 0) throw std::runtime_error("camera start failed");
    running_.store(true);
    for (auto &req : requests_) {
      if (camera_->queueRequest(req.get()) != 0)
        throw std::runtime_error("queueRequest failed");
    }
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    running_.store(false);
    camera_->stop();
    camera_->requestCompleted.disconnect(this);
    const auto n = frames_.load();
    const auto first = first_sensor_ns_.load();
    const auto last = last_sensor_ns_.load();
    std::cout << "FRAMES " << n << "\n";
    std::cout << "SENSOR_TIMESTAMPS " << sensor_timestamps_.load() << "\n";
    if (last > first && n > 1)
      std::cout << "SENSOR_FPS " << (double)(n-1)*1e9/(last-first) << "\n";
    camera_->release();
    camera_.reset();
    allocator_.reset();
    manager_.stop();
  }
private:
  void completed(libcamera::Request *req) {
    if (req->status() == libcamera::Request::RequestCancelled || !running_.load()) return;
    const auto timestamp = req->metadata().get(libcamera::controls::SensorTimestamp);
    if (timestamp) {
      const int64_t ns = *timestamp;
      int64_t expected = 0;
      first_sensor_ns_.compare_exchange_strong(expected, ns);
      last_sensor_ns_.store(ns);
      ++sensor_timestamps_;
    }
    ++frames_;
    req->reuse(libcamera::Request::ReuseBuffers);
    if (running_.load()) camera_->queueRequest(req);
  }
  libcamera::CameraManager manager_;
  std::shared_ptr<libcamera::Camera> camera_;
  std::unique_ptr<libcamera::CameraConfiguration> config_;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<libcamera::Request>> requests_;
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> frames_{0}, sensor_timestamps_{0};
  std::atomic<int64_t> first_sensor_ns_{0}, last_sensor_ns_{0};
};
}
int main(int argc, char **argv) {
  try {
    int seconds = argc > 1 ? std::stoi(argv[1]) : 10;
    int fps = argc > 2 ? std::stoi(argv[2]) : 60;
    if (seconds < 1 || seconds > 120 || fps < 1 || fps > 62)
      throw std::runtime_error("usage: csi_capture_probe [seconds=10] [fps=60]");
    Capture capture;
    capture.run(seconds, fps);
  } catch (const std::exception &e) {
    std::cerr << "CSI PROBE ERROR: " << e.what() << "\n";
    return 1;
  }
}