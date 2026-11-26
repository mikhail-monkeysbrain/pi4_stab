#pragma once
#include <linux/videodev2.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <iostream>
#include <thread>
#include <vector>

inline std::atomic<bool> g_running{true};
inline void onSignal(int){ g_running=false; }

inline int64_t monoNs(){
  timespec t{};
  if(clock_gettime(CLOCK_MONOTONIC,&t)!=0) throw std::runtime_error("clock_gettime");
  return (int64_t)t.tv_sec*1000000000LL+t.tv_nsec;
}

[[noreturn]] inline void fail(const std::string& s){
  throw std::runtime_error(s+": "+std::strerror(errno));
}

inline int xioctl(int fd,unsigned long req,void* arg){
  int r;
  do { r=ioctl(fd,req,arg); } while(r<0 && errno==EINTR);
  return r;
}

struct CameraBuffer { void* p=nullptr; size_t n=0; };

struct Camera {
  int fd=-1;
  std::atomic<int> exposure_absolute{50};
  int gain=0;
  std::vector<CameraBuffer> bufs;

  // OV9281_ASYNC_AE_V1: VIDIOC_S_CTRL takes ~90 ms on the production OV9281.
  // Never execute it in the realtime frame/OF thread. Only the newest request
  // matters; intermediate requests are deliberately coalesced.
  std::mutex exposure_mu;
  std::condition_variable exposure_cv;
  std::thread exposure_worker;
  bool exposure_stop=false;
  bool exposure_pending=false;
  int exposure_requested=50;

  ~Camera(){ close(); }

  void openDev(const std::string& dev){
    // WORKED5_INPUT_GUARD_V1
    // Promoted to frozen after explicit operator approval.
    // 120 FPS left too little CPU headroom on RPi5 during fast/jerky image
    // motion: queue drops increased the adjacent-frame interval to 16-28 ms,
    // which increased image displacement, degraded LK/RANSAC and could trigger
    // a positive-feedback reason5 cascade that permanently lost translation.
    //
    // 100 FPS increases the nominal frame budget from 8.33 ms to 10 ms while
    // keeping the WORKED5 estimator itself unchanged. Blind validation on the
    // test branch produced:
    //   GT 393 mm -> WORKED5 381.192 mm (-3.005%)
    //   GT 465 mm -> WORKED5 455.067 mm (-2.136%)
    // The second run had 2935/2935 valid frames with no reason5/reason6.
    constexpr int kWidth=640, kHeight=480, kCameraFps=60;
    constexpr int kExposureAbsolute=50, kGain=0;

    fd=::open(dev.c_str(),O_RDWR|O_NONBLOCK);
    if(fd<0) fail("open camera");

    v4l2_format f{};
    f.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f.fmt.pix.width=kWidth;
    f.fmt.pix.height=kHeight;
    f.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;
    f.fmt.pix.field=V4L2_FIELD_ANY;
    if(xioctl(fd,VIDIOC_S_FMT,&f)<0) fail("VIDIOC_S_FMT");
    if((int)f.fmt.pix.width!=kWidth || (int)f.fmt.pix.height!=kHeight ||
       f.fmt.pix.pixelformat!=V4L2_PIX_FMT_MJPEG)
      throw std::runtime_error("камера не приняла 640x480 MJPG");

    v4l2_streamparm sp{};
    sp.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sp.parm.capture.timeperframe.numerator=1;
    sp.parm.capture.timeperframe.denominator=kCameraFps;
    if(xioctl(fd,VIDIOC_S_PARM,&sp)<0) fail("VIDIOC_S_PARM");

    // Read back the accepted V4L2 frame period. This is diagnostic only: it
    // proves whether the camera/driver actually accepted the requested 100 FPS
    // and does not alter WORKED5 math or flow thresholds.
    v4l2_streamparm actual{};
    actual.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(fd,VIDIOC_G_PARM,&actual)<0) fail("VIDIOC_G_PARM");
    const auto num=actual.parm.capture.timeperframe.numerator;
    const auto den=actual.parm.capture.timeperframe.denominator;
    const double actual_fps=(num>0)?static_cast<double>(den)/num:0.0;
    std::cerr<<"WORKED5_INPUT_GUARD_V1 camera="
             <<kWidth<<"x"<<kHeight
             <<" MJPG requested_fps="<<kCameraFps
             <<" actual_fps="<<actual_fps<<"\n";

    auto setc=[&](uint32_t id,int32_t v){
      v4l2_control c{}; c.id=id; c.value=v;
      if(xioctl(fd,VIDIOC_S_CTRL,&c)<0) fail("VIDIOC_S_CTRL");
    };
    setc(V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_MANUAL);
    setc(V4L2_CID_EXPOSURE_AUTO_PRIORITY,0);
    setc(V4L2_CID_EXPOSURE_ABSOLUTE,kExposureAbsolute);
    setc(V4L2_CID_GAIN,kGain);
    exposure_absolute.store(kExposureAbsolute,std::memory_order_relaxed);
    gain=kGain;

    v4l2_requestbuffers rb{};
    rb.count=8;
    rb.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory=V4L2_MEMORY_MMAP;
    if(xioctl(fd,VIDIOC_REQBUFS,&rb)<0 || rb.count<2) fail("VIDIOC_REQBUFS");

    bufs.resize(rb.count);
    for(unsigned i=0;i<rb.count;i++){
      v4l2_buffer b{};
      b.type=rb.type; b.memory=rb.memory; b.index=i;
      if(xioctl(fd,VIDIOC_QUERYBUF,&b)<0) fail("VIDIOC_QUERYBUF");
      bufs[i].n=b.length;
      bufs[i].p=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
      if(bufs[i].p==MAP_FAILED) fail("mmap");
      if(xioctl(fd,VIDIOC_QBUF,&b)<0) fail("VIDIOC_QBUF");
    }
    v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(fd,VIDIOC_STREAMON,&t)<0) fail("VIDIOC_STREAMON");

    {
      std::lock_guard<std::mutex> lk(exposure_mu);
      exposure_stop=false;
      exposure_pending=false;
      exposure_requested=exposure_absolute.load(std::memory_order_relaxed);
    }
    exposure_worker=std::thread([this]{
      for(;;){
        int value=0;
        {
          std::unique_lock<std::mutex> lk(exposure_mu);
          exposure_cv.wait(lk,[this]{ return exposure_stop || exposure_pending; });
          if(exposure_stop) break;
          value=exposure_requested;
          exposure_pending=false;
        }

        v4l2_control c{};
        c.id=V4L2_CID_EXPOSURE_ABSOLUTE;
        c.value=value;
        const int64_t t0=monoNs();
        const int rc=xioctl(fd,VIDIOC_S_CTRL,&c);
        const double ms=(monoNs()-t0)*1e-6;
        if(rc<0){
          std::cerr<<"OV9281_ASYNC_AE_V1 set failed requested="<<value
                   <<" errno="<<errno<<" set_ms="<<ms<<"\n";
        } else {
          exposure_absolute.store(value,std::memory_order_release);
          std::cerr<<"OV9281_ASYNC_AE_V1 applied="<<value
                   <<" set_ms="<<ms<<"\n";
        }
      }
    });
  }

  bool requestExposureAbsolute(int value){
    if(fd<0) return false;
    value=std::clamp(value,1,200);
    {
      std::lock_guard<std::mutex> lk(exposure_mu);
      if(exposure_stop) return false;
      if(value==exposure_absolute.load(std::memory_order_acquire) &&
         !exposure_pending) return true;
      exposure_requested=value;
      exposure_pending=true;
    }
    exposure_cv.notify_one();
    return true;
  }

  int exposureAbsolute() const {
    return exposure_absolute.load(std::memory_order_acquire);
  }

  void stopExposureWorker(){
    {
      std::lock_guard<std::mutex> lk(exposure_mu);
      exposure_stop=true;
      exposure_pending=false;
    }
    exposure_cv.notify_one();
    if(exposure_worker.joinable()) exposure_worker.join();
  }

  void close(){
    stopExposureWorker();
    if(fd<0) return;
    v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd,VIDIOC_STREAMOFF,&t);
    for(auto& b:bufs) if(b.p && b.p!=MAP_FAILED) munmap(b.p,b.n);
    ::close(fd);
    fd=-1;
  }
};

struct LunaSample {
  double distance_m=0;
  int strength=0;
  int64_t recv_ns=0;
};

struct LunaState {
  std::mutex mu;
  double distance_m=0;
  int strength=0;
  int64_t recv_ns=0;
  bool valid=false;
  std::deque<LunaSample> history;
};

struct LunaReader {
  int fd=-1;
  std::thread th;
  LunaState state;
  ~LunaReader(){ stop(); }

  void start(const std::string& dev){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd<0) fail("open TF-Luna");
    termios t{};
    if(tcgetattr(fd,&t)<0) fail("TF-Luna tcgetattr");
    cfmakeraw(&t);
    cfsetispeed(&t,B115200); cfsetospeed(&t,B115200);
    t.c_cflag|=CLOCAL|CREAD;
    t.c_cflag&=~CSTOPB; t.c_cflag&=~CRTSCTS;
    if(tcsetattr(fd,TCSANOW,&t)<0) fail("TF-Luna tcsetattr");
    tcflush(fd,TCIFLUSH);

    th=std::thread([this]{
      std::vector<uint8_t> q;
      uint8_t tmp[128];
      while(g_running){
        pollfd p{fd,POLLIN,0};
        if(poll(&p,1,50)<=0) continue;
        const ssize_t n=read(fd,tmp,sizeof(tmp));
        if(n<=0) continue;
        q.insert(q.end(),tmp,tmp+n);
        while(q.size()>=9){
          size_t s=0;
          while(s+1<q.size() && !(q[s]==0x59 && q[s+1]==0x59)) ++s;
          if(s){
            q.erase(q.begin(),q.begin()+s);
            if(q.size()<9) break;
          }
          unsigned sum=0;
          for(int i=0;i<8;i++) sum+=q[i];
          const bool ok=((sum&0xff)==q[8]);
          const uint16_t d=q[2]|(uint16_t(q[3])<<8);
          const uint16_t st=q[4]|(uint16_t(q[5])<<8);
          if(ok && d>0){
            const int64_t recv_ns=monoNs();
            std::lock_guard<std::mutex> l(state.mu);
            state.distance_m=d/100.0;
            state.strength=st;
            state.recv_ns=recv_ns;
            state.valid=true;
            state.history.push_back(LunaSample{state.distance_m,state.strength,recv_ns});
            while(state.history.size()>2 && recv_ns-state.history.front().recv_ns>3000000000LL)
              state.history.pop_front();
          }
          q.erase(q.begin(),q.begin()+9);
        }
      }
    });
  }

  bool latest(double* d,int* s,int64_t* t){
    std::lock_guard<std::mutex> l(state.mu);
    if(!state.valid) return false;
    *d=state.distance_m; *s=state.strength; *t=state.recv_ns;
    return true;
  }

  std::deque<LunaSample> historySnapshot(){
    std::lock_guard<std::mutex> l(state.mu);
    return state.history;
  }

  void stop(){
    if(th.joinable()) th.join();
    if(fd>=0){ ::close(fd); fd=-1; }
  }
};

struct CameraCalib {
  double fx=0,fy=0,cx=0,cy=0;
  cv::Matx33d B_R_C=cv::Matx33d::eye();
  cv::Mat K,D;
};

inline CameraCalib loadCameraCalib(const std::string& path){
  cv::FileStorage fs(path,cv::FileStorage::READ);
  if(!fs.isOpened()) throw std::runtime_error("не удалось открыть camera yaml");
  std::vector<double> intr,dist,data;
  fs["intrinsics"]>>intr;
  fs["distortion_coefficients"]>>dist;
  cv::FileNode tbs=fs["T_BS"];
  tbs["data"]>>data;
  if(intr.size()<4 || data.size()!=16)
    throw std::runtime_error("camera yaml: неверные intrinsics/T_BS");
  CameraCalib c;
  c.fx=intr[0]; c.fy=intr[1]; c.cx=intr[2]; c.cy=intr[3];
  for(int r=0;r<3;r++) for(int k=0;k<3;k++) c.B_R_C(r,k)=data[r*4+k];
  c.K=(cv::Mat_<double>(3,3)<<c.fx,0,c.cx,0,c.fy,c.cy,0,0,1);
  c.D=cv::Mat(dist).clone().reshape(1,1);
  return c;
}

inline double median(std::vector<double> v){
  if(v.empty()) return 0.0;
  const size_t n=v.size()/2;
  std::nth_element(v.begin(),v.begin()+n,v.end());
  double m=v[n];
  if(v.size()%2==0){
    std::nth_element(v.begin(),v.begin()+n-1,v.end());
    m=0.5*(m+v[n-1]);
  }
  return m;
}