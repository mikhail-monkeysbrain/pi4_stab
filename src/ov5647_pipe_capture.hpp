#pragma once
// OV5647 CSI capture transport for Raspberry Pi 4.
// Non-flight transport: pipe has no sensor timestamps. Do not use receive
// timestamps as gyro-synchronised capture timestamps in production.
#include <opencv2/core.hpp>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <sys/wait.h>

class Ov5647PipeCapture {
public:
  static constexpr int width=640, height=480;
  static constexpr size_t frame_size=width*height*3/2;
  Ov5647PipeCapture()=default;
  Ov5647PipeCapture(const Ov5647PipeCapture&)=delete;
  Ov5647PipeCapture& operator=(const Ov5647PipeCapture&)=delete;
  ~Ov5647PipeCapture(){stop();}

  void start(int fps=60){
    if(pid_>0) throw std::runtime_error("OV5647 already started");
    int fds[2];
    if(pipe(fds)!=0) throw std::runtime_error(std::strerror(errno));
    const pid_t child=fork();
    if(child<0){::close(fds[0]);::close(fds[1]);throw std::runtime_error("fork failed");}
    if(child==0){
      ::close(fds[0]);
      if(dup2(fds[1],STDOUT_FILENO)<0) _exit(127);
      ::close(fds[1]);
      const std::string fps_s=std::to_string(fps);
      execlp("rpicam-vid","rpicam-vid","--camera","0",
             "--width","640","--height","480","--framerate",fps_s.c_str(),
             "--codec","yuv420","--nopreview","--timeout","0",
             "--output","-",static_cast<char*>(nullptr));
      _exit(127);
    }
    ::close(fds[1]);
    fd_=fds[0];pid_=child;
    running_=true;
    worker_=std::thread([this]{
      std::vector<unsigned char> bytes(frame_size);
      while(running_){
        size_t offset=0;
        while(offset<bytes.size() && running_){
          const ssize_t n=::read(fd_,bytes.data()+offset,bytes.size()-offset);
          if(n>0) offset+=static_cast<size_t>(n);
          else if(n<0 && errno==EINTR) continue;
          else {running_=false;break;}
        }
        if(!running_) break;
        cv::Mat gray(height,width,CV_8UC1,bytes.data());
        {
          std::lock_guard<std::mutex> lk(mu_);
          latest_=gray.clone();
          ++sequence_;
        }
      }
    });
  }

  // Returns only the newest complete frame, never a FIFO backlog.
  // No camera sensor timestamp is available from this transport.
  bool latest(cv::Mat& gray,uint64_t& sequence){
    std::lock_guard<std::mutex> lk(mu_);
    if(sequence_==0 || sequence_==sequence) return false;
    gray=latest_.clone();
    sequence=sequence_;
    return true;
  }
  bool running() const{return running_.load();}
  void stop(){
    running_=false;
    if(pid_>0){::kill(pid_,SIGTERM);}
    if(fd_>=0){::close(fd_);fd_=-1;}
    if(worker_.joinable()) worker_.join();
    if(pid_>0){int status=0;::waitpid(pid_,&status,0);pid_=-1;}
  }
private:
  int fd_=-1;
  pid_t pid_=-1;
  std::atomic<bool> running_{false};
  std::thread worker_;
  std::mutex mu_;
  cv::Mat latest_;
  uint64_t sequence_=0;
};
