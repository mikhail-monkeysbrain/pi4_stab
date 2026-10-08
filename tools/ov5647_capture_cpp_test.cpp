#include "ov5647_pipe_capture.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <opencv2/imgproc.hpp>
int main(){
  try{
    Ov5647PipeCapture cap;
    cap.start(60);
    uint64_t seq=0,frames=0,dropped=0;
    const auto start=std::chrono::steady_clock::now();
    auto prev=start;
    double max_gap_ms=0, mean_luma=0;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<15){
      cv::Mat gray;
      const auto before=seq;
      if(cap.latest(gray,seq)){
        if(gray.rows!=480 || gray.cols!=640 || gray.type()!=CV_8UC1)
          throw std::runtime_error("unexpected image geometry/type");
        if(before && seq>before+1) dropped+=seq-before-1;
        const auto now=std::chrono::steady_clock::now();
        if(frames) max_gap_ms=std::max(max_gap_ms,
          std::chrono::duration<double,std::milli>(now-prev).count());
        prev=now;
        mean_luma+=cv::mean(gray)[0];
        ++frames;
      }else{
        if(!cap.running()) throw std::runtime_error("camera pipe stopped");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    const double seconds=std::chrono::duration<double>(
      std::chrono::steady_clock::now()-start).count();
    cap.stop();
    std::cout<<"OV5647_CXX_PASS frames="<<frames
             <<" fps="<<frames/seconds
             <<" skipped_latest="<<dropped
             <<" max_poll_gap_ms="<<max_gap_ms
             <<" mean_luma="<<(frames?mean_luma/frames:0)<<"\n";
    return frames>0?0:1;
  }catch(const std::exception& e){
    std::cerr<<"OV5647_CXX_FAIL: "<<e.what()<<"\n";
    return 1;
  }
}
