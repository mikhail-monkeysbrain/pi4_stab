// OV5647 -> OpenCV LK/RANSAC input diagnostic. No FC, height or flight output.
// Does NOT claim to exercise the full production WORKED5 pipeline.
#include "ov5647_pipe_capture.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <chrono>
#include <iostream>
#include <thread>
int main(){
  try{
    cv::setNumThreads(2);
    Ov5647PipeCapture cam; cam.start(60);
    cv::Mat prev;
    uint64_t seq=0, pairs=0, accepted=0, rejected=0, skipped=0;
    double lk_total_ms=0, max_lk_ms=0;
    const auto t0=std::chrono::steady_clock::now();
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count()<15){
      cv::Mat gray;
      const uint64_t old=seq;
      if(!cam.latest(gray,seq)){
        if(!cam.running()) throw std::runtime_error("camera stopped");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      if(old && seq>old+1) skipped+=seq-old-1;
      if(!prev.empty()){
        ++pairs;
        std::vector<cv::Point2f> p,q;
        cv::goodFeaturesToTrack(prev,p,300,0.01,8);
        if(p.size()>=20){
          std::vector<uchar> ok;
          std::vector<float> err;
          const auto a=std::chrono::steady_clock::now();
          cv::calcOpticalFlowPyrLK(prev,gray,p,q,ok,err);
          const double ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-a).count();
          lk_total_ms+=ms;max_lk_ms=std::max(max_lk_ms,ms);
          std::vector<cv::Point2f> good_p,good_q;
          for(size_t i=0;i<ok.size();++i)if(ok[i]){
            good_p.push_back(p[i]);good_q.push_back(q[i]);
          }
          if(good_p.size()>=20){
            cv::Mat mask;
            cv::estimateAffinePartial2D(good_p,good_q,mask,cv::RANSAC,3.0);
            if(!mask.empty() && cv::countNonZero(mask)>=20) ++accepted;
            else ++rejected;
          }else ++rejected;
        }else ++rejected;
      }
      prev=gray;
    }
    cam.stop();
    std::cout<<"OV5647_LK_DIAG pairs="<<pairs<<" ransac_ok="<<accepted
             <<" rejected="<<rejected<<" skipped_frames="<<skipped
             <<" avg_lk_ms="<<(pairs?lk_total_ms/pairs:0)
             <<" max_lk_ms="<<max_lk_ms<<"\n";
    return pairs>0?0:1;
  }catch(const std::exception& e){
    std::cerr<<"OV5647_LK_FAIL "<<e.what()<<"\n";
    return 1;
  }
}
