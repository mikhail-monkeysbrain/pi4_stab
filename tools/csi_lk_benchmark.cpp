// Standalone CSI + LK/RANSAC + frozen WORKED5 estimator throughput test.
// NOT the full production pipeline: no IMU, no rangefinder, no FC, no MAVLink.
// Metric output intentionally disabled until OV5647 calibration.
#include "csi_gray_camera.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
int main(){
  try {
    cv::setNumThreads(1);
    CsiGrayCamera cam;
    cam.open(60);
    cv::Mat prev;
    int64_t prev_ts=0,first_ts=0,last_ts=0;
    uint64_t frames=0,valid=0,queue_drops=0,lk_pairs=0;
    double total_ms=0,max_ms=0;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(20);
    while(std::chrono::steady_clock::now()<deadline){
      CsiGrayCamera::Frame f;
      if(!cam.take(f,100)) continue;
      if(f.gray.empty()) continue;
      if(!first_ts) first_ts=f.sensor_ns;
      last_ts=f.sensor_ns;
      ++frames;
      queue_drops+=f.dropped;
      if(!prev.empty() && f.sensor_ns>prev_ts){
        const auto t0=std::chrono::steady_clock::now();
        std::vector<cv::Point2f> a,b,aa,bb;
        std::vector<uchar> ok;
        std::vector<float> err;
        cv::goodFeaturesToTrack(prev,a,500,0.01,8.0);
        if(a.size()>=20){
          cv::calcOpticalFlowPyrLK(prev,f.gray,a,b,ok,err);
          for(size_t i=0;i<a.size();++i)
            if(ok[i]) {aa.push_back(a[i]);bb.push_back(b[i]);}
          if(aa.size()>=20){
            cv::Mat mask;
            cv::estimateAffinePartial2D(aa,bb,mask,cv::RANSAC,3.0);
            int inliers=mask.empty()?0:cv::countNonZero(mask);
            if(inliers>=20) ++valid;
          }
        }
        const double ms=std::chrono::duration<double,std::milli>(
          std::chrono::steady_clock::now()-t0).count();
        total_ms+=ms;max_ms=std::max(max_ms,ms);++lk_pairs;
      }
      prev=std::move(f.gray);
      prev_ts=f.sensor_ns;
    }
    cam.close();
    std::cout<<"CSI_LK_FRAMES "<<frames<<"\n"
             <<"CSI_LK_SENSOR_FPS "<<(frames>1 && last_ts>first_ts?(frames-1)*1e9/(last_ts-first_ts):0)<<"\n"
             <<"CSI_LK_QUEUE_DROPS "<<queue_drops<<"\n"
             <<"CSI_LK_PAIRS "<<lk_pairs<<"\n"
             <<"CSI_LK_RANSAC_VALID "<<valid<<"\n"
             <<"CSI_LK_MEAN_MS "<<(lk_pairs?total_ms/lk_pairs:0)<<"\n"
             <<"CSI_LK_MAX_MS "<<max_ms<<"\n";
    return frames>0?0:1;
  } catch(const std::exception &e) {
    std::cerr<<"CSI_LK_ERROR "<<e.what()<<"\n";
    return 1;
  }
}
