// Real frozen worked5::estimate() invoked on OV5647 LK/RANSAC inliers.
// DIAGNOSTIC ONLY: K is provisional, height is synthetic 1m, no FC output.
// No metric result is reported or trusted.
#include "ov5647_pipe_capture.hpp"
#include "worked5_estimator.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

int main(){
  try{
    cv::setNumThreads(2);
    Ov5647PipeCapture cam; cam.start(60);
    cv::Mat prev;
    uint64_t seq=0,pairs=0,ransac_ok=0,w5_ok=0,skipped=0;
    double lk_ms_sum=0,w5_ms_sum=0,max_pair_gap_ms=0;
    // Provisional normalized-ray geometry, NOT an OV5647 calibration.
    const cv::Mat K=(cv::Mat_<double>(3,3)<<600.,0.,320.,0.,600.,240.,0.,0.,1.);
    const cv::Mat D=cv::Mat::zeros(1,5,CV_64F);
    auto prev_time=std::chrono::steady_clock::now();
    const auto start=prev_time;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<15){
      cv::Mat gray;
      const auto old=seq;
      if(!cam.latest(gray,seq)){
        if(!cam.running()) throw std::runtime_error("camera stopped");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      if(old && seq>old+1) skipped+=seq-old-1;
      const auto now=std::chrono::steady_clock::now();
      const double dt=std::chrono::duration<double>(now-prev_time).count();
      prev_time=now;
      if(!prev.empty()){
        ++pairs;
        max_pair_gap_ms=std::max(max_pair_gap_ms,dt*1000.);
        std::vector<cv::Point2f> p,q;
        cv::goodFeaturesToTrack(prev,p,300,0.01,8);
        if(p.size()>=20){
          std::vector<uchar> ok;
          std::vector<float> err;
          const auto a=std::chrono::steady_clock::now();
          cv::calcOpticalFlowPyrLK(prev,gray,p,q,ok,err);
          lk_ms_sum+=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-a).count();
          std::vector<cv::Point2f> good_p,good_q;
          for(size_t i=0;i<ok.size();++i)if(ok[i]){
            good_p.push_back(p[i]);good_q.push_back(q[i]);
          }
          if(good_p.size()>=20){
            cv::Mat mask;
            cv::estimateAffinePartial2D(good_p,good_q,mask,cv::RANSAC,3.0);
            if(!mask.empty() && cv::countNonZero(mask)>=20){
              ++ransac_ok;
              std::vector<cv::Point2f> in_p,in_q;
              for(int i=0;i<mask.rows;++i){
                if(mask.at<uchar>(i,0)){
                  in_p.push_back(good_p[(size_t)i]);
                  in_q.push_back(good_q[(size_t)i]);
                }
              }
              const auto b=std::chrono::steady_clock::now();
              // 1.0m is a synthetic test argument ONLY, not an altitude estimate.
              const auto s=worked5::estimate(in_p,in_q,K,1.0,D,1.0,dt);
              w5_ms_sum+=std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-b).count();
              if(s.valid && std::isfinite(s.du_norm) && std::isfinite(s.dv_norm))
                ++w5_ok;
            }
          }
        }
      }
      prev=gray;
    }
    cam.stop();
    std::cout<<"OV5647_WORKED5_DIAG pairs="<<pairs
             <<" ransac_ok="<<ransac_ok
             <<" worked5_valid="<<w5_ok
             <<" skipped_frames="<<skipped
             <<" mean_lk_ms="<<(pairs?lk_ms_sum/pairs:0)
             <<" mean_worked5_ms="<<(ransac_ok?w5_ms_sum/ransac_ok:0)
             <<" max_pair_gap_ms="<<max_pair_gap_ms
             <<" calibration=PROVISIONAL height=SYNTHETIC no_fc=1\n";
    return w5_ok>0?0:1;
  }catch(const std::exception& e){
    std::cerr<<"OV5647_WORKED5_FAIL "<<e.what()<<"\n";
    return 1;
  }
}
