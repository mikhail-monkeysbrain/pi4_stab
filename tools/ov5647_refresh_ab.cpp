// OV5647 feature-refresh A/B: retain tracked inlier points between frames.
// Diagnostic only. Provisional intrinsics, synthetic height, NO FC output.
// Does not alter production WORKED5 or production feature tracking.
#include "ov5647_pipe_capture.hpp"
#include "worked5_estimator.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
using Clock=std::chrono::steady_clock;
static double ms(Clock::time_point a,Clock::time_point b){
  return std::chrono::duration<double,std::milli>(b-a).count();
}
int main(int argc,char**argv){
  try{
    const int refresh=argc>1?std::atoi(argv[1]):1;
    const int limit=argc>2?std::atoi(argv[2]):150;
    if(refresh<1||refresh>30||limit<20||limit>1000)
      throw std::runtime_error("usage: ov5647_refresh_ab <refresh_interval 1..30> <points 20..1000>");
    cv::setNumThreads(2);
    Ov5647PipeCapture cam;cam.start(60);
    cv::Mat prev;
    std::vector<cv::Point2f> retained;
    uint64_t seq=0,pairs=0,valid=0,skipped=0,refreshes=0,forced=0,low_inliers=0;
    double feature_total=0,lk_total=0,ransac_total=0,w5_total=0;
    double found_total=0,tracked_total=0,inlier_total=0,max_gap=0;
    const cv::Mat K=(cv::Mat_<double>(3,3)<<600.,0.,320.,0.,600.,240.,0.,0.,1.);
    const cv::Mat D=cv::Mat::zeros(1,5,CV_64F);
    const auto start=Clock::now();
    auto prev_time=start;
    while(std::chrono::duration<double>(Clock::now()-start).count()<15){
      cv::Mat gray;
      const uint64_t old=seq;
      if(!cam.latest(gray,seq)){
        if(!cam.running())throw std::runtime_error("camera stopped");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      if(old&&seq>old+1)skipped+=seq-old-1;
      const auto now=Clock::now();
      const double dt=std::chrono::duration<double>(now-prev_time).count();
      prev_time=now;
      if(!prev.empty()){
        ++pairs;
        max_gap=std::max(max_gap,dt*1000.);
        std::vector<cv::Point2f> p,q;
        // On refresh=1 this reproduces the old per-pair detection behavior.
        const bool scheduled=(pairs-1)%refresh==0;
        const bool need_more=retained.size()<20;
        if(scheduled||need_more){
          if(need_more&&!scheduled)++forced;
          const auto a=Clock::now();
          cv::goodFeaturesToTrack(prev,p,limit,0.01,8);
          feature_total+=ms(a,Clock::now());
          ++refreshes;
        }else{
          p=retained;
        }
        found_total+=p.size();
        retained.clear();
        if(p.size()>=20){
          std::vector<uchar> status;
          std::vector<float> error;
          const auto a=Clock::now();
          cv::calcOpticalFlowPyrLK(prev,gray,p,q,status,error);
          lk_total+=ms(a,Clock::now());
          std::vector<cv::Point2f> gp,gq;
          for(size_t i=0;i<status.size();++i)if(status[i]){
            gp.push_back(p[i]);gq.push_back(q[i]);
          }
          tracked_total+=gp.size();
          if(gp.size()>=20){
            cv::Mat mask;
            const auto b=Clock::now();
            cv::estimateAffinePartial2D(gp,gq,mask,cv::RANSAC,3.0);
            ransac_total+=ms(b,Clock::now());
            if(!mask.empty()&&cv::countNonZero(mask)>=20){
              std::vector<cv::Point2f> ip,iq;
              for(int i=0;i<mask.rows;++i)if(mask.at<uchar>(i,0)){
                ip.push_back(gp[(size_t)i]);
                iq.push_back(gq[(size_t)i]);
              }
              inlier_total+=ip.size();
              retained=iq; // in CURRENT frame, for next prev->gray pair
              const auto c=Clock::now();
              const auto s=worked5::estimate(ip,iq,K,1.0,D,1.0,dt);
              w5_total+=ms(c,Clock::now());
              if(s.valid&&std::isfinite(s.du_norm)&&std::isfinite(s.dv_norm))++valid;
            }else ++low_inliers;
          }else ++low_inliers;
        }else ++low_inliers;
      }
      prev=gray;
    }
    const double sec=std::chrono::duration<double>(Clock::now()-start).count();
    cam.stop();
    std::cout<<"REFRESH_AB interval="<<refresh<<" limit="<<limit
      <<" pairs="<<pairs<<" fps="<<pairs/sec<<" valid="<<valid
      <<" valid_ratio="<<(pairs?double(valid)/pairs:0)
      <<" skipped="<<skipped<<" refreshes="<<refreshes
      <<" forced_refreshes="<<forced<<" low_inliers="<<low_inliers
      <<" avg_input="<<(pairs?found_total/pairs:0)
      <<" avg_tracked="<<(pairs?tracked_total/pairs:0)
      <<" avg_inliers="<<(pairs?inlier_total/pairs:0)
      <<" feature_ms_per_pair="<<(pairs?feature_total/pairs:0)
      <<" lk_ms_per_pair="<<(pairs?lk_total/pairs:0)
      <<" ransac_ms_per_pair="<<(pairs?ransac_total/pairs:0)
      <<" worked5_ms_per_pair="<<(pairs?w5_total/pairs:0)
      <<" max_pair_gap_ms="<<max_gap
      <<" calibration=PROVISIONAL height=SYNTHETIC no_fc=1\n";
    return valid?0:1;
  }catch(const std::exception&e){
    std::cerr<<"REFRESH_AB_FAIL "<<e.what()<<"\n";return 1;
  }
}
