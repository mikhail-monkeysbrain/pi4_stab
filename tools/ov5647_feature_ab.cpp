// Benchmark feature count with the same live OV5647 -> LK -> RANSAC ->
// frozen WORKED5 diagnostic. No FC output or real height.
#include "ov5647_pipe_capture.hpp"
#include "worked5_estimator.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <cstdlib>
int main(int argc,char**argv){
  try{
    const int limit=argc>1?std::atoi(argv[1]):300;
    if(limit<20||limit>1000) throw std::runtime_error("feature limit must be 20..1000");
    cv::setNumThreads(2);
    Ov5647PipeCapture cam;cam.start(60);
    cv::Mat prev;
    uint64_t seq=0,pairs=0,ok=0,skipped=0;
    double detected=0,tracked=0,inliers=0,feature_ms=0,lk_ms=0,ransac_ms=0,w5_ms=0;
    const cv::Mat K=(cv::Mat_<double>(3,3)<<600.,0.,320.,0.,600.,240.,0.,0.,1.);
    const cv::Mat D=cv::Mat::zeros(1,5,CV_64F);
    auto prev_time=std::chrono::steady_clock::now();
    const auto start=prev_time;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<15){
      cv::Mat gray;
      const auto old=seq;
      if(!cam.latest(gray,seq)){
        if(!cam.running())throw std::runtime_error("camera stopped");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      if(old&&seq>old+1)skipped+=seq-old-1;
      const auto now=std::chrono::steady_clock::now();
      const double dt=std::chrono::duration<double>(now-prev_time).count();
      prev_time=now;
      if(!prev.empty()){
        ++pairs;
        std::vector<cv::Point2f> p,q;
        const auto a=std::chrono::steady_clock::now();
        cv::goodFeaturesToTrack(prev,p,limit,0.01,8);
        const auto b=std::chrono::steady_clock::now();
        feature_ms+=std::chrono::duration<double,std::milli>(b-a).count();
        detected+=p.size();
        if(p.size()>=20){
          std::vector<uchar> status;std::vector<float> err;
          cv::calcOpticalFlowPyrLK(prev,gray,p,q,status,err);
          const auto c=std::chrono::steady_clock::now();
          lk_ms+=std::chrono::duration<double,std::milli>(c-b).count();
          std::vector<cv::Point2f> gp,gq;
          for(size_t i=0;i<status.size();++i)if(status[i]){
            gp.push_back(p[i]);gq.push_back(q[i]);
          }
          tracked+=gp.size();
          if(gp.size()>=20){
            cv::Mat mask;
            cv::estimateAffinePartial2D(gp,gq,mask,cv::RANSAC,3.0);
            const auto d=std::chrono::steady_clock::now();
            ransac_ms+=std::chrono::duration<double,std::milli>(d-c).count();
            if(!mask.empty()&&cv::countNonZero(mask)>=20){
              std::vector<cv::Point2f> ip,iq;
              for(int i=0;i<mask.rows;++i)if(mask.at<uchar>(i,0)){
                ip.push_back(gp[(size_t)i]);iq.push_back(gq[(size_t)i]);
              }
              inliers+=ip.size();
              const auto e=std::chrono::steady_clock::now();
              const auto s=worked5::estimate(ip,iq,K,1.0,D,1.0,dt);
              w5_ms+=std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-e).count();
              if(s.valid)++ok;
            }
          }
        }
      }
      prev=gray;
    }
    const double sec=std::chrono::duration<double>(
      std::chrono::steady_clock::now()-start).count();
    cam.stop();
    std::cout<<"FEATURE_AB limit="<<limit<<" pairs="<<pairs
             <<" fps="<<pairs/sec<<" valid="<<ok
             <<" valid_ratio="<<(pairs?double(ok)/pairs:0)
             <<" skipped="<<skipped
             <<" avg_detected="<<(pairs?detected/pairs:0)
             <<" avg_tracked="<<(pairs?tracked/pairs:0)
             <<" avg_inliers="<<(pairs?inliers/pairs:0)
             <<" feature_ms="<<(pairs?feature_ms/pairs:0)
             <<" lk_ms="<<(pairs?lk_ms/pairs:0)
             <<" ransac_ms="<<(pairs?ransac_ms/pairs:0)
             <<" worked5_ms="<<(pairs?w5_ms/pairs:0)
             <<" provisional_calibration=1 synthetic_height=1 no_fc=1\n";
    return ok?0:1;
  }catch(const std::exception&e){std::cerr<<"FEATURE_AB_FAIL "<<e.what()<<"\n";return 1;}
}
