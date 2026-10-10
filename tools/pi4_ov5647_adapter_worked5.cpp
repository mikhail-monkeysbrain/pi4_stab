// Pi4 OV5647 adapter -> unchanged WORKED5 estimator, diagnostic only.
#include "pi4_ov5647_capture.hpp"
#include "worked5_estimator.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

int main(int argc,char **argv) {
    try {
        const int seconds=argc>1?std::stoi(argv[1]):15;
        if(seconds<1 || seconds>120) return 2;
        pi4_capture::Ov5647Capture camera;
        camera.start();
        cv::setNumThreads(2);
        cv::Mat prev;
        int64_t prev_sensor_ts=0;
        uint64_t pairs=0, ransac_ok=0, worked5_ok=0, invalid_dt=0;
        double lk_ms_sum=0, worked5_ms_sum=0, max_pair_gap_ms=0;
        // Provisional geometry and synthetic height: no metric navigation claims.
        const cv::Mat K=(cv::Mat_<double>(3,3)<<600.,0.,320.,0.,600.,240.,0.,0.,1.);
        const cv::Mat D=cv::Mat::zeros(1,5,CV_64F);
        struct Sample { cv::Mat gray; int64_t ts; };
        std::mutex sample_mu;
        std::condition_variable sample_cv;
        std::deque<Sample> samples;
        bool producer_done=false;
        std::mutex overlay_mu;
        std::vector<cv::Point2f> overlay_points;
        uint64_t captured=0, dropped=0;
        // Optional third argument: diagnostic per-step CSV, no FC publishing.
        std::ofstream steps_csv;
        int stream_fd=-1;
        const char *stream_path=std::getenv("PI4_FLOW_SOCKET");
        if(stream_path && *stream_path) {
            stream_fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK,0);
            if(stream_fd<0) throw std::runtime_error("flow socket create failed");
        }
        if (argc > 2) {
            steps_csv.open(argv[2]);
            if (!steps_csv) throw std::runtime_error("cannot open WORKED5 steps CSV");
            steps_csv << "sensor_ts_ns,dt_s,points,du_norm,dv_norm,scale_per_s,yaw_per_s,synthetic_dx_m,synthetic_dy_m,metric_valid\n";
        }
        std::thread worker([&] {
            for (;;) {
                Sample sample;
                {
                    std::unique_lock<std::mutex> lk(sample_mu);
                    sample_cv.wait(lk,[&]{return producer_done || !samples.empty();});
                    if (samples.empty()) {
                        if (producer_done) break;
                        continue;
                    }
                    sample=std::move(samples.front());
                    samples.pop_front();
                }
                const int64_t ts=sample.ts;
                if (!prev.empty() && prev_sensor_ts>0) {
                    const double dt=(ts-prev_sensor_ts)*1e-9;
                    ++pairs;
                    max_pair_gap_ms=std::max(max_pair_gap_ms,dt*1000.);
                    if (!(dt>0. && dt<0.2)) {
                        ++invalid_dt;
                    } else {
                        std::vector<cv::Point2f> pts,next;
                        cv::goodFeaturesToTrack(prev,pts,300,0.01,8);
                        if (pts.size()>=20) {
                            std::vector<uchar> ok;
                            std::vector<float> err;
                            const auto t0=std::chrono::steady_clock::now();
                            cv::calcOpticalFlowPyrLK(prev,sample.gray,pts,next,ok,err);
                            lk_ms_sum+=std::chrono::duration<double,std::milli>(
                                std::chrono::steady_clock::now()-t0).count();
                            std::vector<cv::Point2f> a,b;
                            for (size_t i=0;i<ok.size();++i) if (ok[i]) {
                                a.push_back(pts[i]); b.push_back(next[i]);
                            }
                            {
                                std::lock_guard<std::mutex> lk(overlay_mu);
                                overlay_points=b;
                            }
                            if (a.size()>=20) {
                                cv::Mat mask;
                                cv::estimateAffinePartial2D(a,b,mask,cv::RANSAC,3.0);
                                if (!mask.empty() && cv::countNonZero(mask)>=20) {
                                    ++ransac_ok;
                                    std::vector<cv::Point2f> in_a,in_b;
                                    for (int i=0;i<mask.rows;++i) if (mask.at<uchar>(i,0)) {
                                        in_a.push_back(a[(size_t)i]);
                                        in_b.push_back(b[(size_t)i]);
                                    }
                                    const auto t1=std::chrono::steady_clock::now();
                                    const auto result=worked5::estimate(in_a,in_b,K,1.0,D,1.0,dt);
                                    worked5_ms_sum+=std::chrono::duration<double,std::milli>(
                                        std::chrono::steady_clock::now()-t1).count();
                                    if (result.valid && std::isfinite(result.du_norm) &&
                                        std::isfinite(result.dv_norm)) {
                                        ++worked5_ok;
                                        if(stream_fd>=0) {
                                            sockaddr_un addr{};
                                            addr.sun_family=AF_UNIX;
                                            if(std::strlen(stream_path)<sizeof(addr.sun_path)) {
                                                std::strcpy(addr.sun_path,stream_path);
                                                std::string packet=std::to_string(ts)+","+std::to_string(dt)+","+std::to_string(result.points)+","+std::to_string(result.du_norm)+","+std::to_string(result.dv_norm)+","+std::to_string(result.scale)+","+std::to_string(result.yaw)+",0";
                                                sendto(stream_fd,packet.data(),packet.size(),MSG_DONTWAIT,(sockaddr*)&addr,sizeof(addr));
                                            }
                                        }
                                        if (steps_csv) {
                                            steps_csv << ts << "," << dt << "," << result.points
                                                      << "," << result.du_norm << "," << result.dv_norm
                                                      << "," << result.scale << "," << result.yaw
                                                      << "," << result.dx_m << "," << result.dy_m
                                                      << ",0\n";
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                prev=std::move(sample.gray);
                prev_sensor_ts=ts;
            }
        });
        int preview_fd=-1;
        const char* preview_port=std::getenv("PI4_PREVIEW_UDP_PORT");
        sockaddr_in preview_addr{};
        if(preview_port && *preview_port) {
            preview_fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);
            preview_addr.sin_family=AF_INET;
            preview_addr.sin_port=htons(static_cast<uint16_t>(std::stoi(preview_port)));
            inet_pton(AF_INET,"127.0.0.1",&preview_addr.sin_addr);
        }
        auto next_preview=std::chrono::steady_clock::now();
        const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(seconds);
        while(std::chrono::steady_clock::now()<until) {
            pi4_capture::Frame frame;
            if(!camera.next(frame,1000)) continue;
            if(preview_fd>=0 && std::chrono::steady_clock::now()>=next_preview) {
                next_preview=std::chrono::steady_clock::now()+std::chrono::milliseconds(200);
                std::vector<uchar> jpg;
                cv::Mat annotated;
                cv::cvtColor(frame.gray,annotated,cv::COLOR_GRAY2BGR);
                cv::rectangle(annotated,cv::Point(128,154),cv::Point(512,432),cv::Scalar(0,220,255),2);
                std::vector<cv::Point2f> points;
                { std::lock_guard<std::mutex> lk(overlay_mu); points=overlay_points; }
                for(const auto& point:points) cv::circle(annotated,point,2,cv::Scalar(0,255,0),-1);
                cv::putText(annotated,"OV5647 WORKED5 tracks: "+std::to_string(points.size()),
                            cv::Point(8,25),cv::FONT_HERSHEY_SIMPLEX,0.55,cv::Scalar(0,255,255),1);
                if(cv::imencode(".jpg",annotated,jpg,{cv::IMWRITE_JPEG_QUALITY,55}) && jpg.size()+4<60000) {
                    std::vector<uchar> payload={77,74,80,71};
                    payload.insert(payload.end(),jpg.begin(),jpg.end());
                    sendto(preview_fd,payload.data(),payload.size(),MSG_DONTWAIT,
                           reinterpret_cast<sockaddr*>(&preview_addr),sizeof(preview_addr));
                }
            }
            {
                std::lock_guard<std::mutex> lk(sample_mu);
                ++captured;
                if(!samples.empty()) { dropped+=samples.size(); samples.clear(); }
                samples.push_back({std::move(frame.gray),frame.sensor_timestamp_ns});
            }
            sample_cv.notify_one();
        }
        const auto stats=camera.stats();
        camera.stop();
        {
            std::lock_guard<std::mutex> lk(sample_mu);
            producer_done=true;
        }
        sample_cv.notify_one();
        worker.join();
        if(steps_csv) steps_csv.flush();
        if(stream_fd>=0) close(stream_fd);
        if(preview_fd>=0) close(preview_fd);
        std::cout<<"PI4_ADAPTER_WORKED5 pairs="<<pairs
                 <<" ransac_ok="<<ransac_ok<<" worked5_valid="<<worked5_ok
                 <<" invalid_dt="<<invalid_dt<<" captured="<<captured
                 <<" dropped_latest="<<dropped<<" mean_lk_ms="<<(pairs?lk_ms_sum/pairs:0)
                 <<" mean_worked5_ms="<<(ransac_ok?worked5_ms_sum/ransac_ok:0)
                 <<" max_pair_gap_ms="<<max_pair_gap_ms
                 <<" calibration=PROVISIONAL height=SYNTHETIC no_fc_tx=1 no_worked5_change=1\n";
        std::cout<<"PI4_ADAPTER_CAPTURE received="<<stats.received
                 <<" missing_ts="<<stats.missing_timestamp
                 <<" nonmonotonic_ts="<<stats.nonmonotonic_timestamp
                 <<" cancelled="<<stats.cancelled<<"\n";
        return worked5_ok>0 && stats.missing_timestamp==0 &&
               stats.nonmonotonic_timestamp==0 ? 0:1;
    } catch(const std::exception &e) {
        std::cerr<<"PI4_ADAPTER_WORKED5_FAIL "<<e.what()<<"\n";
        return 2;
    }
}