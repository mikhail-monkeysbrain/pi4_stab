            // Image and SensorTimestamp originate from the same completed Request.
            if (sensor_ts && !gray.empty()) {
                const int64_t ts=*sensor_ts;
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
                            cv::calcOpticalFlowPyrLK(prev,gray,pts,next,ok,err);
                            lk_ms_sum+=std::chrono::duration<double,std::milli>(
                                std::chrono::steady_clock::now()-t0).count();
                            std::vector<cv::Point2f> a,b;
                            for (size_t i=0;i<ok.size();++i) if (ok[i]) {
                                a.push_back(pts[i]); b.push_back(next[i]);
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
                                        std::isfinite(result.dv_norm)) ++worked5_ok;
                                }
                            }
                        }
                    }
                }
                prev=gray;
                prev_sensor_ts=ts;
            }
            if (sensor_ts && !gray.empty()) {
                std::lock_guard<std::mutex> lk(sample_mu);
                ++captured;
                if (!samples.empty()) { dropped+=samples.size(); samples.clear(); }
                samples.push_back({std::move(gray), *sensor_ts});
                sample_cv.notify_one();
            }
            r->reuse(Request::ReuseBuffers);
            if (camera->queueRequest(r)) throw std::runtime_error("requeue failed");
        }
        camera->stop();
        {
            std::lock_guard<std::mutex> lk(sample_mu);
            producer_done=true;
        }
        sample_cv.notify_one();
        worker.join();
        std::cout<<"LIBCAMERA_WORKED5 pairs="<<pairs
                 <<" ransac_ok="<<ransac_ok
                 <<" worked5_valid="<<worked5_ok
                 <<" invalid_dt="<<invalid_dt
                 <<" captured="<<captured<<" dropped_latest="<<dropped
                 <<" mean_lk_ms="<<(pairs?lk_ms_sum/pairs:0)
                 <<" mean_worked5_ms="<<(ransac_ok?worked5_ms_sum/ransac_ok:0)
                 <<" max_pair_gap_ms="<<max_pair_gap_ms
                 <<" dt_source=SensorTimestamp association=SAME_LIBCAMERA_REQUEST"
                 <<" processing=SEPARATE_THREAD queue=LATEST_ONLY"
                 <<" calibration=PROVISIONAL height=SYNTHETIC no_fc_tx=1\n";
        std::cout<<"REQUEST_FINAL received="<<received<<" valid="<<valid
                 <<" missing_ts="<<missing_ts<<" nonmonotonic_ts="<<nonmono
                 <<" cancelled="<<cancelled
                 <<" min_gap_ms="<<(min_gap==INT64_MAX?0:min_gap/1e6)
                 <<" max_gap_ms="<<max_gap/1e6
                 <<" association=SAME_LIBCAMERA_REQUEST"
                 <<" no_fc_tx=1 no_worked5_change=1\n";
        camera->requestCompleted.disconnect(&completed, &Completed::done);
        camera->release();
        // CameraManager destructor runs after camera, requests and allocator destructors.
        // Explicit stop here would remove media devices while objects are still alive.
        return received>0 && worked5_ok>0 && missing_ts==0 && nonmono==0 && valid==received ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr<<"REQUEST_PROBE_FAIL "<<e.what()<<"\n";
        return 2;
    }
}
