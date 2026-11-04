#pragma once
#include "../src/runtime.hpp"
#include <array>
#include <set>
#include <sys/syscall.h>
#include <time.h>

struct FlowStep {
  bool valid=false;
  int invalid_reason=0; // 0=OK,1=DT,2=FEATURES,3=TRACKED,4=HOMOGRAPHY,5=INLIERS,6=MAGNITUDE
  int features=0,tracked=0,inliers=0;
  bool feature_fallback=false;
  double t_features_ms=0.0,t_lk_ms=0.0,t_ransac_ms=0.0,t_post_ms=0.0;
  double inlier_ratio=0;
  double du_norm=0,dv_norm=0;
  double du_px=0,dv_px=0;
  double yaw_rate_cam_z=0; // fitted optical-axis rotation, rad/s, removed before MAVLink
  double scale_rate=0;      // fitted isotropic image scale rate, 1/s; removed from XY flow
  double lk_height_scale=1; // initial KLT scale guess from TF-Luna, curr image / prev image
  double flow_cam_x=0,flow_cam_y=0;
  double flow_body_x=0,flow_body_y=0;

  // V2 shadow: remove the full known camera rotation ΔR before fitting XY
  // translation/scale. Diagnostic only until A/B tests prove an improvement.
  // Lever-arm shadow: convert optical flow measured at the displaced camera
  // focal point to the FC/IMU reference point using v_cam = omega x r.
  // Proven by repeated bench yaw regression; used for production send when valid.
  bool lever_shadow_valid=false;
  double lever_flow_body_x=0.0,lever_flow_body_y=0.0;
  double lever_pred_flow_x=0.0,lever_pred_flow_y=0.0;
  // Diagnostic A/B shadow path. A is the production result above. B applies
  // forward/backward KLT consistency to the SAME forward correspondences, then
  // runs the same homography RANSAC and 4-parameter fit. B is never sent to FC.
  bool fb_shadow_valid=false;
  int fb_checked=0,fb_pass=0,fb_inliers=0;
  double fb_ratio=0.0;
  double fb_flow_body_x=0.0,fb_flow_body_y=0.0;
  double fb_t_ms=0.0;

  // C shadow: same FB-filtered + RANSAC inliers as B, but the final
  // translation/scale/yaw fit is Huber IRLS instead of ordinary LS.
  // The Huber scale is estimated independently on every frame from MAD of
  // signed 2-D residual components. C is diagnostic only and never published.
  bool robust_shadow_valid=false;
  double robust_flow_body_x=0.0,robust_flow_body_y=0.0;
  double robust_sigma=0.0;
  double robust_mean_weight=0.0;
  int robust_downweighted=0;
  int robust_iters=0;

  // D shadow: same B inliers, but the final 4-parameter fit is weighted by
  // local 2-D observability from the structure-tensor eigenvalue ratio.
  // Weights are normalized to the per-frame median ratio, so there is no
  // absolute brightness/gradient threshold to tune.
  bool obs_shadow_valid=false;
  double obs_flow_body_x=0.0,obs_flow_body_y=0.0;
  double obs_median_ratio=0.0;
  double obs_mean_weight=0.0;
  int obs_downweighted=0;

  std::vector<cv::Point2f> inlier_points; // current-frame RANSAC inliers for web diagnostics
  // Metric-shadow input: preserve BOTH sides of the exact production
  // homography-RANSAC inlier correspondences. Diagnostic only.
  std::vector<cv::Point2f> metric_prev_points;
  std::vector<cv::Point2f> metric_curr_points;

  // 3x3 spatial diagnostics inside the configured feature ROI.
  // Each cell stores median inlier flow transformed to body FRD.
  std::array<int,9> cell_n{};
  std::array<double,9> cell_body_x{};
  std::array<double,9> cell_body_y{};
};

FlowStep estimateRawFlow(const cv::Mat& prev,const cv::Mat& curr,double dt,const CameraCalib& calib,
                         double prev_camera_height_m=0.0,double curr_camera_height_m=0.0,
                         const cv::Matx33d* C1_R_C0=nullptr,double dr_interp_gap_ms=-1.0){
  // OPENCV_PARALLEL_PROBE_V1 -- one-shot diagnostic from the production flow thread.
  {
    static std::once_flag jtzero_parallel_probe_once;
    std::call_once(jtzero_parallel_probe_once, [] {
      std::mutex mu;
      std::set<long> tids;
      cv::parallel_for_(cv::Range(0, 4096), [&](const cv::Range& r) {
        const long tid = static_cast<long>(::syscall(SYS_gettid));
        {
          std::lock_guard<std::mutex> lk(mu);
          tids.insert(tid);
        }
        volatile double sink = 0.0;
        for (int i = r.start; i < r.end; ++i)
          for (int k = 0; k < 4000; ++k)
            sink += (i + 1) * 1e-12 + k * 1e-15;
        (void)sink;
      }, 64.0);
      std::cerr << "OPENCV_PARALLEL_PROBE workers=" << tids.size()
                << " configured_threads=" << cv::getNumThreads()
                << " cpus=" << cv::getNumberOfCPUs()
                << " tids=";
      for (long tid : tids) std::cerr << tid << ",";
      std::cerr << "\n";
    });
  }

  FlowStep o;
  if(prev.empty()||curr.empty()||!(dt>0&&dt<0.2)){ o.invalid_reason=1; return o; }

  const int x0=std::clamp((int)std::lround(g_feature_roi.x0*prev.cols),0,prev.cols-1);
  const int y0=std::clamp((int)std::lround(g_feature_roi.y0*prev.rows),0,prev.rows-1);
  const int x1=std::clamp((int)std::lround(g_feature_roi.x1*prev.cols),x0+1,prev.cols);
  const int y1=std::clamp((int)std::lround(g_feature_roi.y1*prev.rows),y0+1,prev.rows);

  std::vector<cv::Point2f> p0,p1;
  const int64_t t_feat0=monoNs();

  // Detect corners independently in a 3x3 grid. A single global GFTT call
  // normalises quality against the strongest corner in the whole ROI, so a
  // pair of bright/high-contrast patches can consume nearly all features.
  // Per-cell GFTT preserves the same qualityLevel/minDistance while allowing
  // weaker textured regions to contribute real corners. This also improves
  // conditioning of the downstream translation/scale/yaw fit.
  constexpr int kFeatureGrid=3;
  const int per_cell=std::max(1,(g_max_features+kFeatureGrid*kFeatureGrid-1)/
                                (kFeatureGrid*kFeatureGrid));
  p0.reserve(g_max_features);
  for(int gy=0;gy<kFeatureGrid;gy++){
    const int cy0=y0+(y1-y0)*gy/kFeatureGrid;
    const int cy1=y0+(y1-y0)*(gy+1)/kFeatureGrid;
    for(int gx=0;gx<kFeatureGrid;gx++){
      const int cx0=x0+(x1-x0)*gx/kFeatureGrid;
      const int cx1=x0+(x1-x0)*(gx+1)/kFeatureGrid;
      if(cx1<=cx0 || cy1<=cy0) continue;

      const cv::Rect cell(cx0,cy0,cx1-cx0,cy1-cy0);
      std::vector<cv::Point2f> local;
      cv::goodFeaturesToTrack(prev(cell),local,per_cell,0.01,7);
      for(auto p:local){
        p.x+=(float)cell.x;
        p.y+=(float)cell.y;
        p0.push_back(p);
        if((int)p0.size()>=g_max_features) break;
      }
      if((int)p0.size()>=g_max_features) break;
    }
    if((int)p0.size()>=g_max_features) break;
  }

  // If the normal ROI becomes texture-starved (typical when crossing a sharp
  // table/floor boundary), widen only the ground-facing part of the image and
  // relax the corner detector slightly. The top quarter stays excluded so the
  // frame/cables cannot become navigation features.
  if(p0.size()<30){
    cv::Mat fallback_mask(prev.size(),CV_8UC1,cv::Scalar(0));
    const int fx0=std::clamp((int)std::lround(0.05*prev.cols),0,prev.cols-1);
    const int fy0=std::clamp((int)std::lround(0.25*prev.rows),0,prev.rows-1);
    const int fx1=std::clamp((int)std::lround(0.95*prev.cols),fx0+1,prev.cols);
    const int fy1=std::clamp((int)std::lround(0.98*prev.rows),fy0+1,prev.rows);
    fallback_mask(cv::Rect(fx0,fy0,fx1-fx0,fy1-fy0)).setTo(255);
    std::vector<cv::Point2f> pf;
    cv::goodFeaturesToTrack(prev,pf,g_max_features,0.005,5,fallback_mask);
    if(pf.size()>p0.size()){
      p0.swap(pf);
      o.feature_fallback=true;
    }
  }

  o.t_features_ms=(monoNs()-t_feat0)*1e-6;
  o.features=(int)p0.size();
  if(p0.size()<30){ o.invalid_reason=2; return o; }

  std::vector<uchar> st; std::vector<float> err;

  // Do not derive KLT image scale directly from TF-Luna. At a terrain step the
  // range can jump although the vehicle did not move vertically. The visual
  // 4-parameter fit below estimates image scale from tracked features instead.
  o.lk_height_scale=1.0;

  // LK_RUNTIME_TIMING_V1
  // Diagnostic only. Production LK inputs, parameters and output are unchanged.
  timespec lk_thr0{}, lk_thr1{}, lk_proc0{}, lk_proc1{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID,&lk_thr0);
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&lk_proc0);
  const int64_t t_lk0=monoNs();
  cv::calcOpticalFlowPyrLK(prev,curr,p0,p1,st,err,{21,21},3,
                           cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01),
                           0,1e-4);
  o.t_lk_ms=(monoNs()-t_lk0)*1e-6;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&lk_proc1);
  clock_gettime(CLOCK_THREAD_CPUTIME_ID,&lk_thr1);
  const auto lk_ts_ms=[](const timespec& a,const timespec& b){
    return double(b.tv_sec-a.tv_sec)*1000.0+
           double(b.tv_nsec-a.tv_nsec)/1000000.0;
  };
  const double lk_thread_cpu_ms=lk_ts_ms(lk_thr0,lk_thr1);
  const double lk_process_cpu_ms=lk_ts_ms(lk_proc0,lk_proc1);
  if(o.t_lk_ms>20.0){
    std::cerr<<"LK_RUNTIME wall_ms="<<o.t_lk_ms
             <<" thread_cpu_ms="<<lk_thread_cpu_ms
             <<" process_cpu_ms="<<lk_process_cpu_ms
             <<" features="<<p0.size()<<"\n";
  }
  std::vector<cv::Point2f> a,b;
  for(size_t i=0;i<p0.size();++i){if(st[i]){a.push_back(p0[i]);b.push_back(p1[i]);}}
  o.tracked=(int)a.size();
  if(a.size()<20){ o.invalid_reason=3; return o; }

  // B shadow: forward/backward consistency on exactly the correspondences used
  // by production A. This makes A/B share frames, GFTT points, forward KLT and
  // dt; only the FB gate differs. The extra work exists only in explicit A/B
  // mode and cannot alter the flow that is published to ArduPilot.
  if(g_fb_shadow_max_px>0.0){
    const int64_t tfb0=monoNs();
    o.fb_checked=(int)a.size();
    std::vector<cv::Point2f> back;
    std::vector<uchar> st_back;
    std::vector<float> err_back;
    cv::calcOpticalFlowPyrLK(curr,prev,b,back,st_back,err_back,{21,21},3,
                             cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01),
                             0,1e-4);
    std::vector<cv::Point2f> af,bf;
    af.reserve(a.size()); bf.reserve(a.size());
    for(size_t i=0;i<a.size();++i){
      if(!st_back[i]) continue;
      const double fb_err=cv::norm(back[i]-a[i]);
      if(std::isfinite(fb_err) && fb_err<=g_fb_shadow_max_px){
        af.push_back(a[i]); bf.push_back(b[i]);
      }
    }
    o.fb_pass=(int)af.size();
    o.fb_ratio=o.fb_checked?((double)o.fb_pass/o.fb_checked):0.0;

    if(af.size()>=20){
      cv::Mat fmask;
      constexpr int kFbHomographyMaxIters=350;
      constexpr double kFbHomographyConfidence=0.99;
      cv::findHomography(af,bf,cv::RANSAC,2.0,fmask,
                         kFbHomographyMaxIters,kFbHomographyConfidence);
      if(!fmask.empty()){
        std::vector<cv::Point2f> afi,bfi;
        for(size_t i=0;i<af.size();++i){
          if(fmask.at<uchar>((int)i)){ afi.push_back(af[i]); bfi.push_back(bf[i]); }
        }
        o.fb_inliers=(int)afi.size();
        if(afi.size()>=20){
          std::vector<cv::Point2f> au_fb,bu_fb;
          cv::undistortPoints(afi,au_fb,calib.K,calib.D);
          cv::undistortPoints(bfi,bu_fb,calib.K,calib.D);

          cv::Mat A_fb((int)afi.size()*2,4,CV_64F);
          cv::Mat bb_fb((int)afi.size()*2,1,CV_64F);
          for(size_t k=0;k<afi.size();++k){
            const double x=(double)au_fb[k].x, y=(double)au_fb[k].y;
            const double du=(double)bu_fb[k].x-au_fb[k].x;
            const double dv=(double)bu_fb[k].y-au_fb[k].y;
            A_fb.at<double>((int)(2*k),0)=1.0;
            A_fb.at<double>((int)(2*k),1)=0.0;
            A_fb.at<double>((int)(2*k),2)=x;
            A_fb.at<double>((int)(2*k),3)=-y;
            bb_fb.at<double>((int)(2*k),0)=du;
            A_fb.at<double>((int)(2*k+1),0)=0.0;
            A_fb.at<double>((int)(2*k+1),1)=1.0;
            A_fb.at<double>((int)(2*k+1),2)=y;
            A_fb.at<double>((int)(2*k+1),3)=x;
            bb_fb.at<double>((int)(2*k+1),0)=dv;
          }
          cv::Mat sol_fb;
          if(cv::solve(A_fb,bb_fb,sol_fb,cv::DECOMP_SVD) && sol_fb.rows==4){
            const double du_fb=sol_fb.at<double>(0,0);
            const double dv_fb=sol_fb.at<double>(1,0);
            const double fcx=dv_fb/dt;
            const double fcy=-du_fb/dt;
            const cv::Matx33d FLU_TO_FRD_FB(1,0,0, 0,-1,0, 0,0,-1);
            const cv::Matx33d FRD_R_C_FB=FLU_TO_FRD_FB*calib.B_R_C;
            const cv::Vec3d fbody=FRD_R_C_FB*cv::Vec3d(fcx,fcy,0.0);
            o.fb_flow_body_x=fbody[0];
            o.fb_flow_body_y=fbody[1];
            const double fmag=std::hypot(o.fb_flow_body_x,o.fb_flow_body_y);
            o.fb_shadow_valid=std::isfinite(fmag) && fmag<4.0;

            // C shadow: Huber IRLS on the exact same afi/bfi set as B.
            // Start from ordinary LS, estimate robust scale from the signed
            // residual components, then solve weighted LS.  No hand-tuned
            // pixel residual threshold is introduced here.
            cv::Mat sol_r=sol_fb.clone();
            constexpr int kRobustMaxIters=5;
            constexpr double kHuberK=1.345;
            for(int iter=0;iter<kRobustMaxIters;iter++){
              std::vector<double> signed_res;
              signed_res.reserve(afi.size()*2);
              std::vector<double> rnorm(afi.size(),0.0);
              for(size_t k=0;k<afi.size();++k){
                const double x=(double)au_fb[k].x, y=(double)au_fb[k].y;
                const double du=(double)bu_fb[k].x-au_fb[k].x;
                const double dv=(double)bu_fb[k].y-au_fb[k].y;
                const double tx=sol_r.at<double>(0,0);
                const double ty=sol_r.at<double>(1,0);
                const double sc=sol_r.at<double>(2,0);
                const double wz=sol_r.at<double>(3,0);
                const double eu=du-(tx+sc*x-wz*y);
                const double ev=dv-(ty+sc*y+wz*x);
                signed_res.push_back(eu);
                signed_res.push_back(ev);
                rnorm[k]=std::hypot(eu,ev);
              }
              const double med_r=median(signed_res);
              std::vector<double> abs_dev;
              abs_dev.reserve(signed_res.size());
              for(double r:signed_res) abs_dev.push_back(std::abs(r-med_r));
              const double sigma=std::max(1e-7,1.4826*median(abs_dev));
              const double delta=kHuberK*sigma;

              cv::Mat Aw=A_fb.clone(), bw=bb_fb.clone();
              double wsum=0.0;
              int down=0;
              for(size_t k=0;k<afi.size();++k){
                const double rr=rnorm[k];
                const double w=(rr<=delta || rr<=1e-15)?1.0:(delta/rr);
                const double sw=std::sqrt(std::max(0.0,w));
                if(w<0.999999) down++;
                wsum+=w;
                const int r0=(int)(2*k), r1=r0+1;
                for(int c=0;c<4;c++){
                  Aw.at<double>(r0,c)*=sw;
                  Aw.at<double>(r1,c)*=sw;
                }
                bw.at<double>(r0,0)*=sw;
                bw.at<double>(r1,0)*=sw;
              }
              cv::Mat next;
              if(!cv::solve(Aw,bw,next,cv::DECOMP_SVD) || next.rows!=4) break;
              const double dsol=cv::norm(next-sol_r);
              sol_r=next;
              o.robust_sigma=sigma;
              o.robust_mean_weight=wsum/std::max<size_t>(1,afi.size());
              o.robust_downweighted=down;
              o.robust_iters=iter+1;
              if(dsol<1e-10) break;
            }
            if(o.robust_iters>0 && sol_r.rows==4){
              const double du_r=sol_r.at<double>(0,0);
              const double dv_r=sol_r.at<double>(1,0);
              const double rcx=dv_r/dt;
              const double rcy=-du_r/dt;
              const cv::Vec3d rbody=FRD_R_C_FB*cv::Vec3d(rcx,rcy,0.0);
              o.robust_flow_body_x=rbody[0];
              o.robust_flow_body_y=rbody[1];
              const double rmag=std::hypot(o.robust_flow_body_x,o.robust_flow_body_y);
              o.robust_shadow_valid=std::isfinite(rmag) && rmag<4.0;
            }

            if(g_obs_shadow_enabled){
            // D shadow: local aperture/conditioning test.  cornerEigenValsAndVecs
            // gives two structure-tensor eigenvalues per pixel.  Their ratio is
            // near zero for edge-like/one-dimensional texture and closer to one
            // for isotropic corners.  Normalize against the median ratio of this
            // very frame; therefore D asks whether relatively weak-axis tracks
            // are biasing the fit without introducing a global gradient cutoff.
            cv::Mat eig;
            cv::cornerEigenValsAndVecs(prev,eig,7,3);
            std::vector<double> q(afi.size(),0.0), qcopy;
            qcopy.reserve(afi.size());
            for(size_t k=0;k<afi.size();++k){
              const int px=std::clamp((int)std::lround(afi[k].x),0,prev.cols-1);
              const int py=std::clamp((int)std::lround(afi[k].y),0,prev.rows-1);
              const cv::Vec6f ev=eig.at<cv::Vec6f>(py,px);
              const double l1=std::max(0.0,(double)ev[0]);
              const double l2=std::max(0.0,(double)ev[1]);
              const double hi=std::max(l1,l2), lo=std::min(l1,l2);
              q[k]=(hi>1e-20)?(lo/hi):0.0;
              qcopy.push_back(q[k]);
            }
            const double qmed=std::max(1e-6,median(qcopy));
            cv::Mat Ao=A_fb.clone(), bo=bb_fb.clone();
            double owsum=0.0;
            int odown=0;
            for(size_t k=0;k<afi.size();++k){
              const double w=std::clamp(q[k]/qmed,0.0,1.0);
              const double sw=std::sqrt(w);
              if(w<0.999999) odown++;
              owsum+=w;
              const int r0=(int)(2*k), r1=r0+1;
              for(int c=0;c<4;c++){
                Ao.at<double>(r0,c)*=sw;
                Ao.at<double>(r1,c)*=sw;
              }
              bo.at<double>(r0,0)*=sw;
              bo.at<double>(r1,0)*=sw;
            }
            cv::Mat sol_o;
            if(cv::solve(Ao,bo,sol_o,cv::DECOMP_SVD) && sol_o.rows==4){
              const double du_o=sol_o.at<double>(0,0);
              const double dv_o=sol_o.at<double>(1,0);
              const double ocx=dv_o/dt;
              const double ocy=-du_o/dt;
              const cv::Vec3d obody=FRD_R_C_FB*cv::Vec3d(ocx,ocy,0.0);
              o.obs_flow_body_x=obody[0];
              o.obs_flow_body_y=obody[1];
              o.obs_median_ratio=qmed;
              o.obs_mean_weight=owsum/std::max<size_t>(1,afi.size());
              o.obs_downweighted=odown;
              const double omag=std::hypot(o.obs_flow_body_x,o.obs_flow_body_y);
              o.obs_shadow_valid=std::isfinite(omag) && omag<4.0;
            }
            }
          }
        }
      }
    }
    o.fb_t_ms=(monoNs()-tfb0)*1e-6;
  }

  cv::Mat mask;
  const int64_t t_ransac0=monoNs();
  // Bound worst-case runtime. With low inlier ratio OpenCV's default 2000
  // RANSAC iterations can dominate the camera interval and make the most
  // informative high-motion frames stale before MAVLink transmit.
  constexpr int kHomographyMaxIters=350;
  constexpr double kHomographyConfidence=0.99;
  cv::findHomography(a,b,cv::RANSAC,2.0,mask,kHomographyMaxIters,kHomographyConfidence);
  o.t_ransac_ms=(monoNs()-t_ransac0)*1e-6;
  if(mask.empty()){ o.invalid_reason=4; return o; }

  std::vector<cv::Point2f> ai,bi;
  for(size_t i=0;i<a.size();++i){if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(b[i]);}}
  o.inliers=(int)ai.size();
  o.inlier_ratio=a.empty()?0.0:(double)ai.size()/a.size();
  o.inlier_points=bi;
  o.metric_prev_points=ai;
  o.metric_curr_points=bi;
  if(ai.size()<20){ o.invalid_reason=5; return o; }

  const int64_t t_post0=monoNs();
  std::vector<cv::Point2f> au,bu;
  cv::undistortPoints(ai,au,calib.K,calib.D);
  cv::undistortPoints(bi,bu,calib.K,calib.D);

  // BF 3-axis gyro derotation: normalized C0 ray projected into C1.
  // Null pointer preserves the original ArduPilot algorithm.
  std::vector<cv::Point2f> au_rot = au;
  if (C1_R_C0 != nullptr) {
    for (size_t i=0; i<au.size(); ++i) {
      const cv::Vec3d q =
          (*C1_R_C0) * cv::Vec3d(au[i].x, au[i].y, 1.0);
      if (std::isfinite(q[2]) && q[2] > 0.1) {
        au_rot[i] = cv::Point2f(
            static_cast<float>(q[0]/q[2]),
            static_cast<float>(q[1]/q[2]));
      }
    }
  }


  std::vector<double> dun,dvn,dup,dvp;
  dun.reserve(ai.size()); dvn.reserve(ai.size()); dup.reserve(ai.size()); dvp.reserve(ai.size());

  std::array<std::vector<double>,9> cell_du,cell_dv;

  for(size_t i=0;i<ai.size();++i){
    const double du=(double)bu[i].x-au[i].x;
    const double dv=(double)bu[i].y-au[i].y;
    dun.push_back(du);
    dvn.push_back(dv);
    dup.push_back((double)bi[i].x-ai[i].x);
    dvp.push_back((double)bi[i].y-ai[i].y);

    const double nx=(ai[i].x/(double)prev.cols-g_feature_roi.x0)/
                    (g_feature_roi.x1-g_feature_roi.x0);
    const double ny=(ai[i].y/(double)prev.rows-g_feature_roi.y0)/
                    (g_feature_roi.y1-g_feature_roi.y0);
    const int cx=std::clamp((int)std::floor(nx*3.0),0,2);
    const int cy=std::clamp((int)std::floor(ny*3.0),0,2);
    const int ci=cy*3+cx;
    cell_du[ci].push_back(du);
    cell_dv[ci].push_back(dv);
  }
  // Separate horizontal translation from two motions that must NOT become
  // horizontal velocity:
  //   1) optical-axis rotation (yaw),
  //   2) isotropic image scaling caused by vertical motion (change of height).
  //
  // On undistorted normalized coordinates:
  //   du = tx + s*x - wz*y
  //   dv = ty + s*y + wz*x
  //
  // tx/ty are the constant image translation that ArduPilot needs. s is the
  // inter-frame scale change (mainly Z motion) and wz is camera-axis rotation.
  // The old 3-parameter fit omitted s, so a rapid climb/descent could leak the
  // radial scale field into tx/ty when features were not perfectly symmetric.
  cv::Mat A((int)ai.size()*2,4,CV_64F);
  cv::Mat bb((int)ai.size()*2,1,CV_64F);
  for(size_t k=0;k<ai.size();++k){
    const double x=(double)au_rot[k].x;
    const double y=(double)au_rot[k].y;
    const double du=(double)bu[k].x-au_rot[k].x;
    const double dv=(double)bu[k].y-au_rot[k].y;
    A.at<double>((int)(2*k),0)=1.0;   // tx
    A.at<double>((int)(2*k),1)=0.0;   // ty
    A.at<double>((int)(2*k),2)=x;     // scale
    A.at<double>((int)(2*k),3)=-y;    // yaw
    bb.at<double>((int)(2*k),0)=du;
    A.at<double>((int)(2*k+1),0)=0.0;
    A.at<double>((int)(2*k+1),1)=1.0;
    A.at<double>((int)(2*k+1),2)=y;
    A.at<double>((int)(2*k+1),3)=x;
    bb.at<double>((int)(2*k+1),0)=dv;
  }
  cv::Mat sol;
  const bool fit_ok=cv::solve(A,bb,sol,cv::DECOMP_SVD);
  if(fit_ok && sol.rows==4){
    o.du_norm=sol.at<double>(0,0);
    o.dv_norm=sol.at<double>(1,0);
    o.scale_rate=sol.at<double>(2,0)/dt;
    o.yaw_rate_cam_z=sol.at<double>(3,0)/dt;
  } else {
    o.du_norm=median(dun);
    o.dv_norm=median(dvn);
    o.scale_rate=0.0;
    o.yaw_rate_cam_z=0.0;
  }
  o.du_px=median(dup); o.dv_px=median(dvp);

  // OpenCV camera: +X image-right, +Y image-down, +Z optical-forward.
  // Positive RH camera rotation about Cx -> +dv, about Cy -> -du.
  o.flow_cam_x=o.dv_norm/dt;
  o.flow_cam_y=-o.du_norm/dt;

  // camera -> body FLU из T_BS, затем FLU -> ArduPilot body FRD.
  const cv::Matx33d FLU_TO_FRD(1,0,0, 0,-1,0, 0,0,-1);
  const cv::Matx33d FRD_R_C=FLU_TO_FRD*calib.B_R_C;
  const cv::Vec3d fb=FRD_R_C*cv::Vec3d(o.flow_cam_x,o.flow_cam_y,0.0);
  o.flow_body_x=fb[0]; o.flow_body_y=fb[1];

  for(int ci=0;ci<9;ci++){
    o.cell_n[ci]=(int)cell_du[ci].size();
    if(o.cell_n[ci]>=3){
      // Cell diagnostic uses the same globally fitted yaw removal.
      std::vector<double> cdu_clean,cdv_clean;
      cdu_clean.reserve(cell_du[ci].size());
      cdv_clean.reserve(cell_dv[ci].size());
      for(size_t k=0;k<ai.size();++k){
        const double nx=(ai[k].x/(double)prev.cols-g_feature_roi.x0)/
                        (g_feature_roi.x1-g_feature_roi.x0);
        const double ny=(ai[k].y/(double)prev.rows-g_feature_roi.y0)/
                        (g_feature_roi.y1-g_feature_roi.y0);
        const int cx=std::clamp((int)std::floor(nx*3.0),0,2);
        const int cy=std::clamp((int)std::floor(ny*3.0),0,2);
        if(cy*3+cx!=ci) continue;
        const double x=(double)au[k].x, y=(double)au[k].y;
        const double wzdt=o.yaw_rate_cam_z*dt;
        const double sdt=o.scale_rate*dt;
        cdu_clean.push_back(((double)bu[k].x-au[k].x) - sdt*x + wzdt*y);
        cdv_clean.push_back(((double)bu[k].y-au[k].y) - sdt*y - wzdt*x);
      }
      const double cdu=cdu_clean.empty()?median(cell_du[ci]):median(cdu_clean);
      const double cdv=cdv_clean.empty()?median(cell_dv[ci]):median(cdv_clean);
      const double cfx=cdv/dt;
      const double cfy=-cdu/dt;
      const cv::Vec3d cfb=FRD_R_C*cv::Vec3d(cfx,cfy,0.0);
      o.cell_body_x[ci]=cfb[0];
      o.cell_body_y[ci]=cfb[1];
    }
  }

  const double mag=std::hypot(o.flow_body_x,o.flow_body_y);
  o.valid=std::isfinite(mag) && mag<4.0;
  o.invalid_reason=o.valid?0:6;
  o.t_post_ms=(monoNs()-t_post0)*1e-6;
  return o;
}
