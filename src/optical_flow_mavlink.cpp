// OPENCV_PARALLEL_PROBE_V1 headers
#include <mutex>
#include <set>
#include <sys/syscall.h>
#include <unistd.h>
#include <time.h>
// monkeysStab — standalone OpticalFlow MAVLink publisher (migrated from JT-Zero).
//
// Production chain: OV9281 -> optical-flow rate -> MAVLink OPTICAL_FLOW -> ArduPilot EKF3.
// TF-Luna is published separately as DISTANCE_SENSOR.
//
// ВАЖНО: сначала подключаем ardupilotmega dialect. Включаемый ниже legacy/base
// файл сам включает common/mavlink.h; после этого include guard уже не даст
// переопределить dialect, поэтому порядок здесь принципиален.
#include "ardupilotmega/mavlink.h"

#include "runtime.hpp"
#include "mavlink_io.hpp"
#include "metric_odometry_shadow.hpp"
#include "metric_shadow_sync.hpp"
#include "metric_shadow_range_sync.hpp"
#include "worked5_estimator.hpp"
#include "pi4_ov5647_capture.hpp"
#include "variant_b_angular_shadow.hpp"
#include "imu_dead_reckoning.hpp"

#include <deque>
#include <condition_variable>
#include <thread>
#include <filesystem>
#include <sstream>
#include <atomic>
#include <array>
#include <map>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#if __has_include(<opencv2/freetype.hpp>)
#include <opencv2/freetype.hpp>
#define JTZERO_GUI_FREETYPE 1
#else
#define JTZERO_GUI_FREETYPE 0
#endif

namespace {

#if JTZERO_GUI_FREETYPE
cv::Ptr<cv::freetype::FreeType2> g_gui_font;
#endif

class AsyncCsvWriter {
 public:
  explicit AsyncCsvWriter(std::ofstream& out):out_(out),worker_([this]{run();}){}
  ~AsyncCsvWriter(){ stop(); }
  bool tryEnqueue(std::string row){
    std::unique_lock<std::mutex> lk(mu_,std::try_to_lock);
    if(!lk.owns_lock() || stop_ || q_.size()>=kMaxRows){
      dropped_.fetch_add(1,std::memory_order_relaxed);
      return false;
    }
    q_.push_back(std::move(row));
    lk.unlock();
    cv_.notify_one();
    return true;
  }
  void requestFlush(){ flush_.store(true,std::memory_order_release); cv_.notify_one(); }
  void stop(){
    {
      std::lock_guard<std::mutex> lk(mu_);
      if(stop_) return;
      stop_=true;
    }
    cv_.notify_one();
    if(worker_.joinable()) worker_.join();
  }
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
  uint64_t bytes() const { return bytes_.load(std::memory_order_relaxed); }
 private:
  void run(){
    for(;;){
      std::string row;
      {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait_for(lk,std::chrono::milliseconds(50),[this]{
          return stop_ || !q_.empty() || flush_.load(std::memory_order_acquire);
        });
        if(!q_.empty()){ row=std::move(q_.front()); q_.pop_front(); }
        else if(stop_) break;
      }
      if(!row.empty()){
        out_.write(row.data(),static_cast<std::streamsize>(row.size()));
        bytes_.fetch_add(row.size(),std::memory_order_relaxed);
      }
      if(flush_.exchange(false,std::memory_order_acq_rel)) out_.flush();
    }
    for(;;){
      std::string row;
      {
        std::lock_guard<std::mutex> lk(mu_);
        if(q_.empty()) break;
        row=std::move(q_.front()); q_.pop_front();
      }
      out_.write(row.data(),static_cast<std::streamsize>(row.size()));
      bytes_.fetch_add(row.size(),std::memory_order_relaxed);
    }
    out_.flush();
  }
  static constexpr size_t kMaxRows=512;
  std::ofstream& out_;
  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::deque<std::string> q_;
  bool stop_=false;
  std::atomic<bool> flush_{false};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<uint64_t> bytes_{0};
  std::thread worker_;
};

bool initGuiFont(){
#if JTZERO_GUI_FREETYPE
  const std::array<const char*,6> candidates{{
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf",
    "/usr/share/fonts/opentype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/dejavu/DejaVuSans.ttf"
  }};
  for(const char* p:candidates){
    std::ifstream fh(p,std::ios::binary);
    if(!fh.good()) continue;
    try{
      g_gui_font=cv::freetype::createFreeType2();
      g_gui_font->loadFontData(p,0);
      std::cerr<<"GUI: русский UTF-8 шрифт: "<<p<<"\n";
      return true;
    }catch(const cv::Exception&){}
  }
#endif
  std::cerr<<"ПРЕДУПРЕЖДЕНИЕ: UTF-8 шрифт GUI не найден; кириллица может отображаться некорректно.\n";
  return false;
}

void putGuiText(cv::Mat& img,const std::string& text,cv::Point org,
                double scale,cv::Scalar color,int thickness=1){
#if JTZERO_GUI_FREETYPE
  if(g_gui_font){
    const int h=std::max(12,(int)std::lround(31.0*scale));
    // FreeType: thickness > 0 draws only the glyph contour. That produced the
    // hollow/outlined text seen in the GUI. Use filled anti-aliased glyphs.
    g_gui_font->putText(img,text,org,h,color,-1,cv::LINE_AA,true);
    return;
  }
#endif
  cv::putText(img,text,org,cv::FONT_HERSHEY_SIMPLEX,scale,color,thickness,cv::LINE_AA);
}

static std::string jsonNumber(double v){
  if(!std::isfinite(v)) return "null";
  std::ostringstream o;
  o<<std::setprecision(10)<<v;
  return o.str();
}

// FORENSIC_SHADOW_FLUSH_AB_V1
// Default preserves the frozen baseline. Set MONKEYS_SHADOW_FLUSH=0 only for
// the controlled A/B test: shadow CSV rows are still written, but per-frame
// forced flushes are suppressed. Production optical_flow_mavlink.csv and the
// RAW causal35 publication path are not changed by this switch.
bool shadowFlushEnabled(){
  static const bool enabled=[]{
    const char* e=std::getenv("MONKEYS_SHADOW_FLUSH");
    if(!e || !*e) return true;
    const std::string v(e);
    return !(v=="0" || v=="false" || v=="FALSE" || v=="off" || v=="OFF");
  }();
  return enabled;
}

bool csvLiveFlushEnabled(){
  static const bool enabled=[]{
    const char* e=std::getenv("MONKEYS_CSV_LIVE_FLUSH");
    if(!e || !*e) return true;
    const std::string v(e);
    return !(v=="0" || v=="false" || v=="FALSE" || v=="off" || v=="OFF");
  }();
  return enabled;
}

bool hotPathDiagnosticIoEnabled(){
  static const bool enabled=[]{
    const char* e=std::getenv("MONKEYS_HOTPATH_DIAG_IO");
    if(!e || !*e) return true;
    const std::string v(e);
    return !(v=="0" || v=="false" || v=="FALSE" || v=="off" || v=="OFF");
  }();
  return enabled;
}

bool diagnosticFileIoEnabled(){
  static const bool enabled=[]{
    const char* e=std::getenv("MONKEYS_DIAG_FILE_IO");
    if(!e || !*e) return true;
    const std::string v(e);
    return !(v=="0" || v=="false" || v=="FALSE" || v=="off" || v=="OFF");
  }();
  return enabled;
}

struct FeatureRoi {
  double x0=0.20;
  double y0=0.20;
  double x1=0.80;
  double y1=0.80;
};

struct LiveWebTelemetryUdp {
  int fd=-1;
  sockaddr_in dst{};
  int64_t last_send_ns=0;
  int64_t period_ns=50000000LL; // 20 Hz max
  int64_t last_preview_ns=0;
  int64_t preview_period_ns=166666667LL; // <=6 Hz diagnostic web preview
  std::string preview_path;

  LiveWebTelemetryUdp(){
    if(const char* p=std::getenv("MONKEYS_WEB_PREVIEW_PATH"); p && *p){
      preview_path=p;
      std::cerr<<"WEB CAMERA PREVIEW: "<<preview_path<<" @ <=6 Hz\n";
    }
    const char* e=std::getenv("MONKEYS_WEB_TELEMETRY_UDP_PORT");
    if(!e || !*e) return;
    const int port=std::atoi(e);
    if(port<=0 || port>65535) return;
    fd=::socket(AF_INET,SOCK_DGRAM,0);
    if(fd<0) return;
    std::memset(&dst,0,sizeof(dst));
    dst.sin_family=AF_INET;
    dst.sin_port=htons(static_cast<uint16_t>(port));
    dst.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    std::cerr<<"WEB LIVE TELEMETRY: udp://127.0.0.1:"<<port<<" @ <=20 Hz\n";
  }
  ~LiveWebTelemetryUdp(){ if(fd>=0) ::close(fd); }

  void send(int64_t now,const std::string& json){
    if(fd<0) return;
    if(last_send_ns && now-last_send_ns<period_ns) return;
    (void)::sendto(fd,json.data(),json.size(),MSG_DONTWAIT,
                   reinterpret_cast<const sockaddr*>(&dst),sizeof(dst));
    last_send_ns=now;
  }

  void sendPreview(int64_t now,const cv::Mat& gray,
                   const std::vector<cv::Point2f>& inliers,
                   const FeatureRoi& roi){
    if((fd<0 && preview_path.empty()) || gray.empty()) return;
    if(last_preview_ns && now-last_preview_ns<preview_period_ns) return;
    constexpr int out_w=320;
    const int out_h=std::max(1,(int)std::lround((double)gray.rows*out_w/std::max(1,gray.cols)));
    cv::Mat small,bgr;
    cv::resize(gray,small,cv::Size(out_w,out_h),0,0,cv::INTER_AREA);
    cv::cvtColor(small,bgr,cv::COLOR_GRAY2BGR);
    const double sx=(double)out_w/std::max(1,gray.cols);
    const double sy=(double)out_h/std::max(1,gray.rows);
    for(const auto& p:inliers){
      cv::circle(bgr,cv::Point((int)std::lround(p.x*sx),(int)std::lround(p.y*sy)),
                 2,cv::Scalar(0,255,0),-1,cv::LINE_AA);
    }
    cv::rectangle(bgr,
      cv::Point((int)std::lround(roi.x0*out_w),(int)std::lround(roi.y0*out_h)),
      cv::Point((int)std::lround(roi.x1*out_w),(int)std::lround(roi.y1*out_h)),
      cv::Scalar(0,220,255),1,cv::LINE_AA);
    std::vector<uchar> jpg;
    const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY,65};
    if(!cv::imencode(".jpg",bgr,jpg,params)) return;

    // Primary preview transport: an atomic RAM-file snapshot.  This avoids
    // UDP datagram size/bind failures while keeping the existing telemetry
    // socket completely independent. /dev/shm is supplied by web_service.py.
    if(!preview_path.empty()){
      const std::string tmp=preview_path+".tmp";
      {
        std::ofstream out(tmp,std::ios::binary|std::ios::trunc);
        if(out.good()) out.write(reinterpret_cast<const char*>(jpg.data()),
                                static_cast<std::streamsize>(jpg.size()));
      }
      (void)::rename(tmp.c_str(),preview_path.c_str());
    }

    // Keep UDP preview as a best-effort compatibility path when it fits in one
    // datagram. The web UI no longer depends on this path.
    if(fd>=0 && jpg.size()<=60000){
      std::vector<uint8_t> packet;
      packet.reserve(jpg.size()+4);
      packet.insert(packet.end(),{'M','J','P','G'});
      packet.insert(packet.end(),jpg.begin(),jpg.end());
      (void)::sendto(fd,packet.data(),packet.size(),MSG_DONTWAIT,
                     reinterpret_cast<const sockaddr*>(&dst),sizeof(dst));
    }
    last_preview_ns=now;
  }
};

struct FlowFcLocal {
  float x=0,y=0,z=0,vx=0,vy=0,vz=0;
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowEkfStatus {
  uint16_t flags=0;
  float velocity_variance=0;
  float pos_horiz_variance=0;
  float pos_vert_variance=0;
  float compass_variance=0;
  float terrain_alt_variance=0;
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowFcGyro {
  double roll=0,pitch=0,yaw=0; // ATTITUDE angles, rad
  double x=0,y=0,z=0;          // body FRD roll/pitch/yaw rates, rad/s
  int64_t recv_ns=0;
  uint32_t time_boot_ms=0;
  int64_t sample_ns=0; // legacy ATTITUDE history key: RPi receive time
  int64_t mapped_sample_ns=0; // FC time_boot_ms mapped by HIGHRES clock model
  bool valid=false;
};

struct FlowFcImu {
  double ax=0,ay=0,az=0;       // HIGHRES_IMU body acceleration, m/s^2
  double gx=0,gy=0,gz=0;       // HIGHRES_IMU body gyro, rad/s
  uint64_t time_usec=0;         // FC-provided HIGHRES_IMU measurement timestamp
  int64_t recv_ns=0;            // RPi CLOCK_MONOTONIC receive timestamp
  bool valid=false;
};

struct FlowFcRawGyro {
  double x=0,y=0,z=0;           // HIGHRES_IMU gyro, body FRD rad/s
  double drift_x=0,drift_y=0,drift_z=0; // AHRS omegaI sampled at receive time
  uint64_t fc_time_usec=0;       // FC measurement timestamp from MAVLink
  int64_t recv_ns=0;             // RPi receive timestamp
  bool drift_valid=false;
  bool valid=false;
};

struct FlowFcTarget {
  float x=0,y=0,vx=0,vy=0;
  uint16_t type_mask=0;
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowFcAttTarget {
  double roll=0,pitch=0,yaw=0;
  float thrust=0;
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowFcOutputs {
  std::array<uint16_t,8> pwm{};
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowFcRc {
  std::array<uint16_t,18> pwm{};
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowFc {
  int fd=-1;
  std::thread th;
  std::mutex mu;
  FlowFcLocal local{};
  FlowEkfStatus ekf{};
  double baro_pressure_hpa=0.0;
  int64_t baro_recv_ns=0;
  FlowFcGyro gyro{};
  FlowFcImu imu{};
  imu_dr::State imu_dr_state{};

  // Camera velocity-constraint shadow.
  // Diagnostic only: never modifies the real imu_dr_state.
  imu_dr::State imu_camvc_state{};
  int imu_camvc_stop_samples=0;
  bool imu_camvc_active=false;
  uint64_t imu_camvc_activations=0;

  // Shadow-only camera gate for IMU ZUPT diagnostics.
  double imu_cam_vn=0.0,imu_cam_ve=0.0;
  int64_t imu_cam_recv_ns=0;
  bool imu_cam_valid=false;

  // FUSED-V1 event-driven shadow.
  uint64_t imu_cam_seq=0;
  double imu_cam_dN=0.0,imu_cam_dE=0.0,imu_cam_dt=0.0;

  // FUSED_V2_CAPTURE_V1
  // Diagnostic-only time-aligned IMU history for a future V2 bridge.
  struct FusedV2ImuSample {
    int64_t recv_ns=0;
    double pos_n=0.0,pos_e=0.0;
    double vel_n=0.0,vel_e=0.0;
  };
  std::deque<FusedV2ImuSample> fused_v2_imu_history;

  uint64_t fused_v1_seen_cam_seq=0;
  uint64_t fused_v1_visual_updates=0;
  uint64_t fused_v1_imu_predictions=0;
  uint64_t fused_v1_stop_constraints=0;
  double fused_v1_n=0.0,fused_v1_e=0.0;
  double fused_v1_vn=0.0,fused_v1_ve=0.0;
  double fused_v1_vn_hist[5]{};
  double fused_v1_ve_hist[5]{};
  int fused_v1_vhist_count=0;
  int fused_v1_vhist_head=0;
  bool fused_v1_stationary=false;
  int fused_v1_stop_confirm=0;
  bool imu_zupt_cam_fresh=false;
  bool imu_zupt_cam_stationary=false;
  bool imu_zupt_shadow=false;
  uint64_t imu_zupt_shadow_accepts=0;
  uint64_t imu_zupt_shadow_blocks=0;
  FlowFcTarget target{};
  FlowFcAttTarget att_target{};
  FlowFcOutputs outputs{};
  FlowFcRc rc{};
  std::deque<FlowFcGyro> attitude_history; // ATTITUDE, currently keyed by RPi receive time
  std::deque<FlowFcRawGyro> highres_gyro_history; // independent HIGHRES_IMU gyro stream

  // FLOWFC_RX_TIMING_DIAG_V1: shadow-only timing inside the TCP/direct-UART
  // consumer.  No filesystem I/O and no estimator/publisher dependency.
  int64_t flowfc_highres_last_read_ns=0;
  int64_t flowfc_diag_last_report_ns=0;
  double flowfc_highres_max_read_gap_ms=0.0;
  double flowfc_highres_max_parse_lag_ms=0.0;
  uint64_t flowfc_highres_read_gap_gt25=0;
  // FLOWFC_HANDLER_TIMING_DIAG_V1: distinguish time spent inside HIGHRES
  // handling from time spent elsewhere before the next kernel read.
  int64_t flowfc_prev_read_done_ns=0;
  double flowfc_highres_max_handler_ms=0.0;
  double flowfc_max_between_reads_ms=0.0;
  uint64_t flowfc_highres_handler_gt5=0;
  uint64_t flowfc_between_reads_gt25=0;
  // HIGHRES_CLOCK_MAP_V2: affine FC->RPi clock map fitted to one-second
  // lower-envelope receive offsets.  FC and RPi clocks measurably run at
  // different rates, so a constant offset is not sufficient.
  bool highres_clock_valid=false;
  int64_t highres_clock_fc0_ns=0;
  int64_t highres_clock_bin=-1;
  int64_t highres_clock_bin_min_offset_ns=0;
  uint64_t highres_clock_fit_n=0;
  long double highres_clock_sum_t=0.0L;
  long double highres_clock_sum_o=0.0L;
  long double highres_clock_sum_tt=0.0L;
  long double highres_clock_sum_to=0.0L;
  double highres_clock_offset0_ns=0.0;
  double highres_clock_drift_ns_per_s=0.0;

  void updateHighresClockMap(int64_t fc_ns,int64_t recv_ns){
    const int64_t off=recv_ns-fc_ns;
    if(!highres_clock_valid){
      highres_clock_valid=true;
      highres_clock_fc0_ns=fc_ns;
      highres_clock_bin=0;
      highres_clock_bin_min_offset_ns=off;
      highres_clock_offset0_ns=static_cast<double>(off);
      return;
    }
    const double t_s=(fc_ns-highres_clock_fc0_ns)*1e-9;
    const int64_t bin=static_cast<int64_t>(std::floor(std::max(0.0,t_s)));
    if(bin==highres_clock_bin){
      highres_clock_bin_min_offset_ns=std::min(highres_clock_bin_min_offset_ns,off);
      return;
    }
    if(bin>highres_clock_bin){
      const long double tb=static_cast<long double>(highres_clock_bin)+0.5L;
      const long double ob=static_cast<long double>(highres_clock_bin_min_offset_ns);
      ++highres_clock_fit_n;
      highres_clock_sum_t+=tb;
      highres_clock_sum_o+=ob;
      highres_clock_sum_tt+=tb*tb;
      highres_clock_sum_to+=tb*ob;
      if(highres_clock_fit_n>=3){
        const long double n=static_cast<long double>(highres_clock_fit_n);
        const long double den=n*highres_clock_sum_tt-highres_clock_sum_t*highres_clock_sum_t;
        if(std::abs(den)>1e-9L){
          const long double m=(n*highres_clock_sum_to-highres_clock_sum_t*highres_clock_sum_o)/den;
          const long double c=(highres_clock_sum_o-m*highres_clock_sum_t)/n;
          highres_clock_drift_ns_per_s=static_cast<double>(m);
          highres_clock_offset0_ns=static_cast<double>(c);
        }
      } else {
        highres_clock_offset0_ns=std::min(
          highres_clock_offset0_ns,static_cast<double>(highres_clock_bin_min_offset_ns));
      }
      highres_clock_bin=bin;
      highres_clock_bin_min_offset_ns=off;
    }
  }

  int64_t mapHighresFcToMono(int64_t fc_ns) const {
    if(!highres_clock_valid) return fc_ns;
    const double t_s=(fc_ns-highres_clock_fc0_ns)*1e-9;
    const double off=highres_clock_offset0_ns+highres_clock_drift_ns_per_s*t_s;
    return fc_ns+static_cast<int64_t>(std::llround(off));
  }
  // STARTUP_CLOCK_RESYNC_V1:
  // Reset only the affine FC->RPi clock-fit state. HIGHRES history,
  // WORKED5, optical-flow state and FC state are intentionally untouched.
  void resetHighresClockMap(){
    highres_clock_valid=false;
    highres_clock_fc0_ns=0;
    highres_clock_bin=-1;
    highres_clock_bin_min_offset_ns=0;
    highres_clock_fit_n=0;
    highres_clock_sum_t=0.0L;
    highres_clock_sum_o=0.0L;
    highres_clock_sum_tt=0.0L;
    highres_clock_sum_to=0.0L;
    highres_clock_offset0_ns=0.0;
    highres_clock_drift_ns_per_s=0.0;
  }

  std::ofstream highres_gyro_shadow_ofs;
  // ATTITUDE_CAUSAL_LOG_V1: raw FC ATTITUDE measurement/receive timing.
  // Shadow-only. Used to validate causal absolute-orientation anchoring.
  std::ofstream attitude_shadow_ofs;
  uint64_t attitude_shadow_seq=0;
  uint64_t local_count=0;
  uint64_t ekf_count=0;
  uint64_t gyro_count=0;
  uint64_t imu_count=0;
  double gyro_sum_x=0,gyro_sum_y=0,gyro_sum_z=0;
  uint64_t gyro_sum_count=0;
  double ahrs_omega_i_x=0.0,ahrs_omega_i_y=0.0,ahrs_omega_i_z=0.0;
  int64_t ahrs_omega_i_recv_ns=0;
  bool ahrs_omega_i_valid=false;
  bool armed=false;
  bool heartbeat_valid=false;
  int64_t heartbeat_recv_ns=0;
  uint32_t heartbeat_custom_mode=0;
  uint8_t heartbeat_base_mode=0;
  uint8_t heartbeat_system_status=0;
  uint8_t target_sys=0,target_comp=0;

  static constexpr size_t remote_block_size=MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN;
  std::ofstream remote_ofs;
  std::map<uint32_t,std::array<uint8_t,remote_block_size>> remote_pending;
  uint32_t remote_expected=0;
  uint64_t remote_blocks_rx=0,remote_blocks_written=0,remote_duplicates=0;
  bool remote_active=false;
  std::string remote_path;

  static constexpr uint8_t self_sys=191;
  static constexpr uint8_t self_comp=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

  // FUSED_V2_FRAME_CAPTURE_V1
  // Snapshot latest causal buffered IMU DR state for every camera frame, including
  // invalid reason5/reason6 frames. Diagnostic only.
  void writeFusedV2FrameCapture(const std::string& csvpath,
                                uint64_t frame,
                                int64_t cam_ns,
                                bool production_valid,
                                int invalid_reason,
                                int tracked,
                                int inliers,
                                double inlier_ratio,
                                double dt_s){
    if(!diagnosticFileIoEnabled()) return;
    std::unique_lock<std::mutex> l(mu);
    if(fused_v2_imu_history.empty()) return;
    // FUSED_V2_CAUSAL_CAPTURE_V1
    // Realtime-causal lookup: never use an IMU sample newer than this camera frame.
    auto best=fused_v2_imu_history.end();
    for(auto it=fused_v2_imu_history.begin();it!=fused_v2_imu_history.end();++it){
      if(it->recv_ns<=cam_ns && (best==fused_v2_imu_history.end() ||
                                it->recv_ns>best->recv_ns)){
        best=it;
      }
    }
    if(best==fused_v2_imu_history.end()) return;
    const int64_t log_imu_recv_ns=best->recv_ns;
    const double log_pos_n=best->pos_n,log_pos_e=best->pos_e;
    const double log_vel_n=best->vel_n,log_vel_e=best->vel_e;
    l.unlock();
    static std::ofstream out;
    static bool header=false;
    if(!out.is_open()){
      const std::filesystem::path production_csv_path(csvpath);
      out.open(production_csv_path.parent_path()/"fused_v2_frame_capture.csv",
               std::ios::out|std::ios::trunc);
    }
    if(!out.is_open()) return;
    if(!header){
      out<<"frame,cam_ns,imu_recv_ns,age_ms,production_valid,invalid_reason,"
           "tracked,inliers,inlier_ratio,dt_s,imu_n_m,imu_e_m,imu_vn,imu_ve\n";
      header=true;
    }
    out<<frame<<','<<cam_ns<<','<<log_imu_recv_ns<<','
       <<(cam_ns-log_imu_recv_ns)*1e-6<<','
       <<(production_valid?1:0)<<','<<invalid_reason<<','
       <<tracked<<','<<inliers<<','<<inlier_ratio<<','<<dt_s<<','
       <<log_pos_n<<','<<log_pos_e<<','<<log_vel_n<<','<<log_vel_e<<'\n';
    if(shadowFlushEnabled()) out.flush();
  }

  // FUSED_V2_REALTIME_SHADOW_V1
  // Frozen policy, identical to tools/analyze_fused_v2_full_shadow.py:
  // BAD      = eligible && ratio < 0.50 && inliers < 100
  // RECOVER  = two consecutive eligible frames with ratio > 0.70 && inliers >= 100
  // pre-roll = 150 ms
  // IMU      = latest causal sample (recv_ns <= camera monotonic timestamp)
  //
  // Diagnostic shadow only. It never changes WORKED5, FUSED-V1 or MAVLink output.
  struct FusedV2RtSnap {
    uint64_t frame=0;
    int64_t cam_ns=0;
    double shadow_n=0.0,shadow_e=0.0;
    double imu_n=0.0,imu_e=0.0;
  };
  std::deque<FusedV2RtSnap> fused_v2_rt_history;
  double fused_v2_rt_n=0.0,fused_v2_rt_e=0.0;
  uint64_t fused_v2_rt_last_cam_seq=0;
  bool fused_v2_rt_bridge=false;
  int fused_v2_rt_good_streak=0;
  uint64_t fused_v2_rt_events=0;
  uint64_t fused_v2_rt_anchor_frame=0,fused_v2_rt_bad_frame=0;
  double fused_v2_rt_base_n=0.0,fused_v2_rt_base_e=0.0;
  double fused_v2_rt_base_imu_n=0.0,fused_v2_rt_base_imu_e=0.0;

  void updateFusedV2RealtimeShadow(const std::string& csvpath,
                                   uint64_t frame,
                                   int64_t cam_ns,
                                   bool production_valid,
                                   int invalid_reason,
                                   int tracked,
                                   int inliers,
                                   double inlier_ratio,
                                   double dt_s){
    if(!diagnosticFileIoEnabled()) return;
    std::unique_lock<std::mutex> l(mu);
    if(fused_v2_imu_history.empty()) return;

    // Strictly causal IMU lookup: never use a sample from the future.
    auto imu_it=fused_v2_imu_history.end();
    for(auto it=fused_v2_imu_history.begin();it!=fused_v2_imu_history.end();++it){
      if(it->recv_ns<=cam_ns &&
         (imu_it==fused_v2_imu_history.end() || it->recv_ns>imu_it->recv_ns))
        imu_it=it;
    }
    if(imu_it==fused_v2_imu_history.end()) return;

    // Consume every unique WORKED5 event exactly once. While bridging, the event
    // is deliberately consumed but not added: offline shadow removes the same
    // WORKED5 increments from (anchor,recovery].
    bool new_w5=false;
    double w5_dn=0.0,w5_de=0.0;
    if(imu_cam_seq!=fused_v2_rt_last_cam_seq){
      fused_v2_rt_last_cam_seq=imu_cam_seq;
      if(imu_cam_valid){
        new_w5=true;
        w5_dn=imu_cam_dN;
        w5_de=imu_cam_dE;
      }
    }

    const bool eligible=(dt_s>0.0 && tracked>=20);
    const bool is_bad=(eligible && inlier_ratio<0.50 && inliers<100);
    const bool is_good=(eligible && inlier_ratio>0.70 && inliers>=100);

    // Healthy mode follows frozen WORKED5 exactly.
    if(!fused_v2_rt_bridge && new_w5){
      fused_v2_rt_n+=w5_dn;
      fused_v2_rt_e+=w5_de;
    }

    if(!fused_v2_rt_bridge && is_bad){
      const int64_t target=cam_ns-150000000LL;
      auto a=fused_v2_rt_history.end();
      for(auto it=fused_v2_rt_history.begin();it!=fused_v2_rt_history.end();++it)
        if(it->cam_ns<=target) a=it;

      if(a!=fused_v2_rt_history.end()){
        fused_v2_rt_bridge=true;
        fused_v2_rt_good_streak=0;
        ++fused_v2_rt_events;
        fused_v2_rt_anchor_frame=a->frame;
        fused_v2_rt_bad_frame=frame;
        fused_v2_rt_base_n=a->shadow_n;
        fused_v2_rt_base_e=a->shadow_e;
        fused_v2_rt_base_imu_n=a->imu_n;
        fused_v2_rt_base_imu_e=a->imu_e;
        // Rewind the already accumulated visual trajectory to the frozen
        // 150-ms anchor, then replace it with causal IMU displacement.
        fused_v2_rt_n=fused_v2_rt_base_n+(imu_it->pos_n-fused_v2_rt_base_imu_n);
        fused_v2_rt_e=fused_v2_rt_base_e+(imu_it->pos_e-fused_v2_rt_base_imu_e);
      }
    }else if(fused_v2_rt_bridge){
      fused_v2_rt_n=fused_v2_rt_base_n+(imu_it->pos_n-fused_v2_rt_base_imu_n);
      fused_v2_rt_e=fused_v2_rt_base_e+(imu_it->pos_e-fused_v2_rt_base_imu_e);
      if(is_good) ++fused_v2_rt_good_streak;
      else fused_v2_rt_good_streak=0;
      if(fused_v2_rt_good_streak>=2){
        // Recovery confirmation frame is still covered by IMU. Future WORKED5
        // events resume from the next processed frame.
        fused_v2_rt_bridge=false;
        fused_v2_rt_good_streak=0;
      }
    }

    fused_v2_rt_history.push_back({
      frame,cam_ns,fused_v2_rt_n,fused_v2_rt_e,imu_it->pos_n,imu_it->pos_e});
    while(!fused_v2_rt_history.empty() &&
          cam_ns-fused_v2_rt_history.front().cam_ns>500000000LL)
      fused_v2_rt_history.pop_front();

    const int64_t log_imu_recv_ns=imu_it->recv_ns;
    const bool log_bridge=fused_v2_rt_bridge;
    const uint64_t log_events=fused_v2_rt_events;
    const uint64_t log_anchor=fused_v2_rt_anchor_frame,log_bad=fused_v2_rt_bad_frame;
    const double log_n=fused_v2_rt_n,log_e=fused_v2_rt_e;
    l.unlock();

    static std::ofstream out;
    static bool header=false;
    if(!out.is_open()){
      const std::filesystem::path production_csv_path(csvpath);
      out.open(production_csv_path.parent_path()/"fused_v2_realtime_shadow.csv",
               std::ios::out|std::ios::trunc);
    }
    if(!out.is_open()) return;
    if(!header){
      out<<"frame,cam_ns,production_valid,invalid_reason,tracked,inliers,inlier_ratio,dt_s,"
           "new_w5,w5_dN_m,w5_dE_m,bridge,event_count,anchor_frame,bad_frame,"
           "imu_age_ms,shadow_n_m,shadow_e_m,shadow_endpoint_m\n";
      header=true;
    }
    out<<frame<<','<<cam_ns<<','<<(production_valid?1:0)<<','<<invalid_reason<<','
       <<tracked<<','<<inliers<<','<<inlier_ratio<<','<<dt_s<<','
       <<(new_w5?1:0)<<','<<w5_dn<<','<<w5_de<<','
       <<(log_bridge?1:0)<<','<<log_events<<','
       <<log_anchor<<','<<log_bad<<','
       <<(cam_ns-log_imu_recv_ns)*1e-6<<','
       <<log_n<<','<<log_e<<','
       <<std::hypot(log_n,log_e)<<'\n';
    if(shadowFlushEnabled()) out.flush();
  }

  ~FlowFc(){ stop(); }

  static void writeAll(int fd,const uint8_t* p,size_t n){
    size_t o=0;
    while(o<n){
      const ssize_t k=::write(fd,p+o,n-o);
      if(k>0){o+=static_cast<size_t>(k);continue;}
      if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){
        pollfd q{fd,POLLOUT,0}; poll(&q,1,10); continue;
      }
      if(k<0&&errno==EINTR)continue;
      fail("FC write");
    }
  }

  static void sendRemoteStatus(int fd,uint8_t sys,uint8_t comp,uint32_t seq,uint8_t status){
    mavlink_message_t m{};
    mavlink_msg_remote_log_block_status_pack(self_sys,self_comp,&m,sys,comp,seq,status);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    const auto n=mavlink_msg_to_send_buffer(b,&m);
    writeAll(fd,b,n);
  }

  static void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(self_sys,self_comp,&m,sys,comp,
      MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    const auto n=mavlink_msg_to_send_buffer(b,&m);
    writeAll(fd,b,n);
  }

  static int openEndpoint(const std::string& dev){
    constexpr const char* kTcp="tcp://";
    if(dev.rfind(kTcp,0)==0){
      const std::string hp=dev.substr(std::strlen(kTcp));
      const auto colon=hp.rfind(':');
      if(colon==std::string::npos) throw std::runtime_error("FC TCP endpoint: ожидается tcp://host:port");
      const std::string host=hp.substr(0,colon);
      const std::string port=hp.substr(colon+1);
      addrinfo hints{},*res=nullptr;
      hints.ai_family=AF_UNSPEC; hints.ai_socktype=SOCK_STREAM;
      const int gr=getaddrinfo(host.c_str(),port.c_str(),&hints,&res);
      if(gr!=0) throw std::runtime_error(std::string("FC TCP getaddrinfo: ")+gai_strerror(gr));
      int s=-1;
      for(addrinfo* p=res;p;p=p->ai_next){
        s=::socket(p->ai_family,p->ai_socktype,p->ai_protocol);
        if(s<0) continue;
        if(::connect(s,p->ai_addr,p->ai_addrlen)==0) break;
        ::close(s); s=-1;
      }
      freeaddrinfo(res);
      if(s<0) throw std::runtime_error("FC TCP connect failed: "+dev);
      // Match the router's low-latency localhost stream in both directions.
      // This changes transport latency only; MAVLink payloads are untouched.
      const int one=1;
      if(setsockopt(s,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one))<0)
        std::cerr<<"FC TCP warning: TCP_NODELAY failed: "<<std::strerror(errno)<<"\n";
      const int fl=fcntl(s,F_GETFL,0);
      if(fl>=0) fcntl(s,F_SETFL,fl|O_NONBLOCK);
      std::cerr<<"FC endpoint: "<<dev<<" (через MAVLink router)\n";
      return s;
    }

    int s=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(s<0)fail("open FC");
    termios t{};
    if(tcgetattr(s,&t)<0)fail("FC tcgetattr");
    cfmakeraw(&t);
    cfsetispeed(&t,B460800); cfsetospeed(&t,B460800);
    t.c_cflag|=CLOCAL|CREAD;
    t.c_cflag&=~CRTSCTS; t.c_cflag&=~PARENB; t.c_cflag&=~CSTOPB;
    t.c_cflag&=~CSIZE; t.c_cflag|=CS8;
    if(tcsetattr(s,TCSANOW,&t)<0)fail("FC tcsetattr");
    tcflush(s,TCIFLUSH);
    std::cerr<<"FC endpoint: "<<dev<<" @ 460800 (direct UART)\n";
    return s;
  }

  void start(const std::string& dev){
    fd=openEndpoint(dev);

    th=std::thread([this]{
      mavlink_status_t st{}; mavlink_message_t m{}; uint8_t buf[4096];
      uint8_t sys=0,comp=0;
      const int64_t deadline=monoNs()+10000000000LL;

      while(g_running&&!sys&&monoNs()<deadline){
        pollfd p{fd,POLLIN,0};
        if(poll(&p,1,100)<=0)continue;
        const ssize_t n=read(fd,buf,sizeof(buf));
        if(n<=0)continue;
        for(ssize_t i=0;i<n;i++){
          if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
          if(m.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
          mavlink_heartbeat_t hb{}; mavlink_msg_heartbeat_decode(&m,&hb);
          if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){
            sys=m.sysid; comp=m.compid;
            {
              std::lock_guard<std::mutex> l(mu);
              armed=(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)!=0;
              heartbeat_custom_mode=hb.custom_mode;
              heartbeat_base_mode=hb.base_mode;
              heartbeat_system_status=hb.system_status;
              heartbeat_valid=true;
              heartbeat_recv_ns=monoNs();
            }
            break;
          }
        }
      }

      if(!sys){std::cerr<<"FC: ArduPilot HEARTBEAT timeout\n";g_running=false;return;}
      target_sys=sys; target_comp=comp;
      std::cerr<<"FC: ArduPilot heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";
      // Request HIGHRES first and ATTITUDE last.  Some ArduPilot telemetry
      // configurations clamp/override selected streams; repeat critical
      // requests after the rest of the subscriptions have been installed.
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_PRESSURE,10);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_LOCAL_POSITION_NED,20);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_EKF_STATUS_REPORT,5);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,100);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);
      // AHRS omegaI is ArduPilot's gyro drift correction.  Capture it only
      // for shadow diagnostics; production optical flow is unchanged.
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_AHRS,20);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,20);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE_TARGET,20);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,20);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_RC_CHANNELS,20);
      // Reassert the two streams required by the metric/RAW contract after
      // all auxiliary subscriptions.  This is intentionally one-shot: it
      // avoids a request storm while proving whether another subscription
      // overwrites the FC scheduler.
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,100);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_LOCAL_POSITION_NED,20);

      while(g_running){
        pollfd p{fd,POLLIN,0};
        if(poll(&p,1,50)<=0)continue;
        for(;;){
          const ssize_t n=read(fd,buf,sizeof(buf));
          if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
          if(n<=0)break;
          // Timestamp the socket/UART read once for every kernel read.  Every
          // MAVLink frame parsed from this buffer inherits the same boundary
          // timestamp, which also exposes batched delivery explicitly.
          const int64_t flowfc_read_ns=monoNs();
          if(flowfc_prev_read_done_ns>0){
            const double between_reads_ms=
              (flowfc_read_ns-flowfc_prev_read_done_ns)*1e-6;
            flowfc_max_between_reads_ms=
              std::max(flowfc_max_between_reads_ms,between_reads_ms);
            if(between_reads_ms>25.0) ++flowfc_between_reads_gt25;
          }
          for(ssize_t i=0;i<n;i++){
            if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
            if(m.sysid!=sys)continue;
            if(m.msgid==MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK){
              mavlink_remote_log_data_block_t q{}; mavlink_msg_remote_log_data_block_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              if(remote_active && q.target_system==self_sys && q.target_component==self_comp){
                ++remote_blocks_rx;
                if(q.seqno<remote_expected || remote_pending.count(q.seqno)){
                  ++remote_duplicates;
                } else {
                  std::array<uint8_t,remote_block_size> a{};
                  std::memcpy(a.data(),q.data,remote_block_size);
                  remote_pending.emplace(q.seqno,a);
                }
                sendRemoteStatus(fd,sys,comp,q.seqno,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
                for(;;){
                  auto it=remote_pending.find(remote_expected);
                  if(it==remote_pending.end())break;
                  remote_ofs.write(reinterpret_cast<const char*>(it->second.data()),remote_block_size);
                  remote_pending.erase(it);
                  ++remote_expected; ++remote_blocks_written;
                }
              }
            } else if(m.msgid==MAVLINK_MSG_ID_HEARTBEAT){
              mavlink_heartbeat_t hb{}; mavlink_msg_heartbeat_decode(&m,&hb);
              if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){
                std::lock_guard<std::mutex> l(mu);
                armed=(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)!=0;
                heartbeat_valid=true;
                heartbeat_recv_ns=monoNs();
              }
            } else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t q{}; mavlink_msg_attitude_decode(&m,&q);
              std::unique_lock<std::mutex> l(mu);
              gyro.roll=q.roll; gyro.pitch=q.pitch; gyro.yaw=q.yaw;
              gyro.x=q.rollspeed; gyro.y=q.pitchspeed; gyro.z=q.yawspeed;
              gyro.recv_ns=monoNs(); gyro.time_boot_ms=q.time_boot_ms;
              // Online ΔR uses one RPi monotonic clock for both camera dequeue
              // and MAVLink receive.  This deliberately avoids mixing FC boot
              // time with a live frame whose transport latency is not known.
              gyro.sample_ns=gyro.recv_ns;
              const int64_t attitude_fc_sample_ns=
                static_cast<int64_t>(q.time_boot_ms)*1000000LL;
              gyro.mapped_sample_ns=highres_clock_valid
                ? mapHighresFcToMono(attitude_fc_sample_ns) : 0;
              gyro.valid=true; ++gyro_count;

              // Diagnostic filesystem I/O must never run while fc.mu is held.
              const auto attitude_log_gyro=gyro;
              l.unlock();
              if(diagnosticFileIoEnabled() && !attitude_shadow_ofs.is_open()){
                attitude_shadow_ofs.open(
                  "/home/vio/Desktop/monkeysStab/attitude_shadow_latest.csv",
                  std::ios::out|std::ios::trunc);
                if(diagnosticFileIoEnabled() && attitude_shadow_ofs.is_open())
                  attitude_shadow_ofs
                    <<"seq,time_boot_ms,fc_sample_ns,recv_ns,mapped_sample_ns,"
                    <<"mapped_transport_ms,clock_map_valid,roll_rad,pitch_rad,yaw_rad,"
                    <<"rollspeed_rad_s,pitchspeed_rad_s,yawspeed_rad_s\n";
              }
              if(diagnosticFileIoEnabled() && attitude_shadow_ofs.is_open()){
                const int64_t fc_sample_ns=attitude_fc_sample_ns;
                const bool map_valid=attitude_log_gyro.mapped_sample_ns>0;
                const int64_t mapped_sample_ns=attitude_log_gyro.mapped_sample_ns;
                const double mapped_transport_ms=
                  map_valid?(attitude_log_gyro.recv_ns-mapped_sample_ns)*1e-6:-1.0;
                attitude_shadow_ofs
                  <<(++attitude_shadow_seq)<<','<<q.time_boot_ms<<','
                  <<fc_sample_ns<<','<<attitude_log_gyro.recv_ns<<','<<mapped_sample_ns<<','
                  <<mapped_transport_ms<<','<<(map_valid?1:0)<<','
                  <<q.roll<<','<<q.pitch<<','<<q.yaw<<','
                  <<q.rollspeed<<','<<q.pitchspeed<<','<<q.yawspeed<<'\n';
                // Do not flush every 100 Hz ATTITUDE sample. std::ofstream
                // buffers the diagnostic shadow and flushes on close; forcing a
                // flush here can stall the single MAVLink RX thread and age all
                // FC measurements seen by causal35.
              }
              l.lock();

              attitude_history.push_back(gyro);
              while(attitude_history.size()>2 &&
                    gyro.sample_ns-attitude_history.front().sample_ns>3000000000LL)
                attitude_history.pop_front();
              gyro_sum_x+=q.rollspeed; gyro_sum_y+=q.pitchspeed; gyro_sum_z+=q.yawspeed;
              ++gyro_sum_count;
            } else if(m.msgid==MAVLINK_MSG_ID_AHRS){
              mavlink_ahrs_t q{}; mavlink_msg_ahrs_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              ahrs_omega_i_x=q.omegaIx;
              ahrs_omega_i_y=q.omegaIy;
              ahrs_omega_i_z=q.omegaIz;
              ahrs_omega_i_recv_ns=monoNs();
              ahrs_omega_i_valid=true;
            } else if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              const int64_t flowfc_highres_handler_start_ns=monoNs();
              mavlink_highres_imu_t q{}; mavlink_msg_highres_imu_decode(&m,&q);

              // Measure where HIGHRES_IMU latency appears after the router:
              // read_gap = time between kernel reads carrying HIGHRES_IMU;
              // parse_lag = time spent after this read before this frame is
              // decoded/handled.  Diagnostic only.
              const int64_t flowfc_parse_ns=monoNs();
              const double flowfc_parse_lag_ms=
                (flowfc_parse_ns-flowfc_read_ns)*1e-6;
              double flowfc_read_gap_ms=-1.0;
              if(flowfc_highres_last_read_ns>0)
                flowfc_read_gap_ms=
                  (flowfc_read_ns-flowfc_highres_last_read_ns)*1e-6;
              flowfc_highres_last_read_ns=flowfc_read_ns;
              if(flowfc_read_gap_ms>=0.0)
                flowfc_highres_max_read_gap_ms=
                  std::max(flowfc_highres_max_read_gap_ms,flowfc_read_gap_ms);
              flowfc_highres_max_parse_lag_ms=
                std::max(flowfc_highres_max_parse_lag_ms,flowfc_parse_lag_ms);
              if(flowfc_read_gap_ms>25.0)
                ++flowfc_highres_read_gap_gt25;
              if(flowfc_diag_last_report_ns==0)
                flowfc_diag_last_report_ns=flowfc_parse_ns;
              if(flowfc_parse_ns-flowfc_diag_last_report_ns>=5000000000LL){
                std::cerr<<"FLOWFC_RX_RATE_DIAG"
                         <<" max_read_gap_ms="<<flowfc_highres_max_read_gap_ms
                         <<" max_parse_lag_ms="<<flowfc_highres_max_parse_lag_ms
                         <<" read_gap_gt25="<<flowfc_highres_read_gap_gt25
                         <<" max_handler_ms="<<flowfc_highres_max_handler_ms
                         <<" handler_gt5="<<flowfc_highres_handler_gt5
                         <<" max_between_reads_ms="<<flowfc_max_between_reads_ms
                         <<" between_reads_gt25="<<flowfc_between_reads_gt25
                         <<"\n";
                flowfc_diag_last_report_ns=flowfc_parse_ns;
              }

              std::unique_lock<std::mutex> l(mu);
              imu.ax=q.xacc; imu.ay=q.yacc; imu.az=q.zacc;
              imu.gx=q.xgyro; imu.gy=q.ygyro; imu.gz=q.zgyro;
              imu.time_usec=q.time_usec; imu.recv_ns=monoNs(); imu.valid=true; ++imu_count;

              // HIGHRES_GYRO_SHADOW_V1: independent raw gyro capture.
              // This does not use ATTITUDE.rollspeed/pitchspeed/yawspeed and
              // does not feed any production estimator or MAVLink output.
              const int64_t fc_sample_ns=static_cast<int64_t>(q.time_usec)*1000LL;
              updateHighresClockMap(fc_sample_ns,imu.recv_ns);
              const double drift_age_ms=ahrs_omega_i_valid
                  ? (imu.recv_ns-ahrs_omega_i_recv_ns)*1e-6 : -1.0;
              const bool drift_fresh=ahrs_omega_i_valid &&
                  drift_age_ms>=0.0 && drift_age_ms<250.0;
              highres_gyro_history.push_back(
                {q.xgyro,q.ygyro,q.zgyro,
                 drift_fresh?ahrs_omega_i_x:0.0,
                 drift_fresh?ahrs_omega_i_y:0.0,
                 drift_fresh?ahrs_omega_i_z:0.0,
                 q.time_usec,imu.recv_ns,drift_fresh,true});
              while(highres_gyro_history.size()>2 &&
                    imu.recv_ns-highres_gyro_history.front().recv_ns>3000000000LL)
                highres_gyro_history.pop_front();

              const auto highres_log_imu=imu;
              const auto highres_log_gyro=gyro;
              const uint64_t highres_log_count=imu_count;
              l.unlock();
              if(diagnosticFileIoEnabled() && !highres_gyro_shadow_ofs.is_open()){
                highres_gyro_shadow_ofs.open(
                  "/home/vio/Desktop/monkeysStab/highres_gyro_shadow_latest.csv",
                  std::ios::out|std::ios::trunc);
                if(diagnosticFileIoEnabled() && highres_gyro_shadow_ofs.is_open())
                  highres_gyro_shadow_ofs
                    <<"seq,fc_time_usec,recv_ns,fields_updated,ax_mps2,ay_mps2,az_mps2,gx_rad_s,gy_rad_s,gz_rad_s,"
                    <<"ahrs_omegaIx,ahrs_omegaIy,ahrs_omegaIz,ahrs_drift_age_ms,"
                    <<"corr_gx_rad_s,corr_gy_rad_s,corr_gz_rad_s\n";
              }
              if(diagnosticFileIoEnabled() && highres_gyro_shadow_ofs.is_open()){
                highres_gyro_shadow_ofs
                  <<highres_log_count<<','<<q.time_usec<<','<<highres_log_imu.recv_ns<<','
                  <<q.fields_updated<<','
                  <<q.xacc<<','<<q.yacc<<','<<q.zacc<<','
                  <<q.xgyro<<','<<q.ygyro<<','<<q.zgyro<<',';
                highres_gyro_shadow_ofs
                  <<(drift_fresh?ahrs_omega_i_x:0.0)<<','
                  <<(drift_fresh?ahrs_omega_i_y:0.0)<<','
                  <<(drift_fresh?ahrs_omega_i_z:0.0)<<','
                  <<drift_age_ms<<','
                  <<(q.xgyro+(drift_fresh?ahrs_omega_i_x:0.0))<<','
                  <<(q.ygyro+(drift_fresh?ahrs_omega_i_y:0.0))<<','
                  <<(q.zgyro+(drift_fresh?ahrs_omega_i_z:0.0))<<'\n';
                // Same rule for HIGHRES_IMU: diagnostic logging must never
                // block the single MAVLink RX thread at 100 Hz.
              }

              // Diagnostic only: compare FC timestamps and RPi receive timing.
              // Does not change IMU DR inputs or integration.
              if(highres_log_gyro.valid && (highres_log_count % 25u)==0u) {
                const double highres_ms=static_cast<double>(q.time_usec)*1e-3;
                const double fc_delta_ms=
                    highres_ms-static_cast<double>(highres_log_gyro.time_boot_ms);
                const double recv_delta_ms=
                    (highres_log_imu.recv_ns-highres_log_gyro.recv_ns)*1e-6;

                const double dt_s=fc_delta_ms*1e-3;
                const double droll_deg=
                    highres_log_gyro.x*dt_s*180.0/M_PI;
                const double dpitch_deg=
                    highres_log_gyro.y*dt_s*180.0/M_PI;

                // First-order gravity projection caused by attitude age.
                const double g_roll_mps2=
                    9.80665*std::sin(std::abs(droll_deg)*M_PI/180.0);
                const double g_pitch_mps2=
                    9.80665*std::sin(std::abs(dpitch_deg)*M_PI/180.0);

                if(hotPathDiagnosticIoEnabled() && diagnosticFileIoEnabled()) std::cerr<<"IMU_TIME"
                         <<" fc_dt_ms="<<fc_delta_ms
                         <<" recv_dt_ms="<<recv_delta_ms
                         <<" rateRP_deg_s=["
                         <<highres_log_gyro.x*180.0/M_PI<<","
                         <<highres_log_gyro.y*180.0/M_PI<<"]"
                         <<" dRP_deg=["
                         <<droll_deg<<","
                         <<dpitch_deg<<"]"
                         <<" gerrRP_mps2=["
                         <<g_roll_mps2<<","
                         <<g_pitch_mps2<<"]\\n";
              }
              l.lock();

              if(gyro.valid) {
                imu_dr::update(imu_dr_state,q.xacc,q.yacc,q.zacc,q.xgyro,q.ygyro,q.zgyro,
                               gyro.roll,gyro.pitch,gyro.yaw,q.time_usec,
                (imu_cam_valid && (monoNs()-imu_cam_recv_ns)>=0 &&
                 (monoNs()-imu_cam_recv_ns)<100000000LL &&
                 std::hypot(imu_cam_vn,imu_cam_ve)<0.01));
      // FUSED-V1 IMU velocity prediction. Position remains WORKED5-only in V1.
      if(imu_dr_state.calibrated &&
         imu_dr_state.diag_dt>0.0 &&
         imu_dr_state.diag_dt<0.1){
        fused_v1_vn += imu_dr_state.acc_n * imu_dr_state.diag_dt;
        fused_v1_ve += imu_dr_state.acc_e * imu_dr_state.diag_dt;
        ++fused_v1_imu_predictions;
      }
      // FUSED_V2_CAPTURE_V1: keep 500 ms of the exact DR state, keyed
      // by RPi monotonic receive time. No production state is modified.
      if(imu_dr_state.calibrated){
        const int64_t v2_now_ns=imu.recv_ns;
        fused_v2_imu_history.push_back({
          v2_now_ns,
          imu_dr_state.pos_n,imu_dr_state.pos_e,
          imu_dr_state.vel_n,imu_dr_state.vel_e});
        while(!fused_v2_imu_history.empty() &&
              v2_now_ns-fused_v2_imu_history.front().recv_ns>500000000LL)
          fused_v2_imu_history.pop_front();
      }
              const int64_t zupt_now_ns=monoNs();
              const double cam_age_ms=imu_cam_valid
                ? (zupt_now_ns-imu_cam_recv_ns)*1e-6 : 1e9;
              imu_zupt_cam_fresh=imu_cam_valid && cam_age_ms>=0.0 && cam_age_ms<100.0;
              const double cam_speed=std::hypot(imu_cam_vn,imu_cam_ve);
              imu_zupt_cam_stationary=imu_zupt_cam_fresh && cam_speed<0.01;
              const bool imu_stationary=imu_dr_state.diag_stationary;

      // FUSED-V1 horizontal stop constraint is maintained by unique
      // WORKED5 events in the camera producer below.
      if(fused_v1_stationary){
        fused_v1_vn=0.0;
        fused_v1_ve=0.0;
      }

              // Independent DR shadow from the same IMU sample.
              // Internal IMU ZUPT is disabled for this shadow.
              imu_dr::update(
                  imu_camvc_state,
                  q.xacc,q.yacc,q.zacc,
                  q.xgyro,q.ygyro,q.zgyro,
                  gyro.roll,gyro.pitch,gyro.yaw,
                  q.time_usec,
                  false);

              // Three consecutive fresh WORKED5 stationary observations
              // confirm horizontal zero velocity in the shadow only.
              if(imu_zupt_cam_stationary)
                ++imu_camvc_stop_samples;
              else
                imu_camvc_stop_samples=0;

              const bool camvc_now=imu_camvc_stop_samples>=3;
              if(camvc_now){
                if(!imu_camvc_active) ++imu_camvc_activations;
                imu_camvc_state.vel_n=0.0;
                imu_camvc_state.vel_e=0.0;
              }
              imu_camvc_active=camvc_now;

              imu_zupt_shadow=imu_stationary && imu_zupt_cam_stationary;
              if(imu_stationary){
                if(imu_zupt_shadow) ++imu_zupt_shadow_accepts;
                else ++imu_zupt_shadow_blocks;
              }
              }
              const double flowfc_highres_handler_ms=
                (monoNs()-flowfc_highres_handler_start_ns)*1e-6;
              flowfc_highres_max_handler_ms=
                std::max(flowfc_highres_max_handler_ms,flowfc_highres_handler_ms);
              if(flowfc_highres_handler_ms>5.0) ++flowfc_highres_handler_gt5;
            } else if(m.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){
              mavlink_local_position_ned_t q{}; mavlink_msg_local_position_ned_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              local.x=q.x; local.y=q.y; local.z=q.z;
              local.vx=q.vx; local.vy=q.vy; local.vz=q.vz;
              local.recv_ns=monoNs(); local.valid=true; ++local_count;
            } else if(m.msgid==MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED){
              mavlink_position_target_local_ned_t q{}; mavlink_msg_position_target_local_ned_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              target.x=q.x; target.y=q.y; target.vx=q.vx; target.vy=q.vy;
              target.type_mask=q.type_mask; target.recv_ns=monoNs(); target.valid=true;
            } else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE_TARGET){
              mavlink_attitude_target_t q{}; mavlink_msg_attitude_target_decode(&m,&q);
              // MAVLink quaternion is [w,x,y,z]. Convert only for display.
              const double w=q.q[0], x=q.q[1], y=q.q[2], z=q.q[3];
              const double sinr=2.0*(w*x+y*z), cosr=1.0-2.0*(x*x+y*y);
              const double sinp=2.0*(w*y-z*x);
              const double siny=2.0*(w*z+x*y), cosy=1.0-2.0*(y*y+z*z);
              std::lock_guard<std::mutex> l(mu);
              att_target.roll=std::atan2(sinr,cosr);
              att_target.pitch=std::asin(std::clamp(sinp,-1.0,1.0));
              att_target.yaw=std::atan2(siny,cosy);
              att_target.thrust=q.thrust; att_target.recv_ns=monoNs(); att_target.valid=true;
            } else if(m.msgid==MAVLINK_MSG_ID_RC_CHANNELS){
              mavlink_rc_channels_t q{}; mavlink_msg_rc_channels_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              rc.pwm={q.chan1_raw,q.chan2_raw,q.chan3_raw,q.chan4_raw,q.chan5_raw,q.chan6_raw,
                      q.chan7_raw,q.chan8_raw,q.chan9_raw,q.chan10_raw,q.chan11_raw,q.chan12_raw,
                      q.chan13_raw,q.chan14_raw,q.chan15_raw,q.chan16_raw,q.chan17_raw,q.chan18_raw};
              rc.recv_ns=monoNs(); rc.valid=true;
            } else if(m.msgid==MAVLINK_MSG_ID_SERVO_OUTPUT_RAW){
              mavlink_servo_output_raw_t q{}; mavlink_msg_servo_output_raw_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              outputs.pwm={q.servo1_raw,q.servo2_raw,q.servo3_raw,q.servo4_raw,
                           q.servo5_raw,q.servo6_raw,q.servo7_raw,q.servo8_raw};
              outputs.recv_ns=monoNs(); outputs.valid=true;
            } else if(m.msgid==MAVLINK_MSG_ID_SCALED_PRESSURE){
              mavlink_scaled_pressure_t q{};
              mavlink_msg_scaled_pressure_decode(&m,&q);
              if(std::isfinite(q.press_abs) && q.press_abs>100.0f){
                std::lock_guard<std::mutex> l(mu);
                baro_pressure_hpa=q.press_abs;
                baro_recv_ns=monoNs();
              }
            } else if(m.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){
              mavlink_ekf_status_report_t q{}; mavlink_msg_ekf_status_report_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              ekf.flags=q.flags;
              ekf.velocity_variance=q.velocity_variance;
              ekf.pos_horiz_variance=q.pos_horiz_variance;
              ekf.pos_vert_variance=q.pos_vert_variance;
              ekf.compass_variance=q.compass_variance;
              ekf.terrain_alt_variance=q.terrain_alt_variance;
              ekf.recv_ns=monoNs(); ekf.valid=true; ++ekf_count;
            }
          }
          flowfc_prev_read_done_ns=monoNs();
        }
      }
    });
  }

  bool latestBaro(double* pressure_hpa,int64_t* recv_ns){
    std::lock_guard<std::mutex> l(mu);
    if(baro_recv_ns==0)return false;
    *pressure_hpa=baro_pressure_hpa;
    *recv_ns=baro_recv_ns;
    return true;
  }

  bool latestLocal(FlowFcLocal* out,double* age_ms,uint64_t* count=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(count)*count=local_count;
    if(!local.valid)return false;
    *out=local;
    if(age_ms)*age_ms=(monoNs()-local.recv_ns)*1e-6;
    return true;
  }

  bool startRemoteLog(const std::string& path,double timeout_s=5.0){
    const int64_t deadline=monoNs()+(int64_t)(timeout_s*1e9);
    while(g_running && monoNs()<deadline){
      {
        std::lock_guard<std::mutex> l(mu);
        if(target_sys!=0)break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    uint8_t sys=0,comp=0;
    {
      std::lock_guard<std::mutex> l(mu);
      sys=target_sys; comp=target_comp;
      if(!sys)return false;
      remote_ofs.open(path,std::ios::binary|std::ios::trunc);
      if(!remote_ofs)return false;
      remote_path=path;
      remote_pending.clear(); remote_expected=0;
      remote_blocks_rx=remote_blocks_written=remote_duplicates=0;
      remote_active=true;
    }
    sendRemoteStatus(fd,sys,comp,MAV_REMOTE_LOG_DATA_BLOCK_START,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
    return true;
  }

  void stopRemoteLog(){
    uint8_t sys=0,comp=0;
    {
      std::lock_guard<std::mutex> l(mu);
      if(!remote_active)return;
      sys=target_sys; comp=target_comp;
      remote_active=false;
    }
    if(sys)sendRemoteStatus(fd,sys,comp,MAV_REMOTE_LOG_DATA_BLOCK_STOP,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
    std::lock_guard<std::mutex> l(mu);
    remote_ofs.flush();
    remote_ofs.close();
  }

  void remoteStats(uint64_t* rx,uint64_t* written,uint64_t* dup,size_t* pending){
    std::lock_guard<std::mutex> l(mu);
    if(rx)*rx=remote_blocks_rx;
    if(written)*written=remote_blocks_written;
    if(dup)*dup=remote_duplicates;
    if(pending)*pending=remote_pending.size();
  }

  bool latestArm(bool* out,double* age_ms=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(!heartbeat_valid)return false;
    *out=armed;
    if(age_ms)*age_ms=(monoNs()-heartbeat_recv_ns)*1e-6;
    return true;
  }

  bool latestHeartbeat(uint32_t* custom_mode,uint8_t* base_mode,uint8_t* system_status,
                       double* age_ms=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(!heartbeat_valid)return false;
    if(custom_mode)*custom_mode=heartbeat_custom_mode;
    if(base_mode)*base_mode=heartbeat_base_mode;
    if(system_status)*system_status=heartbeat_system_status;
    if(age_ms)*age_ms=(monoNs()-heartbeat_recv_ns)*1e-6;
    return true;
  }

  bool consumeGyroAverage(FlowFcGyro* out,double* age_ms,uint64_t* sample_count=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(!gyro.valid)return false;
    *out=gyro;
    if(gyro_sum_count>0){
      out->x=gyro_sum_x/gyro_sum_count;
      out->y=gyro_sum_y/gyro_sum_count;
      out->z=gyro_sum_z/gyro_sum_count;
      if(sample_count)*sample_count=gyro_sum_count;
      gyro_sum_x=gyro_sum_y=gyro_sum_z=0.0;
      gyro_sum_count=0;
    } else {
      if(sample_count)*sample_count=0;
    }
    if(age_ms)*age_ms=(monoNs()-gyro.recv_ns)*1e-6;
    return true;
  }

  bool latestRc(FlowFcRc* out,double* age_ms=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(!rc.valid)return false;
    *out=rc;
    if(age_ms)*age_ms=(monoNs()-rc.recv_ns)*1e-6;
    return true;
  }

  bool latestControl(FlowFcTarget* t,FlowFcAttTarget* a,FlowFcOutputs* o,
                     double* t_age,double* a_age,double* o_age){
    std::lock_guard<std::mutex> l(mu);
    const int64_t now=monoNs();
    if(t){*t=target;if(t_age)*t_age=target.valid?(now-target.recv_ns)*1e-6:1e9;}
    if(a){*a=att_target;if(a_age)*a_age=att_target.valid?(now-att_target.recv_ns)*1e-6:1e9;}
    if(o){*o=outputs;if(o_age)*o_age=outputs.valid?(now-outputs.recv_ns)*1e-6:1e9;}
    return target.valid || att_target.valid || outputs.valid;
  }

  bool latestEkf(FlowEkfStatus* out,double* age_ms,uint64_t* count=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(count)*count=ekf_count;
    if(!ekf.valid)return false;
    *out=ekf;
    if(age_ms)*age_ms=(monoNs()-ekf.recv_ns)*1e-6;
    return true;
  }

  void stop(){
    if(remote_active)stopRemoteLog();
    if(th.joinable())th.join();
    if(fd>=0){::close(fd);fd=-1;}
  }
};

static bool g_pi4_block_flow_tx=false;
bool sendOpticalFlow(int fd,uint64_t time_usec,float rate_x,float rate_y,uint8_t quality){
  if(g_pi4_block_flow_tx) return false;
  if(fd<0 || !std::isfinite(rate_x) || !std::isfinite(rate_y))return false;
  mavlink_message_t msg{};
  mavlink_msg_optical_flow_pack(
    FlowFc::self_sys,FlowFc::self_comp,&msg,time_usec,
    0,                 // sensor_id
    0,0,               // legacy integer flow_x/y intentionally unused
    0.0f,0.0f,         // flow_comp_m_x/y unused by ArduPilot MAV backend
    quality,
    -1.0f,              // range independently through DISTANCE_SENSOR
    rate_x,rate_y);
  return GroundMotionMavlinkPublisher::writeMessage(fd,msg);
}

bool sendHealthRadioStatus(int fd){
  if(fd<0)return false;
  mavlink_message_t msg{};
  mavlink_msg_radio_status_pack(
    FlowFc::self_sys,FlowFc::self_comp,&msg,
    254,   // rssi: JT-Zero healthy
    254,   // remrssi
    100,   // txbuf
    0,     // noise
    0,     // remnoise
    0,     // rxerrors
    0);    // fixed
  return GroundMotionMavlinkPublisher::writeMessage(fd,msg);
}

FeatureRoi g_feature_roi{};
int g_max_features=500; // production default; diagnostic sweeps may override in-process
double g_fb_shadow_max_px=0.0; // 0=disabled; diagnostic A/B only, never changes MAVLink production flow
bool g_obs_shadow_enabled=true; // D observability arm; dynamic A/B/C tests disable it to save CPU

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
    const double x=(double)au[k].x;
    const double y=(double)au[k].y;
    const double du=(double)bu[k].x-au[k].x;
    const double dv=(double)bu[k].y-au[k].y;
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

std::string ekfFlagsText(uint16_t f){
  std::ostringstream s;
  s<<"att="<<((f&1)?1:0)
   <<" velH="<<((f&2)?1:0)
   <<" velV="<<((f&4)?1:0)
   <<" posRel="<<((f&8)?1:0)
   <<" posAbs="<<((f&16)?1:0)
   <<" posVAbs="<<((f&32)?1:0)
   <<" posVAGL="<<((f&64)?1:0)
   <<" constPos="<<((f&128)?1:0)
   <<" predRel="<<((f&256)?1:0)
   <<" predAbs="<<((f&512)?1:0)
   <<" uninit="<<((f&1024)?1:0);
  return s.str();
}

} // namespace

#ifndef JTZERO_OPTFLOW_LIBRARY
int main(int argc,char** argv){
  // OPENCV_RUNTIME_DIAG_V1 -- startup diagnostics only.
  std::cerr << "OPENCV_RUNTIME threads=" << cv::getNumThreads()
            << " cpus=" << cv::getNumberOfCPUs()
            << " optimized=" << (cv::useOptimized() ? 1 : 0)
            << "\n";

  if(argc<7){
    std::cerr<<"Использование: "<<argv[0]
             <<" <camera> <luna> <fc> <csv> <camera_yaml> <focal_scale>\n";
    return 2;
  }

  const std::string camdev=argv[1], lunadev=argv[2], fcdev=argv[3];
  const std::string csvpath=argv[4], yaml=argv[5];
  const double focal_scale=std::stod(argv[6]);
  bool no_luna=false;
  bool pi4_baro=false;
  double pi4_baro_initial_height_m=0.18;
  bool pi4_runtime_safe=false;
  bool pi4_ov5647=false;
  bool pi4_camera_only=false;
  bool guided=false;
  bool continuous_guided=false;
  int continuous_legs=1;
  double guided_target_mm=175.0;
  bool require_armed=false;
  bool nominal_target_only=false;
  bool return_gui=false;
  bool return_cli=false;
  bool blind4_cli=false;
  bool rotation_gui=false;
  bool return_manual_target=false;
  bool stabilised_unified_publish=false;
  bool raw_unified_publish=false;
  std::string dataset_dir;
  std::string dataset_surface;
  double dataset_duration_sec=0.0;
  double diag_camera_x_m=std::numeric_limits<double>::quiet_NaN();
  double diag_camera_y_m=std::numeric_limits<double>::quiet_NaN();
  double diag_camera_z_m=std::numeric_limits<double>::quiet_NaN();
  double diag_range_z_m=std::numeric_limits<double>::quiet_NaN();
  double bench_height_override=0.0;
  double bench_true_camera_height=0.0;
  double pre_static_sec=5.0;
  double post_static_sec=5.0;
  std::string remote_log_path;
  for(int i=7;i<argc;i++){
    const std::string a=argv[i];
    if(a=="--guided-175"){ guided=true; guided_target_mm=175.0; }
    else if(a=="--guided-mm" && i+1<argc){ guided=true; guided_target_mm=std::stod(argv[++i]); }
    else if(a=="--continuous-legs" && i+1<argc){
      guided=true; continuous_guided=true; continuous_legs=std::stoi(argv[++i]);
    }
    else if(a=="--no-luna") no_luna=true;
    else if(a=="--pi4-baro-height" && i+1<argc){
      pi4_baro=true; no_luna=true;
      pi4_baro_initial_height_m=std::stod(argv[++i]);
    }
    else if(a=="--pi4-runtime-safe") { pi4_runtime_safe=true; no_luna=true; pi4_ov5647=true; }
    else if(a=="--pi4-ov5647") pi4_ov5647=true;
    else if(a=="--pi4-camera-only") { pi4_camera_only=true; pi4_ov5647=true; }
    else if(a=="--require-armed") require_armed=true;
    else if(a=="--nominal-target") nominal_target_only=true;
    else if(a=="--return-gui") return_gui=true;
    else if(a=="--return-cli") return_cli=true;
    else if(a=="--blind4-cli") blind4_cli=true;
    else if(a=="--rotation-gui") rotation_gui=true;
    else if(a=="--return-manual-target") return_manual_target=true;
    else if(a=="--stabilised-unified-publish") stabilised_unified_publish=true;
    else if(a=="--raw-unified-publish") raw_unified_publish=true;
    else if(a=="--dataset-dir" && i+1<argc) dataset_dir=argv[++i];
    else if(a=="--dataset-surface" && i+1<argc) dataset_surface=argv[++i];
    else if(a=="--dataset-duration-sec" && i+1<argc) dataset_duration_sec=std::stod(argv[++i]);
    else if(a=="--diag-camera-x-m" && i+1<argc) diag_camera_x_m=std::stod(argv[++i]);
    else if(a=="--diag-camera-y-m" && i+1<argc) diag_camera_y_m=std::stod(argv[++i]);
    else if(a=="--diag-camera-z-m" && i+1<argc) diag_camera_z_m=std::stod(argv[++i]);
    else if(a=="--diag-range-z-m" && i+1<argc) diag_range_z_m=std::stod(argv[++i]);
    else if(a=="--bench-height" && i+1<argc) bench_height_override=std::stod(argv[++i]);
    else if(a=="--bench-true-camera-height" && i+1<argc) bench_true_camera_height=std::stod(argv[++i]);
    else if(a=="--remote-log" && i+1<argc) remote_log_path=argv[++i];
    else if(a=="--pre-static-sec" && i+1<argc) pre_static_sec=std::stod(argv[++i]);
    else if(a=="--post-static-sec" && i+1<argc) post_static_sec=std::stod(argv[++i]);
    else if(a=="--feature-roi" && i+4<argc){
      g_feature_roi.x0=std::stod(argv[++i]);
      g_feature_roi.y0=std::stod(argv[++i]);
      g_feature_roi.x1=std::stod(argv[++i]);
      g_feature_roi.y1=std::stod(argv[++i]);
    }
    else if(a=="--max-features" && i+1<argc){
      g_max_features=std::stoi(argv[++i]);
    }
    else if(a=="--fb-shadow-max-px" && i+1<argc){
      g_fb_shadow_max_px=std::stod(argv[++i]);
    }
    else if(a=="--no-obs-shadow"){
      g_obs_shadow_enabled=false;
    }
  }
  if(stabilised_unified_publish && raw_unified_publish){
    std::cerr<<"ОШИБКА: --stabilised-unified-publish и --raw-unified-publish взаимоисключающие\n";
    return 2;
  }
  if(stabilised_unified_publish){
    std::cerr<<"STABILISED UNIFIED PUBLISH: ENABLED (experimental Variant B)\n"
             <<"REQUIRES FC FLOW_OPTIONS=1 (Stabilised). No parameter is changed automatically.\n";
  }
  if(raw_unified_publish){
    std::cerr<<"RAW UNIFIED PUBLISH: ENABLED (experimental causal35 raw-sensor contract)\n"
             <<"REQUIRES FC FLOW_OPTIONS=0. No parameter is changed automatically.\n";
  }

  if(continuous_guided && (continuous_legs<2 || continuous_legs>30)){
    std::cerr<<"ОШИБКА: --continuous-legs разрешён только 2..30\n";
    return 2;
  }
  if(guided && !(guided_target_mm>=50.0 && guided_target_mm<=1000.0)){
    std::cerr<<"ОШИБКА: --guided-mm разрешён только 50..1000 мм для стенда\n";
    return 2;
  }
  if(bench_height_override!=0.0 && !(bench_height_override>=0.20 && bench_height_override<=2.0)){
    std::cerr<<"ОШИБКА: --bench-height разрешён только 0.20..2.0 м для bench-диагностики\n";
    return 2;
  }
  if(bench_true_camera_height!=0.0 && !(bench_true_camera_height>=0.05 && bench_true_camera_height<=2.0)){
    std::cerr<<"ОШИБКА: --bench-true-camera-height разрешён только 0.05..2.0 м\n";
    return 2;
  }
  if(bench_true_camera_height>0.0 && bench_height_override<=0.0){
    std::cerr<<"ОШИБКА: --bench-true-camera-height требует --bench-height\n";
    return 2;
  }
  if(!(pre_static_sec>=1.0&&pre_static_sec<=30.0) || !(post_static_sec>=1.0&&post_static_sec<=30.0)){
    std::cerr<<"ОШИБКА: --pre-static-sec/--post-static-sec разрешены 1..30 с\n";
    return 2;
  }
  if(dataset_duration_sec<0.0 || dataset_duration_sec>3600.0){
    std::cerr<<"ОШИБКА: --dataset-duration-sec разрешён 0..3600 с\n";
    return 2;
  }
  if(!(focal_scale>0.5&&focal_scale<2.0)){
    std::cerr<<"ОШИБКА: focal_scale вне разумного диапазона 0.5..2.0\n";
    return 2;
  }
  if(!(g_feature_roi.x0>=0.0 && g_feature_roi.y0>=0.0 &&
       g_feature_roi.x1<=1.0 && g_feature_roi.y1<=1.0 &&
       g_feature_roi.x1-g_feature_roi.x0>=0.20 &&
       g_feature_roi.y1-g_feature_roi.y0>=0.20)){
    std::cerr<<"ОШИБКА: --feature-roi должен быть x0 y0 x1 y1 в 0..1 и иметь размер >=0.20\n";
    return 2;
  }
  if(g_max_features<100 || g_max_features>1000){
    std::cerr<<"ОШИБКА: --max-features разрешён только 100..1000\n";
    return 2;
  }
  if(g_fb_shadow_max_px!=0.0 && !(g_fb_shadow_max_px>=0.1 && g_fb_shadow_max_px<=5.0)){
    std::cerr<<"ОШИБКА: --fb-shadow-max-px должен быть 0 (off) или 0.1..5.0 px\n";
    return 2;
  }

  try{
    CameraCalib calib=loadCameraCalib(yaml);
    calib.fx*=focal_scale; calib.fy*=focal_scale;
    calib.K=(cv::Mat_<double>(3,3)<<calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1);

    Camera cam;
    std::unique_ptr<pi4_capture::Ov5647Capture> csi_camera;
    if(pi4_ov5647){
      csi_camera=std::make_unique<pi4_capture::Ov5647Capture>();
      csi_camera->start();
      std::cerr<<"CAMERA: OV5647 libcamera capture in original runtime\n";
    }else cam.openDev(camdev);
    LunaReader luna;
    if(!no_luna) luna.start(lunadev);
    else std::cerr<<"NO-LUNA: дальномер не запущен; метрическая навигация требует отдельного источника AGL.\n";
    if(pi4_camera_only){
      // Diagnostic inside original binary: no FC connection or MAVLink TX.
      uint64_t frames=0;
      int64_t first_ns=0,last_ns=0;
      const int64_t until_ns=monoNs()+15000000000LL;
      while(g_running && monoNs()<until_ns){
        pi4_capture::Frame f;
        if(!csi_camera->next(f,1000)) continue;
        if(!first_ns) first_ns=f.sensor_timestamp_ns;
        last_ns=f.sensor_timestamp_ns;
        ++frames;
      }
      const auto st=csi_camera->stats();
      std::cout<<"ORIGINAL_RUNTIME_CAMERA_TEST frames="<<frames
               <<" missing_ts="<<st.missing_timestamp
               <<" nonmonotonic_ts="<<st.nonmonotonic_timestamp
               <<" first_ts_ns="<<first_ns<<" last_ts_ns="<<last_ns
               <<" no_fc_connection=1 no_mavlink_tx=1\n";
      return frames>0 && st.missing_timestamp==0 &&
             st.nonmonotonic_timestamp==0 ? 0 : 1;
    }
    if(pi4_baro && (!std::isfinite(pi4_baro_initial_height_m) || pi4_baro_initial_height_m<=0.05))
      throw std::runtime_error("PI4 BARO: initial camera height must be > 0.05 m");
    if(pi4_runtime_safe){
      g_pi4_block_flow_tx=true;
      std::cerr<<"PI4 SAFE RUNTIME: OPTICAL_FLOW TX blocked; no rangefinder; FC RX enabled.\\n";
    }
    FlowFc fc; fc.start(fcdev);
    double pi4_baro_reference_hpa=0.0;
    if(!remote_log_path.empty()){
      if(fc.startRemoteLog(remote_log_path)){
        std::cerr<<"REMOTE DATAFLASH: запись запущена -> "<<remote_log_path<<"\n";
      } else {
        std::cerr<<"ПРЕДУПРЕЖДЕНИЕ: не удалось запустить REMOTE DATAFLASH logging.\n"
                 <<"Проверь LOG_BACKEND_TYPE=2 и reboot FC. Тест продолжится без BIN.\n";
      }
    }
    GroundMotionMavlinkPublisher range_pub;
    range_pub.system_id=FlowFc::self_sys;
    range_pub.component_id=FlowFc::self_comp;
    LiveWebTelemetryUdp web_live;

    std::ofstream csv(csvpath,std::ios::trunc);
    if(!csv) throw std::runtime_error("не удалось открыть CSV: "+csvpath);

    std::ofstream dataset_frames_bin;
    std::ofstream dataset_frames_csv;
    uint64_t dataset_saved_frames=0;
    uint64_t dataset_saved_bytes=0;
    int64_t dataset_start_ns=0;
    if(!dataset_dir.empty()){
      const std::string frames_bin_path=dataset_dir+"/frames.mjpgbin";
      const std::string frames_csv_path=dataset_dir+"/frames.csv";
      dataset_frames_bin.open(frames_bin_path,std::ios::binary|std::ios::trunc);
      dataset_frames_csv.open(frames_csv_path,std::ios::trunc);
      if(!dataset_frames_bin || !dataset_frames_csv)
        throw std::runtime_error("не удалось открыть файлы датасета в "+dataset_dir);
      dataset_frames_csv<<"dataset_frame,camera_ts_ns,mono_ns,jpeg_size\n";
      std::cerr<<"DATASET CAPTURE: surface="<<(dataset_surface.empty()?"unknown":dataset_surface)
               <<" dir="<<dataset_dir
               <<" duration="<<(dataset_duration_sec>0.0?std::to_string(dataset_duration_sec):std::string("manual"))
               <<" s\n";
    }
    int64_t last_csv_flush_ns=monoNs();
    constexpr int64_t kCsvLiveFlushNs=50000000LL; // 50 ms: low-latency web telemetry without per-frame fsync
    constexpr std::streamoff kCsvMaxBytes=250LL*1024LL*1024LL;
    bool csv_logging_enabled=true;
    bool csv_limit_reported=false;
    csv<<"mono_ns,camera_ts_ns,v4l2_timestamp_ns,camera_dequeue_ns,v4l2_flags,v4l2_to_dequeue_ms,camera_poll_enter_ns,camera_poll_exit_ns,camera_poll_ms,camera_dq_enter_ns,camera_dq_exit_ns,camera_dq_ioctl_ms,camera_dq_batch_ms,camera_dq_count,loop_tail_to_poll_ms,prev_send_to_csv_ms,prev_csv_block_ms,prev_csv_helpers_ms,prev_csv_stream_ms,prev_csv_flush_ms,prev_csv_tellp_ms,prev_csv_limit_ms,prev_web_block_ms,prev_postweb_ms,prev_anchor_ms,flow_send_ns,frame_pipeline_latency_ms,camera_queue_dropped,camera_queue_dropped_total,frame,guide_leg,guide_stage,valid,invalid_reason,bridge_pending,dt_s,features,tracked,inliers,inlier_ratio,t_features_ms,t_lk_ms,t_ransac_ms,t_post_ms,du_px,dv_px,du_norm,dv_norm,yaw_rate_cam_z,scale_rate,lk_height_scale,flow_cam_x,flow_cam_y,flow_body_x,flow_body_y,lever_valid,lever_production_applied,lever_flow_body_x,lever_flow_body_y,lever_pred_flow_x,lever_pred_flow_y,ab_fb_enabled,ab_fb_max_px,ab_fb_checked,ab_fb_pass,ab_fb_ratio,ab_fb_inliers,ab_fb_valid,ab_fb_flow_body_x,ab_fb_flow_body_y,ab_fb_t_ms,ab_robust_valid,ab_robust_flow_body_x,ab_robust_flow_body_y,ab_robust_sigma,ab_robust_mean_weight,ab_robust_downweighted,ab_robust_iters,ab_obs_valid,ab_obs_flow_body_x,ab_obs_flow_body_y,ab_obs_median_ratio,ab_obs_mean_weight,ab_obs_downweighted,quality,luna_m,luna_age_ms,range_to_fc_m,flow_send_x,flow_send_y,flow_sent,flow_tx_x,flow_tx_y,flow_tx_dt_s,flow_tx_inputs,raw_publish_mode,causal35_translation_valid,causal35_translation_x,causal35_translation_y,causal35_raw_gyro_x,causal35_raw_gyro_y,causal35_raw_valid,causal35_raw_x,causal35_raw_y,causal35_optical_depth_m,causal35_reject_reason,causal35_anchor_recv_age_ms,causal35_anchor_sample_age_ms,causal35_deltar_hold_ms,stabilised_publish_mode,stabilised_publish_ready,stabilised_publish_source,range_sent,fc_armed,heartbeat_valid,heartbeat_age_ms,fc_custom_mode,fc_base_mode,fc_system_status,ekf_local_valid,ekf_x_ned,ekf_y_ned,ekf_z_ned,ekf_vx_ned,ekf_vy_ned,ekf_vz_ned,ekf_age_ms,ekf_count,ekf_status_valid,ekf_flags,ekf_status_age_ms,ekf_status_count,ekf_vel_var,ekf_pos_h_var,ekf_pos_v_var,ekf_compass_var,ekf_terrain_var,return_event,rc_zero_seq,rc_valid,rc_age_ms,rc1_us,rc2_us,rc3_us,rc4_us,rc5_us,rc6_us,rc7_us,rc8_us,rc9_us,rc10_us,rc11_us,rc12_us,rc13_us,rc14_us,rc15_us,rc16_us,rc17_us,rc18_us,worked5_valid,worked5_points,worked5_hcam_m,worked5_du_norm,worked5_dv_norm,worked5_dx_m,worked5_dy_m,worked5_dN_m,worked5_dE_m,worked5_acc_n_m,worked5_acc_e_m,highdyn_active,highdyn_reason6,highdyn_raw_dx_m,highdyn_raw_dy_m,highdyn_raw_dN_m,highdyn_raw_dE_m,highdyn_acc_n_m,highdyn_acc_e_m,highdyn_confidence,fc_roll,fc_pitch,fc_yaw,fc_gyro_x,fc_gyro_y,fc_gyro_z,fc_gyro_age_ms,fc_gyro_samples,ctrl_target_valid,ctrl_target_x,ctrl_target_y,ctrl_target_vx,ctrl_target_vy,ctrl_target_age_ms,att_target_valid,att_target_roll,att_target_pitch,att_target_yaw,att_target_thrust,att_target_age_ms,outputs_valid,out1,out2,out3,out4,out5,out6,out7,out8,outputs_age_ms,c0_n,c0_bx,c0_by,c1_n,c1_bx,c1_by,c2_n,c2_bx,c2_by,c3_n,c3_bx,c3_by,c4_n,c4_bx,c4_by,c5_n,c5_bx,c5_by,c6_n,c6_bx,c6_by,c7_n,c7_bx,c7_by,c8_n,c8_bx,c8_by\n";
    csv.flush();
    AsyncCsvWriter csv_writer(csv);

    if(g_fb_shadow_max_px>0.0){
      std::cerr<<(g_obs_shadow_enabled?"A/B/C/D SHADOW: ":"A/B/C SHADOW: ")
               <<"A=production publish, B=FB-consistency <= "
               <<g_fb_shadow_max_px
               <<" px + ordinary LS, C=same B inliers + adaptive Huber IRLS";
      if(g_obs_shadow_enabled)
        std::cerr<<", D=same B inliers + adaptive structure-tensor observability weights";
      std::cerr<<"; shadow arms diagnostic only and NEVER sent to FC\n";
    }
    cv::setNumThreads(4);
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);
    if(!dataset_dir.empty()) dataset_start_ns=monoNs();

    cv::Mat prev; int64_t prev_ts=0; uint64_t frame=0;
    double prev_camera_height_m=0.0;
    bool prev_camera_height_valid=false;
    uint64_t flow_sent_total=0,flow_invalid_total=0,range_sent_total=0;
    uint64_t camera_queue_dropped_total=0, stale_flow_rejected_total=0;
    uint64_t bridge_hold_total=0, bridge_recovered_total=0, bridge_reset_total=0;
    uint64_t terrain_step_reject_total=0;
    bool bridge_pending=false;
    int64_t last_range_send_ns=0;
    double terrain_prev_range_m=0.0;
    bool terrain_prev_range_valid=false;
    int64_t terrain_guard_until_ns=0;
    constexpr double kTerrainStepAbsM=0.18;
    constexpr double kTerrainStepRatio=1.50;
    constexpr int64_t kTerrainGuardNs=400000000LL; // 0.4 s
    constexpr double kMaxFlowPipelineAgeMs=80.0;
    // TEMPORAL_OF_AGGREGATE_V1
    // Preserve angular displacement across short causal35 intervals and publish
    // one mean flow rate over the complete contiguous accumulation window.
    constexpr double kTemporalOfPublishMinDtS=0.060;
    double temporal_of_angle_x=0.0;
    double temporal_of_angle_y=0.0;
    double temporal_of_dt_s=0.0;
    uint64_t temporal_of_inputs=0;
    int64_t temporal_of_last_input_ns=0;

    // HIGH_DYNAMIC_RECOVERY_SHADOW_V1
    // Diagnostic only. Mirrors accepted WORKED5 N/E steps and, for reason=6,
    // also integrates the already-computed body flow using the current camera
    // height. It never changes WORKED5, causal35, MAVLink publication or EKF.
    double highdyn_shadow_n=0.0,highdyn_shadow_e=0.0;
    uint64_t highdyn_reason6_total=0;

    // Flight-only readiness gate. It does not arm or inhibit ArduPilot; it is an
    // explicit operator indication that the same signals used by the EKF are healthy.
    const bool flight_ready_gate=!guided;
    bool flight_ready=false;
    int64_t flight_ready_since_ns=0;
    int64_t flight_gate_begin_ns=monoNs();
    int64_t last_not_ready_print_ns=0;
    constexpr double kReadyStableSec=3.0;
    constexpr double kReadyTimeoutSec=20.0;
    constexpr double kReadyMinRangeM=0.10;
    constexpr double kReadyMaxRangeM=10.0;
    constexpr double kReadyMaxSpeedMps=0.03;
    // Readiness checks liveness of successful production OF publication,
    // not whether this exact camera-loop iteration happened to publish.
    // Normal temporal aggregation intentionally publishes about every 60 ms.
    constexpr int64_t kReadyFlowFreshNs=150000000LL; // 150 ms watchdog
    int64_t last_ready_flow_send_ns=0;

    // STARTUP_AUTO_ZERO_V1:
    // One automatic zero per runtime, only after the complete flight-readiness
    // condition has remained stable. Never repeats after an in-flight dropout.
    bool startup_zero_done=false;

    // OSD_RSSI_HEALTH_V1:
    // RSSI is a fail-safe JT-Zero health heartbeat.
    // While production OF is fresh and EKF horizontal aiding is healthy,
    // RADIO_STATUS with high RSSI is periodically sent to the FC.
    // On OF/runtime/RPi/link loss transmission stops; FC-side RSSI timeout
    // returns the OSD indication to zero without requiring any final RPi message.
    constexpr int64_t kOsdFlowFreshNs=300000000LL; // 300 ms OF watchdog
    constexpr int64_t kOsdRssiPeriodNs=500000000LL; // 2 Hz heartbeat
    int64_t last_osd_rssi_send_ns=0;

    bool return_target_set=false;
    double return_target_n=0.0,return_target_e=0.0;
    double return_view_halfspan_m=0.50;
    std::deque<cv::Point2d> return_trail;

    // Return-to-target forensic state. RAW is accumulated in native body-flow
    // measurement coordinates using the camera height above the observed plane.
    // It is intentionally kept independent from EKF position.
    // Native LOS integral is kept for continuity with earlier diagnostics.
    double return_raw_x=0.0,return_raw_y=0.0;
    // AP-model translational displacement, first in body FRD, then rotated to NED.
    double return_body_dx=0.0,return_body_dy=0.0;
    double return_ned_n=0.0,return_ned_e=0.0;

    // Always-on, diagnostic-only optical-flow integral for the Web UI.
    // It mirrors the proven return-gui RAW NED computation but never feeds FC.
    double web_raw_n=0.0,web_raw_e=0.0;
    double web_raw_vn=0.0,web_raw_ve=0.0;
    bool web_raw_step_valid=false;

    // RC6/RC8 act as a hardware HOME/zero button for monkeysStab.
    // Trigger only on a high edge; re-arm after both channels return below 1500 us.
    uint64_t rc_zero_seq=0;
    bool rc_zero_latched=false;
    uint16_t rc6_last_us=0,rc8_last_us=0,rc10_last_us=0;
    constexpr uint16_t kRcZeroPressUs=1700;
    constexpr uint16_t kRcZeroReleaseUs=1500;

    double return_yaw0=0.0;
    bool return_yaw0_set=false;
    bool return_b_marked=false;
    bool return_home_marked=false;
    double return_b_n=0.0,return_b_e=0.0;
    double return_b_raw_x=0.0,return_b_raw_y=0.0,return_b_yaw=0.0;
    double return_b_body_dx=0.0,return_b_body_dy=0.0;
    double return_b_ned_n=0.0,return_b_ned_e=0.0;
    int pending_return_event=0; // 1=A/target, 2=B/turn, 3=H/physical-home mark
    const std::string return_window_name="JT-Zero — Возврат в исходную точку";
    const std::string rotation_window_name="JT-Zero — Полёт / 3D положение";
    bool traj3d_origin_set=false;
    double traj3d_n0=0.0,traj3d_e0=0.0,traj3d_z0=0.0;
    // 3D GUI: fixed operator scale ±500 mm on every axis. Never auto-zoom:
    // the apparent displacement must remain visually comparable during the test.
    constexpr double traj3d_halfspan_m=0.50;
    std::deque<cv::Vec3d> traj3d; // N,E,UP relative to hover/reference point
    cv::Vec3d traj3d_prev(0,0,0);
    bool traj3d_prev_set=false;
    double traj3d_path_total=0.0;              // accumulated 3D path length, m
    cv::Vec3d traj3d_path_axis(0,0,0);         // accumulated |dN|,|dE|,|dUP|, m
    cv::Vec3d traj3d_peak_abs(0,0,0);           // max |X|,|Y|,|Z| since SPACE
    bool traj3d_preview_origin_set=false;       // live preview before SPACE
    double traj3d_preview_n0=0.0,traj3d_preview_e0=0.0,traj3d_preview_z0=0.0;
    bool traj3d_range_origin_set=false;
    double traj3d_range_vertical0=0.0;           // tilt-compensated TF-Luna vertical distance at SPACE
    bool traj3d_preview_range_origin_set=false;
    double traj3d_preview_range_vertical0=0.0;
    // Canonical bench mode: keyboard commands come from the terminal, with no OpenCV window.
    // stdin is put into non-canonical/no-echo mode and restored automatically on exit.
    struct CliTerminalGuard {
      bool active=false;
      termios saved{};
      int saved_flags=-1;
      explicit CliTerminalGuard(bool enable){
        if(!enable || !::isatty(STDIN_FILENO)) return;
        if(::tcgetattr(STDIN_FILENO,&saved)!=0) return;
        termios raw=saved;
        raw.c_lflag &= ~(ICANON|ECHO);
        raw.c_cc[VMIN]=0;
        raw.c_cc[VTIME]=0;
        if(::tcsetattr(STDIN_FILENO,TCSANOW,&raw)!=0) return;
        saved_flags=::fcntl(STDIN_FILENO,F_GETFL,0);
        if(saved_flags>=0) ::fcntl(STDIN_FILENO,F_SETFL,saved_flags|O_NONBLOCK);
        active=true;
      }
      ~CliTerminalGuard(){
        if(!active) return;
        ::tcsetattr(STDIN_FILENO,TCSANOW,&saved);
        if(saved_flags>=0) ::fcntl(STDIN_FILENO,F_SETFL,saved_flags);
      }
      int readKey(){
        if(!active) return -1;
        unsigned char c=0;
        const ssize_t n=::read(STDIN_FILENO,&c,1);
        return n==1 ? (int)c : -1;
      }
    } cli_terminal(return_cli || blind4_cli);
    if((return_cli || blind4_cli) && !cli_terminal.active)
      throw std::runtime_error("--return-cli/--blind4-cli требует интерактивный TTY stdin");

    // Strict one-way state machine for the canonical hand test.
    // Canonical metric A/B protocol:
    // 0=WAIT_A, 1=GO_B, 2=ENTER_GT, 3=WAIT_RETURN_SPACE, 4=RETURN_A.
    int canonical_state=0;
    // Blind4 events: 11=A1,12=B1,13=A2,14=B2,15=A3,16=B3,17=A4,18=B4.
    int blind4_state=0;
    std::string canonical_gt_buf;
    double canonical_gt_mm=0.0;
    double return_fb_body_dx=0.0,return_fb_body_dy=0.0;
    double return_fb_ned_n=0.0,return_fb_ned_e=0.0;
    double return_b_fb_body_dx=0.0,return_b_fb_body_dy=0.0;
    double return_b_fb_ned_n=0.0,return_b_fb_ned_e=0.0;

    if(return_gui || rotation_gui) initGuiFont();
    if(return_gui){
      cv::namedWindow(return_window_name,cv::WINDOW_NORMAL);
      cv::resizeWindow(return_window_name,1500,900);
      std::cerr<<"GUI ВОЗВРАТА: "<<(return_manual_target?"точка A задаётся вручную после подъёма":"точка A задаётся автоматически после готовности")<<".\n"
               <<"Клавиши: SPACE=A/домой, B=дальняя точка, H=физический возврат, C=очистить хвост, Q/ESC=выход.\n";
    }
    if(rotation_gui){
      cv::namedWindow(rotation_window_name,cv::WINDOW_NORMAL);
      cv::resizeWindow(rotation_window_name,1500,900);
      std::cerr<<"GUI ПОЛЁТА: состояние Optical Flow + 3D положение по оценке FC.\n"
               <<"SPACE = принять текущую точку за новый 0; Q/ESC = выход.\n";
    }

    std::atomic<int> guide_stage{0}; // 0=pre-static, 1=move, 2=post-static, 3=wait-next, 4=done
    std::atomic<int> guide_leg{0};
    std::atomic<bool> arm_lost{false};
    FlowFcLocal guide_start{}, guide_end{};
    std::thread guide_thread;
    if(guided){
      guide_thread=std::thread([&]{
        std::this_thread::sleep_for(std::chrono::milliseconds(750));
        bool arm=false; double arm_age=1e9;
        const bool have_arm=fc.latestArm(&arm,&arm_age) && arm_age<2500.0;

        std::cerr<<"\n======================================================================\n"
                 <<(continuous_guided?"CONTINUOUS RECIPROCAL TEST":"GUIDED TEST")
                 <<" — НОМИНАЛЬНЫЙ СДВИГ "<<guided_target_mm<<" мм\n"
                 <<"======================================================================\n"
                 <<"Проходов: "<<(continuous_guided?continuous_legs:1)<<"\n"
                 <<"Один процесс камеры/MAVLink/DataFlash на всю серию.\n"
                 <<"Фактическое расстояние измеряется после каждого прохода.\n"
                 <<"======================================================================\n"
                 <<"ARM STATE: "<<(have_arm?(arm?"ARMED":"DISARMED"):"NO_DATA")<<"\n";
        if(require_armed && (!have_arm || !arm)){
          std::cerr<<"ОШИБКА: этот тест требует ARMED.\n";
          g_running=false; return;
        }

        auto wait_height=[&](double stable_sec)->bool{
          if(bench_height_override<=0.0) return true;
          constexpr double kHgtTolM=0.035;
          constexpr double kTimeoutSec=20.0;
          std::cerr<<"\n>>> СИНХРОНИЗАЦИЯ ВЫСОТЫ. НЕ ДВИГАТЬ.\n"
                   <<">>> Ждём LOCAL Z около -"<<bench_height_override
                   <<" м (±"<<kHgtTolM<<" м) непрерывно "<<stable_sec<<" с.\n";
          const int64_t sync_begin=monoNs();
          int64_t stable_begin=0;
          double last_z=0.0,last_age=1e9;
          while(g_running){
            FlowFcLocal q{}; double qage=1e9; uint64_t qcount=0;
            const bool qok=fc.latestLocal(&q,&qage,&qcount) && qage<500.0;
            if(qok){
              last_z=q.z; last_age=qage;
              const bool in_band=std::abs((-double(q.z))-bench_height_override)<=kHgtTolM;
              if(in_band){
                if(stable_begin==0) stable_begin=monoNs();
                if((monoNs()-stable_begin)*1e-9>=stable_sec){
                  std::cerr<<">>> ВЫСОТА СТАБИЛЬНА: LOCAL Z="<<q.z
                           <<" м, inferred HAGL="<<(-q.z)<<" м.\n";
                  return true;
                }
              } else {
                stable_begin=0;
              }
            }
            if((monoNs()-sync_begin)*1e-9>=kTimeoutSec){
              std::cerr<<"\nОШИБКА: EKF height не сошёлся за "<<kTimeoutSec
                       <<" с. Последний LOCAL Z="<<last_z<<" м age="<<last_age<<" ms.\n";
              return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          return false;
        };

        const int legs=continuous_guided?continuous_legs:1;
        for(int leg=1; leg<=legs && g_running; ++leg){
          guide_leg=leg;
          guide_stage=0;
          if(!wait_height(leg==1?2.0:1.0)){ g_running=false; return; }

          std::cerr<<"\n======================================================================\n"
                   <<"LEG "<<leg<<" / "<<legs<<"\n"
                   <<"======================================================================\n"
                   <<"СТАТИКА "<<pre_static_sec<<" секунд. НЕ ДВИГАТЬ.\n";
          std::this_thread::sleep_for(std::chrono::milliseconds((int)std::llround(pre_static_sec*1000.0)));

          double age=0; uint64_t count=0;
          if(!fc.latestLocal(&guide_start,&age,&count) || age>500){
            std::cerr<<"ОШИБКА GUIDE: нет свежего LOCAL_POSITION_NED перед движением.\n";
            g_running=false; return;
          }

          guide_stage=1;
          std::cerr<<"\n>>> LEG "<<leg<<" ДВИГАЙТЕ\n"
                   <<">>> Сдвиньте аппарат строго по столу. После полной остановки нажмите Enter.\n";
          std::string line; std::getline(std::cin,line);

          guide_stage=2;
          std::cerr<<"\n>>> LEG "<<leg<<" СТОП. НЕ ТРОГАТЬ аппарат "<<post_static_sec<<" секунд.\n";
          std::this_thread::sleep_for(std::chrono::milliseconds((int)std::llround(post_static_sec*1000.0)));

          if(!fc.latestLocal(&guide_end,&age,&count) || age>500){
            std::cerr<<"ОШИБКА GUIDE: нет свежего LOCAL_POSITION_NED после движения.\n";
            g_running=false; return;
          }
          if(require_armed && arm_lost.load()){
            std::cerr<<"ARMed-test прерван из-за DISARM.\n";
            g_running=false; return;
          }

          const double dn=guide_end.x-guide_start.x, de=guide_end.y-guide_start.y;
          const double dist=std::hypot(dn,de);
          std::cerr<<"\n======================================================================\n"
                   <<"CONTINUOUS LEG "<<leg<<" RESULT\n"
                   <<"START N/E = ("<<guide_start.x<<", "<<guide_start.y<<") m\n"
                   <<"END   N/E = ("<<guide_end.x<<", "<<guide_end.y<<") m\n"
                   <<"DELTA N/E = ("<<dn<<", "<<de<<") m\n"
                   <<"EKF horizontal displacement = "<<dist*1000.0<<" mm\n"
                   <<"Nominal guided target = "<<guided_target_mm
                   <<" mm (ТОЛЬКО ИНСТРУКЦИЯ; физический эталон вводится в GUI)\n"
                   <<"======================================================================\n";

          guide_stage=3;
          std::cerr<<"\n>>> LEG "<<leg<<" COMPLETE.\n";
          if(leg<legs){
            std::cerr<<">>> Нажмите Enter, чтобы перейти к следующему проходу. "
                     <<"ЧИСЛА ЗДЕСЬ НЕ ВВОДЯТСЯ. Физический эталон задаётся самим протоколом.\n";
            std::getline(std::cin,line);
          }
        }

        guide_stage=4;
        guide_leg=legs;
        std::cerr<<"\n>>> CONTINUOUS SERIES COMPLETE\n";
        g_running=false;
      });
    }

    std::cerr<<"JT-ZERO OPTICAL FLOW MAVLINK MVP v2\n"
             <<"camera="<<camdev<<"\n"
             <<"fx/fy effective="<<calib.fx<<" / "<<calib.fy
             <<" (focal_scale="<<focal_scale<<")\n"
             <<"max_features="<<g_max_features<<"\n"
             <<"ВАЖНО: publisher выдаёт body-FRD flow; ожидается FLOW_ORIENT_YAW=0, FLOW_OPTIONS=0\n"
             <<"DIAG: запрошен EKF_STATUS_REPORT 5 Hz; LOCAL_POSITION_NED 20 Hz; ATTITUDE 100 Hz\n";
    if(return_gui && std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
      std::cerr<<"RETURN GUI geometry: camera_z="<<diag_camera_z_m
               <<" m range_z="<<diag_range_z_m
               <<" m, camera-range dz="<<(diag_camera_z_m-diag_range_z_m)<<" m\n";
    }
    if(bench_height_override>0.0){
      std::cerr<<"BENCH HEIGHT OVERRIDE: FC получает "<<bench_height_override
               <<" м вместо реального TF-Luna.\n";
      if(bench_true_camera_height>0.0){
        std::cerr<<"FIXED TRUE CAMERA HEIGHT: "<<bench_true_camera_height
                 <<" м; TF-Luna НЕ используется для метрического масштаба flow.\n";
      } else {
        std::cerr<<"Для synthetic range масштабируется ТОЛЬКО translational flow; rotational flow остаётся неизменным.\n";
      }
      std::cerr<<"ЭТО ТОЛЬКО СТЕНДОВАЯ ДИАГНОСТИКА, НЕ FLIGHT-РЕЖИМ.\n";
    }

    // Переводим AP_OpticalFlow_MAV в high-precision flow_rate mode.
    // quality=0: это не валидное aiding measurement.
    sendOpticalFlow(fc.fd,(uint64_t)(monoNs()/1000),1.0e-6f,0.0f,0);

    // MAIN_LOOP_STAGE_FORENSIC_V1: carry previous-iteration timings into
    // the next CSV row so stages after the CSV write can also be measured.
    double prev_send_to_csv_ms=-1.0;
    double prev_csv_block_ms=-1.0;
    double prev_csv_helpers_ms=-1.0;
    double prev_csv_stream_ms=-1.0;
    double prev_csv_flush_ms=-1.0;
    double prev_csv_tellp_ms=-1.0;
    double prev_csv_limit_ms=-1.0;
    double prev_web_block_ms=-1.0;
    double prev_postweb_ms=-1.0;
    double prev_anchor_ms=-1.0;
    int64_t prev_loop_end_ns=0;

    // STARTUP_CLOCK_RESYNC_V1:
    // Start timing only after OV9281 has produced a successfully decoded frame.
    // HIGHRES_CLOCK_MAP_V2 is reset exactly once 10 s later.
    int64_t startup_first_decoded_ns=0;
    bool startup_clock_reset_done=false;

    while(g_running){
      const int64_t camera_poll_enter_ns=monoNs();
      int pr=1;
      if(!pi4_ov5647){
        pollfd p{cam.fd,POLLIN,0};
        pr=poll(&p,1,20);
      }
      const int64_t camera_poll_exit_ns=monoNs();
      if(pr<0){if(errno==EINTR)continue;fail("camera poll");}
      if(pr<=0)continue;

      // КРИТИЧЕСКИ: обработка KLT медленнее capture-rate камеры. Если
      // обрабатывать каждый queued MJPEG кадр, возникает постоянный backlog
      // (~0.4-0.8 с в плохом прогоне), а ArduPilot компенсирует такой старый
      // flow СВЕЖИМ gyro. Поэтому всегда выкидываем промежуточные queued
      // кадры и обрабатываем только самый свежий доступный кадр.
      // FPS FORENSIC: считаем все DQBUF, выбранные newest кадры, decode и WORKED5.
      static uint64_t fps_dqbuf=0, fps_selected=0, fps_queue_drop=0;
      static uint64_t fps_decoded=0, fps_w5_attempt=0, fps_w5_valid=0;
      static int64_t fps_t0_ns=monoNs();
      static int64_t fps_prev_selected_ts=0;
      static double fps_dt_sum_ms=0.0, fps_dt_max_ms=0.0;
      static uint64_t fps_dt_n=0;

      // W5 WINDOW FORENSIC: локальное окно 250 ms, чтобы короткий провал
      // не растворялся в cumulative FPS_FORENSIC.
      static uint64_t w5w_dqbuf=0, w5w_selected=0, w5w_drop=0;
      static uint64_t w5w_decoded=0, w5w_attempt=0, w5w_valid=0;
      static uint64_t w5w_dt_n=0;
      static double w5w_dt_sum_ms=0.0, w5w_dt_max_ms=0.0;
      static int64_t w5w_t0_ns=monoNs();

      // LK_FORENSIC_CAPTURE_V1: compressed MJPEG ring in RAM.
      // Diagnostic only: no WORKED5/LK parameters or measurements are changed.
      struct LkForensicFrame {
        uint64_t frame=0;
        int64_t mono_ns=0, v4l2_ns=0, dq_ns=0;
        uint32_t v4l2_flags=0;
        std::vector<uint8_t> jpeg;
      };
      static std::deque<LkForensicFrame> lkfc_ring;
      static bool lkfc_triggered=false;
      static int64_t lkfc_trigger_ns=0;
      static uint64_t lkfc_trigger_frame=0;
      static constexpr int64_t kLkfcPreNs=2000000000LL;
      static constexpr int64_t kLkfcPostNs=2000000000LL;
      static constexpr double kLkfcTriggerMs=20.0;
      static uint64_t lkfc_seq=0;

      std::vector<uint8_t> latest_jpeg;
      int64_t ts=0;
      int64_t selected_v4l2_ts_ns=0;
      int64_t selected_dq_mono_ns=0;
      uint32_t selected_v4l2_flags=0;
      uint64_t camera_queue_dropped=0;
      const int64_t camera_dq_batch_enter_ns=monoNs();
      int64_t camera_dq_first_enter_ns=0;
      int64_t camera_dq_last_exit_ns=0;
      double camera_dq_ioctl_max_ms=0.0;
      uint64_t camera_dq_count=0;
      if(pi4_ov5647){
        pi4_capture::Frame captured;
        const int64_t dq_enter_ns=monoNs();
        if(!csi_camera->next(captured,1000)) continue;
        camera_dq_first_enter_ns=dq_enter_ns;
        camera_dq_last_exit_ns=monoNs();
        camera_dq_ioctl_max_ms=(camera_dq_last_exit_ns-dq_enter_ns)*1e-6;
        camera_dq_count=1;
        ++fps_dqbuf; ++w5w_dqbuf;
        ts=captured.sensor_timestamp_ns;
        selected_v4l2_ts_ns=ts;
        selected_dq_mono_ns=camera_dq_last_exit_ns;
        // Preserve the original downstream MJPEG/decode pipeline.
        if(!cv::imencode(".jpg",captured.gray,latest_jpeg))
          throw std::runtime_error("OV5647 JPEG encode failed");
      }else{
      while(g_running){
        v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        const int64_t dq_enter_ns=monoNs();
        if(camera_dq_first_enter_ns==0) camera_dq_first_enter_ns=dq_enter_ns;
        const int dq_rc=xioctl(cam.fd,VIDIOC_DQBUF,&b);
        const int64_t dq_exit_ns=monoNs();
        camera_dq_last_exit_ns=dq_exit_ns;
        camera_dq_ioctl_max_ms=std::max(
          camera_dq_ioctl_max_ms,(dq_exit_ns-dq_enter_ns)*1e-6);
        if(dq_rc<0){
          if(errno==EAGAIN)break;
          fail("VIDIOC_DQBUF");
        }
        ++camera_dq_count;
        ++fps_dqbuf; ++w5w_dqbuf;
        const int64_t bts=(int64_t)b.timestamp.tv_sec*1000000000LL+(int64_t)b.timestamp.tv_usec*1000LL;
        // UVC/V4L2 reports a monotonic frame timestamp on this production
        // camera (verified from V4L2 buffer flags and measured against DQBUF).
        // Use the frame timestamp for camera/ATTITUDE alignment; keep DQBUF
        // monotonic time only for pipeline-age diagnostics.
        const int64_t dq_mono_ns=monoNs();
        const bool v4l2_monotonic=(b.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)!=0;
        const int64_t frame_mono_ns=(v4l2_monotonic && bts>0) ? bts : dq_mono_ns;
        if(!latest_jpeg.empty()) ++camera_queue_dropped;
        const uint8_t* pjpeg=reinterpret_cast<const uint8_t*>(cam.bufs[b.index].p);
        latest_jpeg.assign(pjpeg,pjpeg+b.bytesused);
        ts=frame_mono_ns;
        selected_v4l2_ts_ns=bts;
        selected_dq_mono_ns=dq_mono_ns;
        selected_v4l2_flags=b.flags;
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");
      }
      }
      const int64_t camera_dq_batch_exit_ns=monoNs();
      if(latest_jpeg.empty()) continue;
      camera_queue_dropped_total += camera_queue_dropped;
      ++fps_selected; ++w5w_selected;
      fps_queue_drop += camera_queue_dropped;
      w5w_drop += camera_queue_dropped;
      if(fps_prev_selected_ts>0 && ts>fps_prev_selected_ts){
        const double dms=(ts-fps_prev_selected_ts)*1e-6;
        fps_dt_sum_ms+=dms;
        fps_dt_max_ms=std::max(fps_dt_max_ms,dms);
        ++fps_dt_n;
        w5w_dt_sum_ms+=dms; w5w_dt_max_ms=std::max(w5w_dt_max_ms,dms); ++w5w_dt_n;
      }
      fps_prev_selected_ts=ts;

      const int64_t now=monoNs();

      if(dataset_frames_bin.is_open()){
        const uint64_t ts64=(uint64_t)std::max<int64_t>(0,ts);
        const uint32_t sz32=(uint32_t)std::min<size_t>(latest_jpeg.size(),0xffffffffu);
        dataset_frames_bin.write(reinterpret_cast<const char*>(&ts64),sizeof(ts64));
        dataset_frames_bin.write(reinterpret_cast<const char*>(&sz32),sizeof(sz32));
        dataset_frames_bin.write(reinterpret_cast<const char*>(latest_jpeg.data()),sz32);
        ++dataset_saved_frames;
        dataset_saved_bytes += sizeof(ts64)+sizeof(sz32)+sz32;
        dataset_frames_csv<<dataset_saved_frames<<','<<ts<<','<<now<<','<<sz32<<'\n';
        if((dataset_saved_frames%120)==0){
          dataset_frames_bin.flush();
          dataset_frames_csv.flush();
        }
      }

      if(dataset_start_ns>0 && dataset_duration_sec>0.0 &&
         (now-dataset_start_ns)*1e-9 >= dataset_duration_sec){
        std::cerr<<"DATASET CAPTURE COMPLETE: "
                 <<dataset_saved_frames<<" frames, "
                 <<dataset_saved_bytes<<" bytes\n";
        g_running=false;
        break;
      }

      cv::Mat raw(1,(int)latest_jpeg.size(),CV_8UC1,latest_jpeg.data());
      cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);
      if(gray.empty()) continue;
      ++fps_decoded; ++w5w_decoded;
      ++frame;

      if(startup_first_decoded_ns==0)
        startup_first_decoded_ns=monoNs();

      if(!startup_clock_reset_done &&
         monoNs()-startup_first_decoded_ns>=10000000000LL){
        {
          std::lock_guard<std::mutex> l(fc.mu);
          fc.resetHighresClockMap();
        }
        startup_clock_reset_done=true;
        std::cerr
          <<"STARTUP_CLOCK_RESYNC_V1: HIGHRES_CLOCK_MAP_V2 reset "
          <<"10 s after first decoded OV9281 frame\n";
      }

      // OV9281_OF_AE_V2: lightweight adaptive shutter for optical flow.
      // Keep the proven OF path untouched. Brightness is estimated from a
      // sparse histogram inside the OF ROI: no ROI clone, sort or OpenCV work.
      // Update only at ~2 Hz (100 fps / 50 frames) to avoid control chatter.
      static const int64_t ae_capture_start_ns=monoNs();
      if((frame%50)==0 && monoNs()-ae_capture_start_ns>=1000000000LL){
        const int ax0=std::clamp((int)std::lround(g_feature_roi.x0*gray.cols),0,gray.cols-1);
        const int ay0=std::clamp((int)std::lround(g_feature_roi.y0*gray.rows),0,gray.rows-1);
        const int ax1=std::clamp((int)std::lround(g_feature_roi.x1*gray.cols),ax0+1,gray.cols);
        const int ay1=std::clamp((int)std::lround(g_feature_roi.y1*gray.rows),ay0+1,gray.rows);

        std::array<uint32_t,256> ae_hist{};
        uint32_t ae_samples=0;
        constexpr int kAeSampleStep=8;
        for(int y=ay0;y<ay1;y+=kAeSampleStep){
          const uint8_t* row=gray.ptr<uint8_t>(y);
          for(int x=ax0;x<ax1;x+=kAeSampleStep){
            ++ae_hist[row[x]];
            ++ae_samples;
          }
        }

        int ae_median=0;
        if(ae_samples>0){
          const uint32_t half=(ae_samples+1)/2;
          uint32_t acc=0;
          for(int v=0;v<256;++v){
            acc+=ae_hist[(size_t)v];
            if(acc>=half){ ae_median=v; break; }
          }
        }

        // Sweep measurements show useful texture around this brightness range.
        // The OV9281 bright plateau is near 244, so median is more reliable
        // here than a >=250 clipping counter.
        constexpr int kAeLow=110;
        constexpr int kAeHigh=170;
        const int old_exp=cam.exposureAbsolute();
        int next_exp=old_exp;
        if(ae_samples>0 && ae_median>kAeHigh){
          next_exp=std::max(1,(int)std::floor(old_exp*0.80));
          if(next_exp==old_exp) --next_exp;
        } else if(ae_samples>0 && ae_median<kAeLow){
          next_exp=std::min(200,(int)std::ceil(old_exp*1.15));
          if(next_exp==old_exp) ++next_exp;
        }
        next_exp=std::clamp(next_exp,1,200);

        if(next_exp!=old_exp){
          if(!cam.requestExposureAbsolute(next_exp)){
            std::cerr<<"OV9281_OF_AE_V2 request exposure failed old="<<old_exp
                     <<" requested="<<next_exp<<"\n";
          } else {
            std::cerr<<"OV9281_OF_AE_V2 median="<<ae_median
                     <<" exposure_request="<<old_exp<<"->"<<next_exp<<"\n";
          }
        }
      }

      // RPZ2 diagnostic snapshots: the console experiment creates a marker,
      // and the production camera loop saves the next decoded frame.
      for(const auto& snap : std::array<std::pair<const char*,const char*>,2>{{
            {"/tmp/monkeys_rpz2_capture_first","rpz2_first.jpg"},
            {"/tmp/monkeys_rpz2_capture_last","rpz2_last.jpg"}}}){
        std::error_code ec;
        if(std::filesystem::exists(snap.first,ec)){
          const std::filesystem::path production_csv_path(csvpath);
          const auto jpg_path=production_csv_path.parent_path()/snap.second;
          if(cv::imwrite(jpg_path.string(),gray)){
            const auto meta_path=production_csv_path.parent_path()/"rpz2_frames.csv";
            const bool new_meta=!std::filesystem::exists(meta_path);
            std::ofstream meta(meta_path,std::ios::out|std::ios::app);
            if(new_meta) meta<<"label,frame,mono_ns,file\n";
            const char* label=(std::string(snap.second)=="rpz2_first.jpg")?"FIRST":"LAST";
            meta<<label<<','<<frame<<','<<ts<<','<<snap.second<<'\n';
            std::cerr<<"RPZ2 SNAPSHOT: "<<jpg_path<<" frame="<<frame
                     <<" mono_ns="<<ts<<"\n";
          }
          std::filesystem::remove(snap.first,ec);
        }
      }

      // Preserve only successfully decoded selected MJPEG frames. frame now
      // exactly matches the production CSV frame counter.
      lkfc_ring.push_back(LkForensicFrame{frame,ts,selected_v4l2_ts_ns,
                                          selected_dq_mono_ns,selected_v4l2_flags,
                                          latest_jpeg});
      while(lkfc_ring.size()>1 && ts-lkfc_ring.front().mono_ns>kLkfcPreNs+kLkfcPostNs)
        lkfc_ring.pop_front();

        if(now-w5w_t0_ns>=250000000LL){
          const double wsec=(now-w5w_t0_ns)*1e-9;
          if(hotPathDiagnosticIoEnabled()) std::cerr<<"W5_WINDOW"
                   <<" dqbuf_hz="<<(w5w_dqbuf/wsec)
                   <<" selected_hz="<<(w5w_selected/wsec)
                   <<" dropped_hz="<<(w5w_drop/wsec)
                   <<" decoded_hz="<<(w5w_decoded/wsec)
                   <<" attempt_hz="<<(w5w_attempt/wsec)
                   <<" valid_hz="<<(w5w_valid/wsec)
                   <<" valid_ratio="<<(w5w_attempt?double(w5w_valid)/double(w5w_attempt):0.0)
                   <<" dt_mean_ms="<<(w5w_dt_n?w5w_dt_sum_ms/w5w_dt_n:0.0)
                   <<" dt_max_ms="<<w5w_dt_max_ms
                   <<"\n";
          w5w_dqbuf=w5w_selected=w5w_drop=w5w_decoded=w5w_attempt=w5w_valid=0;
          w5w_dt_n=0; w5w_dt_sum_ms=0.0; w5w_dt_max_ms=0.0;
          w5w_t0_ns=now;
        }

        if(now-fps_t0_ns>=2000000000LL){
          const double sec=(now-fps_t0_ns)*1e-9;
          if(hotPathDiagnosticIoEnabled()) std::cerr<<"FPS_FORENSIC"
                   <<" dqbuf_hz="<<(fps_dqbuf/sec)
                   <<" selected_hz="<<(fps_selected/sec)
                   <<" dropped_hz="<<(fps_queue_drop/sec)
                   <<" decoded_hz="<<(fps_decoded/sec)
                   <<" w5_attempt_hz="<<(fps_w5_attempt/sec)
                   <<" w5_valid_hz="<<(fps_w5_valid/sec)
                   <<" selected_dt_mean_ms="<<(fps_dt_n?fps_dt_sum_ms/fps_dt_n:0.0)
                   <<" selected_dt_max_ms="<<fps_dt_max_ms
                   <<"\n";
          // Start a fresh forensic window. Without this reset the condition
          // remains true forever after the first 2 s and stderr is written on
          // every camera frame (~100 Hz). Under web_service stderr is a regular
          // file, so that accidental log flood can block the main camera loop.
          fps_dqbuf=fps_selected=fps_queue_drop=fps_decoded=0;
          fps_w5_attempt=fps_w5_valid=0;
          fps_dt_n=0;
          fps_dt_sum_ms=0.0;
          fps_dt_max_ms=0.0;
          fps_t0_ns=now;
        }

        double lm=0; int strength=0; int64_t lns=0;
        bool hl=luna.latest(&lm,&strength,&lns);
        if(pi4_baro){
          double pressure=0.0; int64_t pressure_ns=0;
          hl=false;
          if(fc.latestBaro(&pressure,&pressure_ns) && now-pressure_ns<500000000LL){
            if(pi4_baro_reference_hpa==0.0)pi4_baro_reference_hpa=pressure;
            const double delta_h=44330.0*(1.0-std::pow(pressure/pi4_baro_reference_hpa,0.190294957));
            lm=pi4_baro_initial_height_m+delta_h;
            lns=pressure_ns;
            hl=std::isfinite(lm) && lm>0.05;
          }
        }
        const double lage=hl?(now-lns)*1e-6:1e9;
        bool range_sent=false;
        const double range_to_fc=(bench_height_override>0.0)?bench_height_override:lm;

        // A downward rangefinder can jump from table to floor (or back) while the
        // vehicle itself has not moved vertically. During that short transition
        // the camera often sees BOTH depth planes, so there is no single metric
        // scale for optical flow. Do not feed those mixed-plane frames to EKF.
        // Resume automatically after 0.4 s with the newest frame anchor.
        if(!pi4_baro && bench_height_override<=0.0 && hl && lage<100.0 && lm>0.05){
          if(terrain_prev_range_valid){
            const double d=std::abs(lm-terrain_prev_range_m);
            const double ratio=std::max(lm,terrain_prev_range_m)/
                               std::max(0.05,std::min(lm,terrain_prev_range_m));
            if(d>=kTerrainStepAbsM && ratio>=kTerrainStepRatio){
              terrain_guard_until_ns=now+kTerrainGuardNs;
            }
          }
          terrain_prev_range_m=lm;
          terrain_prev_range_valid=true;
        }
        const bool terrain_step_guard = now < terrain_guard_until_ns;

        if(!pi4_baro && hl&&lage<200&&(last_range_send_ns==0||now-last_range_send_ns>=50000000LL)){
          range_sent=range_pub.sendDistanceSensor(fc.fd,(uint32_t)(now/1000000LL),range_to_fc);
          last_range_send_ns=now;
          if(range_sent)++range_sent_total;
        }

        double current_camera_height_m=0.0;
        bool current_camera_height_valid=hl && lage<100.0 && lm>0.05;
        if(current_camera_height_valid){
          current_camera_height_m=lm;
          // Convert rangefinder optical origin to camera optical origin using
          // the already audited current-mount Z offsets.
          if(!pi4_baro && std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
            current_camera_height_m=lm-(diag_camera_z_m-diag_range_z_m);
          }
          if(!(current_camera_height_m>0.05 && std::isfinite(current_camera_height_m)))
            current_camera_height_valid=false;
        }

        const double dt=prev_ts?(ts-prev_ts)*1e-9:0.0;
        FlowStep s;
        if(!prev.empty())s=estimateRawFlow(
          prev,gray,dt,calib,
          prev_camera_height_valid?prev_camera_height_m:0.0,
          current_camera_height_valid?current_camera_height_m:0.0);

        // Trigger on production forward-LK wall time.  Do not use valid=0:
        // the observed collapse begins before the final validity gate fails.
        if(!lkfc_triggered && s.t_lk_ms>kLkfcTriggerMs){
          lkfc_triggered=true;
          lkfc_trigger_ns=ts;
          lkfc_trigger_frame=frame;
          std::cerr<<"LK_FORENSIC TRIGGER frame="<<frame
                   <<" lk_ms="<<s.t_lk_ms<<" dt_ms="<<(dt*1000.0)<<"\n";
        }
        if(lkfc_triggered && ts-lkfc_trigger_ns>=kLkfcPostNs){
          const auto dir=std::filesystem::path("/tmp")/
            ("monkeysstab_lk_forensic_"+std::to_string(++lkfc_seq));
          std::filesystem::create_directories(dir);
          std::ofstream meta(dir/"frames.csv");
          meta<<"seq,frame,mono_ns,v4l2_ns,dq_ns,v4l2_flags,jpeg\n";
          size_t n=0;
          for(const auto& q:lkfc_ring){
            if(q.mono_ns < lkfc_trigger_ns-kLkfcPreNs ||
               q.mono_ns > lkfc_trigger_ns+kLkfcPostNs) continue;
            const std::string name="frame_"+std::to_string(q.frame)+".jpg";
            std::ofstream jf(dir/name,std::ios::binary);
            jf.write(reinterpret_cast<const char*>(q.jpeg.data()),
                     static_cast<std::streamsize>(q.jpeg.size()));
            meta<<n++<<','<<q.frame<<','<<q.mono_ns<<','<<q.v4l2_ns<<','
                <<q.dq_ns<<','<<q.v4l2_flags<<','<<name<<"\n";
          }
          meta.flush();
          std::ofstream trig(dir/"trigger.txt");
          trig<<"trigger_frame="<<lkfc_trigger_frame<<"\n"
              <<"trigger_mono_ns="<<lkfc_trigger_ns<<"\n"
              <<"threshold_lk_ms="<<kLkfcTriggerMs<<"\n";
          trig.flush();
          std::cerr<<"LK_FORENSIC SAVED dir="<<dir.string()
                   <<" frames="<<n<<" trigger_frame="<<lkfc_trigger_frame<<"\n";
          lkfc_triggered=false;
          lkfc_ring.clear();
        }

        // WEB_PREVIEW_AB_V1: diagnostic-only A/B switch.
        // Default ON preserves the existing web contour. MONKEYS_WEB_PREVIEW=0
        // skips all preview JPEG work without changing live JSON telemetry or OF.
        {
          const char* e=std::getenv("MONKEYS_WEB_PREVIEW");
          const bool preview_enabled=
            !e || !*e ||
            !(std::string(e)=="0" || std::string(e)=="false" ||
              std::string(e)=="FALSE" || std::string(e)=="off" ||
              std::string(e)=="OFF");
          if(preview_enabled)
            web_live.sendPreview(now,gray,s.inlier_points,g_feature_roi);
        }

        // Metric odometry shadow. This path is diagnostic only: it consumes
        // the exact production RANSAC correspondences but never changes
        // flow_send_x/y and never calls sendOpticalFlow().
        static metric_shadow::Integrator metric_shadow_integrator;
        static uint64_t metric_shadow_interval_id=0;
        static int64_t metric_shadow_last_print_ns=0;
        metric_shadow::Step metric_step;
        metric_shadow::Step metric_gyro_step;
        metric_shadow::Step metric_highres_gyro_step;
        metric_shadow::Step metric_highres_corr_gyro_step;
        // STABILISED_FLOW_SHADOW_V1: AP FLOW_OPTIONS=Stabilised candidate.
        // This is the body-FRD angular translation flow reconstructed from the
        // corrected metric IMU displacement. It is LOGGING ONLY and is never
        // sent to the flight controller.
        bool stabilised_shadow_valid=false;
        double stabilised_shadow_flow_x=0.0;
        double stabilised_shadow_flow_y=0.0;
        double stabilised_shadow_h0_m=0.0;
        double stabilised_shadow_v_local_n=0.0;
        double stabilised_shadow_v_local_e=0.0;
        double stabilised_shadow_v_local_d=0.0;
        double stabilised_shadow_v_body_x=0.0;
        double stabilised_shadow_v_body_y=0.0;
        double stabilised_shadow_v_body_z=0.0;
        double stabilised_shadow_roundtrip_vx=0.0;
        double stabilised_shadow_roundtrip_vy=0.0;
        double stabilised_shadow_roundtrip_err=0.0;
        bool stabilised_sensor_shadow_valid=false;
        double stabilised_sensor_shadow_flow_x=0.0;
        double stabilised_sensor_shadow_flow_y=0.0;
        double stabilised_sensor_shadow_v_body_x=0.0;
        double stabilised_sensor_shadow_v_body_y=0.0;
        double stabilised_sensor_shadow_v_body_z=0.0;
        double stabilised_sensor_shadow_roundtrip_err=0.0;
        bool stabilised_lever_audit_valid=false;
        double stabilised_lever_observed_vx=0.0;
        double stabilised_lever_observed_vy=0.0;
        double stabilised_lever_pred_vx=0.0;
        double stabilised_lever_pred_vy=0.0;
        double stabilised_lever_err_mps=0.0;
        // Unified SENSOR-centric Stabilised candidate. Prefer corrected
        // HIGHRES delta-R; fall back to ATTITUDE/AHRS body rates while keeping
        // identical stabilised-flow semantics. Shadow only.
        bool stabilised_unified_shadow_valid=false;
        int stabilised_unified_shadow_source=0; // 0=none, 1=HIGHRES_CORR, 2=ATTITUDE_RATE
        double stabilised_unified_shadow_flow_x=0.0;
        double stabilised_unified_shadow_flow_y=0.0;
        double stabilised_unified_shadow_roundtrip_err=0.0;
        // Strict causal35 SENSOR-centric candidate promoted to Variant B only
        // when --stabilised-unified-publish is explicitly enabled.
        bool causal35_publish_valid=false;
        double causal35_publish_flow_x=0.0;
        double causal35_publish_flow_y=0.0;
        // RAW_OF_CONTRACT_V1: same causal35 translational measurement, with
        // rotation over the exact camera interval restored for FLOW_OPTIONS=0.
        bool causal35_raw_publish_valid=false;
        double causal35_raw_publish_flow_x=0.0;
        double causal35_raw_publish_flow_y=0.0;
        double causal35_raw_gyro_x=0.0;
        double causal35_raw_gyro_y=0.0;
        double causal35_optical_depth_m=0.0;
        // CAUSAL35_REJECT_DIAG_V1: diagnostic only; never changes publication.
        // 0=accepted/not-attempted, 1=no anchor, 2=anchor age,
        // 3=anchor->t1 HIGHRES, 4=d01 HIGHRES, 5=metric,
        // 6=range/extrinsics, 7=depth, 8=translation magnitude, 9=RAW magnitude.
        int causal35_reject_reason=0;
        double causal35_anchor_recv_age_diag_ms=-1.0;
        double causal35_anchor_sample_age_diag_ms=-1.0;
        double causal35_deltar_hold_diag_ms=-1.0;
        // PIXEL_ROTATION_SHADOW_V1: diagnostic only. Compare measured LK px1
        // with px1 predicted from px0 by HIGHRES delta-R. No range, lever arm,
        // ground-plane reconstruction, EKF, or production flow is involved.
        bool pixel_rot_valid=false;
        int pixel_rot_points=0;
        double pixel_rot_median_px=0.0;
        double pixel_rot_p95_px=0.0;
        double pixel_rot_du_median_px=0.0;
        double pixel_rot_dv_median_px=0.0;
        bool pixel_rot_direct_valid=false;
        double pixel_rot_direct_median_px=0.0;
        double pixel_rot_direct_du_median_px=0.0;
        double pixel_rot_direct_dv_median_px=0.0;
        bool pixel_rot_att_valid=false;
        double pixel_rot_att_median_px=0.0;
        double pixel_rot_att_du_median_px=0.0;
        double pixel_rot_att_dv_median_px=0.0;
        static constexpr int kPixelExtrAxisN=3;
        static constexpr int kPixelExtrOffN=8;
        static constexpr int kPixelExtrOffDeg[kPixelExtrOffN]={-5,-3,-2,-1,1,2,3,5};
        std::array<std::array<double,kPixelExtrOffN>,kPixelExtrAxisN> pixel_extr_med{};
        std::array<std::array<double,kPixelExtrOffN>,kPixelExtrAxisN> pixel_extr_du{};
        std::array<std::array<double,kPixelExtrOffN>,kPixelExtrAxisN> pixel_extr_dv{};
        std::array<std::array<int,kPixelExtrOffN>,kPixelExtrAxisN> pixel_extr_valid{};
        bool pixel_field_valid=false;
        int pixel_field_points=0;
        double pixel_field_affine_rms_px=0.0;
        double pixel_field_const_rms_px=0.0;
        double pixel_field_a00=0.0,pixel_field_a01=0.0,pixel_field_a10=0.0,pixel_field_a11=0.0;
        double pixel_field_bu=0.0,pixel_field_bv=0.0;
        metric_shadow::AttitudeLookup metric_a0{}, metric_a1{};
        metric_shadow::BodyRateIntegration metric_gyro_delta{};
        metric_shadow::BodyRateIntegration metric_highres_gyro_delta{};
        metric_shadow::BodyRateIntegration metric_highres_corr_gyro_delta{};
        // HIGHRES_PHASE_SWEEP_V1: diagnostic-only camera/gyro phase sweep.
        // Offsets shift the HIGHRES integration window in RPi CLOCK_MONOTONIC.
        // Production WORKED5 / OPTICAL_FLOW paths are untouched.
        constexpr int kHighresPhaseN=6;
        constexpr int kHighresPhaseOffsetMs[kHighresPhaseN]={-5,0,3,5,7,10};
        std::array<metric_shadow::BodyRateIntegration,kHighresPhaseN> metric_highres_phase_delta{};
        std::array<metric_shadow::Step,kHighresPhaseN> metric_highres_phase_step{};
        bool metric_attempted=false;
        // HIGHRES_PHASE_SWEEP_V2: positive offsets need future gyro samples.
        // Keep complete metric inputs for ~30 ms and evaluate them later against
        // the then-current HIGHRES history. Separate CSV avoids mixing causal
        // availability with the actual phase comparison.
        struct HighresPhasePending {
          uint64_t frame=0;
          int64_t t0_ns=0,t1_ns=0;
          metric_shadow::Input input{};
          cv::Matx33d R0=cv::Matx33d::eye();
        };
        static std::deque<HighresPhasePending> highres_phase_pending;
        static std::ofstream highres_phase_csv;
        static bool highres_phase_header=false;
        // HIGHRES_CAUSAL15_SHADOW_V1: diagnostic-only bounded causal hold.
        // Never feeds Variant B publication until its coverage/error is audited.
        static std::ofstream highres_causal15_csv;
        static bool highres_causal15_header=false;
        double metric_att_gap0_ms=-1.0,metric_att_gap1_ms=-1.0;
        double metric_range_gap0_ms=-1.0,metric_range_gap1_ms=-1.0;
        if(!prev.empty() && prev_ts>0 && ts>prev_ts){
          metric_attempted=true;
          ++metric_shadow_interval_id;

          std::deque<metric_shadow::TimedAttitude> ah;
          FlowFcGyro causal_att_anchor{};
          bool causal_att_anchor_valid=false;
          std::deque<metric_shadow::TimedBodyRate> gh;
          std::deque<metric_shadow::TimedBodyRate> hgh;
          std::deque<metric_shadow::TimedBodyRate> hgh_corr;
          // Strict camera-dequeue-causal HIGHRES stream for CAUSAL_METRIC35.
          // Keep the existing hgh/hgh_corr snapshots unchanged because legacy
          // diagnostics intentionally use the complete history available later.
          std::deque<metric_shadow::TimedBodyRate> hgh_corr_causal;
          {
            std::lock_guard<std::mutex> lock(fc.mu);
            ah.clear();
            gh.clear();
            hgh.clear();
            hgh_corr.clear();
            hgh_corr_causal.clear();
            ah.resize(fc.attitude_history.size());
            gh.resize(fc.attitude_history.size());
            for(size_t i=0;i<fc.attitude_history.size();++i){
              const auto& g=fc.attitude_history[i];
              ah[i]={g.roll,g.pitch,g.yaw,g.sample_ns,g.valid};
              gh[i]={g.x,g.y,g.z,g.sample_ns,g.valid};
              // Strict causal absolute anchor: packet was already received by
              // camera dequeue and its mapped FC sample is not newer than t1.
              if(g.valid && g.recv_ns<=selected_dq_mono_ns &&
                 g.mapped_sample_ns>0 && g.mapped_sample_ns<=ts){
                if(!causal_att_anchor_valid ||
                   g.mapped_sample_ns>causal_att_anchor.mapped_sample_ns){
                  causal_att_anchor=g;
                  causal_att_anchor_valid=true;
                }
              }
            }
            hgh.resize(fc.highres_gyro_history.size());
            hgh_corr.resize(fc.highres_gyro_history.size());
            for(size_t i=0;i<fc.highres_gyro_history.size();++i){
              const auto& g=fc.highres_gyro_history[i];
              // HIGHRES_CLOCK_MAP_V2: affine FC measurement time -> camera
              // CLOCK_MONOTONIC, using the causal one-second lower-envelope fit.
              const int64_t mapped_ns=fc.highres_clock_valid
                ? fc.mapHighresFcToMono(static_cast<int64_t>(g.fc_time_usec)*1000LL)
                : g.recv_ns;
              hgh[i]={g.x,g.y,g.z,mapped_ns,g.valid};
              hgh_corr[i]={g.x+g.drift_x,g.y+g.drift_y,g.z+g.drift_z,
                           mapped_ns,g.valid && g.drift_valid};
              // CAUSAL_METRIC35 may only consume IMU packets that had already
              // reached the RPi when this camera frame was dequeued.
              if(g.recv_ns<=selected_dq_mono_ns){
                hgh_corr_causal.push_back(
                  {g.x+g.drift_x,g.y+g.drift_y,g.z+g.drift_z,
                   mapped_ns,g.valid && g.drift_valid});
              }
            }
          }
          // HIGHRES_PHASE_SWEEP_V2 delayed evaluation. A +10 ms test
          // cannot be evaluated causally at t1 because those gyro samples do
          // not exist yet. Wait 30 ms, then evaluate every offset on the same
          // stored camera correspondences and geometry.
          if(diagnosticFileIoEnabled() && !highres_phase_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            highres_phase_csv.open(
              production_csv_path.parent_path()/"highres_phase_sweep_v2.csv",
              std::ios::out|std::ios::trunc);
          }
          if(diagnosticFileIoEnabled() && highres_phase_csv.is_open() && !highres_phase_header){
            highres_phase_csv<<"frame,t0_ns,t1_ns";
            for(int pi=0;pi<kHighresPhaseN;++pi){
              highres_phase_csv<<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_valid"
                <<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_angle_deg"
                <<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_imu_dN_m"
                <<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_imu_dE_m"
                <<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_residual_median_m";
            }
            highres_phase_csv<<'\n';
            highres_phase_header=true;
          }
          while(!highres_phase_pending.empty() &&
                ts-highres_phase_pending.front().t1_ns>=30000000LL){
            const auto q=highres_phase_pending.front();
            highres_phase_pending.pop_front();
            if(diagnosticFileIoEnabled() && highres_phase_csv.is_open()){
              highres_phase_csv<<q.frame<<','<<q.t0_ns<<','<<q.t1_ns;
              for(int pi=0;pi<kHighresPhaseN;++pi){
                const int64_t off_ns=
                  static_cast<int64_t>(kHighresPhaseOffsetMs[pi])*1000000LL;
                const auto pd=metric_shadow::integrateBodyRates(
                  hgh,q.t0_ns+off_ns,q.t1_ns+off_ns,30.0);
                metric_shadow::Step ps{};
                if(pd.valid){
                  const cv::Matx33d R1=q.R0*pd.delta_R;
                  ps=metric_shadow::estimateWithRotations(q.input,q.R0,R1);
                }
                highres_phase_csv<<','<<(ps.valid?1:0)
                  <<','<<pd.integrated_angle_deg
                  <<','<<ps.delta_local_m[0]
                  <<','<<ps.delta_local_m[1]
                  <<','<<ps.residual_median_m;
              }
              highres_phase_csv<<'\n';
              if(shadowFlushEnabled()) highres_phase_csv.flush();
            }
          }

          const auto a0=metric_shadow::interpolateAttitude(ah,prev_ts,30.0);
          const auto a1=metric_shadow::interpolateAttitude(ah,ts,30.0);
          metric_a0=a0;
          metric_a1=a1;
          metric_att_gap0_ms=a0.bracket_gap_ms;
          metric_att_gap1_ms=a1.bracket_gap_ms;

          const auto rh=luna.historySnapshot();
          const auto r0=metric_shadow_sync::interpolateRange(rh,prev_ts,40.0,25.0);
          const auto r1=metric_shadow_sync::interpolateRange(rh,ts,40.0,25.0);
          metric_range_gap0_ms=r0.bracket_gap_ms;
          metric_range_gap1_ms=r1.bracket_gap_ms;

          metric_shadow::Input mi;
          mi.t0_ns=prev_ts; mi.t1_ns=ts;
          mi.px0=s.metric_prev_points; mi.px1=s.metric_curr_points;
          // Variant-B metric geometry must use the same independently validated
          // focal scale as frozen WORKED5. calib.K is already scaled by the
          // production focal_scale (normally 0.931), so undo that scale and
          // apply WORKED5's frozen 1.10 without changing the production A path.
          mi.K=calib.K.clone();
          if(focal_scale>0.0 && std::isfinite(focal_scale)){
            const double metric_k=worked5::kFocalScale/focal_scale;
            mi.K.at<double>(0,0)*=metric_k;
            mi.K.at<double>(1,1)*=metric_k;
          }
          mi.D=calib.D;
          if(a0.valid) mi.a0=a0.attitude;
          if(a1.valid) mi.a1=a1.attitude;
          mi.range0_m=r0.distance_m; mi.range1_m=r1.distance_m;
          mi.range0_valid=r0.valid; mi.range1_valid=r1.valid;
          const cv::Matx33d metric_FLU_TO_FRD(1,0,0, 0,-1,0, 0,0,-1);
          mi.body_R_camera_frd=metric_FLU_TO_FRD*calib.B_R_C;
          mi.body_R_camera_valid=true;
          mi.camera_pos_body_frd=cv::Vec3d(diag_camera_x_m,diag_camera_y_m,diag_camera_z_m);
          mi.range_pos_body_frd=cv::Vec3d(0.0855,0.0,diag_range_z_m);

          metric_step=metric_shadow::estimate(mi);

          if(a0.valid){
            HighresPhasePending pq;
            pq.frame=frame;
            pq.t0_ns=prev_ts;
            pq.t1_ns=ts;
            pq.input=mi;
            pq.R0=metric_shadow::bodyToLocal(
              a0.attitude.roll,a0.attitude.pitch,a0.attitude.yaw);
            highres_phase_pending.push_back(std::move(pq));
            // Bound diagnostic memory even if camera timestamps stop advancing.
            while(highres_phase_pending.size()>16) highres_phase_pending.pop_front();
          }

          // DELTAR_GYRO_SHADOW_V1: use the same absolute R0 only as the local
          // frame anchor, but obtain the inter-frame rotation from integrated
          // body rates instead of the second ATTITUDE Euler endpoint.
          metric_gyro_delta=metric_shadow::integrateBodyRates(
            gh,prev_ts,ts,30.0);
          if(a0.valid && metric_gyro_delta.valid){
            const cv::Matx33d gyro_R0=metric_shadow::bodyToLocal(
              a0.attitude.roll,a0.attitude.pitch,a0.attitude.yaw);
            const cv::Matx33d gyro_R1=gyro_R0*metric_gyro_delta.delta_R;
            metric_gyro_step=metric_shadow::estimateWithRotations(
              mi,gyro_R0,gyro_R1);
          }

          // HIGHRES_DELTAR_SHADOW_V2: same geometry and same absolute R0,
          // but inter-frame delta-R comes from independent HIGHRES_IMU gyro.
          // FC time_usec is mapped to camera CLOCK_MONOTONIC by the causal
          // affine lower-envelope clock fit above; receive-time jitter and
          // measured FC/RPi clock-rate drift are excluded.
          metric_highres_gyro_delta=metric_shadow::integrateBodyRates(
            hgh,prev_ts,ts,30.0);
          if(a0.valid && metric_highres_gyro_delta.valid){
            const cv::Matx33d raw_R0=metric_shadow::bodyToLocal(
              a0.attitude.roll,a0.attitude.pitch,a0.attitude.yaw);
            const cv::Matx33d raw_R1=raw_R0*metric_highres_gyro_delta.delta_R;
            metric_highres_gyro_step=metric_shadow::estimateWithRotations(
              mi,raw_R0,raw_R1);
          }

          // HIGHRES_CORRECTED_SHADOW_V1: retain HIGHRES measurement timestamps
          // and affine FC->RPi mapping, but apply ArduPilot AHRS omegaI drift
          // correction captured with each HIGHRES sample. Diagnostic only.
          metric_highres_corr_gyro_delta=metric_shadow::integrateBodyRates(
            hgh_corr,prev_ts,ts,30.0);
          if(a0.valid && metric_highres_corr_gyro_delta.valid){
            const cv::Matx33d corr_R0=metric_shadow::bodyToLocal(
              a0.attitude.roll,a0.attitude.pitch,a0.attitude.yaw);
            const cv::Matx33d corr_R1=corr_R0*metric_highres_corr_gyro_delta.delta_R;
            metric_highres_corr_gyro_step=metric_shadow::estimateWithRotations(
              mi,corr_R0,corr_R1);
          }

          // CAUSAL_METRIC35_SHADOW_V1: full SENSOR-centric metric replay
          // using only information available by camera dequeue. Absolute R1
          // comes from the latest received ATTITUDE anchor propagated with
          // corrected HIGHRES; R0 is recovered from the same causal inter-frame
          // delta-R. Shadow only: never feeds Variant-B publication.
          static std::ofstream causal_metric35_csv;
          static bool causal_metric35_header=false;
          metric_shadow::Step causal_metric35_step{};
          variant_b_angular_shadow::Step angular_b_shadow{};
          bool causal_metric35_ready=false;
          double causal_metric35_anchor_recv_age_ms=-1.0;
          double causal_metric35_anchor_sample_age_ms=-1.0;
          double causal_metric35_deltar_hold_ms=-1.0;
          double causal_metric35_deltar_angle_deg=0.0;
          // CAUSAL35_REJECT_DIAG_V1: shadow-only explanation of why the
          // anchor->t1 causal HIGHRES integration (reason=3) was unavailable.
          // This never changes the production accept/reject decision.
          int causal35_r3_diag=0;
          double causal35_r3_start_hold_ms=-1.0;
          double causal35_r3_end_hold_ms=-1.0;
          uint64_t causal35_r3_samples=0;
          if(!causal_att_anchor_valid) causal35_reject_reason=1;
          if(causal_att_anchor_valid){
            causal_metric35_anchor_recv_age_ms=
              (selected_dq_mono_ns-causal_att_anchor.recv_ns)*1e-6;
            causal_metric35_anchor_sample_age_ms=
              (ts-causal_att_anchor.mapped_sample_ns)*1e-6;
            causal35_anchor_recv_age_diag_ms=causal_metric35_anchor_recv_age_ms;
            causal35_anchor_sample_age_diag_ms=causal_metric35_anchor_sample_age_ms;
            if(causal_metric35_anchor_recv_age_ms>=0.0 &&
               causal_metric35_anchor_recv_age_ms<=35.0 &&
               causal_metric35_anchor_sample_age_ms>=0.0){
              const auto anchor_to_t1=
                metric_shadow::integrateBodyRatesCausalHold(
                  hgh_corr_causal,causal_att_anchor.mapped_sample_ns,ts,25.0);
              const auto d01=metric_shadow::integrateBodyRatesCausalHold(
                hgh_corr_causal,prev_ts,ts,25.0);
              causal_metric35_deltar_hold_ms=d01.max_bracket_gap_ms;
              causal35_deltar_hold_diag_ms=causal_metric35_deltar_hold_ms;
              causal_metric35_deltar_angle_deg=d01.integrated_angle_deg;
              if(!anchor_to_t1.valid){
                causal35_reject_reason=3;

                // Mirror only the preconditions of integrateBodyRatesCausalHold
                // so reason=3 can be split without altering that integrator.
                // Codes: 1=no causal HIGHRES history, 2=bad interval,
                // 3=no sample at/before t0, 4=invalid start sample,
                // 5=start hold >25 ms, 6=bad/non-monotonic internal segment,
                // 7=end hold >25 ms, 8=no integrated segment, 9=other.
                causal35_r3_samples=hgh_corr_causal.size();
                const int64_t r3_t0=causal_att_anchor.mapped_sample_ns;
                const int64_t r3_t1=ts;
                if(hgh_corr_causal.empty()){
                  causal35_r3_diag=1;
                } else if(r3_t0<=0 || r3_t1<=r3_t0){
                  causal35_r3_diag=2;
                } else {
                  auto r3_first=std::upper_bound(
                    hgh_corr_causal.begin(),hgh_corr_causal.end(),r3_t0,
                    [](int64_t t,const metric_shadow::TimedBodyRate& a){
                      return t<a.sample_ns;
                    });
                  if(r3_first==hgh_corr_causal.begin()){
                    causal35_r3_diag=3;
                  } else {
                    auto r3_cur=std::prev(r3_first);
                    if(!r3_cur->valid || r3_cur->sample_ns<=0){
                      causal35_r3_diag=4;
                    } else {
                      causal35_r3_start_hold_ms=
                        (r3_t0-r3_cur->sample_ns)*1e-6;
                      if(!(causal35_r3_start_hold_ms>=0.0 &&
                           causal35_r3_start_hold_ms<=25.0)){
                        causal35_r3_diag=5;
                      } else {
                        int64_t r3_seg_start=r3_t0;
                        int64_t r3_rate_sample_ns=r3_cur->sample_ns;
                        bool r3_bad_segment=false;
                        int r3_segments=0;
                        for(auto it=r3_first;
                            it!=hgh_corr_causal.end() && it->sample_ns<r3_t1;
                            ++it){
                          if(!it->valid || it->sample_ns<=r3_seg_start) continue;
                          const double r3_dt=(it->sample_ns-r3_seg_start)*1e-9;
                          if(!(r3_dt>0.0 && r3_dt<0.1)){
                            r3_bad_segment=true;
                            break;
                          }
                          ++r3_segments;
                          r3_rate_sample_ns=it->sample_ns;
                          r3_seg_start=it->sample_ns;
                        }
                        if(r3_bad_segment){
                          causal35_r3_diag=6;
                        } else {
                          causal35_r3_end_hold_ms=
                            (r3_t1-r3_rate_sample_ns)*1e-6;
                          if(!(causal35_r3_end_hold_ms>=0.0 &&
                               causal35_r3_end_hold_ms<=25.0)){
                            causal35_r3_diag=7;
                          } else {
                            const double r3_tail_dt=(r3_t1-r3_seg_start)*1e-9;
                            if(r3_tail_dt>0.0){
                              if(!(r3_tail_dt<0.1)) causal35_r3_diag=6;
                              else ++r3_segments;
                            }
                            if(causal35_r3_diag==0 && r3_segments<=0)
                              causal35_r3_diag=8;
                            if(causal35_r3_diag==0)
                              causal35_r3_diag=9;
                          }
                        }
                      }
                    }
                  }
                }
              } else if(!d01.valid) causal35_reject_reason=4;
              if(anchor_to_t1.valid && d01.valid){
                const cv::Matx33d Ra=metric_shadow::bodyToLocal(
                  causal_att_anchor.roll,causal_att_anchor.pitch,causal_att_anchor.yaw);
                const cv::Matx33d R1c=Ra*anchor_to_t1.delta_R;
                const cv::Matx33d R0c=R1c*d01.delta_R.t();
                causal_metric35_step=
                  metric_shadow::estimateWithRotations(mi,R0c,R1c);
                causal_metric35_ready=causal_metric35_step.valid;
                if(!causal_metric35_ready) causal35_reject_reason=5;

                // VARIANT_B_ANGULAR_SHADOW_V1: range-independent candidate.
                // Diagnostic only: never writes causal35_publish_flow_*,
                // flow_send_*, WORKED5 state, or MAVLink.
                angular_b_shadow=variant_b_angular_shadow::estimate(
                  mi.px0,mi.px1,mi.K,mi.D,mi.body_R_camera_frd,
                  R0c,R1c,(ts-prev_ts)*1e-9);

                // Production-format Variant B candidate. Keep SENSOR-centric
                // camera displacement; EKF FLOW_POS handles the lever arm.
                if(causal_metric35_ready &&
                   (!mi.range0_valid || !mi.body_R_camera_valid ||
                    !(causal_metric35_step.dt>0.0)))
                  causal35_reject_reason=6;
                if(causal_metric35_ready && mi.range0_valid &&
                   mi.body_R_camera_valid && causal_metric35_step.dt>0.0){
                  cv::Vec3d lidar_ray_body=mi.range_ray_body_frd;
                  const double lrnorm=cv::norm(lidar_ray_body);
                  if(lrnorm>0.5 && std::isfinite(lrnorm)){
                    lidar_ray_body*=1.0/lrnorm;
                    const cv::Vec3d down(0,0,1);
                    const double h0=down.dot(R0c*(
                      mi.range_pos_body_frd-mi.camera_pos_body_frd+
                      mi.range0_m*lidar_ray_body));
                    const double optical_axis_down=R0c(2,2);
                    const double optical_depth=
                      (optical_axis_down>0.08)?(h0/optical_axis_down):0.0;
                    causal35_optical_depth_m=optical_depth;
                    if(!(h0>0.03 && std::isfinite(h0) &&
                         optical_depth>0.03 && std::isfinite(optical_depth)))
                      causal35_reject_reason=7;
                    if(h0>0.03 && std::isfinite(h0) &&
                       optical_depth>0.03 && std::isfinite(optical_depth)){
                      // ASTRA_DEPTH_FIX_V1: h0 is vertical camera-to-plane height.
                      // Synthetic central-ray angular flow requires optical-axis
                      // depth rho=h0/(local_down dot camera_optical_axis).
                      // This change intentionally does not alter FLOW_OPTIONS,
                      // FLOW_POS, WORKED5, or the SENSOR-centric reference point.
                      // STABILISED_SENSOR_CENTRIC_V1: FLOW_OPTIONS=1 tells ArduPilot
                      // that image roll/pitch rotation is already stabilised. Keep
                      // the measurement tied to the physical optical-flow sensor:
                      // publish the camera optical-centre displacement. FLOW_POS
                      // remains the sensor offset supplied separately to EKF3.
                      const cv::Vec3d sensor_v_local=
                        causal_metric35_step.delta_camera_local_m*
                        (1.0/causal_metric35_step.dt);
                      const cv::Vec3d sensor_v_body=R0c.t()*sensor_v_local;
                      const double fx=-sensor_v_body[1]/optical_depth;
                      const double fy= sensor_v_body[0]/optical_depth;
                      causal35_publish_valid=
                        std::isfinite(fx) && std::isfinite(fy) &&
                        std::hypot(fx,fy)<4.0;
                      if(!causal35_publish_valid) causal35_reject_reason=8;
                      if(causal35_publish_valid){
                        causal35_publish_flow_x=fx;
                        causal35_publish_flow_y=fy;

                        // d01.delta_R is composed from corrected HIGHRES body-FRD
                        // rates over exactly [prev_ts,ts]. log(dR)/dt therefore
                        // has the same x/y sign convention as the FC body gyro.
                        const cv::Matx33d& dR=d01.delta_R;
                        double cc=(dR(0,0)+dR(1,1)+dR(2,2)-1.0)*0.5;
                        cc=std::max(-1.0,std::min(1.0,cc));
                        const double th=std::acos(cc);
                        cv::Vec3d rv(0,0,0);
                        if(th<1e-7){
                          rv=cv::Vec3d(
                            0.5*(dR(2,1)-dR(1,2)),
                            0.5*(dR(0,2)-dR(2,0)),
                            0.5*(dR(1,0)-dR(0,1)));
                        } else if(th<3.0){
                          const double k=th/(2.0*std::sin(th));
                          rv=cv::Vec3d(
                            k*(dR(2,1)-dR(1,2)),
                            k*(dR(0,2)-dR(2,0)),
                            k*(dR(1,0)-dR(0,1)));
                        }
                        const double raw_wx=rv[0]/causal_metric35_step.dt;
                        const double raw_wy=rv[1]/causal_metric35_step.dt;
                        causal35_raw_gyro_x=raw_wx;
                        causal35_raw_gyro_y=raw_wy;
                        const double raw_fx=raw_wx+fx;
                        const double raw_fy=raw_wy+fy;
                        if(std::isfinite(raw_fx) && std::isfinite(raw_fy) &&
                           std::hypot(raw_fx,raw_fy)<4.0){
                          causal35_raw_publish_valid=true;
                          causal35_reject_reason=0;
                          causal35_raw_publish_flow_x=raw_fx;
                          causal35_raw_publish_flow_y=raw_fy;
                        } else {
                          causal35_reject_reason=9;
                        }
                      }
                    }
                  }
                }
              }
            } else {
              causal35_reject_reason=2;
            }
          }
          // VARIANT_B_ANGULAR_SHADOW_V1: keep OLD-B and NEW-B on the
          // same camera interval. Range is logged only as an explanatory input;
          // it is not consumed by angular_b_shadow.
          static std::ofstream angular_b_csv;
          static bool angular_b_header=false;
          if(diagnosticFileIoEnabled() && !angular_b_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            angular_b_csv.open(
              production_csv_path.parent_path()/"variant_b_angular_shadow.csv",
              std::ios::out|std::ios::trunc);
          }
          if(diagnosticFileIoEnabled() && angular_b_csv.is_open()){
            if(!angular_b_header){
              angular_b_csv
                <<"frame,t0_ns,t1_ns,dt_s,new_valid,new_points,"
                <<"new_du_norm,new_dv_norm,new_scale_rate,new_residual_rotation_rate,"
                <<"new_flow_x,new_flow_y,new_rms_norm,"
                <<"old_valid,old_flow_x,old_flow_y,"
                <<"range0_valid,range1_valid,range0_m,range1_m,delta_range_m\n";
              angular_b_header=true;
            }
            angular_b_csv
              <<frame<<','<<prev_ts<<','<<ts<<','<<((ts-prev_ts)*1e-9)<<','
              <<(angular_b_shadow.valid?1:0)<<','<<angular_b_shadow.points<<','
              <<angular_b_shadow.du_norm<<','<<angular_b_shadow.dv_norm<<','
              <<angular_b_shadow.scale_rate<<','<<angular_b_shadow.residual_rotation_rate<<','
              <<angular_b_shadow.flow_x<<','<<angular_b_shadow.flow_y<<','
              <<angular_b_shadow.rms_norm<<','
              <<(causal35_publish_valid?1:0)<<','
              <<causal35_publish_flow_x<<','<<causal35_publish_flow_y<<','
              <<(r0.valid?1:0)<<','<<(r1.valid?1:0)<<','
              <<r0.distance_m<<','<<r1.distance_m<<','
              <<(r1.distance_m-r0.distance_m)<<'\n';
            if(shadowFlushEnabled()) angular_b_csv.flush();
          }

          if(diagnosticFileIoEnabled() && !causal_metric35_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            causal_metric35_csv.open(
              production_csv_path.parent_path()/"causal_metric35_shadow.csv",
              std::ios::out|std::ios::trunc);
          }
          if(diagnosticFileIoEnabled() && causal_metric35_csv.is_open()){
            if(!causal_metric35_header){
              causal_metric35_csv
                <<"frame,t0_ns,t1_ns,ready,anchor_recv_age_ms,anchor_sample_age_ms,"
                <<"deltar_hold_ms,deltar_angle_deg,camera_dN_m,camera_dE_m,"
                <<"lever_dN_m,lever_dE_m,imu_dN_m,imu_dE_m,residual_median_m,"
                <<"reject_reason,r3_diag,r3_start_hold_ms,r3_end_hold_ms,r3_samples\n";
              causal_metric35_header=true;
            }
            causal_metric35_csv
              <<frame<<','<<prev_ts<<','<<ts<<','<<(causal_metric35_ready?1:0)<<','
              <<causal_metric35_anchor_recv_age_ms<<','
              <<causal_metric35_anchor_sample_age_ms<<','
              <<causal_metric35_deltar_hold_ms<<','
              <<causal_metric35_deltar_angle_deg<<','
              <<causal_metric35_step.delta_camera_local_m[0]<<','
              <<causal_metric35_step.delta_camera_local_m[1]<<','
              <<causal_metric35_step.lever_local_m[0]<<','
              <<causal_metric35_step.lever_local_m[1]<<','
              <<causal_metric35_step.delta_local_m[0]<<','
              <<causal_metric35_step.delta_local_m[1]<<','
              <<causal_metric35_step.residual_median_m<<','
              <<causal35_reject_reason<<','
              <<causal35_r3_diag<<','
              <<causal35_r3_start_hold_ms<<','
              <<causal35_r3_end_hold_ms<<','
              <<causal35_r3_samples<<'\n';
            if(shadowFlushEnabled()) causal_metric35_csv.flush();
          }

          // HIGHRES_CAUSAL15_SHADOW_V1: evaluate the same corrected HIGHRES
          // stream without requiring a future sample beyond the current camera
          // endpoint. Missing endpoint coverage is held for at most 15 ms.
          // This is shadow-only: production A, WORKED5 and Variant-B publish
          // selection remain unchanged.
          const auto metric_highres_causal15_delta=
            metric_shadow::integrateBodyRatesCausalHold(
              hgh_corr,prev_ts,ts,15.0);
          metric_shadow::Step metric_highres_causal15_step{};
          if(a0.valid && metric_highres_causal15_delta.valid){
            const cv::Matx33d causal_R0=metric_shadow::bodyToLocal(
              a0.attitude.roll,a0.attitude.pitch,a0.attitude.yaw);
            const cv::Matx33d causal_R1=
              causal_R0*metric_highres_causal15_delta.delta_R;
            metric_highres_causal15_step=metric_shadow::estimateWithRotations(
              mi,causal_R0,causal_R1);
          }

          if(diagnosticFileIoEnabled() && !highres_causal15_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            highres_causal15_csv.open(
              production_csv_path.parent_path()/"highres_causal15_shadow.csv",
              std::ios::out|std::ios::trunc);
          }
          if(diagnosticFileIoEnabled() && highres_causal15_csv.is_open() && !highres_causal15_header){
            highres_causal15_csv
              <<"frame,t0_ns,t1_ns,strict_valid,causal15_valid,"
              <<"causal15_max_hold_ms,strict_segments,causal15_segments,"
              <<"strict_angle_deg,causal15_angle_deg,deltaR_diff_deg,"
              <<"strict_sensor_dN_m,strict_sensor_dE_m,"
              <<"causal15_sensor_dN_m,causal15_sensor_dE_m,"
              <<"sensor_delta_diff_m\n";
            highres_causal15_header=true;
          }
          if(diagnosticFileIoEnabled() && highres_causal15_csv.is_open()){
            double dr_diff_deg=-1.0;
            if(metric_highres_corr_gyro_delta.valid &&
               metric_highres_causal15_delta.valid){
              dr_diff_deg=metric_shadow::rotationDistanceDeg(
                metric_highres_corr_gyro_delta.delta_R,
                metric_highres_causal15_delta.delta_R);
            }
            double sensor_delta_diff_m=-1.0;
            if(metric_highres_corr_gyro_step.valid &&
               metric_highres_causal15_step.valid){
              const cv::Vec3d dd=
                metric_highres_causal15_step.delta_camera_local_m-
                metric_highres_corr_gyro_step.delta_camera_local_m;
              sensor_delta_diff_m=cv::norm(dd);
            }
            highres_causal15_csv
              <<frame<<','<<prev_ts<<','<<ts
              <<','<<(metric_highres_corr_gyro_delta.valid?1:0)
              <<','<<(metric_highres_causal15_delta.valid?1:0)
              <<','<<metric_highres_causal15_delta.max_bracket_gap_ms
              <<','<<metric_highres_corr_gyro_delta.segments
              <<','<<metric_highres_causal15_delta.segments
              <<','<<metric_highres_corr_gyro_delta.integrated_angle_deg
              <<','<<metric_highres_causal15_delta.integrated_angle_deg
              <<','<<dr_diff_deg
              <<','<<metric_highres_corr_gyro_step.delta_camera_local_m[0]
              <<','<<metric_highres_corr_gyro_step.delta_camera_local_m[1]
              <<','<<metric_highres_causal15_step.delta_camera_local_m[0]
              <<','<<metric_highres_causal15_step.delta_camera_local_m[1]
              <<','<<sensor_delta_diff_m<<'\n';
          }

          // Convert the already rotation- and lever-arm-compensated FC/IMU
          // displacement back to AP body-FRD angular flow. For a downward
          // camera: flow_x ~= -v_body_y/h, flow_y ~= +v_body_x/h.
          // R0^T converts local velocity to body0 FRD. Use the same camera
          // height geometry as the metric estimator so this shadow has a
          // well-defined contract for a future FLOW_OPTIONS=Stabilised A/B.
          if(metric_highres_corr_gyro_step.valid && a0.valid &&
             mi.range0_valid && mi.body_R_camera_valid){
            const cv::Matx33d corr_R0=metric_shadow::bodyToLocal(
              a0.attitude.roll,a0.attitude.pitch,a0.attitude.yaw);
            cv::Vec3d lidar_ray_body=mi.range_ray_body_frd;
            const double lrnorm=cv::norm(lidar_ray_body);
            if(lrnorm>0.5 && std::isfinite(lrnorm)){
              lidar_ray_body*=1.0/lrnorm;
              const cv::Vec3d down(0,0,1);
              const double h0=down.dot(corr_R0*(
                mi.range_pos_body_frd-mi.camera_pos_body_frd+
                mi.range0_m*lidar_ray_body));
              if(h0>0.03 && std::isfinite(h0) &&
                 metric_highres_corr_gyro_step.dt>0.0){
                const cv::Vec3d v_local=
                  metric_highres_corr_gyro_step.delta_local_m*
                  (1.0/metric_highres_corr_gyro_step.dt);
                const cv::Vec3d v_body=corr_R0.t()*v_local;
                stabilised_shadow_h0_m=h0;
                stabilised_shadow_v_local_n=v_local[0];
                stabilised_shadow_v_local_e=v_local[1];
                stabilised_shadow_v_local_d=v_local[2];
                stabilised_shadow_v_body_x=v_body[0];
                stabilised_shadow_v_body_y=v_body[1];
                stabilised_shadow_v_body_z=v_body[2];
                stabilised_shadow_flow_x=-v_body[1]/h0;
                stabilised_shadow_flow_y= v_body[0]/h0;
                // Exact inverse of the proposed AP-stabilised flow contract.
                // This must recover body vx/vy algebraically; any non-zero
                // error means a sign/unit bug in this interface.
                stabilised_shadow_roundtrip_vx=
                  stabilised_shadow_flow_y*h0;
                stabilised_shadow_roundtrip_vy=
                 -stabilised_shadow_flow_x*h0;
                stabilised_shadow_roundtrip_err=std::hypot(
                  stabilised_shadow_roundtrip_vx-v_body[0],
                  stabilised_shadow_roundtrip_vy-v_body[1]);
                stabilised_shadow_valid=
                  std::isfinite(stabilised_shadow_flow_x) &&
                  std::isfinite(stabilised_shadow_flow_y) &&
                  std::isfinite(stabilised_shadow_roundtrip_err) &&
                  std::hypot(stabilised_shadow_flow_x,
                             stabilised_shadow_flow_y)<4.0;

                // SENSOR-centric Stabilised candidate: retain the optical
                // centre lever-arm displacement so EKF FLOW_POS can model it.
                const cv::Vec3d sensor_v_local=
                  metric_highres_corr_gyro_step.delta_camera_local_m*
                  (1.0/metric_highres_corr_gyro_step.dt);
                const cv::Vec3d sensor_v_body=corr_R0.t()*sensor_v_local;
                stabilised_sensor_shadow_v_body_x=sensor_v_body[0];
                stabilised_sensor_shadow_v_body_y=sensor_v_body[1];
                stabilised_sensor_shadow_v_body_z=sensor_v_body[2];
                stabilised_sensor_shadow_flow_x=-sensor_v_body[1]/h0;
                stabilised_sensor_shadow_flow_y= sensor_v_body[0]/h0;
                const double sensor_back_vx=
                  stabilised_sensor_shadow_flow_y*h0;
                const double sensor_back_vy=
                 -stabilised_sensor_shadow_flow_x*h0;
                stabilised_sensor_shadow_roundtrip_err=std::hypot(
                  sensor_back_vx-sensor_v_body[0],
                  sensor_back_vy-sensor_v_body[1]);
                stabilised_sensor_shadow_valid=
                  std::isfinite(stabilised_sensor_shadow_flow_x) &&
                  std::isfinite(stabilised_sensor_shadow_flow_y) &&
                  std::isfinite(stabilised_sensor_shadow_roundtrip_err) &&
                  std::hypot(stabilised_sensor_shadow_flow_x,
                             stabilised_sensor_shadow_flow_y)<4.0;

                // Lever-arm contract audit. SENSOR-centric minus IMU-centric
                // must equal the exact finite-rotation focal-point velocity
                // implied by the configured camera position. This is the
                // finite-dt counterpart of omega x FLOW_POS.
                const cv::Vec3d lever_v_local=
                  metric_highres_corr_gyro_step.lever_local_m*
                  (1.0/metric_highres_corr_gyro_step.dt);
                const cv::Vec3d lever_v_body=corr_R0.t()*lever_v_local;
                stabilised_lever_observed_vx=
                  sensor_v_body[0]-v_body[0];
                stabilised_lever_observed_vy=
                  sensor_v_body[1]-v_body[1];
                stabilised_lever_pred_vx=lever_v_body[0];
                stabilised_lever_pred_vy=lever_v_body[1];
                stabilised_lever_err_mps=std::hypot(
                  stabilised_lever_observed_vx-stabilised_lever_pred_vx,
                  stabilised_lever_observed_vy-stabilised_lever_pred_vy);
                stabilised_lever_audit_valid=
                  stabilised_shadow_valid &&
                  stabilised_sensor_shadow_valid &&
                  std::isfinite(stabilised_lever_err_mps);

                // Preferred unified path: corrected HIGHRES SENSOR-centric.
                if(stabilised_sensor_shadow_valid){
                  stabilised_unified_shadow_valid=true;
                  stabilised_unified_shadow_source=1;
                  stabilised_unified_shadow_flow_x=stabilised_sensor_shadow_flow_x;
                  stabilised_unified_shadow_flow_y=stabilised_sensor_shadow_flow_y;
                  stabilised_unified_shadow_roundtrip_err=
                    stabilised_sensor_shadow_roundtrip_err;
                }
              }
            }
          }

          // STABILISED_UNIFIED_SHADOW_V1 fallback. ATTITUDE roll/pitch/yaw
          // rates come from AP::ahrs().get_gyro(), i.e. the same corrected
          // AHRS angular-rate domain that already backs the ordinary delta-R
          // shadow. Keep SENSOR-centric semantics so FLOW_POS remains coherent.
          if(!stabilised_unified_shadow_valid &&
             metric_gyro_step.valid && a0.valid &&
             mi.range0_valid && mi.body_R_camera_valid &&
             metric_gyro_step.dt>0.0){
            const cv::Matx33d fb_R0=metric_shadow::bodyToLocal(
              a0.attitude.roll,a0.attitude.pitch,a0.attitude.yaw);
            cv::Vec3d lidar_ray_body=mi.range_ray_body_frd;
            const double lrnorm=cv::norm(lidar_ray_body);
            if(lrnorm>0.5 && std::isfinite(lrnorm)){
              lidar_ray_body*=1.0/lrnorm;
              const cv::Vec3d down(0,0,1);
              const double h0=down.dot(fb_R0*(
                mi.range_pos_body_frd-mi.camera_pos_body_frd+
                mi.range0_m*lidar_ray_body));
              if(h0>0.03 && std::isfinite(h0)){
                const cv::Vec3d sensor_v_local=
                  metric_gyro_step.delta_camera_local_m*(1.0/metric_gyro_step.dt);
                const cv::Vec3d sensor_v_body=fb_R0.t()*sensor_v_local;
                const double fx=-sensor_v_body[1]/h0;
                const double fy= sensor_v_body[0]/h0;
                const double back_vx=fy*h0;
                const double back_vy=-fx*h0;
                const double rt_err=std::hypot(
                  back_vx-sensor_v_body[0],back_vy-sensor_v_body[1]);
                if(std::isfinite(fx) && std::isfinite(fy) &&
                   std::isfinite(rt_err) && std::hypot(fx,fy)<4.0){
                  stabilised_unified_shadow_valid=true;
                  stabilised_unified_shadow_source=2;
                  stabilised_unified_shadow_flow_x=fx;
                  stabilised_unified_shadow_flow_y=fy;
                  stabilised_unified_shadow_roundtrip_err=rt_err;
                }
              }
            }
          }

          // PIXEL_ROTATION_SHADOW_V1. OpenCV undistorted normalized rays are
          // rotated C0 -> body0 -> body1 -> C1. For a stationary world point
          // and a pure camera rotation, c1 = C_R_B * dR^T * B_R_C * c0.
          if(metric_highres_gyro_delta.valid && !mi.K.empty() &&
             mi.px0.size()==mi.px1.size() && mi.px0.size()>=20){
            std::vector<cv::Point2f> uq0,uq1;
            cv::undistortPoints(mi.px0,uq0,mi.K,mi.D);
            cv::undistortPoints(mi.px1,uq1,mi.K,mi.D);
            const cv::Matx33d B_R_C=mi.body_R_camera_frd;
            const cv::Matx33d C_R_B=B_R_C.t();
            const cv::Matx33d C1_R_C0=
              C_R_B*metric_highres_gyro_delta.delta_R.t()*B_R_C;
            std::vector<double> er,edu,edv;
            er.reserve(uq0.size()); edu.reserve(uq0.size()); edv.reserve(uq0.size());
            const double fx=mi.K.at<double>(0,0), fy=mi.K.at<double>(1,1);
            for(size_t i=0;i<uq0.size();++i){
              const cv::Vec3d q=C1_R_C0*cv::Vec3d(uq0[i].x,uq0[i].y,1.0);
              if(!(q[2]>0.1) || !std::isfinite(q[2])) continue;
              const double px=q[0]/q[2], py=q[1]/q[2];
              const double du=(static_cast<double>(uq1[i].x)-px)*fx;
              const double dv=(static_cast<double>(uq1[i].y)-py)*fy;
              if(!std::isfinite(du)||!std::isfinite(dv)) continue;
              edu.push_back(du); edv.push_back(dv); er.push_back(std::hypot(du,dv));
            }
            if(er.size()>=20){
              auto med=[](std::vector<double> v){
                const size_t n=v.size(), k=n/2;
                std::nth_element(v.begin(),v.begin()+k,v.end());
                const double hi=v[k];
                if(n&1) return hi;
                std::nth_element(v.begin(),v.begin()+k-1,v.end());
                return 0.5*(v[k-1]+hi);
              };
              pixel_rot_valid=true;
              pixel_rot_points=static_cast<int>(er.size());
              pixel_rot_median_px=med(er);
              pixel_rot_du_median_px=med(edu);
              pixel_rot_dv_median_px=med(edv);
              const size_t k95=std::min(er.size()-1,
                static_cast<size_t>(std::floor(0.95*static_cast<double>(er.size()-1))));
              std::nth_element(er.begin(),er.begin()+k95,er.end());
              pixel_rot_p95_px=er[k95];
            }

            // PIXEL_RESIDUAL_FIELD_V1: fit residual flow as an affine field
            // [du,dv] = A*[x-cx,y-cy] + b. A constant field is translation-like
            // on a near-planar nadir scene; spatial gradients expose residual
            // rotation/projective structure. Diagnostic only.
            {
              cv::Mat M(static_cast<int>(uq0.size()),3,CV_64F);
              cv::Mat yu(static_cast<int>(uq0.size()),1,CV_64F);
              cv::Mat yv(static_cast<int>(uq0.size()),1,CV_64F);
              int n=0;
              const double cx=mi.K.at<double>(0,2), cy=mi.K.at<double>(1,2);
              for(size_t i=0;i<uq0.size();++i){
                const cv::Vec3d q=C1_R_C0*cv::Vec3d(uq0[i].x,uq0[i].y,1.0);
                if(!(q[2]>0.1) || !std::isfinite(q[2])) continue;
                const double du=(static_cast<double>(uq1[i].x)-q[0]/q[2])*fx;
                const double dv=(static_cast<double>(uq1[i].y)-q[1]/q[2])*fy;
                if(!std::isfinite(du)||!std::isfinite(dv)) continue;
                // uq coordinates are normalized; center them using normalized principal point = 0.
                M.at<double>(n,0)=uq0[i].x;
                M.at<double>(n,1)=uq0[i].y;
                M.at<double>(n,2)=1.0;
                yu.at<double>(n,0)=du;
                yv.at<double>(n,0)=dv;
                ++n;
              }
              (void)cx; (void)cy;
              if(n>=20){
                M=M.rowRange(0,n).clone(); yu=yu.rowRange(0,n).clone(); yv=yv.rowRange(0,n).clone();
                cv::Mat cu,cvv;
                if(cv::solve(M,yu,cu,cv::DECOMP_SVD) && cv::solve(M,yv,cvv,cv::DECOMP_SVD)){
                  pixel_field_valid=true; pixel_field_points=n;
                  pixel_field_a00=cu.at<double>(0); pixel_field_a01=cu.at<double>(1); pixel_field_bu=cu.at<double>(2);
                  pixel_field_a10=cvv.at<double>(0); pixel_field_a11=cvv.at<double>(1); pixel_field_bv=cvv.at<double>(2);
                  double sa=0.0,sc=0.0;
                  for(int j=0;j<n;++j){
                    const double pu=M.at<double>(j,0)*pixel_field_a00+M.at<double>(j,1)*pixel_field_a01+pixel_field_bu;
                    const double pv=M.at<double>(j,0)*pixel_field_a10+M.at<double>(j,1)*pixel_field_a11+pixel_field_bv;
                    const double eu=yu.at<double>(j)-pu, ev=yv.at<double>(j)-pv;
                    sa+=eu*eu+ev*ev;
                    const double ecu=yu.at<double>(j)-pixel_field_bu, ecv=yv.at<double>(j)-pixel_field_bv;
                    sc+=ecu*ecu+ecv*ecv;
                  }
                  pixel_field_affine_rms_px=std::sqrt(sa/n);
                  pixel_field_const_rms_px=std::sqrt(sc/n);
                }
              }
            }

            // Same correspondences, two controls:
            // 1) direct delta_R instead of delta_R^T (convention/sign check);
            // 2) ATTITUDE endpoint rotation, independent of gyro integration.
            auto evalPixelRotation=[&](const cv::Matx33d& C1_R_C0,
                                       bool& valid,double& mederr,
                                       double& meddu,double& meddv){
              std::vector<double> er2,du2,dv2;
              er2.reserve(uq0.size()); du2.reserve(uq0.size()); dv2.reserve(uq0.size());
              for(size_t i=0;i<uq0.size();++i){
                const cv::Vec3d q=C1_R_C0*cv::Vec3d(uq0[i].x,uq0[i].y,1.0);
                if(!(q[2]>0.1) || !std::isfinite(q[2])) continue;
                const double du=(static_cast<double>(uq1[i].x)-q[0]/q[2])*fx;
                const double dv=(static_cast<double>(uq1[i].y)-q[1]/q[2])*fy;
                if(!std::isfinite(du)||!std::isfinite(dv)) continue;
                du2.push_back(du); dv2.push_back(dv); er2.push_back(std::hypot(du,dv));
              }
              if(er2.size()>=20){
                auto med2=[](std::vector<double> v){
                  const size_t n=v.size(), k=n/2;
                  std::nth_element(v.begin(),v.begin()+k,v.end());
                  const double hi=v[k];
                  if(n&1) return hi;
                  std::nth_element(v.begin(),v.begin()+k-1,v.end());
                  return 0.5*(v[k-1]+hi);
                };
                valid=true; mederr=med2(er2); meddu=med2(du2); meddv=med2(dv2);
              }
            };

            evalPixelRotation(
              C_R_B*metric_highres_gyro_delta.delta_R*B_R_C,
              pixel_rot_direct_valid,pixel_rot_direct_median_px,
              pixel_rot_direct_du_median_px,pixel_rot_direct_dv_median_px);

            if(a0.valid && a1.valid){
              const cv::Matx33d A0=metric_shadow::bodyToLocal(
                a0.attitude.roll,a0.attitude.pitch,a0.attitude.yaw);
              const cv::Matx33d A1=metric_shadow::bodyToLocal(
                a1.attitude.roll,a1.attitude.pitch,a1.attitude.yaw);
              evalPixelRotation(
                C_R_B*A1.t()*A0*B_R_C,
                pixel_rot_att_valid,pixel_rot_att_median_px,
                pixel_rot_att_du_median_px,pixel_rot_att_dv_median_px);
            }

            // PIXEL_EXTRINSIC_SWEEP_V1: perturb only camera angular extrinsic.
            // Evaluate against the same HIGHRES delta-R and LK correspondences.
            // This is diagnostic-only and never changes production geometry.
            auto axisRot=[](int axis,double a){
              const double cs=std::cos(a), sn=std::sin(a);
              if(axis==0) return cv::Matx33d(1,0,0, 0,cs,-sn, 0,sn,cs);
              if(axis==1) return cv::Matx33d(cs,0,sn, 0,1,0, -sn,0,cs);
              return cv::Matx33d(cs,-sn,0, sn,cs,0, 0,0,1);
            };
            for(int ax=0;ax<kPixelExtrAxisN;++ax){
              for(int oi=0;oi<kPixelExtrOffN;++oi){
                const double a=kPixelExtrOffDeg[oi]*M_PI/180.0;
                const cv::Matx33d B_R_C_test=B_R_C*axisRot(ax,a);
                const cv::Matx33d C_R_B_test=B_R_C_test.t();
                bool vv=false; double mm=0.0,duv=0.0,dvv=0.0;
                evalPixelRotation(
                  C_R_B_test*metric_highres_gyro_delta.delta_R.t()*B_R_C_test,
                  vv,mm,duv,dvv);
                pixel_extr_valid[ax][oi]=vv?1:0;
                pixel_extr_med[ax][oi]=mm;
                pixel_extr_du[ax][oi]=duv;
                pixel_extr_dv[ax][oi]=dvv;
              }
            }
          }

          // HIGHRES_PHASE_SWEEP_V1: run the exact same ray/lever geometry at
          // several fixed phase offsets. This is logging-only A/B/C... data.
          if(a0.valid){
            const cv::Matx33d phase_R0=metric_shadow::bodyToLocal(
              a0.attitude.roll,a0.attitude.pitch,a0.attitude.yaw);
            for(int pi=0;pi<kHighresPhaseN;++pi){
              const int64_t off_ns=static_cast<int64_t>(kHighresPhaseOffsetMs[pi])*1000000LL;
              metric_highres_phase_delta[pi]=metric_shadow::integrateBodyRates(
                hgh,prev_ts+off_ns,ts+off_ns,30.0);
              if(metric_highres_phase_delta[pi].valid){
                const cv::Matx33d phase_R1=
                  phase_R0*metric_highres_phase_delta[pi].delta_R;
                metric_highres_phase_step[pi]=metric_shadow::estimateWithRotations(
                  mi,phase_R0,phase_R1);
              }
            }
          }

          // Startup before the first synchronized metric interval is not a GAP.
          // After start, every rejected interval remains visible.
          static bool metric_shadow_started=false;
          const bool metric_shadow_startup_wait =
              !metric_shadow_started && !metric_step.valid &&
              (metric_step.reason==metric_shadow::RejectReason::BAD_ATTITUDE ||
               metric_step.reason==metric_shadow::RejectReason::BAD_RANGE);

          if(metric_step.valid) metric_shadow_started=true;
          if(!metric_shadow_startup_wait)
            metric_shadow_integrator.consume(metric_step);

          // Per-interval Metric Shadow log. Production OPTICAL_FLOW is untouched.
          static std::ofstream metric_shadow_csv;
          static bool metric_shadow_csv_header=false;

          if(diagnosticFileIoEnabled() && !metric_shadow_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            const auto metric_shadow_path =
                production_csv_path.parent_path() / "metric_shadow.csv";
            metric_shadow_csv.open(
                metric_shadow_path,
                std::ios::out | std::ios::trunc);
          }

          if(diagnosticFileIoEnabled() && metric_shadow_csv.is_open()){
            if(!metric_shadow_csv_header){
              metric_shadow_csv
                <<"interval_id,t0_ns,t1_ns,started,startup_wait,"
                <<"valid,reason,pairs,used,"
                <<"dN_m,dE_m,dD_m,pN_m,pE_m,pD_m,"
                <<"accepted,rejected,complete,"
                <<"att0_valid,att1_valid,att0_gap_ms,att1_gap_ms,"
                <<"range0_valid,range1_valid,range0_m,range1_m,"
                <<"range0_gap_ms,range1_gap_ms,residual_median_m\n";
              metric_shadow_csv_header=true;
            }

            const auto& mp=metric_shadow_integrator.position_m;

            metric_shadow_csv
              <<metric_shadow_interval_id<<','
              <<prev_ts<<','<<ts<<','
              <<(metric_shadow_started?1:0)<<','
              <<(metric_shadow_startup_wait?1:0)<<','
              <<(metric_step.valid?1:0)<<','
              <<metric_shadow::rejectReasonName(metric_step.reason)<<','
              <<s.metric_prev_points.size()<<','
              <<metric_step.points<<','
              <<metric_step.delta_local_m[0]<<','
              <<metric_step.delta_local_m[1]<<','
              <<metric_step.delta_local_m[2]<<','
              <<mp[0]<<','<<mp[1]<<','<<mp[2]<<','
              <<metric_shadow_integrator.accepted<<','
              <<metric_shadow_integrator.rejected<<','
              <<(metric_shadow_integrator.complete?1:0)<<','
              <<(a0.valid?1:0)<<','<<(a1.valid?1:0)<<','
              <<metric_att_gap0_ms<<','<<metric_att_gap1_ms<<','
              <<(r0.valid?1:0)<<','<<(r1.valid?1:0)<<','
              <<r0.distance_m<<','<<r1.distance_m<<','
              <<metric_range_gap0_ms<<','<<metric_range_gap1_ms<<','
              <<metric_step.residual_median_m
              <<'\n';

            if(shadowFlushEnabled()) metric_shadow_csv.flush();
          }

          if(diagnosticFileIoEnabled() &&
             (metric_shadow_last_print_ns==0 || now-metric_shadow_last_print_ns>=500000000LL)){
            metric_shadow_last_print_ns=now;
            const auto& mp=metric_shadow_integrator.position_m;
            std::cerr<<"METRIC_SHADOW interval="<<metric_shadow_interval_id
                     <<" valid="<<(metric_step.valid?1:0)
                     <<" reason="<<metric_shadow::rejectReasonName(metric_step.reason)
                     <<" pairs="<<s.metric_prev_points.size()
                     <<" used="<<metric_step.points
                     <<" dNE_mm=["<<metric_step.delta_local_m[0]*1000.0
                     <<","<<metric_step.delta_local_m[1]*1000.0<<"]"
                     <<" posNE_mm=["<<mp[0]*1000.0<<","<<mp[1]*1000.0<<"]"
                     <<" complete="<<(metric_shadow_integrator.complete?1:0)
                     <<" accepted="<<metric_shadow_integrator.accepted
                     <<" rejected="<<metric_shadow_integrator.rejected
                     <<" att_gap_ms=["<<metric_att_gap0_ms<<","<<metric_att_gap1_ms<<"]"
                     <<" range_gap_ms=["<<metric_range_gap0_ms<<","<<metric_range_gap1_ms<<"]"
                     <<" residual_med_mm="<<metric_step.residual_median_m*1000.0
                     <<"\n";
          }
        }

        // Consume the FC gyro for THIS processed camera interval before any
        // bench-only range remapping.  Pure rotational optical flow must remain
        // unscaled so ArduPilot can cancel it with bodyRate X/Y.
        FlowFcGyro fg{}; double fg_age=1e9; uint64_t fg_samples=0;
        const bool fg_ok=fc.consumeGyroAverage(&fg,&fg_age,&fg_samples);

        // WORKED5_REASON5_SHADOW_V1
        // Diagnostic C arms only. Production validity and frozen WORKED5 are untouched.
        // Re-fit the exact production RANSAC inliers on reason5 frames with lower
        // diagnostic-only point-count floors: 15, 10, 7.
        {
          static std::ofstream r5_csv;
          static bool r5_header=false;
          static double c15_n=0.0,c15_e=0.0,c10_n=0.0,c10_e=0.0,c7_n=0.0,c7_e=0.0;
          if(diagnosticFileIoEnabled() && !r5_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            r5_csv.open(production_csv_path.parent_path()/"worked5_reason5_shadow.csv",
                        std::ios::out|std::ios::trunc);
          }
          if(diagnosticFileIoEnabled() && r5_csv.is_open() && !r5_header){
            r5_csv<<"frame,mono_ns,production_valid,invalid_reason,pairs,dt_s,hcam_m,"
                     "c15_valid,c15_dN_m,c15_dE_m,c15_pN_m,c15_pE_m,"
                     "c10_valid,c10_dN_m,c10_dE_m,c10_pN_m,c10_pE_m,"
                     "c7_valid,c7_dN_m,c7_dE_m,c7_pN_m,c7_pE_m\n";
            r5_header=true;
          }

          double h=0.0;
          if(bench_true_camera_height>0.0) h=bench_true_camera_height;
          else if(current_camera_height_valid) h=current_camera_height_m;
          const int np=(int)std::min(s.metric_prev_points.size(),s.metric_curr_points.size());

          auto fit=[&](int minpts, double& dN, double& dE)->bool{
            dN=dE=0.0;
            if(s.invalid_reason!=5 || !fg_ok || np<minpts ||
               !(dt>0.0 && dt<0.2) || !(h>0.02) || !std::isfinite(h)) return false;

            cv::Mat K=calib.K.clone();
            const double k=worked5::kFocalScale/focal_scale;
            K.at<double>(0,0)*=k; K.at<double>(1,1)*=k;
            std::vector<cv::Point2f> a,b;
            cv::undistortPoints(s.metric_prev_points,a,K,calib.D);
            cv::undistortPoints(s.metric_curr_points,b,K,calib.D);
            if(a.size()!=b.size() || (int)a.size()<minpts) return false;

            cv::Mat A((int)a.size()*2,4,CV_64F), rhs((int)a.size()*2,1,CV_64F);
            for(size_t i=0;i<a.size();++i){
              const double x=a[i].x,y=a[i].y;
              const double du=b[i].x-a[i].x,dv=b[i].y-a[i].y;
              const int r=(int)(2*i);
              A.at<double>(r,0)=1.0; A.at<double>(r,1)=0.0;
              A.at<double>(r,2)=x;   A.at<double>(r,3)=-y;
              rhs.at<double>(r,0)=du;
              A.at<double>(r+1,0)=0.0; A.at<double>(r+1,1)=1.0;
              A.at<double>(r+1,2)=y;   A.at<double>(r+1,3)=x;
              rhs.at<double>(r+1,0)=dv;
            }
            cv::Mat sol;
            if(!cv::solve(A,rhs,sol,cv::DECOMP_SVD) || sol.rows!=4) return false;
            const double du=sol.at<double>(0,0), dv=sol.at<double>(1,0);
            const double dx=dv*h, dy=-du*h;
            if(!std::isfinite(dx)||!std::isfinite(dy)) return false;

            const double cr=std::cos(fg.roll),sr=std::sin(fg.roll);
            const double cp=std::cos(fg.pitch),sp=std::sin(fg.pitch);
            const double cy=std::cos(fg.yaw),sy=std::sin(fg.yaw);
            const double r00=cy*cp, r01=cy*sp*sr-sy*cr;
            const double r10=sy*cp, r11=sy*sp*sr+cy*cr;
            dN=r00*dx+r01*dy; dE=r10*dx+r11*dy;
            return std::isfinite(dN)&&std::isfinite(dE);
          };

          double n15=0,e15=0,n10=0,e10=0,n7=0,e7=0;
          const bool v15=fit(15,n15,e15);
          const bool v10=fit(10,n10,e10);
          const bool v7 =fit(7,n7,e7);
          if(v15){c15_n+=n15;c15_e+=e15;}
          if(v10){c10_n+=n10;c10_e+=e10;}
          if(v7 ){c7_n +=n7; c7_e +=e7;}

          if(diagnosticFileIoEnabled() && r5_csv.is_open()){
            r5_csv<<frame<<','<<ts<<','<<(s.valid?1:0)<<','<<s.invalid_reason<<','
                  <<np<<','<<dt<<','<<h<<','
                  <<(v15?1:0)<<','<<n15<<','<<e15<<','<<c15_n<<','<<c15_e<<','
                  <<(v10?1:0)<<','<<n10<<','<<e10<<','<<c10_n<<','<<c10_e<<','
                  <<(v7?1:0)<<','<<n7<<','<<e7<<','<<c7_n<<','<<c7_e<<'\n';
            if(shadowFlushEnabled()) r5_csv.flush();
          }
        }

        // WORKED5_MAG_SHADOW_V1
        // Diagnostic B arm only. Production s.valid, MAVLink and WORKED5-A are untouched.
        // Reuse the exact production RANSAC correspondences before the mag<4 gate.
        if(false){  // PERF_AB: disable WORKED5_MAG_SHADOW_V1
          static double mag_shadow_n=0.0, mag_shadow_e=0.0;
          static uint64_t mag_shadow_attempts=0, mag_shadow_valid=0;
          static std::ofstream mag_shadow_csv;
          static bool mag_shadow_header=false;

          if(diagnosticFileIoEnabled() && !mag_shadow_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            mag_shadow_csv.open(
              production_csv_path.parent_path() / "worked5_mag_shadow.csv",
              std::ios::out | std::ios::trunc);
          }
          if(diagnosticFileIoEnabled() && mag_shadow_csv.is_open() && !mag_shadow_header){
            mag_shadow_csv
              <<"frame,mono_ns,production_valid,invalid_reason,pairs,dt_s,hcam_m,"
              <<"shadow_attempted,shadow_valid,dN_m,dE_m,pN_m,pE_m,"
              <<"du_norm,dv_norm,dx_m,dy_m\n";
            mag_shadow_header=true;
          }

          double shadow_hcam=0.0;
          if(bench_true_camera_height>0.0) shadow_hcam=bench_true_camera_height;
          else if(current_camera_height_valid) shadow_hcam=current_camera_height_m;

          const int shadow_pairs=(int)std::min(
            s.metric_prev_points.size(),s.metric_curr_points.size());
          const bool shadow_attempt =
            fg_ok && dt>0.0 && dt<0.2 &&
            shadow_hcam>0.02 && std::isfinite(shadow_hcam) &&
            shadow_pairs>=20;

          bool shadow_valid=false;
          double shadow_dN=0.0,shadow_dE=0.0;
          worked5::Step shadow_w5{};
          if(shadow_attempt){
            ++mag_shadow_attempts;
            shadow_w5=worked5::estimate(
              s.metric_prev_points,s.metric_curr_points,
              calib.K,focal_scale,calib.D,shadow_hcam,dt);
            if(shadow_w5.valid){
              const double cr=std::cos(fg.roll),  sr=std::sin(fg.roll);
              const double cp=std::cos(fg.pitch), sp=std::sin(fg.pitch);
              const double cy=std::cos(fg.yaw),   sy=std::sin(fg.yaw);
              const double r00=cy*cp;
              const double r01=cy*sp*sr-sy*cr;
              const double r10=sy*cp;
              const double r11=sy*sp*sr+cy*cr;
              shadow_dN=r00*shadow_w5.dx_m+r01*shadow_w5.dy_m;
              shadow_dE=r10*shadow_w5.dx_m+r11*shadow_w5.dy_m;
              if(std::isfinite(shadow_dN) && std::isfinite(shadow_dE)){
                shadow_valid=true;
                ++mag_shadow_valid;
                mag_shadow_n+=shadow_dN;
                mag_shadow_e+=shadow_dE;
              }
            }
          }

          if(diagnosticFileIoEnabled() && mag_shadow_csv.is_open()){
            mag_shadow_csv
              <<frame<<','<<ts<<','<<(s.valid?1:0)<<','<<s.invalid_reason<<','
              <<shadow_pairs<<','<<dt<<','<<shadow_hcam<<','
              <<(shadow_attempt?1:0)<<','<<(shadow_valid?1:0)<<','
              <<shadow_dN<<','<<shadow_dE<<','
              <<mag_shadow_n<<','<<mag_shadow_e<<','
              <<shadow_w5.du_norm<<','<<shadow_w5.dv_norm<<','
              <<shadow_w5.dx_m<<','<<shadow_w5.dy_m<<'\n';
            if(shadowFlushEnabled()) mag_shadow_csv.flush();
          }
        }

        // Production lever-arm candidate. flow_body_x/y are angular image rates in AP body
        // convention, not linear velocity. For a downward camera, a camera
        // translation [vx,vy] produces approximately [-vy/h,+vx/h].
        // Camera focal-point velocity caused only by body rotation is omega x r.
        // r is the already audited camera position relative to the FC IMU in FRD.
        if(s.valid && fg_ok && current_camera_height_valid &&
           current_camera_height_m>0.05 &&
           std::isfinite(diag_camera_x_m) && std::isfinite(diag_camera_y_m) &&
           std::isfinite(diag_camera_z_m)){
          const cv::Vec3d omega(fg.x,fg.y,fg.z);
          const cv::Vec3d r_cam(diag_camera_x_m,diag_camera_y_m,diag_camera_z_m);
          const cv::Vec3d v_lever=omega.cross(r_cam);
          const double pred_x=-v_lever[1]/current_camera_height_m;
          const double pred_y= v_lever[0]/current_camera_height_m;
          const double corrected_x=s.flow_body_x-pred_x;
          const double corrected_y=s.flow_body_y-pred_y;
          if(std::isfinite(corrected_x) && std::isfinite(corrected_y) &&
             std::hypot(corrected_x,corrected_y)<4.0){
            s.lever_shadow_valid=true;
            s.lever_pred_flow_x=pred_x;
            s.lever_pred_flow_y=pred_y;
            s.lever_flow_body_x=corrected_x;
            s.lever_flow_body_y=corrected_y;
          }
        }

        bool flow_sent=false; uint8_t quality=0;
        // Production lever-arm compensation.  The camera focal point has real
        // linear velocity omega x r when the rigid body rotates about the FC/IMU.
        // Remove only that translation-like optical-flow component.  If the
        // shadow cannot be computed, preserve the proven pre-change flow path.
        const bool lever_production_applied=s.valid && s.lever_shadow_valid;
        double flow_send_x=lever_production_applied ? s.lever_flow_body_x : s.flow_body_x;
        double flow_send_y=lever_production_applied ? s.lever_flow_body_y : s.flow_body_y;
        if(s.valid && bench_height_override>0.0){
          double real_camera_height=0.0;
          double fake_camera_height=bench_height_override;
          if(std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
            fake_camera_height=bench_height_override-(diag_camera_z_m-diag_range_z_m);
          }

          if(bench_true_camera_height>0.0){
            real_camera_height=bench_true_camera_height;
          } else if(hl && lm>0.05){
            real_camera_height=lm;
            if(std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
              real_camera_height=lm-(diag_camera_z_m-diag_range_z_m);
            }
          }

          if(real_camera_height>0.02 && fake_camera_height>0.02 && fg_ok){
            const double k=real_camera_height/fake_camera_height;

            // IMPORTANT: raw optical flow contains BOTH body rotation and
            // translation.  ArduPilot later computes roughly:
            //   flow_comp = -flow_raw + body_rate
            // Therefore scaling the whole raw flow by k corrupts rotation
            // cancellation during roll/pitch.  Scale only the translational
            // residual and keep the rotational component at full magnitude:
            //
            //   flow_raw = gyro + translation
            //   flow_send = gyro + k * translation
            //
            // This makes AP's post-compensation residual k*translation, which
            // paired with the synthetic range preserves the real metric speed.
            flow_send_x = fg.x + k*(flow_send_x - fg.x);
            flow_send_y = fg.y + k*(flow_send_y - fg.y);
          }
        }
        // Variant B production path: strict camera-dequeue-causal35 only.
        // If unavailable, suppress the interval rather than mixing Variant A
        // raw semantics into FLOW_OPTIONS=Stabilised.
        const bool unified_publish_ready =
          (!stabilised_unified_publish || causal35_publish_valid) &&
          (!raw_unified_publish || causal35_raw_publish_valid);
        if(stabilised_unified_publish && causal35_publish_valid){
          flow_send_x=causal35_publish_flow_x;
          flow_send_y=causal35_publish_flow_y;
        } else if(raw_unified_publish && causal35_raw_publish_valid){
          flow_send_x=causal35_raw_publish_flow_x;
          flow_send_y=causal35_raw_publish_flow_y;
        }

        int64_t flow_send_ns=monoNs();
        const double frame_pipeline_latency_ms =
          (ts>0) ? (flow_send_ns-ts)*1e-6 : -1.0;
        const bool flow_fresh = frame_pipeline_latency_ms>=0.0 &&
                                frame_pipeline_latency_ms<=kMaxFlowPipelineAgeMs;

        // TEMPORAL_OF_TX_LOG_V1
        // Diagnostic only: exact aggregate values successfully transmitted
        // in this loop iteration. flow_sent is the validity flag.
        double flow_tx_x=0.0;
        double flow_tx_y=0.0;
        double flow_tx_dt_s=0.0;
        uint64_t flow_tx_inputs=0;

        const bool temporal_of_input =
          s.valid &&
          flow_fresh &&
          !terrain_step_guard &&
          unified_publish_ready &&
          dt>0.0 && dt<0.2 &&
          std::isfinite(flow_send_x) &&
          std::isfinite(flow_send_y);

        if(temporal_of_input){
          // flow_send_* is a rate over this camera interval.
          // Integrate it back to angular displacement so no valid interval
          // is lost when the FC consumes optical flow more slowly.
          temporal_of_angle_x += flow_send_x*dt;
          temporal_of_angle_y += flow_send_y*dt;
          temporal_of_dt_s += dt;
          ++temporal_of_inputs;
          temporal_of_last_input_ns=flow_send_ns;

          if(temporal_of_dt_s >= kTemporalOfPublishMinDtS){
            const double temporal_flow_x =
              temporal_of_angle_x/temporal_of_dt_s;
            const double temporal_flow_y =
              temporal_of_angle_y/temporal_of_dt_s;

            quality=255;
            flow_sent=sendOpticalFlow(
              fc.fd,
              (uint64_t)(flow_send_ns/1000),
              (float)temporal_flow_x,
              (float)temporal_flow_y,
              quality);

            if(flow_sent){
              ++flow_sent_total;
              if(s.valid && s.inliers>=30)
                last_ready_flow_send_ns=monoNs();
              flow_tx_x=temporal_flow_x;
              flow_tx_y=temporal_flow_y;
              flow_tx_dt_s=temporal_of_dt_s;
              flow_tx_inputs=temporal_of_inputs;

              // Consume the accumulator only after successful transmission.
              temporal_of_angle_x=0.0;
              temporal_of_angle_y=0.0;
              temporal_of_dt_s=0.0;
              temporal_of_inputs=0;
              temporal_of_last_input_ns=0;
            }
          }
        } else {
          // TEMPORAL_OF_DISCONTINUITY_FLUSH_V1:
          // Do not bridge across an invalid/missing interval, but also do not
          // destroy valid angular displacement accumulated immediately before
          // it. Flush that contiguous prefix as a short packet using the
          // timestamp of its last valid input, then reset unconditionally.
          if(temporal_of_inputs>0 &&
             temporal_of_dt_s>0.0 &&
             std::isfinite(temporal_of_angle_x) &&
             std::isfinite(temporal_of_angle_y)){
            const double temporal_flow_x=
              temporal_of_angle_x/temporal_of_dt_s;
            const double temporal_flow_y=
              temporal_of_angle_y/temporal_of_dt_s;
            quality=255;
            flow_sent=sendOpticalFlow(
              fc.fd,
              (uint64_t)(std::max<int64_t>(0,temporal_of_last_input_ns)/1000),
              (float)temporal_flow_x,
              (float)temporal_flow_y,
              quality);
            if(flow_sent){
              ++flow_sent_total;
              if(s.valid && s.inliers>=30)
                last_ready_flow_send_ns=monoNs();
              flow_tx_x=temporal_flow_x;
              flow_tx_y=temporal_flow_y;
              flow_tx_dt_s=temporal_of_dt_s;
              flow_tx_inputs=temporal_of_inputs;
            }
          }

          // The discontinuity itself is never integrated or bridged.
          temporal_of_angle_x=0.0;
          temporal_of_angle_y=0.0;
          temporal_of_dt_s=0.0;
          temporal_of_inputs=0;
          temporal_of_last_input_ns=0;

          if(!prev.empty() && !s.valid) ++flow_invalid_total;
          if(s.valid && !flow_fresh) ++stale_flow_rejected_total;
          if(s.valid && flow_fresh && terrain_step_guard) ++terrain_step_reject_total;
        }

        FlowFcLocal ep{}; double eage=1e9; uint64_t ec=0;
        const bool eok=fc.latestLocal(&ep,&eage,&ec);
        const bool efresh=eok&&eage<500.0;
        FlowEkfStatus es{}; double esage=1e9; uint64_t esc=0;
        const bool esok=fc.latestEkf(&es,&esage,&esc);
        const bool esfresh=esok&&esage<1000.0;

        // OSD_RSSI_HEALTH_V1.
        // High RSSI is transmitted only while production OF is fresh and
        // EKF horizontal aiding is confirmed. On any unhealthy state we send
        // nothing and deliberately let the FC-side RSSI timeout fall to zero.
        {
          const int64_t osd_now_ns=monoNs();
          const bool osd_flow_fresh=
            last_ready_flow_send_ns>0 &&
            osd_now_ns>=last_ready_flow_send_ns &&
            (osd_now_ns-last_ready_flow_send_ns)<=kOsdFlowFreshNs;
          const bool osd_ekf_aiding=
            esfresh &&
            (es.flags & EKF_ATTITUDE) &&
            (es.flags & EKF_VELOCITY_HORIZ) &&
            !(es.flags & EKF_UNINITIALIZED);
          const bool osd_healthy=
            startup_clock_reset_done &&
            startup_zero_done &&
            osd_flow_fresh &&
            osd_ekf_aiding;

          if(osd_healthy &&
             (last_osd_rssi_send_ns==0 ||
              osd_now_ns-last_osd_rssi_send_ns>=kOsdRssiPeriodNs)){
            if(sendHealthRadioStatus(fc.fd)){
              last_osd_rssi_send_ns=osd_now_ns;
            }
          }
        }

        bool arm_now=false; double arm_age_now=1e9;
        const bool arm_ok=fc.latestArm(&arm_now,&arm_age_now) && arm_age_now<2500.0;

        // RC6 or RC8: one HOME/zero event per physical press.
        // Read RC input channels, never SERVO outputs.
        {
          FlowFcRc rcin{}; double rc_age_ms=1e9;
          const bool rc_fresh=fc.latestRc(&rcin,&rc_age_ms) && rc_age_ms<500.0;
          if(rc_fresh){
            const uint16_t rc6=rcin.pwm[5];
            const uint16_t rc8=rcin.pwm[7];
            const uint16_t rc10=rcin.pwm[9];
            rc6_last_us=rc6;
            rc8_last_us=rc8;
            rc10_last_us=rc10;
            const bool pressed=(rc6>=kRcZeroPressUs)||(rc8>=kRcZeroPressUs)||(rc10>=kRcZeroPressUs);
            const bool released=(rc6<=kRcZeroReleaseUs)&&(rc8<=kRcZeroReleaseUs)&&(rc10<=kRcZeroReleaseUs);
            if(pressed && !rc_zero_latched){
              rc_zero_latched=true;
              ++rc_zero_seq;
        { std::lock_guard<std::mutex> l(fc.mu); imu_dr::reset(fc.imu_dr_state); }

              // Reset local diagnostic reference points immediately as well.
              web_raw_n=web_raw_e=0.0;
              web_raw_vn=web_raw_ve=0.0;
              web_raw_step_valid=false;

              if(rotation_gui && efresh){
                traj3d_n0=ep.x; traj3d_e0=ep.y; traj3d_z0=ep.z;
                traj3d_preview_n0=ep.x; traj3d_preview_e0=ep.y; traj3d_preview_z0=ep.z;
                traj3d_preview_origin_set=true;
                traj3d_origin_set=true;
                traj3d.clear();
                traj3d.emplace_back(0.0,0.0,0.0);
                traj3d_prev=cv::Vec3d(0,0,0);
                traj3d_prev_set=true;
                traj3d_path_total=0.0;
                traj3d_path_axis=cv::Vec3d(0,0,0);
                traj3d_peak_abs=cv::Vec3d(0,0,0);
              }

              if(return_gui && efresh){
                return_target_n=ep.x; return_target_e=ep.y;
                return_target_set=true; return_trail.clear();
                return_raw_x=return_raw_y=0.0;
                return_body_dx=return_body_dy=0.0;
                return_ned_n=return_ned_e=0.0;
                return_b_marked=false;
                return_home_marked=false;
                if(fg_ok){return_yaw0=fg.yaw;return_yaw0_set=true;}
                pending_return_event=1;
              }

              std::cerr<<"RC HOME ZERO: RC6="<<rc6<<" RC8="<<rc8<<" RC10="<<rc10
                       <<" seq="<<rc_zero_seq
                       <<" current position accepted as 0/0/0\n";
            } else if(released){
              rc_zero_latched=false;
            }
          }
        }

        // WORKED 5% closure estimator for Web UI / HOME return.
        // The estimator itself is frozen in worked5_estimator.hpp.  This layer
        // only maps its metric camera-plane delta into the existing N/E HOME
        // coordinate system.  The ArduPilot OPTICAL_FLOW publisher above is
        // intentionally untouched and remains an independent comparison path.
        // Per-frame forensic values below are logging only; they do not feed FC.
        bool worked5_diag_valid=false;
        int worked5_diag_points=0;
        double worked5_diag_hcam=0.0;
        double worked5_diag_du_norm=0.0,worked5_diag_dv_norm=0.0;
        double worked5_diag_dx=0.0,worked5_diag_dy=0.0;
        double worked5_diag_dN=0.0,worked5_diag_dE=0.0;
        web_raw_step_valid=false;
        web_raw_vn=web_raw_ve=0.0;
        if((s.valid || s.invalid_reason==6) && fg_ok && dt>0.0 && dt<0.2){
          double hcam=0.0;
          if(bench_true_camera_height>0.0){
            hcam=bench_true_camera_height;
          } else if(current_camera_height_valid){
            hcam=current_camera_height_m;
          }
          worked5_diag_hcam=hcam;
          if(hcam>0.02 && std::isfinite(hcam)){
            worked5_diag_points=(int)std::min(s.metric_prev_points.size(),s.metric_curr_points.size());
            ++fps_w5_attempt; ++w5w_attempt;
            const auto w5=worked5::estimate(
              s.metric_prev_points,s.metric_curr_points,
              calib.K,focal_scale,calib.D,hcam,dt);
            worked5_diag_du_norm=w5.du_norm;
            worked5_diag_dv_norm=w5.dv_norm;
            worked5_diag_dx=w5.dx_m;
            worked5_diag_dy=w5.dy_m;
            if(w5.valid){
              ++fps_w5_valid; ++w5w_valid;
              // Frozen blind convention gives a metric displacement in the
              // camera/body horizontal plane: X=+dv*H, Y=-du*H.  Rotate that
              // already-metric delta into NED using the FC attitude.  No extra
              // empirical scale, axis correction or gyro subtraction is added.
              const double cr=std::cos(fg.roll),  sr=std::sin(fg.roll);
              const double cp=std::cos(fg.pitch), sp=std::sin(fg.pitch);
              const double cy=std::cos(fg.yaw),   sy=std::sin(fg.yaw);
              const double r00=cy*cp;
              const double r01=cy*sp*sr-sy*cr;
              const double r10=sy*cp;
              const double r11=sy*sp*sr+cy*cr;

              const double dN=r00*w5.dx_m + r01*w5.dy_m;
              const double dE=r10*w5.dx_m + r11*w5.dy_m;
              worked5_diag_valid=true;
              worked5_diag_dN=dN;
              worked5_diag_dE=dE;
              web_raw_n += dN;
              web_raw_e += dE;
              web_raw_vn=dN/dt;
              web_raw_ve=dE/dt;
              web_raw_step_valid=true;
              bool v2_capture_ready=false;
              uint64_t v2_capture_cam_seq=0;
              int64_t v2_capture_cam_ns=0,v2_capture_imu_ns=0;
              double v2_capture_imu_n=0.0,v2_capture_imu_e=0.0;
              double v2_capture_imu_vn=0.0,v2_capture_imu_ve=0.0;
              {
                std::lock_guard<std::mutex> l(fc.mu);
                fc.imu_cam_vn=web_raw_vn;
                fc.imu_cam_ve=web_raw_ve;
                fc.imu_cam_dN=dN;
                fc.imu_cam_dE=dE;
                fc.imu_cam_dt=dt;
                ++fc.imu_cam_seq;
                fc.imu_cam_recv_ns=monoNs();
                fc.imu_cam_valid=true;

                // Snapshot diagnostic data under fc.mu; filesystem I/O is
                // deliberately deferred until after the critical section.
                if(!fc.fused_v2_imu_history.empty()){
                  const int64_t v2_cam_ns=fc.imu_cam_recv_ns;
                  auto best=fc.fused_v2_imu_history.begin();
                  int64_t best_abs=std::llabs(best->recv_ns-v2_cam_ns);
                  for(auto it=fc.fused_v2_imu_history.begin();it!=fc.fused_v2_imu_history.end();++it){
                    const int64_t d=std::llabs(it->recv_ns-v2_cam_ns);
                    if(d<best_abs){best=it;best_abs=d;}
                  }
                  v2_capture_ready=true;
                  v2_capture_cam_seq=fc.imu_cam_seq;
                  v2_capture_cam_ns=v2_cam_ns;
                  v2_capture_imu_ns=best->recv_ns;
                  v2_capture_imu_n=best->pos_n;
                  v2_capture_imu_e=best->pos_e;
                  v2_capture_imu_vn=best->vel_n;
                  v2_capture_imu_ve=best->vel_e;
                }

                // FUSED-V1 visual update happens HERE, once per unique WORKED5
                // observation. No latest-value mailbox is consumed by IMU.
                ++fc.fused_v1_visual_updates;
                fc.fused_v1_seen_cam_seq=fc.imu_cam_seq;
                fc.fused_v1_n += dN;
                fc.fused_v1_e += dE;

                const double vobs_n=dN/dt;
                const double vobs_e=dE/dt;
                fc.fused_v1_vn_hist[fc.fused_v1_vhist_head]=vobs_n;
                fc.fused_v1_ve_hist[fc.fused_v1_vhist_head]=vobs_e;
                fc.fused_v1_vhist_head=(fc.fused_v1_vhist_head+1)%5;
                if(fc.fused_v1_vhist_count<5) ++fc.fused_v1_vhist_count;

                double fused_sn=0.0,fused_se=0.0;
                for(int k=0;k<fc.fused_v1_vhist_count;++k){
                  fused_sn+=fc.fused_v1_vn_hist[k];
                  fused_se+=fc.fused_v1_ve_hist[k];
                }

                const double fused_cam_speed=std::hypot(web_raw_vn,web_raw_ve);
                if(fc.fused_v1_stationary){
                  if(fused_cam_speed>0.010){
                    fc.fused_v1_stationary=false;
                    fc.fused_v1_stop_confirm=0;
                  }
                }else{
                  if(fused_cam_speed<0.005){
                    ++fc.fused_v1_stop_confirm;
                    if(fc.fused_v1_stop_confirm>=3){
                      fc.fused_v1_stationary=true;
                      fc.fused_v1_stop_confirm=3;
                      ++fc.fused_v1_stop_constraints;
                    }
                  }else{
                    fc.fused_v1_stop_confirm=0;
                  }
                }

                if(fc.fused_v1_stationary){
                  fc.fused_v1_vn=0.0;
                  fc.fused_v1_ve=0.0;
                }else{
                  fc.fused_v1_vn=fused_sn/fc.fused_v1_vhist_count;
                  fc.fused_v1_ve=fused_se/fc.fused_v1_vhist_count;
                }
              }

              // FUSED_V2_CAPTURE_V1 diagnostic write: never hold fc.mu here.
              if(v2_capture_ready){
                static std::ofstream v2_capture_csv;
                static bool v2_capture_header=false;
                if(diagnosticFileIoEnabled() && !v2_capture_csv.is_open()){
                  const std::filesystem::path production_csv_path(csvpath);
                  v2_capture_csv.open(production_csv_path.parent_path()/"fused_v2_capture.csv",
                                      std::ios::out|std::ios::trunc);
                }
                if(diagnosticFileIoEnabled() && v2_capture_csv.is_open()){
                  if(!v2_capture_header){
                    v2_capture_csv<<"cam_seq,cam_recv_ns,imu_recv_ns,age_ms,imu_n_m,imu_e_m,imu_vn,imu_ve,dN_m,dE_m,dt_s\n";
                    v2_capture_header=true;
                  }
                  v2_capture_csv<<v2_capture_cam_seq<<','<<v2_capture_cam_ns<<','<<v2_capture_imu_ns<<','
                    <<(v2_capture_cam_ns-v2_capture_imu_ns)*1e-6<<','
                    <<v2_capture_imu_n<<','<<v2_capture_imu_e<<','
                    <<v2_capture_imu_vn<<','<<v2_capture_imu_ve<<','
                    <<dN<<','<<dE<<','<<dt<<'\n';
                  if(shadowFlushEnabled()) v2_capture_csv.flush();
                }
              }
            }
          }
        }

        // HIGH_DYNAMIC_RECOVERY_SHADOW_V2
        // Shadow-only A/B diagnostic. Normal WORKED5 steps are copied exactly.
        // A reason=6 interval is NOT promoted to production: the exact frozen
        // WORKED5 estimator is run on its retained RANSAC metric point pairs
        // and the resulting metric step is accumulated for logging only.
        bool highdyn_active=false;
        bool highdyn_reason6=false;
        double highdyn_raw_dx=0.0,highdyn_raw_dy=0.0;
        double highdyn_raw_dN=0.0,highdyn_raw_dE=0.0;
        double highdyn_confidence=0.0;
        if(worked5_diag_valid){
          highdyn_raw_dx=worked5_diag_dx;
          highdyn_raw_dy=worked5_diag_dy;
          highdyn_raw_dN=worked5_diag_dN;
          highdyn_raw_dE=worked5_diag_dE;
          highdyn_shadow_n+=highdyn_raw_dN;
          highdyn_shadow_e+=highdyn_raw_dE;
          highdyn_confidence=1.0;
        } else if(s.invalid_reason==6 && fg_ok && dt>0.0 && dt<0.2){
          double highdyn_hcam=0.0;
          if(bench_true_camera_height>0.0) highdyn_hcam=bench_true_camera_height;
          else if(current_camera_height_valid) highdyn_hcam=current_camera_height_m;
          if(highdyn_hcam>0.02 && std::isfinite(highdyn_hcam)){
            // reason=6 is assigned only after RANSAC has already populated
            // metric_prev_points/metric_curr_points.  Run the exact frozen
            // WORKED5 estimator on those points in SHADOW only.  This bypasses
            // the outer 4 rad/s validity gate for diagnostics, without changing
            // normal WORKED5, MAVLink publication, causal35 or EKF.
            const auto highdyn_w5=worked5::estimate(
              s.metric_prev_points,s.metric_curr_points,
              calib.K,focal_scale,calib.D,highdyn_hcam,dt);
            if(highdyn_w5.valid){
              highdyn_active=true;
              highdyn_reason6=true;
              ++highdyn_reason6_total;
              highdyn_raw_dx=highdyn_w5.dx_m;
              highdyn_raw_dy=highdyn_w5.dy_m;
              const double cr=std::cos(fg.roll),  sr=std::sin(fg.roll);
              const double cp=std::cos(fg.pitch), sp=std::sin(fg.pitch);
              const double cy=std::cos(fg.yaw),   sy=std::sin(fg.yaw);
              const double r00=cy*cp;
              const double r01=cy*sp*sr-sy*cr;
              const double r10=sy*cp;
              const double r11=sy*sp*sr+cy*cr;
              highdyn_raw_dN=r00*highdyn_raw_dx+r01*highdyn_raw_dy;
              highdyn_raw_dE=r10*highdyn_raw_dx+r11*highdyn_raw_dy;
              highdyn_shadow_n+=highdyn_raw_dN;
              highdyn_shadow_e+=highdyn_raw_dE;
              // Diagnostic metadata only; never gates or rescales the shadow.
              highdyn_confidence=std::clamp(s.inlier_ratio,0.0,1.0);
            }
          }
        }

        // DELTAR_ROTATION_SHADOW_V1
        // Dedicated A/B diagnostic for rotation contamination:
        //   A = frozen WORKED5 displacement;
        //   B = full-attitude ray geometry using ATTITUDE R0/R1;
        //   C = B minus camera lever-arm motion, i.e. FC/IMU-center translation;
        //   D = same geometry, but inter-frame delta-R is integrated from FC
        //       body rates and then lever-arm corrected.
        // This block is logging only. It never changes WORKED5, OPTICAL_FLOW,
        // ArduPilot output, FUSED-V1/V2, or any production state.
        if(false){  // PERF_AB: disable DELTAR_ROTATION_SHADOW_V1
          static std::ofstream dr_csv;
          static bool dr_header=false;
          if(!dr_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            dr_csv.open(
              production_csv_path.parent_path()/"deltar_rotation_shadow.csv",
              std::ios::out|std::ios::trunc);
          }
          if(dr_csv.is_open() && !dr_header){
            dr_csv
              <<"frame,t0_ns,t1_ns,dt_s,"
              <<"att0_valid,att1_valid,att0_gap_ms,att1_gap_ms,"
              <<"roll0,pitch0,yaw0,roll1,pitch1,yaw1,"
              <<"droll_deg,dpitch_deg,dyaw_deg,"
              <<"w5_valid,w5_dN_m,w5_dE_m,"
              <<"deltar_valid,deltar_reason,"
              <<"camera_dN_m,camera_dE_m,"
              <<"lever_dN_m,lever_dE_m,"
              <<"imu_dN_m,imu_dE_m,"
              <<"gyro_valid,gyro_segments,gyro_bracket_gap_ms,gyro_angle_deg,"
              <<"gyro_camera_dN_m,gyro_camera_dE_m,"
              <<"gyro_lever_dN_m,gyro_lever_dE_m,"
              <<"gyro_imu_dN_m,gyro_imu_dE_m,"
              <<"highres_valid,highres_segments,highres_bracket_gap_ms,highres_angle_deg,"
              <<"highres_camera_dN_m,highres_camera_dE_m,"
              <<"highres_lever_dN_m,highres_lever_dE_m,"
              <<"highres_imu_dN_m,highres_imu_dE_m,"
              <<"highres_corr_valid,highres_corr_segments,highres_corr_bracket_gap_ms,highres_corr_angle_deg,"
              <<"highres_corr_camera_dN_m,highres_corr_camera_dE_m,"
              <<"highres_corr_lever_dN_m,highres_corr_lever_dE_m,"
              <<"highres_corr_imu_dN_m,highres_corr_imu_dE_m,highres_corr_residual_median_m,"
              <<"stabilised_shadow_valid,stabilised_shadow_flow_x,stabilised_shadow_flow_y,"
              <<"stabilised_shadow_h0_m,"
              <<"stabilised_shadow_v_local_n,stabilised_shadow_v_local_e,stabilised_shadow_v_local_d,"
              <<"stabilised_shadow_v_body_x,stabilised_shadow_v_body_y,stabilised_shadow_v_body_z,"
              <<"stabilised_shadow_roundtrip_vx,stabilised_shadow_roundtrip_vy,stabilised_shadow_roundtrip_err,"
              <<"stabilised_sensor_shadow_valid,stabilised_sensor_shadow_flow_x,stabilised_sensor_shadow_flow_y,"
              <<"stabilised_sensor_shadow_v_body_x,stabilised_sensor_shadow_v_body_y,stabilised_sensor_shadow_v_body_z,"
              <<"stabilised_sensor_shadow_roundtrip_err,"
              <<"stabilised_lever_audit_valid,stabilised_lever_observed_vx,stabilised_lever_observed_vy,"
              <<"stabilised_lever_pred_vx,stabilised_lever_pred_vy,stabilised_lever_err_mps,"
              <<"stabilised_unified_shadow_valid,stabilised_unified_shadow_source,"
              <<"stabilised_unified_shadow_flow_x,stabilised_unified_shadow_flow_y,"
              <<"stabilised_unified_shadow_roundtrip_err,"
              <<"pairs,used,residual_median_m,gyro_residual_median_m,highres_residual_median_m,"
              <<"pixel_rot_valid,pixel_rot_points,pixel_rot_median_px,pixel_rot_p95_px,"
              <<"pixel_rot_du_median_px,pixel_rot_dv_median_px,"
              <<"pixel_rot_direct_valid,pixel_rot_direct_median_px,"
              <<"pixel_rot_direct_du_median_px,pixel_rot_direct_dv_median_px,"
              <<"pixel_rot_att_valid,pixel_rot_att_median_px,"
              <<"pixel_rot_att_du_median_px,pixel_rot_att_dv_median_px,"
              <<"pixel_field_valid,pixel_field_points,pixel_field_affine_rms_px,"
              <<"pixel_field_const_rms_px,pixel_field_a00,pixel_field_a01,"
              <<"pixel_field_a10,pixel_field_a11,pixel_field_bu,pixel_field_bv";
            static const char* kPixelExtrAxisName[kPixelExtrAxisN]={"roll","pitch","yaw"};
            for(int ax=0;ax<kPixelExtrAxisN;++ax){
              for(int oi=0;oi<kPixelExtrOffN;++oi){
                dr_csv<<",pixel_extr_"<<kPixelExtrAxisName[ax]<<"_"<<kPixelExtrOffDeg[oi]<<"deg_valid"
                      <<",pixel_extr_"<<kPixelExtrAxisName[ax]<<"_"<<kPixelExtrOffDeg[oi]<<"deg_median_px"
                      <<",pixel_extr_"<<kPixelExtrAxisName[ax]<<"_"<<kPixelExtrOffDeg[oi]<<"deg_du_px"
                      <<",pixel_extr_"<<kPixelExtrAxisName[ax]<<"_"<<kPixelExtrOffDeg[oi]<<"deg_dv_px";
              }
            }
            for(int pi=0;pi<kHighresPhaseN;++pi){
              dr_csv<<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_valid"
                    <<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_angle_deg"
                    <<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_imu_dN_m"
                    <<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_imu_dE_m"
                    <<",phase_"<<kHighresPhaseOffsetMs[pi]<<"ms_residual_median_m";
            }
            dr_csv<<"\n";
            dr_header=true;
          }

          const bool av0=metric_a0.valid;
          const bool av1=metric_a1.valid;
          double droll=0.0,dpitch=0.0,dyaw=0.0;
          if(av0 && av1){
            droll=metric_shadow::wrapPi(
              metric_a1.attitude.roll-metric_a0.attitude.roll)*180.0/M_PI;
            dpitch=metric_shadow::wrapPi(
              metric_a1.attitude.pitch-metric_a0.attitude.pitch)*180.0/M_PI;
            dyaw=metric_shadow::wrapPi(
              metric_a1.attitude.yaw-metric_a0.attitude.yaw)*180.0/M_PI;
          }

          dr_csv
            <<frame<<','<<prev_ts<<','<<ts<<','<<dt<<','
            <<(av0?1:0)<<','<<(av1?1:0)<<','
            <<metric_att_gap0_ms<<','<<metric_att_gap1_ms<<','
            <<(av0?metric_a0.attitude.roll:0.0)<<','
            <<(av0?metric_a0.attitude.pitch:0.0)<<','
            <<(av0?metric_a0.attitude.yaw:0.0)<<','
            <<(av1?metric_a1.attitude.roll:0.0)<<','
            <<(av1?metric_a1.attitude.pitch:0.0)<<','
            <<(av1?metric_a1.attitude.yaw:0.0)<<','
            <<droll<<','<<dpitch<<','<<dyaw<<','
            <<(worked5_diag_valid?1:0)<<','
            <<worked5_diag_dN<<','<<worked5_diag_dE<<','
            <<(metric_step.valid?1:0)<<','
            <<metric_shadow::rejectReasonName(metric_step.reason)<<','
            <<metric_step.delta_camera_local_m[0]<<','
            <<metric_step.delta_camera_local_m[1]<<','
            <<metric_step.lever_local_m[0]<<','
            <<metric_step.lever_local_m[1]<<','
            <<metric_step.delta_local_m[0]<<','
            <<metric_step.delta_local_m[1]<<','
            <<(metric_gyro_step.valid?1:0)<<','
            <<metric_gyro_delta.segments<<','
            <<metric_gyro_delta.max_bracket_gap_ms<<','
            <<metric_gyro_delta.integrated_angle_deg<<','
            <<metric_gyro_step.delta_camera_local_m[0]<<','
            <<metric_gyro_step.delta_camera_local_m[1]<<','
            <<metric_gyro_step.lever_local_m[0]<<','
            <<metric_gyro_step.lever_local_m[1]<<','
            <<metric_gyro_step.delta_local_m[0]<<','
            <<metric_gyro_step.delta_local_m[1]<<','
            <<(metric_highres_gyro_step.valid?1:0)<<','
            <<metric_highres_gyro_delta.segments<<','
            <<metric_highres_gyro_delta.max_bracket_gap_ms<<','
            <<metric_highres_gyro_delta.integrated_angle_deg<<','
            <<metric_highres_gyro_step.delta_camera_local_m[0]<<','
            <<metric_highres_gyro_step.delta_camera_local_m[1]<<','
            <<metric_highres_gyro_step.lever_local_m[0]<<','
            <<metric_highres_gyro_step.lever_local_m[1]<<','
            <<metric_highres_gyro_step.delta_local_m[0]<<','
            <<metric_highres_gyro_step.delta_local_m[1]<<','
            <<(metric_highres_corr_gyro_step.valid?1:0)<<','
            <<metric_highres_corr_gyro_delta.segments<<','
            <<metric_highres_corr_gyro_delta.max_bracket_gap_ms<<','
            <<metric_highres_corr_gyro_delta.integrated_angle_deg<<','
            <<metric_highres_corr_gyro_step.delta_camera_local_m[0]<<','
            <<metric_highres_corr_gyro_step.delta_camera_local_m[1]<<','
            <<metric_highres_corr_gyro_step.lever_local_m[0]<<','
            <<metric_highres_corr_gyro_step.lever_local_m[1]<<','
            <<metric_highres_corr_gyro_step.delta_local_m[0]<<','
            <<metric_highres_corr_gyro_step.delta_local_m[1]<<','
            <<metric_highres_corr_gyro_step.residual_median_m<<','
            <<(stabilised_shadow_valid?1:0)<<','
            <<stabilised_shadow_flow_x<<','
            <<stabilised_shadow_flow_y<<','
            <<stabilised_shadow_h0_m<<','
            <<stabilised_shadow_v_local_n<<','
            <<stabilised_shadow_v_local_e<<','
            <<stabilised_shadow_v_local_d<<','
            <<stabilised_shadow_v_body_x<<','
            <<stabilised_shadow_v_body_y<<','
            <<stabilised_shadow_v_body_z<<','
            <<stabilised_shadow_roundtrip_vx<<','
            <<stabilised_shadow_roundtrip_vy<<','
            <<stabilised_shadow_roundtrip_err<<','
            <<(stabilised_sensor_shadow_valid?1:0)<<','
            <<stabilised_sensor_shadow_flow_x<<','
            <<stabilised_sensor_shadow_flow_y<<','
            <<stabilised_sensor_shadow_v_body_x<<','
            <<stabilised_sensor_shadow_v_body_y<<','
            <<stabilised_sensor_shadow_v_body_z<<','
            <<stabilised_sensor_shadow_roundtrip_err<<','
            <<(stabilised_lever_audit_valid?1:0)<<','
            <<stabilised_lever_observed_vx<<','
            <<stabilised_lever_observed_vy<<','
            <<stabilised_lever_pred_vx<<','
            <<stabilised_lever_pred_vy<<','
            <<stabilised_lever_err_mps<<','
            <<(stabilised_unified_shadow_valid?1:0)<<','
            <<stabilised_unified_shadow_source<<','
            <<stabilised_unified_shadow_flow_x<<','
            <<stabilised_unified_shadow_flow_y<<','
            <<stabilised_unified_shadow_roundtrip_err<<','
            <<s.metric_prev_points.size()<<','
            <<metric_step.points<<','
            <<metric_step.residual_median_m<<','
            <<metric_gyro_step.residual_median_m<<','
            <<metric_highres_gyro_step.residual_median_m<<','
            <<(pixel_rot_valid?1:0)<<','
            <<pixel_rot_points<<','
            <<pixel_rot_median_px<<','
            <<pixel_rot_p95_px<<','
            <<pixel_rot_du_median_px<<','
            <<pixel_rot_dv_median_px<<','
            <<(pixel_rot_direct_valid?1:0)<<','
            <<pixel_rot_direct_median_px<<','
            <<pixel_rot_direct_du_median_px<<','
            <<pixel_rot_direct_dv_median_px<<','
            <<(pixel_rot_att_valid?1:0)<<','
            <<pixel_rot_att_median_px<<','
            <<pixel_rot_att_du_median_px<<','
            <<pixel_rot_att_dv_median_px<<','
            <<(pixel_field_valid?1:0)<<','
            <<pixel_field_points<<','
            <<pixel_field_affine_rms_px<<','
            <<pixel_field_const_rms_px<<','
            <<pixel_field_a00<<','<<pixel_field_a01<<','
            <<pixel_field_a10<<','<<pixel_field_a11<<','
            <<pixel_field_bu<<','<<pixel_field_bv;
          for(int ax=0;ax<kPixelExtrAxisN;++ax){
            for(int oi=0;oi<kPixelExtrOffN;++oi){
              dr_csv<<','<<pixel_extr_valid[ax][oi]
                    <<','<<pixel_extr_med[ax][oi]
                    <<','<<pixel_extr_du[ax][oi]
                    <<','<<pixel_extr_dv[ax][oi];
            }
          }
          for(int pi=0;pi<kHighresPhaseN;++pi){
            dr_csv<<','<<(metric_highres_phase_step[pi].valid?1:0)
                  <<','<<metric_highres_phase_delta[pi].integrated_angle_deg
                  <<','<<metric_highres_phase_step[pi].delta_local_m[0]
                  <<','<<metric_highres_phase_step[pi].delta_local_m[1]
                  <<','<<metric_highres_phase_step[pi].residual_median_m;
          }
          dr_csv<<'\n';
          dr_csv.flush();
        }

        const int64_t stage_csv_enter_ns=monoNs();
        FlowFcTarget csv_ct{}; FlowFcAttTarget csv_ca{}; FlowFcOutputs csv_co{};
        FlowFcRc csv_rc{};
        uint32_t csv_custom_mode=0;
        uint8_t csv_base_mode=0,csv_system_status=0;
        double csv_hb_age=1e9;
        const bool csv_hb_ok=fc.latestHeartbeat(
          &csv_custom_mode,&csv_base_mode,&csv_system_status,&csv_hb_age);
        double csv_ct_age=1e9,csv_ca_age=1e9,csv_co_age=1e9,csv_rc_age=1e9;
        fc.latestControl(&csv_ct,&csv_ca,&csv_co,&csv_ct_age,&csv_ca_age,&csv_co_age);
        const bool csv_rc_have=fc.latestRc(&csv_rc,&csv_rc_age);
        const bool csv_ct_ok=csv_ct.valid&&csv_ct_age<500.0;
        const bool csv_ca_ok=csv_ca.valid&&csv_ca_age<500.0;
        const bool csv_co_ok=csv_co.valid&&csv_co_age<500.0;
        const bool csv_rc_ok=csv_rc_have&&csv_rc.valid&&csv_rc_age<500.0;
        if(require_armed && arm_ok && !arm_now && guide_stage.load()<3){
          if(!arm_lost.exchange(true)){
            std::cerr<<"\nОШИБКА: FC ПЕРЕШЁЛ В DISARMED ВО ВРЕМЯ ARMED-ТЕСТА.\n"
                     <<"Тест остановлен; результат движения недействителен.\n";
          }
          g_running=false;
        }

        int64_t stage_csv_helpers_enter_ns=monoNs();
        int64_t stage_csv_helpers_exit_ns=stage_csv_helpers_enter_ns;
        int64_t stage_csv_stream_enter_ns=stage_csv_helpers_enter_ns;
        int64_t stage_csv_stream_exit_ns=stage_csv_helpers_enter_ns;
        int64_t stage_csv_flush_enter_ns=stage_csv_helpers_enter_ns;
        int64_t stage_csv_flush_exit_ns=stage_csv_helpers_enter_ns;
        int64_t stage_csv_tellp_enter_ns=stage_csv_helpers_enter_ns;
        int64_t stage_csv_tellp_exit_ns=stage_csv_helpers_enter_ns;
        int64_t stage_csv_limit_exit_ns=stage_csv_helpers_enter_ns;
        if(csv_logging_enabled){
        const double v4l2_to_dequeue_ms =
          (selected_v4l2_ts_ns>0 && selected_dq_mono_ns>0)
            ? (selected_dq_mono_ns-selected_v4l2_ts_ns)*1e-6 : -1.0;
        // FUSED_V2_FRAME_CAPTURE_V1
        // 'now' is the monotonic frame timestamp used for causal IMU lookup.
        fc.writeFusedV2FrameCapture(
          csvpath,frame,now,s.valid,s.invalid_reason,
          s.tracked,s.inliers,s.inlier_ratio,dt);
        // FUSED_V2_REALTIME_SHADOW_V1
        fc.updateFusedV2RealtimeShadow(
          csvpath,frame,now,s.valid,s.invalid_reason,
          s.tracked,s.inliers,s.inlier_ratio,dt);

        stage_csv_helpers_exit_ns=monoNs();
        const double camera_poll_ms=(camera_poll_exit_ns-camera_poll_enter_ns)*1e-6;
        const double camera_dq_ioctl_ms=camera_dq_ioctl_max_ms;
        const double camera_dq_batch_ms=(camera_dq_batch_exit_ns-camera_dq_batch_enter_ns)*1e-6;
        stage_csv_stream_enter_ns=monoNs();
        std::ostringstream csv_row;
        csv_row<<now<<','<<ts<<','<<selected_v4l2_ts_ns<<','<<selected_dq_mono_ns<<','
           <<selected_v4l2_flags<<','<<v4l2_to_dequeue_ms<<','
           <<camera_poll_enter_ns<<','<<camera_poll_exit_ns<<','<<camera_poll_ms<<','
           <<camera_dq_first_enter_ns<<','<<camera_dq_last_exit_ns<<','
           <<camera_dq_ioctl_ms<<','<<camera_dq_batch_ms<<','<<camera_dq_count<<','
           <<(prev_loop_end_ns>0?(camera_poll_enter_ns-prev_loop_end_ns)*1e-6:-1.0)<<','
           <<prev_send_to_csv_ms<<','<<prev_csv_block_ms<<','
           <<prev_csv_helpers_ms<<','<<prev_csv_stream_ms<<','<<prev_csv_flush_ms<<','
           <<prev_csv_tellp_ms<<','<<prev_csv_limit_ms<<','<<prev_web_block_ms<<','
           <<prev_postweb_ms<<','<<prev_anchor_ms<<','
           <<flow_send_ns<<','<<frame_pipeline_latency_ms<<','
           <<camera_queue_dropped<<','<<camera_queue_dropped_total<<','
           <<frame<<','<<guide_leg.load()<<','<<guide_stage.load()<<','<<(s.valid?1:0)<<','<<s.invalid_reason<<','<<(bridge_pending?1:0)<<','<<dt<<','
           <<s.features<<','<<s.tracked<<','<<s.inliers<<','<<s.inlier_ratio<<','
           <<s.t_features_ms<<','<<s.t_lk_ms<<','<<s.t_ransac_ms<<','<<s.t_post_ms<<','
           <<s.du_px<<','<<s.dv_px<<','<<s.du_norm<<','<<s.dv_norm<<','<<s.yaw_rate_cam_z<<','
           <<s.scale_rate<<','<<s.lk_height_scale<<','
           <<s.flow_cam_x<<','<<s.flow_cam_y<<','<<s.flow_body_x<<','<<s.flow_body_y<<','
           <<(s.lever_shadow_valid?1:0)<<','<<(lever_production_applied?1:0)<<','<<s.lever_flow_body_x<<','<<s.lever_flow_body_y<<','
           <<s.lever_pred_flow_x<<','<<s.lever_pred_flow_y<<','
           <<(g_fb_shadow_max_px>0.0?1:0)<<','<<g_fb_shadow_max_px<<','<<s.fb_checked<<','<<s.fb_pass<<','<<s.fb_ratio<<','<<s.fb_inliers<<','<<(s.fb_shadow_valid?1:0)<<','<<s.fb_flow_body_x<<','<<s.fb_flow_body_y<<','<<s.fb_t_ms<<','
           <<(s.robust_shadow_valid?1:0)<<','<<s.robust_flow_body_x<<','<<s.robust_flow_body_y<<','<<s.robust_sigma<<','<<s.robust_mean_weight<<','<<s.robust_downweighted<<','<<s.robust_iters<<','
           <<(s.obs_shadow_valid?1:0)<<','<<s.obs_flow_body_x<<','<<s.obs_flow_body_y<<','<<s.obs_median_ratio<<','<<s.obs_mean_weight<<','<<s.obs_downweighted<<','
           <<(int)quality<<','<<lm<<','<<lage<<','<<range_to_fc<<','<<flow_send_x<<','<<flow_send_y<<','<<(flow_sent?1:0)<<','
           <<flow_tx_x<<','<<flow_tx_y<<','<<flow_tx_dt_s<<','<<flow_tx_inputs<<','
           <<(raw_unified_publish?1:0)<<','
           <<(causal35_publish_valid?1:0)<<','<<causal35_publish_flow_x<<','<<causal35_publish_flow_y<<','
           <<causal35_raw_gyro_x<<','<<causal35_raw_gyro_y<<','
           <<(causal35_raw_publish_valid?1:0)<<','<<causal35_raw_publish_flow_x<<','<<causal35_raw_publish_flow_y<<','
           <<causal35_optical_depth_m<<','
           <<causal35_reject_reason<<','
           <<causal35_anchor_recv_age_diag_ms<<','<<causal35_anchor_sample_age_diag_ms<<','
           <<causal35_deltar_hold_diag_ms<<','
           <<(stabilised_unified_publish?1:0)<<','<<(unified_publish_ready?1:0)<<','
           <<stabilised_unified_shadow_source<<','<<(range_sent?1:0)<<','
           <<(arm_ok?(arm_now?1:0):-1)<<','
           <<(csv_hb_ok?1:0)<<','<<(csv_hb_ok?csv_hb_age:-1.0)<<','
           <<csv_custom_mode<<','<<(int)csv_base_mode<<','<<(int)csv_system_status<<','
           <<(efresh?1:0)<<','<<ep.x<<','<<ep.y<<','<<ep.z<<','<<ep.vx<<','<<ep.vy<<','<<ep.vz<<','<<eage<<','<<ec<<','
           <<(esfresh?1:0)<<','<<es.flags<<','<<esage<<','<<esc<<','
           <<es.velocity_variance<<','<<es.pos_horiz_variance<<','<<es.pos_vert_variance<<','<<es.compass_variance<<','<<es.terrain_alt_variance<<','
           <<pending_return_event<<','
           <<rc_zero_seq<<','
           <<(csv_rc_ok?1:0)<<','<<(csv_rc_have?csv_rc_age:-1.0);
        for(int ri=0;ri<18;ri++) csv_row<<','<<csv_rc.pwm[ri];
        csv_row<<','<<(worked5_diag_valid?1:0)<<','<<worked5_diag_points<<','<<worked5_diag_hcam<<','
           <<worked5_diag_du_norm<<','<<worked5_diag_dv_norm<<','
           <<worked5_diag_dx<<','<<worked5_diag_dy<<','<<worked5_diag_dN<<','<<worked5_diag_dE<<','
           <<web_raw_n<<','<<web_raw_e<<','
           <<(highdyn_active?1:0)<<','<<(highdyn_reason6?1:0)<<','
           <<highdyn_raw_dx<<','<<highdyn_raw_dy<<','<<highdyn_raw_dN<<','<<highdyn_raw_dE<<','
           <<highdyn_shadow_n<<','<<highdyn_shadow_e<<','<<highdyn_confidence<<','
           <<(fg_ok?fg.roll:0.0)<<','<<(fg_ok?fg.pitch:0.0)<<','<<(fg_ok?fg.yaw:0.0)<<','
           <<(fg_ok?fg.x:0.0)<<','<<(fg_ok?fg.y:0.0)<<','<<(fg_ok?fg.z:0.0)<<','<<(fg_ok?fg_age:-1.0)<<','<<fg_samples<<','
           <<(csv_ct_ok?1:0)<<','<<csv_ct.x<<','<<csv_ct.y<<','<<csv_ct.vx<<','<<csv_ct.vy<<','<<(csv_ct_ok?csv_ct_age:-1.0)<<','
           <<(csv_ca_ok?1:0)<<','<<csv_ca.roll<<','<<csv_ca.pitch<<','<<csv_ca.yaw<<','<<csv_ca.thrust<<','<<(csv_ca_ok?csv_ca_age:-1.0)<<','
           <<(csv_co_ok?1:0);
        for(int oi=0;oi<8;oi++) csv_row<<','<<csv_co.pwm[oi];
        csv_row<<','<<(csv_co_ok?csv_co_age:-1.0);
        for(int ci=0;ci<9;ci++){
          csv_row<<','<<s.cell_n[ci]<<','<<s.cell_body_x[ci]<<','<<s.cell_body_y[ci];
        }
        csv_row<<'\n';
        csv_writer.tryEnqueue(csv_row.str());
        stage_csv_stream_exit_ns=monoNs();
        stage_csv_flush_enter_ns=stage_csv_stream_exit_ns;
        // The web UI tails this CSV.  std::ofstream otherwise buffers many
        // rows, which creates seconds of apparent telemetry lag.  Flush the
        // userspace stream at 20 Hz; this is flush(), not fsync(), so we avoid
        // forcing physical storage on every camera frame.
        if(csvLiveFlushEnabled() && now-last_csv_flush_ns>=kCsvLiveFlushNs){
          csv_writer.requestFlush();
          last_csv_flush_ns=now;
        }
        stage_csv_flush_exit_ns=monoNs();
        stage_csv_tellp_enter_ns=stage_csv_flush_exit_ns;

        // The writer thread owns file I/O.  The flight loop only checks the
        // asynchronously counted payload bytes and never flushes/closes here.
        const std::streamoff csv_pos=static_cast<std::streamoff>(csv_writer.bytes());
        stage_csv_tellp_exit_ns=monoNs();
        if(csv_pos>=kCsvMaxBytes){
          csv_logging_enabled=false;
          if(!csv_limit_reported){
            csv_limit_reported=true;
            std::cerr<<"\nПРЕДУПРЕЖДЕНИЕ: CSV достиг лимита 250 MiB; постановка новых строк остановлена. "
                     <<"Полётный publisher продолжает работать. CSV="<<csvpath<<"\n";
          }
        }
        stage_csv_limit_exit_ns=monoNs();
        }
        const int64_t stage_csv_exit_ns=monoNs();

        {
          std::ostringstream js;
          js<<"{\"type\":\"telemetry\""
            <<",\"mono_ns\":"<<now
            <<",\"frame\":"<<frame
            <<",\"valid\":"<<(s.valid?1:0)
            <<",\"quality\":"<<(int)quality
            <<",\"features\":"<<s.features
            <<",\"tracked\":"<<s.tracked
            <<",\"inliers\":"<<s.inliers
            <<",\"range_m\":"<<jsonNumber(lm)
            <<",\"range_age_ms\":"<<jsonNumber(lage)
            <<",\"armed\":"<<(arm_ok&&arm_now?"true":"false")
            <<",\"ekf_valid\":"<<(efresh?"true":"false")
            <<",\"x\":"<<jsonNumber(ep.x)
            <<",\"y\":"<<jsonNumber(ep.y)
            <<",\"z\":"<<jsonNumber(ep.z)
            <<",\"vx\":"<<jsonNumber(ep.vx)
            <<",\"vy\":"<<jsonNumber(ep.vy)
            <<",\"vz\":"<<jsonNumber(ep.vz)
            <<",\"raw_of_valid\":"<<(web_raw_step_valid?"true":"false")
            <<",\"raw_of_n\":"<<jsonNumber(web_raw_n)
            <<",\"raw_of_e\":"<<jsonNumber(web_raw_e)
            <<",\"raw_of_vn\":"<<jsonNumber(web_raw_vn)
            <<",\"raw_of_ve\":"<<jsonNumber(web_raw_ve)
            <<",\"imu_raw_ax\":"<<jsonNumber(fc.imu.ax)
            <<",\"imu_raw_ay\":"<<jsonNumber(fc.imu.ay)
            <<",\"imu_raw_az\":"<<jsonNumber(fc.imu.az)
            <<",\"imu_dr_calibrated\":"<<(fc.imu_dr_state.calibrated?"true":"false")
            <<",\"imu_dr_calibrating\":"<<(fc.imu_dr_state.calibrating?"true":"false")
            <<",\"imu_dr_bias_samples\":"<<fc.imu_dr_state.bias_samples
            <<",\"imu_dr_bias_bx\":"<<jsonNumber(fc.imu_dr_state.bias_bx)
            <<",\"imu_dr_bias_by\":"<<jsonNumber(fc.imu_dr_state.bias_by)
            <<",\"imu_dr_bias_bz\":"<<jsonNumber(fc.imu_dr_state.bias_bz)
            <<",\"imu_dr_n_mm\":"<<jsonNumber(fc.imu_dr_state.pos_n*1000.0)
            <<",\"imu_dr_e_mm\":"<<jsonNumber(fc.imu_dr_state.pos_e*1000.0)
            <<",\"imu_dr_d_mm\":"<<jsonNumber(fc.imu_dr_state.pos_d*1000.0)
            <<",\"imu_dr_acc_n\":"<<jsonNumber(fc.imu_dr_state.acc_n)
            <<",\"imu_dr_acc_e\":"<<jsonNumber(fc.imu_dr_state.acc_e)
            <<",\"imu_dr_acc_d\":"<<jsonNumber(fc.imu_dr_state.acc_d)
            <<",\"imu_dr_vn\":"<<jsonNumber(fc.imu_dr_state.vel_n)
            <<",\"imu_dr_ve\":"<<jsonNumber(fc.imu_dr_state.vel_e)
            <<",\"imu_dr_vd\":"<<jsonNumber(fc.imu_dr_state.vel_d)
            <<",\"imu_dr_n\":"<<jsonNumber(fc.imu_dr_state.pos_n)
            <<",\"imu_dr_e\":"<<jsonNumber(fc.imu_dr_state.pos_e)
            <<",\"imu_dr_d\":"<<jsonNumber(fc.imu_dr_state.pos_d)
            <<",\"imu_dr_stationary_samples\":"<<fc.imu_dr_state.stationary_samples
            <<",\"imu_dr_amag\":"<<jsonNumber(fc.imu_dr_state.diag_amag)
            <<",\"imu_dr_gmag\":"<<jsonNumber(fc.imu_dr_state.diag_gmag)
            <<",\"imu_dr_dt\":"<<jsonNumber(fc.imu_dr_state.diag_dt)
            <<",\"imu_dr_acc_ok\":"<<(fc.imu_dr_state.diag_acc_ok?"true":"false")
            <<",\"imu_dr_gyro_ok\":"<<(fc.imu_dr_state.diag_gyro_ok?"true":"false")
            <<",\"imu_dr_stationary\":"<<(fc.imu_dr_state.diag_stationary?"true":"false")
            <<",\"imu_dr_acc_rejects\":"<<fc.imu_dr_state.diag_acc_rejects
            <<",\"imu_dr_gyro_rejects\":"<<fc.imu_dr_state.diag_gyro_rejects
            <<",\"imu_cam_vn\":"<<jsonNumber(fc.imu_cam_vn)
            <<",\"imu_cam_ve\":"<<jsonNumber(fc.imu_cam_ve)
            <<",\"imu_cam_speed\":"<<jsonNumber(std::hypot(fc.imu_cam_vn,fc.imu_cam_ve))
            <<",\"imu_cam_age_ms\":"<<jsonNumber(fc.imu_cam_valid?(monoNs()-fc.imu_cam_recv_ns)*1e-6:-1.0)
            <<",\"imu_cam_fresh\":"<<(fc.imu_zupt_cam_fresh?"true":"false")
            <<",\"imu_cam_stationary\":"<<(fc.imu_zupt_cam_stationary?"true":"false")
            <<",\"imu_zupt_shadow\":"<<(fc.imu_zupt_shadow?"true":"false")
            <<",\"imu_zupt_shadow_accepts\":"<<fc.imu_zupt_shadow_accepts
            <<",\"imu_zupt_shadow_blocks\":"<<fc.imu_zupt_shadow_blocks
            <<",\"imu_cam_seq\":"<<fc.imu_cam_seq
            <<",\"fused_v1_visual_updates\":"<<fc.fused_v1_visual_updates
            <<",\"fused_v1_imu_predictions\":"<<fc.fused_v1_imu_predictions
            <<",\"fused_v1_stop_constraints\":"<<fc.fused_v1_stop_constraints
            <<",\"fused_v1_stationary\":"<<(fc.fused_v1_stationary?"true":"false")
            <<",\"fused_v1_stop_confirm\":"<<fc.fused_v1_stop_confirm
            <<",\"fused_v1_n_mm\":"<<jsonNumber(fc.fused_v1_n*1000.0)
            <<",\"fused_v1_e_mm\":"<<jsonNumber(fc.fused_v1_e*1000.0)
            <<",\"fused_v1_vn\":"<<jsonNumber(fc.fused_v1_vn)
            <<",\"fused_v1_ve\":"<<jsonNumber(fc.fused_v1_ve)
            <<",\"imu_camvc_active\":"<<(fc.imu_camvc_active?"true":"false")
            <<",\"imu_camvc_stop_samples\":"<<fc.imu_camvc_stop_samples
            <<",\"imu_camvc_activations\":"<<fc.imu_camvc_activations
            <<",\"imu_camvc_n_mm\":"<<jsonNumber(fc.imu_camvc_state.pos_n*1000.0)
            <<",\"imu_camvc_e_mm\":"<<jsonNumber(fc.imu_camvc_state.pos_e*1000.0)
            <<",\"imu_camvc_d_mm\":"<<jsonNumber(fc.imu_camvc_state.pos_d*1000.0)
            <<",\"imu_camvc_vn\":"<<jsonNumber(fc.imu_camvc_state.vel_n)
            <<",\"imu_camvc_ve\":"<<jsonNumber(fc.imu_camvc_state.vel_e)
            <<",\"imu_camvc_vd\":"<<jsonNumber(fc.imu_camvc_state.vel_d)
            <<",\"imu_dr_attitude_age_ms\":"<<jsonNumber(fc.gyro.valid?(monoNs()-fc.gyro.recv_ns)*1e-6:-1.0)
            <<",\"rc_zero_seq\":"<<rc_zero_seq
            <<",\"rc6_us\":"<<rc6_last_us
            <<",\"rc8_us\":"<<rc8_last_us
            <<",\"rc10_us\":"<<rc10_last_us
            <<",\"roll_deg\":"<<jsonNumber(fg_ok?fg.roll*180.0/M_PI:0.0)
            <<",\"pitch_deg\":"<<jsonNumber(fg_ok?fg.pitch*180.0/M_PI:0.0)
            <<",\"yaw_deg\":"<<jsonNumber(fg_ok?fg.yaw*180.0/M_PI:0.0)
            <<"}";
          web_live.send(now,js.str());
        }
        const int64_t stage_web_exit_ns=monoNs();
        const bool blind4_final_event_written =
          blind4_cli && blind4_state>=8 && pending_return_event==18;
        pending_return_event=0;
        if(blind4_final_event_written){
          csv_writer.requestFlush();
          std::cerr<<"BLIND4 ЗАВЕРШЁН. GT программе не сообщался.\n";
          g_running=false;
        }

        if(flight_ready_gate && !flight_ready){
          const double speed_h=efresh?std::hypot((double)ep.vx,(double)ep.vy):1e9;
          const double ready_range=(bench_height_override>0.0)?bench_height_override:lm;
          const bool luna_ok=(bench_true_camera_height>0.0 && bench_height_override>0.0)
            ? (ready_range>=kReadyMinRangeM && ready_range<=kReadyMaxRangeM)
            : (hl && lage>=-2.0 && lage<100.0 &&
               ready_range>=kReadyMinRangeM && ready_range<=kReadyMaxRangeM);
          const int64_t ready_check_ns=monoNs();
          const bool flow_ok=
            last_ready_flow_send_ns>0 &&
            ready_check_ns>=last_ready_flow_send_ns &&
            (ready_check_ns-last_ready_flow_send_ns)<=kReadyFlowFreshNs;
          const bool ekf_ok=esfresh &&
                            (es.flags & EKF_ATTITUDE) &&
                            (es.flags & EKF_VELOCITY_HORIZ) &&
                            !(es.flags & EKF_UNINITIALIZED);

          // STARTUP_READY_NO_LOCAL_V1:
          // LOCAL_POSITION_NED may appear ~25 s after EKF horizontal velocity
          // aiding and production optical flow are already healthy. Do not make
          // startup zero / OSD health depend on that late telemetry message.
          const bool ready_now=luna_ok && flow_ok && ekf_ok;
          if(ready_now){
            if(flight_ready_since_ns==0) flight_ready_since_ns=now;
            if((now-flight_ready_since_ns)*1e-9>=kReadyStableSec){
              flight_ready=true;

              if(!startup_zero_done){
                {
                  std::lock_guard<std::mutex> l(fc.mu);
                  imu_dr::reset(fc.imu_dr_state);
                }

                web_raw_n=web_raw_e=0.0;
                web_raw_vn=web_raw_ve=0.0;
                web_raw_step_valid=false;

                startup_zero_done=true;
                ++rc_zero_seq;

                std::cerr
                  <<"STARTUP_AUTO_ZERO_V1: flight readiness stable "
                  <<kReadyStableSec
                  <<" s; current position accepted as 0/0/0"
                  <<" seq="<<rc_zero_seq<<"\n";
              }

              if(return_cli || blind4_cli){
                std::cerr<<"\nСИСТЕМА ГОТОВА.\n";
                if(blind4_cli){
                  std::cerr<<"BLIND4: ПОЛОЖИ БПЛА В ТОЧКУ A1. После полной остановки нажми SPACE.\n"
                           <<"GT В ЭТУ ПРОГРАММУ НЕ ВВОДИТЬ.\n";
                } else {
                  std::cerr<<"ПОЛОЖИ БПЛА В ТОЧКУ A.\n"
                           <<"После полной остановки нажми SPACE.\n";
                }
              } else {
                std::cerr<<"\n======================================================================\n"
                         <<"СИСТЕМА ГОТОВА\n"
                         <<"range="<<((bench_height_override>0.0)?bench_height_override:lm)
                         <<" m, flow valid, EKF velH valid"
                         <<", LOCAL="<<(efresh?"available":"pending")
                         <<", |vH|="<<(efresh?speed_h:-1.0)<<" m/s\n"
                         <<"Состояние было непрерывно стабильным "<<kReadyStableSec<<" с.\n"
                         <<"======================================================================\n";
              }
            }
          }else{
            flight_ready_since_ns=0;
            if(last_not_ready_print_ns==0 || now-last_not_ready_print_ns>1000000000LL){
              if(!(return_cli || blind4_cli)){
                std::cerr<<"\nНЕ ГОТОВО:"
                         <<" luna="<<(luna_ok?"OK":"NO")
                         <<" flow="<<(flow_ok?"OK":"NO")
                         <<" ekf="<<(ekf_ok?"OK":"NO")
                         <<" local="<<(efresh?"INFO":"PENDING")
                         <<" range="<<((bench_height_override>0.0)?bench_height_override:(hl?lm:-1.0))
                         <<" vH="<<(efresh?speed_h:-1.0)<<"\n";
              }
              last_not_ready_print_ns=now;
            }
          }
          if((now-flight_gate_begin_ns)*1e-9>kReadyTimeoutSec && !flight_ready){
            std::cerr<<"\nПРЕДУПРЕЖДЕНИЕ: СИСТЕМА ГОТОВА не достигнут за "
                     <<kReadyTimeoutSec<<" с. Publisher продолжает работать; взлёт не выполнять.\n";
            flight_gate_begin_ns=now;
          }
        }

        if((return_gui || return_cli || blind4_cli) && return_target_set && s.valid && flow_sent && dt>0.0 && dt<0.2){
          double hcam=0.0;
          if(bench_true_camera_height>0.0){
            hcam=bench_true_camera_height;
          } else if(hl){
            hcam=lm;
            if(std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
              hcam=lm-(diag_camera_z_m-diag_range_z_m);
            }
          }
          if(hcam>0.02){
            // RAW forensic integrates the physical camera measurement,
            // before any synthetic-range remapping used only for ArduPilot.
            double native_fx=s.flow_body_x, native_fy=s.flow_body_y;

            // Legacy/native LOS integral.
            return_raw_x += native_fx*hcam*dt;
            return_raw_y += native_fy*hcam*dt;

            // Mirror ArduPilot EKF3 optical-flow conventions:
            //   internal flowRadXY = -rawFlowRates
            //   flowRadXYcomp = flowRadXY + bodyRateXY
            //   losPred.x = v_body_y/range
            //   losPred.y = -v_body_x/range
            // Therefore:
            //   v_body_x = -flowComp.y * range
            //   v_body_y =  flowComp.x * range
            // Use FC ATTITUDE/gyro from the same camera interval average.
            if(fg_ok){
              const double comp_x=-native_fx + fg.x;
              const double comp_y=-native_fy + fg.y;
              const double dbx=(-comp_y)*hcam*dt;
              const double dby=( comp_x)*hcam*dt;
              return_body_dx += dbx;
              return_body_dy += dby;

              // Full 3-2-1 body-FRD -> NED rotation, planar body displacement z=0.
              const double cr=std::cos(fg.roll),  sr=std::sin(fg.roll);
              const double cp=std::cos(fg.pitch), sp=std::sin(fg.pitch);
              const double cy=std::cos(fg.yaw),   sy=std::sin(fg.yaw);
              const double r00=cy*cp;
              const double r01=cy*sp*sr-sy*cr;
              const double r10=sy*cp;
              const double r11=sy*sp*sr+cy*cr;
              return_ned_n += r00*dbx + r01*dby;
              return_ned_e += r10*dbx + r11*dby;

              // FB shadow metric integral on the exact same accepted camera interval.
              // Diagnostic only: never published to FC.
              if(g_fb_shadow_max_px>0.0 && s.fb_shadow_valid){
                const double fb_comp_x=-s.fb_flow_body_x + fg.x;
                const double fb_comp_y=-s.fb_flow_body_y + fg.y;
                const double fb_dbx=(-fb_comp_y)*hcam*dt;
                const double fb_dby=( fb_comp_x)*hcam*dt;
                return_fb_body_dx += fb_dbx;
                return_fb_body_dy += fb_dby;
                return_fb_ned_n += r00*fb_dbx + r01*fb_dby;
                return_fb_ned_e += r10*fb_dbx + r11*fb_dby;
              }
            }
          }
        }

        if(rotation_gui){
          cv::Mat hud(900,1500,CV_8UC3,cv::Scalar(18,18,18));

          // Before SPACE, still show that the graph is live by using the first
          // available FC position as a temporary preview origin. SPACE replaces
          // it with the operator-selected hover point and resets all counters.
          const bool gui_range_ok=hl && std::isfinite(lm) && lm>0.02 &&
                                  fg_ok && std::isfinite(fg.roll) && std::isfinite(fg.pitch);
          // TF-Luna measures along its own/body-down axis. For vertical Z on a flat
          // surface use the vertical component, otherwise roll/pitch alone would
          // look like a height change.
          const double gui_range_vertical=gui_range_ok
              ? lm*std::cos(fg.roll)*std::cos(fg.pitch)
              : std::numeric_limits<double>::quiet_NaN();

          if(efresh && !traj3d_preview_origin_set){
            traj3d_preview_n0=ep.x; traj3d_preview_e0=ep.y; traj3d_preview_z0=ep.z;
            traj3d_preview_origin_set=true;
          }
          if(gui_range_ok && !traj3d_preview_range_origin_set){
            traj3d_preview_range_vertical0=gui_range_vertical;
            traj3d_preview_range_origin_set=true;
          }

          // This GUI is intentionally a POSITION/HOVER monitor, not a rotation diagnostic.
          // SPACE defines the operator's hover reference from the FC's own LOCAL_POSITION_NED
          // estimate. Repeated SPACE replaces that reference and resets distance counters.
          if(traj3d_origin_set && efresh){
            const double z_up = -((double)ep.z-traj3d_z0);
            const cv::Vec3d p3(
              (double)ep.x-traj3d_n0,
              (double)ep.y-traj3d_e0,
              z_up); // 3D Z is EKF/baro vertical position; TF-Luna is surface distance only

            traj3d_peak_abs[0]=std::max(traj3d_peak_abs[0],std::abs(p3[0]));
            traj3d_peak_abs[1]=std::max(traj3d_peak_abs[1],std::abs(p3[1]));
            traj3d_peak_abs[2]=std::max(traj3d_peak_abs[2],std::abs(p3[2]));
            if(traj3d_prev_set){
              const cv::Vec3d dp=p3-traj3d_prev;
              if(cv::norm(dp)>=0.0005){
                traj3d_path_total += cv::norm(dp);
                traj3d_path_axis[0] += std::abs(dp[0]);
                traj3d_path_axis[1] += std::abs(dp[1]);
                traj3d_path_axis[2] += std::abs(dp[2]);
                traj3d_prev=p3;
              }
            } else {
              traj3d_prev=p3;
              traj3d_prev_set=true;
            }

            if(traj3d.empty() || cv::norm(p3-traj3d.back())>=0.002){
              traj3d.push_back(p3);
              while(traj3d.size()>1600) traj3d.pop_front();
            }
          }

          // ------------------------------------------------------------------
          // LEFT: large fixed-scale 3D plot. Range is ALWAYS +/-500 mm/axis.
          // ------------------------------------------------------------------
          const cv::Rect graph(20,20,960,850);
          cv::rectangle(hud,graph,cv::Scalar(24,24,24),cv::FILLED);
          cv::rectangle(hud,graph,cv::Scalar(95,95,95),1);

          putGuiText(hud,"ПОЛОЖЕНИЕ ОТ ТОЧКИ ЗАВИСАНИЯ",{45,55},0.72,cv::Scalar(240,240,240),1);
          putGuiText(hud,"фиксированный масштаб ±500 мм по X / Y / Z",{45,83},0.43,cv::Scalar(165,165,165),1);

          const cv::Point c3(500,500);
          const double sc3=500.0; // px/m, fixed. 500 mm = 250 px per single axis.
          auto proj3=[&](const cv::Vec3d& p)->cv::Point{
            const double n=std::clamp(p[0],-0.50,0.50);
            const double e=std::clamp(p[1],-0.50,0.50);
            const double u=std::clamp(p[2],-0.50,0.50);
            const int x=(int)std::lround(c3.x + (e-n)*0.70*sc3);
            const int y=(int)std::lround(c3.y + (e+n)*0.32*sc3 - u*0.95*sc3);
            return {x,y};
          };

          // Floor grid z=0, every 100 mm.
          for(int k=-5;k<=5;k++){
            const double v=0.1*k;
            cv::line(hud,proj3(cv::Vec3d(-0.5,v,0)),proj3(cv::Vec3d(0.5,v,0)),
                     k==0?cv::Scalar(85,85,85):cv::Scalar(48,48,48),1,cv::LINE_AA);
            cv::line(hud,proj3(cv::Vec3d(v,-0.5,0)),proj3(cv::Vec3d(v,0.5,0)),
                     k==0?cv::Scalar(85,85,85):cv::Scalar(48,48,48),1,cv::LINE_AA);
          }

          // Main axes, each from -500 to +500 mm.
          cv::line(hud,proj3(cv::Vec3d(-0.5,0,0)),proj3(cv::Vec3d(0.5,0,0)),cv::Scalar(210,170,75),2,cv::LINE_AA);
          cv::line(hud,proj3(cv::Vec3d(0,-0.5,0)),proj3(cv::Vec3d(0,0.5,0)),cv::Scalar(75,210,170),2,cv::LINE_AA);
          cv::line(hud,proj3(cv::Vec3d(0,0,-0.5)),proj3(cv::Vec3d(0,0,0.5)),cv::Scalar(180,180,245),2,cv::LINE_AA);

          putGuiText(hud,"X(N) +500",proj3(cv::Vec3d(0.5,0,0))+cv::Point(8,-4),0.38,cv::Scalar(210,170,75),1);
          putGuiText(hud,"X -500",proj3(cv::Vec3d(-0.5,0,0))+cv::Point(-75,18),0.38,cv::Scalar(210,170,75),1);
          putGuiText(hud,"Y(E) +500",proj3(cv::Vec3d(0,0.5,0))+cv::Point(8,-4),0.38,cv::Scalar(75,210,170),1);
          putGuiText(hud,"Y -500",proj3(cv::Vec3d(0,-0.5,0))+cv::Point(-75,18),0.38,cv::Scalar(75,210,170),1);
          putGuiText(hud,"Z +500",proj3(cv::Vec3d(0,0,0.5))+cv::Point(8,0),0.38,cv::Scalar(180,180,245),1);
          putGuiText(hud,"Z -500",proj3(cv::Vec3d(0,0,-0.5))+cv::Point(8,16),0.38,cv::Scalar(180,180,245),1);

          // Hover/reference point is always the graph origin.
          cv::circle(hud,c3,12,cv::Scalar(0,220,0),2,cv::LINE_AA);
          cv::circle(hud,c3,3,cv::Scalar(0,255,0),cv::FILLED,cv::LINE_AA);
          putGuiText(hud,"ТОЧКА ЗАВИСАНИЯ",{c3.x+16,c3.y-10},0.40,cv::Scalar(0,220,0),1);

          cv::Vec3d p3(0,0,0);
          const bool have_locked_p3=traj3d_origin_set&&efresh;
          const bool have_preview_p3=!traj3d_origin_set&&traj3d_preview_origin_set&&efresh;
          const bool have_p3=have_locked_p3||have_preview_p3;
          if(have_locked_p3){
            const double z_up=-((double)ep.z-traj3d_z0);
            p3=cv::Vec3d((double)ep.x-traj3d_n0,
                         (double)ep.y-traj3d_e0,
                         z_up);
          } else if(have_preview_p3){
            const double z_up=-((double)ep.z-traj3d_preview_z0);
            p3=cv::Vec3d((double)ep.x-traj3d_preview_n0,
                         (double)ep.y-traj3d_preview_e0,
                         z_up);
          }

          if(have_p3){
            if(have_locked_p3){
              for(size_t i=1;i<traj3d.size();++i)
                cv::line(hud,proj3(traj3d[i-1]),proj3(traj3d[i]),cv::Scalar(0,170,255),2,cv::LINE_AA);
            }

            const cv::Point cur=proj3(p3);
            cv::circle(hud,cur,11,have_locked_p3?cv::Scalar(0,255,255):cv::Scalar(220,220,220),cv::FILLED,cv::LINE_AA);
            cv::line(hud,c3,cur,cv::Scalar(90,90,90),1,cv::LINE_AA);

            const bool outside=std::abs(p3[0])>0.5||std::abs(p3[1])>0.5||std::abs(p3[2])>0.5;
            if(outside)
              putGuiText(hud,"ВНЕ ДИАПАЗОНА ±500 мм",{45,118},0.52,cv::Scalar(0,80,255),1);
          }

          // High-resolution center inset. The main graph stays fixed at ±500 mm,
          // while this inset makes small false motion from roll/pitch visible.
          const cv::Rect zoom(690,105,260,260);
          cv::rectangle(hud,zoom,cv::Scalar(20,20,20),cv::FILLED);
          cv::rectangle(hud,zoom,cv::Scalar(100,100,100),1);
          putGuiText(hud,"ЦЕНТР ±100 мм",{zoom.x+12,zoom.y+24},0.38,cv::Scalar(190,190,190),1);
          const cv::Point zc(zoom.x+zoom.width/2,zoom.y+zoom.height/2+10);
          const double zsc=900.0; // 100 mm = 90 px per horizontal axis
          auto projZoom=[&](const cv::Vec3d& p)->cv::Point{
            const double n=std::clamp(p[0],-0.10,0.10);
            const double e=std::clamp(p[1],-0.10,0.10);
            const double u=std::clamp(p[2],-0.10,0.10);
            return {(int)std::lround(zc.x+(e-n)*0.55*zsc),
                    (int)std::lround(zc.y+(e+n)*0.23*zsc-u*0.75*zsc)};
          };
          for(int k=-2;k<=2;k++){
            const double v=0.05*k;
            cv::line(hud,projZoom(cv::Vec3d(-0.1,v,0)),projZoom(cv::Vec3d(0.1,v,0)),cv::Scalar(42,42,42),1,cv::LINE_AA);
            cv::line(hud,projZoom(cv::Vec3d(v,-0.1,0)),projZoom(cv::Vec3d(v,0.1,0)),cv::Scalar(42,42,42),1,cv::LINE_AA);
          }
          cv::line(hud,projZoom(cv::Vec3d(-0.1,0,0)),projZoom(cv::Vec3d(0.1,0,0)),cv::Scalar(130,105,55),1,cv::LINE_AA);
          cv::line(hud,projZoom(cv::Vec3d(0,-0.1,0)),projZoom(cv::Vec3d(0,0.1,0)),cv::Scalar(55,130,105),1,cv::LINE_AA);
          cv::line(hud,projZoom(cv::Vec3d(0,0,-0.1)),projZoom(cv::Vec3d(0,0,0.1)),cv::Scalar(120,120,170),1,cv::LINE_AA);
          cv::circle(hud,zc,5,cv::Scalar(0,210,0),1,cv::LINE_AA);
          if(have_p3){
            cv::circle(hud,projZoom(p3),7,cv::Scalar(0,255,255),cv::FILLED,cv::LINE_AA);
          }

          // ------------------------------------------------------------------
          // RIGHT: only information needed for this PosHold experiment.
          // ------------------------------------------------------------------
          const cv::Rect info(1000,20,480,850);
          cv::rectangle(hud,info,cv::Scalar(24,24,24),cv::FILLED);
          cv::rectangle(hud,info,cv::Scalar(95,95,95),1);

          putGuiText(hud,"POSHOLD — КОНТРОЛЬ ТОЧКИ",{1025,55},0.62,cv::Scalar(240,240,240),1);

          std::string arm_text="ARM: НЕТ ДАННЫХ";
          cv::Scalar arm_col(0,170,255);
          if(arm_ok){
            arm_text=arm_now?"ARM: ВКЛЮЧЕН (МОТОРЫ РАЗРЕШЕНЫ)":"ARM: ВЫКЛЮЧЕН";
            arm_col=arm_now?cv::Scalar(0,220,0):cv::Scalar(190,190,190);
          }
          putGuiText(hud,arm_text,{1025,92},0.48,arm_col,1);

          if(have_p3){
            const double xmm=p3[0]*1000.0, ymm=p3[1]*1000.0, zmm=p3[2]*1000.0;
            std::ostringstream pos;
            pos<<std::fixed<<std::setprecision(0)<<std::showpos
               <<"X "<<xmm<<" мм   Y "<<ymm<<" мм   Z "<<zmm<<" мм"<<std::noshowpos;
            putGuiText(hud,"ТЕКУЩЕЕ ОТКЛОНЕНИЕ:",{1025,137},0.46,cv::Scalar(180,180,180),1);
            putGuiText(hud,pos.str(),{1025,172},0.64,cv::Scalar(255,255,255),1);
            std::ostringstream zsrc;
            zsrc<<std::fixed<<std::setprecision(0)
                <<"Z = оценка FC (EKF)";
            if(traj3d_range_origin_set && gui_range_ok){
              const double agl_delta_mm=(gui_range_vertical-traj3d_range_vertical0)*1000.0;
              zsrc<<"   |   Δ до поверхности "
                  <<std::showpos<<agl_delta_mm<<" мм"<<std::noshowpos;
            }
            putGuiText(hud,zsrc.str(),{1025,198},0.34,cv::Scalar(155,155,155),1);

            std::ostringstream ret;
            ret<<std::fixed<<std::setprecision(0)<<std::showpos
               <<"X "<<(-xmm)<<"   Y "<<(-ymm)<<"   Z "<<(-zmm)<<" мм"<<std::noshowpos;
            putGuiText(hud,"ДЛЯ ВОЗВРАТА К НУЛЮ:",{1025,230},0.46,cv::Scalar(180,180,180),1);
            putGuiText(hud,ret.str(),{1025,262},0.61,
                       cv::norm(p3)<0.015?cv::Scalar(0,255,0):cv::Scalar(0,220,255),1);

            std::ostringstream dist;
            dist<<std::fixed<<std::setprecision(0)
                <<"РАССТОЯНИЕ ОТ ТОЧКИ: "<<cv::norm(p3)*1000.0<<" мм";
            putGuiText(hud,dist.str(),{1025,310},0.48,cv::Scalar(220,220,220),1);

            std::ostringstream walked;
            walked<<std::fixed<<std::setprecision(0)
                  <<"ВСЕГО: "<<traj3d_path_total*1000.0<<" мм";
            putGuiText(hud,"ПРОЙДЕННЫЙ ПУТЬ:",{1025,360},0.46,cv::Scalar(180,180,180),1);
            putGuiText(hud,walked.str(),{1025,392},0.50,cv::Scalar(230,230,230),1);

            std::ostringstream axes;
            axes<<std::fixed<<std::setprecision(0)
                <<"X "<<traj3d_path_axis[0]*1000.0
                <<"   Y "<<traj3d_path_axis[1]*1000.0
                <<"   Z "<<traj3d_path_axis[2]*1000.0<<" мм";
            putGuiText(hud,axes.str(),{1025,422},0.46,cv::Scalar(210,210,210),1);
            std::ostringstream peaks;
            peaks<<std::fixed<<std::setprecision(0)
                 <<"МАКС. ОТКЛОНЕНИЕ: X "<<traj3d_peak_abs[0]*1000.0
                 <<"  Y "<<traj3d_peak_abs[1]*1000.0
                 <<"  Z "<<traj3d_peak_abs[2]*1000.0<<" мм";
            putGuiText(hud,peaks.str(),{1025,444},0.34,cv::Scalar(170,170,170),1);
          } else {
            putGuiText(hud,"ТОЧКА ЗАВИСАНИЯ НЕ ЗАДАНА",{1025,145},0.48,cv::Scalar(0,210,255),1);
            putGuiText(hud,"Нажмите SPACE в нужной физической точке.",{1025,178},0.43,cv::Scalar(220,220,220),1);
            if(have_preview_p3){
              const double xmm=p3[0]*1000.0, ymm=p3[1]*1000.0, zmm=p3[2]*1000.0;
              std::ostringstream prev;
              prev<<std::fixed<<std::setprecision(0)<<std::showpos
                  <<"живое превью: X "<<xmm<<"  Y "<<ymm<<"  Z "<<zmm<<" мм"<<std::noshowpos;
              putGuiText(hud,prev.str(),{1025,211},0.41,cv::Scalar(180,180,180),1);
            }
          }

          FlowFcTarget ct{}; FlowFcAttTarget ca{}; FlowFcOutputs co{};
          double ct_age=1e9,ca_age=1e9,co_age=1e9;
          fc.latestControl(&ct,&ca,&co,&ct_age,&ca_age,&co_age);
          const bool ca_ok=ca.valid&&ca_age<500.0;
          const bool co_ok=co.valid&&co_age<500.0;

          putGuiText(hud,"КОМАНДА FC:",{1025,468},0.46,cv::Scalar(180,180,180),1);
          if(ca_ok){
            std::ostringstream at;
            at<<std::fixed<<std::setprecision(1)
              <<"крен "<<ca.roll*180.0/M_PI<<"°   тангаж "<<ca.pitch*180.0/M_PI<<"°";
            putGuiText(hud,at.str(),{1025,496},0.50,cv::Scalar(230,230,230),1);
          }else{
            putGuiText(hud,"крен/тангаж: нет данных",{1025,496},0.46,cv::Scalar(150,150,150),1);
          }

          if(co_ok){
            std::ostringstream motors;
            motors<<"M1 "<<co.pwm[0]<<"  M2 "<<co.pwm[1]
                  <<"  M3 "<<co.pwm[2]<<"  M4 "<<co.pwm[3];
            putGuiText(hud,motors.str(),{1025,527},0.45,cv::Scalar(0,220,0),1);
          }else{
            putGuiText(hud,"M1..M4: нет данных",{1025,514},0.45,cv::Scalar(150,150,150),1);
          }

          const bool posrel_ok=esfresh && (es.flags & EKF_POS_HORIZ_REL);
          const bool velh_ok=esfresh && (es.flags & EKF_VELOCITY_HORIZ);
          std::ostringstream health;
          health<<"ОЦЕНКА FC: "<<((posrel_ok&&velh_ok)?"OK":"НЕТ ПОЗИЦИИ")
                <<"   OF "<<(s.valid?"OK":"BAD")
                <<"   до поверхности "<<std::fixed<<std::setprecision(2)<<(hl?lm:-1.0)<<" м";
          putGuiText(hud,health.str(),{1025,562},0.43,
                     (posrel_ok&&velh_ok)?cv::Scalar(0,220,0):cv::Scalar(0,80,255),1);

          // Compact live camera preview. Previous 430x322 image did not fit into
          // the 900px HUD at y=625, so it was silently not drawn.
          putGuiText(hud,"КАМЕРА OV9281",{1025,600},0.42,cv::Scalar(190,190,190),1);
          cv::Mat cam_bgr,cam_view;
          cv::cvtColor(gray,cam_bgr,cv::COLOR_GRAY2BGR);
          const int cam_w=300;
          const int cam_h=(int)std::lround((double)cam_bgr.rows*cam_w/cam_bgr.cols);
          cv::resize(cam_bgr,cam_view,cv::Size(cam_w,cam_h),0,0,cv::INTER_AREA);
          const int cam_x=1025, cam_y=614;
          cam_view.copyTo(hud(cv::Rect(cam_x,cam_y,cam_w,cam_h)));
          const int rx0=cam_x+(int)std::lround(g_feature_roi.x0*cam_w);
          const int ry0=cam_y+(int)std::lround(g_feature_roi.y0*cam_h);
          const int rx1=cam_x+(int)std::lround(g_feature_roi.x1*cam_w);
          const int ry1=cam_y+(int)std::lround(g_feature_roi.y1*cam_h);
          cv::rectangle(hud,cv::Point(rx0,ry0),cv::Point(rx1,ry1),cv::Scalar(0,255,255),1,cv::LINE_AA);

          putGuiText(hud,"SPACE — задать/сменить точку      Q / ESC — выход",
                     {1025,862},0.38,cv::Scalar(170,170,170),1);

          cv::imshow(rotation_window_name,hud);
          const int rkey=cv::waitKey(1)&0xff;
          if(rkey==' ' && efresh){
            traj3d_n0=ep.x; traj3d_e0=ep.y; traj3d_z0=ep.z;
            traj3d_preview_n0=ep.x; traj3d_preview_e0=ep.y; traj3d_preview_z0=ep.z;
            traj3d_preview_origin_set=true;
            if(gui_range_ok){
              traj3d_range_vertical0=gui_range_vertical;
              traj3d_range_origin_set=true;
              traj3d_preview_range_vertical0=gui_range_vertical;
              traj3d_preview_range_origin_set=true;
            } else {
              traj3d_range_origin_set=false;
            }
            traj3d_origin_set=true;
            traj3d.clear();
            traj3d.emplace_back(0.0,0.0,0.0);
            traj3d_prev=cv::Vec3d(0,0,0);
            traj3d_prev_set=true;
            traj3d_path_total=0.0;
            traj3d_path_axis=cv::Vec3d(0,0,0);
            traj3d_peak_abs=cv::Vec3d(0,0,0);
            std::cerr<<"3D GUI HOVER POINT: current FC estimate accepted as X/Y/Z = 0/0/0; path counters reset\n";
          } else if(rkey=='q'||rkey=='Q'||rkey==27){
            g_running=false;
          }
        }

        if(blind4_cli){
          const int key=cli_terminal.readKey();
          if(key=='q' || key=='Q' || key==27){
            std::cerr<<"\nBLIND4: отменено оператором.\n";
            g_running=false;
          } else if(key==' ' && efresh && blind4_state<8){
            const bool is_a=(blind4_state%2)==0;
            const int leg=blind4_state/2+1;
            if(is_a){
              return_target_n=ep.x; return_target_e=ep.y;
              return_target_set=true; return_trail.clear();
              return_raw_x=return_raw_y=0.0;
              return_body_dx=return_body_dy=0.0;
              return_ned_n=return_ned_e=0.0;
              return_fb_body_dx=return_fb_body_dy=0.0;
              return_fb_ned_n=return_fb_ned_e=0.0;
              return_b_marked=false; return_home_marked=false;
              if(fg_ok){ return_yaw0=fg.yaw; return_yaw0_set=true; }
              pending_return_event=9+2*leg;
              ++blind4_state;
              std::cerr<<"\nBLIND4 A"<<leg<<" ЗАФИКСИРОВАНА.\n"
                       <<"Выполни проход A"<<leg<<" -> B"<<leg
                       <<", полностью остановись и нажми SPACE.\n";
            } else {
              pending_return_event=10+2*leg;
              ++blind4_state;
              std::cerr<<"\nBLIND4 B"<<leg<<" ЗАФИКСИРОВАНА.\n";
              if(leg==4){
                // Do not stop in the key-handler: pending_return_event=18 must
                // survive until the next CSV row is written. The CSV writer
                // clears pending_return_event only after persisting it.
                std::cerr<<"BLIND4 B4 ЗАФИКСИРОВАНА. Финализация записи...\n";
              } else {
                std::cerr<<"Измерь GT"<<leg<<" физически и запиши ОТДЕЛЬНО (не вводи сюда).\n"
                         <<"Поставь аппарат в удобную точку A"<<(leg+1)
                         <<", полностью остановись и нажми SPACE.\n";
              }
            }
          }
        }

        if(return_cli){
          const int key=cli_terminal.readKey();

          if(canonical_state==0 && key==' ' && efresh){
            return_target_n=ep.x; return_target_e=ep.y;
            return_target_set=true; return_trail.clear();
            return_view_halfspan_m=0.50;
            return_raw_x=return_raw_y=0.0;
            return_body_dx=return_body_dy=0.0;
            return_ned_n=return_ned_e=0.0;
            return_fb_body_dx=return_fb_body_dy=0.0;
            return_fb_ned_n=return_fb_ned_e=0.0;
            return_b_marked=false;
            return_home_marked=false;
            canonical_gt_buf.clear(); canonical_gt_mm=0.0;
            if(fg_ok){ return_yaw0=fg.yaw; return_yaw0_set=true; }
            pending_return_event=1;
            canonical_state=1;
            std::cerr<<"\nТОЧКА A ЗАФИКСИРОВАНА.\n"
                     <<"Двигай БПЛА по столу в B. После полной остановки нажми SPACE.\n";

          } else if(canonical_state==1 && key==' ' && efresh){
            return_b_marked=true;
            return_b_n=ep.x; return_b_e=ep.y;
            return_b_raw_x=return_raw_x; return_b_raw_y=return_raw_y;
            return_b_body_dx=return_body_dx; return_b_body_dy=return_body_dy;
            return_b_ned_n=return_ned_n; return_b_ned_e=return_ned_e;
            return_b_fb_body_dx=return_fb_body_dx; return_b_fb_body_dy=return_fb_body_dy;
            return_b_fb_ned_n=return_fb_ned_n; return_b_fb_ned_e=return_fb_ned_e;
            return_b_yaw=fg_ok?fg.yaw:0.0;
            pending_return_event=2;
            canonical_state=2;
            canonical_gt_buf.clear();
            std::cerr<<"\nТОЧКА B ЗАФИКСИРОВАНА. БПЛА НЕ ДВИГАТЬ.\n"
                     <<"Измерь физическое A->B и введи расстояние в мм, затем ENTER.\n"
                     <<"GT mm: "<<std::flush;

          } else if(canonical_state==2){
            if((key>='0'&&key<='9') || key=='.' || key==','){
              const char ch=(key==',')?'.':(char)key;
              canonical_gt_buf.push_back(ch);
              std::cerr<<ch<<std::flush;
            } else if((key==8 || key==127) && !canonical_gt_buf.empty()){
              canonical_gt_buf.pop_back();
              std::cerr<<"\b \b"<<std::flush;
            } else if(key=='\r' || key=='\n'){
              try{ canonical_gt_mm=std::stod(canonical_gt_buf); }catch(...){ canonical_gt_mm=0.0; }
              if(canonical_gt_mm>=50.0 && canonical_gt_mm<=2000.0){
                canonical_state=3;
                std::cerr<<"\nGT ПРИНЯТ: "<<canonical_gt_mm<<" мм.\n"
                         <<"Нажми SPACE, затем возвращай БПЛА в физическую точку A.\n";
              } else {
                canonical_gt_buf.clear(); canonical_gt_mm=0.0;
                std::cerr<<"\nОШИБКА: расстояние должно быть 50..2000 мм. Введи заново.\nGT mm: "<<std::flush;
              }
            }

          } else if(canonical_state==3 && key==' '){
            canonical_state=4;
            std::cerr<<"\nОБРАТНЫЙ ПРОХОД НАЧАТ. Верни БПЛА в A.\n"
                     <<"После полной остановки нажми SPACE.\n";

          } else if(canonical_state==4 && key==' ' && efresh){
            pending_return_event=3;
            return_home_marked=true;

            // Canonical metric must measure the same native optical-flow quantity
            // that is published by the frozen production estimator.  Do NOT use
            // gyro-compensated return_ned_* here: that is a separate EKF forensic
            // diagnostic and previously produced a misleading 31..137 mm report.
            auto metric=[&](const char* name,
                            double ab_x,double ab_y,double total_x,double total_y){
              const double ab=1000.0*std::hypot(ab_x,ab_y);
              const double ba=1000.0*std::hypot(total_x-ab_x,total_y-ab_y);
              const double close=1000.0*std::hypot(total_x,total_y);
              const double ab_err=ab-canonical_gt_mm;
              const double ba_err=ba-canonical_gt_mm;
              std::cerr<<name<<"\n"
                       <<"  A->B: X/Y=("<<ab_x*1000.0<<", "<<ab_y*1000.0<<") mm  mag="<<ab
                       <<" mm  error="<<ab_err<<" mm ("<<(100.0*ab_err/canonical_gt_mm)<<" %)\n"
                       <<"  B->A: X/Y=("<<(total_x-ab_x)*1000.0<<", "<<(total_y-ab_y)*1000.0
                       <<") mm  mag="<<ba<<" mm  error="<<ba_err<<" mm ("<<(100.0*ba_err/canonical_gt_mm)<<" %)\n"
                       <<"  CLOSURE: X/Y=("<<total_x*1000.0<<", "<<total_y*1000.0
                       <<") mm  mag="<<close<<" mm ("<<(100.0*close/canonical_gt_mm)<<" % GT)\n";
            };

            std::cerr<<"\n======================================================================\n"
                     <<"CANONICAL NATIVE METRIC RESULT\n"
                     <<"PHYSICAL GT A->B = "<<canonical_gt_mm<<" mm\n"
                     <<"Metric: native production flow_body * real TF-Luna camera height * dt\n"
                     <<"FB shadow: NOT USED in this result\n"
                     <<"======================================================================\n";
            metric("NATIVE",return_b_raw_x,return_b_raw_y,return_raw_x,return_raw_y);
            std::cerr<<"---------------------------------------------------------------------\n"
                     <<"GYRO-COMPENSATED EKF FORENSIC (diagnostic only; NOT metric result)\n";
            metric("EKF_FORENSIC",return_b_ned_n,return_b_ned_e,return_ned_n,return_ned_e);
            std::cerr<<"======================================================================\n";
            canonical_state=5;
            std::cerr<<"ТЕСТ ЗАВЕРШЁН.\n";
            g_running=false;
          }
        }

        if(return_gui){
          if(flight_ready && efresh && !return_target_set && !return_manual_target){
            return_target_n=ep.x;
            return_target_e=ep.y;
            return_target_set=true;
            return_trail.clear();
            return_raw_x=return_raw_y=0.0;
            return_body_dx=return_body_dy=0.0;
            return_ned_n=return_ned_e=0.0;
            return_b_marked=false;
            return_home_marked=false;
            if(fg_ok){ return_yaw0=fg.yaw; return_yaw0_set=true; }
            pending_return_event=1;
            std::cerr<<"RETURN GUI TARGET SET: N="<<return_target_n<<" E="<<return_target_e
                     <<" yaw_deg="<<(return_yaw0_set?return_yaw0*180.0/M_PI:0.0)<<"\n";
          }

          // Screen-recording HUD: trajectory stays on the left; the live OV9281
          // image is shown on the right with the exact feature ROI used by KLT.
          cv::Mat hud(900,1500,CV_8UC3,cv::Scalar(20,20,20));
          const cv::Point center(450,450);

          // Крупная русская инструкция оператору — её видно на записи экрана.
          std::string step_title, step_line1, step_line2;
          cv::Scalar step_color(220,220,220);
          if(!flight_ready){
            step_title="ШАГ 1 / 5 — ЖДИТЕ ГОТОВНОСТИ";
            step_line1="Держите аппарат неподвижно. Тест пока не начинайте.";
            step_line2="После ГОТОВНОСТИ поднимите аппарат на удобную высоту.";
            step_color=cv::Scalar(0,200,255);
          } else if(!return_target_set){
            step_title="ШАГ 2 / 5 — ЗАДАЙТЕ ФИЗИЧЕСКУЮ ТОЧКУ A";
            step_line1="Поднимите аппарат и удерживайте его неподвижно 2–3 секунды.";
            step_line2="Нажмите SPACE. Эта позиция станет точкой A / ДОМОЙ.";
            step_color=cv::Scalar(0,255,255);
          } else if(!return_b_marked){
            step_title="ШАГ 3 / 5 — ПЕРЕНЕСИТЕ A → B";
            step_line1="Перенесите аппарат примерно на 200–400 мм с естественными roll/pitch/yaw.";
            step_line2="Без резких рывков. В B остановитесь на 2–3 секунды и нажмите B.";
            step_color=cv::Scalar(0,255,0);
          } else if(!return_home_marked){
            step_title="ШАГ 4 / 5 — ФИЗИЧЕСКИ ВЕРНИТЕСЬ В A";
            step_line1="Вернитесь в реальную исходную точку естественным движением, НЕ по метке EKF.";
            step_line2="Полностью остановитесь на 3–5 секунд и нажмите H.";
            step_color=cv::Scalar(0,180,255);
          } else {
            step_title="ШАГ 5 / 5 — ТЕСТ ЗАВЕРШЁН";
            step_line1="Ещё 2–3 секунды держите аппарат неподвижно.";
            step_line2="Результат уже записан. Нажмите Q или ESC для выхода.";
            step_color=cv::Scalar(255,255,0);
          }
          cv::rectangle(hud,cv::Rect(25,255,840,105),cv::Scalar(30,30,30),cv::FILLED);
          cv::rectangle(hud,cv::Rect(25,255,840,105),step_color,2);
          putGuiText(hud,step_title,{45,285},0.72,step_color,2);
          putGuiText(hud,step_line1,{45,318},0.48,cv::Scalar(230,230,230),1);
          putGuiText(hud,step_line2,{45,345},0.48,cv::Scalar(230,230,230),1);
          cv::line(hud,{450,45},{450,855},cv::Scalar(70,70,70),1);
          cv::line(hud,{45,450},{855,450},cv::Scalar(70,70,70),1);
          cv::circle(hud,center,16,cv::Scalar(0,220,0),2);
          cv::line(hud,{435,450},{465,450},cv::Scalar(0,220,0),2);
          cv::line(hud,{450,435},{450,465},cv::Scalar(0,220,0),2);

          double dn=0.0,de=0.0,dist=0.0,vh=0.0;
          if(return_target_set && efresh){
            dn=(double)ep.x-return_target_n;
            de=(double)ep.y-return_target_e;
            dist=std::hypot(dn,de);
            vh=std::hypot((double)ep.vx,(double)ep.vy);

            return_view_halfspan_m=std::max(0.30,std::max(return_view_halfspan_m*0.999,
                                      1.20*std::max(std::abs(dn),std::abs(de))));
            return_view_halfspan_m=std::min(return_view_halfspan_m,5.0);
            const double px_per_m=360.0/return_view_halfspan_m;
            const cv::Point cur(
              std::clamp((int)std::lround(center.x+de*px_per_m),50,850),
              std::clamp((int)std::lround(center.y-dn*px_per_m),50,850));

            return_trail.emplace_back(de,dn);
            while(return_trail.size()>1200)return_trail.pop_front();
            for(size_t ti=1;ti<return_trail.size();++ti){
              const cv::Point a(
                std::clamp((int)std::lround(center.x+return_trail[ti-1].x*px_per_m),50,850),
                std::clamp((int)std::lround(center.y-return_trail[ti-1].y*px_per_m),50,850));
              const cv::Point bpt(
                std::clamp((int)std::lround(center.x+return_trail[ti].x*px_per_m),50,850),
                std::clamp((int)std::lround(center.y-return_trail[ti].y*px_per_m),50,850));
              cv::line(hud,a,bpt,cv::Scalar(110,110,110),1);
            }
            cv::circle(hud,cur,10,cv::Scalar(0,180,255),-1);
            cv::arrowedLine(hud,cur,center,cv::Scalar(0,220,255),3,cv::LINE_AA,0,0.08);

            // Cyan RAW-NED point: independent optical-flow+gyro+attitude integration.
            const cv::Point raw_cur(
              std::clamp((int)std::lround(center.x+return_ned_e*px_per_m),50,850),
              std::clamp((int)std::lround(center.y-return_ned_n*px_per_m),50,850));
            cv::circle(hud,raw_cur,8,cv::Scalar(255,255,0),2,cv::LINE_AA);

            if(dist<=0.025){
              cv::circle(hud,center,34,cv::Scalar(0,255,0),3);
              putGuiText(hud,"ТОЧКА ДОСТИГНУТА",{285,95},1.0,cv::Scalar(0,255,0),3);
            }
          }

          const double raw_closure_mm=1000.0*std::hypot(return_raw_x,return_raw_y);
          const double raw_ned_closure_mm=1000.0*std::hypot(return_ned_n,return_ned_e);
          const double raw_body_closure_mm=1000.0*std::hypot(return_body_dx,return_body_dy);
          const double roll_deg=fg_ok?fg.roll*180.0/M_PI:0.0;
          const double pitch_deg=fg_ok?fg.pitch*180.0/M_PI:0.0;
          const double yaw_deg=fg_ok?fg.yaw*180.0/M_PI:0.0;
          const double dyaw_deg=(fg_ok&&return_yaw0_set)?std::remainder(fg.yaw-return_yaw0,2.0*M_PI)*180.0/M_PI:0.0;
          std::ostringstream l1,l2,l3,l4,l5,l6,l7,l8,l9,l10;
          l1<<std::fixed<<std::setprecision(0)<<"ДО ЦЕЛИ: "<<dist*1000.0<<" мм";
          l2<<std::fixed<<std::setprecision(1)<<"ОШИБКА N: "<<dn*1000.0<<" мм";
          l3<<std::fixed<<std::setprecision(1)<<"ОШИБКА E: "<<de*1000.0<<" мм";
          l4<<std::fixed<<std::setprecision(3)<<"СКОРОСТЬ XY: "<<vh<<" м/с   ДАЛЬНОМЕР: "<<(hl?lm:-1.0)<<" м";
          l5<<std::fixed<<std::setprecision(2)<<"МАСШТАБ: ±"<<return_view_halfspan_m<<" м";
          l6<<std::fixed<<std::setprecision(1)<<"RAW LOS, старый: "<<raw_closure_mm<<" мм";
          l7<<std::fixed<<std::setprecision(1)<<"КУРС: "<<yaw_deg<<"°   ΔКУРС(A): "<<dyaw_deg<<"°";
          l8<<std::fixed<<std::setprecision(1)<<"RAW NED, замыкание: "<<raw_ned_closure_mm
            <<" мм   ΔN/E "<<return_ned_n*1000.0<<"/"<<return_ned_e*1000.0;
          l9<<std::fixed<<std::setprecision(1)<<"RAW BODY: "<<raw_body_closure_mm
            <<" мм   ΔX/Y "<<return_body_dx*1000.0<<"/"<<return_body_dy*1000.0;
          l10<<std::fixed<<std::setprecision(1)<<"КРЕН/ТАНГАЖ/КУРС: "<<roll_deg<<"/"<<pitch_deg<<"/"<<yaw_deg
             <<"°   TF-Luna: "<<(hl?lm:-1.0)<<" м";
          putGuiText(hud,return_target_set?l1.str():"ОЖИДАНИЕ ГОТОВНОСТИ / ТОЧКИ A",{35,40},0.85,cv::Scalar(240,240,240),2);
          putGuiText(hud,"6-DoF РЕГРЕССИЯ • CAP=500 • BRIDGE=OFF",{920,40},0.50,cv::Scalar(170,220,255),1);
          if(bench_height_override>0.0){
            putGuiText(hud,"РУЧНОЙ ТЕСТ • FC RANGE=0.60 м • МАСШТАБ ПО REAL TF-LUNA",
                       {920,70},0.48,cv::Scalar(0,180,255),2);
          }
          putGuiText(hud,l2.str(),{35,75},0.65,cv::Scalar(220,220,220),2);
          putGuiText(hud,l3.str(),{35,105},0.65,cv::Scalar(220,220,220),2);
          putGuiText(hud,l6.str(),{35,140},0.58,cv::Scalar(190,190,190),1);
          putGuiText(hud,l8.str(),{35,172},0.62,cv::Scalar(0,220,255),2);
          putGuiText(hud,l9.str(),{35,204},0.55,cv::Scalar(190,190,190),1);
          putGuiText(hud,l7.str(),{35,236},0.58,cv::Scalar(190,190,190),1);
          putGuiText(hud,l10.str(),{35,382},0.52,
                      (hl&&lm<0.20)?cv::Scalar(0,80,255):cv::Scalar(190,190,190),1);
          if(hl&&lm<0.20){
            putGuiText(hud,"TF-LUNA < 0,20 м: показания могут быть ненадёжны",{35,410},
                        0.50,cv::Scalar(0,80,255),1);
          }
          putGuiText(hud,l4.str(),{35,850},0.58,cv::Scalar(200,200,200),1);
          putGuiText(hud,l5.str(),{650,850},0.52,cv::Scalar(180,180,180),1);
          cv::putText(hud,"N",{458,65},cv::FONT_HERSHEY_SIMPLEX,0.65,cv::Scalar(160,160,160),2,cv::LINE_AA);
          cv::putText(hud,"E",{825,440},cv::FONT_HERSHEY_SIMPLEX,0.65,cv::Scalar(160,160,160),2,cv::LINE_AA);
          putGuiText(hud,"SPACE: ТОЧКА A   B: ДАЛЬНЯЯ   H: ВОЗВРАТ   C: ОЧИСТИТЬ   Q/ESC: ВЫХОД",{35,885},
                      0.50,cv::Scalar(160,160,160),1);

          // Live camera panel. Use the already decoded frame so this does not
          // open a second V4L2 stream or alter the optical-flow pipeline.
          cv::Mat cam_bgr,cam_view;
          cv::cvtColor(gray,cam_bgr,cv::COLOR_GRAY2BGR);
          const int cam_w=560;
          const int cam_h=(int)std::lround((double)cam_bgr.rows*cam_w/cam_bgr.cols);
          cv::resize(cam_bgr,cam_view,cv::Size(cam_w,cam_h),0,0,cv::INTER_AREA);
          const int cam_x=920;
          const int cam_y=105;
          if(cam_y+cam_h<=hud.rows && cam_x+cam_w<=hud.cols){
            cam_view.copyTo(hud(cv::Rect(cam_x,cam_y,cam_w,cam_h)));
            const int rx0=cam_x+(int)std::lround(g_feature_roi.x0*cam_w);
            const int ry0=cam_y+(int)std::lround(g_feature_roi.y0*cam_h);
            const int rx1=cam_x+(int)std::lround(g_feature_roi.x1*cam_w);
            const int ry1=cam_y+(int)std::lround(g_feature_roi.y1*cam_h);
            cv::rectangle(hud,cv::Point(rx0,ry0),cv::Point(rx1,ry1),
                          cv::Scalar(0,255,255),2,cv::LINE_AA);
            putGuiText(hud,"OV9281 — ЖИВОЕ ВИДЕО",{cam_x,70},0.80,
                        cv::Scalar(240,240,240),2);
            putGuiText(hud,"ЖЁЛТАЯ РАМКА — ОБЛАСТЬ KLT",{cam_x,cam_y+cam_h+32},
                        0.55,cv::Scalar(0,255,255),1);
            std::ostringstream cam_diag;
            cam_diag<<"КАДР "<<frame<<"   ВАЛИДЕН "<<(s.valid?1:0)
                    <<"   ИНЛАЙЕРЫ "<<s.inliers<<"/"<<s.tracked;
            putGuiText(hud,cam_diag.str(),{cam_x,cam_y+cam_h+62},
                        0.52,cv::Scalar(210,210,210),1);
          }

          cv::imshow(return_window_name,hud);
          const int key=cv::waitKey(1)&0xff;
          if(key==' ' && efresh){
            return_target_n=ep.x; return_target_e=ep.y;
            return_target_set=true; return_trail.clear();
            return_view_halfspan_m=0.50;
            return_raw_x=return_raw_y=0.0;
            return_body_dx=return_body_dy=0.0;
            return_ned_n=return_ned_e=0.0;
            return_b_marked=false;
            return_home_marked=false;
            if(fg_ok){ return_yaw0=fg.yaw; return_yaw0_set=true; }
            pending_return_event=1;
            std::cerr<<"RETURN GUI TARGET RESET: N="<<return_target_n<<" E="<<return_target_e
                     <<" yaw_deg="<<(return_yaw0_set?return_yaw0*180.0/M_PI:0.0)<<"\n";
          } else if((key=='b'||key=='B') && efresh){
            return_b_marked=true;
            return_b_n=ep.x; return_b_e=ep.y;
            return_b_raw_x=return_raw_x; return_b_raw_y=return_raw_y;
            return_b_body_dx=return_body_dx; return_b_body_dy=return_body_dy;
            return_b_ned_n=return_ned_n; return_b_ned_e=return_ned_e;
            return_b_yaw=fg_ok?fg.yaw:0.0;
            pending_return_event=2;
            std::cerr<<"RETURN GUI B MARK: EKF_from_A="<<1000.0*std::hypot(ep.x-return_target_n,ep.y-return_target_e)
                     <<" mm RAW_NED_from_A="<<1000.0*std::hypot(return_ned_n,return_ned_e)
                     <<" mm RAW_BODY_from_A="<<1000.0*std::hypot(return_body_dx,return_body_dy)
                     <<" mm RAW_LOS_legacy="<<1000.0*std::hypot(return_raw_x,return_raw_y)
                     <<" mm dYaw="<<(fg_ok&&return_yaw0_set?std::remainder(fg.yaw-return_yaw0,2.0*M_PI)*180.0/M_PI:0.0)<<" deg\n";
          } else if((key=='h'||key=='H') && efresh){
            pending_return_event=3;
            return_home_marked=true;
            const double ekf_close=1000.0*std::hypot(ep.x-return_target_n,ep.y-return_target_e);
            const double raw_close=1000.0*std::hypot(return_raw_x,return_raw_y);
            const double raw_body_close=1000.0*std::hypot(return_body_dx,return_body_dy);
            const double raw_ned_close=1000.0*std::hypot(return_ned_n,return_ned_e);
            std::cerr<<"\n======================================================================\n"
                     <<"RETURN CLOSURE MARK (PHYSICAL HOME)\n"
                     <<"EKF closure = "<<ekf_close<<" mm\n"
                     <<"RAW NED closure = "<<raw_ned_close<<" mm"
                     <<"  dN/E="<<return_ned_n*1000.0<<"/"<<return_ned_e*1000.0<<" mm\n"
                     <<"RAW BODY metric closure = "<<raw_body_close<<" mm\n"
                     <<"RAW LOS legacy closure = "<<raw_close<<" mm\n";
            if(return_b_marked){
              std::cerr<<"A->B EKF = "<<1000.0*std::hypot(return_b_n-return_target_n,return_b_e-return_target_e)<<" mm\n"
                       <<"B->H EKF = "<<1000.0*std::hypot(ep.x-return_b_n,ep.y-return_b_e)<<" mm\n"
                       <<"A->B RAW NED = "<<1000.0*std::hypot(return_b_ned_n,return_b_ned_e)<<" mm\n"
                       <<"B->H RAW NED = "<<1000.0*std::hypot(return_ned_n-return_b_ned_n,return_ned_e-return_b_ned_e)<<" mm\n"
                       <<"A->B RAW BODY = "<<1000.0*std::hypot(return_b_body_dx,return_b_body_dy)<<" mm\n"
                       <<"B->H RAW BODY = "<<1000.0*std::hypot(return_body_dx-return_b_body_dx,return_body_dy-return_b_body_dy)<<" mm\n"
                       <<"A->B RAW LOS legacy = "<<1000.0*std::hypot(return_b_raw_x,return_b_raw_y)<<" mm\n"
                       <<"B->H RAW LOS legacy = "<<1000.0*std::hypot(return_raw_x-return_b_raw_x,return_raw_y-return_b_raw_y)<<" mm\n";
            }
            std::cerr<<"dYaw(A->H) = "<<(fg_ok&&return_yaw0_set?std::remainder(fg.yaw-return_yaw0,2.0*M_PI)*180.0/M_PI:0.0)<<" deg\n"
                     <<"======================================================================\n";
          } else if(key=='c'||key=='C'){
            return_trail.clear();
          } else if(key=='q'||key=='Q'||key==27){
            g_running=false;
          }
        }

        // В guided-режиме подробная телеметрия остаётся в CSV, но не засоряет терминал.
        if(hotPathDiagnosticIoEnabled() && !guided && !return_cli && !blind4_cli && frame%100==0){
          std::cerr<<"OF frame="<<frame
                   <<" valid="<<(s.valid?1:0)
                   <<" rateFRD=("<<s.flow_body_x<<","<<s.flow_body_y<<") rad/s"
                   <<" scaleRate="<<s.scale_rate<<"/s"
                   <<" featFB="<<(s.feature_fallback?1:0)
                   <<" inliers="<<s.inliers<<"/"<<s.tracked
                   <<" sent="<<flow_sent_total<<" invalid="<<flow_invalid_total
                   <<" stale_reject="<<stale_flow_rejected_total
                   <<" terrain_guard="<<terrain_step_reject_total
                   <<" bridge_disabled[h/r/x]="<<bridge_hold_total<<"/"<<bridge_recovered_total<<"/"<<bridge_reset_total
                   <<" cam_drop="<<camera_queue_dropped_total
                   <<" latency="<<frame_pipeline_latency_ms<<"ms"
                   <<" stage_ms[F/L/R/P]="<<s.t_features_ms<<"/"<<s.t_lk_ms<<"/"<<s.t_ransac_ms<<"/"<<s.t_post_ms
                   <<" range="<<range_sent_total
                   <<" luna="<<(hl?lm:-1.0)<<"m age="<<(hl?lage:-1.0)<<"ms";
          if(esfresh){
            std::cerr<<" EKFSTAT flags=0x"<<std::hex<<es.flags<<std::dec
                     <<" ["<<ekfFlagsText(es.flags)<<"]"
                     <<" varV="<<es.velocity_variance
                     <<" varPH="<<es.pos_horiz_variance;
          } else {
            std::cerr<<" EKFSTAT=NO_DATA";
          }
          if(efresh)std::cerr<<" LOCAL pN/E=("<<ep.x<<","<<ep.y<<") vN/E=("<<ep.vx<<","<<ep.vy<<")";
          else std::cerr<<" LOCAL=NO_DATA";
          std::cerr<<"\r"<<std::flush;
        }

        const int64_t stage_anchor_enter_ns=monoNs();

        // Anchor policy: always advance to the newest decoded frame.
        //
        // The experimental bridge policy was removed after stress testing:
        // 23 bridge holds produced 0 successful recoveries, while the retained
        // old anchor inflated dt into the 100-200 ms range and caused cascaded
        // few-inliers/bad-dt failures.  Keeping the newest frame minimizes
        // inter-frame baseline and is therefore the safer production behavior.
        prev=gray.clone();
        prev_ts=ts;
        prev_camera_height_m=current_camera_height_m;
        prev_camera_height_valid=current_camera_height_valid;
        bridge_pending=false;

        const int64_t stage_anchor_exit_ns=monoNs();
        prev_send_to_csv_ms=(stage_csv_enter_ns-flow_send_ns)*1e-6;
        prev_csv_block_ms=(stage_csv_exit_ns-stage_csv_enter_ns)*1e-6;
        prev_csv_helpers_ms=(stage_csv_helpers_exit_ns-stage_csv_helpers_enter_ns)*1e-6;
        prev_csv_stream_ms=(stage_csv_stream_exit_ns-stage_csv_stream_enter_ns)*1e-6;
        prev_csv_flush_ms=(stage_csv_flush_exit_ns-stage_csv_flush_enter_ns)*1e-6;
        prev_csv_tellp_ms=(stage_csv_tellp_exit_ns-stage_csv_tellp_enter_ns)*1e-6;
        prev_csv_limit_ms=(stage_csv_limit_exit_ns-stage_csv_tellp_exit_ns)*1e-6;
        prev_web_block_ms=(stage_web_exit_ns-stage_csv_exit_ns)*1e-6;
        prev_postweb_ms=(stage_anchor_enter_ns-stage_web_exit_ns)*1e-6;
        prev_anchor_ms=(stage_anchor_exit_ns-stage_anchor_enter_ns)*1e-6;
        prev_loop_end_ns=stage_anchor_exit_ns;
    }

    g_running=false;
    csv_writer.stop();
    std::cerr<<"CSV ASYNC: dropped_rows="<<csv_writer.dropped()<<" bytes="<<csv_writer.bytes()<<"\n";
    if(guide_thread.joinable()) guide_thread.join();
    if(!remote_log_path.empty()){
      uint64_t rrx=0,rwr=0,rdup=0; size_t rpend=0;
      fc.remoteStats(&rrx,&rwr,&rdup,&rpend);
      fc.stopRemoteLog();
      std::cerr<<"REMOTE DATAFLASH: blocks_rx="<<rrx<<" written="<<rwr
               <<" duplicates="<<rdup<<" pending="<<rpend
               <<" BIN="<<remote_log_path<<"\n";
    }
    fc.stop(); luna.stop();
    std::cerr<<"\nОстановлено. CSV: "<<csvpath
             <<" flow_sent="<<flow_sent_total
             <<" invalid="<<flow_invalid_total
             <<" stale_reject="<<stale_flow_rejected_total
             <<" terrain_guard="<<terrain_step_reject_total
             <<" bridge_hold="<<bridge_hold_total
             <<" bridge_recovered="<<bridge_recovered_total
             <<" bridge_reset="<<bridge_reset_total
             <<" camera_queue_dropped="<<camera_queue_dropped_total
             <<" range_sent="<<range_sent_total<<"\n";
    return 0;
  } catch(const std::exception& e){
    g_running=false;
    std::cerr<<"ОШИБКА: "<<e.what()<<"\n";
    return 1;
  }
}
#endif // JTZERO_OPTFLOW_LIBRARY
