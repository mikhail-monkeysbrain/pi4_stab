// Verify CSI grayscale frames and SensorTimestamp without MAVLink or WORKED5.
#include "csi_gray_camera.hpp"
#include <iostream>
#include <chrono>
int main(){
  try{
    CsiGrayCamera camera;
    camera.open(60);
    const auto start=std::chrono::steady_clock::now();
    uint64_t frames=0,drops=0;
    int64_t first=0,last=0,max_age_ns=0;
    while(std::chrono::steady_clock::now()-start<std::chrono::seconds(10)){
      CsiGrayCamera::Frame f;
      if(!camera.take(f,100)) continue;
      if(f.gray.empty() || f.gray.cols!=640 || f.gray.rows!=480)
        throw std::runtime_error("invalid CSI gray frame");
      if(first==0) first=f.sensor_ns;
      if(last && f.sensor_ns<=last) throw std::runtime_error("non-monotonic SensorTimestamp");
      last=f.sensor_ns;
      ++frames; drops+=f.dropped;
      max_age_ns=std::max(max_age_ns,f.received_ns-f.sensor_ns);
    }
    camera.close();
    std::cout<<"CSI_GRAY_FRAMES "<<frames<<"\nCSI_GRAY_DROPS "<<drops
             <<"\nCSI_GRAY_SENSOR_FPS "<<(last>first && frames>1?(frames-1)*1e9/(last-first):0.0)
             <<"\nCSI_GRAY_MAX_SENSOR_AGE_MS "<<max_age_ns/1e6<<"\n";
    return frames>0?0:1;
  }catch(const std::exception &e){
    std::cerr<<"CSI_GRAY_ERROR "<<e.what()<<"\n";
    return 1;
  }
}
