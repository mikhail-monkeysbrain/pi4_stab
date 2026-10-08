// Standalone smoke test of reusable Pi4 OV5647 capture adapter.
// No WORKED5, altitude, MAVLink or flight output.
#include "pi4_ov5647_capture.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
int main(int argc,char **argv) {
    try {
        const int seconds=argc>1?std::stoi(argv[1]):10;
        if(seconds<1 || seconds>120) return 2;
        pi4_capture::Ov5647Capture camera;
        camera.start();
        uint64_t accepted=0;
        int64_t first_ts=0,last_ts=0;
        const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(seconds);
        while(std::chrono::steady_clock::now()<until) {
            pi4_capture::Frame frame;
            if(!camera.next(frame,1000)) continue;
            if(!first_ts) first_ts=frame.sensor_timestamp_ns;
            last_ts=frame.sensor_timestamp_ns;
            ++accepted;
        }
        const auto stats=camera.stats();
        camera.stop();
        std::cout<<"OV5647_ADAPTER received="<<stats.received
                 <<" accepted="<<accepted
                 <<" missing_ts="<<stats.missing_timestamp
                 <<" nonmonotonic_ts="<<stats.nonmonotonic_timestamp
                 <<" cancelled="<<stats.cancelled
                 <<" first_ts_ns="<<first_ts<<" last_ts_ns="<<last_ts
                 <<" no_fc_tx=1 no_worked5_change=1\n";
        return accepted>0 && stats.missing_timestamp==0 &&
               stats.nonmonotonic_timestamp==0 ? 0 : 1;
    } catch(const std::exception &e) {
        std::cerr<<"OV5647_ADAPTER_FAIL "<<e.what()<<"\n";
        return 2;
    }
}
