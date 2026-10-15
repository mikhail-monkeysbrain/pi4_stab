#pragma once
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <unistd.h>

struct GroundMotionMavlinkPublisher {
  uint8_t system_id=191;
  uint8_t component_id=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

  static bool writeMessage(int fd,const mavlink_message_t& msg){
    if(fd<0) return false;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n=mavlink_msg_to_send_buffer(buf,&msg);
    size_t off=0;
    while(off<n){
      const ssize_t k=::write(fd,buf+off,n-off);
      if(k>0){ off+=(size_t)k; continue; }
      if(k<0 && errno==EINTR) continue;
      if(k<0 && (errno==EAGAIN || errno==EWOULDBLOCK)) return false;
      return false;
    }
    return true;
  }

  bool sendDistanceSensor(int fd,uint32_t time_boot_ms,double distance_m) const {
    if(fd<0 || !std::isfinite(distance_m)) return false;
    if(distance_m<0.03 || distance_m>8.00) return false;
    const uint16_t current_cm=(uint16_t)std::lround(std::clamp(distance_m,0.03,8.00)*100.0);
    float quaternion[4]={0,0,0,0};
    mavlink_message_t msg{};
    mavlink_msg_distance_sensor_pack(
      system_id,component_id,&msg,time_boot_ms,
      3,800,current_cm,MAV_DISTANCE_SENSOR_LASER,
      0,MAV_SENSOR_ROTATION_PITCH_270,0,
      0.0f,0.0f,quaternion,0);
    return writeMessage(fd,msg);
  }
};
