#pragma once

#include <ros/ros.h>
#include <pandar_api/tcp_client.hpp>

namespace pandar_driver
{
class Input;
class PandarDriver
{
public:
  PandarDriver(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~PandarDriver(){};
  bool poll(void);

private:
  std::string device_ip_;
  int lidar_port_;
  int gps_port_;
  double scan_phase_;
  size_t azimuth_index_;

  std::string model_;
  std::string frame_id_;
  std::string pcap_path_;
  
  bool use_variable_length_packet_ = true;

  ros::Publisher pandar_packet_pub_;
  std::shared_ptr<Input> input_;
  std::shared_ptr<pandar_api::TCPClient> client_;

  std::function<bool(size_t)> is_valid_packet_;
};
}  // namespace pandar_driver