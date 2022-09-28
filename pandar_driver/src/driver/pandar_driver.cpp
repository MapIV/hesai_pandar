#include <pandar_driver/pandar_driver.h>
#include <pandar_driver/input.h>
#include <pandar_driver/pcap_input.h>
#include <pandar_driver/socket_input.h>
#include <pandar_msgs/PandarPacket.h>
#include <pandar_msgs/PandarScan.h>

using namespace pandar_driver;

PandarDriver::PandarDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  private_nh.getParam("pcap", pcap_path_);
  private_nh.getParam("device_ip", device_ip_);
  private_nh.getParam("lidar_port", lidar_port_);
  private_nh.getParam("gps_port", gps_port_);
  private_nh.getParam("scan_phase", scan_phase_);
  private_nh.getParam("model", model_);
  private_nh.getParam("frame_id", frame_id_);

  pandar_packet_pub_ = node.advertise<pandar_msgs::PandarScan>("pandar_packets", 10);

  if (!pcap_path_.empty()) {
    input_.reset(new PcapInput(lidar_port_, gps_port_, pcap_path_, model_));
  }
  else {
    input_.reset(new SocketInput(device_ip_, lidar_port_, gps_port_));
  }

  client_ = std::make_shared<pandar_api::TCPClient>(device_ip_);
  // uint16_t range[2];

  if (model_ == "Pandar40P" || model_ == "Pandar40M") {
    azimuth_index_ = 2;  // 2 + 124 * [0-9]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1262 || packet_size == 1266); };
  }
  else if (model_ == "PandarQT") {
    azimuth_index_ = 12;  // 12 + 258 * [0-3]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1072); };
  }
  else  if (model_ == "PandarXT-32") {
    azimuth_index_ = 12;  // 12 + 130 * [0-7]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1080); };
  }
  else if (model_ == "Pandar64") {
    azimuth_index_ = 8;  // 8 + 192 * [0-5]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1194 || packet_size == 1198); };
  }
  else if (model_ == "Pandar128") {
    azimuth_index_ = 12;  // 12 + 386 * [0-1]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 812); };
  }
  else if (model_ == "PandarXTM") {
    azimuth_index_ = 12;  // 12 + 130 * [0-7]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 820); };
  }
  else if (model_ == "PandarQT128") {
    azimuth_index_ = 12;  // 12 + 512 * [0-1]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1127); };
  }
  else {
    ROS_ERROR("Invalid model name");
  }
}

bool PandarDriver::poll(void)
{
  int scan_phase = static_cast<int>(scan_phase_ * 100.0);

  pandar_msgs::PandarScanPtr scan(new pandar_msgs::PandarScan);
  for (int prev_phase = 0;;) {  // finish scan
    while (true) {              // until receive lidar packet
      pandar_msgs::PandarPacket packet;
      Input::PacketType packet_type = input_->getPacket(&packet);
      if (packet_type == Input::PacketType::LIDAR && is_valid_packet_(packet.size)) {
        scan->packets.push_back(packet);
        break;
      }
    }

    int current_phase = 0;
    {
      const auto& data = scan->packets.back().data;
      current_phase = (data[azimuth_index_] & 0xff) | ((data[azimuth_index_ + 1] & 0xff) << 8);
      current_phase = (static_cast<int>(current_phase) + 36000 - scan_phase) % 36000;
    }
    if (current_phase >= prev_phase || scan->packets.size() < 2) {
      prev_phase = current_phase;
    }
    else {
      // has scanned !
      break;
    }
  }

  scan->header.stamp = scan->packets.front().stamp;
  scan->header.frame_id = frame_id_;
  pandar_packet_pub_.publish(scan);
  return true;
}
