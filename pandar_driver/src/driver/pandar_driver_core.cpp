#include <pandar_driver/pandar_driver_core.h>
#include <pandar_driver/input.h>
#include <pandar_driver/pcap_input.h>
#include <pandar_driver/socket_input.h>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <pandar_msgs/msg/pandar_packet.hpp>

using namespace pandar_driver;

PandarDriverCore::PandarDriverCore(rclcpp::Node *node)
: node_(node)
{
  pcap_path_ = node->declare_parameter("pcap", "");
  device_ip_ = node->declare_parameter("device_ip", "");
  lidar_port_ = node->declare_parameter("lidar_port", 0);
  gps_port_ = node->declare_parameter("gps_port", 0);
  scan_phase_ = node->declare_parameter("scan_phase", 0.0);
  model_ = node->declare_parameter("model", "");
  frame_id_ = node->declare_parameter("frame_id", "");

  pandar_packet_pub_ = node_->create_publisher<pandar_msgs::msg::PandarScan>("pandar_packets", rclcpp::SensorDataQoS());


  if (!pcap_path_.empty()) {
    input_.reset(new PcapInput(node, lidar_port_, gps_port_, pcap_path_, model_));
  }
  else {
    input_.reset(new SocketInput(node, device_ip_, lidar_port_, gps_port_));
    
    { // EXPERIMENTAL: If the scan angle is set outside of the scan range, fix it.
      client_ = std::make_shared<pandar_api::TCPClient>(device_ip_);
      uint16_t range[2];
      auto code = client_->getLidarRange(range);
      if(code == pandar_api::TCPClient::ReturnCode::SUCCESS){
        int scan_angle = (static_cast<int>(range[1]) + 36000 - range[0]) % 36000;
        int max_angle = (static_cast<int>(scan_phase_ * 100) + 36000 - range[0]) % 36000;
        if(scan_angle < max_angle){
          scan_phase_ = static_cast<double>(range[1] / 100.0);
        }
      }
    }
  }

  if (model_ == "Pandar40P" || model_ == "Pandar40M") {
    azimuth_index_ = 2;  // 2 + 124 * [0-9]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1262 || packet_size == 1266); };
  }
  else if (model_ == "PandarQT") {
    azimuth_index_ = 12;  // 12 + 258 * [0-3]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1072); };
  }
  else if (model_ == "PandarQT128") {
    azimuth_index_ = 12;  // 12 + 512 * [0-1]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1127); };
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
  else  if (model_ == "PandarXT-16") {
    RCLCPP_INFO_STREAM(node->get_logger(), "PandarXT16");
    azimuth_index_ = 12;
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 568); };
  }
  else {
    RCLCPP_ERROR(node->get_logger(), "Invalid model name");
  }
}

bool PandarDriverCore::poll(void)
{
  int scan_phase = static_cast<int>(scan_phase_ * 100.0);

  auto scan = std::make_shared<pandar_msgs::msg::PandarScan>();
  for (int prev_phase = 0;;) {  // finish scan
    while (true) {              // until receive lidar packet
      pandar_msgs::msg::PandarPacket packet;
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
  pandar_packet_pub_->publish(*scan);
  return true;
}
