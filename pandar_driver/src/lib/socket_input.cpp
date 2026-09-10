#include "pandar_driver/socket_input.h"

using namespace pandar_driver;

namespace
{
const size_t ETHERNET_MTU = 1500;
}

SocketInput::SocketInput(rclcpp::Node * node, const std::string& device_ip, uint16_t port, uint16_t gps_port, int timeout)
: clock_(node->get_clock()), logger_(node->get_logger()), io_service_()
{
  device_ip_ = boost::asio::ip::make_address(device_ip);
  timeout_ = timeout;

  lidar_socket_ = std::make_unique<udp::socket>(io_service_, udp::endpoint(udp::v4(), port));
  gps_socket_ = std::make_unique<udp::socket>(io_service_, udp::endpoint(udp::v4(), gps_port));
  deadline_ = std::make_unique<boost::asio::steady_timer>(io_service_);
  deadline_->expires_at(boost::asio::steady_timer::time_point::max());
  checkDeadline();
}

SocketInput::~SocketInput(void)
{
  lidar_socket_->close();
}

SocketInput::PacketType SocketInput::getPacket(pandar_msgs::msg::PandarPacket* pkt)
{
  deadline_->expires_after(std::chrono::milliseconds(timeout_));

  boost::system::error_code error_code = boost::asio::error::would_block;
  udp::endpoint remote_endpoint{};
  size_t packet_size = 0;

  lidar_socket_->async_receive_from(boost::asio::buffer(pkt->data, ETHERNET_MTU), remote_endpoint,
                               [&error_code, &packet_size](const boost::system::error_code & error, std::size_t length){error_code = error; packet_size=length;});

  while (error_code == boost::asio::error::would_block){
    io_service_.run_one();
  }
  if(error_code == boost::system::errc::success && remote_endpoint.address() == device_ip_){
    pkt->stamp = clock_->now();
    pkt->size = static_cast<uint32_t>(packet_size);
    return PacketType::LIDAR;
  }else if(error_code == boost::system::errc::operation_canceled){
    // timeout & operation canceld
    // RCLCPP_WARN(logger_,"timeout"); 
    return PacketType::ERROR;
  }else{
    return PacketType::ERROR;
  }
}

void SocketInput::checkDeadline()
{
  if (deadline_->expiry() <= std::chrono::steady_clock::now())
  {
    lidar_socket_->cancel();
    deadline_->expires_at(boost::asio::steady_timer::time_point::max());
  }
  deadline_->async_wait(std::bind(&SocketInput::checkDeadline, this));
}