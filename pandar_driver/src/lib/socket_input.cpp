#include "pandar_driver/socket_input.h"

using namespace pandar_driver;

namespace
{
const size_t ETHERNET_MTU = 1500;
}

SocketInput::SocketInput(const std::string& device_ip, uint16_t port, uint16_t gps_port, int timeout)
  : io_service_()
{
  device_ip_ = boost::asio::ip::address::from_string(device_ip);
  timeout_ = timeout;

  lidar_socket_ = std::make_unique<udp::socket>(io_service_, udp::endpoint(udp::v4(), port));
  gps_socket_ = std::make_unique<udp::socket>(io_service_, udp::endpoint(udp::v4(), gps_port));
  deadline_ = std::make_unique<boost::asio::deadline_timer>(io_service_);
  deadline_->expires_at(boost::posix_time::pos_infin);
  checkDeadline();
}

SocketInput::~SocketInput(void)
{
  lidar_socket_->close();
}

SocketInput::PacketType SocketInput::getPacket(pandar_msgs::PandarPacket2* pkt)
{
  deadline_->expires_from_now(boost::posix_time::milliseconds(timeout_));

  boost::system::error_code error_code = boost::asio::error::would_block;
  udp::endpoint remote_endpoint{};
  size_t packet_size = 0;

  pkt->data.resize(ETHERNET_MTU);

  lidar_socket_->async_receive_from(boost::asio::buffer(pkt->data, ETHERNET_MTU), remote_endpoint,
                               [&error_code, &packet_size](const boost::system::error_code & error, std::size_t length){error_code = error; packet_size=length;});

  while (error_code == boost::asio::error::would_block){
    io_service_.run_one();
  }

  if(error_code == boost::system::errc::success && remote_endpoint.address() == device_ip_){
    pkt->stamp = ros::Time::now();
    pkt->size = static_cast<uint32_t>(packet_size);
    return PacketType::LIDAR;
  }else{
    return PacketType::ERROR;
  }
}

SocketInput::PacketType SocketInput::getPacket(pandar_msgs::PandarPacket* pkt)
{
  deadline_->expires_from_now(boost::posix_time::milliseconds(timeout_));

  boost::system::error_code error_code = boost::asio::error::would_block;
  udp::endpoint remote_endpoint{};
  size_t packet_size = 0;

  lidar_socket_->async_receive_from(boost::asio::buffer(pkt->data, ETHERNET_MTU), remote_endpoint,
                               [&error_code, &packet_size](const boost::system::error_code & error, std::size_t length){error_code = error; packet_size=length;});

  while (error_code == boost::asio::error::would_block){
    io_service_.run_one();
  }

  if(error_code == boost::system::errc::success && remote_endpoint.address() == device_ip_){
    pkt->stamp = ros::Time::now();
    pkt->size = static_cast<uint32_t>(packet_size);
    return PacketType::LIDAR;
  }else{
    return PacketType::ERROR;
  }
}

void SocketInput::checkDeadline()
{
  if (deadline_->expires_at() <= boost::asio::deadline_timer::traits_type::now())
  {
    lidar_socket_->cancel();
    deadline_->expires_at(boost::posix_time::pos_infin);
  }
  deadline_->async_wait(std::bind(&SocketInput::checkDeadline, this));
}