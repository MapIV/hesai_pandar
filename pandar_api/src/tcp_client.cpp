#include <string>
#include <iostream>
#include <cstdint>
#include <boost/bind/bind.hpp>
using namespace boost::placeholders;

#include "pandar_api/tcp_client.hpp"

namespace{
  const uint16_t API_PORT = 9347;
  const size_t HEADER_SIZE = 8;

  inline int64_t parse64(uint8_t* raw){
    return htobe64(*((int64_t*)raw)); 
  }

  inline uint32_t parse32(uint8_t* raw){
    return htobe32(*((uint32_t*)raw)); 
  }

  inline uint16_t parse16(uint8_t* raw){
    return htobe16(*((uint16_t*)raw));
  }
}

namespace pandar_api
{
TCPClient::TCPClient(const std::string& device_ip, int32_t timeout)
  : io_service_(),
    socket_(io_service_),
    timer_(io_service_),
    return_code_(ReturnCode::SUCCESS),
    timeout_(timeout)
{
  device_ip_ = boost::asio::ip::make_address(device_ip);
}


TCPClient::ReturnCode TCPClient::getInventoryInfo(InventoryInfo& info)
{
  header_ = MessageHeader();
  header_.cmd = PTC_COMMAND_GET_INVENTORY_INFO;
  payload_.clear();
  connect();
  if(return_code_ != ReturnCode::SUCCESS){
    return return_code_;
  }
  uint8_t* it = payload_.data();

  info.sn = std::string(it, it + 18);
  it += 18;

  info.date_of_manufacture = std::string(it, it + 16);
  it += 16;

  info.mac = std::string(it, it + 6);
  it += 6;

  info.sw_ver = std::string(it, it + 16);
  it += 16;

  info.hw_ver = std::string(it, it + 16);
  it += 16;

  info.control_fw_ver = std::string(it, it + 16);
  it += 16;

  info.sensor_fw_ver = std::string(it, it + 16);
  it += 16;

  info.angle_offset = parse16(it);
  it += 2;

  if(*it == 0){
    info.model = "Pandar40P";
  }else if(*it == 2){
    info.model = "Pandar64";
  }else if(*it == 3){
    info.model = "Pandar128";
  }else if(*it == 15){
    info.model = "PandarQT";
  }else if(*it == 17){
    info.model = "Pandar40M";
  }else{
    info.model = "unknown";
  }    
  it += 1;

  if(*it == 0){
    info.motor_type = "single direction";
  }else{
    info.motor_type = "dual direction";
  }
  it += 1;

  info.num_of_lines = *it;
  it += 1;
  return return_code_;
}

TCPClient::ReturnCode TCPClient::getLidarCalibration(std::string& content)
{
  header_ = MessageHeader();
  payload_.clear();
  header_.cmd = PTC_COMMAND_GET_LIDAR_CALIBRATION;
  connect();
  if(return_code_ == ReturnCode::SUCCESS){
    content = std::string(payload_.data(), payload_.data() + payload_.size());
  }
  return return_code_;
}

TCPClient::ReturnCode TCPClient::getLidarRange(uint16_t* range)
{
  header_ = MessageHeader();
  header_.cmd = PTC_COMMAND_GET_LIDAR_RANGE;
  payload_.clear();
  connect();
  if(return_code_ == ReturnCode::SUCCESS){
    if(payload_[0] != 0){
      // not support each-channel / multi-section
      return ReturnCode::NO_SUPPORT;
    }else{
      range[0] = parse16(&payload_[1]) * 10;
      range[1] = parse16(&payload_[3]) * 10;
      return return_code_;
    }
  }else{
    return return_code_;
  }
}

TCPClient::ReturnCode TCPClient::getLidarStatus(LidarStatus& status)
{
  header_ = MessageHeader();
  header_.cmd = PTC_COMMAND_GET_LIDAR_STATUS;
  payload_.clear();
  connect();
  if(return_code_ == ReturnCode::SUCCESS){
    uint8_t* it = payload_.data();

    status.uptime = parse32(it);
    it += 4;
    status.motor_speed = parse16(it);
    it += 2;
    for(int i = 0; i < 8; i++){
      status.temp[i] = parse32(it);
      it += 4;
    }
    status.gps_pps_lock = *it;
    it+=1;

    status.gps_gprmc_status = *it;
    it+=1;

    status.startup_times = parse32(it);
    it += 4;
    
    status.total_operation_time = parse32(it);
    it += 4;
    
    status.ptp_clock_status = *it;

    return return_code_;
  }else{
    return return_code_;
  }
}

TCPClient::ReturnCode TCPClient::getPTPDiagnostics(PTPDiag& diag)
{
  header_ = MessageHeader();
  header_.cmd = PTC_COMMAND_PTP_DIAGNOSTICS;
  header_.payload_length = 1;
  payload_.clear();
  payload_.push_back(0x01);

  connect();
  if(return_code_ == ReturnCode::SUCCESS){
    uint8_t* it = payload_.data();
    diag.master_offset = parse64(it);
    it += 8;
    uint32_t test = parse32(it);
    it += 4;
    diag.elapsed_millisec = parse32(it);
    it += 4;
    std::cout << diag.master_offset << std::endl;
    std::cout << test << std::endl;
    std::cout << diag.elapsed_millisec << std::endl;
  }
  return return_code_;
}

void TCPClient::connect()
{
  io_service_.restart();
  socket_.async_connect(
    boost::asio::ip::tcp::endpoint(device_ip_, API_PORT),
    boost::bind(&TCPClient::on_connect, this, boost::asio::placeholders::error));

  timer_.expires_after(std::chrono::milliseconds(timeout_));
  timer_.async_wait(boost::bind(&TCPClient::on_timer, this, boost::placeholders::_1));
  io_service_.run();
}

void TCPClient::on_connect(const boost::system::error_code& error)
{
  if (error) {
    return_code_ = ReturnCode::CONNECTION_FAILED;
    return;
  }

  // send header
  buffer_.clear();

  buffer_.resize(HEADER_SIZE + payload_.size());
  header_.write(buffer_.data());
  std::memcpy(buffer_.data() + HEADER_SIZE, payload_.data(), payload_.size());

  boost::asio::async_write(
      socket_,
      boost::asio::buffer(buffer_),
      boost::bind(&TCPClient::on_send, this, boost::asio::placeholders::error));
}


void TCPClient::on_send(const boost::system::error_code& error)
{
  if (error) {
    return_code_ = ReturnCode::CONNECTION_FAILED;
    return;
  }

  boost::asio::async_read(
  socket_,
  boost::asio::buffer(buffer_),
  boost::asio::transfer_exactly(HEADER_SIZE),
  boost::bind(&TCPClient::on_receive, this, boost::asio::placeholders::error));
}

void TCPClient::on_receive(const boost::system::error_code& error)
{
  if (error) {
    return_code_ = ReturnCode::CONNECTION_FAILED;
    return;
  }
  header_.read(buffer_.data());
  if(header_.protocol_identifier[0] != 0x47 || header_.protocol_identifier[1] != 0x74){
    return_code_ = ReturnCode::CONNECTION_FAILED;
    socket_.close();
    return;
  }else if((ReturnCode)header_.return_code != ReturnCode::SUCCESS) {
    return_code_ = (ReturnCode)header_.return_code;
    socket_.close();
    return;
  }

  // receive payload
  if (header_.payload_length > 0){
    payload_.resize(header_.payload_length);
    boost::asio::async_read(
        socket_,
        boost::asio::buffer(payload_),
        boost::asio::transfer_exactly(payload_.size()),
        [this](const boost::system::error_code& error, std::size_t)
        {
          if (error){
            return_code_ = ReturnCode::CONNECTION_FAILED;
          }
          else {
            socket_.close();
            timer_.cancel();
            return;
          }
        });
  }
}

void TCPClient::on_timer(const boost::system::error_code& error)
{
  if (!error) {
    return_code_ = ReturnCode::CONNECTION_FAILED;
    socket_.close();
  }
}

}  // namespace pandar_api