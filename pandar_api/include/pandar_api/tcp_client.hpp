#pragma once

#include <string>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>

namespace pandar_api
{

struct LidarStatus
{
  uint32_t uptime;
  uint16_t motor_speed;
  int32_t temp[8];
  uint8_t gps_pps_lock;
  uint8_t gps_gprmc_status;
  uint32_t startup_times;
  uint32_t total_operation_time;    
  uint8_t ptp_clock_status;
  uint8_t reserved[5];
};

struct InventoryInfo
{
  std::string sn;
  std::string date_of_manufacture;
  std::string mac;
  std::string sw_ver;
  std::string hw_ver;
  std::string control_fw_ver;
  std::string sensor_fw_ver;
  uint16_t angle_offset; 
  std::string model;
  std::string motor_type;
  uint8_t num_of_lines;
};

struct PTPDiag
{
  int64_t master_offset;
  std::string ptp_state;
  uint32_t elapsed_millisec;
};

class TCPClient
{
public:
  TCPClient(const std::string& device_ip, int32_t timeout=1000/*msec*/);

  enum class ReturnCode : uint8_t
  {
    SUCCESS       = 0x00, // - no error
    INVALID_INPUT = 0x01, // - invalid input parameter
    CONNECTION_FAILED = 0x02, // - failure to connect to server
    NO_VALID_DATA = 0x03, // - no valid data returned
    NO_MEMORY     = 0x04, // - server does not have enough memory
    NO_SUPPORT    = 0x05, // - server does not support this command yet
    FPGA_ERROR    = 0x06, // - server fails to communicate with the inner FPGA
  };

  ReturnCode getInventoryInfo(InventoryInfo& info);
  ReturnCode getLidarCalibration(std::string& content);
  ReturnCode getLidarRange(uint16_t* range);
  ReturnCode getLidarStatus(LidarStatus& status);
  ReturnCode getPTPDiagnostics(PTPDiag& diag);
  std::vector<uint8_t> payload_;

private:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::ip::address device_ip_;
  boost::asio::steady_timer timer_;

  enum PTC_COMMAND : uint8_t
  {
    PTC_COMMAND_GET_CALIBRATION       = 0x00,
    PTC_COMMAND_SET_CALIBRATION       = 0x01,
    PTC_COMMAND_HEARTBEAT             = 0x02,
    PTC_COMMAND_RESET_CALIBRATION     = 0x03,
    PTC_COMMAND_TEST                  = 0x04,
    PTC_COMMAND_GET_LIDAR_CALIBRATION = 0x05, // To retrieve the LiDAR's calibration file
    PTC_COMMAND_PTP_DIAGNOSTICS       = 0x06, // To retrieve the PTP diagnostics for a specified PTP Query Type
    PTC_COMMAND_GET_INVENTORY_INFO    = 0x07, // To retrieve inventory info
    PTC_COMMAND_GET_CONFIG_INFO       = 0x08, // To retrieve configuration parameters
    PTC_COMMAND_GET_LIDAR_STATUS      = 0x09, // To retrieve status info, such as temperature and system uptime
    PTC_COMMAND_SET_CLOCK_DATA_FMT    = 0x12, // To select the GPS NMEA data format
    PTC_COMMAND_SET_SPIN_RATE         = 0x17, // To set the LiDAR's spin rate
    PTC_COMMAND_SET_SYNC_ANGLE        = 0x18, // To set the sync angle between the LiDAR's 0° position and the PPS signal
    PTC_COMMAND_SET_UDP_SEQUENCE      = 0x19, // To enable/disable the UDP sequence of Point Cloud Data Packets
    PTC_COMMAND_SET_NOISE_FILTERING   = 0x1a, // To enable/disable noise filtering for rain and fog
    PTC_COMMAND_SET_TRIGGER_METHOD    = 0x1b, // To select the method for triggering laser firings: angle/time based
    PTC_COMMAND_SET_STANDBY_MODE      = 0x1c, // To enter/exit standby mode
    PTC_COMMAND_SET_REFLECTIVITY_MAPPING
                                      = 0x1d, // To select the reflectivity mapping mode: linear/nonlinear
    PTC_COMMAND_SET_RETURN_MODE       = 0x1e, // To set the return mode: last/strongest/dual return
    PTC_COMMAND_SET_CLOCK_SOURCE      = 0x1f, // To select the LiDAR's clock source: GPS/PTP
    PTC_COMMAND_SET_DESTINATION_IP    = 0x20, // To set the LiDAR's Destination IP and Port
    PTC_COMMAND_SET_CONTROL_PORT      = 0x21, // To set the LiDAR's IPv4, mask, gateway, and VLAN settings
    PTC_COMMAND_SET_LIDAR_RANGE       = 0x22, // To set the LiDAR's azimuth FOV for all channels
    PTC_COMMAND_GET_LIDAR_RANGE       = 0x23, // To retrieve the LiDAR's azimuth FOV for all channels
    PTC_COMMAND_SET_PTP_CONFIG        = 0x24, // To configure PTP settings
    PTC_COMMAND_RESET                 = 0x25, // To reset all configurable parameters to factory defaults and reboot LiDAR
    PTC_COMMAND_GET_PTP_CONFIG        = 0x26, // To retrieve PTP settings
    PTC_COMMAND_SET_CODE_RANGE        = 0x0e, // To set each channel's code range (fingerprint) for interference rejection
    PTC_COMMAND_GET_CODE_RANGE        = 0x0f, // To retrieve each channel's code range (fingerprint) for interference rejection
  };

  struct MessageHeader
  {
    uint8_t protocol_identifier[2];
    uint8_t cmd;
    uint8_t return_code;
    uint32_t payload_length;

    MessageHeader()
     : cmd(0),return_code(0),payload_length(0)
    {
      protocol_identifier[0] = 0x47;
      protocol_identifier[1] = 0x74;
    }

    void read(uint8_t* buffer)
    {
      int index = 0;
      protocol_identifier[0] = buffer[index++];
      protocol_identifier[1] = buffer[index++];
      cmd = buffer[index++];
      return_code = buffer[index++];
      payload_length = htobe32(*((uint32_t*) &buffer[index]));
    }

    void write(uint8_t* buffer)
    {
      int index = 0;
      buffer[index++] = protocol_identifier[0];
      buffer[index++] = protocol_identifier[1];
      buffer[index++] = cmd;
      buffer[index++] = return_code;

      uint32_t length = be32toh(payload_length);
      // uint32_t length = payload_length;
      std::memcpy(buffer + index, &length, sizeof(uint32_t));

      // buffer[index++] = (payload_length >> 24) & 0xff;
      // buffer[index++] = (payload_length >> 16) & 0xff;
      // buffer[index++] = (payload_length >> 8) & 0xff;
      // buffer[index++] = (payload_length >> 0) & 0xff;

    }

  };

  MessageHeader header_;
  // std::vector<uint8_t> payload_;
  std::vector<uint8_t> buffer_;
  ReturnCode return_code_;
  int32_t timeout_;

  void connect();
  void on_connect(const boost::system::error_code& error);
  void on_send(const boost::system::error_code& error);
  void on_receive(const boost::system::error_code& error);
  void on_timer(const boost::system::error_code& error);
};

}  // namespace pandar_pointcloud