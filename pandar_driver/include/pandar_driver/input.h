#pragma once
#include <pandar_msgs/PandarPacket.h>
#include <pandar_msgs/PandarPacket2.h>

namespace pandar_driver
{
class Input
{
public:
  enum class PacketType : int {
    LIDAR = 0,
    GPS = 1,
    ERROR = -1
  };
  virtual ~Input(){};
  virtual PacketType getPacket(pandar_msgs::PandarPacket* pkt) = 0;
  virtual PacketType getPacket(pandar_msgs::PandarPacket2* pkt) = 0;

  bool use_variable_length_packet_ = true;
};
}  // namespace pandar_driver
