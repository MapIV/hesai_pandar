#pragma once

#include <array>
#include "pandar_pointcloud/calibration.hpp"
#include "packet_decoder.hpp"
#include "pandar_qt.hpp"

namespace pandar_pointcloud
{
namespace pandar_qt
{
class PandarQTDecoder : public PacketDecoder
{
public:
  enum class ReturnMode : int8_t
  {
    DUAL,
    FIRST,
    LAST,
  };
  enum ReturnType : uint8_t
  {
    INVALID = 0,
    SINGLE_FIRST,
    SINGLE_LAST,
    DUAL_FIRST,
    DUAL_LAST,
    DUAL_ONLY,
  };

  PandarQTDecoder(Calibration& calibration, float scan_phase = 0.0f, double dual_return_distance_threshold = 0.1, ReturnMode return_mode = ReturnMode::DUAL);
  void unpack(const pandar_msgs::PandarPacket& raw_packet) override;
  void unpack(const pandar_msgs::PandarPacket2& raw_packet) override;
  PointXYZIRADT build_point(int block_id, int unit_id, uint8_t return_type);
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;

private:
  bool parsePacket(const pandar_msgs::PandarPacket& raw_packet);
  bool parsePacket(const pandar_msgs::PandarPacket2& raw_packet);
  bool parseBinary(const uint8_t* buf);
  PointcloudXYZIRADT convert(const int block_id);
  PointcloudXYZIRADT convert_dual(const int block_id);

  std::array<float, UNIT_NUM> elev_angle_;
  std::array<float, UNIT_NUM> azimuth_offset_;

  std::array<float, UNIT_NUM> firing_offset_;
  std::array<float, BLOCK_NUM> block_offset_single_;
  std::array<float, BLOCK_NUM> block_offset_dual_;

  ReturnMode return_mode_;
  double dual_return_distance_threshold_;
  Packet packet_;

  PointcloudXYZIRADT scan_pc_;
  PointcloudXYZIRADT overflow_pc_;

  uint16_t scan_phase_;
  int last_phase_;
  bool has_scanned_;
};

}  // namespace pandar_qt
}  // namespace pandar_pointcloud
