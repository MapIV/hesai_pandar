#pragma once

#include <array>
#include "pandar_pointcloud/calibration.hpp"
#include "packet_decoder.hpp"
#include "pandar_128_e4x.hpp"

namespace pandar_pointcloud
{
namespace pandar_128_e4x
{
class Pandar128E4XDecoder : public PacketDecoder
{
public:
  enum class ReturnMode : int8_t
  {
    DUAL,
    STRONGEST,
    LAST,
  };
  enum ReturnType : uint8_t
  {
    INVALID = 0,
    SINGLE_STRONGEST,
    SINGLE_LAST,
    DUAL_STRONGEST_FIRST,
    DUAL_STRONGEST_LAST,
    DUAL_WEAK_FIRST,
    DUAL_WEAK_LAST,
    DUAL_ONLY,
  };

  Pandar128E4XDecoder(Calibration& calibration,
                      float scan_phase = 0.0f,
                      double dual_return_distance_threshold = 0.1,
                      ReturnMode return_mode = ReturnMode::DUAL);
  void unpack(const pandar_msgs::PandarPacket& raw_packet) override;
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;

private:
  bool parsePacket(const pandar_msgs::PandarPacket& raw_packet);
  PointXYZIRADT build_point(int block_id, int unit_id, uint8_t return_type);
  PointcloudXYZIRADT convert(const int block_id);
  PointcloudXYZIRADT convert_dual(const int block_id);

  std::array<float, LASER_COUNT> elev_angle_{};
  std::array<float, LASER_COUNT> elev_angle_rad_{};
  std::array<float, LASER_COUNT> cos_elev_angle_{};
  std::array<float, LASER_COUNT> sin_elev_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};

  Packet packet_{};

  PointcloudXYZIRADT scan_pc_;
  PointcloudXYZIRADT overflow_pc_;

  uint16_t scan_phase_;
  int last_phase_;
  bool has_scanned_;

  double dual_return_distance_threshold_;
};

}  // namespace pandar40
}  // namespace pandar_pointcloud