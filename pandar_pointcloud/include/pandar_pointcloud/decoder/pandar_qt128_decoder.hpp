#pragma once

#include <array>
#include "pandar_pointcloud/calibration.hpp"
#include "packet_decoder.hpp"
#include "pandar_qt128.hpp"

namespace pandar_pointcloud
{
namespace pandar_qt128
{
class PandarQT128Decoder : public PacketDecoder
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

  PandarQT128Decoder(Calibration& calibration, float scan_phase = 0.0f, double dual_return_distance_threshold = 0.1, ReturnMode return_mode = ReturnMode::DUAL);
  /// @brief Parsing and shaping PandarPacket2
  /// @param pandar_packet
  void unpack(const pandar_msgs::PandarPacket& raw_packet);
  void unpack(const pandar_msgs::PandarPacket2& raw_packet);
  /// @brief Get the flag indicating whether one cycle is ready
  /// @return Readied
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;

private:
  /// @brief Parsing PandarPacket2 based on packet structure
  /// @param pandar_packet
  /// @return Resulting flag
  bool parsePacket(const pandar_msgs::PandarPacket& pandar_packet);
  bool parsePacket(const pandar_msgs::PandarPacket2& pandar_packet);
  bool parseBinary(const uint8_t * buf);
  /// @brief Constructing a point cloud of the target part
  /// @param block_id Target block
  /// @param unit_id Target unit
  /// @param dual_return Return mode is dual
  /// @param unix_second Packet time
  /// @return Point cloud
  PointXYZIRADT build_point(
    size_t block_id, size_t unit_id, bool dual_return, const double & unix_second);
  /// @brief Convert to point cloud
  /// @param block_id target block
  /// @return Point cloud
  PointcloudXYZIRADT convert(size_t block_id);
  /// @brief Convert to point cloud for dual return
  /// @param block_id target block
  /// @return Point cloud
  //PointcloudXYZIRADT convert_dual(size_t block_id);

  /// @brief Checking packet_.return mode
  /// @return packet_.return mode is dual mode
  bool is_dual_return();

  std::array<float, LASER_COUNT> elevation_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};
  std::array<float, LASER_COUNT> elev_angle_rad_{};
  std::array<float, LASER_COUNT> cos_elevation_angle_{};
  std::array<float, LASER_COUNT> sin_elevation_angle_{};
  std::array<float, LASER_COUNT> elevation_angle_rad_{};
  std::array<float, LASER_COUNT> azimuth_offset_rad_{};

  std::array<float, MAX_AZIMUTH_STEPS> block_azimuth_rad_{};

  std::map<int, float> firing_time_offset1_{};
  std::map<int, float> firing_time_offset2_{};

  std::array<float, BLOCKS_PER_PACKET> block_time_offset_single_{};
  std::array<float, BLOCKS_PER_PACKET> block_time_offset_dual_{};

  uint8_t first_return_type_{};
  uint8_t second_return_type_{};

  Packet packet_{};
  double scan_timestamp_;
  double scan_phase_, last_phase_;
  double dual_return_distance_threshold_;
  PointcloudXYZIRADT scan_pc_, overflow_pc_;
  bool has_scanned_;
};

}  // namespace pandar_qt128
}  // namespace pandar_pointcloud
