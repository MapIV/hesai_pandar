#pragma once

#include <array>
#include "pandar_pointcloud/calibration.hpp"
#include "packet_decoder.hpp"
#include "pandar_xtm.hpp"
#include "pandar_pointcloud/decoder/hesai_distance_correction.hpp"

namespace pandar_pointcloud
{
namespace pandar_xtm
{

  // Ported from the batch pipeline (multi_lidar_controller/lidar_drivers/hesai) so the realtime
  // decoder applies the same geometric distance correction the batch pipeline always had.
  const hesai::OpticalCenter pandarXTM_optical_center{ -0.013f, 0.0305f, 0.0f };

  // Per-channel firetime in microseconds, ported from the batch pipeline's
  // pandarXTM_default_firetime (hesai_lidar/data/default_firetime.hpp).
  const float pandarXTM_firetime[] = {
    6.0f,    8.888f,  11.776f, 14.664f, 17.552f, 20.44f,  23.328f, 26.216f,
    29.104f, 31.992f, 34.88f,  37.768f, 40.656f, 43.544f, 46.432f, 49.32f,
    6.0f,    8.888f,  11.776f, 14.664f, 17.552f, 20.44f,  23.328f, 26.216f,
    29.104f, 31.992f, 34.88f,  37.768f, 40.656f, 43.544f, 46.432f, 49.32f
  };

  const float pandarXTM_elev_angle_map[] = {
    19.5f, 18.2f, 16.9f, 15.6f, 14.3f, 13.0f, 11.7f, 10.4f, \
  9.1f,  7.8f,  6.5f,  5.2f,  3.9f,  2.6f,  1.3f, 0.0f,  \
  -1.3f, -2.6f, -3.9f, -5.2f, -6.5f, -7.8f, -9.1f, -10.4f, \
  -11.7f, -13.0f, -14.3f, -15.6f, -16.9f, -18.2f, -19.5f, -20.8f
  };

  const float pandarXTM_horizontal_azimuth_offset_map[] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
  };

  const float blockXTMOffsetTriple[] = {
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f
  };

  const float blockXTMOffsetDual[] = {
    5.632f - 50.0f * 2.0f,
    5.632f - 50.0f * 2.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f
  };
  const float blockXTMOffsetSingle[] = {
    5.632f - 50.0f * 5.0f,
    5.632f - 50.0f * 4.0f,
    5.632f - 50.0f * 3.0f,
    5.632f - 50.0f * 2.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 0.0f
  };

  const float laserXTMOffset[] = {
    2.856f * 0.0f + 0.368f,
    2.856f * 1.0f + 0.368f,
    2.856f * 2.0f + 0.368f,
    2.856f * 3.0f + 0.368f,
    2.856f * 4.0f + 0.368f,
    2.856f * 5.0f + 0.368f,
    2.856f * 6.0f + 0.368f,
    2.856f * 7.0f + 0.368f,

    2.856f * 8.0f + 0.368f,
    2.856f * 9.0f + 0.368f,
    2.856f * 10.0f + 0.368f,
    2.856f * 11.0f + 0.368f,
    2.856f * 12.0f + 0.368f,
    2.856f * 13.0f + 0.368f,
    2.856f * 14.0f + 0.368f,
    2.856f * 15.0f + 0.368f,

    2.856f * 0.0f + 0.368f,
    2.856f * 1.0f + 0.368f,
    2.856f * 2.0f + 0.368f,
    2.856f * 3.0f + 0.368f,
    2.856f * 4.0f + 0.368f,
    2.856f * 5.0f + 0.368f,
    2.856f * 6.0f + 0.368f,
    2.856f * 7.0f + 0.368f,

    2.856f * 8.0f + 0.368f,
    2.856f * 9.0f + 0.368f,
    2.856f * 10.0f + 0.368f,
    2.856f * 11.0f + 0.368f,
    2.856f * 12.0f + 0.368f,
    2.856f * 13.0f + 0.368f,
    2.856f * 14.0f + 0.368f,
    2.856f * 15.0f + 0.368f
  };

const uint16_t MAX_AZIMUTH_DEGREE_NUM=36000;

class PandarXTMDecoder : public PacketDecoder
{
public:
  enum class ReturnMode : int8_t
  {
    DUAL,
    FIRST,
    STRONGEST,
    LAST,
    TRIPLE
  };

  PandarXTMDecoder(Calibration& calibration, float scan_phase = 0.0f, double dual_return_distance_threshold = 0.1, ReturnMode return_mode = ReturnMode::DUAL);
  void unpack(const pandar_msgs::PandarPacket& raw_packet) override;
  void unpack(const pandar_msgs::PandarPacket2& raw_packet) override;
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;

private:
  bool parsePacket(const pandar_msgs::PandarPacket& raw_packet);
  bool parsePacket(const pandar_msgs::PandarPacket2& raw_packet);
  bool parseBinary(const uint8_t* buf);
  PointcloudXYZIRADT convert(const int block_id);

  std::array<float, UNIT_NUM> elev_angle_;
  std::array<float, UNIT_NUM> azimuth_offset_;

  std::array<float, BLOCK_NUM> block_offset_single_;
  std::array<float, BLOCK_NUM> block_offset_dual_;
  std::array<float, BLOCK_NUM> block_offset_triple_;

  std::vector<float> m_sin_elevation_map_;
  std::vector<float> m_cos_elevation_map_;

  std::vector<float> m_sin_azimuth_map_;
  std::vector<float> m_cos_azimuth_map_;

  // Used for the ported distance-correction path (see CalcXTPointXYZIT); operates on fine angle
  // units (1/25600 degree) as the batch pipeline does, independent of m_sin/cos_azimuth_map_'s
  // coarser 0.01-degree table used for the uncorrected fast path.
  hesai::AngleLookupTable angle_lut_;

  ReturnMode return_mode_;
  Packet packet_;

  PointcloudXYZIRADT scan_pc_;
  PointcloudXYZIRADT overflow_pc_;

  uint16_t scan_phase_;
  int last_phase_;
  bool has_scanned_;
  uint16_t last_azimuth_;
  int start_angle_;
  double last_timestamp_;

  void CalcXTPointXYZIT(int blockid, char chLaserNumber, boost::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld);
};

}  // namespace pandar_xt
}  // namespace pandar_pointcloud
