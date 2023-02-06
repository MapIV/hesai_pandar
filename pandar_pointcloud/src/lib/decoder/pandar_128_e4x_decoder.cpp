#include "pandar_pointcloud/decoder/pandar_128_e4x_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_128_e4x.hpp"

namespace
{
static inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
}  // namespace

namespace pandar_pointcloud
{
namespace pandar_128_e4x
{
Pandar128E4XDecoder::Pandar128E4XDecoder(Calibration& calibration,
                                         float scan_phase,
                                         double dual_return_distance_threshold,
                                         ReturnMode return_mode)
{
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration.elev_angle_map[laser];
    azimuth_offset_[laser] = calibration.azimuth_offset_map[laser];
  }

  size_t i = 0;
  for (const auto &angle:elev_angle_){
    auto rads = deg2rad(angle);
    elev_angle_rad_[i] = rads;
    cos_elev_angle_[i] = cosf(rads);
    sin_elev_angle_[i++] = sinf(rads);
  }

  scan_phase_ = static_cast<uint16_t>(scan_phase * 100.0f);
  dual_return_distance_threshold_ = dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new PointcloudXYZIRADT);
  scan_pc_->reserve(LASER_COUNT*MAX_AZIMUTH_STEPS);
  overflow_pc_.reset(new PointcloudXYZIRADT);
  overflow_pc_->reserve(LASER_COUNT*MAX_AZIMUTH_STEPS);
}

bool Pandar128E4XDecoder::hasScanned()
{
  return has_scanned_;
}

PointcloudXYZIRADT Pandar128E4XDecoder::getPointcloud()
{
  return scan_pc_;
}

bool Pandar128E4XDecoder::parsePacket(const pandar_msgs::PandarPacket& raw_packet)
{

  return true;
}

void Pandar128E4XDecoder::unpack(const pandar_msgs::PandarPacket& raw_packet)
{

  return;
}

PointXYZIRADT Pandar128E4XDecoder::build_point(int block_id, int unit_id, uint8_t return_type)
{

  PointXYZIRADT point;

  //double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));

  return point;
}

PointcloudXYZIRADT Pandar128E4XDecoder::convert(int block_id)
{
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);

  return block_pc;
}

PointcloudXYZIRADT Pandar128E4XDecoder::convert_dual(int block_id)
{
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);

  return block_pc;
}



}  // namespace pandar40
}  // namespace pandar_pointcloud