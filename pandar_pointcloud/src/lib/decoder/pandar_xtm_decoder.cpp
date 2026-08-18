#include "pandar_pointcloud/decoder/pandar_xtm_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_xtm.hpp"

namespace
{
static inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
}

namespace pandar_pointcloud
{
namespace pandar_xtm
{
PandarXTMDecoder::PandarXTMDecoder(Calibration& calibration, float scan_phase, double dual_return_distance_threshold, ReturnMode return_mode)
{
  for (size_t laser = 0; laser < UNIT_NUM; ++laser) {
    elev_angle_[laser] = calibration.elev_angle_map[laser];
    azimuth_offset_[laser] = calibration.azimuth_offset_map[laser];
  }

  m_sin_elevation_map_.resize(UNIT_NUM);
  m_cos_elevation_map_.resize(UNIT_NUM);
  for (size_t laser = 0; laser < UNIT_NUM; ++laser) {
    m_sin_elevation_map_[laser] = sinf(deg2rad(elev_angle_[laser]));
    m_cos_elevation_map_[laser] = cosf(deg2rad(elev_angle_[laser]));
  }
  m_sin_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  m_cos_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  for(int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    m_sin_azimuth_map_[i] = sinf(i * M_PI / 18000);
    m_cos_azimuth_map_[i] = cosf(i * M_PI / 18000);
  }

  scan_phase_ = static_cast<uint16_t>(scan_phase * 100.0f);
  return_mode_ = return_mode;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
  overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
}

bool PandarXTMDecoder::hasScanned()
{
  return has_scanned_;
}

PointcloudXYZIRADT PandarXTMDecoder::getPointcloud()
{
  return scan_pc_;
}

void PandarXTMDecoder::unpack(const pandar_msgs::PandarPacket& raw_packet)
{
  if (!parsePacket(raw_packet)) {
    return;
  }
  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
    has_scanned_ = false;
  }
  for (int block_id = 0; block_id < packet_.header.chBlockNumber; ++block_id) {
    int azimuthGap = 0; /* To do */
    double timestampGap = 0; /* To do */
    if(last_azimuth_ > packet_.blocks[block_id].azimuth) {
      azimuthGap = static_cast<int>(packet_.blocks[block_id].azimuth) + (36000 - static_cast<int>(last_azimuth_));
    } else {
      azimuthGap = static_cast<int>(packet_.blocks[block_id].azimuth) - static_cast<int>(last_azimuth_);
    }
    timestampGap = packet_.usec - last_timestamp_ + 0.001;
    if (last_azimuth_ != packet_.blocks[block_id].azimuth && \
            (azimuthGap / timestampGap) < 36000 * 100 ) {
      /* for all the blocks */
      if ((last_azimuth_ > packet_.blocks[block_id].azimuth &&
           start_angle_ <= packet_.blocks[block_id].azimuth) ||
          (last_azimuth_ < start_angle_ &&
           start_angle_ <= packet_.blocks[block_id].azimuth)) {
          has_scanned_ = true;
      }
    } else {
      //printf("last_azimuth_:%d pkt.blocks[block_id].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[block_id].azimuth, azimuthGap);
    }
    CalcXTPointXYZIT(block_id, packet_.header.chLaserNumber, scan_pc_);
    last_azimuth_ = packet_.blocks[block_id].azimuth;
    last_timestamp_ = packet_.usec;
  }
}

void PandarXTMDecoder::unpack(const pandar_msgs::PandarPacket2& raw_packet)
{
  if (!parsePacket(raw_packet)) {
    return;
  }
  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
    has_scanned_ = false;
  }
  for (int block_id = 0; block_id < packet_.header.chBlockNumber; ++block_id) {
    int azimuthGap = 0; /* To do */
    double timestampGap = 0; /* To do */
    if(last_azimuth_ > packet_.blocks[block_id].azimuth) {
      azimuthGap = static_cast<int>(packet_.blocks[block_id].azimuth) + (36000 - static_cast<int>(last_azimuth_));
    } else {
      azimuthGap = static_cast<int>(packet_.blocks[block_id].azimuth) - static_cast<int>(last_azimuth_);
    }
    timestampGap = packet_.usec - last_timestamp_ + 0.001;
    if (last_azimuth_ != packet_.blocks[block_id].azimuth && \
            (azimuthGap / timestampGap) < 36000 * 100 ) {
      /* for all the blocks */
      if ((last_azimuth_ > packet_.blocks[block_id].azimuth &&
           start_angle_ <= packet_.blocks[block_id].azimuth) ||
          (last_azimuth_ < start_angle_ &&
           start_angle_ <= packet_.blocks[block_id].azimuth)) {
          has_scanned_ = true;
      }
    } else {
      //printf("last_azimuth_:%d pkt.blocks[block_id].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[block_id].azimuth, azimuthGap);
    }
    CalcXTPointXYZIT(block_id, packet_.header.chLaserNumber, scan_pc_);
    last_azimuth_ = packet_.blocks[block_id].azimuth;
    last_timestamp_ = packet_.usec;
  }
}

void PandarXTMDecoder::CalcXTPointXYZIT(int blockid, \
    char chLaserNumber, boost::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld) {
  Block *block = &packet_.blocks[blockid];

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    Unit &unit = block->units[i];
    PointXYZIRADT point{};

    /* skip wrong points */
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    // Ported from the batch pipeline's decodeChannelSDK (model_decoders.hpp): compute in fine
    // angle units (1/25600 degree) so applyDistanceCorrection's ray-sphere intersection sees the
    // same precision the batch pipeline uses, instead of this decoder's coarser 0.01-degree table.
    {
      float distance = static_cast<float>(unit.distance);
      int fine_azimuth = static_cast<int>(block->azimuth) * hesai::AngleLookupTable::FINE_RESOLUTION_INT;

      const double firetime_deg = static_cast<double>(pandarXTM_firetime[i]) * packet_.motor_speed * 6e-6;
      if (firetime_deg != 0.0)
        fine_azimuth += static_cast<int>(firetime_deg * hesai::AngleLookupTable::ALL_FINE_RESOLUTION_INT + 0.0625);

      int azimuth_coll = static_cast<int>(
          azimuth_offset_[i] * hesai::AngleLookupTable::ALL_FINE_RESOLUTION_FLOAT + 0.0625f);
      int elevation_corr = static_cast<int>(
          elev_angle_[i] * hesai::AngleLookupTable::ALL_FINE_RESOLUTION_FLOAT + 0.0625f);

      hesai::applyDistanceCorrection(pandarXTM_optical_center, angle_lut_, azimuth_coll, elevation_corr, distance);

      fine_azimuth += azimuth_coll;
      hesai::AngleLookupTable::circleRevise(fine_azimuth);
      int elevation = elevation_corr;
      hesai::AngleLookupTable::circleRevise(elevation);

      float xyDistance = distance * angle_lut_.cos(elevation);
      point.x = xyDistance * angle_lut_.sin(fine_azimuth);
      point.y = xyDistance * angle_lut_.cos(fine_azimuth);
      point.z = distance * angle_lut_.sin(elevation);
    }

    point.intensity = unit.intensity;

    double unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)
    point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
    point.time_stamp += (static_cast<double>(blockXTMOffsetSingle[blockid] + laserXTMOffset[i]) / 1000000.0f);

    if (packet_.return_mode == 0x3d){
      point.time_stamp =
        point.time_stamp + (static_cast<double>(blockXTMOffsetTriple[blockid] +
          laserXTMOffset[i]) /
                           1000000.0f);
    }
    else if (packet_.return_mode == 0x39 || packet_.return_mode == 0x3b || packet_.return_mode == 0x3c) {
      point.time_stamp =
        point.time_stamp + (static_cast<double>(blockXTMOffsetDual[blockid] +
          laserXTMOffset[i]) /
                           1000000.0f);
    } else {
      point.time_stamp = point.time_stamp + \
          (static_cast<double>(blockXTMOffsetSingle[blockid] + laserXTMOffset[i]) / \
          1000000.0f);
    }

    point.return_type = packet_.return_mode;
    point.ring = i;
    cld->points.emplace_back(point);
  }
}

bool PandarXTMDecoder::parsePacket(const pandar_msgs::PandarPacket& raw_packet)
{
  if (raw_packet.size != PACKET_SIZE) {
    return false;
  }
  const uint8_t* buf = &raw_packet.data[0];

  return parseBinary(buf);
}

bool PandarXTMDecoder::parsePacket(const pandar_msgs::PandarPacket2& raw_packet)
{
  if (raw_packet.size != PACKET_SIZE) {
    return false;
  }
  const uint8_t* buf = &raw_packet.data[0];

  return parseBinary(buf);
}

bool PandarXTMDecoder::parseBinary(const uint8_t* buf)
{
  size_t index = 0;
  // Parse 12 Bytes Header
  packet_.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
  packet_.header.chProtocolMajor = buf[index + 2] & 0xff;
  packet_.header.chProtocolMinor = buf[index + 3] & 0xff;
  packet_.header.chLaserNumber = buf[index + 6] & 0xff;
  packet_.header.chBlockNumber = buf[index + 7] & 0xff;
  packet_.header.chReturnType = buf[index + 8] & 0xff;
  packet_.header.chDisUnit = buf[index + 9] & 0xff;
  index += HEAD_SIZE;

  if (packet_.header.sob != 0xEEFF) {
    // Error Start of Packet!
    return false;
  }

  for (size_t block = 0; block < packet_.header.chBlockNumber; block++) {
    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    index += BLOCK_HEADER_AZIMUTH;

    for (int unit = 0; unit < packet_.header.chLaserNumber; unit++) {
      unsigned int unRange = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      packet_.blocks[block].units[unit].distance =
          (static_cast<double>(unRange * packet_.header.chDisUnit)) / (double)1000;
      packet_.blocks[block].units[unit].intensity = (buf[index + 2] & 0xff);
      packet_.blocks[block].units[unit].confidence = (buf[index + 3] & 0xff);
      index += UNIT_SIZE;
    }
  }

  index += RESERVED_SIZE;  // skip reserved bytes
  packet_.return_mode = buf[index] & 0xff;

  index += RETURN_SIZE;

  packet_.motor_speed = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
  index += ENGINE_VELOCITY;

  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;
  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet_.t.tm_mday = buf[index + 2] & 0xff;
  packet_.t.tm_hour = buf[index + 3] & 0xff;
  packet_.t.tm_min = buf[index + 4] & 0xff;
  packet_.t.tm_sec = buf[index + 5] & 0xff;
  packet_.t.tm_isdst = 0;
  // in case of time error
  if (packet_.t.tm_year >= 200) {
    packet_.t.tm_year -= 100;
  }

  index += UTC_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 | ((buf[index + 2] & 0xff) << 16) |
                 ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;
  index += FACTORY_SIZE;

  return true;
}
}
}