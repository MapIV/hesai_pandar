#include "pandar_pointcloud/decoder/pandar_qt_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_qt.hpp"

namespace
{
static inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
}

namespace pandar_pointcloud
{
namespace pandar_qt
{
PandarQTDecoder::PandarQTDecoder(Calibration& calibration, float scan_phase, double dual_return_distance_threshold, ReturnMode return_mode)
{
  firing_offset_ = {
    12.31,  14.37,  16.43,  18.49,  20.54,  22.6,   24.66,  26.71,  29.16,  31.22,  33.28,  35.34,  37.39,
    39.45,  41.5,   43.56,  46.61,  48.67,  50.73,  52.78,  54.84,  56.9,   58.95,  61.01,  63.45,  65.52,
    67.58,  69.63,  71.69,  73.74,  75.8,   77.86,  80.9,   82.97,  85.02,  87.08,  89.14,  91.19,  93.25,
    95.3,   97.75,  99.82,  101.87, 103.93, 105.98, 108.04, 110.1,  112.15, 115.2,  117.26, 119.32, 121.38,
    123.43, 125.49, 127.54, 129.6,  132.05, 134.11, 136.17, 138.22, 140.28, 142.34, 144.39, 146.45,
  };

  for (int block = 0; block < BLOCK_NUM; ++block) {
    block_offset_single_[block] = 25.71f + 500.00f / 3.0f * block;
    block_offset_dual_[block] = 25.71f + 500.00f / 3.0f * (block / 2);
  }

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }
  for (size_t laser = 0; laser < UNIT_NUM; ++laser) {
    elev_angle_[laser] = calibration.elev_angle_map[laser];
    azimuth_offset_[laser] = calibration.azimuth_offset_map[laser];
  }

  scan_phase_ = static_cast<uint16_t>(scan_phase * 100.0f);
  return_mode_ = return_mode;
  dual_return_distance_threshold_ = dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
  overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
}

bool PandarQTDecoder::hasScanned()
{
  return has_scanned_;
}

PointcloudXYZIRADT PandarQTDecoder::getPointcloud()
{
  return scan_pc_;
}

void PandarQTDecoder::unpack(const pandar_msgs::PandarPacket& raw_packet)
{
  if (!parsePacket(raw_packet)) {
    return;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
    has_scanned_ = false;
  }

  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  auto step = dual_return ? 2 : 1;

  if (!dual_return) {
    if ((packet_.return_mode == FIRST_RETURN && return_mode_ != ReturnMode::FIRST) || 
        (packet_.return_mode == LAST_RETURN && return_mode_ != ReturnMode::LAST)) {
      ROS_WARN ("Sensor return mode configuration does not match requested return mode");
    }
  }

  for (int block_id = 0; block_id < BLOCK_NUM; block_id += step) {
    auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
    int current_phase = (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
    if (current_phase > last_phase_ && !has_scanned_) {
      *scan_pc_ += *block_pc;
    }
    else {
      *overflow_pc_ += *block_pc;
      has_scanned_ = true;
    }
    last_phase_ = current_phase;
  }
  return;
}

void PandarQTDecoder::unpack(const pandar_msgs::PandarPacket2& raw_packet)
{
  if (!parsePacket(raw_packet)) {
    return;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
    has_scanned_ = false;
  }

  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  auto step = dual_return ? 2 : 1;

  if (!dual_return) {
    if ((packet_.return_mode == FIRST_RETURN && return_mode_ != ReturnMode::FIRST) || 
        (packet_.return_mode == LAST_RETURN && return_mode_ != ReturnMode::LAST)) {
      ROS_WARN ("Sensor return mode configuration does not match requested return mode");
    }
  }

  for (int block_id = 0; block_id < BLOCK_NUM; block_id += step) {
    auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
    int current_phase = (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
    if (current_phase > last_phase_ && !has_scanned_) {
      *scan_pc_ += *block_pc;
    }
    else {
      *overflow_pc_ += *block_pc;
      has_scanned_ = true;
    }
    last_phase_ = current_phase;
  }
  return;
}

PointXYZIRADT PandarQTDecoder::build_point(int block_id, int unit_id, uint8_t return_type)
{
  const auto& block = packet_.blocks[block_id];
  const auto& unit = block.units[unit_id];
  double unix_second = static_cast<double>(timegm(&packet_.t));
  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  PointXYZIRADT point;

  double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));

  point.x = static_cast<float>(
      xyDistance * sinf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
  point.y = static_cast<float>(
      xyDistance * cosf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
  point.z = static_cast<float>(unit.distance * sinf(deg2rad(elev_angle_[unit_id])));

  point.intensity = unit.intensity;
  point.distance = unit.distance;
  point.ring = unit_id;
  point.azimuth = block.azimuth + round(azimuth_offset_[unit_id] * 100.0f);
  point.return_type = return_type;
  point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
  point.time_stamp += dual_return ? (static_cast<double>(block_offset_dual_[block_id] + firing_offset_[unit_id]) / 1000000.0f) :
                                    (static_cast<double>(block_offset_single_[block_id] + firing_offset_[unit_id]) / 1000000.0f); 

  return point;
}

PointcloudXYZIRADT PandarQTDecoder::convert(const int block_id)
{
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);

  const auto& block = packet_.blocks[block_id];
  for (size_t unit_id = 0; unit_id < UNIT_NUM; ++unit_id) {
    PointXYZIRADT point;
    const auto& unit = block.units[unit_id];
    // skip invalid points
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }
    block_pc->push_back(build_point(block_id, unit_id, (packet_.return_mode == FIRST_RETURN) ? ReturnType::SINGLE_FIRST : ReturnType::SINGLE_LAST));
  }
  return block_pc;
}

PointcloudXYZIRADT PandarQTDecoder::convert_dual(const int block_id)
{
  //   Under the Dual Return mode, the ranging data from each firing is stored in two adjacent blocks:
  // · The even number block is the first return
  // · The odd number block is the last return
  // · The Azimuth changes every two blocks
  // · Important note: Hesai datasheet block numbering starts from 0, not 1, so odd/even are reversed here 
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);

  int even_block_id = block_id;
  int odd_block_id = block_id + 1;
  const auto& even_block = packet_.blocks[even_block_id];
  const auto& odd_block = packet_.blocks[odd_block_id];

  for (size_t unit_id = 0; unit_id < UNIT_NUM; ++unit_id) {

    const auto& even_unit = even_block.units[unit_id];
    const auto& odd_unit = odd_block.units[unit_id];

    bool even_usable = (even_unit.distance <= 0.1 || even_unit.distance > 200.0) ? 0 : 1;
    bool odd_usable = (odd_unit.distance <= 0.1 || odd_unit.distance > 200.0) ? 0 : 1;  

    if (return_mode_ == ReturnMode::FIRST && even_usable) {
      // First return is in even block
      block_pc->push_back(build_point(even_block_id, unit_id, ReturnType::SINGLE_FIRST));     
    }
    else if (return_mode_ == ReturnMode::LAST && even_usable) {
      // Last return is in odd block
      block_pc->push_back(build_point(odd_block_id, unit_id, ReturnType::SINGLE_LAST)); 
    }
    else if (return_mode_ == ReturnMode::DUAL) {
      // If the two returns are too close, only return the last one
      if ((abs(even_unit.distance - odd_unit.distance) < dual_return_distance_threshold_) && odd_usable) {
        block_pc->push_back(build_point(odd_block_id, unit_id, ReturnType::DUAL_ONLY));
      }
      else {
        if (even_usable) {
          block_pc->push_back(build_point(even_block_id, unit_id, ReturnType::DUAL_FIRST));
        }
        if (odd_usable) {
          block_pc->push_back(build_point(odd_block_id, unit_id, ReturnType::DUAL_LAST));
        }
      }
    }
  }
  return block_pc;
}

bool PandarQTDecoder::parsePacket(const pandar_msgs::PandarPacket& raw_packet)
{
  if (raw_packet.size != PACKET_SIZE && raw_packet.size != PACKET_WITHOUT_UDPSEQ_SIZE) {
    return false;
  }
  const uint8_t* buf = &raw_packet.data[0];

  return parseBinary(buf);
}

bool PandarQTDecoder::parsePacket(const pandar_msgs::PandarPacket2& raw_packet)
{
  if (raw_packet.size != PACKET_SIZE && raw_packet.size != PACKET_WITHOUT_UDPSEQ_SIZE) {
    return false;
  }
  const uint8_t* buf = &raw_packet.data[0];

  return parseBinary(buf);
}

bool PandarQTDecoder::parseBinary(const uint8_t* buf)
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
  index += ENGINE_VELOCITY;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 | ((buf[index + 2] & 0xff) << 16) |
                 ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;

  packet_.return_mode = buf[index] & 0xff;

  index += RETURN_SIZE;
  index += FACTORY_SIZE;

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

  return true;
}
}
}