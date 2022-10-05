#include "pandar_pointcloud/decoder/pandar_qt128_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_qt128.hpp"

namespace
{
static inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
}  // namespace

namespace pandar_pointcloud
{
namespace pandar_qt128
{
PandarQT128Decoder::PandarQT128Decoder(Calibration& calibration, float scan_phase,
                                       double dual_return_distance_threshold, ReturnMode return_mode)
{
  initFiringOffset();

  block_offset_single_[1] = 7.0f * 1.0f + 111.11f;
  block_offset_single_[0] = 7.0f * 0.0f + 111.11f;

  block_offset_dual_[1] = 7.0f * 0.0f + 111.11f;
  block_offset_dual_[0] = 7.0f * 0.0f + 111.11f;

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }

  scan_phase_ = static_cast<uint16_t>(scan_phase * 100.0f);
  return_mode_ = return_mode;
  dual_return_distance_threshold_ = dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
  overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
}

bool PandarQT128Decoder::hasScanned()
{
  return has_scanned_;
}

PointcloudXYZIRADT PandarQT128Decoder::getPointcloud()
{
  return scan_pc_;
}

void PandarQT128Decoder::unpack(const pandar_msgs::PandarPacket& raw_packet)
{
  if (!parsePacket(raw_packet))
  {
    return;
  }

  if (has_scanned_)
  {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
    has_scanned_ = false;
  }

  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  auto step = dual_return ? 2 : 1;

  if (!dual_return)
  {
    if ((packet_.return_mode == FIRST_RETURN && return_mode_ != ReturnMode::FIRST) ||
        (packet_.return_mode == LAST_RETURN && return_mode_ != ReturnMode::LAST))
    {
      ROS_WARN("Sensor return mode configuration does not match requested return mode");
    }
  }

  for (int block_id = 0; block_id < BLOCK_NUM; block_id += step)
  {
    auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
    int current_phase = (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
    if (current_phase > last_phase_ && !has_scanned_)
    {
      *scan_pc_ += *block_pc;
    }
    else
    {
      *overflow_pc_ += *block_pc;
      has_scanned_ = true;
    }
    last_phase_ = current_phase;
  }
  return;
}

PointXYZIRADT PandarQT128Decoder::build_point(int block_id, int unit_id, int seq_id, uint8_t return_type)
{
  const auto& block = packet_.blocks[block_id];
  const auto& unit = block.units[unit_id];
  double unix_second = static_cast<double>(timegm(&packet_.t));
  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  PointXYZIRADT point;

  double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));

  point.x = static_cast<float>(xyDistance *
                               sinf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) * 1e-02)));
  point.y = static_cast<float>(xyDistance *
                               cosf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) * 1e-02)));
  point.z = static_cast<float>(unit.distance * sinf(deg2rad(elev_angle_[unit_id])));

  point.intensity = unit.intensity;
  point.distance = unit.distance;
  point.ring = unit_id;
  point.azimuth = block.azimuth + round(azimuth_offset_[unit_id] * 100.0f);
  point.return_type = return_type;
  point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) * 1e-06f;
  point.time_stamp +=
      dual_return ? (static_cast<double>(block_offset_dual_[block_id] + firing_offset_[seq_id][unit_id]) * 1e-06f) :
                    (static_cast<double>(block_offset_single_[block_id] + firing_offset_[seq_id][unit_id]) * 1e-06f);

  return point;
}

PointcloudXYZIRADT PandarQT128Decoder::convert(const int block_id)
{
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);

  int seq_id = block_id;

  const auto& block = packet_.blocks[block_id];
  for (size_t unit_id = 0; unit_id < UNIT_NUM; ++unit_id)
  {
    PointXYZIRADT point;
    const auto& unit = block.units[unit_id];
    // skip invalid points
    if (unit.distance <= 0.1 || unit.distance > 200.0)
    {
      continue;
    }
    block_pc->push_back(
        build_point(block_id, unit_id, seq_id,
                    (packet_.return_mode == FIRST_RETURN) ? ReturnType::SINGLE_FIRST : ReturnType::SINGLE_LAST));
  }
  return block_pc;
}

PointcloudXYZIRADT PandarQT128Decoder::convert_dual(const int block_id)
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

  int seq_id;
  if (return_mode_ == ReturnMode::DUAL)
  {
    if ((packet_.mode_flag >> 0) & 0x01)
    {
      if ((packet_.mode_flag >> 1) & 0x01)
        seq_id = 0;
      else
        seq_id = block_id;
    }
    else
    {
      if ((packet_.mode_flag >> 1) & 0x01)
        seq_id = 1 - block_id;
      else
        seq_id = 1;
    }
  }
  else
  {
    seq_id = block_id;
  }

  for (size_t unit_id = 0; unit_id < UNIT_NUM; ++unit_id)
  {
    const auto& even_unit = even_block.units[unit_id];
    const auto& odd_unit = odd_block.units[unit_id];

    bool even_usable = (even_unit.distance <= 0.1 || even_unit.distance > 200.0) ? 0 : 1;
    bool odd_usable = (odd_unit.distance <= 0.1 || odd_unit.distance > 200.0) ? 0 : 1;

    if (return_mode_ == ReturnMode::FIRST && even_usable)
    {
      // First return is in even block
      block_pc->push_back(build_point(even_block_id, unit_id, seq_id, ReturnType::SINGLE_FIRST));
    }
    else if (return_mode_ == ReturnMode::LAST && even_usable)
    {
      // Last return is in odd block
      block_pc->push_back(build_point(odd_block_id, unit_id, seq_id, ReturnType::SINGLE_LAST));
    }
    else if (return_mode_ == ReturnMode::DUAL)
    {
      // If the two returns are too close, only return the last one
      if ((abs(even_unit.distance - odd_unit.distance) < dual_return_distance_threshold_) && odd_usable)
      {
        block_pc->push_back(build_point(odd_block_id, unit_id, seq_id, ReturnType::DUAL_ONLY));
      }
      else
      {
        if (even_usable)
        {
          block_pc->push_back(build_point(even_block_id, unit_id, seq_id, ReturnType::DUAL_FIRST));
        }
        if (odd_usable)
        {
          block_pc->push_back(build_point(odd_block_id, unit_id, seq_id, ReturnType::DUAL_LAST));
        }
      }
    }
  }
  return block_pc;
}

bool PandarQT128Decoder::parsePacket(const pandar_msgs::PandarPacket& raw_packet)
{
  if (raw_packet.size != PACKET_SIZE)
  {
    return false;
  }
  const uint8_t* buf = &raw_packet.data[0];

  size_t index = 0;
  // Parse 12 Bytes Header
  packet_.header.u16Sob = ((buf[index] & 0xff) << 8) | ((buf[index + 1] & 0xff));
  packet_.header.u8ProtocolMajor = buf[index + 2] & 0xff;
  packet_.header.u8ProtocolMinor = buf[index + 3] & 0xff;
  /* <--- Reserved 2 bytes [index + 4], [index + 5] ---> */
  packet_.header.u8LaserNum = buf[index + 6] & 0xff;
  packet_.header.u8BlockNum = buf[index + 7] & 0xff;
  /* <--- Reserved 1 byte [index + 8] ---> */
  packet_.header.u8DistUnit = buf[index + 9] & 0xff;
  packet_.header.u8EchoNum = buf[index + 10] & 0xff;
  packet_.header.u8Flags = buf[index + 11] & 0xff;
  index += HEAD_SIZE;

  if (packet_.header.u16Sob != 0xEEFF)
  {
    // Error Start of Packet!
    return false;
  }

  // Parse 1032 Bytes Body
  for (size_t block = 0; block < packet_.header.u8BlockNum; block++)
  {
    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    index += BLOCK_HEADER_AZIMUTH;

    for (int unit = 0; unit < packet_.header.u8LaserNum; unit++)
    {
      unsigned int unRange = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      packet_.blocks[block].units[unit].distance =
          (static_cast<double>(unRange * packet_.header.u8DistUnit)) / (double)1000;
      packet_.blocks[block].units[unit].intensity = (buf[index + 2] & 0xff);
      index += UNIT_SIZE;
    }
  }

  index += CRC_1_SIZE;

  // Parse 17 Bytes Functional Safety
  index += FUNCTIONAL_SAFETY_SIZE;

  // Parse 34 Bytes Tail
  index += RESERVED_1_SIZE;

  packet_.mode_flag = buf[index] & 0xff;
  index += MODE_FLAG_SIZE;

  index += RESERVED_2_SIZE;

  packet_.return_mode = buf[index] & 0xff;
  index += RETURN_MODE_SIZE;

  index += MOTOR_SPEED_SIZE;

  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;  // Year (Current year minus 1900)
  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;     // Month
  packet_.t.tm_mday = buf[index + 2] & 0xff;          // Day
  packet_.t.tm_hour = buf[index + 3] & 0xff;          // Hour
  packet_.t.tm_min = buf[index + 4] & 0xff;           // Minute
  packet_.t.tm_sec = buf[index + 5] & 0xff;           // Second
  packet_.t.tm_isdst = 0;

  // in case of time error
  if (packet_.t.tm_year >= 200)
  {
    packet_.t.tm_year -= 100;
  }

  index += DATE_TIME_SIZE;

  return true;
}

void PandarQT128Decoder::initFiringOffset()
{
  /* Firing Order 1-10 @ p. 62 User Manual */
  /* Firing Order 1 */
  firing_offset_[0][99 - 1] = 0.6f;
  firing_offset_[1][65 - 1] = 0.6f;

  /* Firing Order 2 */
  firing_offset_[0][65 - 1] = 1.456f;
  firing_offset_[1][99 - 1] = 1.456f;

  /* Firing Order 3 */
  firing_offset_[0][35 - 1] = 2.312f;
  firing_offset_[1][1 - 1] = 2.312f;

  /* Firing Order 4 */
  firing_offset_[0][102 - 1] = 3.768f;
  firing_offset_[1][72 - 1] = 3.768f;

  /* Firing Order 5 */
  firing_offset_[0][72 - 1] = 4.624f;
  firing_offset_[1][102 - 1] = 4.624f;

  /* Firing Order 6 */
  firing_offset_[0][38 - 1] = 5.48f;
  firing_offset_[1][8 - 1] = 5.48f;

  /* Firing Order 7 */
  firing_offset_[0][107 - 1] = 6.936f;
  firing_offset_[1][73 - 1] = 6.936f;

  /* Firing Order 8 */
  firing_offset_[0][73 - 1] = 7.792f;
  firing_offset_[1][107 - 1] = 7.792f;

  /* Firing Order 9 */
  firing_offset_[0][43 - 1] = 8.648f;
  firing_offset_[1][9 - 1] = 8.648f;

  /* Firing Order 10 */
  firing_offset_[0][110 - 1] = 10.104f;
  firing_offset_[1][80 - 1] = 10.104f;

  /* Firing Order 11-20 @ p. 62 User Manual */
  /* Firing Order 11 */
  firing_offset_[0][80 - 1] = 10.96f;
  firing_offset_[1][110 - 1] = 10.96f;

  /* Firing Order 12 */
  firing_offset_[0][46 - 1] = 11.816f;
  firing_offset_[1][16 - 1] = 11.816f;

  /* Firing Order 13 */
  firing_offset_[0][115 - 1] = 13.272f;
  firing_offset_[1][81 - 1] = 13.272f;

  /* Firing Order 14 */
  firing_offset_[0][81 - 1] = 14.128f;
  firing_offset_[1][115 - 1] = 14.128f;

  /* Firing Order 15 */
  firing_offset_[0][51 - 1] = 14.984f;
  firing_offset_[1][17 - 1] = 14.984f;

  /* Firing Order 16 */
  firing_offset_[0][118 - 1] = 16.44f;
  firing_offset_[1][88 - 1] = 16.44f;

  /* Firing Order 17 */
  firing_offset_[0][88 - 1] = 17.296f;
  firing_offset_[1][118 - 1] = 17.296f;

  /* Firing Order 18 */
  firing_offset_[0][54 - 1] = 18.152f;
  firing_offset_[1][24 - 1] = 18.152f;

  /* Firing Order 19 */
  firing_offset_[0][123 - 1] = 19.608f;
  firing_offset_[1][89 - 1] = 19.608f;

  /* Firing Order 20 */
  firing_offset_[0][89 - 1] = 20.464f;
  firing_offset_[1][123 - 1] = 20.464f;

  /* Firing Order 21-30 @ p. 62 User Manual */
  /* Firing Order 21 */
  firing_offset_[0][59 - 1] = 21.32f;
  firing_offset_[1][25 - 1] = 21.32f;

  /* Firing Order 22 */
  firing_offset_[0][126 - 1] = 22.776f;
  firing_offset_[1][96 - 1] = 22.776f;

  /* Firing Order 23 */
  firing_offset_[0][96 - 1] = 23.632f;
  firing_offset_[1][126 - 1] = 23.632f;

  /* Firing Order 24 */
  firing_offset_[0][62 - 1] = 24.488f;
  firing_offset_[1][32 - 1] = 24.488f;

  /* Firing Order 25 */
  firing_offset_[0][97 - 1] = 25.944f;
  firing_offset_[1][67 - 1] = 25.944f;

  /* Firing Order 26 */
  firing_offset_[0][67 - 1] = 26.8f;
  firing_offset_[1][97 - 1] = 26.8f;

  /* Firing Order 27 */
  firing_offset_[0][33 - 1] = 27.656f;
  firing_offset_[1][3 - 1] = 27.656f;

  /* Firing Order 28 */
  firing_offset_[0][104 - 1] = 29.112f;
  firing_offset_[1][70 - 1] = 29.112f;

  /* Firing Order 29 */
  firing_offset_[0][70 - 1] = 29.968f;
  firing_offset_[1][104 - 1] = 29.968f;

  /* Firing Order 30 */
  firing_offset_[0][40 - 1] = 30.824f;
  firing_offset_[1][6 - 1] = 30.824f;

  /* Firing Order 31-40 @ p. 62 User Manual */
  /* Firing Order 31 */
  firing_offset_[0][105 - 1] = 32.28f;
  firing_offset_[1][75 - 1] = 32.28f;

  /* Firing Order 32 */
  firing_offset_[0][75 - 1] = 33.136f;
  firing_offset_[1][105 - 1] = 33.136f;

  /* Firing Order 33 */
  firing_offset_[0][41 - 1] = 33.992f;
  firing_offset_[1][11 - 1] = 33.992f;

  /* Firing Order 34 */
  firing_offset_[0][112 - 1] = 35.448f;
  firing_offset_[1][78 - 1] = 35.448f;

  /* Firing Order 35 */
  firing_offset_[0][78 - 1] = 36.304f;
  firing_offset_[1][112 - 1] = 36.304f;

  /* Firing Order 36 */
  firing_offset_[0][48 - 1] = 37.16f;
  firing_offset_[1][14 - 1] = 37.16f;

  /* Firing Order 37 */
  firing_offset_[0][113 - 1] = 38.616f;
  firing_offset_[1][83 - 1] = 38.616f;

  /* Firing Order 38 */
  firing_offset_[0][83 - 1] = 39.472f;
  firing_offset_[1][113 - 1] = 39.472f;

  /* Firing Order 39 */
  firing_offset_[0][49 - 1] = 40.328f;
  firing_offset_[1][19 - 1] = 40.328f;

  /* Firing Order 40 */
  firing_offset_[0][120 - 1] = 41.784f;
  firing_offset_[1][86 - 1] = 41.784f;

  /* Firing Order 41-50 @ p. 63 User Manual */
  /* Firing Order 41 */
  firing_offset_[0][86 - 1] = 42.64f;
  firing_offset_[1][120 - 1] = 42.64f;

  /* Firing Order 42 */
  firing_offset_[0][56 - 1] = 43.496f;
  firing_offset_[1][22 - 1] = 43.496f;

  /* Firing Order 43 */
  firing_offset_[0][121 - 1] = 44.952f;
  firing_offset_[1][91 - 1] = 44.952f;

  /* Firing Order 44 */
  firing_offset_[0][91 - 1] = 45.808f;
  firing_offset_[1][121 - 1] = 45.808f;

  /* Firing Order 45 */
  firing_offset_[0][57 - 1] = 46.664f;
  firing_offset_[1][27 - 1] = 46.664f;

  /* Firing Order 46 */
  firing_offset_[0][128 - 1] = 48.12f;
  firing_offset_[1][94 - 1] = 48.12f;

  /* Firing Order 47 */
  firing_offset_[0][94 - 1] = 48.976f;
  firing_offset_[1][128 - 1] = 48.976f;

  /* Firing Order 48 */
  firing_offset_[0][64 - 1] = 49.832f;
  firing_offset_[1][30 - 1] = 49.832f;

  /* Firing Order 49 */
  firing_offset_[0][98 - 1] = 51.288f;
  firing_offset_[1][68 - 1] = 51.288f;

  /* Firing Order 50 */
  firing_offset_[0][68 - 1] = 52.144f;
  firing_offset_[1][98 - 1] = 52.144f;

  /* Firing Order 51-60 @ p. 63 User Manual */
  /* Firing Order 51 */
  firing_offset_[0][34 - 1] = 53.0f;
  firing_offset_[1][4 - 1] = 53.0f;

  /* Firing Order 52 */
  firing_offset_[0][103 - 1] = 54.456f;
  firing_offset_[1][69 - 1] = 54.456f;

  /* Firing Order 53 */
  firing_offset_[0][69 - 1] = 55.312f;
  firing_offset_[1][103 - 1] = 55.312f;

  /* Firing Order 54 */
  firing_offset_[0][39 - 1] = 56.168f;
  firing_offset_[1][5 - 1] = 56.168f;

  /* Firing Order 55 */
  firing_offset_[0][106 - 1] = 57.624f;
  firing_offset_[1][76 - 1] = 57.624f;

  /* Firing Order 56 */
  firing_offset_[0][76 - 1] = 58.48f;
  firing_offset_[1][106 - 1] = 58.48f;

  /* Firing Order 57 */
  firing_offset_[0][42 - 1] = 59.336f;
  firing_offset_[1][12 - 1] = 59.336f;

  /* Firing Order 58 */
  firing_offset_[0][111 - 1] = 60.792f;
  firing_offset_[1][77 - 1] = 60.792f;

  /* Firing Order 59 */
  firing_offset_[0][77 - 1] = 61.648f;
  firing_offset_[1][111 - 1] = 61.648f;

  /* Firing Order 60 */
  firing_offset_[0][47 - 1] = 62.504f;
  firing_offset_[1][13 - 1] = 62.504f;

  /* Firing Order 61-70 @ p. 63 User Manual */
  /* Firing Order 61 */
  firing_offset_[0][114 - 1] = 63.96f;
  firing_offset_[1][84 - 1] = 63.96f;

  /* Firing Order 62 */
  firing_offset_[0][84 - 1] = 64.816f;
  firing_offset_[1][114 - 1] = 64.816f;

  /* Firing Order 63 */
  firing_offset_[0][50 - 1] = 65.672f;
  firing_offset_[1][20 - 1] = 65.672f;

  /* Firing Order 64 */
  firing_offset_[0][119 - 1] = 67.128f;
  firing_offset_[1][85 - 1] = 67.128f;

  /* Firing Order 65 */
  firing_offset_[0][85 - 1] = 67.984f;
  firing_offset_[1][119 - 1] = 67.984f;

  /* Firing Order 66 */
  firing_offset_[0][55 - 1] = 68.84f;
  firing_offset_[1][21 - 1] = 68.84f;

  /* Firing Order 67 */
  firing_offset_[0][122 - 1] = 70.296f;
  firing_offset_[1][92 - 1] = 70.296f;

  /* Firing Order 68 */
  firing_offset_[0][92 - 1] = 71.152f;
  firing_offset_[1][122 - 1] = 71.152f;

  /* Firing Order 69 */
  firing_offset_[0][58 - 1] = 72.008f;
  firing_offset_[1][28 - 1] = 72.008f;

  /* Firing Order 70 */
  firing_offset_[0][127 - 1] = 73.464f;
  firing_offset_[1][93 - 1] = 73.464f;

  /* Firing Order 71-80 @ p. 63 User Manual */
  /* Firing Order 71 */
  firing_offset_[0][93 - 1] = 74.32f;
  firing_offset_[1][127 - 1] = 74.32f;

  /* Firing Order 72 */
  firing_offset_[0][63 - 1] = 75.176f;
  firing_offset_[1][29 - 1] = 75.176f;

  /* Firing Order 73 */
  firing_offset_[0][100 - 1] = 76.632f;
  firing_offset_[1][66 - 1] = 76.632f;

  /* Firing Order 74 */
  firing_offset_[0][66 - 1] = 77.488f;
  firing_offset_[1][100 - 1] = 77.488f;

  /* Firing Order 75 */
  firing_offset_[0][36 - 1] = 78.344f;
  firing_offset_[1][2 - 1] = 78.344f;

  /* Firing Order 76 */
  firing_offset_[0][101 - 1] = 79.8f;
  firing_offset_[1][71 - 1] = 79.8f;

  /* Firing Order 77 */
  firing_offset_[0][71 - 1] = 80.656f;
  firing_offset_[1][101 - 1] = 80.656f;

  /* Firing Order 78 */
  firing_offset_[0][37 - 1] = 81.512f;
  firing_offset_[1][7 - 1] = 81.512f;

  /* Firing Order 79 */
  firing_offset_[0][108 - 1] = 82.968f;
  firing_offset_[1][74 - 1] = 82.968f;

  /* Firing Order 80 */
  firing_offset_[0][74 - 1] = 83.824f;
  firing_offset_[1][108 - 1] = 83.824f;

  /* Firing Order 81-90 @ p. 64 User Manual */
  /* Firing Order 81 */
  firing_offset_[0][44 - 1] = 84.68f;
  firing_offset_[1][10 - 1] = 84.68f;

  /* Firing Order 82 */
  firing_offset_[0][109 - 1] = 86.136f;
  firing_offset_[1][79 - 1] = 86.136f;

  /* Firing Order 83 */
  firing_offset_[0][79 - 1] = 86.992f;
  firing_offset_[1][109 - 1] = 86.992f;

  /* Firing Order 84 */
  firing_offset_[0][45 - 1] = 87.848f;
  firing_offset_[1][15 - 1] = 87.848f;

  /* Firing Order 85 */
  firing_offset_[0][116 - 1] = 89.304f;
  firing_offset_[1][82 - 1] = 89.304f;

  /* Firing Order 86 */
  firing_offset_[0][82 - 1] = 90.16f;
  firing_offset_[1][116 - 1] = 90.16f;

  /* Firing Order 87 */
  firing_offset_[0][52 - 1] = 91.016f;
  firing_offset_[1][18 - 1] = 91.016f;

  /* Firing Order 88 */
  firing_offset_[0][117 - 1] = 92.472f;
  firing_offset_[1][87 - 1] = 92.472f;

  /* Firing Order 89 */
  firing_offset_[0][87 - 1] = 93.328f;
  firing_offset_[1][117 - 1] = 93.328f;

  /* Firing Order 90 */
  firing_offset_[0][53 - 1] = 94.184f;
  firing_offset_[1][23 - 1] = 94.184f;

  /* Firing Order 91-96 @ p. 64 User Manual */
  /* Firing Order 91 */
  firing_offset_[0][124 - 1] = 95.64f;
  firing_offset_[1][90 - 1] = 95.64f;

  /* Firing Order 92 */
  firing_offset_[0][90 - 1] = 96.496f;
  firing_offset_[1][124 - 1] = 96.496f;

  /* Firing Order 93 */
  firing_offset_[0][60 - 1] = 97.352f;
  firing_offset_[1][26 - 1] = 97.352f;

  /* Firing Order 94 */
  firing_offset_[0][125 - 1] = 98.808f;
  firing_offset_[1][95 - 1] = 98.808f;

  /* Firing Order 95 */
  firing_offset_[0][95 - 1] = 99.664f;
  firing_offset_[1][125 - 1] = 99.664f;

  /* Firing Order 96 */
  firing_offset_[0][61 - 1] = 100.52f;
  firing_offset_[1][31 - 1] = 100.52f;
}

}  // namespace pandar_qt128
}  // namespace pandar_pointcloud
