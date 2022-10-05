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
  void unpack(const pandar_msgs::PandarPacket& raw_packet) override;
  PointXYZIRADT build_point(int block_id, int unit_id, int seq_id, uint8_t return_type);
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;

private:
  bool parsePacket(const pandar_msgs::PandarPacket& raw_packet);
  PointcloudXYZIRADT convert(const int block_id);
  PointcloudXYZIRADT convert_dual(const int block_id);

  void initFiringOffset();

  std::array<std::array<float, UNIT_NUM>, BLOCK_NUM> firing_offset_;
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

  std::array<float, UNIT_NUM> elev_angle_ = {
  -52.6267f, -51.0280f, -49.5149f, -48.0739f, -46.6946f, -45.3688f, -44.0897f, -42.8519f, -41.6506f, -40.4822f,
  -39.3432f, -38.2308f, -37.1426f, -36.0763f, -35.0301f, -34.0024f, -32.9915f, -31.9963f, -31.0154f, -30.0478f,
  -29.0926f, -28.1488f, -27.2156f, -26.2924f, -25.3783f, -24.4728f, -23.5754f, -22.6854f, -21.8024f, -20.9259f,
  -20.0555f, -19.1908f, -18.3313f, -17.4769f, -16.6270f, -15.7814f, -14.9399f, -14.1020f, -13.2676f, -12.4364f,
  -11.6081f, -10.7826f, -9.9595f,  -9.1386f,  -8.3199f,  -7.5030f,  -6.6878f,  -5.8739f,  -5.0613f,  -4.2499f,
  -3.4394f,  -2.6296f,  -1.8203f,  -1.0115f,  -0.2028f,  0.6057f,   1.4144f,   2.2234f,   3.0329f,   3.8431f,
  4.6540f,   5.4660f,   6.2791f,   7.0936f,   7.9097f,   8.7275f,   9.5473f,   10.3692f,  11.1935f,  12.0204f,
  12.8501f,  13.6829f,  14.5190f,  15.3586f,  16.2022f,  17.0498f,  17.9020f,  18.7589f,  19.6209f,  20.4884f,
  21.3618f,  22.2414f,  23.1279f,  24.0215f,  24.9229f,  25.8326f,  26.7511f,  27.6792f,  28.6176f,  29.5670f,
  30.5282f,  31.5023f,  32.4902f,  33.4931f,  34.5122f,  35.5489f,  19.1908f,  20.0555f,  20.9259f,  21.8024f,
  22.6854f,  23.5754f,  24.4728f,  25.3783f,  26.2924f,  27.2156f,  28.1488f,  29.0926f,  30.0478f,  31.0154f,
  31.9963f,  32.9915f,  34.0024f,  35.0301f,  36.0763f,  37.1426f,  38.2308f,  39.3432f,  40.4822f,  41.6506f,
  42.8519f,  44.0897f,  45.3688f,  46.6946f,  48.0739f,  49.5149f,  51.0280f,  52.6267f
};

std::array<float, UNIT_NUM> azimuth_offset_ = {
  10.6267f, 9.0280f,  9.5149f,  9.0739f,  8.6946f,  8.3688f,  8.0897f,  8.8519f,  8.6506f,  7.4822f,  7.3432f,
  7.2308f,  7.1426f,  7.0763f,  7.0301f,  7.0024f,  7.9915f,  7.9963f,  6.0154f,  6.0478f,  6.0926f,  6.1488f,
  6.2156f,  6.2924f,  6.3783f,  6.4728f,  6.5754f,  6.6854f,  6.8024f,  6.9259f,  6.0555f,  6.1908f,  -6.3313f,
  -6.4769f, -6.6270f, -6.7814f, -6.9399f, -6.1020f, -6.2676f, -6.4364f, -6.6081f, -5.7826f, -5.9595f, -5.1386f,
  -5.3199f, -5.5030f, -5.6878f, -5.8739f, -5.0613f, -5.2499f, -5.4394f, -5.6296f, -5.8203f, -5.0115f, -5.2028f,
  -5.6057f, -5.4144f, -5.2234f, -5.0329f, -5.8431f, -5.6540f, -5.4660f, -5.2791f, -5.0936f, 5.9097f,  5.7275f,
  5.5473f,  5.3692f,  6.1935f,  6.0204f,  6.8501f,  6.6829f,  6.5190f,  6.3586f,  6.2022f,  6.0498f,  6.9020f,
  6.7589f,  6.6209f,  6.4884f,  6.3618f,  6.2414f,  6.1279f,  6.0215f,  6.9229f,  6.8326f,  6.7511f,  6.6792f,
  6.6176f,  6.5670f,  6.5282f,  6.5023f,  7.4902f,  7.4931f,  7.5122f,  7.5489f,  -6.1908f, -6.0555f, -6.9259f,
  -6.8024f, -6.6854f, -6.5754f, -6.4728f, -6.3783f, -6.2924f, -6.2156f, -6.1488f, -6.0926f, -6.0478f, -6.0154f,
  -7.9963f, -7.9915f, -7.0024f, -7.0301f, -7.0763f, -7.1426f, -7.2308f, -7.3432f, -7.4822f, -8.6506f, -8.8519f,
  -8.0897f, -8.3688f, -8.6946f, -9.0739f, -9.5149f, -9.0280f, -10.6267f
};
};

}  // namespace pandar_qt128
}  // namespace pandar_pointcloud
