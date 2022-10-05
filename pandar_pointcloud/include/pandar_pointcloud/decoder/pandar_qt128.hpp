#pragma once

/**
 * Pandar QT128
 */

#include <cstdint>

namespace pandar_pointcloud
{
namespace pandar_qt128
{

// Head
constexpr std::size_t HEAD_SIZE = 12;
constexpr std::size_t PRE_HEADER_SIZE = 6;
constexpr std::size_t HEADER_SIZE = 6;

// Body
constexpr std::size_t BLOCK_NUM = 2;
constexpr std::size_t BLOCK_HEADER_AZIMUTH = 2;
constexpr std::size_t UNIT_NUM = 128;
constexpr std::size_t UNIT_SIZE = 4;
constexpr std::size_t CRC_1_SIZE = 4;
constexpr std::size_t BLOCK_SIZE = UNIT_SIZE * UNIT_NUM + BLOCK_HEADER_AZIMUTH;
constexpr std::size_t BODY_SIZE = BLOCK_SIZE * BLOCK_NUM + CRC_1_SIZE;

// Functional Safety
constexpr std::size_t FUNCTIONAL_SAFETY_SIZE = 17;

// Tail
constexpr std::size_t RESERVED_1_SIZE = 5;
constexpr std::size_t MODE_FLAG_SIZE = 1;
constexpr std::size_t RESERVED_2_SIZE = 6;
constexpr std::size_t RETURN_MODE_SIZE = 1;
constexpr std::size_t MOTOR_SPEED_SIZE = 2;
constexpr std::size_t DATE_TIME_SIZE = 6;
constexpr std::size_t TIMESTAMP_SIZE = 4;
constexpr std::size_t FACTORY_INFO_SIZE = 1;
constexpr std::size_t UDP_SEQUENCE_SIZE = 4;
constexpr std::size_t CRC_3_SIZE = 4;
constexpr std::size_t PACKET_TAIL_SIZE = 34;

// Cyber Security
constexpr std::size_t CYBER_SECURITY_SIZE = 32;

// All
constexpr std::size_t PACKET_SIZE =
    HEAD_SIZE + BODY_SIZE + FUNCTIONAL_SAFETY_SIZE + PACKET_TAIL_SIZE + CYBER_SECURITY_SIZE;

constexpr uint32_t FIRST_RETURN = 0x33;
constexpr uint32_t LAST_RETURN = 0x38;
constexpr uint32_t DUAL_RETURN = 0x3B;

struct Header
{
  std::uint16_t u16Sob;          // Start of block 0xFFEE 2bytes
  std::uint8_t u8ProtocolMajor;  // Protocol Version Major 1byte
  std::uint8_t u8ProtocolMinor;  // Protocol Version Minor 1byte
  std::uint8_t u8LaserNum;       // Laser Num 1byte (0x80 = 128 channels)
  std::uint8_t u8BlockNum;       // Block Num 1byte (0x02 = 2 blocks per packe)
  std::uint8_t u8DistUnit;       // Distance unit 1byte (0x04 = 4mm)
  std::uint8_t u8EchoNum;        // Number of returns that each channel generates. 1byte
  std::uint8_t u8Flags;          // Flags 1byte
};

struct Unit
{
  double distance;
  std::uint16_t intensity;
};

struct Block
{
  std::uint16_t azimuth;
  Unit units[UNIT_NUM];
};

struct Packet
{
  Header header;
  Block blocks[BLOCK_NUM];
  std::uint32_t timestamp;  // ms
  std::uint32_t mode_flag;
  uint32_t usec;  // ms
  uint32_t return_mode;
  tm t;
};
}  // namespace pandar_qt128
}  // namespace pandar_pointcloud
