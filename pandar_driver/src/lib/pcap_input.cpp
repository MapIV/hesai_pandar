#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <map>
#include <cstring>
#include "pandar_driver/pcap_input.h"

using namespace pandar_driver;

namespace
{
const uint8_t PKT_HEADER_SIZE = 42;
const size_t LIMIT_PACKET_NUM = 100;
}  // namespace

PcapInput::PcapInput(uint16_t port, uint16_t gps_port, std::string path, std::string model)
  : pcap_(nullptr), last_pkt_ts_(0)
{
  initTimeIndexMap();
  pcap_path_ = path;
  frame_id_ = model;
  std::map<std::string, std::pair<int, int>>::iterator iter = time_index_map_.find(frame_id_);
  if (iter != time_index_map_.end()) {
    ts_index_ = iter->second.first;
    utc_index_ = iter->second.second;
  }
  else {
    ts_index_ = 0;
    utc_index_ = 0;
  }

  if (pcap_path_.empty()) {
    return;
  }

  pcap_ = pcap_open_offline(pcap_path_.c_str(), errbuf_);

  if (pcap_ == nullptr) {
    printf("open pcap file %s fail\n", pcap_path_.c_str());
    return;
  }

  std::stringstream filter;
  filter << "udp dst port " << port;
  if (pcap_compile(pcap_, &pcap_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN) == -1) {
    printf("compile pcap file fail\n");
    return;
  }

  if (pcap_setfilter(pcap_, &pcap_filter_) == -1) {
    printf("pcap set filter fail\n");
    return;
  }
}

PcapInput::~PcapInput()
{
  pcap_close(pcap_);
}

void PcapInput::initTimeIndexMap()
{
  time_index_map_.insert(std::pair<std::string, std::pair<int, int>>("Pandar40P", std::pair<int, int>(1250, 1256)));
  time_index_map_.insert(std::pair<std::string, std::pair<int, int>>("Pandar40M", std::pair<int, int>(1250, 1256)));
  time_index_map_.insert(std::pair<std::string, std::pair<int, int>>("Pandar64", std::pair<int, int>(1182, 1188)));
  time_index_map_.insert(std::pair<std::string, std::pair<int, int>>("PandarQT", std::pair<int, int>(1056, 1062)));
  time_index_map_.insert(std::pair<std::string, std::pair<int, int>>("Pandar20A", std::pair<int, int>(1258, 1264)));
  time_index_map_.insert(std::pair<std::string, std::pair<int, int>>("Pandar20B", std::pair<int, int>(1258, 1264)));
  time_index_map_.insert(std::pair<std::string, std::pair<int, int>>("PandarXT-32", std::pair<int, int>(1071, 1065)));
  time_index_map_.insert(std::pair<std::string, std::pair<int, int>>("PandarXT-16", std::pair<int, int>(559, 553)));
}

PcapInput::PacketType PcapInput::getPacket(pandar_msgs::PandarPacket* pandar_pkt)
{
  pcap_pkthdr* pkt_header;
  const uint8_t* pkt_data;
  int64_t current_time;
  int64_t pkt_ts = 0;

  while (true) {
    if (pcap_next_ex(pcap_, &pkt_header, &pkt_data) >= 0) {
      if (pcap_offline_filter(&pcap_filter_, pkt_header, pkt_data) == 0) {
        continue;
      }
      const uint8_t* packet = pkt_data + PKT_HEADER_SIZE;
      int pkt_size = pkt_header->len - PKT_HEADER_SIZE;
      pandar_pkt->stamp = ros::Time::now();
      pandar_pkt->size = pkt_size;
      std::memcpy(&pandar_pkt->data[0], packet, pkt_size);

      packet_count_++;
      // Sleep
      if (packet_count_ >= LIMIT_PACKET_NUM && utc_index_ != 0) {
        packet_count_ = 0;

        struct tm t;
        t.tm_year = packet[utc_index_];
        t.tm_mon = packet[utc_index_ + 1] - 1;
        t.tm_mday = packet[utc_index_ + 2];
        t.tm_hour = packet[utc_index_ + 3];
        t.tm_min = packet[utc_index_ + 4];
        t.tm_sec = packet[utc_index_ + 5];
        t.tm_isdst = 0;

        pkt_ts =
            mktime(&t) * 1000000 + ((packet[ts_index_] & 0xff) | (packet[ts_index_ + 1] & 0xff) << 8 |
                                    ((packet[ts_index_ + 2] & 0xff) << 16) | ((packet[ts_index_ + 3] & 0xff) << 24));
        struct timeval sys_time;
        gettimeofday(&sys_time, nullptr);
        current_time = sys_time.tv_sec * 1000000 + sys_time.tv_usec;

        if (0 == last_pkt_ts_) {
          last_pkt_ts_ = pkt_ts;
          last_time_ = current_time;
        }
        else {
          int64_t sleep_time = (pkt_ts - last_pkt_ts_) - (current_time - last_time_);
          if (sleep_time > 0) {
            struct timeval waitTime;
            waitTime.tv_sec = sleep_time / 1000000;
            waitTime.tv_usec = sleep_time % 1000000;

            int err;

            do {
              err = select(0, nullptr, nullptr, nullptr, &waitTime);
            } while (err < 0 && errno != EINTR);
          }

          last_pkt_ts_ = pkt_ts;
          last_time_ = current_time;
          last_time_ += sleep_time;
        }
      }
      return PacketType::LIDAR;
    }
    else {
      return PacketType::ERROR;
    }
  }
}

PcapInput::PacketType PcapInput::getPacket(pandar_msgs::PandarPacket2* pandar_pkt)
{
  pcap_pkthdr* pkt_header;
  const uint8_t* pkt_data;
  int64_t current_time;
  int64_t pkt_ts = 0;

  while (true) {
    if (pcap_next_ex(pcap_, &pkt_header, &pkt_data) >= 0) {
      if (pcap_offline_filter(&pcap_filter_, pkt_header, pkt_data) == 0) {
        continue;
      }
      const uint8_t* packet = pkt_data + PKT_HEADER_SIZE;
      int pkt_size = pkt_header->len - PKT_HEADER_SIZE;
      pandar_pkt->stamp = ros::Time::now();
      pandar_pkt->size = pkt_size;
      pandar_pkt->data.resize(pkt_size);
      std::memcpy(&pandar_pkt->data[0], packet, pkt_size);

      packet_count_++;
      // Sleep
      if (packet_count_ >= LIMIT_PACKET_NUM && utc_index_ != 0) {
        packet_count_ = 0;

        struct tm t;
        t.tm_year = packet[utc_index_];
        t.tm_mon = packet[utc_index_ + 1] - 1;
        t.tm_mday = packet[utc_index_ + 2];
        t.tm_hour = packet[utc_index_ + 3];
        t.tm_min = packet[utc_index_ + 4];
        t.tm_sec = packet[utc_index_ + 5];
        t.tm_isdst = 0;

        pkt_ts =
            mktime(&t) * 1000000 + ((packet[ts_index_] & 0xff) | (packet[ts_index_ + 1] & 0xff) << 8 |
                                    ((packet[ts_index_ + 2] & 0xff) << 16) | ((packet[ts_index_ + 3] & 0xff) << 24));
        struct timeval sys_time;
        gettimeofday(&sys_time, nullptr);
        current_time = sys_time.tv_sec * 1000000 + sys_time.tv_usec;

        if (0 == last_pkt_ts_) {
          last_pkt_ts_ = pkt_ts;
          last_time_ = current_time;
        }
        else {
          int64_t sleep_time = (pkt_ts - last_pkt_ts_) - (current_time - last_time_);
          if (sleep_time > 0) {
            struct timeval waitTime;
            waitTime.tv_sec = sleep_time / 1000000;
            waitTime.tv_usec = sleep_time % 1000000;

            int err;

            do {
              err = select(0, nullptr, nullptr, nullptr, &waitTime);
            } while (err < 0 && errno != EINTR);
          }

          last_pkt_ts_ = pkt_ts;
          last_time_ = current_time;
          last_time_ += sleep_time;
        }
      }
      return PacketType::LIDAR;
    }
    else {
      return PacketType::ERROR;
    }
  }
}
