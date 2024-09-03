#pragma once

#include <ros/ros.h>
#include <pandar_msgs/PandarScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pandar_api/tcp_client.hpp>
#include "pandar_pointcloud/calibration.hpp"
#include "pandar_pointcloud/decoder/packet_decoder.hpp"
// #include "pandar_pointcloud/tcp_command_client.hpp"


#include <string>

namespace pandar_pointcloud
{
class PandarCloud
{
public:
  PandarCloud(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~PandarCloud();
  const double MAX_ROS_TIME = 4294967296.;//edge case when ROS Time exceeds 2^32
private:
  bool setupCalibration();
  void onProcessScan(const pandar_msgs::PandarScan::ConstPtr& msg);
  pcl::PointCloud<PointXYZIR>::Ptr convertPointcloud(const pcl::PointCloud<PointXYZIRADT>::ConstPtr& input_pointcloud);

  std::string model_;
  std::string return_mode_;
  std::string device_ip_;
  std::string calibration_path_;
  double dual_return_distance_threshold_;
  double scan_phase_;

  ros::Subscriber pandar_packet_sub_;
  ros::Publisher pandar_points_pub_;
  ros::Publisher pandar_points_ex_pub_;

  std::shared_ptr<PacketDecoder> decoder_;
  std::shared_ptr<pandar_api::TCPClient> tcp_client_;
  Calibration calibration_;
};

}  // namespace pandar_pointcloud