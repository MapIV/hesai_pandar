#include "pandar_pointcloud/pandar_cloud.hpp"
#include <pandar_msgs/PandarScan.h>
#include "pandar_pointcloud/calibration.hpp"
#include "pandar_pointcloud/decoder/pandar40_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_qt_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_xt_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_xtm_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar64_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_qt128_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_128_e4x_decoder.hpp"

#include <chrono>
#include <thread>
#include <limits>

namespace
{
const size_t TCP_RETRY_NUM = 5;
const double TCP_RETRY_WAIT_SEC = 0.1;
}  // namespace

namespace pandar_pointcloud
{
PandarCloud::PandarCloud(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  private_nh.getParam("scan_phase", scan_phase_);
  private_nh.getParam("return_mode", return_mode_);
  private_nh.getParam("dual_return_distance_threshold", dual_return_distance_threshold_);
  private_nh.getParam("calibration", calibration_path_);
  private_nh.getParam("model", model_);
  private_nh.getParam("device_ip", device_ip_);

  tcp_client_ = std::make_shared<pandar_api::TCPClient>(device_ip_);
  if (!setupCalibration()) {
    ROS_ERROR("Unable to load calibration data");
    return;
  }
  if (model_ == "Pandar40P" || model_ == "Pandar40M") {
    pandar40::Pandar40Decoder::ReturnMode selected_return_mode;
    if (return_mode_ == "Strongest")
      selected_return_mode = pandar40::Pandar40Decoder::ReturnMode::STRONGEST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar40::Pandar40Decoder::ReturnMode::LAST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar40::Pandar40Decoder::ReturnMode::DUAL;
    else {
      ROS_ERROR("Invalid return mode, defaulting to strongest return mode");
      selected_return_mode = pandar40::Pandar40Decoder::ReturnMode::STRONGEST;
    }
    decoder_ = std::make_shared<pandar40::Pandar40Decoder>(calibration_, scan_phase_,
                                                           dual_return_distance_threshold_,
                                                           selected_return_mode);
  }
  else if (model_ == "PandarQT") {
    pandar_qt::PandarQTDecoder::ReturnMode selected_return_mode;
    if (return_mode_ == "First")
      selected_return_mode = pandar_qt::PandarQTDecoder::ReturnMode::FIRST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar_qt::PandarQTDecoder::ReturnMode::LAST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar_qt::PandarQTDecoder::ReturnMode::DUAL;
    else {
      ROS_ERROR("Invalid return mode, defaulting to dual return mode");
      selected_return_mode = pandar_qt::PandarQTDecoder::ReturnMode::DUAL;
    }
    decoder_ = std::make_shared<pandar_qt::PandarQTDecoder>(calibration_, scan_phase_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode);
  }
  else if (model_ == "PandarXT-32") {
    pandar_xt::PandarXTDecoder::ReturnMode selected_return_mode;
    if (return_mode_ == "First")
      selected_return_mode = pandar_xt::PandarXTDecoder::ReturnMode::FIRST;
    else if (return_mode_ == "STRONGEST")
      selected_return_mode = pandar_xt::PandarXTDecoder::ReturnMode::STRONGEST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar_xt::PandarXTDecoder::ReturnMode::LAST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar_xt::PandarXTDecoder::ReturnMode::DUAL;
    else {
      ROS_ERROR("Invalid return mode, defaulting to dual return mode");
      selected_return_mode = pandar_xt::PandarXTDecoder::ReturnMode::DUAL;
    }
    decoder_ = std::make_shared<pandar_xt::PandarXTDecoder>(calibration_, scan_phase_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode);
  }
  else if (model_ == "PandarXTM") {
    pandar_xtm::PandarXTMDecoder::ReturnMode selected_return_mode;
    if (return_mode_ == "First")
      selected_return_mode = pandar_xtm::PandarXTMDecoder::ReturnMode::FIRST;
    else if (return_mode_ == "Strongest")
      selected_return_mode = pandar_xtm::PandarXTMDecoder::ReturnMode::STRONGEST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar_xtm::PandarXTMDecoder::ReturnMode::LAST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar_xtm::PandarXTMDecoder::ReturnMode::DUAL;
    else if (return_mode_ == "Triple") {
      //selected_return_mode = pandar_xtm::PandarXTMDecoder::ReturnMode::TRIPLE;
      ROS_ERROR("Triple return mode not implemented, defaulting to dual return mode");
      selected_return_mode = pandar_xtm::PandarXTMDecoder::ReturnMode::DUAL;
    }
    else {
      ROS_ERROR("Invalid return mode, defaulting to dual return mode");
      selected_return_mode = pandar_xtm::PandarXTMDecoder::ReturnMode::DUAL;
    }
    ROS_INFO_STREAM("XTM Decoder");
    decoder_ = std::make_shared<pandar_xtm::PandarXTMDecoder>(calibration_, scan_phase_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode);
    ROS_INFO_STREAM("XTM Decoder OK");
  }
  else if (model_ == "Pandar64") {
    pandar64::Pandar64Decoder::ReturnMode selected_return_mode;
    if (return_mode_ == "First")
      selected_return_mode = pandar64::Pandar64Decoder::ReturnMode::STRONGEST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar64::Pandar64Decoder::ReturnMode::LAST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar64::Pandar64Decoder::ReturnMode::DUAL;
    else {
      ROS_ERROR("Invalid return mode, defaulting to dual return mode");
      selected_return_mode = pandar64::Pandar64Decoder::ReturnMode::DUAL;
    }
    decoder_ = std::make_shared<pandar64::Pandar64Decoder>(calibration_, scan_phase_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode);
  }
  else if (model_ == "PandarQT128") {
    std::cout << "Constructor" << std::endl;
    pandar_qt128::PandarQT128Decoder::ReturnMode selected_return_mode;
    if (return_mode_ == "First")
      selected_return_mode = pandar_qt128::PandarQT128Decoder::ReturnMode::FIRST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar_qt128::PandarQT128Decoder::ReturnMode::LAST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar_qt128::PandarQT128Decoder::ReturnMode::DUAL;
    else {
      ROS_ERROR("Invalid return mode, defaulting to dual return mode");
      selected_return_mode = pandar_qt128::PandarQT128Decoder::ReturnMode::DUAL;
    }
    decoder_ = std::make_shared<pandar_qt128::PandarQT128Decoder>(calibration_, scan_phase_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode);
  }
  else if (model_ == "Pandar128E4X") {
    pandar_128_e4x::Pandar128E4XDecoder::ReturnMode selected_return_mode;
    if (return_mode_ == "First")
      selected_return_mode = pandar_128_e4x::Pandar128E4XDecoder::ReturnMode::FIRST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar_128_e4x::Pandar128E4XDecoder::ReturnMode::LAST;
    else if (return_mode_ == "Strongest")
      selected_return_mode = pandar_128_e4x::Pandar128E4XDecoder::ReturnMode::STRONGEST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar_128_e4x::Pandar128E4XDecoder::ReturnMode::DUAL;
    else {
      ROS_ERROR("Invalid return mode, defaulting to dual return mode");
      selected_return_mode = pandar_128_e4x::Pandar128E4XDecoder::ReturnMode::DUAL;
    }
    decoder_ = std::make_shared<pandar_128_e4x::Pandar128E4XDecoder>(calibration_, scan_phase_,
                                                                  dual_return_distance_threshold_,
                                                                  selected_return_mode);
  }
  else {
    // TODO : Add other models
    ROS_ERROR("Invalid model name");
    return;
  }

  pandar_packet_sub_ =
      node.subscribe("pandar_packets", 10, &PandarCloud::onProcessScan, this, ros::TransportHints().tcpNoDelay(true));
  pandar_points_pub_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);
  pandar_points_ex_pub_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points_ex", 10);
  ROS_INFO_STREAM("Ready");
}

PandarCloud::~PandarCloud()
{
}

bool PandarCloud::setupCalibration()
{
  if (!calibration_path_.empty() && calibration_.loadFile(calibration_path_) == 0) {
    ROS_INFO_STREAM("Loaded Calibration from:" << calibration_path_);
    return true;
  }
  ROS_INFO_STREAM("Loading Calibration file from device...");
  if (tcp_client_) {
    std::string content("");
    for (size_t i = 0; i < TCP_RETRY_NUM; ++i) {
      auto ret = tcp_client_->getLidarCalibration(content);
      if (ret == pandar_api::TCPClient::ReturnCode::SUCCESS) {
        break;
      }
      ros::Duration(TCP_RETRY_WAIT_SEC).sleep();
    }
    if (!content.empty()) {
      calibration_.loadContent(content);
      if (!calibration_path_.empty()) {
        calibration_.saveFile(calibration_path_);
      }
      return true;
    }
    else {
      return false;
    }
  }
  return false;
}

void PandarCloud::onProcessScan(const pandar_msgs::PandarScan::ConstPtr& scan_msg)
{
  PointcloudXYZIRADT pointcloud;
  pandar_msgs::PandarPacket pkt;

  for (auto& packet : scan_msg->packets) {
    decoder_->unpack(packet);
    if (decoder_->hasScanned()) {
      pointcloud = decoder_->getPointcloud();
      if (pointcloud->points.size() > 0 
          && (pointcloud->points[0].time_stamp < PandarCloud::MAX_ROS_TIME)
        ) {
        // pointcloud->header.stamp = pcl_conversions::toPCL(ros::Time(pointcloud->points[0].time_stamp));
        pointcloud->header.stamp = pcl_conversions::toPCL(ros::Time(decoder_->getTimestamp()));
        pointcloud->header.frame_id = scan_msg->header.frame_id;
        pointcloud->height = 1;

        pandar_points_ex_pub_.publish(pointcloud);
        if (pandar_points_pub_.getNumSubscribers() > 0) {
          pandar_points_pub_.publish(convertPointcloud(pointcloud));
        }
      }
    }
  }
}

pcl::PointCloud<PointXYZIR>::Ptr
PandarCloud::convertPointcloud(const pcl::PointCloud<PointXYZIRADT>::ConstPtr& input_pointcloud)
{
  pcl::PointCloud<PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<PointXYZIR>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  PointXYZIR point;
  for (const auto& p : input_pointcloud->points) {
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point.ring = p.ring;
    output_pointcloud->points.push_back(point);
  }

  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}
}  // namespace pandar_pointcloud
