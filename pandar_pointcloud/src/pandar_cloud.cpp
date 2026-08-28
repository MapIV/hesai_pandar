#include "pandar_pointcloud/pandar_cloud.hpp"
#include <pandar_msgs/msg/pandar_scan.hpp>
#include "pandar_pointcloud/calibration.hpp"
#include "pandar_pointcloud/decoder/pandar40_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_qt_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_qt128_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_xt32_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_xt16_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar64_decoder.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <thread>

namespace
{
const size_t TCP_RETRY_NUM = 5;
const double TCP_RETRY_WAIT_SEC = 0.1;

inline std::chrono::nanoseconds toChronoNanoSeconds(const double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(seconds));
}
}  // namespace

namespace pandar_pointcloud
{
PandarCloud::PandarCloud(const rclcpp::NodeOptions & options)
: Node("pandar_cloud_node", options)
{
  scan_phase_ = declare_parameter("scan_phase", 0.0);
  angle_range_ = declare_parameter("angle_range", std::vector<double>{0.0, 360.0});
  distance_range_ = declare_parameter("distance_range", std::vector<double>{0.1, 200.0});

  return_mode_ = declare_parameter("return_mode", "");
  dual_return_distance_threshold_ = declare_parameter("dual_return_distance_threshold", 0.1);
  calibration_path_ = declare_parameter("calibration", "");
  model_ = declare_parameter("model", "");
  device_ip_ = declare_parameter("device_ip","");

  tcp_client_ = std::make_shared<pandar_api::TCPClient>(device_ip_);
  if (!setupCalibration()) {
    RCLCPP_WARN(get_logger(), "Unable to load calibration data");
    return;
  }
  disable_rings_ = declare_parameter("disable_rings", std::vector<long>{});
  if(!disable_rings_.empty() && (model_ != "PandarXT-16" || model_ != "PandarQT")) {
    RCLCPP_WARN(get_logger(), "Disable Rings function is only available for the Pandar XT-16, QT");
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
      RCLCPP_WARN(get_logger(),"Invalid return mode, defaulting to strongest return mode"); 
      selected_return_mode = pandar40::Pandar40Decoder::ReturnMode::STRONGEST;
    }
    decoder_ = std::make_shared<pandar40::Pandar40Decoder>(*this, calibration_, scan_phase_,
                                                           angle_range_, distance_range_,
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
      RCLCPP_WARN(get_logger(),"Invalid return mode, defaulting to dual return mode"); 
      selected_return_mode = pandar_qt::PandarQTDecoder::ReturnMode::DUAL;
    }
    std::vector<long> disabled_rings;
    if (!disable_rings_.empty()) {
      std::string rings;
      for (const auto& ring: disable_rings_) {
        if (ring >= 0 ) {
          rings += std::to_string(ring) + " ";
          disabled_rings.emplace_back(ring);
        }
      }
      RCLCPP_INFO_STREAM(get_logger(), "Disabling: " << disabled_rings.size() << " ring(s).");
      RCLCPP_INFO_STREAM(get_logger(), "Disabling rings:" << rings);
      RCLCPP_INFO_STREAM(get_logger(), "Only valid ring values will be disabled.");
    }
    decoder_ = std::make_shared<pandar_qt::PandarQTDecoder>(*this, calibration_, scan_phase_,
                                                            angle_range_, distance_range_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode,
                                                            disabled_rings);
  }
  else if (model_ == "PandarQT128") {
    pandar_qt128::PandarQT128Decoder::ReturnMode selected_return_mode;
    if (return_mode_ == "First")
      selected_return_mode = pandar_qt128::PandarQT128Decoder::ReturnMode::FIRST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar_qt128::PandarQT128Decoder::ReturnMode::LAST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar_qt128::PandarQT128Decoder::ReturnMode::DUAL;
    else {
      RCLCPP_WARN(get_logger(),"Invalid return mode, defaulting to dual return mode");
      selected_return_mode = pandar_qt128::PandarQT128Decoder::ReturnMode::DUAL;
    }
    decoder_ = std::make_shared<pandar_qt128::PandarQT128Decoder>(*this, calibration_, scan_phase_,
                                                            distance_range_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode);
  }
  else if (model_ == "PandarXT-32") {
    pandar_xt32::PandarXT32Decoder::ReturnMode selected_return_mode;
    if (return_mode_ == "First")
      selected_return_mode = pandar_xt32::PandarXT32Decoder::ReturnMode::FIRST;
    else if (return_mode_ == "STRONGEST")
      selected_return_mode = pandar_xt32::PandarXT32Decoder::ReturnMode::STRONGEST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar_xt32::PandarXT32Decoder::ReturnMode::LAST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar_xt32::PandarXT32Decoder::ReturnMode::DUAL;
    else {
      RCLCPP_WARN(get_logger(),"Invalid return mode, defaulting to dual return mode"); 
      selected_return_mode = pandar_xt32::PandarXT32Decoder::ReturnMode::DUAL;
    }
    decoder_ = std::make_shared<pandar_xt32::PandarXT32Decoder>(*this, calibration_, scan_phase_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode);
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
      RCLCPP_WARN(get_logger(),"Invalid return mode, defaulting to dual return mode");
      selected_return_mode = pandar64::Pandar64Decoder::ReturnMode::DUAL;
    }
    decoder_ = std::make_shared<pandar64::Pandar64Decoder>(*this, calibration_, scan_phase_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode);
  }
  else if (model_ == "PandarXT-16") {
    pandar_xt16::PandarXT16Decoder::ReturnMode selected_return_mode;
    if (return_mode_ == "First")
      selected_return_mode = pandar_xt16::PandarXT16Decoder::ReturnMode::FIRST;
    else if (return_mode_ == "STRONGEST")
      selected_return_mode = pandar_xt16::PandarXT16Decoder::ReturnMode::STRONGEST;
    else if (return_mode_ == "Last")
      selected_return_mode = pandar_xt16::PandarXT16Decoder::ReturnMode::LAST;
    else if (return_mode_ == "Dual")
      selected_return_mode = pandar_xt16::PandarXT16Decoder::ReturnMode::DUAL;
    else {
      RCLCPP_WARN(get_logger(),"Invalid return mode, defaulting to dual return mode");
      selected_return_mode = pandar_xt16::PandarXT16Decoder::ReturnMode::DUAL;
    }
    std::vector<long> disabled_rings;
    if (!disable_rings_.empty()) {
      std::string rings;
      for (const auto& ring: disable_rings_) {
        if (ring >= 0 ) {
          rings += std::to_string(ring) + " ";
          disabled_rings.emplace_back(ring);
        }
      }
      RCLCPP_INFO_STREAM(get_logger(), "Disabling: " << disabled_rings.size() << " ring(s).");
      RCLCPP_INFO_STREAM(get_logger(), "Disabling rings:" << rings);
      RCLCPP_INFO_STREAM(get_logger(), "Only valid ring values will be disabled.");
    }
    decoder_ = std::make_shared<pandar_xt16::PandarXT16Decoder>(*this, calibration_, scan_phase_,
                                                            dual_return_distance_threshold_,
                                                            selected_return_mode, disabled_rings);
  }
  else {
    // TODO : Add other models
    RCLCPP_WARN(get_logger(), "Invalid model name");
    return;
  }

  pandar_packet_sub_ =
    create_subscription<pandar_msgs::msg::PandarScan>(
    "pandar_packets", rclcpp::SensorDataQoS(),
    std::bind(&PandarCloud::onProcessScan, this, std::placeholders::_1));

  pandar_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", rclcpp::SensorDataQoS());
  pandar_points_ex_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points_ex", rclcpp::SensorDataQoS());
}

PandarCloud::~PandarCloud()
{
}

bool PandarCloud::setupCalibration()
{
  if (!calibration_path_.empty() && calibration_.loadFile(calibration_path_) == 0) {
    return true;
  }
  if (tcp_client_) {
    std::string content("");
    for (size_t i = 0; i < TCP_RETRY_NUM; ++i) {
      auto ret = tcp_client_->getLidarCalibration(content);
      if (ret == pandar_api::TCPClient::ReturnCode::SUCCESS) {
        break;
      }
      rclcpp::Rate(1.0/TCP_RETRY_WAIT_SEC).sleep();
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

void PandarCloud::onProcessScan(const pandar_msgs::msg::PandarScan::SharedPtr scan_msg)
{
  PointcloudXYZIRADT pointcloud;
  pandar_msgs::msg::PandarPacket pkt;

  for (auto& packet : scan_msg->packets) {
    decoder_->unpack(packet);
    if (decoder_->hasScanned()) {
      pointcloud = decoder_->getPointcloud();
      if (pointcloud->points.size() > 0) {
        double first_point_timestamp = pointcloud->points.front().time_stamp;
        pointcloud->header.frame_id = scan_msg->header.frame_id;
        if (pandar_points_pub_->get_subscription_count() > 0) {
          const auto pointcloud_raw = convertPointcloud(pointcloud);
          auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
          pcl::toROSMsg(*pointcloud_raw, *ros_pc_msg_ptr);
          ros_pc_msg_ptr->header.stamp = rclcpp::Time(toChronoNanoSeconds(first_point_timestamp).count());
          pandar_points_pub_->publish(std::move(ros_pc_msg_ptr));
        }
        {
          auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
          pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
          ros_pc_msg_ptr->header.stamp = rclcpp::Time(toChronoNanoSeconds(first_point_timestamp).count());
          pandar_points_ex_pub_->publish(std::move(ros_pc_msg_ptr));
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
