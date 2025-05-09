/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "pandar_monitor/pandar_monitor.hpp"
#include <boost/algorithm/string/join.hpp>

PandarMonitor::PandarMonitor()
{
  pnh_.param<double>("timeout", timeout_, 1.0);
  pnh_.param<std::string>("ip_address", ip_address_, "192.168.1.201");
  pnh_.param<float>("temp_cold_warn", temp_cold_warn_, -5.0);
  pnh_.param<float>("temp_cold_error", temp_cold_error_, -10.0);
  pnh_.param<float>("temp_hot_warn", temp_hot_warn_, 75.0);
  pnh_.param<float>("temp_hot_error", temp_hot_error_, 80.0);
  pnh_.param<float>("rpm_ratio_warn", rpm_ratio_warn_, 0.80);
  pnh_.param<float>("rpm_ratio_error", rpm_ratio_error_, 0.70);

  updater_.add("pandar_connection", this, &PandarMonitor::checkConnection);
  updater_.add("pandar_temperature", this, &PandarMonitor::checkTemperature);
  updater_.add("pandar_ptp", this, &PandarMonitor::checkPTP);
  updater_.add("pandar_gpspps",this, &PandarMonitor::checkGPSPPS);
  updater_.add("pandar_gpsgprmc",this, &PandarMonitor::checkGPSGPRMC);

  client_ = std::make_unique<pandar_api::TCPClient>(ip_address_, static_cast<int>(timeout_ * 1000));

  for (MovingAverage& ave_temp: temp_list_) {
    ave_temp.setWindowSize(10);
  }

  updater_.setHardwareID("pandar");

  timer_ = pnh_.createTimer(ros::Rate(1.0), &PandarMonitor::onTimer, this);
}

void PandarMonitor::checkConnection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  pandar_api::InventoryInfo info;
  auto code = client_->getInventoryInfo(info);
  stat.add("return_code", (int)code); 
  if(code != pandar_api::TCPClient::ReturnCode::SUCCESS){
    stat.summary(DiagStatus::ERROR, "ERROR");
    return;
  }

  updater_.setHardwareIDf(
    "%s: %s", info.model.c_str(), info.sn.c_str());

  stat.summary(DiagStatus::OK, "OK");
}

void PandarMonitor::checkTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  pandar_api::LidarStatus status;
  auto code = client_->getLidarStatus(status);
  stat.add("return_code", (int)code); 
  if(code != pandar_api::TCPClient::ReturnCode::SUCCESS){
    stat.summary(DiagStatus::ERROR, "ERROR");
    return;
  }

  int error = DiagStatus::OK;
  int warn = DiagStatus::OK;

  for(size_t i = 0; i < 8; ++i){
    float raw_temp = static_cast<float>(status.temp[i]) / 100.0f;
    float temp = temp_list_[i].update(status.temp[i]); // unit: degC*100
    temp = temp / 100.0f;

    // Check board temperature
    auto pos = position_[i];
    if (raw_temp < temp_cold_error_) {
      error = DiagStatus::ERROR;
      stat.addf(position_[i], "%.2lf DegC [x]", temp);
      ROS_ERROR_STREAM(pos << ": " << raw_temp);

    } else if (raw_temp < temp_cold_warn_) {
      warn = DiagStatus::WARN;
      stat.addf(position_[i], "%.2lf DegC [!]", temp);
      ROS_WARN_STREAM(pos << ": " << raw_temp);

    } else if (raw_temp > temp_hot_error_) {
      error = DiagStatus::ERROR;
      stat.addf(position_[i], "%.2lf DegC [x]", temp);
      ROS_ERROR_STREAM(pos << ": " << raw_temp);

    } else if (raw_temp > temp_hot_warn_) {
      warn = DiagStatus::WARN;
      stat.addf(position_[i], "%.2lf DegC [!]", temp);
      ROS_WARN_STREAM(pos << ": " << raw_temp);

    } else {
      stat.addf(position_[i], "%.2lf DegC", temp);
    }
  }

  stat.add("threshold_cold_error", temp_cold_error_);
  stat.add("threshold_cold_warn",  temp_cold_warn_);
  stat.add("threshold_hot_warn",   temp_hot_warn_);
  stat.add("threshold_hot_error",  temp_hot_error_);

  std::string msg;
  if (error == DiagStatus::ERROR) {
    msg = "ERROR";
  } else if (warn == DiagStatus::WARN) {
    msg = "WARN";
  } else {
    msg = "OK";
  }

  stat.summary(std::max(error, warn), msg);
}

void PandarMonitor::checkPTP(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  pandar_api::LidarStatus status;
  auto code = client_->getLidarStatus(status);
  stat.add("return_code", (int)code); 
  if(code != pandar_api::TCPClient::ReturnCode::SUCCESS){
    stat.summary(DiagStatus::ERROR, "ERROR");
    return;
  }

  int level = DiagStatus::OK;
  if(status.ptp_clock_status == 0 || status.ptp_clock_status == 3){
    level = DiagStatus::WARN;
  }
  stat.summary(level, ptp_[status.ptp_clock_status]);
}

void PandarMonitor::onTimer(const ros::TimerEvent & event) { updater_.force_update(); }

void PandarMonitor::checkGPSPPS(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  /* get LiDAR status*/
  pandar_api::LidarStatus status;
  auto code = client_->getLidarStatus(status);
  stat.add("return_code", (int)code); 
  if (code == pandar_api::TCPClient::ReturnCode::SUCCESS)
  {
    if(status.gps_pps_lock == 0)
    {
      stat.summary(DiagStatus::ERROR, gps_message_[0]);
    }
    else if(status.gps_pps_lock == 1)
    {
      stat.summary(DiagStatus::OK, gps_message_[1]);
    }
    else
    {
      stat.summary(DiagStatus::WARN, gps_message_[2]);
    }
  }
}

void PandarMonitor::checkGPSGPRMC(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  /* get LiDAR status*/
  pandar_api::LidarStatus status;
  auto code = client_->getLidarStatus(status);
  stat.add("return_code", (int)code); 
  if (code == pandar_api::TCPClient::ReturnCode::SUCCESS)
  {
    if(status.gps_gprmc_status == 0)
    {
      stat.summary(DiagStatus::ERROR, gps_message_[0]);
    }
    else if(status.gps_gprmc_status == 1)
    {
      stat.summary(DiagStatus::OK, gps_message_[1]);
    }
    else
    {
      stat.summary(DiagStatus::WARN, gps_message_[2]);
    }
  }
}