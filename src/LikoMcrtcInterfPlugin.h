/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rtc/ros.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "low_pass_filter.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>

namespace mc_plugin
{

struct LikoMcrtcInterfPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~LikoMcrtcInterfPlugin() override;

private:
  ros::Subscriber liko_sub_;
  void liko_callback(const sensor_msgs::Imu::ConstPtr & msg);

  // Running thread
  std::atomic<bool> running_{false};
  std::thread update_thread_;

  // Update mutex to safely copy between the update thread and the before method
  std::mutex update_mutex_;

  std::shared_ptr<ros::NodeHandle> nh_ = mc_rtc::ROSBridge::get_node_handle();
  Eigen::Vector3d bitbot_position_torso_, bitbot_position_torso_smoothed_, bitbot_velocity_torso_,
      bitbot_orientation_torso_;
  Eigen::Quaterniond bitbot_orientationq_torso_;

  bool disp_liko_coordinate_ = false;
  bool run_ = false;

  LowPassFilter LPFx;
  LowPassFilter LPFy;
  LowPassFilter LPFz;

  void addNumber(double num);

  double getAverage();
  double z_position_bias_;
  std::deque<double> z_position_temp_;

  bool init_imu_pose_;
  double sum;
};

} // namespace mc_plugin
