/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rtc/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace mc_plugin
{

struct LikoMcrtcInterfPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~LikoMcrtcInterfPlugin() override;

  
private:
  
  ros::Subscriber liko_sub_;
  void liko_callback(const nav_msgs::Odometry::ConstPtr & msg);

  // Running thread
  std::atomic<bool> running_{false};
  std::thread update_thread_;

  // Update mutex to safely copy between the update thread and the before method
  std::mutex update_mutex_;

  std::shared_ptr<ros::NodeHandle> nh_ = mc_rtc::ROSBridge::get_node_handle();
  Eigen::Vector3d bitbot_position_torso, bitbot_velocity_torso;
  Eigen::Quaterniond bitbot_orientation_torso;
  
};

} // namespace mc_plugin
