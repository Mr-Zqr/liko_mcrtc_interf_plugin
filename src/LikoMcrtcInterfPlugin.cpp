#include "LikoMcrtcInterfPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

LikoMcrtcInterfPlugin::~LikoMcrtcInterfPlugin() = default;

void LikoMcrtcInterfPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("LikoMcrtcInterfPlugin::init called with configuration:\n{}", config.dump(true, true));
  // ros::NodeHandle this_nh;
  // nh_ = &this_nh;
  liko_sub_ = nh_->subscribe<nav_msgs::Odometry>("/bitbot_se", 1000, &LikoMcrtcInterfPlugin::liko_callback, this);
  controller.controller().datastore().make<Eigen::Quaterniond>("orientation_torso", bitbot_orientation_torso);
  controller.controller().datastore().make<Eigen::Vector3d>("position_torso", bitbot_position_torso);
  controller.controller().datastore().make<Eigen::Vector3d>("velocity_torso", bitbot_velocity_torso);
  update_thread_ = std::thread([this]() {
  if(ros::ok())
  {
    ros::spin();  
  }
  });
}

void LikoMcrtcInterfPlugin::reset(mc_control::MCGlobalController & controller)
{
  if(update_thread_.joinable())
  {
    running_ = false;
    update_thread_.join();
  }
}

void LikoMcrtcInterfPlugin::before(mc_control::MCGlobalController & )
{
  // mc_rtc::log::info("LikoMcrtcInterfPlugin::before");

}

void LikoMcrtcInterfPlugin::after(mc_control::MCGlobalController & controller)
{
  // mc_rtc::log::info("LikoMcrtcInterfPlugin::after");
  std::unique_lock<std::mutex> lock(update_mutex_);
  controller.controller().datastore().assign("orientation_torso", bitbot_orientation_torso);
  controller.controller().datastore().assign("position_torso", bitbot_position_torso);
  controller.controller().datastore().assign("velocity_torso", bitbot_velocity_torso);
}

mc_control::GlobalPlugin::GlobalPluginConfiguration LikoMcrtcInterfPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = false;
  out.should_run_after = true;
  out.should_always_run = false;
  return out;
}

void LikoMcrtcInterfPlugin::liko_callback(const nav_msgs::Odometry::ConstPtr & msg)
{
  std::unique_lock<std::mutex> lock(update_mutex_);
  bitbot_position_torso = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  bitbot_velocity_torso = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  bitbot_orientation_torso = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}
} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("LikoMcrtcInterfPlugin", mc_plugin::LikoMcrtcInterfPlugin)