#include "LikoMcrtcInterfPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

LikoMcrtcInterfPlugin::~LikoMcrtcInterfPlugin()
{
  run_ = false;
}

void LikoMcrtcInterfPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("LikoMcrtcInterfPlugin::init called with configuration:\n{}", config.dump(true, true));
  liko_sub_ = nh_->subscribe<sensor_msgs::Imu>("/bitbot_se", 1, &LikoMcrtcInterfPlugin::liko_callback, this);
  controller.controller().datastore().make<Eigen::Vector3d>("orientation_torso", bitbot_orientation_torso_);
  controller.controller().datastore().make<Eigen::Vector3d>("position_torso", bitbot_position_torso_smoothed_);
  controller.controller().datastore().make<Eigen::Vector3d>("velocity_torso", bitbot_velocity_torso_);

  config("show_coordinate", disp_liko_coordinate_);

  if(disp_liko_coordinate_)
  {
    controller.controller().gui()->addElement({"LikoMcrtcInterfPlugin"}, mc_rtc::gui::Transform("Marker", [this]() {
        return sva::PTransformd{bitbot_orientationq_torso_, bitbot_position_torso_};
    }));    
  }

  LPFx.init(20, 0.001);
  LPFy.init(20, 0.001);
  LPFz.init(2, 0.001);

  run_ = true;

  update_thread_ = std::thread([this]() {
    ros::Rate rate(5000);
    while(ros::ok() && run_)
    {
      ros::spinOnce();  
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

void LikoMcrtcInterfPlugin::before(mc_control::MCGlobalController & controller)
{
  // mc_rtc::log::info("LikoMcrtcInterfPlugin::before");

}

void LikoMcrtcInterfPlugin::after(mc_control::MCGlobalController & controller)
{
  // mc_rtc::log::info("LikoMcrtcInterfPlugin::after");
  std::unique_lock<std::mutex> lock(update_mutex_);
  controller.controller().datastore().assign("orientation_torso", bitbot_orientation_torso_);
  controller.controller().datastore().assign("position_torso", bitbot_position_torso_smoothed_);
  controller.controller().datastore().assign("velocity_torso", bitbot_velocity_torso_);

  // 将bitbot_orientation_torso_转为四元数存到bitbot_orientationq_torso_'
  // 假设bitbot_orientation_torso_是一个Eigen::Vector3d对象
  Eigen::Vector3d euler_angles = bitbot_orientation_torso_;

  // 创建三个AngleAxis对象
  Eigen::AngleAxisd rollAngle(euler_angles[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(euler_angles[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(euler_angles[2], Eigen::Vector3d::UnitZ());

  // 合并旋转
  Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;

  // 存储四元数
  bitbot_orientationq_torso_ = quaternion;

}

mc_control::GlobalPlugin::GlobalPluginConfiguration LikoMcrtcInterfPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = false;
  out.should_run_after = true;
  out.should_always_run = false;
  return out;
}

void LikoMcrtcInterfPlugin::liko_callback(const sensor_msgs::Imu::ConstPtr & msg)
{
  std::unique_lock<std::mutex> lock(update_mutex_);
  bitbot_position_torso_ = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  bitbot_velocity_torso_ = Eigen::Vector3d(msg->angular_velocity.z, msg->angular_velocity.y, msg->angular_velocity.z);
  bitbot_orientation_torso_ = Eigen::Vector3d(msg->orientation.x, msg->orientation.y, msg->orientation.z);
  bitbot_position_torso_smoothed_[0] = LPFx.filter(bitbot_position_torso_[0]);
  bitbot_position_torso_smoothed_[1] = LPFy.filter(bitbot_position_torso_[1]);
  bitbot_position_torso_smoothed_[2] = LPFz.filter(bitbot_position_torso_[2]);
}
} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("LikoMcrtcInterfPlugin", mc_plugin::LikoMcrtcInterfPlugin)
