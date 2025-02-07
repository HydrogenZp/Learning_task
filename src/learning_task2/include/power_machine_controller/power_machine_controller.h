#ifndef POWER_MACHINE_CONTROLLER_H
#define POWER_MACHINE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <cmath>

namespace power_machine_controller
{

class PowerMachineController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  PowerMachineController();
  ~PowerMachineController() override;

  bool init(hardware_interface::EffortJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void commandCallback(const std_msgs::Float64::ConstPtr&msg);  // 命令回调函数
  void computeEfforts(const ros::Time& time, const ros::Duration& period);
  
  hardware_interface::JointHandle motor_joint_;  // 关节句柄
  ros::Subscriber sub_command_;  // 命令订阅器
  
  double command_;  // 存储最新的命令值
  
  control_toolbox::Pid pid_controller_;  // PID控制器
  
  // PID参数
  double p_gain_;
  double i_gain_;
  double d_gain_;

  double spd;
  double a,b,omega;

  double record_time_;
  double last_command_;
};

}  // namespace power_machine_controller

#endif  // POWER_MACHINE_CONTROLLER_H
