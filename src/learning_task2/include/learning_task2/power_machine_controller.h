#ifndef POWER_MACHINE_CONTROLLER_H
#define POWER_MACHINE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>

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
  // 根据需要在此添加成员变量（例如关节句柄、参数等）
};

}  // namespace power_machine_controller

#endif  // POWER_MACHINE_CONTROLLER_H
