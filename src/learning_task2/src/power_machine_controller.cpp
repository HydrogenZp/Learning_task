#include "power_machine_controller/power_machine_controller.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <cmath>
#include <cstdlib>
#include <random>

namespace power_machine_controller {
    PowerMachineController::PowerMachineController()
        : command_(0.0),
          p_gain_(100.0), i_gain_(0.1), d_gain_(10.0),
          feedforward_enabled_(false) // 初始化前馈模式为禁用
    {
        // 在允许范围内随机初始化参数
        std::random_device rd;
        std::mt19937 gen(rd());

        // 为a生成0.780~1.045之间的随机数
        std::uniform_real_distribution<> a_dist(0.780, 1.045);
        a = a_dist(gen);

        // 为omega生成1.884~2.000之间的随机数
        std::uniform_real_distribution<> omega_dist(1.884, 2.000);
        omega = omega_dist(gen);

        // 根据公式计算b
        b = 2.090 - a;

        // 初始化spd为0
        spd = 0.0;

        record_time_ = 0;
        last_command_ = 0;
        last_log_time_ = 0; // 新增：记录上一次日志输出时间
    }

    PowerMachineController::~PowerMachineController() {
        // 析构函数：清理资源
    }

    void PowerMachineController::feedforwardModeCallback(const std_msgs::Float64::ConstPtr &msg) {
        if (msg->data != 0.0) {
            feedforward_enabled_ = true;
            ROS_INFO_STREAM("Feedforward mode enabled");
        } else {
            feedforward_enabled_ = false;
            ROS_INFO_STREAM("Feedforward mode disabled");
        }
    }

    bool PowerMachineController::init(hardware_interface::EffortJointInterface *hw,
                                      ros::NodeHandle &root_nh,
                                      ros::NodeHandle &controller_nh) {
        try {
            // 获取关节句柄
            motor_joint_ = hw->getHandle("bar_joint");

            // 加载PID参数
            controller_nh.getParam("pid/p", p_gain_);
            controller_nh.getParam("pid/i", i_gain_);
            controller_nh.getParam("pid/d", d_gain_);

            // 从yaml文件中读取前馈增益
            controller_nh.getParam("feedforward/k", k_gain_);
            // 初始化PID控制器
            pid_controller_.initPid(p_gain_, i_gain_, d_gain_, 100.0, -100.0);

            // 初始化命令订阅器
            sub_command_ = root_nh.subscribe<std_msgs::Float64>("command", 1,
                                                                &PowerMachineController::commandCallback, this);

            // 初始化前馈模式订阅者
            sub_feedforward_mode_ = root_nh.subscribe<std_msgs::Float64>("feedforward_mode", 1,
                                                                         &PowerMachineController::feedforwardModeCallback,
                                                                         this);

            desired_spd_publisher_ = root_nh.advertise<std_msgs::Float64>("desired_speed", 1);
            error_publisher_ = root_nh.advertise<std_msgs::Float64>("error", 1);

            ROS_INFO_STREAM("PowerMachineController initialized successfully.");
            ROS_INFO_STREAM("PID gains - P: " << p_gain_ << " I: " << i_gain_ << " D: " << d_gain_);
            return true;
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM("Failed to get joint handle: " << e.what());
            return false;
        }
    }

    void PowerMachineController::update(const ros::Time &time, const ros::Duration &period) {
        computeEfforts(time, period);
    }

    void PowerMachineController::commandCallback(const std_msgs::Float64::ConstPtr &msg) {
        command_ = msg->data;
        // 只有当接收到的命令不为0且与上一次命令不同时，才记录时间
        if (command_ != 0.0 && command_ != last_command_) {
            record_time_ = ros::Time::now().toSec(); // 记录当前ROS时间
        }
        last_command_ = command_; // 更新上一次的命令值
    }

    void PowerMachineController::computeEfforts(const ros::Time &time, const ros::Duration &period) {
        // 计算当前时间点的目标速度
        double t = time.toSec(); // 获取当前时间（秒）
        if (command_ == 0) {
            spd = 0;
        }
        else if (command_ == 2)
        {
          spd= 1.047;
        }
        else
        {
          spd = a * sin(omega * (t - record_time_)) + b;
        }

        // 获取当前速度
        double current_velocity = motor_joint_.getVelocity();

        // 计算速度误差
        error = spd - current_velocity;
        std_msgs::Float64 msg;
        std_msgs::Float64 error_msg;

        msg.data = spd;
        error_msg.data = error;

        desired_spd_publisher_.publish(msg);
        error_publisher_.publish(error_msg);
        // 根据前馈模式标志选择控制方式
        if (feedforward_enabled_) {
            // 前馈控制：直接使用误差乘以前馈增益
            double uff = k_gain_ * error;
            double effort = pid_controller_.computeCommand(error, period);
            motor_joint_.setCommand(uff + effort);

            // 限制日志输出频率
            double current_time = ros::Time::now().toSec();
            if (current_time - last_log_time_ >= 1.0) {
                // 每秒输出一次
                ROS_INFO_STREAM("Using feedforward control with gain: " << k_gain_);
                last_log_time_ = current_time;
            }
        } else {
            // PID控制：使用PID控制器计算输出力矩
            double effort = pid_controller_.computeCommand(error, period);
            motor_joint_.setCommand(effort);

            // 限制日志输出频率
            double current_time = ros::Time::now().toSec();
            if (current_time - last_log_time_ >= 1.0) {
                // 每秒输出一次
                ROS_INFO_STREAM("Using PID control");
                last_log_time_ = current_time;
            }
        }
    }
} // namespace power_machine_controller

// 插件注册宏
PLUGINLIB_EXPORT_CLASS(power_machine_controller::PowerMachineController, controller_interface::ControllerBase)
