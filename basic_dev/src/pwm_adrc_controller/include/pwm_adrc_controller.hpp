#ifndef _PWM_ADRC_CONTROLLER_HPP_
#define _PWM_ADRC_CONTROLLER_HPP_

#include <ros/ros.h>
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/RotorPWM.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "Eigen/Dense"
#include "teminal_control.hpp"

// ====================================================
// 全局变量声明：跨文件共享的核心状态变量
// ====================================================
extern bool is_takeoff;
extern bool is_land;
extern bool is_reset;
extern bool is_pwm_published;
extern bool is_cout;
extern bool is_flying; 

extern bool enable_position_loop; 
extern bool g_use_planner_input;
extern double key_target_x;
extern double key_target_y;
extern double term_target_yaw;
extern double term_target_z; 
extern double current_yaw; 

// ⭐ 新增：姿态模式的专属控制变量
extern double term_target_roll;
extern double term_target_pitch;
extern bool g_attitude_lock_enabled;

// 矩阵式在线调参变量 (直接存储真实的 Vector3d 值)
extern Eigen::Vector3d g_tune_kp_pos;
extern Eigen::Vector3d g_tune_wc_vel;
extern Eigen::Vector3d g_tune_wp_vel;
extern Eigen::Vector3d g_tune_kp_ang;
extern Eigen::Vector3d g_tune_wc_att;
extern Eigen::Vector3d g_tune_wp_att;
extern Eigen::Vector3d g_base_b0;
extern Eigen::Vector3d g_tune_b0_scale;
extern Eigen::Vector3d g_tune_b0;
extern double g_tune_tau;

extern int g_tuning_param_idx; // 当前选中的参数类 (1-8)
extern int g_tuning_axis_idx;  // 当前选中的轴 (0:X, 1:Y, 2:Z, 3:Global)

extern int g_keyboard_fd;
extern bool g_key_control_enabled;

// 无人机状态结构体：包含位置、速度、姿态及其导数
struct DroneState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d eulerAngle_velocity;
    double roll;
    double pitch;
    double yaw;
};

// 欧拉角转换工具函数声明
static Eigen::Vector3d quatToEuler(const Eigen::Quaterniond& q); 

class PwmAdrcController {
private:
    ros::Subscriber odom_suber; 
    ros::Subscriber target_pose_suber_;
    ros::Subscriber start_state_suber_;

    airsim_ros::Takeoff takeoff;
    airsim_ros::Land land;
    airsim_ros::Reset reset;
    ros::ServiceClient takeoff_client;
    ros::ServiceClient land_client;
    ros::ServiceClient reset_client;

    ros::Publisher pwm_publisher;
    ros::Publisher reset_publisher;
    ros::Publisher control_start_publisher;
    ros::Publisher target_debug_publisher;

    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void target_pose_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void start_state_cb(const std_msgs::Header::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
    
    double limit(double value, double min_value, double max_value);

    // 外环：位置速度串级自抗扰控制器
    void position_velocity_adrc(
        const Eigen::Vector3d& target_pos, const Eigen::Vector3d& target_vel,
        const Eigen::Vector3d& current_pos, const Eigen::Vector3d& current_vel, double target_yaw, double dt,
        Eigen::Matrix3d& target_Rd, double& base_throttle);

    // 内环：姿态自抗扰控制器与控制分配
    Eigen::Vector4d adrc_controller_block(
        const Eigen::Matrix3d& target_Rd, const Eigen::Matrix3d& current_dcm,
        const Eigen::Vector3d& current_gyro, double base_throttle, double dt);

    DroneState current_drone_state_;
    bool is_initialized_ = false;
    bool is_started_ = false;
    bool is_control_enabled_ = false;
    bool has_planning_target_ = false;
    ros::Time start_time_;
    ros::Time latest_target_stamp_;
    double control_start_delay_sec_ = 2.0;
    double max_planner_target_age_sec_ = 0.5;
    double planner_z_offset_ = 0.0;

    Eigen::Vector3d target_position_;
    Eigen::Vector3d target_velocity_;
    Eigen::Vector3d smooth_target_position_; 
    bool has_smooth_target_ = false;         
    
    ros::Timer command_timer_;

    // 扩张状态观测器 (LESO) 历史状态变量
    Eigen::Vector3d pos_Z1, pos_Z2, pos_U; 
    Eigen::Vector3d att_Z1, att_Z2, att_U; 
    double key_pos_Z1, key_pos_Z2, key_pos_U;

public:
    PwmAdrcController(ros::NodeHandle *nh);
    ~PwmAdrcController() = default;
};

#endif
