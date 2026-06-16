#ifndef _PWM_PID_CONTROLLER_HPP_
#define _PWM_PID_CONTROLLER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include "airsim_ros/RotorPWM.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include  "sensor_msgs/Imu.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include "quadrotor_msgs/PositionCommand.h"

#include "pid.hpp"

#include "teminal_control.hpp"

bool is_takeoff = false;
bool is_land = false;
bool is_reset = false;
bool is_pwm_published = true;
bool is_cout = false;
double target_pitch = 0.0;
double target_roll = 0.0;
double target_yaw = 0.0;
double target_x = 0.0;
double target_y = 0.0;
double target_z = -2.0;

int g_keyboard_fd = -1;
bool g_key_control_enabled = false;

TeminalPidSet teminal_pid_set;

struct DroneState
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d eulerAngle_velocity;
    double roll;
    double pitch;
    double yaw;
};

struct DroneParam
{
    double mass;
    double g; // 重力加速度
    double R; // 轴距（电机至机体中心）
    Eigen::Vector3d I; // 转动惯量，分别为Ixx, Iyy, Izz
    double ct;  // 电机升力系数
    double cm; // 电机反扭矩系数
    double c; // 偏航系数
    double max_n; // 电机最大转速
    double max_w; // 电机最大角速度
};

static Eigen::Vector3d quatToEuler(const Eigen::Quaterniond& q); // 返回z-y-x顺序的欧拉角，单位为弧度

class PwmPidController
{
private:
    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    ros::Subscriber odom_suber; //imu与gps数据融合后的位姿数据
    ros::Subscriber target_point_suber_; // 目标点数据
    ros::Subscriber target_pose_suber_; // 目标位姿数据，包含位置和速度和加速度和航向角命令
    ros::Subscriber target_yaw_suber_;   // 目标偏航角（门法线方向）

    // 调用服务前需要定义特定的调用参数
    airsim_ros::Takeoff takeoff;
    airsim_ros::Land land;
    airsim_ros::Reset reset;
    //通过这三个服务可以调用模拟器中的无人机起飞和降落命令和重置命令
    ros::ServiceClient takeoff_client;
    ros::ServiceClient land_client;
    ros::ServiceClient reset_client;

    ros::Publisher pwm_publisher;
    ros::Publisher reset_publisher;

    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void target_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg);
    void target_pose_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void target_yaw_cb(const std_msgs::Float64::ConstPtr& msg); // 接收门法线偏航角

    void pid_init();
    void position_control(const DroneState& current_state, const Eigen::Vector3d& target_position);
    void pose_control(const DroneState& current_state, const Eigen::Vector3d& target_eulerAngle);
    void publish_pwm_command(const ros::Time& stamp);
    void timerCallback(const ros::TimerEvent& event);
    void controlLoop();
    void startControlThread();
    void stopControlThread();
    double limit(double value, double min_value, double max_value);

    DroneState current_drone_state_;
    DroneParam drone_param_;
    Eigen::Vector3d target_position_;
    Eigen::Vector3d target_eulerAngle_; // roll, pitch, yaw
    double target_throttle_;
    Eigen::Vector3d target_T; // roll, pitch, yaw轴的力矩命令

    // 来自图像解算器的门法线偏航角
    double target_gate_yaw_ = 0.0;      // 最新一次所将接收到的门偏航角(rad)
    bool   gate_yaw_received_ = false;  // 是否已经收到过有效偏航角
    
    Eigen::Matrix4d control_distribution_matrix_; // 控制分配矩阵
    Eigen::Matrix4d control_distribution_matrix_inv_; // 控制分配矩阵的逆矩阵
    Eigen::Matrix2d A_yaw; // 航向角速度->角度变换矩阵
    std::mutex state_mutex_;
    std::mutex pid_mutex_;
    DroneState latest_drone_state_;
    ros::Time latest_state_stamp_;
    ros::Time latest_state_recv_time_;
    std::atomic<bool> state_received_{false};

    std::thread control_thread_;
    std::atomic<bool> control_loop_running_{false};
    double control_loop_hz_ = 200.0;
    double max_state_age_sec_ = 0.2;
    ros::Timer command_timer_;

    std::string target_type_; // "position" 或 "pose"
    double ahead_time_sec_; // 预测提前量，单位秒

    Pid pid_x_;
    Pid pid_y_;
    Pid pid_z_;
    Pid pid_v_x_;
    Pid pid_v_y_;
    Pid pid_v_z_;

    Pid pid_roll_;
    Pid pid_pitch_;
    Pid pid_yaw_;
    Pid pid_v_roll_;
    Pid pid_v_pitch_;
    Pid pid_v_yaw_;
    double feedforward_roll_gain_ = 0.0;
    double feedforward_pitch_gain_ = 0.0;
    double feedforward_yaw_gain_ = 0.0;
    double feedforward_z_gain_ = 0.0;

public:
    PwmPidController(ros::NodeHandle *nh);
    ~PwmPidController();

};

#endif




