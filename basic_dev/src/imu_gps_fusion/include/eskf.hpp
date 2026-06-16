#ifndef _ESKF_HPP_
#define _ESKF_HPP_

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <mutex>

Eigen::Vector3d quatToEuler(const Eigen::Quaterniond& q); // 返回z-y-x顺序的欧拉角，单位为弧度

class ESKF
{
public:
    ESKF();
    ~ESKF();

    void init(Eigen::Vector3d init_position, Eigen::Quaterniond init_orientation, double init_time);
    void setQR(double gyro_bias_noise, double accel_bias_noise, double gps_position_noise, double gps_orientation_noise);
    bool predict(const Eigen::Vector3d& imu_angular_velocity, const Eigen::Vector3d& imu_linear_acceleration, double now_time);
    bool update(const Eigen::Vector3d& gps_position, const Eigen::Quaterniond& gps_q);
    static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);

    // State vector: [position(3), velocity(3), orientation(3), gyro_bias(3), accel_bias(3)]
    static const int state_size_ = 15;
    static const int measurement_size_ = 6;
    static const int noise_size_ = 6; // IMU噪声：陀螺仪噪声(3) + 加速度计噪声(3)
    static const int position_index_ = 0;
    static const int velocity_index_ = 3;
    static const int orientation_index_ = 6;
    static const int gyro_bias_index_ = 9;
    static const int accel_bias_index_ = 12;
    static const int gps_position_index_ = 0;
    static const int gps_orientation_index_ = 3;
    Eigen::Matrix<double, state_size_, state_size_> P_; // Error covariance matrix
    Eigen::Matrix<double, state_size_, state_size_> F_x_; // State transition matrix
    Eigen::Matrix<double, state_size_, noise_size_> m_b_; // noise input matrix
    Eigen::Matrix<double, noise_size_, noise_size_> Q_; // Process noise covariance matrix
    Eigen::Matrix<double, measurement_size_, measurement_size_> R_; // Measurement noise covariance matrix
    Eigen::Matrix<double, measurement_size_, state_size_> H_; // Measurement matrix
    Eigen::Matrix<double, state_size_, 1> x_; // State vector
    Eigen::Matrix<double, measurement_size_, 1> measurement_; // Measurement vector

    // NED坐标系下的重力加速度
    Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, 9.81);
    Eigen::Matrix3d R_imu_to_world_; // IMU坐标系到世界坐标系的旋转矩阵
    Eigen::Vector3d state_position_; // 位置
    Eigen::Vector3d state_velocity_; // 速度
    Eigen::Quaterniond state_orientation_; // 四元数表示的姿态
    Eigen::Vector3d state_gyro_velocity_; // 角速度
    Eigen::Vector3d state_gyro_bias_; // 陀螺仪偏置
    Eigen::Vector3d state_accel_bias_; // 加速度计偏置
    Eigen::Vector3d last_unbias_acc_; // 上一次的去偏加速度
    Eigen::Vector3d last_unbias_gyro_; // 上一次的去偏角速度

    // 互斥锁，保护状态更新的线程安全
    std::mutex mtx_; // 互斥锁，保护状态更新的线程安全

    double last_time_ = 0.0;
};

#endif // _ESKF_HPP_