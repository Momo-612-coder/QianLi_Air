#include <stdlib.h>
#include "Eigen/Dense"
#include <algorithm>

const double g = 9.81; // 重力加速度，单位为m/s^2

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

Eigen::Vector3d vector3dMultiply(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
Eigen::Matrix3d getSkewSymmetricMatrix(const Eigen::Vector3d& v);
Eigen::Vector3d getVee(const Eigen::Matrix3d& skew);

class SE3Controller
{
public:
    SE3Controller();
    void setParams(double quadrotor_mass, double circle_time, const Eigen::Vector3d& I, const Eigen::Vector3d& position_gain, const Eigen::Vector3d& velocity_gain, const Eigen::Vector3d& eulerAngle_gain, const Eigen::Vector3d& eulerAngle_velocity_gain);
    void setPositionGain(const Eigen::Vector3d& position_gain);
    void setVelocityGain(const Eigen::Vector3d& velocity_gain);
    void setEulerAngleGain(const Eigen::Vector3d& eulerAngle_gain);
    void setEulerAngleVelocityGain(const Eigen::Vector3d& eulerAngle_velocity_gain);
    void positionControl(const DroneState& current_state, const Eigen::Vector3d& target_position, const Eigen::Vector3d& target_velocity, const Eigen::Vector3d& target_acceleration);
    void positionControlWithoutAcc(const DroneState& current_state, const Eigen::Vector3d& target_position, const Eigen::Vector3d& target_velocity);
    void eulerAngleControl(const DroneState& current_state, double target_yaw); 
    void resetTargetVelocityDiff();

    double quadrotor_mass_;
    double circle_time_ = 0.0; // 用于记录控制周期时间，单位为秒
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d eulerAngle_gain_;
    Eigen::Vector3d eulerAngle_velocity_gain_;

    Eigen::Matrix3d J; // 无人机的转动惯量矩阵，单位为kg*m^2

    Eigen::Vector3d target_world_thrust_; // 世界坐标系下的预期推力
    Eigen::Vector3d target_body_thrust_; // 机体坐标系下的预期推力
    Eigen::Matrix3d last_target_R; // 上一时刻的目标旋转矩阵，用于计算旋转矩阵误差的微分项
    Eigen::Vector3d target_body_torque_; // 机体坐标系下的预期力矩
    Eigen::Vector3d last_target_velocity_; // 上一时刻的目标速度，用于计算速度误差的微分项
    Eigen::Vector3d filtered_target_acceleration_; // 上一时刻的目标加速度，用于计算加速度误差的微分项
    bool has_last_target_velocity_ = false; // 是否有上一时刻的目标速度


};