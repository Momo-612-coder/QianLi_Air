#include "eskf.hpp"

ESKF::ESKF()
{
    state_position_ = Eigen::Vector3d::Zero();
    state_velocity_ = Eigen::Vector3d::Zero();
    state_orientation_ = Eigen::Quaterniond::Identity();
    state_gyro_bias_ = Eigen::Vector3d::Zero();
    state_accel_bias_ = Eigen::Vector3d::Zero();
    last_unbias_acc_ = Eigen::Vector3d::Zero();
    last_unbias_gyro_ = Eigen::Vector3d::Zero();
    R_imu_to_world_ = state_orientation_.toRotationMatrix();

    P_ = Eigen::Matrix<double, state_size_, state_size_>::Identity() * 0.1;
    F_x_ = Eigen::Matrix<double, state_size_, state_size_>::Zero();
    m_b_ = Eigen::Matrix<double, state_size_, noise_size_>::Zero();
    Q_ = Eigen::Matrix<double, noise_size_, noise_size_>::Identity() * 0.01;
    R_ = Eigen::Matrix<double, measurement_size_, measurement_size_>::Identity() * 0.1;
    H_ = Eigen::Matrix<double, measurement_size_, state_size_>::Zero();
    H_.block<3, 3>(gps_position_index_, position_index_) = Eigen::Matrix3d::Identity();
    H_.block<3, 3>(gps_orientation_index_, orientation_index_) = Eigen::Matrix3d::Identity();

    x_ = Eigen::Matrix<double, state_size_, 1>::Zero();
    measurement_ = Eigen::Matrix<double, measurement_size_, 1>::Zero();
}

ESKF::~ESKF()
{

}

void ESKF::init(Eigen::Vector3d init_position, Eigen::Quaterniond init_orientation, double init_time)
{
    state_position_ = init_position;
    state_orientation_ = init_orientation;
    R_imu_to_world_ = state_orientation_.toRotationMatrix();
    last_time_ = init_time;
    // 初始化加速度为消除重力的理想稳态值，以避免起始时的速度突变
    last_unbias_acc_ = R_imu_to_world_.transpose() * (-gravity_);
}

void ESKF::setQR(double gyro_bias_noise, double accel_bias_noise, double gps_position_noise, double gps_orientation_noise)
{
    Q_.setZero();
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyro_bias_noise * gyro_bias_noise;
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * accel_bias_noise * accel_bias_noise;

    R_.setZero();
    R_.block<3, 3>(gps_position_index_, gps_position_index_) = gps_position_noise * gps_position_noise * Eigen::Matrix3d::Identity();
    R_.block<3, 3>(gps_orientation_index_, gps_orientation_index_) = gps_orientation_noise * gps_orientation_noise * Eigen::Matrix3d::Identity();
}

bool ESKF::predict(const Eigen::Vector3d& imu_angular_velocity, const Eigen::Vector3d& imu_linear_acceleration, double now_time)
{
    double dt = now_time - last_time_;
    if (dt <= 0.0) {
        return false;
    }
    if (dt > 0.1) {
        // 如果间隔过长（例如刚启动或者仿真卡顿），则限制步长避免发散，甚至可以选择重置
        dt = 0.01; 
    }

    std::lock_guard<std::mutex> lock(mtx_);
    Eigen::Vector3d unbias_gyro = imu_angular_velocity - state_gyro_bias_;
    Eigen::Vector3d unbias_acc = imu_linear_acceleration - state_accel_bias_;

    state_gyro_velocity_ = unbias_gyro; // 直接使用原始角速度作为状态的一部分，方便外部访问和调试 

    Eigen::Quaterniond last_orientation = state_orientation_;
    Eigen::Vector3d last_position = state_position_;
    Eigen::Vector3d last_velocity = state_velocity_;

    
    // Update orientation - 采用与上一时刻做平均的方式来计算当前时刻的delta_q，以提高预测的准确性
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d avg_unbias_gyro = 0.5 * (unbias_gyro + last_unbias_gyro_);
    double angle = avg_unbias_gyro.norm() * dt;
    Eigen::Vector3d axis = avg_unbias_gyro.normalized();
    if (angle > 1e-5) {
        delta_q = Eigen::AngleAxisd(angle, axis);
    } else {
        delta_q = Eigen::Quaterniond::Identity();
    }
    state_orientation_ = state_orientation_ * delta_q;
    state_orientation_.normalize();
    R_imu_to_world_ = state_orientation_.toRotationMatrix();

    // Update velocity and position
    Eigen::Vector3d acc_world = R_imu_to_world_ * unbias_acc + gravity_;
    Eigen::Vector3d last_acc_world = last_orientation.toRotationMatrix() * last_unbias_acc_ + gravity_;
    state_velocity_ = last_velocity + (acc_world + last_acc_world) * 0.5 * dt;
    state_position_ = last_position + (state_velocity_ + last_velocity) * 0.5 * dt;

    // Update state transition matrix F_x_ and noise input matrix m_b_ for EKF linearization
    F_x_.setZero();
    F_x_.block<3, 3>(position_index_, velocity_index_) = Eigen::Matrix3d::Identity();
    F_x_.block<3, 3>(velocity_index_, orientation_index_) = -R_imu_to_world_ * skewSymmetric(unbias_acc);
    F_x_.block<3, 3>(velocity_index_, accel_bias_index_) = -R_imu_to_world_;
    F_x_.block<3, 3>(orientation_index_, orientation_index_) = -skewSymmetric(unbias_gyro);
    F_x_.block<3, 3>(orientation_index_, gyro_bias_index_) = -Eigen::Matrix3d::Identity();
    m_b_.setZero();
    m_b_.block<3, 3>(velocity_index_, 3) = -R_imu_to_world_;
    m_b_.block<3, 3>(orientation_index_, 0) = -Eigen::Matrix3d::Identity();
    // 离散化
    Eigen::Matrix<double, state_size_, state_size_> Fk = Eigen::Matrix<double, state_size_, state_size_>::Identity() + F_x_ * dt;
    // Eigen::Matrix<double, state_size_, noise_size_> Bk = m_b_ * dt; // 原始代码这里将Bk写成了乘以dt，会在计算Q_k时变成dt^2，导致过程噪声太小！

    // 更新状态向量和协方差矩阵
    x_ = Fk * x_;
    P_ = Fk * P_ * Fk.transpose() + m_b_ * Q_ * m_b_.transpose() * dt; // 正确的做法是乘以一次dt

    // 给偏置状态直接添加少许过程噪声，防止协方差收敛到0后无法继续估计偏置
    P_.block<3, 3>(gyro_bias_index_, gyro_bias_index_) += Eigen::Matrix3d::Identity() * 1e-8 * dt;
    P_.block<3, 3>(accel_bias_index_, accel_bias_index_) += Eigen::Matrix3d::Identity() * 1e-6 * dt;

    last_time_ = now_time;
    last_unbias_gyro_ = unbias_gyro;
    last_unbias_acc_ = unbias_acc;

    // // 打印预测结果
    // std::cout << "Predicted position: " << state_position_.transpose() << std::endl;
    // std::cout << "Predicted velocity: " << state_velocity_.transpose() << std::endl;
    // std::cout << "Predicted orientation (quaternion): " << state_orientation_.coeffs().transpose() << std::endl;
    // // 计算欧拉角以便更直观地观察姿态变化
    // Eigen::Vector3d eulerAngle = quatToEuler(state_orientation_); // 注意这里的顺序是z-y-x，对应航向角-俯仰角-滚转角
    // std::cout << "Predicted angle:roll: " << eulerAngle[2] * 57.3 << " deg, pitch: " << eulerAngle[1] * 57.3 << " deg, yaw: " << eulerAngle[0] * 57.3 << " deg" << std::endl;
    // std::cout << "Predicted gyro bias: " << state_gyro_bias_.transpose() << std::endl;
    // std::cout << "Predicted accel bias: " << state_accel_bias_.transpose() << std::endl;
    return true;
}

bool ESKF::update(const Eigen::Vector3d& gps_position, const Eigen::Quaterniond& gps_q)
{
    std::lock_guard<std::mutex> lock(mtx_);
    // 计算测量残差
    measurement_.setZero();
    measurement_.block<3, 1>(gps_position_index_, 0) = gps_position - state_position_;
    Eigen::Matrix3d err_rotation = state_orientation_.toRotationMatrix().transpose() * gps_q.toRotationMatrix();
    Eigen::AngleAxisd err_angle_axis(err_rotation);
    measurement_.block<3, 1>(gps_orientation_index_, 0) = err_angle_axis.axis() * err_angle_axis.angle();
    Eigen::Matrix<double, measurement_size_, 1> y = measurement_ - H_ * x_;
    Eigen::Matrix<double, measurement_size_, measurement_size_> S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, state_size_, measurement_size_> K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    //P_ = (Eigen::Matrix<double, state_size_, state_size_>::Identity() - K * H_) * P_;
    // Joseph form for numerical stability
    Eigen::Matrix<double, state_size_, state_size_> I = Eigen::Matrix<double, state_size_, state_size_>::Identity();
    P_ = (I - K * H_) * P_ * (I - K * H_).transpose() + K * R_ * K.transpose();

    // 更新状态
    state_position_ = state_position_ + x_.block<3, 1>(position_index_, 0);
    state_velocity_ = state_velocity_ + x_.block<3, 1>(velocity_index_, 0);
    Eigen::Vector3d delta_angle = x_.block<3, 1>(orientation_index_, 0);
    double delta_angle_norm = delta_angle.norm();
    if (delta_angle_norm > 1e-5) {
        Eigen::Quaterniond delta_q = Eigen::Quaterniond(Eigen::AngleAxisd(delta_angle_norm, delta_angle.normalized()));
        state_orientation_ = state_orientation_ * delta_q;
        state_orientation_.normalize();
    }
    R_imu_to_world_ = state_orientation_.toRotationMatrix();
    state_gyro_bias_ = state_gyro_bias_ + x_.block<3, 1>(gyro_bias_index_, 0);
    state_accel_bias_ = state_accel_bias_ + x_.block<3, 1>(accel_bias_index_, 0);

    state_gyro_velocity_ = state_gyro_velocity_ - state_gyro_bias_; // 更新后的角速度

    x_.setZero(); // 更新后重置误差状态
    return true;
}

Eigen::Matrix3d ESKF::skewSymmetric(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d skew;
    skew << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return skew;
}

Eigen::Vector3d quatToEuler(const Eigen::Quaterniond& q) // 返回z-y-x顺序的欧拉角，单位为弧度
{
    // roll
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // pitch
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);
    
    // yaw
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return Eigen::Vector3d(yaw, pitch, roll);

}