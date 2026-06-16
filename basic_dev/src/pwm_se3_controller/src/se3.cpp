#include "se3.hpp"

SE3Controller::SE3Controller()
{
    // 默认增益值, 具体setGains传入
    position_gain_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    velocity_gain_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    eulerAngle_gain_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    eulerAngle_velocity_gain_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    quadrotor_mass_ = 0.9; // 无人机质量，单位为kg
    circle_time_ = 0.01; // 控制周期时间，单位为秒

    // 预期推力初始值为0
    target_body_thrust_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    last_target_R = Eigen::Matrix3d::Identity();
}

void SE3Controller::setParams(double quadrotor_mass, double circle_time, const Eigen::Vector3d& I, const Eigen::Vector3d& position_gain, const Eigen::Vector3d& velocity_gain, const Eigen::Vector3d& eulerAngle_gain, const Eigen::Vector3d& eulerAngle_velocity_gain)
{
    quadrotor_mass_ = quadrotor_mass;
    circle_time_ = circle_time;
    J = Eigen::Matrix3d::Zero();
    J.diagonal() = I;
    position_gain_ = position_gain;
    velocity_gain_ = velocity_gain;
    eulerAngle_gain_ = eulerAngle_gain;
    eulerAngle_velocity_gain_ = eulerAngle_velocity_gain;
}

void SE3Controller::setPositionGain(const Eigen::Vector3d& position_gain)
{
    position_gain_ = position_gain;
}

void SE3Controller::setVelocityGain(const Eigen::Vector3d& velocity_gain)
{
    velocity_gain_ = velocity_gain;
}

void SE3Controller::setEulerAngleGain(const Eigen::Vector3d& eulerAngle_gain)
{
    eulerAngle_gain_ = eulerAngle_gain;
}

void SE3Controller::setEulerAngleVelocityGain(const Eigen::Vector3d& eulerAngle_velocity_gain)
{
    eulerAngle_velocity_gain_ = eulerAngle_velocity_gain;
}

void SE3Controller::positionControl(const DroneState& current_state, const Eigen::Vector3d& target_position, const Eigen::Vector3d& target_velocity, const Eigen::Vector3d& target_acceleration)
{
    // 位置误差
    Eigen::Vector3d position_error = target_position - current_state.position;
    // 速度误差，这里假设目标速度为0
    Eigen::Vector3d velocity_error = target_velocity - current_state.velocity;
    // e3轴方向的单位向量
    Eigen::Vector3d e3(0.0, 0.0, 1.0);

    // 计算世界坐标系下的预期推力
    target_world_thrust_ = vector3dMultiply(position_gain_, position_error) + vector3dMultiply(velocity_gain_, velocity_error)
                                                - quadrotor_mass_ * g * e3 + quadrotor_mass_ * target_acceleration;

    // 根据世界坐标系到机体坐标系的旋转关系，将预期推力转换到机体坐标系下
    target_body_thrust_ = current_state.orientation.inverse() * target_world_thrust_;

    return;
}

void SE3Controller::positionControlWithoutAcc(const DroneState& current_state, const Eigen::Vector3d& target_position, const Eigen::Vector3d& target_velocity)
{
    Eigen::Vector3d target_acceleration = Eigen::Vector3d::Zero();

    if (has_last_target_velocity_ && circle_time_ > 1e-6) {
        Eigen::Vector3d raw_acceleration =
            (target_velocity - last_target_velocity_) / circle_time_;

        // 简单低通滤波，避免差分噪声太大
        constexpr double alpha = 0.6;
        filtered_target_acceleration_ =
            alpha * raw_acceleration + (1.0 - alpha) * filtered_target_acceleration_;

        // 限制目标加速度在合理范围内，避免控制发散
        const double max_target_acceleration = 8.0; // 最大目标加速度，单位为m/s^2
        for (int i = 0; i < 3; ++i) {
            filtered_target_acceleration_[i] = std::max(
                -max_target_acceleration,
                std::min(filtered_target_acceleration_[i], max_target_acceleration));
        }

        target_acceleration = filtered_target_acceleration_;
    }

    last_target_velocity_ = target_velocity;
    has_last_target_velocity_ = true;

    positionControl(current_state, target_position, target_velocity, target_acceleration);
    return;
}

void SE3Controller::resetTargetVelocityDiff()
{
    has_last_target_velocity_ = false;
    last_target_velocity_.setZero();
    filtered_target_acceleration_.setZero();
}

// target_eulerAngle的顺序为z-y-x，对应航向角-俯仰角-滚转角
void SE3Controller::eulerAngleControl(const DroneState& current_state, double target_yaw) 
{
    // 旋转矩阵误差
    Eigen::Matrix3d R = current_state.orientation.toRotationMatrix();
    // 预测目标旋转矩阵
    // 防止target_world_thrust_为零导致的计算不稳定，增加一个小的阈值来判断是否需要使用当前姿态的b3轴方向
    constexpr double kThrustEps = 1e-4;
    const double thrust_norm = target_world_thrust_.norm();
    Eigen::Vector3d b_3d;
    if (thrust_norm > kThrustEps) {
        b_3d = -target_world_thrust_ / thrust_norm;
    } else {
        b_3d = R.col(2);
    }

    // 限制最大倾斜角，防止期望姿态过大
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMaxTiltRad = 60.0 * kPi / 180.0;
    constexpr double kTiltEps = 1e-6;

    const Eigen::Vector3d e3(0.0, 0.0, 1.0);
    const double cos_max_tilt = std::cos(kMaxTiltRad);
    const double sin_max_tilt = std::sin(kMaxTiltRad);

    if (b_3d.dot(e3) < cos_max_tilt) {
        Eigen::Vector3d b3_horizontal(b_3d.x(), b_3d.y(), 0.0);
        const double horizontal_norm = b3_horizontal.norm();

        if (horizontal_norm > kTiltEps) {
            b3_horizontal /= horizontal_norm;
            b_3d = sin_max_tilt * b3_horizontal + cos_max_tilt * e3;
        } else {
            b_3d = e3;
        }

        b_3d.normalize();
    }

    Eigen::Vector3d b_1d(cos(target_yaw), sin(target_yaw), 0.0); // 期望航向角对应的b1轴在水平面上的投影

    constexpr double kSingularityEps = 1e-3;
    // 奇异点处理：当b3轴和b1轴接近平行时，b2轴的计算会不稳定，选用基于当前实际姿态的最近解来避免奇异点
    if (b_3d.cross(b_1d).norm() < kSingularityEps) {
        b_1d = R.col(0); // 直接使用当前姿态的b1轴作为目标b1轴

        if (b_3d.cross(b_1d).norm() < kSingularityEps) {
            Eigen::Vector3d world_x(1.0, 0.0, 0.0);
            Eigen::Vector3d world_y(0.0, 1.0, 0.0);

            b_1d = (std::abs(b_3d.dot(world_x)) < 0.9) ? world_x : world_y;
        }
    }

    Eigen::Vector3d b_2d = b_3d.cross(b_1d).normalized(); // 通过b3轴和b1轴的叉乘得到b2轴
    Eigen::Vector3d b_1d_orthogonal = b_2d.cross(b_3d); // 通过b2轴和b3轴的叉乘得到与b3轴垂直的b1轴
    Eigen::Matrix3d target_R = Eigen::Matrix3d::Zero();
    target_R.col(0) = b_1d_orthogonal;
    target_R.col(1) = b_2d;
    target_R.col(2) = b_3d;
    Eigen::Matrix3d R_error = 0.5 * (target_R.transpose() * R - R.transpose() * target_R); // 旋转矩阵误差的反对称部分对应的向量即为误差角轴
    Eigen::Vector3d eulerAngle_error = getVee(R_error);

    // 角速度误差
    // 计算目标角速度，通过旋转矩阵误差的微分项来增加阻尼，防止过冲和振荡
    Eigen::Matrix3d target_R_dot = (target_R - last_target_R) / circle_time_;
    Eigen::Vector3d target_eulerAngle_velocity = getVee(target_R.transpose() * target_R_dot);
    Eigen::Vector3d eulerAngle_velocity_error = current_state.eulerAngle_velocity - R.transpose() * target_R * target_eulerAngle_velocity;

    // 得到力矩控制输出， 暂时未加入-J...
    target_body_torque_ = - vector3dMultiply(eulerAngle_gain_, eulerAngle_error) - vector3dMultiply(eulerAngle_velocity_gain_, eulerAngle_velocity_error)
                                + current_state.eulerAngle_velocity.cross(J * current_state.eulerAngle_velocity);

    last_target_R = target_R;
    return;
}

Eigen::Vector3d vector3dMultiply(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    return Eigen::Vector3d(a.x() * b.x(), a.y() * b.y(), a.z() * b.z());
}

Eigen::Matrix3d getSkewSymmetricMatrix(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d skew;
    skew <<     0, -v.z(),  v.y(),
             v.z(),     0, -v.x(),
            -v.y(),  v.x(),     0;
    return skew;
}

Eigen::Vector3d getVee(const Eigen::Matrix3d& skew)
{
    return Eigen::Vector3d(skew(2, 1), skew(0, 2), skew(1, 0));
}
