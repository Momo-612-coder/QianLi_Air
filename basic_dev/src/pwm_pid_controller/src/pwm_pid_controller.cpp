#ifndef _PWM_PID_CONTROLLER_CPP_
#define _PWM_PID_CONTROLLER_CPP_

#include "pwm_pid_controller.hpp"

#include <limits>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "pwm_pid_controller"); // 初始化ros 节点，命名为 pwm_pid_controller
    ros::NodeHandle n; // 创建node控制句柄
    PwmPidController go(&n);
    return 0;
}

PwmPidController::PwmPidController(ros::NodeHandle *nh)
{  
    nh->param("control_loop_hz", control_loop_hz_, 100.0);
    nh->param("max_state_age_sec", max_state_age_sec_, 0.2);
    nh->param("target_type", target_type_, std::string("position")); // 可选 "position" 或 "pose"
    nh->param("ahead_time_sec", ahead_time_sec_, 0.5); // 预测提前量，单位秒

    // 初始化无人机参数
    drone_param_.mass = 0.9;
    drone_param_.g = 9.81;
    drone_param_.R = 0.18;
    drone_param_.I = Eigen::Vector3d(0.0046890742, 0.0069312, 0.010421166);
    drone_param_.ct = 0.000367717;
    drone_param_.cm = 4.888486266072161e-06;
    drone_param_.c = 0.01; // 偏航系数，经验值
    drone_param_.max_n = 11079.03; // 电机最大转速, r/min
    drone_param_.max_w = drone_param_.max_n * 2.0 * M_PI / 60.0; // 电机最大角速度, rad/s
    double ct = drone_param_.ct;
    double cm = drone_param_.cm;
    double sqrt2_2 = std::sqrt(2) / 2.0;
    double M_23 = sqrt2_2 * ct * drone_param_.R;
    control_distribution_matrix_ << ct, ct, ct, ct,
                                    -M_23, M_23, M_23, -M_23,
                                    M_23, -M_23, M_23, -M_23,
                                     cm, cm, -cm, -cm;
    control_distribution_matrix_inv_ = control_distribution_matrix_.inverse();

    ROS_INFO("Control distribution matrix:\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
        control_distribution_matrix_(0,0), control_distribution_matrix_(0,1), control_distribution_matrix_(0,2), control_distribution_matrix_(0,3),
        control_distribution_matrix_(1,0), control_distribution_matrix_(1,1), control_distribution_matrix_(1,2), control_distribution_matrix_(1,3),
        control_distribution_matrix_(2,0), control_distribution_matrix_(2,1), control_distribution_matrix_(2,2), control_distribution_matrix_(2,3),
        control_distribution_matrix_(3,0), control_distribution_matrix_(3,1), control_distribution_matrix_(3,2), control_distribution_matrix_(3,3));

     ROS_INFO("Control distribution matrix inverse:\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
        control_distribution_matrix_inv_(0,0), control_distribution_matrix_inv_(0,1), control_distribution_matrix_inv_(0,2), control_distribution_matrix_inv_(0,3),
        control_distribution_matrix_inv_(1,0), control_distribution_matrix_inv_(1,1), control_distribution_matrix_inv_(1,2), control_distribution_matrix_inv_(1,3),
        control_distribution_matrix_inv_(2,0), control_distribution_matrix_inv_(2,1), control_distribution_matrix_inv_(2,2), control_distribution_matrix_inv_(2,3),
        control_distribution_matrix_inv_(3,0), control_distribution_matrix_inv_(3,1), control_distribution_matrix_inv_(3,2), control_distribution_matrix_inv_(3,3));

    // PID控制器初始化
    pid_init();

    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);
    reset_publisher = nh->advertise<std_msgs::Empty>("/airsim_node/reset_cmd", 1);

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    odom_suber = nh->subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/drone_state", 1, std::bind(&PwmPidController::pose_cb, this, std::placeholders::_1));//imu与gps数据融合后的位姿数据
    if (target_type_ == "position") {
        target_point_suber_ = nh->subscribe<geometry_msgs::PointStamped>("/drone_0_planning/waypoint_cmd", 1, std::bind(&PwmPidController::target_point_cb, this, std::placeholders::_1));//目标点数据
    } else if (target_type_ == "pose") {
        target_pose_suber_ = nh->subscribe<quadrotor_msgs::PositionCommand>("/drone_0_planning/pos_cmd", 1, std::bind(&PwmPidController::target_pose_cb, this, std::placeholders::_1)); // 目标位姿数据，包含位置和速度和加速度和航向角命令
    } else {
        ROS_ERROR("Invalid target_type parameter: %s. Must be 'position' or 'pose'.\n", target_type_.c_str());
        ros::shutdown();
        return;
    }

    takeoff.request.waitOnLastTask = 1;
    land.request.waitOnLastTask = 1;
    //通过三个服务可以调用模拟器中的无人机起飞和降落命令和重置命令
    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/land");
    reset_client = nh->serviceClient<airsim_ros::Reset>("/airsim_node/reset");

    // 创建处理起降/复位/PID热更新的定时器
    command_timer_ = nh->createTimer(ros::Duration(0.1), std::bind(&PwmPidController::timerCallback, this, std::placeholders::_1));

    startControlThread();

    TerminalGuard terminal_guard;
    g_key_control_enabled = terminal_guard.ok;
    g_keyboard_fd = terminal_guard.fd;
    if (g_key_control_enabled) {
        print_keyboard_help();
    } else {
        ROS_WARN("Keyboard control disabled: failed to configure terminal (/dev/tty or stdin).\n");
    }

    ros::Rate loop_rate(200);
    bool need_exit = false;
    while(ros::ok()){
        ros::spinOnce();
        process_key_input(need_exit);
        if (need_exit) {
            ROS_INFO("Exit requested by keyboard (Q).\n");
            break;
        }
        loop_rate.sleep();
    }

    stopControlThread();
}

PwmPidController::~PwmPidController()
{
    stopControlThread();
}

void PwmPidController::pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    DroneState state;
    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Vector3d velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d eulerAngle = quatToEuler(q); // 注意这里的顺序是z-y-x，对应航向角-俯仰角-滚转角
    Eigen::Vector3d eulerAngle_velocity(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    if (is_cout) {
        ROS_INFO("Get pose data. time: %f, eulerangle: %f, %f, %f,eulerangle_velocity: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
            eulerAngle[0], eulerAngle[1], eulerAngle[2], eulerAngle_velocity[0], eulerAngle_velocity[1], eulerAngle_velocity[2]);
    }
    state.position = position;
    state.velocity = velocity;
    state.orientation = q;
    state.eulerAngle_velocity = eulerAngle_velocity;
    state.roll = eulerAngle[2];
    state.pitch = eulerAngle[1];
    state.yaw = eulerAngle[0];

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_drone_state_ = state;
        latest_state_stamp_ = msg->header.stamp;
        latest_state_recv_time_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    }
    state_received_.store(true, std::memory_order_release);
}

void PwmPidController::target_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    Eigen::Vector3d target_position(msg->point.x, -msg->point.y, -msg->point.z + 0.3); // 注意坐标系转换，目标点发布在右手坐标系下，且z轴朝下，需要转换到左手坐标系下并加上0.3m的高度补偿
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_position_ = target_position;
    }
    ROS_INFO("Get target point. time: %f, position: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        target_position[0], target_position[1], target_position[2]);
}

void PwmPidController::target_pose_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    // 获取目标位置
    Eigen::Vector3d target_position(msg->position.x, - msg->position.y, - msg->position.z + 0.3);
    // 获取目标速度
    Eigen::Vector3d target_velocity(msg->velocity.x, - msg->velocity.y, - msg->velocity.z);
    // 根据超前时间预测目标位置
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_position_ = target_position + ahead_time_sec_ * target_velocity; // 简单的线性预测
    }
    ROS_INFO("Get target pose. time: %f, position: %f, %f, %f, velocity: %f, %f, %f, yaw: %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
            target_position[0], target_position[1], target_position[2],
            target_velocity[0], target_velocity[1], target_velocity[2],
            target_yaw);
}

void PwmPidController::pid_init()
{
    // 位置PID初始化 -- 输入为位置误差，输出为期望速度
    // x，y水平位置的PID参数，都采用水平通道算法，且为对称，可以相同参数
    pid_x_.setGains(0.4, 0.0, 0.0, -20.0, 20.0); // 最后两个参数为输出限幅
    pid_y_.setGains(0.4, 0.0, 0.0, -20.0, 20.0);
    // z轴位置的PID参数需根据无人机的升力特性进行调整，通常需要更大的比例增益和微分增益来应对重力和快速变化的高度误差
    pid_z_.setGains(3.0, 0.05, 2.0, -20.0, 20.0);

    // 速度PID初始化 -- 输入为速度误差，x，y输出为期望姿态角度，z输出为期望升力
    // x，y速度的PID参数，主要用于位置控制中的速度环，可以适当增加微分增益以提高响应速度
    pid_v_x_.setGains(2.0, 0.0, 3.0, -8.0, 8.0);
    pid_v_y_.setGains(2.0, 0.0, 3.0, -8.0, 8.0);
    // z轴速度的PID参数，需根据无人机的升力特性进行调整，通常需要更大的比例增益和微分增益来应对重力和快速变化的高度误差
    pid_v_z_.setGains(1.0, 0.0, 0.1, -10.0, 10.0);

    // 姿态PID初始化 -- 输入为姿态误差，输出为期望角速度
    // 滚转角和俯仰角的PID参数，需根据无人机的滚转和俯仰特性进行调整，可相同
    pid_roll_.setGains(0.6, 0.0, 0.0, -30.0/57.3, 30.0/57.3);
    pid_pitch_.setGains(1.0, 0.0, 0.0, -30.0/57.3, 30.0/57.3);
    // 航向角的PID参数，需根据无人机的航向特性进行调整，通常需要较大的比例增益和微分增益来应对快速变化的航向误差
    pid_yaw_.setGains(1.5, 0.0, 0.0, -100.0/57.3, 100.0/57.3);

    // 角速度PID初始化 -- 输入为角速度误差，输出为对应轴所需力矩
    pid_v_roll_.setGains(0.001, 0.0, 0.001, -1.0, 1.0);
    pid_v_pitch_.setGains(0.001, 0.0, 0.001, -1.0, 1.0);
    pid_v_yaw_.setGains(0.001, 0.0, 0.000, -1.0, 1.0);
    feedforward_roll_gain_ = 0.002;
    feedforward_pitch_gain_ = 0.002;
    feedforward_yaw_gain_ = 0.0005;
    feedforward_z_gain_ = 0.0;
}

void PwmPidController::position_control(const DroneState& current_state, const Eigen::Vector3d& target_position)
{
    Eigen::Vector3d position_error = target_position - current_state.position;
    Eigen::Vector3d velocity_command;
    velocity_command.x() = pid_x_.updateWithLimit(position_error.x());
    velocity_command.y() = pid_y_.updateWithLimit(position_error.y());
    velocity_command.z() = pid_z_.updateWithLimit(position_error.z());

    // 这里的速度命令是期望速度，需要通过速度PID转换为姿态角度和升力命令
    Eigen::Vector3d velocity_error = velocity_command - current_state.velocity;
    double ddx = pid_v_x_.updateWithLimit(velocity_error.x());
    double ddy = pid_v_y_.updateWithLimit(velocity_error.y());
    double Fz_command = pid_v_z_.updateWithLimit(velocity_error.z());


    // 水平通道控制器
    Eigen::Vector2d horizontal_acc_command(ddx, ddy);
    Eigen::Matrix2d A_yaw_local;
    A_yaw_local << std::sin(current_state.yaw),  std::cos(current_state.yaw),
                  -std::cos(current_state.yaw),  std::sin(current_state.yaw);
    Eigen::Matrix2d A_yaw_inv = A_yaw_local.inverse();
    Eigen::Vector2d angle_command = - 1.0 / drone_param_.g * A_yaw_inv * horizontal_acc_command; // 期望roll和pitch角度，单位为弧度

    angle_command.x() = limit(angle_command.x(), -15.0/57.3, 15.0/57.3);
    angle_command.y() = limit(angle_command.y(), -15.0/57.3, 15.0/57.3);

    target_eulerAngle_.x() = angle_command.x();
    target_eulerAngle_.y() = angle_command.y();

    target_throttle_ = -Fz_command + feedforward_z_gain_;
    //target_throttle_ = 0.054269;
}

void PwmPidController::pose_control(const DroneState& current_state, const Eigen::Vector3d& target_eulerAngle)
{
    Eigen::Vector3d eulerAngle_error = target_eulerAngle - Eigen::Vector3d(current_state.roll, current_state.pitch, current_state.yaw);
    double roll_v_command = pid_roll_.updateWithLimit(eulerAngle_error.x());
    double pitch_v_command = pid_pitch_.updateWithLimit(eulerAngle_error.y());
    double yaw_v_command = pid_yaw_.updateWithLimit(eulerAngle_error.z());

    // 这里的角速度命令是期望角速度，需要通过角速度PID转换为对应轴所需力矩命令
    // 在角速度处新加入前馈控制
    Eigen::Vector3d eulerAngle_v_error = Eigen::Vector3d(roll_v_command, pitch_v_command, yaw_v_command) - current_state.eulerAngle_velocity;
    double feedforward_roll = feedforward_roll_gain_ * roll_v_command;
    double feedforward_pitch = feedforward_pitch_gain_ * pitch_v_command;
    double feedforward_yaw = feedforward_yaw_gain_ * yaw_v_command;
    double Tx_command = pid_v_roll_.updateWithLimitAndFeedforward(eulerAngle_v_error.x(), feedforward_roll);
    double Ty_command = pid_v_pitch_.updateWithLimitAndFeedforward(eulerAngle_v_error.y(), feedforward_pitch);
    double Tz_command = pid_v_yaw_.updateWithLimitAndFeedforward(eulerAngle_v_error.z(), feedforward_yaw);
    target_T.x() = Tx_command;
    //target_T.x() = 0.0;
    target_T.y() = Ty_command;
    //target_T.y() = 0.0;
    target_T.z() = Tz_command;
    //target_T.z() = 0.0;
}

void PwmPidController::publish_pwm_command(const ros::Time& stamp)
{
    // pwm范围为0～1.0
    // double k = 1.0;
    // //target_throttle_ = 0.5; // 测试
    // double pwm1 = (target_throttle_ - target_T.x() + target_T.y() + target_T.z()) * k;
    // double pwm2 = (target_throttle_ + target_T.x() - target_T.y() + target_T.z()) * k;
    // double pwm3 = (target_throttle_ + target_T.x() + target_T.y() - target_T.z()) * k;
    // double pwm4 = (target_throttle_ - target_T.x() - target_T.y() - target_T.z()) * k;
    Eigen::Vector4d w2 = control_distribution_matrix_inv_ * Eigen::Vector4d(target_throttle_, target_T.x(), target_T.y(), target_T.z());
    //double max_w2 = drone_param_.max_w * drone_param_.max_w;
    w2.x() = limit(w2.x(), 0.0, drone_param_.max_w);
    w2.y() = limit(w2.y(), 0.0, drone_param_.max_w);
    w2.z() = limit(w2.z(), 0.0, drone_param_.max_w);
    w2.w() = limit(w2.w(), 0.0, drone_param_.max_w);
    double sqrt_max_w = sqrt(drone_param_.max_w);
    double pwm1 = sqrt(w2.x()) / sqrt_max_w;
    double pwm2 = sqrt(w2.y()) / sqrt_max_w;
    double pwm3 = sqrt(w2.z()) / sqrt_max_w;
    double pwm4 = sqrt(w2.w()) / sqrt_max_w;
    pwm1 = limit(pwm1, 0.0, 1.0);
    pwm2 = limit(pwm2, 0.0, 1.0);
    pwm3 = limit(pwm3, 0.0, 1.0);
    pwm4 = limit(pwm4, 0.0, 1.0);
    if(is_pwm_published) 
    {
        airsim_ros::RotorPWM pwm_cmd;
        pwm_cmd.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
        pwm_cmd.header.frame_id = "imu_time";
        pwm_cmd.rotorPWM0 = pwm1; // 右前 顺时针
        pwm_cmd.rotorPWM1 = pwm2; // 左后 顺时针
        pwm_cmd.rotorPWM2 = pwm3; // 左前 逆时针
        pwm_cmd.rotorPWM3 = pwm4; // 右后 逆时针
        pwm_publisher.publish(pwm_cmd);
        if(is_cout) {
            ROS_INFO("publishing-------------------------------------------------");
        }
    }
    else 
    {
        airsim_ros::RotorPWM pwm_cmd;
        pwm_cmd.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
        pwm_cmd.header.frame_id = "imu_time";
        pwm_cmd.rotorPWM0 = 0.0; // 右前 顺时针
        pwm_cmd.rotorPWM1 = 0.0; // 左后 顺时针
        pwm_cmd.rotorPWM2 = 0.0; // 左前 逆时针
        pwm_cmd.rotorPWM3 = 0.0; // 右后 逆时针
        pwm_publisher.publish(pwm_cmd);
    }
    if(is_cout) {
        ROS_INFO("target_throttle_: %f, target_T: %f, %f, %f\n", target_throttle_, target_T.x(), target_T.y(), target_T.z());
        ROS_INFO("w2: %f, %f, %f, %f\n", w2.x(), w2.y(), w2.z(), w2.w());
        ROS_INFO("Publish pwm command: %f, %f, %f, %f\n", pwm1, pwm2, pwm3, pwm4);
    }
}

void PwmPidController::timerCallback(const ros::TimerEvent& event)
{
    if(is_takeoff) {
        if (takeoff_client.call(takeoff)) {
            ROS_INFO("Takeoff command sent successfully.\n");
        } else {
            ROS_ERROR("Failed to call takeoff service.\n");
        }
        is_takeoff = false;
    }
    if(is_land) {
        if (land_client.call(land)) {
            ROS_INFO("Land command sent successfully.\n");
        } else {
            ROS_ERROR("Failed to call land service.\n");
        }
        is_land = false;
    }
    if(is_reset) {
        if (reset_client.call(reset)) {
            ROS_INFO("Reset command sent successfully.\n");
            std_msgs::Empty reset_msg;
            reset_publisher.publish(reset_msg);
        } else {
            ROS_ERROR("Failed to call reset service.\n");
        }
        is_reset = false;
    } 
    if(teminal_pid_set.is_pid_changed) {
        std::lock_guard<std::mutex> lock(pid_mutex_);
        switch (teminal_pid_set.pid_controller_index) {
            case 1:
                //pid_roll_.setGains(teminal_pid_set.kp, pid_roll_.ki_, teminal_pid_set.kd, pid_roll_.min_output_, pid_roll_.max_output_);
                pid_y_.setGains(teminal_pid_set.kp, pid_y_.ki_, teminal_pid_set.kd, pid_y_.min_output_, pid_y_.max_output_);
                break;
            case 2:
                //pid_pitch_.setGains(teminal_pid_set.kp, pid_pitch_.ki_, teminal_pid_set.kd, pid_pitch_.min_output_, pid_pitch_.max_output_);
                pid_x_.setGains(teminal_pid_set.kp, pid_x_.ki_, teminal_pid_set.kd, pid_x_.min_output_, pid_x_.max_output_);
                break;
            case 3:
                pid_z_.setGains(teminal_pid_set.kp, pid_z_.ki_, teminal_pid_set.kd, pid_z_.min_output_, pid_z_.max_output_);
                break;
            case 4:
                //pid_v_roll_.setGains(teminal_pid_set.kp, pid_v_roll_.ki_, teminal_pid_set.kd, pid_v_roll_.min_output_, pid_v_roll_.max_output_);
                pid_v_y_.setGains(teminal_pid_set.kp, pid_v_y_.ki_, teminal_pid_set.kd, pid_v_y_.min_output_, pid_v_y_.max_output_);
                break;
            case 5:
                //pid_v_pitch_.setGains(teminal_pid_set.kp, pid_v_pitch_.ki_, teminal_pid_set.kd, pid_v_pitch_.min_output_, pid_v_pitch_.max_output_);
                pid_v_x_.setGains(teminal_pid_set.kp, pid_v_x_.ki_, teminal_pid_set.kd, pid_v_x_.min_output_, pid_v_x_.max_output_);
                break;
            case 6:
                pid_v_z_.setGains(teminal_pid_set.kp, pid_v_z_.ki_, teminal_pid_set.kd, pid_v_z_.min_output_, pid_v_z_.max_output_);
                break;
            default:
                ROS_WARN("Invalid PID controller index: %d\n", teminal_pid_set.pid_controller_index);
        }
        ROS_INFO("Update PID gains: Kp = %f, Ki = %f, Kd = %f", teminal_pid_set.kp, teminal_pid_set.ki, teminal_pid_set.kd);
        teminal_pid_set.is_pid_changed = false;
    }
}

void PwmPidController::startControlThread()
{
    if (control_loop_running_.exchange(true)) {
        return;
    }

    control_thread_ = std::thread(&PwmPidController::controlLoop, this);
}

void PwmPidController::stopControlThread()
{
    if (!control_loop_running_.exchange(false)) {
        return;
    }

    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}

void PwmPidController::controlLoop()
{
    if (control_loop_hz_ <= 0.0) {
        control_loop_hz_ = 100.0;
    }

    const auto period = std::chrono::duration<double>(1.0 / control_loop_hz_);
    auto next_tick = std::chrono::steady_clock::now();

    while (control_loop_running_.load(std::memory_order_acquire) && ros::ok()) {
        DroneState state;
        ros::Time msg_stamp;
        ros::Time recv_time;
        bool has_fresh_state = false;
        double recv_age_sec = 0.0;
        double msg_age_sec = 0.0;
        const ros::Time now = ros::Time::now();

        if (state_received_.load(std::memory_order_acquire)) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state = latest_drone_state_;
                msg_stamp = latest_state_stamp_;
                recv_time = latest_state_recv_time_;
            }

            recv_age_sec = recv_time.isZero() ? std::numeric_limits<double>::infinity() : (now - recv_time).toSec();
            has_fresh_state = (recv_age_sec >= 0.0 && recv_age_sec <= max_state_age_sec_);
            //std::cout << "recv_age_sec: " << recv_age_sec << ", max_state_age_sec_: " << max_state_age_sec_ << std::endl;

            if (!msg_stamp.isZero()) {
                msg_age_sec = (now - msg_stamp).toSec();
            }

            if (!has_fresh_state) {
                ROS_WARN_THROTTLE(1.0,
                    "State stale by receive time: recv_age=%.3f sec, msg_age=%.3f sec, threshold=%.3f sec",
                    recv_age_sec, msg_age_sec, max_state_age_sec_);
            }
        }

        if (has_fresh_state && target_position_.allFinite() && target_position_.z() != 0.0) {
            std::lock_guard<std::mutex> lock(pid_mutex_);
            current_drone_state_ = state;

            // position_control(current_drone_state_, Eigen::Vector3d(target_x, target_y, target_z));
            position_control(current_drone_state_, target_position_);
            target_eulerAngle_.z() = target_yaw;
            pose_control(current_drone_state_, target_eulerAngle_);
            publish_pwm_command(msg_stamp);
        }

        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next_tick);

        // 控制线程发生过载时重置节拍，避免长期漂移和追赶抖动。
        auto now_tick = std::chrono::steady_clock::now();
        if (now_tick > next_tick + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period)) {
            next_tick = now_tick;
        }
    }
}

double PwmPidController::limit(double value, double min_value, double max_value)
{
    if (value > max_value)
        return max_value;
    else if (value < min_value)
        return min_value;
    else
        return value;
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


#endif
