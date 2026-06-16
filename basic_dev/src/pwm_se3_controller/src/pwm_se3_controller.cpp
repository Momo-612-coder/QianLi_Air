#include "pwm_se3_controller.hpp"

#include <algorithm>
#include <limits>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "pwm_se3_controller"); // 初始化ros 节点，命名为 pwm_se3_controller
    ros::NodeHandle n; // 创建node控制句柄
    PwmSe3Controller go(&n);
    return 0;
}

PwmSe3Controller::PwmSe3Controller(ros::NodeHandle *nh)
{  
    nh->param("control_loop_hz", control_loop_hz_, 100.0);
    nh->param("max_state_age_sec", max_state_age_sec_, 0.2);
    nh->param("target_type", target_type_, std::string("pose")); // 可选 "position" 或 "pose"
    nh->param("ahead_time_sec", ahead_time_sec_, 0.5); // 预测提前量，单位秒

    // 初始化无人机参数
    drone_param_.mass = 0.9;
    drone_param_.g = 9.81;
    drone_param_.R = 0.18;
    drone_param_.I = Eigen::Vector3d(0.0046890742, 0.0069312, 0.010421166);
    drone_param_.ct = 0.000367717; // 推力系数，单位为N/(r/s)^2
    drone_param_.cm = 4.888486266072161e-06; // 扭矩系数，单位为Nm/(r/s)^2
    drone_param_.c = 0.01; // 偏航系数，经验值
    drone_param_.max_n = 11079.03; // 电机最大转速, r/min
    drone_param_.max_w = drone_param_.max_n * 2.0 * M_PI / 60.0; // 电机最大角速度, rad/s
    drone_param_.max_throttle = drone_param_.ct * (drone_param_.max_n / 60.0) * (drone_param_.max_n / 60.0);
    drone_param_.max_torque = drone_param_.cm * (drone_param_.max_n / 60.0) * (drone_param_.max_n / 60.0);
    double ct = drone_param_.ct / (2 * M_PI) / (2 * M_PI); // 将推力系数转换为以角速度平方为输入的形式，单位为N/(rad/s)^2
    double cm = drone_param_.cm / (2 * M_PI) / (2 * M_PI); // 将扭矩系数转换为以角速度平方为输入的形式，单位为Nm/(rad/s)^2
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

    ROS_INFO("Max throttle: %f, Max torque: %f\n", drone_param_.max_throttle, drone_param_.max_torque);

    target_position_ = Eigen::Vector3d(0.0, 0.0, -1.0);
    target_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    target_acceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    target_eulerAngle_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    target_yaw_ = 0.0;
    target_throttle_ = 0.0;
    target_T = Eigen::Vector3d(0.0, 0.0, 0.0);

    // 初始化SE3控制器
    SE3Controller_init();

    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);
    reset_publisher = nh->advertise<std_msgs::Empty>("/airsim_node/reset_cmd", 1);
    control_start_publisher = nh->advertise<std_msgs::Header>("/control_start", 1);
    start_state_suber_ = nh->subscribe<std_msgs::Header>("fusion_start", 1, std::bind(&PwmSe3Controller::start_state_cb, this, std::placeholders::_1));


    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    odom_suber = nh->subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/drone_state", 1, std::bind(&PwmSe3Controller::pose_cb, this, std::placeholders::_1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());//imu与gps数据融合后的位姿数据
    if (target_type_ == "position") {
        target_point_suber_ = nh->subscribe<geometry_msgs::PointStamped>(
            "/drone_0_planning/waypoint_cmd", 1, std::bind(&PwmSe3Controller::target_point_cb, this, std::placeholders::_1),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());//目标点数据
    } else if (target_type_ == "pose") {
        //target_pose_suber_ = nh->subscribe<quadrotor_msgs::PositionCommand>("/drone_0_planning/pos_cmd", 1, std::bind(&PwmSe3Controller::target_pose_cb, this, std::placeholders::_1)); // 目标位姿数据，包含位置和速度和加速度和航向角命令
        target_pose_suber_ = nh->subscribe<quadrotor_msgs::PositionCommand>(
            "/pose_cmd", 1, std::bind(&PwmSe3Controller::target_pose_cb, this, std::placeholders::_1),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()); // 目标位姿数据，包含位置和速度和加速度和航向角命令
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
    command_timer_ = nh->createTimer(ros::Duration(0.1), std::bind(&PwmSe3Controller::timerCallback, this, std::placeholders::_1));

    TerminalGuard terminal_guard;
    g_key_control_enabled = terminal_guard.ok;
    g_keyboard_fd = terminal_guard.fd;
    if (g_key_control_enabled) {
        print_keyboard_help();
    } else {
        ROS_WARN("Keyboard control disabled: failed to configure terminal (/dev/tty or stdin).\n");
    }

    //startControlThread();

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

PwmSe3Controller::~PwmSe3Controller()
{
    stopControlThread();
}

void PwmSe3Controller::pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    DroneState state;
    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Vector3d velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d eulerAngle = quatToEuler(q); // 注意这里的顺序是z-y-x，对应航向角-俯仰角-滚转角
    double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())); // 使用 std::atan2 稳定提取偏航角，避免 Eigen eulerAngles 带来边界突变问题
    Eigen::Vector3d eulerAngle_velocity(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    // if (is_cout) {
    //     ROS_INFO("Get pose data. time: %f, eulerangle: %f, %f, %f,eulerangle_velocity: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
    //         eulerAngle[0], eulerAngle[1], eulerAngle[2], eulerAngle_velocity[0], eulerAngle_velocity[1], eulerAngle_velocity[2]);
    // }
    state.position = position;
    state.velocity = velocity;
    state.orientation = q;
    state.eulerAngle_velocity = eulerAngle_velocity;
    state.roll = eulerAngle[2];
    state.pitch = eulerAngle[1];
    state.yaw = yaw;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_drone_state_ = state;
        latest_state_stamp_ = msg->header.stamp;
        latest_state_recv_time_ = ros::Time::now();
    }
    state_received_.store(true, std::memory_order_release);
}

void PwmSe3Controller::target_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    Eigen::Vector3d target_position(msg->point.x, -msg->point.y, -msg->point.z); // 注意坐标系转换，目标点发布在右手坐标系下，且z轴朝下，需要转换到左手坐标系下并加上0.3m的高度补偿
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_position_ = target_position;
    }
    ROS_INFO("Get target point. time: %f, position: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        target_position[0], target_position[1], target_position[2]);
}

void PwmSe3Controller::target_pose_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    // 获取目标位置
    Eigen::Vector3d target_position(msg->position.x, msg->position.y, msg->position.z);
    // 获取目标速度
    Eigen::Vector3d target_velocity(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    // 获取目标加速度
    Eigen::Vector3d target_acceleration(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    // 获取目标航向角
    double target_yaw = msg->yaw;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_position_ = target_position;
        target_velocity_ = target_velocity;
        target_acceleration_ = target_acceleration;
        target_eulerAngle_.z() = target_yaw;
        target_yaw_ = target_yaw;
    }

    // ROS_INFO("Get target pose. time: %f, position: %f, %f, %f, velocity: %f, %f, %f, acceleration: %f, %f, %f, yaw: %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
    //     target_position[0], target_position[1], target_position[2],
    //     target_velocity[0], target_velocity[1], target_velocity[2],
    //     target_acceleration[0], target_acceleration[1], target_acceleration[2],
    //     target_yaw);
    ROS_INFO_THROTTLE(1.0, "Get target pose. time: %f, position: %f, %f, %f, velocity: %f, %f, %f, acceleration: %f, %f, %f, yaw: %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        target_position[0], target_position[1], target_position[2],
        target_velocity[0], target_velocity[1], target_velocity[2],
        target_acceleration[0], target_acceleration[1], target_acceleration[2],
        target_yaw);
}

void PwmSe3Controller::start_state_cb(const std_msgs::Header::ConstPtr& msg)
{
    const ros::Time now = msg->stamp.isZero() ? ros::Time::now() : msg->stamp;
    if (!is_started_) {
        start_time_ = now;
        ROS_INFO("Received start state message. Starting control in 2 seconds...\n");
    }

    // 如果已经开始控制。
    if (is_started_ && (now - start_time_).toSec() > 2.0 && !is_control_thread_running_) {
        startControlThread();
        is_control_thread_running_ = true;
        ROS_INFO("Control thread started.\n");
        // 输出时间间隔
        double interval = (now - start_time_).toSec();
        ROS_INFO("Time since start state message: %f seconds.\n", interval);
    }

    is_started_ = true;
}

void PwmSe3Controller::SE3Controller_init()
{
    Eigen::Vector3d position_gain(2.0, 3.5, 4.5);
    Eigen::Vector3d velocity_gain(1.5, 2.5, 2.5);
    Eigen::Vector3d eulerAngle_gain(3.0, 3.0, 3.0);
    Eigen::Vector3d eulerAngle_velocity_gain(2.0, 2.0, 2.0);
    se3_controller_.setParams(drone_param_.mass, 1.0/control_loop_hz_, drone_param_.I, position_gain, velocity_gain, eulerAngle_gain, eulerAngle_velocity_gain);
}

void PwmSe3Controller::publish_pwm_command(const ros::Time& stamp)
{
    // pwm范围为0～1.0
    // double k = 1.0;
    // //target_throttle_ = 0.5; // 测试
    // double pwm1 = (target_throttle_ - target_T.x() + target_T.y() + target_T.z()) * k;
    // double pwm2 = (target_throttle_ + target_T.x() - target_T.y() + target_T.z()) * k;
    // double pwm3 = (target_throttle_ + target_T.x() + target_T.y() - target_T.z()) * k;
    // double pwm4 = (target_throttle_ - target_T.x() - target_T.y() - target_T.z()) * k;
    // Eigen::Vector4d w2 = control_distribution_matrix_inv_ * Eigen::Vector4d(target_throttle_, target_T.x(), target_T.y(), target_T.z());
    // double max_w2 = drone_param_.max_w * drone_param_.max_w;
    // w2.x() = limit(w2.x(), 0.0, max_w2);
    // w2.y() = limit(w2.y(), 0.0, max_w2);
    // w2.z() = limit(w2.z(), 0.0, max_w2);
    // w2.w() = limit(w2.w(), 0.0, max_w2);
    // double sqrt_max_w = sqrt(max_w2);
    // double pwm1 = sqrt(w2.x()) / sqrt_max_w;
    // double pwm2 = sqrt(w2.y()) / sqrt_max_w;
    // double pwm3 = sqrt(w2.z()) / sqrt_max_w;
    // double pwm4 = sqrt(w2.w()) / sqrt_max_w;

    double f = target_throttle_ / 4; // 每个电机分担的平均推力，单位为N
    double M_x = target_T.x() / 4; // 每个电机分担的绕x轴的力矩，单位为Nm
    double M_y = target_T.y() / 4; // 每个电机分担的绕y轴的力矩，单位为Nm
    double M_z = target_T.z() / 4; // 每个电机分担的绕z轴的力矩，单位为Nm
    f = limit(f, 0.0, drone_param_.max_throttle);
    M_x = limit(M_x, -drone_param_.max_torque / 4, drone_param_.max_torque / 4); // 限制力矩在最大值范围内
    M_y = limit(M_y, -drone_param_.max_torque / 4, drone_param_.max_torque / 4); // 限制力矩在最大值范围内
    M_z = limit(M_z, -drone_param_.max_torque / 4, drone_param_.max_torque / 4); // 限制力矩在最大值范围内
    double f1 = f - M_x / drone_param_.R + M_y / drone_param_.R + M_z / drone_param_.R; // 右前
    double f2 = f + M_x / drone_param_.R - M_y / drone_param_.R + M_z / drone_param_.R; // 左后
    double f3 = f + M_x / drone_param_.R + M_y / drone_param_.R - M_z / drone_param_.R; // 左前
    double f4 = f - M_x / drone_param_.R - M_y / drone_param_.R - M_z / drone_param_.R; // 右后
    f1 = limit(f1, 0.0, drone_param_.max_throttle);
    f2 = limit(f2, 0.0, drone_param_.max_throttle);
    f3 = limit(f3, 0.0, drone_param_.max_throttle);
    f4 = limit(f4, 0.0, drone_param_.max_throttle);

    double pwm1 = f1 / drone_param_.max_throttle;
    double pwm2 = f2 / drone_param_.max_throttle;
    double pwm3 = f3 / drone_param_.max_throttle;
    double pwm4 = f4 / drone_param_.max_throttle;

    pwm1 = limit(pwm1, 0.0, 1.0);
    pwm2 = limit(pwm2, 0.0, 1.0);
    pwm3 = limit(pwm3, 0.0, 1.0);
    pwm4 = limit(pwm4, 0.0, 1.0);
    airsim_ros::RotorPWM pwm_cmd;
    pwm_cmd.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    pwm_cmd.header.frame_id = "imu_time";

    if(is_pwm_published) 
    {
        pwm_cmd.rotorPWM0 = pwm1; // 右前 顺时针
        pwm_cmd.rotorPWM1 = pwm2; // 左后 顺时针
        pwm_cmd.rotorPWM2 = pwm3; // 左前 逆时针
        pwm_cmd.rotorPWM3 = pwm4; // 右后 逆时针
        if(is_cout) {
            ROS_INFO("publishing-------------------------------------------------");
        }
    }
    else 
    {
        pwm_cmd.rotorPWM0 = 0.0; // 右前 顺时针
        pwm_cmd.rotorPWM1 = 0.0; // 左后 顺时针
        pwm_cmd.rotorPWM2 = 0.0; // 左前 逆时针
        pwm_cmd.rotorPWM3 = 0.0; // 右后 逆时针
    }
    pwm_publisher.publish(pwm_cmd);
    ROS_INFO_THROTTLE(1.0, "Publish PWM command. time: %f, pwm: %f, %f, %f, %f\n", pwm_cmd.header.stamp.sec + pwm_cmd.header.stamp.nsec*1e-9,
        pwm_cmd.rotorPWM0, pwm_cmd.rotorPWM1, pwm_cmd.rotorPWM2, pwm_cmd.rotorPWM3);
    ROS_INFO_THROTTLE(1.0, "target_throttle_, M_x, M_y, M_z: %f, %f, %f, %f\n", target_throttle_, target_T.x(), target_T.y(), target_T.z());
}

void PwmSe3Controller::timerCallback(const ros::TimerEvent& event)
{
    (void)event;

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
        Eigen::Vector3d gain;
        switch (teminal_pid_set.pid_controller_index)
        {
            case 1: 
                gain = se3_controller_.position_gain_;
                gain.y() = teminal_pid_set.kp;
                se3_controller_.setPositionGain(gain);
                break;
            case 2:
                gain = se3_controller_.velocity_gain_;
                gain.y() = teminal_pid_set.kp;
                se3_controller_.setVelocityGain(gain);
                break;
            case 3:
                gain = se3_controller_.position_gain_;
                gain.z() = teminal_pid_set.kp;
                se3_controller_.setPositionGain(gain);
                break;
            case 4:
                gain = se3_controller_.eulerAngle_gain_;
                gain.x() = teminal_pid_set.kp;
                se3_controller_.setEulerAngleGain(gain);
                break;
            case 5:
                gain = se3_controller_.eulerAngle_velocity_gain_;
                gain.x() = teminal_pid_set.kp;
                se3_controller_.setEulerAngleVelocityGain(gain);
                break;
            case 6:
                gain = se3_controller_.velocity_gain_;
                gain.z() = teminal_pid_set.kp;
                se3_controller_.setVelocityGain(gain);
                break;
            default:
                break;
        }
        teminal_pid_set.is_pid_changed = false;
    }
    if (is_manual_control == true && last_manual_control_state == false)
    {
        target_x = target_position_.x();
        target_y = target_position_.y();
        target_z = target_position_.z();
        target_yaw = target_yaw_;
        last_manual_control_state = true;
    }
    else if (is_manual_control == false && last_manual_control_state == true)
    {
        last_manual_control_state = false;
    }
}

void PwmSe3Controller::startControlThread()
{
    if (control_loop_running_.exchange(true)) {
        return;
    }

    control_thread_ = std::thread(&PwmSe3Controller::controlLoop, this);
}

void PwmSe3Controller::stopControlThread()
{
    if (!control_loop_running_.exchange(false)) {
        return;
    }

    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}

void PwmSe3Controller::controlLoop()
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

        const double dt = 1.0 / control_loop_hz_;
        const double max_acc = 6.0;      // m/s^2，刹车强度，先试 3~5
        const double max_speed = 15.0;   // m/s，按你的飞行上限设置
        const double max_z_speed = 4.0;
        const double max_z_acc_ff = 2.0;

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

        if (has_fresh_state && target_position_.allFinite() && target_position_.z() != 0.0 && std::isfinite(target_yaw_) && target_velocity_.allFinite() && target_acceleration_.allFinite()) {
            std::lock_guard<std::mutex> lock(pid_mutex_);
            current_drone_state_ = state;

            // 发布控制开始消息，通知外部系统控制器已经开始工作，可以开始记录数据等后续操作。
            std_msgs::Header control_start_msg;
            control_start_msg.stamp = msg_stamp.isZero() ? ros::Time::now() : msg_stamp;
            control_start_msg.frame_id = "imu_time";
            control_start_publisher.publish(control_start_msg);

            // 手动控制
            if (is_manual_control)
            {
                se3_controller_.positionControl(current_drone_state_, Eigen::Vector3d(target_x, target_y, target_z), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
                se3_controller_.eulerAngleControl(current_drone_state_, target_yaw);
            }
            else
            {
                if (!has_smooth_target_) {
                    smooth_target_position_ = target_position_;
                    smooth_target_velocity_ = current_drone_state_.velocity;
                    smooth_target_acceleration_.setZero();
                    has_smooth_target_ = true;
                }

                Eigen::Vector3d desired_velocity = target_velocity_;

                if (desired_velocity.norm() > max_speed) {
                    desired_velocity = desired_velocity.normalized() * max_speed;
                }

                Eigen::Vector3d dv = desired_velocity - smooth_target_velocity_;
                const double max_dv = max_acc * dt;

                if (dv.norm() > max_dv) {
                    dv = dv.normalized() * max_dv;
                }

                smooth_target_velocity_ += dv;
                smooth_target_acceleration_ = target_acceleration_; // 直接使用目标加速度作为feedforward项，后续可以改进为根据速度变化计算得到更合理的加速度feedforward

                // se3_controller_.positionControl(current_drone_state_,
                //                                 smooth_target_position_,
                //                                 smooth_target_velocity_,
                //                                 smooth_target_acceleration_);
                //se3_controller_.positionControl(current_drone_state_, target_position_, smooth_target_velocity_, smooth_target_acceleration_);
                se3_controller_.positionControl(current_drone_state_, target_position_, target_velocity_, target_acceleration_);
                se3_controller_.eulerAngleControl(current_drone_state_, target_yaw_);
            }
            target_throttle_ = - se3_controller_.target_body_thrust_.z();
            target_T = se3_controller_.target_body_torque_;

            // 输出循环一次的时间间隔
            //ROS_INFO("Control loop iteration. dt: %f sec, recv_age: %f sec, msg_age: %f sec\n", dt, recv_age_sec, msg_age_sec);

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

double PwmSe3Controller::limit(double value, double min_value, double max_value)
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
