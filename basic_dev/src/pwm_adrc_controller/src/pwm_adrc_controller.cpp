#include "pwm_adrc_controller.hpp"

#include <cmath>
#include <algorithm>
#include <functional>
#include <iostream>
#include <string>

// 全局变量声明
double key_target_x = 0.0;
double key_target_y = 0.0;
double term_target_yaw = 0.0;
double term_target_z = 0.0; 
double current_yaw = 0.0; 

// ⭐ 姿态模式全局变量
double term_target_roll = 0.0;
double term_target_pitch = 0.0;
bool g_attitude_lock_enabled = false;

// 矩阵式调参初始化
Eigen::Vector3d g_tune_kp_pos(0.6, 0.6, 1.5);//3
Eigen::Vector3d g_tune_wc_vel(10.0, 10.0, 10.0);//4
Eigen::Vector3d g_tune_wp_vel(1.2, 1.2, 3.5);
Eigen::Vector3d g_tune_kp_ang(4.25, 4.25, 2.0);
Eigen::Vector3d g_tune_wc_att(25.0, 25.0, 24.0);
Eigen::Vector3d g_tune_wp_att(12.0, 12.0, 8.0);
Eigen::Vector3d g_base_b0(1.0/0.0046890742, 1.0/0.0069312, 1.0/0.010421166);
Eigen::Vector3d g_tune_b0_scale(1.0, 1.0, 1.0);
Eigen::Vector3d g_tune_b0 = g_base_b0.cwiseProduct(g_tune_b0_scale);
double g_tune_tau = 0.5;
int g_tuning_param_idx = 4; 
int g_tuning_axis_idx = 3;  

bool is_takeoff = false;
bool is_land = false;
bool is_reset = false;
bool is_pwm_published = false; 
bool is_flying = false; 
bool is_cout = false;
bool enable_position_loop = false; // ⭐ 默认启动时为姿态模式
bool g_use_planner_input = true;  // 默认规划器输入，按 T 可切换键盘输入
int g_keyboard_fd = -1;
bool g_key_control_enabled = false;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_adrc_controller"); 
    ros::NodeHandle n; 
    PwmAdrcController go(&n);
    return 0;
}

PwmAdrcController::PwmAdrcController(ros::NodeHandle *nh) {  
    nh->param("control_start_delay_sec", control_start_delay_sec_, 2.0);
    nh->param("max_planner_target_age_sec", max_planner_target_age_sec_, 0.5);
    nh->param("planner_z_offset", planner_z_offset_, 0.0);
    nh->param("use_planner_input", g_use_planner_input, true);
    std::string planner_cmd_topic;
    nh->param<std::string>("planner_cmd_topic", planner_cmd_topic, "/pose_cmd");

    pos_Z1.setZero(); pos_Z2.setZero(); pos_U.setZero();
    att_Z1.setZero(); att_Z2.setZero(); att_U.setZero();
    key_pos_Z1 = 0; key_pos_Z2 = 0; key_pos_U = 0;
    target_position_.setZero(); target_velocity_.setZero();

    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);
    reset_publisher = nh->advertise<std_msgs::Empty>("/airsim_node/reset_cmd", 1);
    control_start_publisher = nh->advertise<std_msgs::Header>("/control_start", 1);
    target_debug_publisher = nh->advertise<nav_msgs::Odometry>("/pwm_adrc_controller/target_debug", 1);

    odom_suber = nh->subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/drone_state", 1, std::bind(&PwmAdrcController::pose_cb, this, std::placeholders::_1));
    target_pose_suber_ = nh->subscribe<quadrotor_msgs::PositionCommand>(
        planner_cmd_topic, 1, std::bind(&PwmAdrcController::target_pose_cb, this, std::placeholders::_1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    start_state_suber_ = nh->subscribe<std_msgs::Header>("fusion_start", 1, std::bind(&PwmAdrcController::start_state_cb, this, std::placeholders::_1));

    takeoff.request.waitOnLastTask = 1; land.request.waitOnLastTask = 1;
    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/land");
    reset_client = nh->serviceClient<airsim_ros::Reset>("/airsim_node/reset");

    command_timer_ = nh->createTimer(ros::Duration(0.1), std::bind(&PwmAdrcController::timerCallback, this, std::placeholders::_1));

    TerminalGuard terminal_guard;
    g_key_control_enabled = terminal_guard.ok;
    g_keyboard_fd = terminal_guard.fd;
    if (g_key_control_enabled) print_keyboard_help();

    ros::Rate loop_rate(100);
    bool need_exit = false;
    while(ros::ok()){
        ros::spinOnce(); 
        process_key_input(need_exit);
        if (need_exit) break;
        loop_rate.sleep();
    }
}

void PwmAdrcController::pose_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    static ros::Time last_stamp = msg->header.stamp;
    double dt = (msg->header.stamp - last_stamp).toSec();
    last_stamp = msg->header.stamp;
    if (dt <= 0.001 || dt > 0.1) dt = 0.01; 

    DroneState state;
    state.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    state.velocity = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    state.eulerAngle_velocity = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    state.orientation = q;
    Eigen::Vector3d euler = quatToEuler(q); 
    state.yaw = euler[0]; state.pitch = euler[1]; state.roll = euler[2];

    current_yaw = state.yaw; 

    if (!is_initialized_) {
        key_target_x = 0.0;
        key_target_y = 0.0;
        term_target_z = 0.0;
        term_target_yaw = 0.0;
        
        term_target_roll = 0.0;
        term_target_pitch = 0.0;
        g_attitude_lock_enabled = false;
        
        smooth_target_position_ = Eigen::Vector3d::Zero();
        has_smooth_target_ = true;
        
        is_pwm_published = false; 
        is_flying = false; 
        is_initialized_ = true;
        std::cout << "\n[Controller] System Ready. Target: (0,0,0) (PWM is OFF)" << std::endl;
        return; 
    }

    current_drone_state_ = state;

    if (is_started_ && !is_control_enabled_) {
        const ros::Time now = ros::Time::now();
        if ((now - start_time_).toSec() > control_start_delay_sec_) {
            is_control_enabled_ = true;
            enable_position_loop = true;
            is_pwm_published = true;
            is_flying = true;
            key_target_x = current_drone_state_.position.x();
            key_target_y = current_drone_state_.position.y();
            term_target_z = current_drone_state_.position.z();
            term_target_yaw = current_yaw;
            smooth_target_position_ = current_drone_state_.position;
            has_smooth_target_ = true;
            pos_Z1.setZero(); pos_Z2.setZero(); pos_U.setZero();
            att_Z1.setZero(); att_Z2.setZero(); att_U.setZero();
            ROS_INFO("ADRC control enabled by fusion_start. PWM on, position loop on.");
        }
    }

    if (is_control_enabled_) {
        std_msgs::Header control_start_msg;
        control_start_msg.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        control_start_msg.frame_id = "imu_time";
        control_start_publisher.publish(control_start_msg);
    }

    Eigen::Matrix3d target_Rd;
    double base_throttle;
    Eigen::Vector3d active_target_position = current_drone_state_.position;
    Eigen::Vector3d active_target_velocity = Eigen::Vector3d::Zero();

    // ====================================================
    // ⭐ 真正的模式切换逻辑
    // ====================================================
    if (enable_position_loop) {
        // [模式2] 全闭环位置模式：可在键盘输入和规划器输入间切换
        const ros::Time target_stamp = latest_target_stamp_.isZero() ? ros::Time::now() : latest_target_stamp_;
        const bool planner_target_fresh =
            has_planning_target_ && (ros::Time::now() - target_stamp).toSec() <= max_planner_target_age_sec_;

        if (g_use_planner_input && planner_target_fresh) {
            active_target_position = target_position_;
            active_target_velocity = target_velocity_;
            position_velocity_adrc(
                target_position_, target_velocity_,
                current_drone_state_.position, current_drone_state_.velocity, term_target_yaw, dt,
                target_Rd, base_throttle);
        } else {
            if (g_use_planner_input && has_planning_target_) {
                ROS_WARN_THROTTLE(1.0, "Planner target timeout. ADRC falls back to keyboard target.");
            } else if (g_use_planner_input) {
                ROS_WARN_THROTTLE(1.0, "Planner input selected but no target received. ADRC uses keyboard target.");
            }

            Eigen::Vector3d raw_key_target(key_target_x, key_target_y, term_target_z);
            active_target_position = raw_key_target;
            double tau = std::max(0.01, g_tune_tau); 
            double alpha = dt / (tau + dt);
            smooth_target_position_ = smooth_target_position_ + alpha * (raw_key_target - smooth_target_position_);

            Eigen::Vector3d smooth_target_velocity = (raw_key_target - smooth_target_position_) / tau;
            if (smooth_target_velocity.norm() > 3.0) smooth_target_velocity = smooth_target_velocity.normalized() * 3.0;
            active_target_velocity = smooth_target_velocity;
            position_velocity_adrc(
                smooth_target_position_, smooth_target_velocity,
                current_drone_state_.position, current_drone_state_.velocity, term_target_yaw, dt,
                target_Rd, base_throttle);
        }
            
    } else {
        // [模式1] 真正的姿态模式 (ATTI)：放弃XY闭环，只闭环Z轴高度
        double m = 0.9, g = 9.81;
        
        // --- Z轴独立定高控制器 ---
        double pos_err_z = term_target_z - current_drone_state_.position.z();
        double target_vel_cmd_z = limit(g_tune_kp_pos.z() * pos_err_z, -3.0, 3.0); 

        int m_steps = 5;
        double m_dt = dt / m_steps;
        double wc_z = g_tune_wc_vel.z(); 
        double wp_z = g_tune_wp_vel.z();  
        
        for (int j = 0; j < m_steps; j++) {
            double e_obs = key_pos_Z1 - current_drone_state_.velocity.z();
            key_pos_Z1 += (key_pos_Z2 + key_pos_U - 2.0 * wc_z * e_obs) * m_dt;
            key_pos_Z2 += (-wc_z * wc_z * e_obs) * m_dt;
            key_pos_Z2 = limit(key_pos_Z2, -5.0, 5.0);
        }
        double e_v_z = target_vel_cmd_z - key_pos_Z1;
        key_pos_U = wp_z * e_v_z; 

        double F_des_z = m * key_pos_U - m * key_pos_Z2 - m * g;
        
        // --- 姿态直接生成 ---
        // 将键盘直接输入的 Roll/Pitch 拼装成旋转矩阵
        Eigen::AngleAxisd rollA(term_target_roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchA(term_target_pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawA(term_target_yaw, Eigen::Vector3d::UnitZ());
        target_Rd = (yawA * pitchA * rollA).toRotationMatrix();
        active_target_position = Eigen::Vector3d(current_drone_state_.position.x(), current_drone_state_.position.y(), term_target_z);
        active_target_velocity.setZero();

        // 计算倾角补偿后的油门
        double tilt_comp = target_Rd(2, 2); 
        if (tilt_comp < 0.5) tilt_comp = 0.5;
        base_throttle = -F_des_z / tilt_comp;
        if (base_throttle < 0.05) base_throttle = 0.05;
        
        // ⭐ 影子跟随系统：当你在姿态模式下被风吹走时，
        // 虚拟的位置目标会时刻跟着你现在的坐标。
        // 这样当你按下 2 键切回位置模式时，飞机会原地刹住，而不是疯狂往回跑。
        key_target_x = current_drone_state_.position.x();
        key_target_y = current_drone_state_.position.y();
        smooth_target_position_ = current_drone_state_.position;
    }

    // ====================================================
    // ⭐ 防弹跳安全锁
    // ====================================================
    if (!is_flying) {
        base_throttle = 0.05; 
        key_target_x = current_drone_state_.position.x();
        key_target_y = current_drone_state_.position.y();
        term_target_z = current_drone_state_.position.z();
        term_target_yaw = current_yaw;
        term_target_roll = 0.0;
        term_target_pitch = 0.0;
        g_attitude_lock_enabled = false;
        smooth_target_position_ = current_drone_state_.position;
        active_target_position = current_drone_state_.position;
        active_target_velocity.setZero();
        
        pos_Z1.setZero(); pos_Z2.setZero(); pos_U.setZero();
        att_Z1.setZero(); att_Z2.setZero(); att_U.setZero();
    }

    Eigen::Matrix3d current_dcm = current_drone_state_.orientation.toRotationMatrix();
    Eigen::Vector4d pwm_cmd = adrc_controller_block(target_Rd, current_dcm, current_drone_state_.eulerAngle_velocity, base_throttle, dt);

    nav_msgs::Odometry target_debug_msg;
    target_debug_msg.header.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    target_debug_msg.header.frame_id = g_use_planner_input ? "planner_target" : "keyboard_target";
    target_debug_msg.child_frame_id = enable_position_loop ? "position_mode" : "attitude_mode";
    target_debug_msg.pose.pose.position.x = active_target_position.x();
    target_debug_msg.pose.pose.position.y = active_target_position.y();
    target_debug_msg.pose.pose.position.z = active_target_position.z();
    Eigen::Quaterniond target_q(target_Rd);
    target_q.normalize();
    target_debug_msg.pose.pose.orientation.w = target_q.w();
    target_debug_msg.pose.pose.orientation.x = target_q.x();
    target_debug_msg.pose.pose.orientation.y = target_q.y();
    target_debug_msg.pose.pose.orientation.z = target_q.z();
    target_debug_msg.twist.twist.linear.x = active_target_velocity.x();
    target_debug_msg.twist.twist.linear.y = active_target_velocity.y();
    target_debug_msg.twist.twist.linear.z = active_target_velocity.z();
    target_debug_publisher.publish(target_debug_msg);
    
    airsim_ros::RotorPWM pwm_msg;
    if(is_pwm_published) {
        pwm_msg.rotorPWM0 = pwm_cmd(0); pwm_msg.rotorPWM1 = pwm_cmd(1); 
        pwm_msg.rotorPWM2 = pwm_cmd(2); pwm_msg.rotorPWM3 = pwm_cmd(3); 
    } else {
        pwm_msg.rotorPWM0 = 0; pwm_msg.rotorPWM1 = 0; 
        pwm_msg.rotorPWM2 = 0; pwm_msg.rotorPWM3 = 0; 
    }
    pwm_publisher.publish(pwm_msg);
}

void PwmAdrcController::target_pose_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    const Eigen::Vector3d raw_position(msg->position.x, msg->position.y, msg->position.z);
    const Eigen::Vector3d raw_velocity(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    target_position_ = raw_position;
    target_position_.z() += planner_z_offset_;
    target_velocity_ = raw_velocity;
    term_target_yaw = msg->yaw;
    latest_target_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_planning_target_ = true;

    if (g_use_planner_input) {
        enable_position_loop = true;
        ROS_INFO_THROTTLE(1.0,
            "ADRC planner target: raw_p=(%.3f, %.3f, %.3f), mapped_p=(%.3f, %.3f, %.3f), raw_v=(%.3f, %.3f, %.3f), yaw=%.3f, z_offset=%.3f",
            raw_position.x(), raw_position.y(), raw_position.z(),
            target_position_.x(), target_position_.y(), target_position_.z(),
            raw_velocity.x(), raw_velocity.y(), raw_velocity.z(),
            term_target_yaw,
            planner_z_offset_);
    }
}

void PwmAdrcController::start_state_cb(const std_msgs::Header::ConstPtr& msg) {
    const ros::Time now = msg->stamp.isZero() ? ros::Time::now() : msg->stamp;
    if (!is_started_) {
        start_time_ = now;
        is_started_ = true;
        ROS_INFO("Received fusion_start. ADRC control will start after %.2f seconds.", control_start_delay_sec_);
    }
}

void PwmAdrcController::timerCallback(const ros::TimerEvent& event) {
    if(is_takeoff) {
        takeoff_client.call(takeoff);
        key_target_x = current_drone_state_.position.x();
        key_target_y = current_drone_state_.position.y();
        term_target_yaw = current_drone_state_.yaw;
        term_target_z = current_drone_state_.position.z() - 1.0; 
        is_takeoff = false;
    }
    if(is_land) { land_client.call(land); is_land = false; }
    
    if(is_reset) { 
        reset_client.call(reset); 
        std_msgs::Empty reset_msg; 
        reset_publisher.publish(reset_msg); 
        
        pos_Z1.setZero(); pos_Z2.setZero(); pos_U.setZero();
        att_Z1.setZero(); att_Z2.setZero(); att_U.setZero();
        key_pos_Z1 = 0; key_pos_Z2 = 0; key_pos_U = 0;
        term_target_roll = 0.0;
        term_target_pitch = 0.0;
        g_attitude_lock_enabled = false;
        has_planning_target_ = false;
        is_started_ = false;
        is_control_enabled_ = false;

        is_initialized_ = false; 
        std::cout << "\n[System] Engine Reset Complete." << std::endl;
        is_reset = false; 
    }
}

void PwmAdrcController::position_velocity_adrc(
    const Eigen::Vector3d& target_pos, const Eigen::Vector3d& target_vel,
    const Eigen::Vector3d& current_pos, const Eigen::Vector3d& current_vel, double target_yaw, double dt,
    Eigen::Matrix3d& target_Rd, double& base_throttle) 
{
    double m = 0.9, g = 9.81;
    
    Eigen::Vector3d Kp_pos = g_tune_kp_pos;
    Eigen::Vector3d wc_vel = g_tune_wc_vel;
    Eigen::Vector3d wp_vel = g_tune_wp_vel;
    
    double z2_limit_xy = 0.0;
    double z2_limit_z = 5.0; 
    double max_tilt_angle = 45.0 * M_PI / 180.0; 
    double max_v_xy = 18.0; 
    double max_v_z_up = -8.0;   
    double max_v_z_down = 6.0;

    Eigen::Vector3d pos_err = target_pos - current_pos;
    Eigen::Vector3d target_vel_cmd = Kp_pos.cwiseProduct(pos_err) + target_vel;
    
    double v_xy_mag = target_vel_cmd.head<2>().norm();
    if (v_xy_mag > max_v_xy) target_vel_cmd.head<2>() = (target_vel_cmd.head<2>() / v_xy_mag) * max_v_xy;
    target_vel_cmd.z() = limit(target_vel_cmd.z(), max_v_z_up, max_v_z_down);

    int m_steps = 5;
    double m_dt = dt / m_steps;
    Eigen::Vector3d target_acc_cmd = Eigen::Vector3d::Zero();

    for (int ch = 0; ch < 3; ch++) {
        double v_feedback = current_vel[ch];
        for (int j = 0; j < m_steps; j++) {
            double e_obs = pos_Z1[ch] - v_feedback;
            pos_Z1[ch] += (pos_Z2[ch] + pos_U[ch] - 2.0 * wc_vel[ch] * e_obs) * m_dt;
            pos_Z2[ch] += (-wc_vel[ch] * wc_vel[ch] * e_obs) * m_dt;
            
            if (ch < 2) pos_Z2[ch] = limit(pos_Z2[ch], -z2_limit_xy, z2_limit_xy);
            else        pos_Z2[ch] = limit(pos_Z2[ch], -z2_limit_z, z2_limit_z);
        }
        double e_v = target_vel_cmd[ch] - pos_Z1[ch];
        double u0 = wp_vel[ch] * e_v;
        pos_U[ch] = u0 - pos_Z2[ch];
        target_acc_cmd[ch] = pos_U[ch];
    }

    Eigen::Vector3d F_dynamic = m * target_acc_cmd;
    Eigen::Vector3d F_des = F_dynamic - m * g * Eigen::Vector3d(0, 0, 1);
    
    double F_des_mag = F_des.norm();
    Eigen::Vector3d b3d = (F_des_mag < 1e-3) ? Eigen::Vector3d(0, 0, 1) : (-F_des / F_des_mag);

    double cos_max_tilt = cos(max_tilt_angle);
    if (b3d.z() < cos_max_tilt) {
        double xy_mag = b3d.head<2>().norm();
        if (xy_mag > 1e-6) {
            double sin_max_tilt = sin(max_tilt_angle);
            b3d.x() = (b3d.x() / xy_mag) * sin_max_tilt;
            b3d.y() = (b3d.y() / xy_mag) * sin_max_tilt;
        } else {
            b3d.x() = 0; b3d.y() = 0;
        }
        b3d.z() = cos_max_tilt;
        b3d.normalize();
    }

    double tilt_comp = b3d.z();
    if (tilt_comp < 0.5) tilt_comp = 0.5;
    base_throttle = -F_des.z() / tilt_comp;
    if (base_throttle < 0.05) base_throttle = 0.05;

    Eigen::Vector3d b1d_temp(cos(target_yaw), sin(target_yaw), 0.0);
    Eigen::Vector3d b3_cross_b1 = b3d.cross(b1d_temp);
    if (b3_cross_b1.norm() < 1e-3) {
        b1d_temp = Eigen::Vector3d(1.0, 0.0, 0.0);
        b3_cross_b1 = b3d.cross(b1d_temp);
    }
    
    Eigen::Vector3d b2d = b3_cross_b1.normalized();
    Eigen::Vector3d b1d = b2d.cross(b3d);
    
    target_Rd.col(0) = b1d;
    target_Rd.col(1) = b2d;
    target_Rd.col(2) = b3d;
}

Eigen::Vector4d PwmAdrcController::adrc_controller_block(
    const Eigen::Matrix3d& target_Rd, const Eigen::Matrix3d& current_dcm,
    const Eigen::Vector3d& current_gyro, double base_throttle, double dt) 
{
    double max_rpm = 11079.03;
    double ct = 0.000367717;
    double max_t = ct * pow(max_rpm/60.0, 2);
    
    Eigen::Vector3d Kp_angle = g_tune_kp_ang; 
    Eigen::Vector3d wc = g_tune_wc_att; 
    Eigen::Vector3d wp = g_tune_wp_att; 
    Eigen::Vector3d b0 = g_tune_b0;
    
    double z2_limit_xy = 5.0; 
    double z2_limit_yaw = 2.0;

    Eigen::Matrix3d err_matrix = 0.5 * (target_Rd.transpose() * current_dcm - current_dcm.transpose() * target_Rd);
    Eigen::Vector3d e_R(err_matrix(2,1), err_matrix(0,2), err_matrix(1,0));
    Eigen::Vector3d target_rate_cmd = -Kp_angle.cwiseProduct(e_R);

    int m_steps = 10;
    double m_dt = dt / m_steps;
    Eigen::Vector3d target_T = Eigen::Vector3d::Zero();

    for (int ch = 0; ch < 3; ch++) {
        for (int j = 0; j < m_steps; j++) {
            double e_obs = att_Z1[ch] - current_gyro[ch];
            att_Z1[ch] += (att_Z2[ch] + b0[ch]*att_U[ch] - 2.0*wc[ch]*e_obs) * m_dt;
            att_Z2[ch] += (-wc[ch]*wc[ch]*e_obs) * m_dt;
            
            double current_limit = (ch == 2) ? z2_limit_yaw : z2_limit_xy;
            att_Z2[ch] = limit(att_Z2[ch], -current_limit, current_limit);
        }
        double e_rate = target_rate_cmd[ch] - att_Z1[ch];
        double u0 = wp[ch] * e_rate;
        att_U[ch] = (u0 - att_Z2[ch]) / b0[ch];
        target_T[ch] = att_U[ch];
    }

    double f = base_throttle / 4.0;
    double Mx = target_T(0) / 4.0;
    double My = target_T(1) / 4.0;
    double Mz = target_T(2) / 4.0;
    
    double k = 4.888486e-06 / ct; 
    double yaw_effect = Mz / k;
    double yaw_limit = f * 1; 
    yaw_effect = limit(yaw_effect, -yaw_limit, yaw_limit);
    
    double L_projected = 0.127279; 
    
    double f1 = f - Mx / L_projected + My / L_projected + yaw_effect; 
    double f2 = f + Mx / L_projected - My / L_projected + yaw_effect; 
    double f3 = f + Mx / L_projected + My / L_projected - yaw_effect; 
    double f4 = f - Mx / L_projected - My / L_projected - yaw_effect; 
    
    double pwm0 = std::sqrt(limit(f1 / max_t, 0.0, 1.0));
    double pwm1 = std::sqrt(limit(f2 / max_t, 0.0, 1.0));
    double pwm2 = std::sqrt(limit(f3 / max_t, 0.0, 1.0));
    double pwm3 = std::sqrt(limit(f4 / max_t, 0.0, 1.0));

    const double min_idle_pwm = 0.05;
    const double min_pwm = std::min({pwm0, pwm1, pwm2, pwm3});
    if (min_pwm < min_idle_pwm) {
        const double boost_amount = min_idle_pwm - min_pwm;
        pwm0 += boost_amount;
        pwm1 += boost_amount;
        pwm2 += boost_amount;
        pwm3 += boost_amount;
    }

    return Eigen::Vector4d(
        limit(pwm0, 0.0, 1.0),
        limit(pwm1, 0.0, 1.0),
        limit(pwm2, 0.0, 1.0),
        limit(pwm3, 0.0, 1.0)
    );
}

double PwmAdrcController::limit(double value, double min_value, double max_value) {
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

Eigen::Vector3d quatToEuler(const Eigen::Quaterniond& q) {
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    double pitch;
    if (std::abs(sinp) >= 1) pitch = std::copysign(M_PI / 2, sinp); 
    else pitch = std::asin(sinp);
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return Eigen::Vector3d(yaw, pitch, roll);
}
