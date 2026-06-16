#include "pwm_mpc_controller.hpp"

#include <limits>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "pwm_mpc_controller"); // 初始化ros 节点，命名为 pwm_mpc_controller
    ros::NodeHandle n; // 创建node控制句柄
    PwmMpcController go(&n);
    return 0;
}

PwmMpcController::PwmMpcController(ros::NodeHandle *nh)
{  
    nh->param("control_loop_hz", control_loop_hz_, 50.0);
    nh->param("max_state_age_sec", max_state_age_sec_, 0.2);
    nh->param("mpc_solve_hz", mpc_solve_hz_, 20.0);
    mpc_solve_hz_ = std::max(1.0, mpc_solve_hz_);

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

    // mpc控制器初始化
    problem_params_t problem_params;
    model_params_t model_params;
    // 按需修改
    problem_params.ts = 0.01;
    problem_params.N = 20;
    problem_params.R = DM::diag({0.04, 10.0, 800.0, 10.0});
    problem_params.Q = DM::diag({7.0, 7.0, 5.0,
                                 0.8, 0.8, 1.5,
                                 8.0, 8.0, 8.0, 8.0,
                                 0.5, 0.5, 0.5});
    problem_params.P = DM::diag({8.0, 8.0, 8.0,
                                 1.0, 1.0, 1.8,
                                 10.0, 10.0, 10.0, 10.0,
                                 0.65, 0.65, 0.65});
    problem_params.q_att_weight = 5.0;
    problem_params.q_att_terminal_weight = 20.0;
    problem_params.max_roll_pitch = 15.0 / 57.3;
    problem_params.max_yaw_rate = 100 / 57.3;
    problem_params.max_F = 20.0;
    problem_params.min_F = -20.0;
    problem_params.max_Tx = 1.0;
    problem_params.max_Ty = 1.0;
    problem_params.max_Tz = 0.5;
    mpc_controller_.init_mpc_problem(problem_params, model_params);
    // 设置求解器选项
    casadi::Dict opts;
    // 基础计时与输出
    opts["print_time"] = true;         // 开启耗时统计
    opts["ipopt.print_level"] = 0;     // 输出详细迭代信息
    
    // 核心性能优化选项
    opts["expand"] = true;             // 展开表达式图，减少函数调用开销
    opts["jit"] = true;                // 启用JIT编译，关键加速选项！
    opts["compiler"] = "shell";        // 配合jit使用的编译器设置
    
    // 求解器内部优化
    opts["ipopt.linear_solver"] = "mumps";      // 更换为高效的线性求解器
    opts["ipopt.max_iter"] = 100;              // 限制最大迭代次数，防止失控
    opts["ipopt.tol"] = 1e-3;                  // 适当放宽精度要求
    opts["ipopt.acceptable_tol"] = 1e-4;       // 设置可接受精度
    opts["ipopt.warm_start_init_point"] = "yes"; // 启用热启动
    opts["ipopt.warm_start_bound_push"] = 1e-9;
    opts["ipopt.warm_start_mult_bound_push"] = 1e-9;
    mpc_controller_.init_solver("ipopt", opts);
    ROS_INFO("MPC configured: control_loop_hz=%.1f, mpc_solve_hz=%.1f, N=%d, ts=%.3f",
               control_loop_hz_, mpc_solve_hz_, problem_params.N, problem_params.ts);

    mixing_matrix_ = mpc_controller_.get_x_mixing_matrix(model_params.ct, model_params.cm, model_params.R);

    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);
    reset_publisher = nh->advertise<std_msgs::Empty>("/airsim_node/reset_cmd", 1);

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    odom_suber = nh->subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/drone_state", 1, std::bind(&PwmMpcController::pose_cb, this, std::placeholders::_1));//imu与gps数据融合后的位姿数据
    target_point_suber_ = nh->subscribe<geometry_msgs::PointStamped>("image_solver/score_gate_target_point", 1, std::bind(&PwmMpcController::target_point_cb, this, std::placeholders::_1));//目标点数据

    takeoff.request.waitOnLastTask = 1;
    land.request.waitOnLastTask = 1;
    //通过三个服务可以调用模拟器中的无人机起飞和降落命令和重置命令
    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/land");
    reset_client = nh->serviceClient<airsim_ros::Reset>("/airsim_node/reset");

    // 创建处理起降/复位/PID热更新的定时器
    command_timer_ = nh->createTimer(ros::Duration(0.1), std::bind(&PwmMpcController::timerCallback, this, std::placeholders::_1));

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

PwmMpcController::~PwmMpcController()
{
    stopControlThread();
}

void PwmMpcController::pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    DroneState state;
    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Vector3d velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d eulerAngle = quatToEuler(q); // 注意这里的顺序是z-y-x，对应航向角-俯仰角-滚转角
    Eigen::Vector3d eulerAngle_velocity(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    // if (is_cout) {
    //     // ROS_INFO("Get pose data. time: %f, eulerangle: %f, %f, %f,eulerangle_velocity: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
    //     //     eulerAngle[0], eulerAngle[1], eulerAngle[2], eulerAngle_velocity[0], eulerAngle_velocity[1], eulerAngle_velocity[2]);
    // }
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

void PwmMpcController::target_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    Eigen::Vector3d target_position(msg->point.x, msg->point.y, msg->point.z);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_position_ = target_position;
    }
    // if (is_cout) {
    //     ROS_INFO("Get target point. time: %f, position: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
    //         target_position[0], target_position[1], target_position[2]);
    // }
}

void PwmMpcController::publish_pwm_command(const ros::Time& stamp)
{
    // pwm范围为0～1.0
    pwm_cmd_[0] = limit(pwm_cmd_[0], 0.0, 1.0);
    pwm_cmd_[1] = limit(pwm_cmd_[1], 0.0, 1.0);
    pwm_cmd_[2] = limit(pwm_cmd_[2], 0.0, 1.0);
    pwm_cmd_[3] = limit(pwm_cmd_[3], 0.0, 1.0);
    if(is_pwm_published) 
    {
        airsim_ros::RotorPWM pwm_cmd;
        pwm_cmd.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
        pwm_cmd.header.frame_id = "imu_time";
        pwm_cmd.rotorPWM0 = pwm_cmd_[0]; // 右前 顺时针
        pwm_cmd.rotorPWM1 = pwm_cmd_[1]; // 左后 顺时针
        pwm_cmd.rotorPWM2 = pwm_cmd_[2]; // 左前 逆时针
        pwm_cmd.rotorPWM3 = pwm_cmd_[3]; // 右后 逆时针
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
        ROS_INFO("Publish pwm command: %f, %f, %f, %f\n", pwm_cmd_[0], pwm_cmd_[1], pwm_cmd_[2], pwm_cmd_[3]);
    }
}

void PwmMpcController::timerCallback(const ros::TimerEvent& event)
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
        ROS_INFO("parameters updated. u_cmd_0: %f\n", u_cmd_0);
        teminal_pid_set.is_pid_changed = false;
    }
}

void PwmMpcController::startControlThread()
{
    if (control_loop_running_.exchange(true)) {
        return;
    }

    control_thread_ = std::thread(&PwmMpcController::controlLoop, this);
}

void PwmMpcController::stopControlThread()
{
    if (!control_loop_running_.exchange(false)) {
        return;
    }

    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}

void PwmMpcController::controlLoop()
{
    if (control_loop_hz_ <= 0.0) {
        control_loop_hz_ = 200.0;
    }

    const auto period = std::chrono::duration<double>(1.0 / control_loop_hz_);
    auto next_tick = std::chrono::steady_clock::now();

    const int n_state = mpc_controller_.n_state;
    const int n_input = mpc_controller_.n_input;
    const int N = mpc_controller_.problem_params.N;

    // NED target: [north, east, down]
    DM target_pos_ned = DM(std::vector<double>{0.0, 0.0, 0.0});

    DM u_guess = DM::zeros(n_input, N);
    u_guess(0, Slice()) = mpc_controller_.model_params.m * mpc_controller_.model_params.g;
    DM x_guess = DM::zeros(n_state, N + 1);
    x_guess(6, Slice()) = 1.0;

    const double w2_min = mpc_controller_.model_params.min_w * mpc_controller_.model_params.min_w;
    const double w2_max = mpc_controller_.model_params.max_w * mpc_controller_.model_params.max_w;
    const double rpm_min = mpc_controller_.model_params.min_w * 60.0 / (2.0 * M_PI);
    const double rpm_max = mpc_controller_.model_params.max_w * 60.0 / (2.0 * M_PI);
    const double pwm_min = 0.0;
    const double pwm_max = 1.0;

    // Lightweight MPC telemetry for runtime tuning.
    std::size_t solve_attempts = 0;
    std::size_t solve_failures = 0;
    std::size_t solve_overruns = 0;
    double solve_ms_avg = 0.0;
    double solve_ms_max = 0.0;
    int last_iter_count = -1;
    const double period_ms = 1000.0 / control_loop_hz_;
    const double solve_budget_ms = 1000.0 / mpc_solve_hz_;
    const auto mpc_period = std::chrono::duration<double>(1.0 / mpc_solve_hz_);
    auto last_mpc_solve_tp = std::chrono::steady_clock::now() - mpc_period;

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

        if (has_fresh_state) {
            const auto now_tp = std::chrono::steady_clock::now();
            if (now_tp - last_mpc_solve_tp < mpc_period) {
                publish_pwm_command(msg_stamp);
                next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
                std::this_thread::sleep_until(next_tick);
                auto now_tick = std::chrono::steady_clock::now();
                if (now_tick > next_tick + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period)) {
                    next_tick = now_tick;
                }
                continue;
            }

            std::lock_guard<std::mutex> lock(pid_mutex_);
            current_drone_state_ = state;

            // State (NED): [pn, pe, pd, vn, ve, vd, qw, qx, qy, qz, p, q, r]
            // Input (body wrench): [F, T_x, T_y, T_z]
            // 根据实际数据，更新MPC控制器的状态和目标
            DM current_state =  DM::zeros(13, 1);
            current_state(0) = current_drone_state_.position.x();
            current_state(1) = current_drone_state_.position.y();
            current_state(2) = current_drone_state_.position.z();
            current_state(3) = current_drone_state_.velocity.x();
            current_state(4) = current_drone_state_.velocity.y();
            current_state(5) = current_drone_state_.velocity.z();
            // 四元数
            current_state(6) = current_drone_state_.orientation.w();
            current_state(7) = current_drone_state_.orientation.x();
            current_state(8) = current_drone_state_.orientation.y();
            current_state(9) = current_drone_state_.orientation.z();
            current_state(10) = current_drone_state_.eulerAngle_velocity.x();
            current_state(11) = current_drone_state_.eulerAngle_velocity.y();
            current_state(12) = current_drone_state_.eulerAngle_velocity.z();

            // Keep the first warm-start state aligned with measured state.
            x_guess(Slice(), 0) = current_state;

            target_pos_ned = DM(std::vector<double>{target_x, target_y, target_z});

            DMDict res;
            auto solve_begin_tp = std::chrono::steady_clock::now();
            ++solve_attempts;
            try {
                res = mpc_controller_.compute_to_target(current_state, target_pos_ned, u_guess, x_guess);
                last_mpc_solve_tp = std::chrono::steady_clock::now();
            } catch (const std::exception& e) {
                ++solve_failures;
                ROS_WARN("MPC solver failed at time %.3f sec: %s. Using previous control.\n",
                         ros::Time::now().toSec(), e.what());
                continue;
            }

            // 日志输出求解时间统计和迭代次数，辅助性能调优。
            // const double solve_ms = std::chrono::duration<double, std::milli>(
            //     std::chrono::steady_clock::now() - solve_begin_tp).count();
            // if (solve_ms > solve_budget_ms) {
            //     ++solve_overruns;
            // }
            // if (solve_ms > solve_ms_max) {
            //     solve_ms_max = solve_ms;
            // }
            // if (solve_attempts == 1) {
            //     solve_ms_avg = solve_ms;
            // } else {
            //     solve_ms_avg += (solve_ms - solve_ms_avg) / static_cast<double>(solve_attempts);
            // }

            // last_iter_count = -1;
            // try {
            //     const Dict stats = mpc_controller_.mpc_solver.stats();
            //     auto iter_it = stats.find("iter_count");
            //     if (iter_it != stats.end()) {
            //         if (iter_it->second.is_int()) {
            //             last_iter_count = static_cast<int>(iter_it->second.to_int());
            //         } else if (iter_it->second.is_double()) {
            //             last_iter_count = static_cast<int>(iter_it->second.to_double());
            //         }
            //     }
            // } catch (const std::exception&) {
            // }

            // const double fail_rate = solve_attempts > 0
            //     ? 100.0 * static_cast<double>(solve_failures) / static_cast<double>(solve_attempts)
            //     : 0.0;
            // ROS_INFO_THROTTLE(1.0,
            //     "MPC stats | solve_ms=%.2f avg=%.2f max=%.2f ctrl_period=%.2f solve_budget=%.2f iter=%d fail=%.2f%% overruns=%zu/%zu",
            //     solve_ms, solve_ms_avg, solve_ms_max, period_ms, solve_budget_ms, last_iter_count,
            //     fail_rate, solve_overruns, solve_attempts);

            DM u_opt_vec = res["x"](Slice(0, n_input * N));
            DM u_opt = reshape(u_opt_vec, n_input, N);
            DM u_cmd = u_opt(Slice(), 0);

            DM x_opt_vec = res["x"](Slice(n_input * N, n_input * N + n_state * (N + 1)));
            DM x_opt = reshape(x_opt_vec, n_state, N + 1);

            // Shift optimal sequence for warm start at next MPC step.
            u_guess = horzcat(u_opt(Slice(), Slice(1, N)), u_opt(Slice(), N - 1));
            x_guess = horzcat(x_opt(Slice(), Slice(1, N + 1)), x_opt(Slice(), N));

            // 测试，先把u_cmd的力矩清零
            //u_cmd(0) = u_cmd_0 / 4.0; // 平均分配到4个电机的升力，保持悬停
            u_cmd(1) = 0.0;
            //u_cmd(2) = 0.0; // 施加一个小的俯仰力矩，测试能否看到姿态变化
            u_cmd(3) = 0.0;

            u_cmd(0) = u_cmd(0) / 4.0; // 平均分配到4个电机的升力，保持悬停
            u_cmd(2) = u_cmd(2) / 4.0; // 平均分配到4个电机的滚转力矩，保持平衡

            std::cout << "MPC control " << ros::Time::now().toSec() << ": F=" << u_cmd(0) << ", Tx=" << u_cmd(1) << ", Ty=" << u_cmd(2) << ", Tz=" << u_cmd(3) << std::endl;

            // Convert [f, tau_x, tau_y, tau_z] to squared motor speeds and then to PWM.
            // DM mixing_matrix_;
            DM w2_cmd = mtimes(mixing_matrix_, u_cmd);
            std::vector<double> pwm_cmd(4, 0.0);
            for (int m = 0; m < 4; ++m) {
                double w2 = static_cast<double>(w2_cmd(m));
                w2 = std::max(w2_min, std::min(w2_max, w2));
                double omega = std::sqrt(w2);
                double rpm = omega * 60.0 / (2.0 * M_PI);
                pwm_cmd[m] = rpm_to_pwm(rpm, rpm_min, rpm_max, pwm_min, pwm_max);
                std::cout << "Raw control " << m << ": w2=" << w2 << ", omega=" << omega << ", rpm=" << rpm << ", pwm=" << pwm_cmd[m] << std::endl;
            }

            pwm_cmd_ = pwm_cmd;
            
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

double PwmMpcController::limit(double value, double min_value, double max_value)
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

// Simple linear rpm->pwm mapping placeholder. Replace by ESC calibration curve.
double rpm_to_pwm(double rpm, double rpm_min, double rpm_max, double pwm_min, double pwm_max) {
    double clamped = std::max(rpm_min, std::min(rpm_max, rpm));
    double alpha = (clamped - rpm_min) / (rpm_max - rpm_min);
    return pwm_min + alpha * (pwm_max - pwm_min);
}
