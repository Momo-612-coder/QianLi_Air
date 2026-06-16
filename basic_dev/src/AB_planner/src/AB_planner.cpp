#include "AB_planner.hpp"

#include "airsim_ros/VelCmd.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "AB_planner"); // 初始化ros 节点，命名为 AB_planner
    ros::NodeHandle n; // 创建node控制句柄
    ABPlanner go(&n);
    ros::spin();
    return 0;
}

ABPlanner::ABPlanner(ros::NodeHandle *nh) : nh_(*nh), pnh_("~")
{  
    pnh_.param("plan_size_x", plan_size_x_, 80.0);
    pnh_.param("plan_size_y", plan_size_y_, 80.0);
    pnh_.param("plan_size_z", plan_size_z_, 60.0);
    pnh_.param("resolution", resolution_, 0.5);
    pnh_.param("reserved_back_space", reserved_back_space_, 10.0);
    pnh_.param("safety_distance", safety_distance_, 1.0);
    pnh_.param("h_weight", h_weight_, 1.0);
    pnh_.param("odometry_topic", odometry_topic_, std::string("/airsim_node/drone_1/drone_state"));
    pnh_.param("fusion_start_topic", fusion_start_topic_, std::string("/fusion_start"));
    pnh_.param("obstacle_topic", obstacle_topic_, std::string("/front_fov_obstacles"));
    pnh_.param("visualization_frame", visualization_frame_, std::string("map_rviz"));
    pnh_.param("bspline_visualization_samples", bspline_visualization_samples_, 500);
    pnh_.param("use_rviz", publish_rviz_visualization_, true);
    pnh_.param("acc_lpf_alpha", acc_lpf_alpha_, 0.15);
    pnh_.param("mpc/command_step", mpc_command_step_, 5);
    pnh_.param("convert_ned_to_position_cmd", convert_ned_to_position_cmd_, false);
    pnh_.param("plan_waypoint_lookahead", plan_waypoint_lookahead_, 8);
    pnh_.param("replan_trigger_distance", replan_trigger_distance_, 25.0);
    pnh_.param("planning_failure_retry_interval", planning_failure_retry_interval_, 0.25);
    pnh_.param("bspline_control_point_spacing", bspline_control_point_spacing_, 2.0);
    pnh_.param("waypoint_reached_distance", waypoint_reached_distance_, 12.0);
    pnh_.param("use_waypoint_yaw", use_waypoint_yaw_, true);
    pnh_.param("max_yaw_rate", max_yaw_rate_, 1.5);
    pnh_.param("segment_turnaround/enabled", segment_turnaround_enabled_, true);
    pnh_.param("segment_turnaround/duration", segment_turnaround_duration_, 5.0);
    pnh_.param("segment_turnaround/yaw_delta", segment_turnaround_yaw_delta_, 3.14159265358979323846);
    pnh_.param("path_optimization/enabled", path_optimization_enabled_, true);
    pnh_.param("path_optimization/iterations", path_optimization_iterations_, 80);
    pnh_.param("path_optimization/data_weight", path_optimization_data_weight_, 0.20);
    pnh_.param("path_optimization/smooth_weight", path_optimization_smooth_weight_, 0.45);
    pnh_.param("path_optimization/z_smooth_weight", path_optimization_z_smooth_weight_, 0.45);
    pnh_.param("path_optimization/max_deviation", path_optimization_max_deviation_, 1.5);
    pnh_.param("path_optimization/max_z_slope", path_optimization_max_z_slope_, 0.45);
    pnh_.param("path_optimization/min_turn_radius", path_optimization_min_turn_radius_, 6.0);
    pnh_.param("bspline_collision_check/enabled", bspline_collision_check_enabled_, true);
    pnh_.param("bspline_collision_check/sample_step", bspline_collision_sample_step_, 0.3);
    pnh_.param("bspline_collision_check/extra_clearance", bspline_collision_extra_clearance_, 0.2);
    pnh_.param("dynamic_avoidance/enabled", dynamic_avoidance_enabled_, true);
    pnh_.param("dynamic_avoidance/replan_period", obstacle_replan_period_, 0.5);
    pnh_.param("dynamic_avoidance/obstacle_stale_time", obstacle_stale_time_, 0.5);
    pnh_.param("dynamic_avoidance/obstacle_history_time", obstacle_history_time_, 0.8);
    pnh_.param("dynamic_avoidance/obstacle_history_max_frames", obstacle_history_max_frames_, 3);
    pnh_.param("dynamic_avoidance/obstacle_merge_distance", obstacle_merge_distance_, 0.8);
    pnh_.param("dynamic_avoidance/immediate_replan_on_collision", immediate_replan_on_collision_, true);
    pnh_.param("dynamic_avoidance/min_replan_interval", obstacle_min_replan_interval_, 0.2);
    pnh_.param("dynamic_avoidance/obstacle_inflation", dynamic_obstacle_inflation_, 2.5);
    pnh_.param("dynamic_avoidance/endpoint_clear_radius", dynamic_obstacle_endpoint_clear_radius_, 2.0);
    pnh_.param("dynamic_avoidance/clear_history_on_empty_frame", clear_obstacle_history_on_empty_frame_, true);
    pnh_.param("dynamic_avoidance/threat_confirm_frames", obstacle_threat_confirm_frames_, 2);
    pnh_.param("dynamic_avoidance/threat_clear_frames", obstacle_threat_clear_frames_, 3);
    pnh_.param("astar/max_search_nodes", astar_max_search_nodes_, 500000);
    pnh_.param("fallback/max_speed", fallback_max_speed_, 8.0);
    pnh_.param("fallback/max_vertical_speed", fallback_max_vertical_speed_, 3.0);
    pnh_.param("startup_altitude_hold/enabled", startup_altitude_hold_enabled_, true);
    pnh_.param("startup_altitude_hold/distance", startup_altitude_hold_distance_, 8.0);
    pnh_.param("startup_altitude_hold/margin", startup_altitude_hold_margin_, 0.3);
    pnh_.param("planning_recovery/enabled", planning_recovery_enabled_, true);
    pnh_.param("planning_recovery/failure_threshold", planning_recovery_failure_threshold_, 2);
    pnh_.param("planning_recovery/step", planning_recovery_step_, 0.5);
    pnh_.param("planning_recovery/duration", planning_recovery_duration_, 1.5);
    pnh_.param("planning_recovery/reached_distance", planning_recovery_reached_distance_, 0.2);
    pnh_.param("planning_recovery/stuck_speed", planning_recovery_stuck_speed_, 0.5);
    pnh_.param("planning_recovery/allow_vertical", planning_recovery_allow_vertical_, false);
    pnh_.param("mpc/N", mpc_config_.N, mpc_config_.N);
    pnh_.param("mpc/dt", mpc_config_.dt, mpc_config_.dt);
    pnh_.param("mpc/max_jerk", mpc_config_.max_jerk, mpc_config_.max_jerk);
    pnh_.param("mpc/max_acc", mpc_config_.max_acc, mpc_config_.max_acc);
    pnh_.param("mpc/max_vel", mpc_config_.max_vel, mpc_config_.max_vel);
    pnh_.param("mpc/w_pos", mpc_config_.w_pos, mpc_config_.w_pos);
    pnh_.param("mpc/w_vel", mpc_config_.w_vel, mpc_config_.w_vel);
    pnh_.param("mpc/w_terminal_pos", mpc_config_.w_terminal_pos, mpc_config_.w_terminal_pos);
    pnh_.param("mpc/w_jerk", mpc_config_.w_jerk, mpc_config_.w_jerk);
    pnh_.param("mpc/w_jerk_delta", mpc_config_.w_jerk_delta, mpc_config_.w_jerk_delta);
    pnh_.param("mpc/obstacle_constraints_enabled", mpc_config_.obstacle_constraints_enabled, mpc_config_.obstacle_constraints_enabled);
    pnh_.param("mpc/obstacle_safe_distance", mpc_config_.obstacle_safe_distance, mpc_config_.obstacle_safe_distance);
    pnh_.param("mpc/obstacle_active_distance", mpc_config_.obstacle_active_distance, mpc_config_.obstacle_active_distance);
    pnh_.param("mpc/obstacle_max_count", mpc_config_.obstacle_max_count, mpc_config_.obstacle_max_count);
    pnh_.param("mpc/obstacle_check_step", mpc_config_.obstacle_check_step, mpc_config_.obstacle_check_step);
    acc_lpf_alpha_ = std::clamp(acc_lpf_alpha_, 0.0, 1.0);
    mpc_command_step_ = std::max(1, std::min(mpc_command_step_, mpc_config_.N));
    plan_waypoint_lookahead_ = std::max(2, plan_waypoint_lookahead_);
    replan_trigger_distance_ = std::max(1.0, replan_trigger_distance_);
    planning_failure_retry_interval_ = std::max(0.05, planning_failure_retry_interval_);
    bspline_control_point_spacing_ = std::max(resolution_, bspline_control_point_spacing_);
    waypoint_reached_distance_ = std::max(distance_to_target_threshold_, waypoint_reached_distance_);
    max_yaw_rate_ = std::max(0.1, max_yaw_rate_);
    segment_turnaround_duration_ = std::max(0.0, segment_turnaround_duration_);
    segment_turnaround_yaw_delta_ = normalizeAngle(segment_turnaround_yaw_delta_);
    path_optimization_iterations_ = std::max(0, path_optimization_iterations_);
    path_optimization_data_weight_ = std::clamp(path_optimization_data_weight_, 0.0, 1.0);
    path_optimization_smooth_weight_ = std::clamp(path_optimization_smooth_weight_, 0.0, 1.0);
    path_optimization_z_smooth_weight_ = std::clamp(path_optimization_z_smooth_weight_, 0.0, 1.0);
    path_optimization_max_deviation_ = std::max(0.0, path_optimization_max_deviation_);
    path_optimization_max_z_slope_ = std::max(0.05, path_optimization_max_z_slope_);
    path_optimization_min_turn_radius_ = std::max(0.0, path_optimization_min_turn_radius_);
    bspline_collision_sample_step_ = std::max(0.05, bspline_collision_sample_step_);
    bspline_collision_extra_clearance_ = std::max(0.0, bspline_collision_extra_clearance_);
    obstacle_replan_period_ = std::max(0.05, obstacle_replan_period_);
    obstacle_stale_time_ = std::max(0.05, obstacle_stale_time_);
    obstacle_history_time_ = std::max(obstacle_stale_time_, obstacle_history_time_);
    obstacle_history_max_frames_ = std::max(1, obstacle_history_max_frames_);
    obstacle_merge_distance_ = std::max(0.0, obstacle_merge_distance_);
    obstacle_min_replan_interval_ = std::max(0.0, obstacle_min_replan_interval_);
    dynamic_obstacle_inflation_ = std::max(0.0, dynamic_obstacle_inflation_);
    dynamic_obstacle_endpoint_clear_radius_ = std::max(resolution_, dynamic_obstacle_endpoint_clear_radius_);
    obstacle_threat_confirm_frames_ = std::max(1, obstacle_threat_confirm_frames_);
    obstacle_threat_clear_frames_ = std::max(1, obstacle_threat_clear_frames_);
    astar_max_search_nodes_ = std::max(1, astar_max_search_nodes_);
    astar_max_search_nodes_ = std::min(10000, astar_max_search_nodes_); // 强制降级，避免A*运算超时严重阻塞控制定时器造成频率抖动
    fallback_max_speed_ = std::max(0.5, fallback_max_speed_);
    fallback_max_vertical_speed_ = std::max(0.2, fallback_max_vertical_speed_);
    startup_altitude_hold_distance_ = std::max(0.0, startup_altitude_hold_distance_);
    startup_altitude_hold_margin_ = std::max(0.0, startup_altitude_hold_margin_);
    planning_recovery_failure_threshold_ = std::max(1, planning_recovery_failure_threshold_);
    planning_recovery_step_ = std::max(0.05, planning_recovery_step_);
    planning_recovery_duration_ = std::max(0.2, planning_recovery_duration_);
    planning_recovery_reached_distance_ = std::max(0.05, planning_recovery_reached_distance_);
    planning_recovery_stuck_speed_ = std::max(0.05, planning_recovery_stuck_speed_);
    max_vel_ = mpc_config_.max_vel;

    planner_state_ = IDLE; // 初始化状态为IDLE，等待起飞命令或目标点

    // TODO: 注意，此处硬编码了"72"作为分段点，如果使用的是另一个比72小的航点文件，会导致段错误或越界。建议改为根据距离或者参数动态载入分段点。
    int hardcoded_segment_pt = 72; 
    readWaypointsFromFile("/basic_dev/waypoint.txt", waypoints_);
    if (waypoints_.empty()) {
        ROS_ERROR("No waypoints loaded. Please check the waypoints file.");
        return;
    }

    if (hardcoded_segment_pt < static_cast<int>(waypoints_.size())) {
        segment_idx_.push_back(hardcoded_segment_pt); // 将规划路径分段
    }
    
    if (segment_idx_.empty() || segment_idx_.back() != static_cast<int>(waypoints_.size())) {
        segment_idx_.push_back(static_cast<int>(waypoints_.size()));
    }
    for (size_t i = 0; i < segment_idx_.size(); ++i) {
        const int prev_idx = (i == 0) ? 0 : segment_idx_[i - 1];
        if (segment_idx_[i] <= prev_idx || segment_idx_[i] > static_cast<int>(waypoints_.size())) {
            ROS_ERROR("Invalid segment index %d at position %lu. Please check the segment index.",
                      segment_idx_[i], i);
            return;
        }
    }

    if (!planNextWaypointSegment()) {
        ROS_ERROR("Failed to initialize first waypoint segment.");
        return;
    }

    a_star_planner_.Init(Eigen::Vector3d(plan_size_x_, plan_size_y_, plan_size_z_), resolution_, reserved_back_space_, safety_distance_, h_weight_);
    a_star_planner_.setDynamicObstacleParams(dynamic_obstacle_inflation_, dynamic_obstacle_endpoint_clear_radius_);
    a_star_planner_.setMaxSearchNodes(static_cast<size_t>(astar_max_search_nodes_));

    // 初始化MPC平滑器
    mpc_ = std::make_unique<MpcSmoother>(mpc_config_);

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    odom_suber = nh->subscribe<nav_msgs::Odometry>(
        odometry_topic_, 1, std::bind(&ABPlanner::pose_cb, this, std::placeholders::_1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());//状态真值，用于赛道一
    start_state_suber_ = nh->subscribe<std_msgs::Header>(fusion_start_topic_, 1, std::bind(&ABPlanner::start_state_cb, this, std::placeholders::_1));
    obstacle_suber_ = nh->subscribe<lidar_solver::ObstacleArray>(
        obstacle_topic_, 1, std::bind(&ABPlanner::obstacle_cb, this, std::placeholders::_1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    //publisher
    control_cmd_pub_ = nh->advertise<quadrotor_msgs::PositionCommand>("pose_cmd", 1);
    if (publish_rviz_visualization_) {
        planned_path_pub_ = pnh_.advertise<nav_msgs::Path>("planned_path", 1, true);
        bspline_path_pub_ = pnh_.advertise<nav_msgs::Path>("bspline_path", 1, true);
        mpc_prediction_path_pub_ = pnh_.advertise<nav_msgs::Path>("mpc_prediction_path", 1);
    }

    // 创建状态判断定时器，周期为0.1秒，回调函数为PlannerCallback，用于定时检查无人机状态并进行路径规划
    planner_timer_ = nh->createTimer(ros::Duration(0.02), std::bind(&ABPlanner::PlannerCallback, this, std::placeholders::_1));
}

ABPlanner::~ABPlanner()
{
}

void ABPlanner::PlannerCallback(const ros::TimerEvent& event)
{
    switch (planner_state_) {
        case IDLE:
            // 在IDLE状态下，等待融合节点发布IMU时间戳对齐完成信号和第一帧对齐后的里程计
            if (have_start_state_ && have_odom_ && !time_waited_after_start_state_) {
                if (start_state_time_.isZero()) {
                    start_state_time_ = latest_odom_stamp_;
                    ROS_WARN("Fusion start signal has no stamp. Using first aligned odom stamp %.9f as start time.",
                             start_state_time_.toSec());
                }

                const double waited_time = (latest_odom_stamp_ - start_state_time_).toSec();
                if (waited_time < 2.0) {
                    ROS_INFO_THROTTLE(1.0,
                        "Received fusion start signal at aligned time %.9f, waiting %.2f/2.00 s on aligned odom clock before switching to TOPLAN.",
                        start_state_time_.toSec(), std::max(0.0, waited_time));
                    break;
                }

                time_waited_after_start_state_ = true;
                ROS_INFO_THROTTLE(1.0, "Waited %.2f seconds on aligned odom clock after fusion start signal. Ready to switch to TOPLAN state.",
                         waited_time);
            }

            if (time_waited_after_start_state_ && have_odom_) {
                planner_state_ = TOPLAN;
                ROS_INFO_THROTTLE(1.0, "Fusion time aligned at %.9f and odom ready at %.9f. Switching to TOPLAN state.",
                         start_state_time_.toSec(), latest_odom_stamp_.toSec());
            }
            break;
        case TOPLAN:
        {
            auto hasExecutableRemaining = [&]() {
                if (bsplines_.empty() || arc_length_tables_.empty() || !have_odom_) {
                    return false;
                }
                const double u_start = current_arc_table.findClosestU(
                    current_position_, current_bspline, last_u_, 0.0, current_bspline.maxU());
                const double current_s = current_arc_table.getLength(u_start);
                const double total_s = current_arc_table.getLength(current_bspline.maxU());
                return current_s < total_s - 1.0;
            };

            const ros::Time now = alignedStamp();
            if (!next_planning_attempt_time_.isZero() && now < next_planning_attempt_time_) {
                if (hasExecutableRemaining()) {
                    planner_state_ = EXECUTING;
                    ROS_WARN_THROTTLE(1.0, "Planning is cooling down after a failed attempt. Continuing existing B-spline instead of holding.");
                } else {
                    publishControlCommand(false, Eigen::MatrixXd(), Eigen::MatrixXd());
                    ROS_WARN_THROTTLE(1.0, "Planning is cooling down after a failed attempt. Publishing hold command until next retry.");
                }
                break;
            }

            if (!hasExecutableRemaining()) {
                publishControlCommand(false, Eigen::MatrixXd(), Eigen::MatrixXd());
            }

            // 在TOPLAN状态下，进行路径规划，如果规划成功则切换到EXECUTING状态
            bool planning_success = planNextWaypoint(); // 调用路径规划函数，返回是否规划成功
            if (planning_success) {
                consecutive_planning_failures_ = 0;
                recovery_nudge_attempt_count_ = 0;
                next_planning_attempt_time_ = ros::Time(0);
                planner_state_ = EXECUTING;
                ROS_INFO_THROTTLE(1.0, "Path planning successful. Switching to EXECUTING state.");
            }
            else {
                consecutive_planning_failures_++;
                next_planning_attempt_time_ = now + ros::Duration(planning_failure_retry_interval_);
                const bool has_executable_remaining = hasExecutableRemaining();
                const bool appears_stuck = current_velocity_.norm() <= planning_recovery_stuck_speed_;
                const bool should_recover =
                    planning_recovery_enabled_ &&
                    consecutive_planning_failures_ >= planning_recovery_failure_threshold_ &&
                    (!has_executable_remaining || appears_stuck || obstacle_path_threatened_);

                ROS_WARN("Path planning failed: consecutive=%d, has_executable_remaining=%s, speed=%.2f, stuck_threshold=%.2f, path_threatened=%s, should_recover=%s.",
                         consecutive_planning_failures_,
                         has_executable_remaining ? "true" : "false",
                         current_velocity_.norm(),
                         planning_recovery_stuck_speed_,
                         obstacle_path_threatened_ ? "true" : "false",
                         should_recover ? "true" : "false");

                if (should_recover && startRecoveryNudge()) {
                    next_planning_attempt_time_ = ros::Time(0);
                    planner_state_ = RECOVERY_NUDGE;
                    ROS_WARN("Path planning failed %d time(s). Starting recovery nudge before retrying A*.",
                             consecutive_planning_failures_);
                } else if (has_executable_remaining) {
                    planner_state_ = EXECUTING;
                    ROS_WARN("Path planning failed, but an existing executable B-spline is available and recovery is not due. Continuing previous trajectory instead of stopping in TOPLAN.");
                } else {
                    publishControlCommand(false, Eigen::MatrixXd(), Eigen::MatrixXd());
                    ROS_WARN("Path planning failed and no executable B-spline remains. Holding current position while waiting for a valid replan.");
                }
            }
            break;
        }
        case EXECUTING:
        {
            // 在EXECUTING状态下，执行规划好的路径，如果完成则继续规划下一段路径，或者如果所有路径都执行完成则切换回IDLE状态
            if (bsplines_.empty() || arc_length_tables_.empty()) {
                ROS_WARN_THROTTLE(1.0, "No B-spline trajectory available for MPC tracking.");
                planner_state_ = TOPLAN;
                break;
            }

            while (current_waypoint_idx_ + 1 < static_cast<int>(segment_waypoints_.size())) {
                const Eigen::Vector3d prev_waypoint = segment_waypoints_[current_waypoint_idx_];
                const Eigen::Vector3d next_waypoint = segment_waypoints_[current_waypoint_idx_ + 1];
                const Eigen::Vector3d segment = next_waypoint - prev_waypoint;
                const double segment_len_sq = segment.squaredNorm();
                const double dist_to_next = (current_position_ - next_waypoint).norm();
                const double progress_ratio = segment_len_sq > 1e-6
                    ? (current_position_ - prev_waypoint).dot(segment) / segment_len_sq
                    : 0.0;

                if (dist_to_next > waypoint_reached_distance_ && progress_ratio < 1.0) {
                    break;
                }

                current_waypoint_idx_++;
                ROS_INFO_THROTTLE(1.0, "Passed waypoint %d/%lu, dist=%.2f m, progress=%.2f, threshold=%.2f m.",
                         current_waypoint_idx_, segment_waypoints_.size() - 1,
                         dist_to_next, progress_ratio, waypoint_reached_distance_);
            }
            // 选取当前弧长表参数对应的参考轨迹点，生成MPC控制器的参考轨迹，并调用MPC控制器发出预期指令
            // 调用MPC控制器发出预期指令
            // 提取当前状态
            Eigen::Matrix<double, 9, 1> current_state = extractState();
            // 将当前位置投影到 B 样条，得到当前的参数 u_start
            double u_start = current_arc_table.findClosestU(current_state.segment<3>(0), current_bspline, last_u_, 0.0, current_bspline.maxU());
            last_u_ = u_start; // 更新 last_u_，用于下一次寻找最近点时的初始猜测，增加连续性和效率
            const double current_s = current_arc_table.getLength(u_start);
            const double total_s = current_arc_table.getLength(current_bspline.maxU());
            const double remaining_s = std::max(0.0, total_s - current_s);
            const bool final_waypoint_in_segment =
                planned_window_end_idx_ >= static_cast<int>(segment_waypoints_.size()) - 1;
            bool replan_after_command = false;

            if (current_s >= total_s - 1.0 && current_waypoint_idx_ < planned_window_end_idx_) {
                current_waypoint_idx_ = planned_window_end_idx_;
                ROS_WARN("Reached the end of current B-spline window before waypoint distance trigger. Force advancing passed waypoint index to %d.",
                         current_waypoint_idx_);
            }

            if (current_s >= total_s - 1.0 &&
                (planned_window_end_idx_ < static_cast<int>(segment_waypoints_.size()) - 1 ||
                 current_segment_ < static_cast<int>(segment_idx_.size()))) {
                if (planned_window_end_idx_ >= static_cast<int>(segment_waypoints_.size()) - 1 &&
                    current_segment_ < static_cast<int>(segment_idx_.size()) &&
                    segment_turnaround_enabled_) {
                    startSegmentTurnaround();
                    ROS_WARN("Current B-spline window is exhausted at segment end. Holding final point before planning next segment.");
                } else {
                    planner_state_ = TOPLAN;
                    ROS_WARN("Current B-spline window is exhausted. Replanning immediately instead of tracking stale reference.");
                }
                break;
            }

            if (current_waypoint_idx_ >= static_cast<int>(segment_waypoints_.size()) - 1) {
                if (current_segment_ < static_cast<int>(segment_idx_.size()) &&
                    segment_turnaround_enabled_) {
                    startSegmentTurnaround();
                    ROS_INFO_THROTTLE(1.0, "Current segment execution completed. Holding final point and turning around before planning next segment.");
                    break;
                }

                if (current_segment_ < static_cast<int>(segment_idx_.size()) ||
                    isPathExecutionComplete(current_position_, segment_waypoints_.back())) {
                    planner_state_ = TOPLAN; // 切换回TOPLAN状态，继续规划下一段路径
                    current_bspline_segment_idx_ = 0; // 重置当前正在执行路径段的索引
                    ROS_INFO_THROTTLE(1.0, "Current segment execution completed. Planning next segment.");
                    break;
                }
            }

            if (!final_waypoint_in_segment && remaining_s <= replan_trigger_distance_) {
                replan_after_command = true;
                ROS_INFO_THROTTLE(1.0,
                    "Remaining path length %.2f m is below %.2f m. Will roll to next waypoint window after this command.",
                    remaining_s, replan_trigger_distance_);
            }
            // 生成参考轨迹，长度为 MPC 预测步数 N，时间间隔为 dt
            Eigen::MatrixXd ref_traj = generateContinuousRefTraj(u_start, mpc_config_.N, mpc_config_.dt);
            // 调用 MPC 优化器，得到优化后的控制命令
            Eigen::MatrixXd out_states, out_inputs;
            const Eigen::Matrix<double, 9, 1> mpc_state = sanitizeMpcState(current_state);
            std::vector<MpcObstacle> mpc_obstacles;
            const std::vector<BoxObstacle> dynamic_obstacles = freshDynamicObstacles();
            const int max_mpc_obstacles = std::max(0, mpc_config_.obstacle_max_count);
            if (mpc_config_.obstacle_constraints_enabled && max_mpc_obstacles > 0) {
                std::vector<std::pair<double, MpcObstacle>> scored_obstacles;
                scored_obstacles.reserve(dynamic_obstacles.size());
                auto obstacleDistanceToRefTraj = [&](const MpcObstacle& obstacle) {
                    double min_distance_sq = std::numeric_limits<double>::infinity();
                    for (int col = 0; col < ref_traj.cols(); col += std::max(1, mpc_config_.obstacle_check_step)) {
                        const Eigen::Vector3d ref_point = ref_traj.block<3, 1>(0, col);
                        min_distance_sq = std::min(min_distance_sq, (obstacle.center - ref_point).squaredNorm());
                    }
                    if (ref_traj.cols() > 0) {
                        const Eigen::Vector3d ref_point = ref_traj.block<3, 1>(0, ref_traj.cols() - 1);
                        min_distance_sq = std::min(min_distance_sq, (obstacle.center - ref_point).squaredNorm());
                    }
                    return min_distance_sq;
                };
                for (const auto& obstacle : dynamic_obstacles) {
                    MpcObstacle mpc_obstacle;
                    mpc_obstacle.center = obstacle.center;
                    mpc_obstacle.size = obstacle.size;
                    mpc_obstacle.orientation = obstacle.orientation;
                    scored_obstacles.emplace_back(obstacleDistanceToRefTraj(mpc_obstacle), mpc_obstacle);
                }
                if (static_cast<int>(scored_obstacles.size()) > max_mpc_obstacles) {
                    std::nth_element(scored_obstacles.begin(),
                                     scored_obstacles.begin() + max_mpc_obstacles,
                                     scored_obstacles.end(),
                                     [](const auto& lhs, const auto& rhs) {
                                         return lhs.first < rhs.first;
                                     });
                    scored_obstacles.resize(max_mpc_obstacles);
                }
                std::sort(scored_obstacles.begin(), scored_obstacles.end(),
                          [](const auto& lhs, const auto& rhs) {
                              return lhs.first < rhs.first;
                          });
                mpc_obstacles.reserve(scored_obstacles.size());
                for (const auto& scored_obstacle : scored_obstacles) {
                    mpc_obstacles.push_back(scored_obstacle.second);
                }
            }
            mpc_->setObstacles(mpc_obstacles);
            bool mpc_success = mpc_->solve(mpc_state, ref_traj, out_states, out_inputs);
            bool mpc_progressing = mpc_success && isMpcPredictionProgressing(out_states, ref_traj);
            if (mpc_success && !mpc_progressing && !mpc_obstacles.empty()) {
                ROS_WARN_THROTTLE(1.0,
                    "MPC prediction is not progressing with obstacle constraints. Retrying this control tick without MPC obstacle constraints.");
                mpc_->setObstacles({});
                mpc_success = mpc_->solve(mpc_state, ref_traj, out_states, out_inputs);
                mpc_progressing = mpc_success && isMpcPredictionProgressing(out_states, ref_traj);
            }

            if (mpc_success && mpc_progressing) {
                publishMpcPredictionPath(out_states);
                // 发布控制命令，执行路径跟踪
                publishControlCommand(true, out_states, out_inputs);
            } else if (mpc_success) {
                publishMpcPredictionPath(ref_traj);
                publishReferenceFallbackCommand(ref_traj, "MPC solved but predicted trajectory did not move forward");
            } else {
                publishMpcPredictionPath(ref_traj);
                publishReferenceFallbackCommand(ref_traj, "MPC solve failed");
                planner_state_ = TOPLAN;
                ROS_WARN_THROTTLE(1.0, "MPC solve failed with current constraints. Triggering replanning instead of staying on stale trajectory.");
            }
            if (replan_after_command) {
                planner_state_ = TOPLAN;
            }
            else if (shouldReplanForObstacles(alignedStamp())) {
                last_obstacle_replan_time_ = alignedStamp();
                planner_state_ = TOPLAN;
                ROS_WARN_THROTTLE(1.0,
                    "Fresh lidar obstacle data triggered periodic dynamic-avoidance replanning from current odom.");
            }
            break;
        }
        case SEGMENT_TURNING:
        {
            publishSegmentTurnaroundCommand();
            const double elapsed = (alignedStamp() - segment_turnaround_start_stamp_).toSec();
            if (elapsed >= segment_turnaround_duration_) {
                last_yaw_ = segment_turnaround_target_yaw_;
                last_yaw_command_stamp_ = alignedStamp();
                have_last_yaw_command_ = true;
                planner_state_ = TOPLAN;
                current_bspline_segment_idx_ = 0; // 重置当前正在执行路径段的索引
                ROS_INFO_THROTTLE(1.0, "Segment turnaround completed in %.2f s. Planning next segment.", elapsed);
            }
            break;
        }
        case RECOVERY_NUDGE:
        {
            publishRecoveryNudgeCommand();
            const double elapsed = (alignedStamp() - recovery_nudge_start_stamp_).toSec();
            const double distance_to_target = (current_position_ - recovery_nudge_target_position_).norm();
            if (distance_to_target <= planning_recovery_reached_distance_ ||
                elapsed >= planning_recovery_duration_) {
                planner_state_ = TOPLAN;
                next_planning_attempt_time_ = ros::Time(0);
                consecutive_planning_failures_ = 0;
                ROS_WARN("Recovery nudge completed: elapsed=%.2f s, distance_to_target=%.2f m. Retrying A* from new odom.",
                         elapsed, distance_to_target);
            }
            break;
        }
        default:
            ROS_ERROR("Unknown planner state!");
    }
}

void ABPlanner::pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (msg->header.stamp.isZero()) {
        ROS_WARN_THROTTLE(1.0, "Ignore odom with zero header.stamp because AB_planner uses odom/IMU-aligned time only.");
        return;
    }

    latest_odom_stamp_ = msg->header.stamp;
    have_odom_ = true;

    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    // 使用 std::atan2 稳定提取偏航角，避免 Eigen eulerAngles 带来边界突变问题
    double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    current_position_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const Eigen::Vector3d measured_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    if (have_last_odom_acc_) {
        const double dt = (latest_odom_stamp_ - last_odom_acc_stamp_).toSec();
        if (dt > 1e-4 && dt < 0.2) {
            const Eigen::Vector3d raw_acceleration = (measured_velocity - last_odom_velocity_) / dt;
            current_acceleration_ = (1.0 - acc_lpf_alpha_) * current_acceleration_ + acc_lpf_alpha_ * raw_acceleration;
        } else if (dt <= 0.0) {
            ROS_WARN_THROTTLE(1.0, "Ignore non-increasing odom stamp for acceleration estimation. dt=%.9f", dt);
        }
    } else {
        current_acceleration_.setZero();
        have_last_odom_acc_ = true;
    }
    last_odom_velocity_ = measured_velocity;
    last_odom_acc_stamp_ = latest_odom_stamp_;
    current_velocity_ = measured_velocity;
    current_orientation_ = q;
    current_yaw_ = yaw;
    current_angular_velocity_ = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
}

void ABPlanner::start_state_cb(const std_msgs::Header::ConstPtr& msg)
{
    if (!have_start_state_)
    {
        start_state_time_ = msg->stamp;
        have_start_state_ = true;
        if (start_state_time_.isZero()) {
            ROS_WARN("Received fusion start signal with zero stamp. Waiting for first aligned odom stamp to initialize start time.");
        } else {
            ROS_INFO_THROTTLE(1.0, "Received fusion start signal at aligned time %.9f.", start_state_time_.toSec());
        }
    }
}

void ABPlanner::obstacle_cb(const lidar_solver::ObstacleArray::ConstPtr& msg)
{
    if (!have_odom_ || latest_odom_stamp_.isZero()) {
        ROS_WARN_THROTTLE(1.0, "Ignore obstacle frame before aligned odom is available.");
        return;
    }

    if (!msg->header.stamp.isZero()) {
        const double stamp_delta = (latest_odom_stamp_ - msg->header.stamp).toSec();
        if (std::abs(stamp_delta) > obstacle_stale_time_) {
            ROS_WARN_THROTTLE(1.0,
                "Obstacle stamp differs from aligned odom time by %.3f s. Re-stamping obstacle frame to aligned odom time.",
                stamp_delta);
        }
    }

    latest_obstacle_stamp_ = latest_odom_stamp_;
    ObstacleFrame frame;
    frame.stamp = latest_obstacle_stamp_;
    frame.obstacles.reserve(msg->obstacles.size());

    for (const auto& obstacle_msg : msg->obstacles) {
        BoxObstacle obstacle;
        obstacle.center = Eigen::Vector3d(
            obstacle_msg.pose.position.x,
            obstacle_msg.pose.position.y,
            obstacle_msg.pose.position.z);
        obstacle.size = Eigen::Vector3d(
            std::max(0.0, obstacle_msg.scale.x),
            std::max(0.0, obstacle_msg.scale.y),
            std::max(0.0, obstacle_msg.scale.z));
        obstacle.orientation = Eigen::Quaterniond(
            obstacle_msg.pose.orientation.w,
            obstacle_msg.pose.orientation.x,
            obstacle_msg.pose.orientation.y,
            obstacle_msg.pose.orientation.z);

        if (!std::isfinite(obstacle.center.x()) || !std::isfinite(obstacle.center.y()) || !std::isfinite(obstacle.center.z()) ||
            !std::isfinite(obstacle.size.x()) || !std::isfinite(obstacle.size.y()) || !std::isfinite(obstacle.size.z())) {
            continue;
        }

        frame.obstacles.push_back(obstacle);
    }

    const size_t frame_obstacle_count = frame.obstacles.size();
    obstacle_history_.push_back(std::move(frame));

    static int consecutive_empty_obstacle_frames = 0;
    if (frame_obstacle_count == 0) {
        consecutive_empty_obstacle_frames++;
    } else {
        consecutive_empty_obstacle_frames = 0;
    }

    if (clear_obstacle_history_on_empty_frame_ && consecutive_empty_obstacle_frames >= 3) {
        obstacle_history_.clear();
        latest_dynamic_obstacles_.clear();
        obstacle_threat_hit_count_ = 0;
        obstacle_threat_clear_count_ = obstacle_threat_clear_frames_;
        obstacle_path_threatened_ = false;
        have_obstacles_ = true;
        ROS_INFO_THROTTLE(1.0,
                          "Received 3 consecutive empty obstacle frames. Cleared obstacle history and active dynamic obstacles.");
        return;
    }

    pruneObstacleHistory(latest_obstacle_stamp_);
    updateActiveObstaclesFromHistory();
    const bool raw_path_threatened = isCurrentBsplineThreatenedByObstacles(latest_dynamic_obstacles_);
    if (raw_path_threatened) {
        obstacle_threat_hit_count_++;
        obstacle_threat_clear_count_ = 0;
        if (!obstacle_path_threatened_ &&
            obstacle_threat_hit_count_ >= obstacle_threat_confirm_frames_) {
            obstacle_path_threatened_ = true;
            ROS_WARN("Current B-spline threat confirmed after %d obstacle frame(s).",
                     obstacle_threat_hit_count_);
        }
    } else {
        obstacle_threat_clear_count_++;
        obstacle_threat_hit_count_ = 0;
        if (obstacle_path_threatened_ &&
            obstacle_threat_clear_count_ >= obstacle_threat_clear_frames_) {
            obstacle_path_threatened_ = false;
            ROS_INFO("Current B-spline threat cleared after %d safe obstacle frame(s).",
                     obstacle_threat_clear_count_);
        }
    }
    have_obstacles_ = true;
    ROS_INFO_THROTTLE(1.0,
                      "Received obstacle frame. frame_obstacles=%lu, active=%lu, history_frames=%lu/%d, raw_path_threatened=%s, current_path_threatened=%s, hit_frames=%d, clear_frames=%d.",
                      static_cast<unsigned long>(frame_obstacle_count),
                      static_cast<unsigned long>(latest_dynamic_obstacles_.size()),
                      static_cast<unsigned long>(obstacle_history_.size()),
                      obstacle_history_max_frames_,
                      raw_path_threatened ? "true" : "false",
                      obstacle_path_threatened_ ? "true" : "false",
                      obstacle_threat_hit_count_, obstacle_threat_clear_count_);
}

void ABPlanner::pruneObstacleHistory(const ros::Time& now)
{
    while (static_cast<int>(obstacle_history_.size()) > obstacle_history_max_frames_) {
        obstacle_history_.pop_front();
    }

    while (!obstacle_history_.empty()) {
        const double age = (now - obstacle_history_.front().stamp).toSec();
        if (age <= obstacle_history_time_) {
            break;
        }
        obstacle_history_.pop_front();
    }
}

void ABPlanner::updateActiveObstaclesFromHistory()
{
    latest_dynamic_obstacles_.clear();

    const double merge_distance_sq = obstacle_merge_distance_ * obstacle_merge_distance_;
    for (const auto& frame : obstacle_history_) {
        for (const auto& obstacle : frame.obstacles) {
            if (!std::isfinite(obstacle.center.x()) || !std::isfinite(obstacle.center.y()) || !std::isfinite(obstacle.center.z()) ||
                !std::isfinite(obstacle.size.x()) || !std::isfinite(obstacle.size.y()) || !std::isfinite(obstacle.size.z())) {
                continue;
            }

            auto merge_it = latest_dynamic_obstacles_.end();
            if (obstacle_merge_distance_ > 1e-6) {
                merge_it = std::find_if(latest_dynamic_obstacles_.begin(),
                                        latest_dynamic_obstacles_.end(),
                                        [&](const BoxObstacle& existing) {
                                            return (existing.center - obstacle.center).squaredNorm() <= merge_distance_sq;
                                        });
            }

            if (merge_it == latest_dynamic_obstacles_.end()) {
                latest_dynamic_obstacles_.push_back(obstacle);
                continue;
            }

            const Eigen::Vector3d existing_half = 0.5 * merge_it->size.cwiseMax(Eigen::Vector3d::Zero());
            const Eigen::Vector3d obstacle_half = 0.5 * obstacle.size.cwiseMax(Eigen::Vector3d::Zero());
            const Eigen::Vector3d min_corner = (merge_it->center - existing_half).cwiseMin(obstacle.center - obstacle_half);
            const Eigen::Vector3d max_corner = (merge_it->center + existing_half).cwiseMax(obstacle.center + obstacle_half);
            merge_it->center = 0.5 * (min_corner + max_corner);
            merge_it->size = (max_corner - min_corner).cwiseMax(Eigen::Vector3d::Zero());
        }
    }
}

bool ABPlanner::isCurrentBsplineThreatenedByObstacles(const std::vector<BoxObstacle>& obstacles) const
{
    if (!dynamic_avoidance_enabled_ || !immediate_replan_on_collision_ ||
        obstacles.empty() || bsplines_.empty() || arc_length_tables_.empty()) {
        return false;
    }

    std::vector<Eigen::Vector3d> clearance_points;
    if (have_odom_) {
        clearance_points.push_back(current_position_);
    }
    if (!segment_waypoints_.empty()) {
        clearance_points.push_back(segment_waypoints_.back());
    }

    return !isBsplineCollisionFree(current_bspline, obstacles, bspline_collision_sample_step_, clearance_points);
}

bool ABPlanner::hasFreshBoxObstacleData() const
{
    if (!dynamic_avoidance_enabled_ || !have_obstacles_ || latest_obstacle_stamp_.isZero()) {
        return false;
    }

    return (alignedStamp() - latest_obstacle_stamp_).toSec() <= obstacle_stale_time_;
}

bool ABPlanner::hasFreshObstacleData() const
{
    return hasFreshBoxObstacleData();
}

std::vector<BoxObstacle> ABPlanner::freshDynamicObstacles() const
{
    if (!hasFreshBoxObstacleData()) {
        return {};
    }

    return latest_dynamic_obstacles_;
}

bool ABPlanner::shouldReplanForObstacles(const ros::Time& now) const
{
    if (!hasFreshObstacleData()) {
        return false;
    }
    const bool has_fresh_box_obstacles = hasFreshBoxObstacleData() && !latest_dynamic_obstacles_.empty();
    if (!has_fresh_box_obstacles && !last_plan_used_dynamic_obstacles_) {
        return false;
    }

    if (immediate_replan_on_collision_ && obstacle_path_threatened_) {
        if (last_obstacle_replan_time_.isZero() ||
            (now - last_obstacle_replan_time_).toSec() >= obstacle_min_replan_interval_) {
            return true;
        }
    }

    if (last_obstacle_replan_time_.isZero()) {
        return true;
    }

    return (now - last_obstacle_replan_time_).toSec() >= obstacle_replan_period_;
}

void ABPlanner::readWaypointsFromFile(const std::string& filename, std::vector<Eigen::Vector3d>& waypoints)
{
    // 数据格式:
    // # p_x, p_y, p_z, q_x, q_y, q_z, q_w
    // 0.0, 0.0, -1.0, 0.001, 0.014, -0.001, 1.000
    // 25.670, -0.008, -3.754, 0.001, 0.014, -0.001, 1.000
    // 44.008, 0.039, -3.819, 0.000, -0.001, 0.002, 1.000
    // 67.270, 1.870, -5.100, -0.000, 0.013, 0.003, 1.000
    // 85.020, 3.682, -5.875, -0.000, 0.011, 0.014, 1.000

    waypoints.clear();
    yaws_.clear();

    std::ifstream infile(filename);
    if (!infile.is_open()) {
        ROS_ERROR("Failed to open waypoints file: %s", filename.c_str());
        return;
    }
    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') {
            continue; // 跳过空行和注释行
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        double p_x, p_y, p_z, q_x, q_y, q_z, q_w;
        if (!(iss >> p_x >> p_y >> p_z >> q_x >> q_y >> q_z >> q_w)) {
            ROS_WARN("Invalid waypoint format: %s", line.c_str());
            continue; // 跳过格式错误的行
        }
        waypoints.emplace_back(p_x, p_y, p_z);
        const double yaw = std::atan2(2.0 * (q_w * q_z + q_x * q_y),
                                      1.0 - 2.0 * (q_y * q_y + q_z * q_z));
        yaws_.push_back(yaw);
    }
    infile.close();

    ROS_INFO_THROTTLE(1.0, "Loaded %lu waypoints and %lu yaw references from file: %s",
             waypoints.size(), yaws_.size(), filename.c_str());
}

bool ABPlanner::planNextWaypointSegment()
{
    // 规划当前分段的路径点列表，单位为世界坐标
    if (current_segment_ >= segment_idx_.size()) {
        ROS_INFO_THROTTLE(1.0, "All segments have been planned.");
        return false; // 所有分段都已规划完成
    }
    const int start_idx = (current_segment_ == 0) ? 0 : std::max(0, segment_idx_[current_segment_ - 1] - 1);
    const int end_idx = segment_idx_[current_segment_];
    segment_waypoints_.assign(waypoints_.begin() + start_idx, waypoints_.begin() + end_idx);
    skipped_segment_waypoints_.assign(segment_waypoints_.size(), false);
    if (yaws_.size() == waypoints_.size()) {
        segment_yaws_.assign(yaws_.begin() + start_idx, yaws_.begin() + end_idx);
    } else {
        segment_yaws_.clear();
        ROS_WARN("Yaw reference size (%lu) does not match waypoint size (%lu). Falling back to velocity-direction yaw.",
                 yaws_.size(), waypoints_.size());
    }
    current_waypoint_idx_ = 0; // 重置当前规划路径段中正在规划的路径点索引
    planned_window_end_idx_ = 0;
    current_bspline_segment_idx_ = 0; // 重置当前正在执行路径段的索引
    ROS_INFO_THROTTLE(1.0, "Loaded waypoint segment %d: global waypoint [%d, %d).",
             current_segment_, start_idx, end_idx);
    current_segment_++; // 切换到下一段
    return true;
}

bool ABPlanner::planNextWaypoint()
{
    while (current_waypoint_idx_ + 1 < static_cast<int>(segment_waypoints_.size()) &&
           current_waypoint_idx_ + 1 < static_cast<int>(skipped_segment_waypoints_.size()) &&
           skipped_segment_waypoints_[current_waypoint_idx_ + 1]) {
        current_waypoint_idx_++;
        ROS_WARN("Advance over skipped unreachable waypoint %d in current segment.", current_waypoint_idx_);
    }

    if (current_waypoint_idx_ >= static_cast<int>(segment_waypoints_.size()) - 1) {
        ROS_INFO_THROTTLE(1.0, "Current segment planned. Moving to next segment.");
        bool next_segment_planned = planNextWaypointSegment(); // 规划下一段
        if (!next_segment_planned) {
            return false; // 所有分段都已规划完成
        }
        bool planning_success = planNextWaypoint(); // 递归调用规划下一段的路径
        return planning_success;
    }

    // 滚动前视规划：中间航点只作为经过点，不作为停车点。
    // 注意 current_waypoint_idx_ 只表示实际已经通过的航点，不能在规划成功时直接跳到窗口末端。
    const int start_idx = current_waypoint_idx_;
    const int target_idx = std::min(start_idx + plan_waypoint_lookahead_,
                                    static_cast<int>(segment_waypoints_.size()) - 1);
    auto buildPlanWaypoints = [&](int end_idx, const std::vector<bool>& skip_mask) {
        std::vector<Eigen::Vector3d> plan_waypoints;
        plan_waypoints.push_back(have_odom_ ? current_position_ : segment_waypoints_[start_idx]);
        for (int idx = start_idx + 1; idx <= end_idx; ++idx) {
            if (idx < static_cast<int>(skip_mask.size()) && skip_mask[idx]) {
                continue;
            }
            plan_waypoints.push_back(segment_waypoints_[idx]);
        }
        return plan_waypoints;
    };

    const std::vector<bool> previous_skipped_segment_waypoints = skipped_segment_waypoints_;
    const int previous_planned_window_end_idx = planned_window_end_idx_;
    const Eigen::Vector3d previous_target_goal = target_goal_;

    std::vector<Eigen::Vector3d> current_plan_waypoints = buildPlanWaypoints(target_idx, skipped_segment_waypoints_);

    // 调用A*算法进行路径规划，输入起点和目标点列表，输出路径点列表，单位为世界坐标
    std::vector<std::vector<Eigen::Vector3d>> planned_paths; // 存储规划得到的路径点列表，单位为世界坐标
    const std::vector<BoxObstacle> dynamic_obstacles = freshDynamicObstacles();
    a_star_planner_.setDynamicObstacles(dynamic_obstacles);
    last_plan_used_dynamic_obstacles_ = !dynamic_obstacles.empty();
    ROS_INFO_THROTTLE(1.0, "Planning from current odom to waypoint window end with %lu box obstacle(s).",
                      static_cast<unsigned long>(dynamic_obstacles.size()));
    bool success = a_star_planner_.planPaths(current_plan_waypoints, planned_paths);
    int successful_target_idx = target_idx;
    std::vector<bool> successful_skip_mask = skipped_segment_waypoints_;

    if (!success && target_idx < static_cast<int>(segment_waypoints_.size()) - 1) {
        std::vector<bool> trial_skip_mask = skipped_segment_waypoints_;
        for (int candidate_idx = target_idx + 1; candidate_idx < static_cast<int>(segment_waypoints_.size()); ++candidate_idx) {
            trial_skip_mask[candidate_idx - 1] = true;
            current_plan_waypoints = buildPlanWaypoints(candidate_idx, trial_skip_mask);
            if (current_plan_waypoints.size() < 2) {
                continue;
            }

            planned_paths.clear();
            success = a_star_planner_.planPaths(current_plan_waypoints, planned_paths);
            if (success) {
                successful_skip_mask = trial_skip_mask;
                successful_target_idx = candidate_idx;
                ROS_WARN("Skipped unreachable waypoint(s) up to %d and planned directly to waypoint %d.",
                         candidate_idx - 1, successful_target_idx);
                break;
            }
        }
    }

    if (success) {
        if (!buildAndPublishBsplinePath(planned_paths)) {
            skipped_segment_waypoints_ = previous_skipped_segment_waypoints;
            planned_window_end_idx_ = previous_planned_window_end_idx;
            target_goal_ = previous_target_goal;
            ROS_WARN("A* path exists, but no collision-free B-spline was built. Rejecting this plan and staying in TOPLAN.");
            return false;
        }
        skipped_segment_waypoints_ = successful_skip_mask;
        planned_window_end_idx_ = successful_target_idx;
        target_goal_ = segment_waypoints_[successful_target_idx]; // 当前滚动窗口的末端目标点
        // 发送rviz可视化路径，调试用；只发布已经通过 B-spline 碰撞检查的计划。
        publishPlannedPath(planned_paths);
        last_obstacle_replan_time_ = alignedStamp();
        obstacle_path_threatened_ = false;
        ROS_INFO_THROTTLE(1.0, "Planned rolling waypoint window: passed=%d, planned_end=%d, goal=(%.2f, %.2f, %.2f).",
                 current_waypoint_idx_, planned_window_end_idx_, target_goal_.x(), target_goal_.y(), target_goal_.z());
    }
    return success;
}

void ABPlanner::publishPlannedPath(const std::vector<std::vector<Eigen::Vector3d>>& paths)
{
    if (!publish_rviz_visualization_) {
        return;
    }

    nav_msgs::Path path_msg;
    path_msg.header.stamp = alignedStamp();
    path_msg.header.frame_id = visualization_frame_;

    for (const auto& segment : paths) {
        for (size_t i = 0; i < segment.size(); ++i) {
            if (!path_msg.poses.empty() && i == 0) {
                const auto& last = path_msg.poses.back().pose.position;
                if ((Eigen::Vector3d(last.x, last.y, last.z) - segment[i]).norm() < 1e-6) {
                    continue;
                }
            }

            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = segment[i].x();
            pose.pose.position.y = segment[i].y();
            pose.pose.position.z = segment[i].z();
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
    }

    planned_path_pub_.publish(path_msg);
}

double ABPlanner::computePathLength(const std::vector<Eigen::Vector3d>& points) const
{
    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += (points[i] - points[i - 1]).norm();
    }
    return length;
}

std::vector<Eigen::Vector3d> ABPlanner::optimizePathForBspline(const std::vector<Eigen::Vector3d>& raw_points) const
{
    if (!path_optimization_enabled_ || path_optimization_iterations_ == 0 || raw_points.size() < 3) {
        return raw_points;
    }

    std::vector<Eigen::Vector3d> optimized = raw_points;
    std::vector<Eigen::Vector3d> next = optimized;

    const double max_deviation = std::max(path_optimization_max_deviation_, resolution_);
    const double min_xy = std::max(resolution_, 1e-3);

    for (int iter = 0; iter < path_optimization_iterations_; ++iter) {
        next = optimized;

        for (size_t i = 1; i + 1 < optimized.size(); ++i) {
            const Eigen::Vector3d neighbor_mid = 0.5 * (optimized[i - 1] + optimized[i + 1]);
            Eigen::Vector3d candidate = optimized[i];

            candidate += path_optimization_data_weight_ * (raw_points[i] - optimized[i]);
            candidate += path_optimization_smooth_weight_ * (neighbor_mid - optimized[i]);
            candidate.z() += path_optimization_z_smooth_weight_ * (neighbor_mid.z() - optimized[i].z());

            if (path_optimization_min_turn_radius_ > 1e-3) {
                const Eigen::Vector3d a = optimized[i] - optimized[i - 1];
                const Eigen::Vector3d b = optimized[i + 1] - optimized[i];
                const double len_a = a.norm();
                const double len_b = b.norm();
                const double chord = (optimized[i + 1] - optimized[i - 1]).norm();
                const double area2 = a.cross(b).norm();

                if (len_a > 1e-6 && len_b > 1e-6 && chord > 1e-6 && area2 > 1e-6) {
                    const double radius = len_a * len_b * chord / (2.0 * area2);
                    if (radius < path_optimization_min_turn_radius_) {
                        const double alpha = std::clamp(
                            (path_optimization_min_turn_radius_ - radius) /
                            std::max(path_optimization_min_turn_radius_, 1e-6),
                            0.0, 0.5);
                        candidate += alpha * (neighbor_mid - candidate);
                    }
                }
            }

            const Eigen::Vector3d deviation = candidate - raw_points[i];
            if (deviation.norm() > max_deviation) {
                candidate = raw_points[i] + deviation.normalized() * max_deviation;
            }

            next[i] = candidate;
        }

        optimized.swap(next);

        for (size_t i = 1; i < optimized.size(); ++i) {
            const Eigen::Vector3d prev = optimized[i - 1];
            const double xy_dist = std::max((optimized[i].head<2>() - prev.head<2>()).norm(), min_xy);
            const double max_dz = path_optimization_max_z_slope_ * xy_dist;
            optimized[i].z() = std::clamp(optimized[i].z(), prev.z() - max_dz, prev.z() + max_dz);
        }
        for (size_t i = optimized.size() - 1; i > 0; --i) {
            const Eigen::Vector3d prev = optimized[i];
            const double xy_dist = std::max((optimized[i - 1].head<2>() - prev.head<2>()).norm(), min_xy);
            const double max_dz = path_optimization_max_z_slope_ * xy_dist;
            optimized[i - 1].z() = std::clamp(optimized[i - 1].z(), prev.z() - max_dz, prev.z() + max_dz);
        }

        optimized.front() = raw_points.front();
        optimized.back() = raw_points.back();
        for (size_t i = 1; i + 1 < optimized.size(); ++i) {
            const Eigen::Vector3d deviation = optimized[i] - raw_points[i];
            if (deviation.norm() > max_deviation) {
                optimized[i] = raw_points[i] + deviation.normalized() * max_deviation;
            }
        }
    }

    ROS_INFO_THROTTLE(1.0, "Optimized A* path for B-spline: raw_len=%.2f m, optimized_len=%.2f m, points=%lu, max_dev=%.2f m, max_z_slope=%.2f, min_turn_radius=%.2f m.",
             computePathLength(raw_points), computePathLength(optimized), optimized.size(),
             max_deviation, path_optimization_max_z_slope_, path_optimization_min_turn_radius_);
    return optimized;
}

std::vector<Eigen::Vector3d> ABPlanner::makeBsplineControlPoints(const std::vector<Eigen::Vector3d>& points) const
{
    return makeBsplineControlPoints(points, bspline_control_point_spacing_);
}

std::vector<Eigen::Vector3d> ABPlanner::makeBsplineControlPoints(const std::vector<Eigen::Vector3d>& points,
                                                                 double spacing) const
{
    std::vector<Eigen::Vector3d> sampled_points;
    if (points.empty()) {
        return sampled_points;
    }

    const double control_spacing = std::max(resolution_, spacing);
    sampled_points.push_back(points.front());
    double accumulated = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        accumulated += (points[i] - points[i - 1]).norm();
        if (accumulated >= control_spacing) {
            sampled_points.push_back(points[i]);
            accumulated = 0.0;
        }
    }

    if ((sampled_points.back() - points.back()).norm() > 1e-6) {
        sampled_points.push_back(points.back());
    }

    std::vector<Eigen::Vector3d> control_points;
    control_points.reserve(sampled_points.size() + 4);

    // Cubic uniform B-spline does not interpolate endpoints unless the ends are clamped.
    // Repeating endpoints keeps the executable curve anchored at the current pose and goal.
    control_points.push_back(sampled_points.front());
    control_points.push_back(sampled_points.front());
    control_points.push_back(sampled_points.front());
    for (size_t i = 1; i + 1 < sampled_points.size(); ++i) {
        control_points.push_back(sampled_points[i]);
    }
    control_points.push_back(sampled_points.back());
    control_points.push_back(sampled_points.back());
    control_points.push_back(sampled_points.back());

    return control_points;
}

bool ABPlanner::isPointInsideInflatedObstacle(const Eigen::Vector3d& point,
                                              const std::vector<BoxObstacle>& obstacles) const
{
    const double clearance = dynamic_obstacle_inflation_ + safety_distance_ + bspline_collision_extra_clearance_;
    for (const auto& obstacle : obstacles) {
        if (!std::isfinite(obstacle.center.x()) || !std::isfinite(obstacle.center.y()) || !std::isfinite(obstacle.center.z()) ||
            !std::isfinite(obstacle.size.x()) || !std::isfinite(obstacle.size.y()) || !std::isfinite(obstacle.size.z())) {
            continue;
        }

        const Eigen::Vector3d half_size = 0.5 * obstacle.size.cwiseMax(Eigen::Vector3d::Zero()) +
                                          Eigen::Vector3d::Constant(clearance);
        
        Eigen::Matrix3d R_inv = obstacle.orientation.toRotationMatrix().transpose();
        const Eigen::Vector3d diff = (R_inv * (point - obstacle.center)).cwiseAbs();
        
        if (diff.x() <= half_size.x() && diff.y() <= half_size.y() && diff.z() <= half_size.z()) {
            return true;
        }
    }

    return false;
}

bool ABPlanner::isBsplineCollisionFree(const BSpline& bspline,
                                       const std::vector<BoxObstacle>& obstacles,
                                       double sample_step,
                                       const std::vector<Eigen::Vector3d>& clearance_points) const
{
    if (!bspline_collision_check_enabled_ || obstacles.empty()) {
        return true;
    }

    ArcLengthTable arc_length_table;
    const int table_samples = std::max(bspline_visualization_samples_, 200);
    arc_length_table.build(bspline, 0.0, bspline.maxU(), table_samples);
    const double length = arc_length_table.getLength(bspline.maxU());
    if (length <= 1e-6) {
        return true;
    }

    const int sample_count = std::max(2, static_cast<int>(std::ceil(length / std::max(sample_step, 0.05))));
    for (int i = 0; i <= sample_count; ++i) {
        const double s = length * static_cast<double>(i) / static_cast<double>(sample_count);
        const double u = arc_length_table.getUfromS(s);
        const Eigen::Vector3d point = bspline.position(u);
        bool inside_endpoint_clearance = false;
        for (const auto& clearance_point : clearance_points) {
            if ((point - clearance_point).norm() <= dynamic_obstacle_endpoint_clear_radius_) {
                inside_endpoint_clearance = true;
                break;
            }
        }
        if (inside_endpoint_clearance) {
            continue;
        }

        if (isPointInsideInflatedObstacle(point, obstacles)) {
            ROS_WARN("B-spline collision check failed at sample %d/%d, point=(%.2f, %.2f, %.2f), clearance=%.2f.",
                     i, sample_count, point.x(), point.y(), point.z(),
                     dynamic_obstacle_inflation_ + safety_distance_ + bspline_collision_extra_clearance_);
            return false;
        }
    }

    return true;
}

bool ABPlanner::buildAndPublishBsplinePath(const std::vector<std::vector<Eigen::Vector3d>>& paths)
{
    const std::vector<BSpline> previous_bsplines = bsplines_;
    const std::vector<ArcLengthTable> previous_arc_length_tables = arc_length_tables_;
    const BSpline previous_current_bspline = current_bspline;
    const ArcLengthTable previous_current_arc_table = current_arc_table;
    const int previous_bspline_segment_idx = current_bspline_segment_idx_;
    const double previous_last_u = last_u_;

    auto restorePreviousBspline = [&]() {
        bsplines_ = previous_bsplines;
        arc_length_tables_ = previous_arc_length_tables;
        current_bspline = previous_current_bspline;
        current_arc_table = previous_current_arc_table;
        current_bspline_segment_idx_ = previous_bspline_segment_idx;
        last_u_ = previous_last_u;
    };

    bsplines_.clear();
    arc_length_tables_.clear();
    current_bspline_segment_idx_ = 0;
    last_u_ = 0.0;

    nav_msgs::Path bspline_path_msg;
    bspline_path_msg.header.stamp = alignedStamp();
    bspline_path_msg.header.frame_id = visualization_frame_;

    std::vector<Eigen::Vector3d> raw_points;
    std::vector<Eigen::Vector3d> collision_clearance_points;
    if (have_odom_) {
        collision_clearance_points.push_back(current_position_);
    }
    for (const auto& path : paths) {
        if (!path.empty()) {
            collision_clearance_points.push_back(path.front());
            collision_clearance_points.push_back(path.back());
        }
        for (size_t i = 0; i < path.size(); ++i) {
            if (!raw_points.empty() && i == 0 &&
                (raw_points.back() - path[i]).norm() < 1e-6) {
                continue;
            }
            raw_points.push_back(path[i]);
        }
    }

    const std::vector<Eigen::Vector3d> optimized_points = optimizePathForBspline(raw_points);
    std::vector<Eigen::Vector3d> control_points = makeBsplineControlPoints(optimized_points);

    if (control_points.size() < 4) {
        ROS_WARN("Skip B-spline visualization: need at least 4 merged control points, got %lu.",
                 control_points.size());
        restorePreviousBspline();
        return false;
    }

    BSpline bspline;
    bspline.setControlPoints(control_points);
    const std::vector<BoxObstacle> dynamic_obstacles = freshDynamicObstacles();
    if (!isBsplineCollisionFree(bspline, dynamic_obstacles, bspline_collision_sample_step_, collision_clearance_points)) {
        auto blendPoints = [&](double alpha) {
            std::vector<Eigen::Vector3d> blended = raw_points;
            if (raw_points.size() != optimized_points.size()) {
                return blended;
            }
            for (size_t i = 1; i + 1 < blended.size(); ++i) {
                blended[i] = raw_points[i] + alpha * (optimized_points[i] - raw_points[i]);
            }
            return blended;
        };

        auto tryCollisionFreeCandidate = [&](const std::vector<Eigen::Vector3d>& candidate_points,
                                             double spacing,
                                             BSpline& candidate_bspline,
                                             std::vector<Eigen::Vector3d>& candidate_control_points) {
            candidate_control_points = makeBsplineControlPoints(candidate_points, spacing);
            if (candidate_control_points.size() < 4) {
                return false;
            }
            candidate_bspline.setControlPoints(candidate_control_points);
            return isBsplineCollisionFree(candidate_bspline, dynamic_obstacles,
                                          bspline_collision_sample_step_, collision_clearance_points);
        };

        bool fallback_found = false;
        double selected_alpha = 0.0;
        double selected_spacing = bspline_control_point_spacing_;
        BSpline candidate_bspline;
        std::vector<Eigen::Vector3d> candidate_control_points;

        for (double alpha : {0.75, 0.5, 0.25}) {
            if (tryCollisionFreeCandidate(blendPoints(alpha), bspline_control_point_spacing_,
                                          candidate_bspline, candidate_control_points)) {
                fallback_found = true;
                selected_alpha = alpha;
                selected_spacing = bspline_control_point_spacing_;
                break;
            }
        }

        if (!fallback_found &&
            tryCollisionFreeCandidate(raw_points, bspline_control_point_spacing_,
                                      candidate_bspline, candidate_control_points)) {
            fallback_found = true;
            selected_alpha = 0.0;
            selected_spacing = bspline_control_point_spacing_;
        }

        const double dense_spacing = std::max(resolution_, std::min(0.5, bspline_control_point_spacing_));
        if (!fallback_found && dense_spacing < bspline_control_point_spacing_ - 1e-6) {
            for (double alpha : {0.75, 0.5, 0.25, 0.0}) {
                const std::vector<Eigen::Vector3d> candidate_points =
                    alpha > 1e-6 ? blendPoints(alpha) : raw_points;
                if (tryCollisionFreeCandidate(candidate_points, dense_spacing,
                                              candidate_bspline, candidate_control_points)) {
                    fallback_found = true;
                    selected_alpha = alpha;
                    selected_spacing = dense_spacing;
                    break;
                }
            }
        }

        if (!fallback_found) {
            ROS_WARN("No collision-free blended/raw A* B-spline fallback found. Rejecting this plan.");
            restorePreviousBspline();
            return false;
        }

        bspline = candidate_bspline;
        control_points = candidate_control_points;
        ROS_WARN("Optimized B-spline is too close to obstacle. Using collision-free blended A* fallback: optimization_blend=%.2f, spacing=%.2f m, control_points=%lu.",
                 selected_alpha, selected_spacing, control_points.size());
    }
    bsplines_.push_back(bspline);

    ArcLengthTable arc_length_table;
    const int samples_per_path = std::max(2, bspline_visualization_samples_);
    const double max_u = bspline.maxU();
    arc_length_table.build(bspline, 0.0, max_u, samples_per_path);
    arc_length_tables_.push_back(arc_length_table);

    for (int i = 0; i < samples_per_path; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples_per_path - 1);
        const double u = ratio * max_u;
        const Eigen::Vector3d point = bspline.position(u);

        geometry_msgs::PoseStamped pose;
        pose.header = bspline_path_msg.header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        pose.pose.orientation.w = 1.0;
        bspline_path_msg.poses.push_back(pose);
    }

    if (publish_rviz_visualization_) {
        bspline_path_pub_.publish(bspline_path_msg);
    }
    current_bspline = bsplines_.front();
    current_arc_table = arc_length_tables_.front();

    ROS_INFO_THROTTLE(1.0, "Built B-spline from %lu raw A* points through %lu optimized points to %lu control points, spacing=%.2f m, collision_check=%s, sample_step=%.2f.",
             raw_points.size(), optimized_points.size(), control_points.size(), bspline_control_point_spacing_,
             bspline_collision_check_enabled_ ? "on" : "off", bspline_collision_sample_step_);
    return true;
}

ros::Time ABPlanner::alignedStamp() const
{
    if (!latest_odom_stamp_.isZero()) {
        return latest_odom_stamp_;
    }
    if (!start_state_time_.isZero()) {
        return start_state_time_;
    }
    return ros::Time(0);
}

// 判断路径执行完成的条件，可以根据无人机当前位置与当前目标路径点的距离是否小于某个阈值来判断
bool ABPlanner::isPathExecutionComplete(Eigen::Vector3d current_position, Eigen::Vector3d current_target_point)
{
    double distance_to_target = (current_position - current_target_point).norm(); // 计算当前无人机位置与目标路径点的距离
    // 判断距离是否小于某个阈值，如果是则认为路径执行完成
    return distance_to_target <= distance_to_target_threshold_;
}

// 判断路径执行过程的条件， 来切换current_bspline和current_arc_table， 根据当前无人机位置与当前路径点的距离是否小于某个阈值来判断是否切换到下一段路径
bool ABPlanner::isPathExecutionProgressing(Eigen::Vector3d current_position, Eigen::Vector3d current_target_point)
{    double distance_to_target = (current_position - current_target_point).norm(); // 计算当前无人机位置与目标路径点的距离
    // 判断距离是否小于某个阈值，如果是则认为路径执行正在进行，可以切换到下一段路径
    return distance_to_target <= distance_to_progress_threshold_;
}

// 当前状态 [px,py,pz,vx,vy,vz,ax,ay,az]
Eigen::Matrix<double, 9, 1> ABPlanner::extractState() const
{
    Eigen::Matrix<double, 9, 1> state;
    state.segment<3>(0) = current_position_;
    state.segment<3>(3) = current_velocity_;
    state.segment<3>(6) = current_acceleration_;
    return state;
}

Eigen::Matrix<double, 9, 1> ABPlanner::sanitizeMpcState(const Eigen::Matrix<double, 9, 1>& state) const
{
    Eigen::Matrix<double, 9, 1> sanitized = state;

    const double vel_margin = std::max(0.2, mpc_config_.max_acc * mpc_config_.dt +
                                              0.5 * mpc_config_.max_jerk * mpc_config_.dt * mpc_config_.dt);
    const double vel_limit = std::max(0.1, mpc_config_.max_vel - vel_margin);

    const Eigen::Vector3d raw_velocity = sanitized.segment<3>(3);
    const Eigen::Vector3d raw_acceleration = sanitized.segment<3>(6);

    sanitized.segment<3>(3) = raw_velocity.cwiseMax(Eigen::Vector3d::Constant(-vel_limit))
                                          .cwiseMin(Eigen::Vector3d::Constant(vel_limit));

    if (have_last_successful_cmd_) {
        sanitized(6) = last_successful_cmd_.acceleration.x;
        sanitized(7) = last_successful_cmd_.acceleration.y;
        sanitized(8) = last_successful_cmd_.acceleration.z;
    } else {
        sanitized.segment<3>(6).setZero();
    }

    const double acc_limit = std::max(0.1, mpc_config_.max_acc - mpc_config_.max_jerk * mpc_config_.dt);
    sanitized.segment<3>(6) = sanitized.segment<3>(6).cwiseMax(Eigen::Vector3d::Constant(-acc_limit))
                                                     .cwiseMin(Eigen::Vector3d::Constant(acc_limit));

    ROS_WARN_THROTTLE(1.0,
        "MPC state sanitized. raw vel=(%.2f, %.2f, %.2f), used vel=(%.2f, %.2f, %.2f), raw acc=(%.2f, %.2f, %.2f), used acc=(%.2f, %.2f, %.2f)",
        raw_velocity.x(), raw_velocity.y(), raw_velocity.z(),
        sanitized(3), sanitized(4), sanitized(5),
        raw_acceleration.x(), raw_acceleration.y(), raw_acceleration.z(),
        sanitized(6), sanitized(7), sanitized(8));

    return sanitized;
}

bool ABPlanner::isMpcPredictionProgressing(const Eigen::MatrixXd& out_states,
                                           const Eigen::MatrixXd& ref_traj) const
{
    if (out_states.rows() < 6 || out_states.cols() < 2 || ref_traj.rows() < 3 || ref_traj.cols() < 2) {
        return false;
    }

    const int cmd_idx = std::max(1, std::min(mpc_command_step_, static_cast<int>(out_states.cols()) - 1));
    const int ref_cmd_idx = std::min(cmd_idx, static_cast<int>(ref_traj.cols()) - 1);
    const Eigen::Vector3d ref_cmd = ref_traj.block<3, 1>(0, ref_cmd_idx);
    const Eigen::Vector3d ref_end = ref_traj.block<3, 1>(0, ref_traj.cols() - 1);
    const Eigen::Vector3d pred_cmd = out_states.block<3, 1>(0, cmd_idx);
    const Eigen::Vector3d pred_end = out_states.block<3, 1>(0, out_states.cols() - 1);
    const Eigen::Vector3d pred_vel = out_states.block<3, 1>(3, cmd_idx);

    const Eigen::Vector3d current_to_ref_cmd = ref_cmd - current_position_;
    const Eigen::Vector3d current_to_ref_end = ref_end - current_position_;
    const Eigen::Vector3d current_to_pred_cmd = pred_cmd - current_position_;
    const Eigen::Vector3d current_to_pred_end = pred_end - current_position_;

    if (current_to_ref_end.norm() < 1.0) {
        return true;
    }

    Eigen::Vector3d forward = current_to_ref_cmd;
    if (forward.norm() < 0.5) {
        forward = current_to_ref_end;
    }
    if (forward.norm() < 1e-3) {
        return true;
    }
    forward.normalize();

    const double cmd_forward_progress = current_to_pred_cmd.dot(forward);
    const double end_forward_progress = current_to_pred_end.dot(forward);
    const double vel_forward = pred_vel.dot(forward);
    const double expected_cmd_progress = std::max(0.2, 0.25 * max_vel_ * mpc_config_.dt * cmd_idx);
    const double expected_end_progress = std::max(1.0, 0.15 * max_vel_ * mpc_config_.dt *
                                                        static_cast<double>(out_states.cols() - 1));

    const bool horizon_moves_forward = end_forward_progress >= expected_end_progress;
    const bool command_moves_forward = cmd_forward_progress >= expected_cmd_progress || vel_forward > 0.5;
    const bool command_strongly_reverses = cmd_forward_progress < -1.0 && vel_forward < -0.5;
    if (!horizon_moves_forward || (!command_moves_forward && command_strongly_reverses)) {
        ROS_WARN_THROTTLE(1.0,
            "MPC prediction progress check failed: cmd_progress=%.2f/%.2f, end_progress=%.2f/%.2f, vel_forward=%.2f, strong_reverse=%s.",
            cmd_forward_progress, expected_cmd_progress, end_forward_progress, expected_end_progress, vel_forward,
            command_strongly_reverses ? "true" : "false");
        return false;
    }

    return true;
}

Eigen::MatrixXd ABPlanner::generateRefTraj(ArcLengthTable arc_length_table, BSpline bspline, double u_start, int N, double dt)
{
    Eigen::MatrixXd ref_traj(3, N + 1);
    double s_start = arc_length_table.getLength(u_start);
    double ds = max_vel_ * dt;
    for (int i = 0; i <= N; ++i) {
        double s = s_start + i * ds;
        double u = arc_length_table.getUfromS(s);
        Eigen::Vector3d pos = bspline.position(u); // 使用第一个B样条
        ref_traj.col(i) = pos;
    }
    ROS_INFO_THROTTLE(1.0,
        "MPC ref traj NED: start=(%.2f, %.2f, %.2f), cmd_step_ref=(%.2f, %.2f, %.2f), end=(%.2f, %.2f, %.2f), ds=%.3f",
        ref_traj(0, 0), ref_traj(1, 0), ref_traj(2, 0),
        ref_traj(0, std::min(mpc_command_step_, N)), ref_traj(1, std::min(mpc_command_step_, N)), ref_traj(2, std::min(mpc_command_step_, N)),
        ref_traj(0, N), ref_traj(1, N), ref_traj(2, N), ds);
    return ref_traj;
}

Eigen::MatrixXd ABPlanner::generateContinuousRefTraj(double u_start, int N, double dt)
{
    Eigen::MatrixXd ref_traj(6, N + 1);
    if (bsplines_.empty() || arc_length_tables_.empty()) {
        ref_traj.setZero();
        return ref_traj;
    }

    const int start_segment = std::max(0, std::min(current_bspline_segment_idx_, static_cast<int>(bsplines_.size()) - 1));
    const double current_seg_s = arc_length_tables_[start_segment].getLength(u_start);
    const double ds = max_vel_ * dt;

    for (int i = 0; i <= N; ++i) {
        int seg_idx = start_segment;
        double s = current_seg_s + i * ds;

        while (seg_idx + 1 < static_cast<int>(arc_length_tables_.size()) &&
               s > arc_length_tables_[seg_idx].getLength(bsplines_[seg_idx].maxU())) {
            s -= arc_length_tables_[seg_idx].getLength(bsplines_[seg_idx].maxU());
            seg_idx++;
        }

        const double seg_len = arc_length_tables_[seg_idx].getLength(bsplines_[seg_idx].maxU());
        const bool stop_at_window_end =
            planned_window_end_idx_ >= static_cast<int>(segment_waypoints_.size()) - 1 &&
            current_segment_ >= static_cast<int>(segment_idx_.size());

        if (s <= seg_len || stop_at_window_end) {
            const double clamped_s = std::min(s, seg_len);
            const double u = arc_length_tables_[seg_idx].getUfromS(clamped_s);
            Eigen::Vector3d tangent = bsplines_[seg_idx].velocity(u);
            if (tangent.norm() < 1e-6) {
                tangent = bsplines_[seg_idx].position(std::min(bsplines_[seg_idx].maxU(), u + 0.2)) -
                          bsplines_[seg_idx].position(std::max(0.0, u - 0.2));
            }
            if (tangent.norm() < 1e-6) {
                tangent = Eigen::Vector3d::UnitX();
            } else {
                tangent.normalize();
            }
            ref_traj.block<3, 1>(0, i) = bsplines_[seg_idx].position(u);
            ref_traj.block<3, 1>(3, i) = tangent * max_vel_;
        } else {
            const double end_u = bsplines_[seg_idx].maxU();
            Eigen::Vector3d tangent = bsplines_[seg_idx].velocity(end_u);
            if (tangent.norm() < 1e-6) {
                tangent = bsplines_[seg_idx].position(end_u) -
                          bsplines_[seg_idx].position(std::max(0.0, end_u - 0.2));
            }
            if (tangent.norm() < 1e-6) {
                tangent = Eigen::Vector3d::UnitX();
            } else {
                tangent.normalize();
            }
            ref_traj.block<3, 1>(0, i) = bsplines_[seg_idx].position(end_u) + tangent * (s - seg_len);
            ref_traj.block<3, 1>(3, i) = tangent * max_vel_;
        }
    }

    ROS_INFO_THROTTLE(1.0,
        "MPC continuous ref NED: seg=%d, start=(%.2f, %.2f, %.2f), cmd_step_ref=(%.2f, %.2f, %.2f), end=(%.2f, %.2f, %.2f), ref_vel=(%.2f, %.2f, %.2f), ds=%.3f",
        start_segment,
        ref_traj(0, 0), ref_traj(1, 0), ref_traj(2, 0),
        ref_traj(0, std::min(mpc_command_step_, N)), ref_traj(1, std::min(mpc_command_step_, N)), ref_traj(2, std::min(mpc_command_step_, N)),
        ref_traj(0, N), ref_traj(1, N), ref_traj(2, N),
        ref_traj(3, std::min(mpc_command_step_, N)), ref_traj(4, std::min(mpc_command_step_, N)), ref_traj(5, std::min(mpc_command_step_, N)),
        ds);
    return ref_traj;
}

void ABPlanner::publishMpcPredictionPath(const Eigen::MatrixXd& out_states)
{
    if (!publish_rviz_visualization_) {
        return;
    }

    if (out_states.rows() < 3 || out_states.cols() == 0) {
        return;
    }

    nav_msgs::Path path_msg;
    path_msg.header.stamp = alignedStamp();
    path_msg.header.frame_id = visualization_frame_;
    path_msg.poses.reserve(out_states.cols());

    for (int i = 0; i < out_states.cols(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = out_states(0, i);
        pose.pose.position.y = out_states(1, i);
        pose.pose.position.z = out_states(2, i);
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    mpc_prediction_path_pub_.publish(path_msg);
}

void ABPlanner::limitStartupVerticalCommand(Eigen::Vector3d& cmd_position,
                                            Eigen::Vector3d& cmd_velocity,
                                            Eigen::Vector3d& cmd_acceleration) const
{
    if (!startup_altitude_hold_enabled_ || segment_waypoints_.empty() || current_waypoint_idx_ > 0) {
        return;
    }

    const double horizontal_progress =
        (current_position_.head<2>() - segment_waypoints_.front().head<2>()).norm();
    if (horizontal_progress >= startup_altitude_hold_distance_) {
        return;
    }

    // In the AirSim/NED convention used here, smaller z means higher altitude.
    const double min_allowed_z = current_position_.z() - startup_altitude_hold_margin_;
    if (cmd_position.z() >= min_allowed_z) {
        return;
    }

    ROS_WARN_THROTTLE(1.0,
        "Startup altitude hold clamps planner command z from %.2f to %.2f while horizontal progress is %.2f/%.2f m.",
        cmd_position.z(), min_allowed_z, horizontal_progress, startup_altitude_hold_distance_);

    cmd_position.z() = min_allowed_z;
    if (cmd_velocity.z() < 0.0) {
        cmd_velocity.z() = 0.0;
    }
    if (cmd_acceleration.z() < 0.0) {
        cmd_acceleration.z() = 0.0;
    }
}

void ABPlanner::publishReferenceFallbackCommand(const Eigen::MatrixXd& ref_traj, const std::string& reason)
{
    if (ref_traj.rows() < 3 || ref_traj.cols() < 2) {
        publishControlCommand(false, Eigen::MatrixXd(), Eigen::MatrixXd());
        return;
    }

    const int cmd_idx = std::max(1, std::min(mpc_command_step_, static_cast<int>(ref_traj.cols()) - 1));
    Eigen::Vector3d cmd_position = ref_traj.block<3, 1>(0, cmd_idx);
    Eigen::Vector3d cmd_velocity = Eigen::Vector3d::Zero();
    if (ref_traj.rows() >= 6) {
        cmd_velocity = ref_traj.block<3, 1>(3, cmd_idx);
    } else {
        const double dt = std::max(1e-3, mpc_config_.dt * cmd_idx);
        cmd_velocity = (cmd_position - current_position_) / dt;
        if (cmd_velocity.norm() > max_vel_) {
            cmd_velocity = cmd_velocity.normalized() * max_vel_;
        }
    }
    Eigen::Vector3d cmd_acceleration = Eigen::Vector3d::Zero();

    const Eigen::Vector3d raw_ned_position = cmd_position;
    const Eigen::Vector3d raw_ned_velocity = cmd_velocity;
    const double fallback_dt = std::max(1e-3, mpc_config_.dt * static_cast<double>(cmd_idx));
    const double speed_limit = std::min(fallback_max_speed_, std::max(0.5, max_vel_));
    const double descend_speed_limit = std::min(fallback_max_vertical_speed_, speed_limit);
    const double climb_speed_limit = std::min(speed_limit, std::max(fallback_max_vertical_speed_, 0.75 * speed_limit));

    if (cmd_velocity.norm() > speed_limit) {
        cmd_velocity = cmd_velocity.normalized() * speed_limit;
    }
    cmd_velocity.z() = std::clamp(cmd_velocity.z(), -climb_speed_limit, descend_speed_limit);

    Eigen::Vector3d limited_delta = cmd_position - current_position_;
    const double max_step = speed_limit * fallback_dt;
    if (limited_delta.norm() > max_step) {
        limited_delta = limited_delta.normalized() * max_step;
    }
    const double max_climb_step = climb_speed_limit * fallback_dt;
    const double max_descend_step = descend_speed_limit * fallback_dt;
    limited_delta.z() = std::clamp(limited_delta.z(), -max_climb_step, max_descend_step);
    cmd_position = current_position_ + limited_delta;

    const Eigen::Vector3d position_velocity = limited_delta / fallback_dt;
    if (position_velocity.norm() > 1e-3) {
        cmd_velocity = 0.5 * cmd_velocity + 0.5 * position_velocity;
        if (cmd_velocity.norm() > speed_limit) {
            cmd_velocity = cmd_velocity.normalized() * speed_limit;
        }
        cmd_velocity.z() = std::clamp(cmd_velocity.z(), -climb_speed_limit, descend_speed_limit);
    }

    limitStartupVerticalCommand(cmd_position, cmd_velocity, cmd_acceleration);

    const Eigen::Vector3d ned_position = cmd_position;
    const Eigen::Vector3d ned_velocity = cmd_velocity;

    if (convert_ned_to_position_cmd_) {
        cmd_position.y() = -cmd_position.y();
        cmd_position.z() = -cmd_position.z();
        cmd_velocity.y() = -cmd_velocity.y();
        cmd_velocity.z() = -cmd_velocity.z();
        cmd_acceleration.y() = -cmd_acceleration.y();
        cmd_acceleration.z() = -cmd_acceleration.z();
    }

    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = alignedStamp();
    cmd.header.frame_id = visualization_frame_;
    cmd.position.x = cmd_position.x();
    cmd.position.y = cmd_position.y();
    cmd.position.z = cmd_position.z();
    cmd.velocity.x = cmd_velocity.x();
    cmd.velocity.y = cmd_velocity.y();
    cmd.velocity.z = cmd_velocity.z();
    cmd.acceleration.x = cmd_acceleration.x();
    cmd.acceleration.y = cmd_acceleration.y();
    cmd.acceleration.z = cmd_acceleration.z();

    double planned_yaw_dot = 0.0;
    cmd.yaw = planYaw(ned_position, ned_velocity, cmd.header.stamp, planned_yaw_dot);
    cmd.yaw_dot = planned_yaw_dot;

    control_cmd_pub_.publish(cmd);
    last_successful_cmd_ = cmd;
    have_last_successful_cmd_ = true;

    ROS_WARN_THROTTLE(1.0,
        "Publishing limited reference fallback command because %s. cmd_idx=%d, raw_NED target=(%.2f, %.2f, %.2f), raw_vel=(%.2f, %.2f, %.2f), limited_NED target=(%.2f, %.2f, %.2f), limited_vel=(%.2f, %.2f, %.2f), limits=(speed %.2f, climb %.2f, descend %.2f).",
        reason.c_str(), cmd_idx,
        raw_ned_position.x(), raw_ned_position.y(), raw_ned_position.z(),
        raw_ned_velocity.x(), raw_ned_velocity.y(), raw_ned_velocity.z(),
        ned_position.x(), ned_position.y(), ned_position.z(),
        ned_velocity.x(), ned_velocity.y(), ned_velocity.z(),
        speed_limit, climb_speed_limit, descend_speed_limit);
}

double ABPlanner::normalizeAngle(double angle)
{
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double ABPlanner::interpolateYaw(double yaw0, double yaw1, double alpha)
{
    const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
    const double delta = normalizeAngle(yaw1 - yaw0);
    return normalizeAngle(yaw0 + clamped_alpha * delta);
}

double ABPlanner::planYaw(const Eigen::Vector3d& target_position,
                          const Eigen::Vector3d& target_velocity,
                          const ros::Time& stamp,
                          double& yaw_dot)
{
    double desired_yaw = last_yaw_;

    if (use_waypoint_yaw_ &&
        segment_yaws_.size() == segment_waypoints_.size() &&
        !segment_yaws_.empty()) {
        const int idx0 = std::max(0, std::min(current_waypoint_idx_,
                                              static_cast<int>(segment_yaws_.size()) - 1));
        const int idx1 = std::min(idx0 + 1, static_cast<int>(segment_yaws_.size()) - 1);
        double alpha = 0.0;

        if (idx1 > idx0) {
            const Eigen::Vector3d p0 = segment_waypoints_[idx0];
            const Eigen::Vector3d p1 = segment_waypoints_[idx1];
            const Eigen::Vector3d segment = p1 - p0;
            const double segment_len_sq = segment.squaredNorm();
            if (segment_len_sq > 1e-6) {
                alpha = (target_position - p0).dot(segment) / segment_len_sq;
            }
        }

        desired_yaw = interpolateYaw(segment_yaws_[idx0], segment_yaws_[idx1], alpha);
    } else if (target_velocity.head<2>().norm() > 0.3) {
        desired_yaw = std::atan2(target_velocity.y(), target_velocity.x());
    } else {
        const Eigen::Vector3d position_diff = target_position - current_position_;
        if (position_diff.head<2>().norm() > 0.3) {
            desired_yaw = std::atan2(position_diff.y(), position_diff.x());
        } else {
            desired_yaw = last_yaw_;
        }
    }

    if (!have_last_yaw_command_) {
        last_yaw_ = normalizeAngle(current_yaw_);
        last_yaw_command_stamp_ = stamp;
        have_last_yaw_command_ = true;
    }

    const double dt = (stamp - last_yaw_command_stamp_).toSec();
    const double raw_delta = normalizeAngle(desired_yaw - last_yaw_);
    double limited_delta = raw_delta;
    if (dt > 1e-4) {
        const double max_delta = max_yaw_rate_ * dt;
        limited_delta = std::clamp(raw_delta, -max_delta, max_delta);
        yaw_dot = limited_delta / dt;
    } else {
        yaw_dot = 0.0;
    }

    const double planned_yaw = normalizeAngle(last_yaw_ + limited_delta);
    last_yaw_ = planned_yaw;
    last_yaw_command_stamp_ = stamp;
    return planned_yaw;
}

bool ABPlanner::isRecoveryNudgeCollisionFree(const Eigen::Vector3d& start,
                                             const Eigen::Vector3d& target,
                                             const std::vector<BoxObstacle>& obstacles) const
{
    if (obstacles.empty()) {
        return true;
    }

    const double length = (target - start).norm();
    if (length <= 1e-6) {
        return true;
    }

    const int sample_count = std::max(2, static_cast<int>(std::ceil(length / std::max(0.1, 0.5 * resolution_))));
    for (int i = 1; i <= sample_count; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(sample_count);
        const Eigen::Vector3d point = start + alpha * (target - start);
        if (isPointInsideInflatedObstacle(point, obstacles)) {
            return false;
        }
    }

    return true;
}

bool ABPlanner::startRecoveryNudge()
{
    if (!have_odom_) {
        return false;
    }

    recovery_nudge_start_position_ = current_position_;
    recovery_nudge_start_stamp_ = alignedStamp();

    Eigen::Vector3d forward_direction = Eigen::Vector3d::Zero();
    if (!segment_waypoints_.empty()) {
        const int next_idx = std::min(current_waypoint_idx_ + 1,
                                      static_cast<int>(segment_waypoints_.size()) - 1);
        forward_direction = segment_waypoints_[next_idx] - current_position_;
        forward_direction.z() = 0.0;
    }
    if (forward_direction.norm() < 1e-3) {
        forward_direction = Eigen::Vector3d(std::cos(current_yaw_), std::sin(current_yaw_), 0.0);
    }
    forward_direction.normalize();

    const Eigen::Vector3d forward_offset = planning_recovery_step_ * forward_direction;
    const Eigen::Vector3d right_direction(forward_direction.y(), -forward_direction.x(), 0.0);
    const Eigen::Vector3d left_offset = planning_recovery_step_ * right_direction;
    const Eigen::Vector3d right_offset = -planning_recovery_step_ * right_direction;
    const Eigen::Vector3d backward_offset = -planning_recovery_step_ * forward_direction;

    std::vector<Eigen::Vector3d> candidate_offsets;
    if (recovery_nudge_attempt_count_ % 2 == 0) {
        candidate_offsets.push_back(left_offset);
        candidate_offsets.push_back(right_offset);
    } else {
        candidate_offsets.push_back(right_offset);
        candidate_offsets.push_back(left_offset);
    }
    candidate_offsets.push_back(backward_offset);
    candidate_offsets.push_back(0.5 * (left_offset + forward_offset));
    candidate_offsets.push_back(0.5 * (right_offset + forward_offset));
    candidate_offsets.push_back(forward_offset);

    if (planning_recovery_allow_vertical_) {
        const Eigen::Vector3d upward_offset(0.0, 0.0, -planning_recovery_step_);
        const Eigen::Vector3d downward_offset(0.0, 0.0, planning_recovery_step_);
        candidate_offsets.push_back(upward_offset);
        candidate_offsets.push_back(downward_offset);
        candidate_offsets.push_back(upward_offset + 0.5 * backward_offset);
    }

    const std::vector<BoxObstacle> obstacles = freshDynamicObstacles();
    for (const auto& offset : candidate_offsets) {
        const Eigen::Vector3d candidate = recovery_nudge_start_position_ + offset;
        if (!candidate.allFinite()) {
            continue;
        }
        if (!isRecoveryNudgeCollisionFree(recovery_nudge_start_position_, candidate, obstacles)) {
            continue;
        }

        recovery_nudge_target_position_ = candidate;
        recovery_nudge_attempt_count_++;
        ROS_WARN("Selected recovery nudge target=(%.2f, %.2f, %.2f) from current=(%.2f, %.2f, %.2f), attempt=%d.",
                 recovery_nudge_target_position_.x(), recovery_nudge_target_position_.y(), recovery_nudge_target_position_.z(),
                 recovery_nudge_start_position_.x(), recovery_nudge_start_position_.y(), recovery_nudge_start_position_.z(),
                 recovery_nudge_attempt_count_);
        return true;
    }

    ROS_WARN("Planning recovery nudge could not find a collision-free %.2f m candidate. Holding and retrying A* later.",
             planning_recovery_step_);
    return false;
}

void ABPlanner::publishRecoveryNudgeCommand()
{
    const ros::Time stamp = alignedStamp();
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = visualization_frame_;

    Eigen::Vector3d cmd_position = recovery_nudge_target_position_;
    Eigen::Vector3d cmd_velocity =
        (recovery_nudge_target_position_ - recovery_nudge_start_position_) /
        std::max(0.2, planning_recovery_duration_);
    const double max_recovery_speed = std::max(0.2, planning_recovery_step_ / std::max(0.2, planning_recovery_duration_));
    if (cmd_velocity.norm() > max_recovery_speed) {
        cmd_velocity = cmd_velocity.normalized() * max_recovery_speed;
    }

    if (convert_ned_to_position_cmd_) {
        cmd_position.y() = -cmd_position.y();
        cmd_position.z() = -cmd_position.z();
        cmd_velocity.y() = -cmd_velocity.y();
        cmd_velocity.z() = -cmd_velocity.z();
    }

    cmd.position.x = cmd_position.x();
    cmd.position.y = cmd_position.y();
    cmd.position.z = cmd_position.z();
    cmd.velocity.x = cmd_velocity.x();
    cmd.velocity.y = cmd_velocity.y();
    cmd.velocity.z = cmd_velocity.z();
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    cmd.yaw = have_last_yaw_command_ ? last_yaw_ : current_yaw_;
    cmd.yaw_dot = 0.0;

    control_cmd_pub_.publish(cmd);
    last_successful_cmd_ = cmd;
    have_last_successful_cmd_ = true;

    ROS_INFO_THROTTLE(1.0,
        "Publishing recovery nudge command: target=(%.2f, %.2f, %.2f), velocity=(%.2f, %.2f, %.2f), elapsed=%.2f/%.2f s.",
        cmd.position.x, cmd.position.y, cmd.position.z,
        cmd.velocity.x, cmd.velocity.y, cmd.velocity.z,
        (stamp - recovery_nudge_start_stamp_).toSec(), planning_recovery_duration_);
}

void ABPlanner::startSegmentTurnaround()
{
    segment_turnaround_position_ = segment_waypoints_.empty() ? current_position_ : segment_waypoints_.back();
    segment_turnaround_start_stamp_ = alignedStamp();

    if (!have_last_yaw_command_) {
        last_yaw_ = normalizeAngle(current_yaw_);
        last_yaw_command_stamp_ = segment_turnaround_start_stamp_;
        have_last_yaw_command_ = true;
    }

    segment_turnaround_start_yaw_ = last_yaw_;
    segment_turnaround_target_yaw_ = normalizeAngle(segment_turnaround_start_yaw_ + segment_turnaround_yaw_delta_);
    planner_state_ = SEGMENT_TURNING;

    ROS_INFO_THROTTLE(1.0, "Starting segment turnaround: hold=(%.2f, %.2f, %.2f), yaw %.2f -> %.2f over %.2f s.",
             segment_turnaround_position_.x(), segment_turnaround_position_.y(), segment_turnaround_position_.z(),
             segment_turnaround_start_yaw_, segment_turnaround_target_yaw_, segment_turnaround_duration_);
}

void ABPlanner::publishSegmentTurnaroundCommand()
{
    const ros::Time stamp = alignedStamp();
    const double elapsed = std::max(0.0, (stamp - segment_turnaround_start_stamp_).toSec());
    const double duration = std::max(segment_turnaround_duration_, 1e-3);
    const double yaw_motion_duration = std::max(1e-3, 0.5 * duration);
    const double alpha = std::clamp(elapsed / yaw_motion_duration, 0.0, 1.0);
    const double yaw = interpolateYaw(segment_turnaround_start_yaw_, segment_turnaround_target_yaw_, alpha);
    const double yaw_dot = alpha < 1.0
        ? normalizeAngle(segment_turnaround_target_yaw_ - segment_turnaround_start_yaw_) / yaw_motion_duration
        : 0.0;

    Eigen::Vector3d cmd_position = segment_turnaround_position_;
    if (convert_ned_to_position_cmd_) {
        cmd_position.y() = -cmd_position.y();
        cmd_position.z() = -cmd_position.z();
    }

    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = visualization_frame_;
    cmd.position.x = cmd_position.x();
    cmd.position.y = cmd_position.y();
    cmd.position.z = cmd_position.z();
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = 0.0;
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    cmd.yaw = yaw;
    cmd.yaw_dot = yaw_dot;

    control_cmd_pub_.publish(cmd);
    last_successful_cmd_ = cmd;
    have_last_successful_cmd_ = true;
    last_yaw_ = yaw;
    last_yaw_command_stamp_ = stamp;
    have_last_yaw_command_ = true;

    ROS_INFO_THROTTLE(1.0,
        "Segment turnaround command: hold=(%.2f, %.2f, %.2f), yaw=%.2f, yaw_dot=%.2f, elapsed=%.2f/%.2f s, yaw_motion_duration=%.2f s.",
        cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw, cmd.yaw_dot,
        elapsed, segment_turnaround_duration_, yaw_motion_duration);
}

void ABPlanner::publishControlCommand(bool solve_sussess, const Eigen::MatrixXd& out_states, const Eigen::MatrixXd& out_inputs)
{
    if (solve_sussess && out_states.cols() < 2) {
        ROS_WARN("MPC output is empty, cannot publish control command.");
        return;
    }

    quadrotor_msgs::PositionCommand cmd;
    if (solve_sussess)
    {
        cmd.header.stamp = alignedStamp();
        cmd.header.frame_id = visualization_frame_;
        
        const int cmd_idx = std::max(1, std::min(mpc_command_step_, static_cast<int>(out_states.cols()) - 1));
        Eigen::Vector3d cmd_position(out_states(0, cmd_idx), out_states(1, cmd_idx), out_states(2, cmd_idx));
        Eigen::Vector3d cmd_velocity(out_states(3, cmd_idx), out_states(4, cmd_idx), out_states(5, cmd_idx));
        Eigen::Vector3d cmd_acceleration(out_states(6, cmd_idx), out_states(7, cmd_idx), out_states(8, cmd_idx));

        limitStartupVerticalCommand(cmd_position, cmd_velocity, cmd_acceleration);

        const Eigen::Vector3d ned_position = cmd_position;
        const Eigen::Vector3d ned_velocity = cmd_velocity;
        const Eigen::Vector3d ned_acceleration = cmd_acceleration;

        if (convert_ned_to_position_cmd_) {
            cmd_position.y() = -cmd_position.y();
            cmd_position.z() = -cmd_position.z();
            cmd_velocity.y() = -cmd_velocity.y();
            cmd_velocity.z() = -cmd_velocity.z();
            cmd_acceleration.y() = -cmd_acceleration.y();
            cmd_acceleration.z() = -cmd_acceleration.z();
        }

        cmd.position.x = cmd_position.x();
        cmd.position.y = cmd_position.y();
        cmd.position.z = cmd_position.z();
        cmd.velocity.x = cmd_velocity.x();
        cmd.velocity.y = cmd_velocity.y();
        cmd.velocity.z = cmd_velocity.z();
        cmd.acceleration.x = cmd_acceleration.x();
        cmd.acceleration.y = cmd_acceleration.y();
        cmd.acceleration.z = cmd_acceleration.z();

        double planned_yaw_dot = 0.0;
        cmd.yaw = planYaw(ned_position, ned_velocity, cmd.header.stamp, planned_yaw_dot);
        cmd.yaw_dot = planned_yaw_dot;

        ROS_INFO_THROTTLE(1.0,
            "MPC solve successful, cmd_idx=%d, NED target: position=(%.2f, %.2f, %.2f), velocity=(%.2f, %.2f, %.2f), acceleration=(%.2f, %.2f, %.2f), yaw=%.2f, yaw_dot=%.2f, published PositionCommand=(%.2f, %.2f, %.2f)",
            cmd_idx,
            ned_position.x(), ned_position.y(), ned_position.z(),
            ned_velocity.x(), ned_velocity.y(), ned_velocity.z(),
            ned_acceleration.x(), ned_acceleration.y(), ned_acceleration.z(),
            cmd.yaw, cmd.yaw_dot,
            cmd.position.x, cmd.position.y, cmd.position.z);
        last_successful_cmd_ = cmd;
        have_last_successful_cmd_ = true;
    }
    else
    {
        ROS_WARN("MPC solve failed, publishing braking hold command.");
        Eigen::Vector3d hold_position = current_position_;
        if (convert_ned_to_position_cmd_) {
            hold_position.y() = -hold_position.y();
            hold_position.z() = -hold_position.z();
        }
        cmd.header.stamp = alignedStamp();
        cmd.header.frame_id = visualization_frame_;
        cmd.position.x = hold_position.x();
        cmd.position.y = hold_position.y();
        cmd.position.z = hold_position.z();
        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
        cmd.yaw = have_last_yaw_command_ ? last_yaw_ : current_yaw_;
        cmd.yaw_dot = 0.0;
        last_successful_cmd_ = cmd;
        have_last_successful_cmd_ = true;
    }

    control_cmd_pub_.publish(cmd);
}
