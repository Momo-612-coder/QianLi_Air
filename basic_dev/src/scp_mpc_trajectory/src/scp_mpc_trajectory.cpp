#include "scp_mpc_trajectory.hpp"

#include <algorithm>
#include <limits>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "scp_mpc_trajectory"); // 初始化ros 节点，命名为 scp_mpc_trajectory
    ros::NodeHandle n; // 创建node控制句柄
    ScpMpcTrajectory go(&n);
    return 0;
}

void ArcLengthTable::build(ego_planner::UniformBspline& bspline, double u0, double u1, int samples)
{
    u_values_.clear();
    s_values_.clear();

    if (samples < 2 || u1 <= u0) {
        return;
    }

    u_values_.reserve(samples + 1);
    s_values_.reserve(samples + 1);

    double accum_s = 0.0;
    Eigen::Vector3d last_pos = bspline.evaluateDeBoorT(u0).head<3>();
    u_values_.push_back(u0);
    s_values_.push_back(accum_s);

    for (int i = 1; i <= samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples);
        const double u = u0 + ratio * (u1 - u0);
        const Eigen::Vector3d pos = bspline.evaluateDeBoorT(u).head<3>();
        accum_s += (pos - last_pos).norm();
        u_values_.push_back(u);
        s_values_.push_back(accum_s);
        last_pos = pos;
    }
}

double ArcLengthTable::getUfromS(double s) const
{
    if (u_values_.empty() || s_values_.empty()) {
        return 0.0;
    }
    if (s <= s_values_.front()) {
        return u_values_.front();
    }
    if (s >= s_values_.back()) {
        return u_values_.back();
    }

    auto upper = std::lower_bound(s_values_.begin(), s_values_.end(), s);
    const size_t idx = static_cast<size_t>(std::distance(s_values_.begin(), upper));
    const double s0 = s_values_[idx - 1];
    const double s1 = s_values_[idx];
    const double u0 = u_values_[idx - 1];
    const double u1 = u_values_[idx];
    const double alpha = (s - s0) / std::max(s1 - s0, 1e-6);
    return u0 + alpha * (u1 - u0);
}

double ArcLengthTable::getSfromU(double u) const
{
    if (u_values_.empty() || s_values_.empty()) {
        return 0.0;
    }
    if (u <= u_values_.front()) {
        return s_values_.front();
    }
    if (u >= u_values_.back()) {
        return s_values_.back();
    }

    auto upper = std::lower_bound(u_values_.begin(), u_values_.end(), u);
    const size_t idx = static_cast<size_t>(std::distance(u_values_.begin(), upper));
    const double u0 = u_values_[idx - 1];
    const double u1 = u_values_[idx];
    const double s0 = s_values_[idx - 1];
    const double s1 = s_values_[idx];
    const double alpha = (u - u0) / std::max(u1 - u0, 1e-6);
    return s0 + alpha * (s1 - s0);
}

double ArcLengthTable::totalLength() const
{
    return s_values_.empty() ? 0.0 : s_values_.back();
}

bool ArcLengthTable::empty() const
{
    return u_values_.empty() || s_values_.empty();
}

ScpMpcTrajectory::ScpMpcTrajectory(ros::NodeHandle *nh) : nh_(*nh), pnh_("~")
{  
    // 读取参数
    pnh_.param("N", N_, 20);
    pnh_.param("dt", dt_, 0.1);
    pnh_.param("max_vel", max_vel_, 5.0);
    pnh_.param("max_acc", max_acc_, 5.0);
    pnh_.param("max_jerk", max_jerk_, 10.0);
    pnh_.param("w_pos", w_pos_, 50.0);
    pnh_.param("w_terminal_pos", w_terminal_pos_, 200.0);
    pnh_.param("w_jerk", w_jerk_, 0.1);
    pnh_.param("w_jerk_delta", w_jerk_delta_, 0.05);
    pnh_.param("bspline_topic", bspline_topic_, std::string("/drone_0_planning/bspline"));
    pnh_.param("cmd_topic", cmd_topic_, std::string("mpc_pose_cmd"));
    pnh_.param("command_step", command_step_, 5);
    pnh_.param("acc_lpf_alpha", acc_lpf_alpha_, 0.35);
    command_step_ = std::max(1, std::min(command_step_, N_));
    acc_lpf_alpha_ = std::max(0.0, std::min(acc_lpf_alpha_, 1.0));

    // 初始化 MPC 优化器
    mpc_ = std::make_unique<MpcSmoother>(MpcSmoother::Config{N_, dt_, max_jerk_, max_acc_, max_vel_, w_pos_, w_terminal_pos_, w_jerk_, w_jerk_delta_});

    bspline_sub_ = nh_.subscribe<traj_utils::Bspline>(bspline_topic_, 1, &ScpMpcTrajectory::bspline_cb, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_up", 50, &ScpMpcTrajectory::odom_cb, this);

    //通过publisher实现对无人机的速度控制和姿态控制和角速度控制
    cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(cmd_topic_, 1);



    ros::spin();
}

ScpMpcTrajectory::~ScpMpcTrajectory()
{
}

void ScpMpcTrajectory::bspline_cb(const traj_utils::BsplineConstPtr& msg)
{
    if (msg->pos_pts.size() < static_cast<size_t>(msg->order + 1) || msg->knots.size() < msg->pos_pts.size()) {
        ROS_WARN("Ignore invalid Bspline: order=%d, pos_pts=%zu, knots=%zu",
                 msg->order, msg->pos_pts.size(), msg->knots.size());
        return;
    }

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
        pos_pts(0, i) = msg->pos_pts[i].x;
        pos_pts(1, i) = msg->pos_pts[i].y;
        pos_pts(2, i) = msg->pos_pts[i].z;
    }

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) {
        knots(static_cast<int>(i)) = msg->knots[i];
    }

    const double interval = msg->knots.size() >= 2 ? msg->knots[1] - msg->knots[0] : dt_;
    bspline_ = ego_planner::UniformBspline(pos_pts, msg->order, interval);
    bspline_.setKnot(knots);

    bspline_duration_ = bspline_.getTimeSum();
    arc_table_.build(bspline_, 0.0, bspline_duration_, 500);
    ref_ready_ = true;
    last_u_ = 0.0;

    ROS_INFO("Received Bspline traj_id=%ld, order=%d, ctrl_pts=%zu, duration=%.3f, arc_len=%.3f",
             msg->traj_id, msg->order, msg->pos_pts.size(), bspline_duration_, arc_table_.totalLength());
}

void ScpMpcTrajectory::odom_cb(const nav_msgs::OdometryConstPtr& msg)
{
    if (!ref_ready_) {
        return;
    }

    const Eigen::Vector3d current_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const Eigen::Vector3d current_vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    if (have_last_odom_) {
        const double odom_dt = (stamp - last_odom_stamp_).toSec();
        if (odom_dt > 1e-4 && odom_dt < 0.2) {
            const Eigen::Vector3d raw_acc = (current_vel - last_odom_vel_) / odom_dt;
            estimated_acc_ = (1.0 - acc_lpf_alpha_) * estimated_acc_ + acc_lpf_alpha_ * raw_acc;
        }
    } else {
        estimated_acc_.setZero();
        have_last_odom_ = true;
    }
    last_odom_vel_ = current_vel;
    last_odom_stamp_ = stamp;

    Eigen::MatrixXd ref_pos;
    if (buildReferenceByArcLength(current_pos, max_vel_ * dt_, ref_pos)) {
        Eigen::MatrixXd out_states, out_inputs;
        const Eigen::Matrix<double, 9, 1> x0 = (Eigen::Matrix<double, 9, 1>() << current_pos,
                                                 current_vel,
                                                 estimated_acc_).finished();
        ROS_INFO("Solving MPC with x0=[pos=(%.2f, %.2f, %.2f), vel=(%.2f, %.2f, %.2f), est_acc=(%.2f, %.2f, %.2f)], ref_pos[0]=(%.2f, %.2f, %.2f)",
                 x0(0), x0(1), x0(2),
                 x0(3), x0(4), x0(5),
                 x0(6), x0(7), x0(8),
                 ref_pos(0, 0), ref_pos(1, 0), ref_pos(2, 0));
        // 输出ref_pos测试
        for (int k = 0; k <= N_; ++k) {
            ROS_INFO("ref_pos[%d] = (%.2f, %.2f, %.2f)", k, ref_pos(0, k), ref_pos(1, k), ref_pos(2, k));
        }
        if (mpc_->solve(x0, ref_pos, out_states, out_inputs)) {
            const int cmd_idx = std::max(1, std::min(command_step_, N_));
            quadrotor_msgs::PositionCommand cmd;
            cmd.header.stamp = ros::Time::now();
            cmd.position.x = out_states(0, cmd_idx);
            cmd.position.y = out_states(1, cmd_idx);
            cmd.position.z = out_states(2, cmd_idx);
            cmd.velocity.x = out_states(3, cmd_idx);
            cmd.velocity.y = out_states(4, cmd_idx);
            cmd.velocity.z = out_states(5, cmd_idx);
            cmd.acceleration.x = out_states(6, cmd_idx);
            cmd.acceleration.y = out_states(7, cmd_idx);
            cmd.acceleration.z = out_states(8, cmd_idx);
            // 偏航用速度方向
            // if (std::hypot(out_states(3, cmd_idx), out_states(4, cmd_idx)) > 0.1) {
            //     cmd.yaw = std::atan2(out_states(4, cmd_idx), out_states(3, cmd_idx));
            // } else {
            //     cmd.yaw = 0.0;
            // }
            cmd.yaw = 0.0; // 暂时不控制偏航，保持0度
            cmd.yaw_dot = 0.0;
            cmd_pub_.publish(cmd);
            ROS_INFO("Published cmd[%d]: pos=(%.2f, %.2f, %.2f), vel=(%.2f, %.2f, %.2f), acc=(%.2f, %.2f, %.2f), yaw=%.2f, est_acc=(%.2f, %.2f, %.2f)",
                     cmd_idx,
                     cmd.position.x, cmd.position.y, cmd.position.z,
                     cmd.velocity.x, cmd.velocity.y, cmd.velocity.z,
                     cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z,
                     cmd.yaw,
                     estimated_acc_.x(), estimated_acc_.y(), estimated_acc_.z());
        }
        else {
            ROS_WARN("MPC solve failed");
            // 发布当前状态减速指令（简单后备）
            quadrotor_msgs::PositionCommand cmd;
            cmd.header.stamp = ros::Time::now();
            cmd.position = msg->pose.pose.position;
            cmd.velocity.x = 0.0;
            cmd.velocity.y = 0.0;
            cmd.velocity.z = 0.0;
            cmd.acceleration.x = 0.0;
            cmd.acceleration.y = 0.0;
            cmd.acceleration.z = 0.0;
            cmd.yaw = 0.0;
            cmd.yaw_dot = 0.0;
            cmd_pub_.publish(cmd);  
        }
    }
}

double ScpMpcTrajectory::findClosestU(const Eigen::Vector3d& pos,
                                      double seed_u,
                                      double min_u,
                                      double max_u,
                                      int samples)
{
    if (!ref_ready_ || samples < 2 || max_u <= min_u) {
        return min_u;
    }

    const double search_radius = std::max(2.0, max_vel_ * dt_ * static_cast<double>(N_));
    const double u0 = std::max(min_u, seed_u - search_radius);
    const double u1 = std::min(max_u, seed_u + search_radius);

    double best_u = std::min(std::max(seed_u, min_u), max_u);
    double best_dist2 = std::numeric_limits<double>::infinity();

    for (int i = 0; i <= samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples);
        const double u = u0 + ratio * (u1 - u0);
        const Eigen::Vector3d p = bspline_.evaluateDeBoorT(u).head<3>();
        const double dist2 = (p - pos).squaredNorm();
        if (dist2 < best_dist2) {
            best_dist2 = dist2;
            best_u = u;
        }
    }

    return best_u;
}

bool ScpMpcTrajectory::buildReferenceByArcLength(const Eigen::Vector3d& current_pos,
                                                 double ds,
                                                 Eigen::MatrixXd& ref_pos)
{
    if (!ref_ready_ || arc_table_.empty() || bspline_duration_ <= 0.0 || ds <= 0.0) {
        return false;
    }

    const double u_now = findClosestU(current_pos, last_u_, 0.0, bspline_duration_, 80);
    last_u_ = u_now;

    const double s_now = arc_table_.getSfromU(u_now);
    ref_pos.resize(3, N_ + 1);
    for (int k = 0; k <= N_; ++k) {
        const double u = arc_table_.getUfromS(s_now + static_cast<double>(k) * ds);
        ref_pos.col(k) = bspline_.evaluateDeBoorT(u).head<3>();
    }

    return true;
}
