#ifndef _SCP_MPC_TRAJECTORY_HPP_
#define _SCP_MPC_TRAJECTORY_HPP_

#include <ros/ros.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include  "sensor_msgs/Imu.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "traj_utils/Bspline.h"
#include "bspline_opt/uniform_bspline.h"
#include "mpc_smoother.hpp"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include <string>
#include <vector>

class ArcLengthTable
{
public:
    void build(ego_planner::UniformBspline& bspline, double u0, double u1, int samples);
    double getUfromS(double s) const;
    double getSfromU(double u) const;
    double totalLength() const;
    bool empty() const;

private:
    std::vector<double> u_values_;
    std::vector<double> s_values_;
};

class ScpMpcTrajectory
{
public:
    ScpMpcTrajectory(ros::NodeHandle *nh);
    ~ScpMpcTrajectory();

    void bspline_cb(const traj_utils::BsplineConstPtr& msg);
    void odom_cb(const nav_msgs::OdometryConstPtr& msg);
    double findClosestU(const Eigen::Vector3d& pos, double seed_u, double min_u, double max_u, int samples);
    bool buildReferenceByArcLength(const Eigen::Vector3d& current_pos, double ds, Eigen::MatrixXd& ref_pos);

    // ---------- 成员变量 ----------
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber bspline_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher  cmd_pub_;
    ros::Timer      timer_;

    std::unique_ptr<MpcSmoother> mpc_;
    ego_planner::UniformBspline bspline_;
    ArcLengthTable    arc_table_;
    bool ref_ready_ = false;
    double last_u_ = 0.0;
    double bspline_duration_ = 0.0;
    std::string bspline_topic_;
    std::string cmd_topic_;
    Eigen::Vector3d last_odom_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d estimated_acc_ = Eigen::Vector3d::Zero();
    ros::Time last_odom_stamp_;
    bool have_last_odom_ = false;
    int command_step_ = 5;
    double acc_lpf_alpha_ = 0.35;

    int N_;
    double dt_, max_vel_, max_acc_, max_jerk_;
    double w_pos_, w_terminal_pos_, w_jerk_, w_jerk_delta_;

};

#endif
