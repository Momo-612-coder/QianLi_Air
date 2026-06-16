#ifndef _AB_PLANNER_HPP_
#define _AB_PLANNER_HPP_

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include <boost/thread/thread.hpp>

#include "quadrotor_msgs/PositionCommand.h"
#include "lidar_solver/ObstacleArray.h"
#include <deque>
#include <memory>
#include <mutex>
#include <fstream>
#include <sstream>
#include <string>

#include "A_star.hpp"
#include "B_spline.hpp"
#include "mpc_smoother.hpp"

enum PlannerState
{
    IDLE, // 空闲状态，等待起飞命令或目标点
    TOPLAN, // 规划状态，正在进行路径规划
    EXECUTING, // 执行状态，正在执行规划好的路径
    SEGMENT_TURNING, // 分段结束后悬停并掉头
    RECOVERY_NUDGE // 规划连续失败后进行小幅换视角脱困
};

class ABPlanner
{
public:
    ABPlanner(ros::NodeHandle *nh);
    ~ABPlanner();

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    ros::Subscriber odom_suber;//状态真值
    ros::Subscriber start_state_suber_;//融合时间对齐完成信号
    ros::Subscriber obstacle_suber_;//lidar_solver输出的前方障碍物

    //publisher
    ros::Publisher control_cmd_pub_;
    ros::Publisher planned_path_pub_;
    ros::Publisher bspline_path_pub_;
    ros::Publisher mpc_prediction_path_pub_;

    // timer
    ros::Timer planner_timer_;

    void PlannerCallback(const ros::TimerEvent& event);
    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void start_state_cb(const std_msgs::Header::ConstPtr& msg);
    void obstacle_cb(const lidar_solver::ObstacleArray::ConstPtr& msg);
    void readWaypointsFromFile(const std::string& filename, std::vector<Eigen::Vector3d>& waypoints);
    bool planNextWaypointSegment();
    bool planNextWaypoint();
    std::vector<BoxObstacle> freshDynamicObstacles() const;
    void pruneObstacleHistory(const ros::Time& now);
    void updateActiveObstaclesFromHistory();
    bool isCurrentBsplineThreatenedByObstacles(const std::vector<BoxObstacle>& obstacles) const;
    bool hasFreshBoxObstacleData() const;
    bool hasFreshObstacleData() const;
    bool shouldReplanForObstacles(const ros::Time& now) const;
    bool isPathExecutionComplete(Eigen::Vector3d current_position, Eigen::Vector3d current_target_point);
    bool isPathExecutionProgressing(Eigen::Vector3d current_position, Eigen::Vector3d current_target_point);
    void publishPlannedPath(const std::vector<std::vector<Eigen::Vector3d>>& paths);
    bool buildAndPublishBsplinePath(const std::vector<std::vector<Eigen::Vector3d>>& paths);
    std::vector<Eigen::Vector3d> optimizePathForBspline(const std::vector<Eigen::Vector3d>& raw_points) const;
    std::vector<Eigen::Vector3d> makeBsplineControlPoints(const std::vector<Eigen::Vector3d>& points) const;
    std::vector<Eigen::Vector3d> makeBsplineControlPoints(const std::vector<Eigen::Vector3d>& points,
                                                          double spacing) const;
    bool isPointInsideInflatedObstacle(const Eigen::Vector3d& point,
                                       const std::vector<BoxObstacle>& obstacles) const;
    bool isBsplineCollisionFree(const BSpline& bspline,
                                const std::vector<BoxObstacle>& obstacles,
                                double sample_step,
                                const std::vector<Eigen::Vector3d>& clearance_points) const;
    double computePathLength(const std::vector<Eigen::Vector3d>& points) const;
    ros::Time alignedStamp() const;

    // 当前状态 [px,py,pz,vx,vy,vz,ax,ay,az]
    // MPC
    Eigen::Matrix<double, 9, 1> extractState() const;
    Eigen::Matrix<double, 9, 1> sanitizeMpcState(const Eigen::Matrix<double, 9, 1>& state) const;
    bool isMpcPredictionProgressing(const Eigen::MatrixXd& out_states, const Eigen::MatrixXd& ref_traj) const;
    Eigen::MatrixXd generateRefTraj(ArcLengthTable arc_length_table, BSpline bspline, double u_start, int N, double dt);
    Eigen::MatrixXd generateContinuousRefTraj(double u_start, int N, double dt);
    void publishReferenceFallbackCommand(const Eigen::MatrixXd& ref_traj, const std::string& reason);
    void publishControlCommand(bool solve_sussess, const Eigen::MatrixXd& out_states, const Eigen::MatrixXd& out_inputs);
    void limitStartupVerticalCommand(Eigen::Vector3d& cmd_position,
                                     Eigen::Vector3d& cmd_velocity,
                                     Eigen::Vector3d& cmd_acceleration) const;
    bool startRecoveryNudge();
    bool isRecoveryNudgeCollisionFree(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& target,
                                      const std::vector<BoxObstacle>& obstacles) const;
    void publishRecoveryNudgeCommand();
    void startSegmentTurnaround();
    void publishSegmentTurnaroundCommand();
    void publishMpcPredictionPath(const Eigen::MatrixXd& out_states);
    double planYaw(const Eigen::Vector3d& target_position,
                   const Eigen::Vector3d& target_velocity,
                   const ros::Time& stamp,
                   double& yaw_dot);
    static double normalizeAngle(double angle);
    static double interpolateYaw(double yaw0, double yaw1, double alpha);

    // ---------- 类成员变量 ----------
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // A*算法相关参数
    std::string visualization_frame_;
    double plan_size_x_;
    double plan_size_y_;
    double plan_size_z_;
    double resolution_;
    double reserved_back_space_;
    double safety_distance_;
    double h_weight_;
    std::string odometry_topic_;
    std::string fusion_start_topic_;
    std::string obstacle_topic_;
    int bspline_visualization_samples_ = 500;
    double acc_lpf_alpha_ = 0.35;
    int mpc_command_step_ = 10;
    bool convert_ned_to_position_cmd_ = false;
    int plan_waypoint_lookahead_ = 8;
    double replan_trigger_distance_ = 25.0;
    double planning_failure_retry_interval_ = 0.25;
    double bspline_control_point_spacing_ = 2.0;
    double waypoint_reached_distance_ = 8.0;
    bool use_waypoint_yaw_ = true;
    double max_yaw_rate_ = 1.5;
    bool segment_turnaround_enabled_ = true;
    double segment_turnaround_duration_ = 5.0;
    double segment_turnaround_yaw_delta_ = 3.14159265358979323846;
    bool path_optimization_enabled_ = true;
    int path_optimization_iterations_ = 80;
    double path_optimization_data_weight_ = 0.20;
    double path_optimization_smooth_weight_ = 0.45;
    double path_optimization_z_smooth_weight_ = 0.25;
    double path_optimization_max_deviation_ = 1.5;
    double path_optimization_max_z_slope_ = 0.45;
    double path_optimization_min_turn_radius_ = 6.0;
    bool bspline_collision_check_enabled_ = true;
    double bspline_collision_sample_step_ = 0.3;
    double bspline_collision_extra_clearance_ = 0.2;
    bool dynamic_avoidance_enabled_ = true;
    bool publish_rviz_visualization_ = true;
    double obstacle_replan_period_ = 0.5;
    double obstacle_stale_time_ = 0.5;
    double obstacle_history_time_ = 0.8;
    int obstacle_history_max_frames_ = 3;
    double obstacle_merge_distance_ = 0.8;
    bool immediate_replan_on_collision_ = true;
    double obstacle_min_replan_interval_ = 0.2;
    double dynamic_obstacle_inflation_ = 2.5;
    double dynamic_obstacle_endpoint_clear_radius_ = 2.0;
    bool clear_obstacle_history_on_empty_frame_ = true;
    int obstacle_threat_confirm_frames_ = 2;
    int obstacle_threat_clear_frames_ = 3;
    int astar_max_search_nodes_ = 500000;
    double fallback_max_speed_ = 8.0;
    double fallback_max_vertical_speed_ = 3.0;
    bool startup_altitude_hold_enabled_ = true;
    double startup_altitude_hold_distance_ = 8.0;
    double startup_altitude_hold_margin_ = 0.3;
    bool planning_recovery_enabled_ = true;
    int planning_recovery_failure_threshold_ = 2;
    double planning_recovery_step_ = 0.5;
    double planning_recovery_duration_ = 1.5;
    double planning_recovery_reached_distance_ = 0.2;
    double planning_recovery_stuck_speed_ = 0.5;
    bool planning_recovery_allow_vertical_ = false;

    double distance_to_target_threshold_ = 2.0; // 距离阈值，单位为米
    double distance_to_progress_threshold_ = 3.0; // 距离阈值，单位为米，用于判断路径执行是否正在进行
    bool have_odom_ = false;
    bool have_start_state_ = false;
    bool have_obstacles_ = false;
    bool last_plan_used_dynamic_obstacles_ = false;
    bool obstacle_path_threatened_ = false;
    int obstacle_threat_hit_count_ = 0;
    int obstacle_threat_clear_count_ = 0;
    bool time_waited_after_start_state_ = false;
    int consecutive_planning_failures_ = 0;
    int recovery_nudge_attempt_count_ = 0;
    ros::Time latest_odom_stamp_;
    ros::Time start_state_time_;
    ros::Time latest_obstacle_stamp_;
    ros::Time last_obstacle_replan_time_;
    ros::Time next_planning_attempt_time_;

    PlannerState planner_state_;

    AStar a_star_planner_;
    std::vector<BoxObstacle> latest_dynamic_obstacles_;
    struct ObstacleFrame
    {
        ros::Time stamp;
        std::vector<BoxObstacle> obstacles;
    };
    std::deque<ObstacleFrame> obstacle_history_;
    std::vector<int> segment_idx_; // 将规划路径分段，如68则分为0-68和68-末尾两段，方便后续进行分段优化
    std::vector<double> yaws_; // 每个路径点对应的航向角，单位为弧度
    std::vector<Eigen::Vector3d> waypoints_;
    std::vector<double> segment_yaws_; // 当前分段的航向角，单位为弧度
    std::vector<Eigen::Vector3d> segment_waypoints_; // 当前分段的路径点列表，单位为世界坐标
    std::vector<bool> skipped_segment_waypoints_; // 当前分段中因不可达而跳过的局部航点
    std::vector<std::vector<Eigen::Vector3d>> planned_path_; // 最近一次规划出的路径，单位为世界坐标
    int current_segment_ = 0; // 当前规划的路径段索引
    int current_waypoint_idx_ = 0; // 当前分段中已经通过的路径点索引
    int planned_window_end_idx_ = 0; // 当前已规划滚动窗口的末端路径点索引
    Eigen::Vector3d target_goal_; // 当前规划的目标点，单位为世界坐标

    // B样条与弧长表相关参数
    double max_vel_ = 15.0; // MPC参考轨迹的期望速度，单位为米/秒

    std::vector<BSpline> bsplines_; // 每段路径对应一个B样条，方便后续进行基于B样条的均匀采样和最近点投影
    std::vector<ArcLengthTable> arc_length_tables_; // 每段路径对应一个弧长表，方便后续进行基于弧长的均匀采样和最近点投影
    BSpline current_bspline;// 当前正在执行路径段的B样条
    ArcLengthTable current_arc_table; // 当前正在执行路径段的弧长表
    int current_bspline_segment_idx_ = 0; // 当前正在执行路径段的索引
    double last_u_ = 0.0; // 上一次最近点投影得到的参数值，单位为B样条参数u

    // MPC相关参数
    MpcSmoother::Config mpc_config_;
    std::unique_ptr<MpcSmoother> mpc_;

    // 无人机状态信息
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_acceleration_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond current_orientation_;
    Eigen::Vector3d current_angular_velocity_;
    Eigen::Vector3d last_odom_velocity_ = Eigen::Vector3d::Zero();
    double current_yaw_ = 0.0;
    double last_yaw_ = 0.0;
    ros::Time last_yaw_command_stamp_;
    bool have_last_yaw_command_ = false;
    Eigen::Vector3d segment_turnaround_position_ = Eigen::Vector3d::Zero();
    ros::Time segment_turnaround_start_stamp_;
    double segment_turnaround_start_yaw_ = 0.0;
    double segment_turnaround_target_yaw_ = 0.0;
    Eigen::Vector3d recovery_nudge_start_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d recovery_nudge_target_position_ = Eigen::Vector3d::Zero();
    ros::Time recovery_nudge_start_stamp_;
    quadrotor_msgs::PositionCommand last_successful_cmd_;
    bool have_last_successful_cmd_ = false;
    ros::Time last_odom_acc_stamp_;
    bool have_last_odom_acc_ = false;
};

#endif
