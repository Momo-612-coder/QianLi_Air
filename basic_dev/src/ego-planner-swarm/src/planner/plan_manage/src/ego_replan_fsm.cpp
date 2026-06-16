
#include <plan_manage/ego_replan_fsm.h>

#include <algorithm>
#include <fstream>
#include <sstream>

#include <geometry_msgs/PointStamped.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    local_target_speed_limit_ = -1.0;
    segment_start_wp_ = 0;
    segment_end_wp_ = -1;
    waiting_at_stop_wp_ = false;
    yaw_turn_sent_at_stop_wp_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/tracking_error_replan_thresh", tracking_error_replan_thresh_, 3.0);
    nh.param("fsm/tracking_error_replan_cooldown", tracking_error_replan_cooldown_, 0.5);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/use_waypoint_file", use_waypoint_file_, false);
    nh.param("fsm/waypoint_file", waypoint_file_, std::string(""));
    nh.param("fsm/waypoint_input_frame", waypoint_input_frame_, std::string("map"));
    nh.param("fsm/planner_frame", planner_frame_, std::string("camera_init"));
    nh.param("fsm/waypoint_tf_timeout", waypoint_tf_timeout_, 0.05);
    nh.param("fsm/stop_waypoint_index", stop_waypoint_index_, 68);
    nh.param("fsm/stop_waypoint_wait_time", stop_waypoint_wait_time_, 3.0);
    nh.param("fsm/stop_yaw_position_tolerance", stop_yaw_position_tolerance_, 1.0);
    nh.param("fsm/stop_yaw_velocity_tolerance", stop_yaw_velocity_tolerance_, 0.5);

    stop_waypoint_wait_time_ = 5.0;

    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    waypoints_.clear();
    raw_waypoint_wait_times_.clear();
    if (waypoint_num_ > 0)
    {
      waypoints_.reserve(waypoint_num_);
      raw_waypoint_wait_times_.reserve(waypoint_num_);
    }
    for (int i = 0; i < waypoint_num_; i++)
    {
      Eigen::Vector3d wp;
      nh.param("fsm/waypoint" + to_string(i) + "_x", wp[0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", wp[1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", wp[2], -1.0);
      waypoints_.push_back(wp);
      raw_waypoint_wait_times_.push_back(0.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, &EGOReplanFSM::swarmTrajsCallback, this, ros::TransportHints().tcpNoDelay());
    }
    string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    swarm_trajs_pub_ = nh.advertise<traj_utils::MultiBsplines>(pub_topic_name.c_str(), 10);

    broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100, &EGOReplanFSM::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());

    bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    yaw_turn_pub_ = nh.advertise<std_msgs::Empty>("planning/yaw_turn_180", 1);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 1000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      start_state_sub_ = nh.subscribe("control_start", 1, &EGOReplanFSM::startStateCallback, this);

      // 等待 start_state 消息；收到后继续等待 2 秒，再开始读取航点并规划
      while (ros::ok())
      {
        ros::spinOnce();

        if (have_start_state_)
        {
          double elapsed = (ros::Time::now() - start_state_time_).toSec();

          if (elapsed >= 2.0)
          {
            // 输出时间间隔
            ROS_INFO("Received start state message. Starting to read waypoints and plan...\n");
            ROS_INFO("Time since start state message: %f seconds.\n", elapsed);
            break;
          }
        }

        ros::Duration(0.001).sleep();
      }

      ROS_WARN("planner wait done: ros_now=%.6f, start_state_time=%.6f, elapsed=%.3f",
         ros::Time::now().toSec(), start_state_time_.toSec(),
         (ros::Time::now() - start_state_time_).toSec());

      if (use_waypoint_file_ && !loadWaypointsFromFile())
      {
        ROS_ERROR("Failed to load waypoint file: %s", waypoint_file_.c_str());
        return;
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  bool EGOReplanFSM::loadWaypointsFromFile()
  {
    if (waypoint_file_.empty())
    {
      ROS_ERROR("Parameter fsm/waypoint_file is empty.");
      return false;
    }

    std::ifstream file(waypoint_file_);
    if (!file.is_open())
    {
      ROS_ERROR("Cannot open waypoint file: %s", waypoint_file_.c_str());
      return false;
    }

    waypoints_.clear();

    string line;
    int line_no = 0;
    while (std::getline(file, line))
    {
      line_no++;
      size_t first_char = line.find_first_not_of(" \t\r\n");
      if (first_char == string::npos || line[first_char] == '#')
        continue;

      std::replace(line.begin(), line.end(), ',', ' ');
      std::stringstream ss(line);
      double x, y, z;
      if (!(ss >> x >> y >> z))
      {
        ROS_WARN("Skip malformed waypoint line %d in %s", line_no, waypoint_file_.c_str());
        continue;
      }

      waypoints_.push_back(Eigen::Vector3d(x, y, z));
      raw_waypoint_wait_times_.push_back(0.0);
    }

    if (waypoints_.empty())
    {
      ROS_ERROR("No valid waypoint is parsed from file: %s", waypoint_file_.c_str());
      return false;
    }

    waypoint_num_ = static_cast<int>(waypoints_.size());
    ROS_INFO("Loaded %d waypoint(s) from %s", waypoint_num_, waypoint_file_.c_str());
    return true;
  }


  bool EGOReplanFSM::transformWaypointToPlannerFrame(const Eigen::Vector3d &pt_in, const string &source_frame, Eigen::Vector3d &pt_out)
  {
    if (source_frame.empty() || planner_frame_.empty())
    {
      ROS_ERROR("Invalid source/planner frame. source_frame='%s', planner_frame='%s'", source_frame.c_str(), planner_frame_.c_str());
      return false;
    }

    if (source_frame == planner_frame_)
    {
      pt_out = pt_in;
      return true;
    }

    geometry_msgs::PointStamped src_pt;
    src_pt.header.stamp = ros::Time(0);
    src_pt.header.frame_id = source_frame;
    src_pt.point.x = pt_in.x();
    src_pt.point.y = pt_in.y();
    src_pt.point.z = pt_in.z();

    try
    {
      geometry_msgs::TransformStamped tf_map_to_planner =
          tf_buffer_->lookupTransform(planner_frame_, source_frame, ros::Time(0), ros::Duration(waypoint_tf_timeout_));
      geometry_msgs::PointStamped transformed_pt;
      tf2::doTransform(src_pt, transformed_pt, tf_map_to_planner);

      pt_out(0) = transformed_pt.point.x;
      pt_out(1) = transformed_pt.point.y;
      pt_out(2) = transformed_pt.point.z;
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN("Failed waypoint transform from %s to %s: %s", source_frame.c_str(), planner_frame_.c_str(), ex.what());
      return false;
    }
  }

  void EGOReplanFSM::readGivenWps()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.clear();
    waypoint_wait_times_.clear();
    wps_.reserve(waypoint_num_);
    waypoint_wait_times_.reserve(waypoint_num_);
    for (size_t i = 0; i < waypoints_.size(); i++)
    {
      Eigen::Vector3d wp_in_planner;
      if (!transformWaypointToPlannerFrame(waypoints_[i], waypoint_input_frame_, wp_in_planner))
      {
        ROS_WARN("Drop waypoint[%zu] due to transform failure.", i);
        continue;
      }
      wps_.push_back(wp_in_planner);
      waypoint_wait_times_.push_back(i < raw_waypoint_wait_times_.size() ? raw_waypoint_wait_times_[i] : 0.0);
    }

    waypoint_num_ = static_cast<int>(wps_.size());
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("No valid waypoint after frame transform.");
      return;
    }

    if (stop_waypoint_index_ >= 0 && stop_waypoint_index_ < waypoint_num_ && stop_waypoint_wait_time_ > 1e-3)
    {
      waypoint_wait_times_[stop_waypoint_index_] = stop_waypoint_wait_time_;
      ROS_INFO("Use waypoint[%d] as %.2fs stop point.", stop_waypoint_index_, stop_waypoint_wait_time_);
    }

    ROS_INFO("Converted %d waypoint(s) into planner frame '%s'", waypoint_num_, planner_frame_.c_str());

    for (size_t i = 0; i < wps_.size(); i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    segment_start_wp_ = 0;
    segment_end_wp_ = -1;
    waiting_at_stop_wp_ = false;
    yaw_turn_sent_at_stop_wp_ = false;
    if (!planNextWaypointSegment())
      ROS_ERROR("Unable to generate first waypoint segment trajectory!");
  }

  bool EGOReplanFSM::planNextWaypointSegment()
  {
    if (segment_start_wp_ < 0 || segment_start_wp_ >= waypoint_num_)
      return false;

    segment_end_wp_ = waypoint_num_ - 1;
    for (int i = segment_start_wp_; i < waypoint_num_; ++i)
    {
      if (i < static_cast<int>(waypoint_wait_times_.size()) && waypoint_wait_times_[i] > 1e-3)
      {
        segment_end_wp_ = i;
        break;
      }
    }

    vector<Eigen::Vector3d> segment_wps;
    segment_wps.reserve(segment_end_wp_ - segment_start_wp_ + 1);
    for (int i = segment_start_wp_; i <= segment_end_wp_; ++i)
      segment_wps.push_back(wps_[i]);

    vector<Eigen::Vector3d> waypoint_vels(segment_wps.size(), Eigen::Vector3d::Zero());
    vector<double> waypoint_speeds(segment_wps.size(), planner_manager_->pp_.max_vel_);
    waypoint_speeds.back() = 0.0;
    if (segment_wps.size() >= 2)
      waypoint_speeds[segment_wps.size() - 2] = planner_manager_->pp_.max_vel_ / 3.0;
    if (segment_wps.size() >= 3)
      waypoint_speeds[segment_wps.size() - 3] = planner_manager_->pp_.max_vel_ / 2.0;

    const double stop_acc = 0.7 * planner_manager_->pp_.max_acc_;
    for (int i = static_cast<int>(segment_wps.size()) - 2; i >= 0; --i)
    {
      const double dist = (segment_wps[i + 1] - segment_wps[i]).norm();
      const double stop_reachable_speed = sqrt(waypoint_speeds[i + 1] * waypoint_speeds[i + 1] + 2.0 * stop_acc * dist);
      waypoint_speeds[i] = std::min(waypoint_speeds[i], stop_reachable_speed);
    }

    for (size_t i = 0; i + 1 < segment_wps.size(); ++i)
    {
      Eigen::Vector3d dir = Eigen::Vector3d::Zero();
      Eigen::Vector3d prev_pt = (i == 0) ? odom_pos_ : segment_wps[i - 1];
      Eigen::Vector3d dir_prev = segment_wps[i] - prev_pt;
      Eigen::Vector3d dir_next = segment_wps[i + 1] - segment_wps[i];

      if (dir_prev.norm() > 1e-3 && dir_next.norm() > 1e-3)
      {
        dir_prev.normalize();
        dir_next.normalize();
        Eigen::Vector3d bisector = dir_prev + dir_next;
        dir = bisector.norm() > 1e-3 ? bisector : dir_next;
      }
      else
      {
        dir = dir_next.norm() > 1e-3 ? dir_next : dir_prev;
      }

      if (dir.norm() > 1e-3)
        waypoint_vels[i] = dir.normalized() * waypoint_speeds[i];
    }
    waypoint_vels.back().setZero();

    bool success = planner_manager_->planGlobalTrajWaypointsWithVels(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        segment_wps, waypoint_vels, Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("Unable to generate waypoint segment trajectory: [%d, %d]", segment_start_wp_, segment_end_wp_);
      return false;
    }

    end_pt_ = segment_wps.back();
    target_stop_pt_ = end_pt_;
    wp_id_ = segment_end_wp_;
    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    vector<Eigen::Vector3d> global_traj(i_end);
    for (int i = 0; i < i_end; i++)
      global_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);

    visualization_->displayGlobalPathList(global_traj, 0.1, 0);
    ROS_INFO("Plan waypoint segment [%d, %d], stop wait %.2fs",
             segment_start_wp_, segment_end_wp_,
             segment_end_wp_ < static_cast<int>(waypoint_wait_times_.size()) ? waypoint_wait_times_[segment_end_wp_] : 0.0);
    return true;
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;
      target_stop_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    // if (msg->pose.position.z < -0.1)
    //   return;

    cout << "Triggered!" << endl;
    // trigger_ = true;
    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp_raw(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Vector3d end_wp;
    string source_frame = msg->header.frame_id.empty() ? waypoint_input_frame_ : msg->header.frame_id;
    if (!transformWaypointToPlannerFrame(end_wp_raw, source_frame, end_wp))
    {
      ROS_WARN("Ignore manual waypoint due to frame transform failure.");
      return;
    }

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
                msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    /* Fill up the buffer */
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;
    // planner_manager_->swarm_trajs_buf_[id].start_time_ = ros::Time::now(); // Un-reliable time sync

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::startStateCallback(const std_msgs::Header::ConstPtr &msg)
  {
    if (have_start_state_ == false)
    {
      have_start_state_ = true;
      start_state_time_ = msg->stamp.isZero() ? ros::Time::now() : msg->stamp;
      ROS_INFO("Received start state signal.");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    // cout << "\033[45;33mmulti_bspline_msgs_buf.drone_id_from=" << multi_bspline_msgs_buf_.drone_id_from << " multi_bspline_msgs_buf_.traj.size()=" << multi_bspline_msgs_buf_.traj.size() << "\033[0m" << endl;

    if (!have_odom_)
    {
      ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      ROS_ERROR("Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
        // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      // return;
      else
      {
        // if ( planner_manager_->pp_.drone_id <= 0 )
        // {
        //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        // }
        // else
        // {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
        // }
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      // cout << "id=" << planner_manager_->pp_.drone_id << " have_recv_pre_agent_=" << have_recv_pre_agent_ << endl;
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            ROS_ERROR("Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      // LocalTrajData *info = &planner_manager_->local_data_;
      // double t_cur = (ros::Time::now() - info->start_time_).toSec();
      // t_cur = min(info->duration_, std::max(0.0, t_cur));
      // Eigen::Vector3d traj_pos = info->position_traj_.evaluateDeBoorT(t_cur);
      // double tracking_error = (odom_pos_ - traj_pos).norm();
      // bool replan_from_odom = tracking_error_replan_thresh_ > 0.0 && tracking_error > tracking_error_replan_thresh_;

      // if (replan_from_odom)
      // {
      //   ROS_WARN_THROTTLE(0.5,
      //                     "Tracking error %.3f m exceeds %.3f m in REPLAN_TRAJ. Replan from odom. odom=(%.3f %.3f %.3f), traj=(%.3f %.3f %.3f)",
      //                     tracking_error, tracking_error_replan_thresh_,
      //                     odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
      //                     traj_pos.x(), traj_pos.y(), traj_pos.z());
      // }

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      //ROS_INFO("t_cur=%.2f, traj_duration=%.2f", t_cur, info->duration_);
      ROS_INFO_THROTTLE(1.0, "odom=(%.3f %.3f %.3f), t_cur=%.2f", odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
      // double tracking_error = (odom_pos_ - pos).norm();
      // bool tracking_error_too_large =
      //     tracking_error_replan_thresh_ > 0.0 && tracking_error > tracking_error_replan_thresh_;

      // if (tracking_error_too_large)
      // {
      //   bool cooldown_passed = last_tracking_error_replan_time_.isZero() ||
      //                          (time_now - last_tracking_error_replan_time_).toSec() >= tracking_error_replan_cooldown_;
      //   ROS_WARN_THROTTLE(0.5,
      //                     "Tracking error %.3f m exceeds %.3f m. odom=(%.3f %.3f %.3f), traj=(%.3f %.3f %.3f), cooldown=%d",
      //                     tracking_error, tracking_error_replan_thresh_,
      //                     odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
      //                     pos.x(), pos.y(), pos.z(), cooldown_passed);

      //   if (cooldown_passed)
      //   {
      //     last_tracking_error_replan_time_ = time_now;
      //     if (planFromGlobalTraj(3))
      //     {
      //       changeFSMExecState(EXEC_TRAJ, "FSM");
      //       publishSwarmTrajs(false);
      //     }
      //     else
      //     {
      //       ROS_WARN("Failed to recover from tracking error with odom-based replan. Retry in GEN_NEW_TRAJ.");
      //       changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //     }
      //     break;
      //   }
      // }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (segment_end_wp_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        const double wait_time = segment_end_wp_ < static_cast<int>(waypoint_wait_times_.size()) ? waypoint_wait_times_[segment_end_wp_] : 0.0;
        if (wait_time > 1e-3)
        {
          if (!waiting_at_stop_wp_)
          {
            waiting_at_stop_wp_ = true;
            yaw_turn_sent_at_stop_wp_ = false;
            break;
          }

          if (!yaw_turn_sent_at_stop_wp_)
          {
            const double stop_pos_error = (odom_pos_ - end_pt_).norm();
            const double stop_vel = odom_vel_.norm();
            if (stop_pos_error > stop_yaw_position_tolerance_ || stop_vel > stop_yaw_velocity_tolerance_)
            {
              ROS_INFO_THROTTLE(0.5,
                                "Wait for stop before yaw turn: pos_error=%.3f/%.3f, vel=%.3f/%.3f",
                                stop_pos_error, stop_yaw_position_tolerance_, stop_vel, stop_yaw_velocity_tolerance_);
              break;
            }

            stop_wait_start_time_ = ros::Time::now();
            callEmergencyStop(end_pt_);
            yaw_turn_pub_.publish(std_msgs::Empty());
            yaw_turn_sent_at_stop_wp_ = true;
            break;
          }

          if ((ros::Time::now() - stop_wait_start_time_).toSec() < wait_time)
          {
            break;
          }
        }

        waiting_at_stop_wp_ = false;
        yaw_turn_sent_at_stop_wp_ = false;
        segment_start_wp_ = segment_end_wp_ + 1;
        if (planNextWaypointSegment())
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        else
          changeFSMExecState(WAIT_TARGET, "FSM");
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
          // return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    static const char *state_str[] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      const int state_id = static_cast<int>(exec_state_);
      const char *state_name = (state_id >= 0 && state_id < 7) ? state_str[state_id] : "UNKNOWN";
      ROS_ERROR("Depth Lost! EMERGENCY_STOP. state=%s, odom=(%.3f %.3f %.3f), vel=(%.3f %.3f %.3f), speed=%.3f",
                state_name, odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
                odom_vel_.x(), odom_vel_.y(), odom_vel_.z(), odom_vel_.norm());
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    double t_cur_global = ros::Time::now().toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d check_pt = info->position_traj_.evaluateDeBoorT(t);
      bool occ = false;
      const char *occ_reason = "free";
      int swarm_collision_id = -1;
      double swarm_collision_dist = -1.0;

      int map_occ = map->getInflateOccupancy(check_pt);
      if (map_occ < 0)
      {
        occ = true;
        occ_reason = "out_of_map";
      }
      else if (map_occ > 0)
      {
        occ = true;
        occ_reason = "map_occupancy";
      }

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.toSec();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          occ_reason = "swarm_clearance";
          swarm_collision_id = planner_manager_->swarm_trajs_buf_.at(id).drone_id;
          swarm_collision_dist = dist;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          ROS_WARN("Trajectory collision risk. reason=%s, dt=%.3f, traj_pt=(%.3f %.3f %.3f), odom=(%.3f %.3f %.3f), vel=(%.3f %.3f %.3f), speed=%.3f, swarm_id=%d, swarm_dist=%.3f, clearance=%.3f",
                   occ_reason, t - t_cur, check_pt.x(), check_pt.y(), check_pt.z(),
                   odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
                   odom_vel_.x(), odom_vel_.y(), odom_vel_.z(), odom_vel_.norm(),
                   swarm_collision_id, swarm_collision_dist, CLEARANCE);
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! reason=%s, time=%f", occ_reason, t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
                                        (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj, local_target_speed_limit_);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      // cout << knots.transpose() << endl;
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_.publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // cout << knots.transpose() << endl;
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        ROS_ERROR("Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_.publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    const double stop_acc = std::max(0.1, 0.5 * planner_manager_->pp_.max_acc_);
    const double dist_target_to_stop = (target_stop_pt_ - local_target_pt_).norm();
    const double dist_start_to_stop = (target_stop_pt_ - start_pt_).norm();
    const double target_brake_speed = sqrt(std::max(0.0, 2.0 * stop_acc * dist_target_to_stop));
    const double start_brake_speed = sqrt(std::max(0.0, 2.0 * stop_acc * dist_start_to_stop));
    local_target_speed_limit_ = std::min(planner_manager_->pp_.max_vel_, start_brake_speed);

    if (dist_target_to_stop < 2.0)
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
      local_target_speed_limit_ = std::max(0.3, local_target_speed_limit_);
    }
    else
    {
      double vel_t = std::min(t, planner_manager_->global_data_.global_duration_);
      local_target_vel_ = planner_manager_->global_data_.getVelocity(vel_t);
      if (local_target_vel_.norm() > target_brake_speed)
        local_target_vel_ = local_target_vel_.normalized() * target_brake_speed;
      if (local_target_vel_.norm() < 1e-3)
      {
        Eigen::Vector3d dir = local_target_pt_ - start_pt_;
        if (dir.norm() > 1e-3)
          local_target_vel_ = dir.normalized() * target_brake_speed;
      }
    }
  }

} // namespace ego_planner
