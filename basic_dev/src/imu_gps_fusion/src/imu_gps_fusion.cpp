#ifndef _IMU_GPS_FUSION_CPP_
#define _IMU_GPS_FUSION_CPP_

#include "imu_gps_fusion.hpp"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "imu_gps_fusion"); // 初始化ros 节点，命名为 imu_gps_fusion
    ros::NodeHandle n; // 创建node控制句柄
    ImuGpsFusion go(&n);
    return 0;
}

ImuGpsFusion::ImuGpsFusion(ros::NodeHandle *nh)
{  
    // 无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    //odom_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, std::bind(&ImuGpsFusion::pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    // 起始姿态/airsim_node/initial_pose
    initial_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, std::bind(&ImuGpsFusion::initial_pose_cb, this, std::placeholders::_1));
    gps_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, std::bind(&ImuGpsFusion::gps_cb, this, std::placeholders::_1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());//状态真值，用于赛道一
    imu_suber = nh->subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, std::bind(&ImuGpsFusion::imu_cb, this, std::placeholders::_1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());//imu数据
    reset_suber = nh->subscribe<std_msgs::Empty>("/airsim_node/reset_cmd", 1, std::bind(&ImuGpsFusion::reset_cb, this, std::placeholders::_1));//重置命令订阅
    // 发布者
    true_path_pub = nh->advertise<nav_msgs::Path>("true_path", 10);
    fused_path_pub = nh->advertise<nav_msgs::Path>("fused_path", 10);
    drone_state_pub = nh->advertise<nav_msgs::Odometry>("/airsim_node/drone_1/drone_state", 1);
    odom_up_pub = nh->advertise<nav_msgs::Odometry>("odom_up", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
    start_state_pub = nh->advertise<std_msgs::Header>("fusion_start", 1, true);

    nh->param<std::string>("target_frame", target_frame_, "map_rviz");
    nh->param<std::string>("source_frame", source_frame_, "map");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    true_path.header.frame_id = "map";
    fused_path.header.frame_id = "map";

    eskf_ = std::make_unique<ESKF>(); // 会自动释放旧的锁并在堆上新建
    eskf_->setQR(GYRO_BIAS_NOISE, ACCEL_BIAS_NOISE, GPS_POSITION_NOISE, GPS_ORIENTATION_NOISE);

    publish_map_rviz_tf();

    startFusionThread();
    ros::spin();
    stopFusionThread();
}

void ImuGpsFusion::publish_map_rviz_tf()
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time(0);
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "map_rviz";
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    // NED -> RViz z-up visualization frame.
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    q.normalize();
    tf_msg.transform.rotation.w = q.w();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();

    static_tf_broadcaster_->sendTransform(tf_msg);
}

ImuGpsFusion::~ImuGpsFusion()
{
    stopFusionThread();
}

void ImuGpsFusion::startFusionThread()
{
    if (fusion_thread_running_.exchange(true)) {
        return;
    }
    fusion_thread_ = std::thread(&ImuGpsFusion::fusionLoop, this);
}

void ImuGpsFusion::stopFusionThread()
{
    if (!fusion_thread_running_.exchange(false)) {
        return;
    }
    data_cv_.notify_all();
    if (fusion_thread_.joinable()) {
        fusion_thread_.join();
    }
}

void ImuGpsFusion::fusionLoop()
{
    while (fusion_thread_running_.load() && ros::ok()) {
        sensor_msgs::Imu::ConstPtr imu_msg;
        geometry_msgs::PoseStamped::ConstPtr gps_msg;
        bool do_reset = false;

        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            data_cv_.wait(lock, [this]() {
                return !fusion_thread_running_.load() || reset_requested_ || has_pending_imu_ || has_pending_gps_;
            });

            if (!fusion_thread_running_.load()) {
                break;
            }

            do_reset = reset_requested_;
            reset_requested_ = false;

            if (has_pending_imu_) {
                imu_msg = latest_imu_msg_;
                has_pending_imu_ = false;
            }
            if (has_pending_gps_) {
                gps_msg = latest_gps_msg_;
                has_pending_gps_ = false;
            }
        }

        if (do_reset) {
            reset_filter_state();
        }
        if (imu_msg) {
            process_imu_msg(imu_msg);
        }
        if (gps_msg) {
            process_gps_msg(gps_msg);
        }
    }
}

void ImuGpsFusion::publish_fused_tf(const ros::Time& stamp)
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = eskf_->state_position_[0];
    tf_msg.transform.translation.y = eskf_->state_position_[1];
    tf_msg.transform.translation.z = eskf_->state_position_[2];
    tf_msg.transform.rotation.w = eskf_->state_orientation_.w();
    tf_msg.transform.rotation.x = eskf_->state_orientation_.x();
    tf_msg.transform.rotation.y = eskf_->state_orientation_.y();
    tf_msg.transform.rotation.z = eskf_->state_orientation_.z();
    tf_broadcaster_->sendTransform(tf_msg);
}

void ImuGpsFusion::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = quatToEuler(q); // 注意这里的顺序是z-y-x，对应航向角-俯仰角-滚转角
    ROS_INFO_THROTTLE(1.0, "Get pose data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // 初始化ESKF位姿
    // if (!is_init_pos.load()) {
    //     std::lock_guard<std::mutex> lock(estimator_mutex_);
    //     if (!is_init_pos.load()) {
    //         eskf_->init(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z), q, msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    //         is_init_pos.store(true);
    //     }
    // }

    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = "map";
    pose.pose = msg->pose;
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        true_path.poses.push_back(pose);
        true_path.header.stamp = msg->header.stamp;
        true_path_pub.publish(true_path);
    }
}

void ImuGpsFusion::initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO("Received initial pose. time: %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
    //     msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // 初始化ESKF位姿
    if (!is_init_pos.load()) {
        std::lock_guard<std::mutex> lock(estimator_mutex_);
        if (!is_init_pos.load()) {
            Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
            eskf_->init(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z + 0.306881), q, msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
            is_init_pos.store(true);
        }
    }
}

void ImuGpsFusion::gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_gps_msg_ = msg;
        has_pending_gps_ = true;
    }
    data_cv_.notify_one();
}

void ImuGpsFusion::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_imu_msg_ = msg;
        has_pending_imu_ = true;
    }
    data_cv_.notify_one();
}

void ImuGpsFusion::reset_cb(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_INFO("Received reset command, fusion thread will reset ESKF state.\n");
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        reset_requested_ = true;
    }
    data_cv_.notify_one();
}

void ImuGpsFusion::process_imu_msg(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (!is_init_pos.load()) {
        return;
    }

    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d eulerAngle = quatToEuler(q); // 注意这里的顺序是z-y-x，对应航向角-俯仰角-滚转角
    ROS_INFO_THROTTLE(1.0, "Get imu data. time: %f, eulerangle: %f, %f, %f, angular_velocity: %f, %f, %f, linear_acceleration: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    nav_msgs::Odometry drone_state;
    bool predicted = false;
    {
        std::lock_guard<std::mutex> lock(estimator_mutex_);
        if (!is_init_pos.load()) {
            return;
        }

        latest_imu_stamp_ = msg->header.stamp;

        Eigen::Vector3d imu_linear_acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        predicted = eskf_->predict(
            Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
            imu_linear_acceleration,
            msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9);

        if (predicted) {
            drone_state.header = msg->header;
            drone_state.header.frame_id = "map";
            drone_state.child_frame_id = "base_link";
            drone_state.pose.pose.position.x = eskf_->state_position_[0];
            drone_state.pose.pose.position.y = eskf_->state_position_[1];
            drone_state.pose.pose.position.z = eskf_->state_position_[2];
            drone_state.pose.pose.orientation.w = eskf_->state_orientation_.w();
            drone_state.pose.pose.orientation.x = eskf_->state_orientation_.x();
            drone_state.pose.pose.orientation.y = eskf_->state_orientation_.y();
            drone_state.pose.pose.orientation.z = eskf_->state_orientation_.z();
            drone_state.twist.twist.linear.x = eskf_->state_velocity_[0];
            drone_state.twist.twist.linear.y = eskf_->state_velocity_[1];
            drone_state.twist.twist.linear.z = eskf_->state_velocity_[2];
            drone_state.twist.twist.angular.x = eskf_->state_gyro_velocity_[0];
            drone_state.twist.twist.angular.y = eskf_->state_gyro_velocity_[1];
            drone_state.twist.twist.angular.z = eskf_->state_gyro_velocity_[2];

            // 输出预测结果
            ROS_INFO_THROTTLE(1.0, "Predicted position: %f, %f, %f", eskf_->state_position_[0], eskf_->state_position_[1], eskf_->state_position_[2]);
            ROS_INFO_THROTTLE(1.0, "Predicted velocity: %f, %f, %f", eskf_->state_velocity_[0], eskf_->state_velocity_[1], eskf_->state_velocity_[2]);
            Eigen::Vector3d eulerAngle = quatToEuler(eskf_->state_orientation_); // 注意这里的顺序是z-y-x，对应航向角-俯仰角-滚转角
            ROS_INFO_THROTTLE(1.0, "Predicted angle: roll: %f deg, pitch: %f deg, yaw: %f deg", eulerAngle[2] * 57.3, eulerAngle[1] * 57.3, eulerAngle[0] * 57.3);
            ROS_INFO_THROTTLE(1.0, "Predicted gyro bias: %f, %f, %f", eskf_->state_gyro_bias_[0], eskf_->state_gyro_bias_[1], eskf_->state_gyro_bias_[2]);
            ROS_INFO_THROTTLE(1.0, "Predicted accel bias: %f, %f, %f", eskf_->state_accel_bias_[0], eskf_->state_accel_bias_[1], eskf_->state_accel_bias_[2]);
        }
    }

    is_imu_predicted.store(predicted);
    if (!predicted) {
        return;
    }

    drone_state_pub.publish(drone_state);

    publish_fused_tf(msg->header.stamp);
    publish_odom_up(msg->header.stamp);

    static auto state_publish_rate_start = std::chrono::steady_clock::now();
    static int state_publish_count = 0;
    ++state_publish_count;
    const auto state_publish_now = std::chrono::steady_clock::now();
    const double state_publish_window_sec = std::chrono::duration<double>(state_publish_now - state_publish_rate_start).count();
    if (state_publish_window_sec >= 1.0) {
        ROS_INFO("Drone state publish rate: %.1f Hz over %.2f sec",
            state_publish_count / state_publish_window_sec, state_publish_window_sec);
        state_publish_count = 0;
        state_publish_rate_start = state_publish_now;
    }
}

void ImuGpsFusion::process_gps_msg(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d gps_position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    const double gps_time = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;
    Eigen::Vector3d eulerAngle = quatToEuler(q); // 注意这里的顺序是z-y-x，对应航向角-俯仰角-滚转角
    ROS_INFO_THROTTLE(1.0, "Get gps data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", gps_time,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    if (!is_init_pos.load()) {
        return;
    }
    if (!is_imu_predicted.load()) {
        ROS_WARN_THROTTLE(1.0, "IMU prediction not available yet, skipping GPS update.");
        return;
    }

    geometry_msgs::PoseStamped fused_pose;
    bool updated = false;
    bool auto_reset_done = false;
    {
        std::lock_guard<std::mutex> lock(estimator_mutex_);
        if (!is_init_pos.load()) {
            return;
        }

        Eigen::Vector3d aligned_gps_position = gps_position;
        constexpr double kMaxGpsDelayCompensationSec = 0.1;
        if (!latest_imu_stamp_.isZero() && !msg->header.stamp.isZero()) {
            const double gps_delay_sec = (latest_imu_stamp_ - msg->header.stamp).toSec();
            if (gps_delay_sec > kMaxGpsDelayCompensationSec) {
                ROS_WARN_THROTTLE(1.0,
                    "GPS measurement is too old for fusion: delay=%.3f sec, max=%.3f sec. Skipping GPS update.",
                    gps_delay_sec, kMaxGpsDelayCompensationSec);
                is_gps_updated.store(false);
                return;
            }
            if (gps_delay_sec > 0.0) {
                aligned_gps_position += eskf_->state_velocity_ * gps_delay_sec;
                ROS_INFO_THROTTLE(1.0,
                    "Align GPS to latest IMU stamp: delay=%.3f sec, raw=(%.3f, %.3f, %.3f), aligned=(%.3f, %.3f, %.3f)",
                    gps_delay_sec,
                    gps_position.x(), gps_position.y(), gps_position.z(),
                    aligned_gps_position.x(), aligned_gps_position.y(), aligned_gps_position.z());
            } else if (gps_delay_sec < -0.005) {
                ROS_WARN_THROTTLE(1.0,
                    "GPS stamp is newer than latest IMU stamp by %.3f sec. Using raw GPS measurement.",
                    -gps_delay_sec);
            }
        }

        updated = eskf_->update(aligned_gps_position, q);

        if (updated) {
            const ros::Time now = msg->header.stamp;
            if (!is_start_) {
                start_time_ = now;
                is_start_ = true;
                ROS_INFO("Received first GPS update, starting timer for auto-reset.\n");
            }

            // 开始时2s后自动重置一次滤波器状态，确保ESKF状态初始化在正确的起始位置附近，避免初始位置误差过大导致滤波器发散
            if (!is_start_reset_ && (now - start_time_).toSec() > 2.0) {
                eskf_ = std::make_unique<ESKF>();
                eskf_->setQR(GYRO_BIAS_NOISE, ACCEL_BIAS_NOISE, GPS_POSITION_NOISE, GPS_ORIENTATION_NOISE);
                eskf_->init(gps_position, q, gps_time);
                is_imu_predicted.store(false);
                is_gps_updated.store(false);
                is_start_reset_ = true;
                publish_start_state_ = true;
                auto_reset_done = true;
                ROS_INFO("Start auto-reset ESKF state after 2 seconds from first GPS update.\n");
            }

            fused_pose.header = msg->header;
            fused_pose.header.frame_id = "map";
            fused_pose.pose.position.x = eskf_->state_position_[0];
            fused_pose.pose.position.y = eskf_->state_position_[1];
            fused_pose.pose.position.z = eskf_->state_position_[2];
            fused_pose.pose.orientation.w = eskf_->state_orientation_.w();
            fused_pose.pose.orientation.x = eskf_->state_orientation_.x();
            fused_pose.pose.orientation.y = eskf_->state_orientation_.y();
            fused_pose.pose.orientation.z = eskf_->state_orientation_.z();

            //publish_fused_tf(msg->header.stamp);

            //publish_odom_up(msg->header.stamp);

            if (publish_start_state_) {
                std_msgs::Header start_state_msg;
                start_state_msg.stamp = latest_imu_stamp_.isZero() ? msg->header.stamp : latest_imu_stamp_;
                start_state_msg.frame_id = "imu_time";
                start_state_pub.publish(start_state_msg);
            }

        }
    }

    is_gps_updated.store(updated);
    if (!updated) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (auto_reset_done) {
            true_path.poses.clear();
            fused_path.poses.clear();
        }
        fused_path.poses.push_back(fused_pose);
        fused_path.header.stamp = msg->header.stamp;
        fused_path_pub.publish(fused_path);
    }
}

void ImuGpsFusion::reset_filter_state()
{
    ROS_INFO("Fusion thread is resetting ESKF state.\n");

    {
        std::lock_guard<std::mutex> lock(estimator_mutex_);
        is_init_pos.store(false);
        is_imu_predicted.store(false);
        is_gps_updated.store(false);
        eskf_ = std::make_unique<ESKF>();
        eskf_->setQR(GYRO_BIAS_NOISE, ACCEL_BIAS_NOISE, GPS_POSITION_NOISE, GPS_ORIENTATION_NOISE);
    }

    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        true_path.poses.clear();
        fused_path.poses.clear();
        true_path_pub.publish(true_path);
        fused_path_pub.publish(fused_path);
    }
}

void ImuGpsFusion::publish_odom_up(const ros::Time& stamp)
{
    // 坐标变换：ESKF状态是NED坐标系下的，需要转换到RViz的z-up坐标系
    // 从map->map_rviz
    nav_msgs::Odometry odom_up_msg;
    try
    {
        geometry_msgs::TransformStamped transformStamped = tf_buffer_->lookupTransform(target_frame_, source_frame_, ros::Time(0));
        tf2::Transform tf_transform;
        tf2::fromMsg(transformStamped.transform, tf_transform);
        tf2::Transform eskf_state;
        eskf_state.setOrigin(tf2::Vector3(eskf_->state_position_[0], eskf_->state_position_[1], eskf_->state_position_[2]));
        eskf_state.setRotation(tf2::Quaternion(eskf_->state_orientation_.x(), eskf_->state_orientation_.y(), eskf_->state_orientation_.z(), eskf_->state_orientation_.w()));
        tf2::Transform transformed_state = tf_transform * eskf_state;

        odom_up_msg.header.stamp = stamp;
        odom_up_msg.header.frame_id = target_frame_;
        odom_up_msg.child_frame_id = "base_link";
        odom_up_msg.pose.pose.position.x = transformed_state.getOrigin().x();
        odom_up_msg.pose.pose.position.y = transformed_state.getOrigin().y();
        odom_up_msg.pose.pose.position.z = transformed_state.getOrigin().z();
        odom_up_msg.pose.pose.orientation.w = transformed_state.getRotation().w();
        odom_up_msg.pose.pose.orientation.x = transformed_state.getRotation().x();
        odom_up_msg.pose.pose.orientation.y = transformed_state.getRotation().y();
        odom_up_msg.pose.pose.orientation.z = transformed_state.getRotation().z();
        odom_up_msg.twist.twist.linear.x = eskf_->state_velocity_[0];
        odom_up_msg.twist.twist.linear.y = -eskf_->state_velocity_[1];
        odom_up_msg.twist.twist.linear.z = -eskf_->state_velocity_[2];
        odom_up_msg.twist.twist.angular.x = eskf_->state_gyro_velocity_[0];
        odom_up_msg.twist.twist.angular.y = -eskf_->state_gyro_velocity_[1];
        odom_up_msg.twist.twist.angular.z = -eskf_->state_gyro_velocity_[2];
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "Could not transform ESKF state to %s frame: %s", target_frame_.c_str(), ex.what());
        return;
    }

    odom_up_pub.publish(odom_up_msg);
}

#endif
