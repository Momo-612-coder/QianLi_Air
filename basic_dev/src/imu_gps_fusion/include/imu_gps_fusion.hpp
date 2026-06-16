#ifndef _IMU_GPS_FUSION_HPP_
#define _IMU_GPS_FUSION_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include  "sensor_msgs/Imu.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>

#include "eskf.hpp"

const double GYRO_BIAS_NOISE = 0.00143; // 陀螺仪偏置噪声标准差
const double ACCEL_BIAS_NOISE = 0.0386; // 加速度计偏置噪声标准差
const double GPS_POSITION_NOISE = 1.0; // GPS位置测量噪声
const double GPS_ORIENTATION_NOISE = 1.0; // GPS姿态测量噪声

class ImuGpsFusion
{
private:
    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    ros::Subscriber odom_suber;//状态真值
    ros::Subscriber initial_pose_suber;//起始姿态
    ros::Subscriber gps_suber;//gps数据
    ros::Subscriber imu_suber;//imu数据
    ros::Subscriber reset_suber;//重置命令订阅

    ros::Publisher true_path_pub;
    ros::Publisher fused_path_pub;
    ros::Publisher drone_state_pub;
    ros::Publisher odom_up_pub;
    nav_msgs::Path true_path;
    nav_msgs::Path fused_path;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    ros::Publisher start_state_pub;

    // tf转换
    std::string target_frame_ = "map_rviz";
    std::string source_frame_ = "map";
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void reset_cb(const std_msgs::Empty::ConstPtr& msg);
    void publish_fused_tf(const ros::Time& stamp);
    void startFusionThread();
    void stopFusionThread();
    void fusionLoop();
    void process_imu_msg(const sensor_msgs::Imu::ConstPtr& msg);
    void process_gps_msg(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void reset_filter_state();
    void publish_map_rviz_tf();
    void publish_odom_up(const ros::Time& stamp);

    std::atomic<bool> is_imu_predicted{false};
    std::atomic<bool> is_init_pos{false};
    std::atomic<bool> is_gps_updated{false};

    std::thread fusion_thread_;
    std::atomic<bool> fusion_thread_running_{false};
    std::mutex data_mutex_;
    std::condition_variable data_cv_;
    sensor_msgs::Imu::ConstPtr latest_imu_msg_;
    geometry_msgs::PoseStamped::ConstPtr latest_gps_msg_;
    bool has_pending_imu_ = false;
    bool has_pending_gps_ = false;
    bool reset_requested_ = false;

    bool publish_start_state_ = false;
    bool is_start_ = false; // 开始时2s后自动重置一次滤波器状态，确保ESKF状态初始化在正确的起始位置附近，避免初始位置误差过大导致滤波器发散
    bool is_start_reset_ = false; // 开始时2s后自动重置一次滤波器状态，确保ESKF状态初始化在正确的起始位置附近，避免初始位置误差过大导致滤波器发散
    ros::Time start_time_;
    ros::Time latest_imu_stamp_;

    std::mutex estimator_mutex_;
    std::mutex path_mutex_;

    std::unique_ptr<ESKF> eskf_; // 使用智能指针管理ESKF对象的生命周期
public:
    ImuGpsFusion(ros::NodeHandle *nh);
    ~ImuGpsFusion();

};

#endif


