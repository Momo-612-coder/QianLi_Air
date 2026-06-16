#ifndef _MY_ORB_SLAM_HPP_
#define _MY_ORB_SLAM_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/GPSYaw.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include  "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifndef ORB_SLAM3_AVAILABLE
#define ORB_SLAM3_AVAILABLE 0
#endif

#if ORB_SLAM3_AVAILABLE
#include "System.h"
#endif

struct DroneData // 无人机目前相对于起飞点的位姿数据
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class OrbSlam
{
private:
    cv_bridge::CvImageConstPtr cv_bottom_ptr, cv_front_left_ptr, cv_front_right_ptr;
    cv::Mat front_left_img, front_right_img, bottom_img;
    cv::Mat front_left_img_undistort, front_right_img_undistort;
    cv::Mat front_left_img_orb, front_right_img_orb;
    cv::Mat prev_left_img_orb_;
    int img_width;
    int img_height;
    bool has_previous_frame_;
    double stereo_baseline_;

    std::unique_ptr<image_transport::ImageTransport> it;
    ros::CallbackQueue go_queue;
    ros::CallbackQueue front_img_queue;

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    ros::Subscriber imu_suber;//imu数据
    ros::Subscriber lidar_suber;//lidar数据
    image_transport::Subscriber front_left_view_suber;
    image_transport::Subscriber front_right_view_suber;
    image_transport::Publisher front_left_view_puber;
    image_transport::Publisher front_right_view_puber;
    ros::Publisher pose_puber;
    ros::Publisher path_puber;
    ros::Publisher odom_puber;
    ros::Publisher local_map_puber;
    ros::Publisher obstacle_dynamic_puber;
    ros::Publisher avoidance_cmd_puber;

    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void front_left_view_cb(const sensor_msgs::ImageConstPtr& msg);
    void front_right_view_cb(const sensor_msgs::ImageConstPtr& msg);
    void try_process_stereo_pair();
    void stereo_correction();
    void orb_process_fallback(const cv::Mat& left_rect, const cv::Mat& right_rect, const ros::Time& stamp);
    void publish_pose(const ros::Time& stamp);
    void publish_avoidance_cmd();
    void publish_local_map(const ros::Time& stamp);
    void append_lidar_points_to_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pts, const ros::Time& stamp);
    bool compute_left_right_clearance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pts, double& min_left, double& min_right) const;

    bool use_orbslam3_;
    bool use_stereo_inertial_;
    bool enable_dynamic_avoidance_;
    bool enable_autonomy_cmd_;
    bool enable_rectification_;
    std::string vocabulary_path_;
    std::string settings_path_;

    double image_sync_tolerance_sec_;
    double obstacle_stop_distance_;
    double obstacle_slow_distance_;
    double obstacle_dynamic_speed_thresh_;
    double obstacle_front_fov_deg_;
    double obstacle_max_distance_;
    double obstacle_min_z_;
    double obstacle_max_z_;
    double cruise_speed_;
    double avoid_lateral_speed_;
    double avoid_yaw_rate_;
    bool enable_vo_static_lock_;
    int vo_min_inliers_;
    double vo_static_trans_thresh_m_;
    double vo_static_rot_thresh_deg_;
    double vo_static_max_pixel_shift_;
    double vo_static_min_inlier_ratio_;
    size_t max_map_points_;

    sensor_msgs::Imu imu_data_;
    DroneData drone_data_;
    nav_msgs::Path path_msg_;
    ros::Time current_frame_stamp_;
    ros::Time current_left_stamp_;
    ros::Time current_right_stamp_;
    ros::Time last_imu_stamp_;
    ros::Time last_lidar_stamp_;
    ros::Time last_pose_publish_stamp_;

    std::mutex data_mutex_;
    std::mutex imu_mutex_;
    std::deque<sensor_msgs::Imu> imu_buffer_;

    struct ObstacleState
    {
        bool has_obstacle = false;
        bool is_dynamic = false;
        double min_front_distance = std::numeric_limits<double>::infinity();
        double min_left_distance = std::numeric_limits<double>::infinity();
        double min_right_distance = std::numeric_limits<double>::infinity();
        ros::Time stamp = ros::Time(0);
    };
    ObstacleState obstacle_state_;
    ObstacleState prev_obstacle_state_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_cloud_;

    // 双目相机内参和外参
    //960*480-60fov
    cv::Mat camera_intrinsic_l_cv_;
    cv::Mat camera_intrinsic_l_3x3_cv_;
    cv::Mat camera_intrinsic_r_cv_;
    cv::Mat camera_intrinsic_r_3x3_cv_;
    cv::Mat camera_Tlr_cv_;
    cv::Mat camera_Tlr_3x3_cv_;
    cv::Mat camera_Trl_cv_;
    cv::Mat camera_Trl_3x3_cv_;
    cv::Mat camera_Distor_l_cv_;
    cv::Mat camera_Distor_r_cv_;
    cv::Mat camera_to_NED_cv_;
    cv::Mat rectified_intrinsic_l_3x3_cv_;
    cv::Mat rectified_intrinsic_r_3x3_cv_;
    cv::Mat map1_l, map2_l, map1_r, map2_r;
    cv::Mat pose_w_c_;
    std::vector<cv::KeyPoint> prev_keypoints_left_;
    cv::Mat prev_descriptors_left_;
    std::vector<cv::Point3f> prev_points_3d_;

#if ORB_SLAM3_AVAILABLE
    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    bool process_with_orbslam3(const cv::Mat& left_rect, const cv::Mat& right_rect, const ros::Time& stamp);
    std::vector<ORB_SLAM3::IMU::Point> collect_imu_measurements(double ts_sec);
#endif

public:
    OrbSlam(ros::NodeHandle *nh);
    ~OrbSlam();

};


#endif





