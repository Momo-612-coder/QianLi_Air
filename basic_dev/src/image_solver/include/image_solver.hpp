#ifndef _IMAGE_SOLVER_HPP_
#define _IMAGE_SOLVER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Vector3Stamped.h"
#include  "sensor_msgs/Imu.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float64.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

#include "cuda_detector.hpp"
#include "path_planner.hpp"
#endif

class ImageSolver
{
private:
    cv_bridge::CvImageConstPtr cv_bottom_ptr, cv_front_left_ptr, cv_front_right_ptr;
    cv::Mat front_left_img, front_right_img, bottom_img;

    std::unique_ptr<image_transport::ImageTransport> it;
    ros::CallbackQueue go_queue;
    ros::CallbackQueue front_img_queue;

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    image_transport::Subscriber front_left_view_suber;
    image_transport::Subscriber front_right_view_suber;
    ros::Subscriber odom_suber;

    image_transport::Publisher front_left_pub;
    image_transport::Publisher front_right_pub;
    ros::Publisher score_gate_marker_pub_;
    ros::Publisher target_point_pub_;
    ros::Publisher target_yaw_pub_; // 发布目标偏航角（门法线方向）

    void front_left_view_cb(const sensor_msgs::ImageConstPtr& msg);
    void front_right_view_cb(const sensor_msgs::ImageConstPtr& msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void init_cuda_detector();
    void init_camera_params();
    ScoreGate find_score_gate_center(const cv::Mat& img);
    void stereo_correction();
    bool get_score_gate_depth();
    bool get_score_gate_map_point(const std_msgs::Header& img_header);
    void publish_score_gate_marker(const ros::Time& stamp);

    // 时间戳
    double front_left_img_timestamp;

    int img_width_;
    int img_height_;
    double stereo_baseline_;

    double gate_half_width_m_ = 1.45; 
    double gate_half_height_m_ = 1.6;


    ScoreGate target_score_gate_;
    std::vector<ScoreGate> right_score_gates_;

    // 双目相机内参和外参
    //960*720-60fov
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

    std::string target_frame_ = "map";
    std::string front_left_frame_override_ = "front_left_camera_optical_link";
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    CudaDetector left_cuda_detector_;
    CudaDetector right_cuda_detector_;
    PathPlanner path_planner_;

public:
    ImageSolver(ros::NodeHandle *nh);
    ~ImageSolver();

};





