#ifndef _MY_ORB_SLAM_CPP_
#define _MY_ORB_SLAM_CPP_

#include "my_orb_slam.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "my_orb_slam"); // 初始化ros 节点，命名为 my_orb_slam
    ros::NodeHandle n; // 创建node控制句柄
    OrbSlam go(&n);
    return 0;
}

OrbSlam::OrbSlam(ros::NodeHandle *nh)
{  
    nh->param<bool>("my_orb_slam/use_orbslam3", use_orbslam3_, false);
    nh->param<bool>("my_orb_slam/use_stereo_inertial", use_stereo_inertial_, false);
    nh->param<bool>("my_orb_slam/enable_dynamic_avoidance", enable_dynamic_avoidance_, true);
    nh->param<bool>("my_orb_slam/enable_autonomy_cmd", enable_autonomy_cmd_, true);
    nh->param<bool>("my_orb_slam/enable_rectification", enable_rectification_, true);
    nh->param<std::string>("my_orb_slam/vocabulary_path", vocabulary_path_, std::string(""));
    nh->param<std::string>("my_orb_slam/settings_path", settings_path_, std::string(""));

    nh->param<double>("my_orb_slam/image_sync_tolerance_sec", image_sync_tolerance_sec_, 0.015);
    nh->param<double>("my_orb_slam/obstacle_stop_distance", obstacle_stop_distance_, 1.5);
    nh->param<double>("my_orb_slam/obstacle_slow_distance", obstacle_slow_distance_, 3.0);
    nh->param<double>("my_orb_slam/obstacle_dynamic_speed_thresh", obstacle_dynamic_speed_thresh_, 0.7);
    nh->param<double>("my_orb_slam/obstacle_front_fov_deg", obstacle_front_fov_deg_, 40.0);
    nh->param<double>("my_orb_slam/obstacle_max_distance", obstacle_max_distance_, 12.0);
    nh->param<double>("my_orb_slam/obstacle_min_z", obstacle_min_z_, -1.8);
    nh->param<double>("my_orb_slam/obstacle_max_z", obstacle_max_z_, 1.2);
    nh->param<double>("my_orb_slam/cruise_speed", cruise_speed_, 1.5);
    nh->param<double>("my_orb_slam/avoid_lateral_speed", avoid_lateral_speed_, 0.8);
    nh->param<double>("my_orb_slam/avoid_yaw_rate", avoid_yaw_rate_, 0.45);
    nh->param<bool>("my_orb_slam/enable_vo_static_lock", enable_vo_static_lock_, true);
    nh->param<int>("my_orb_slam/vo_min_inliers", vo_min_inliers_, 80);
    nh->param<double>("my_orb_slam/vo_static_trans_thresh_m", vo_static_trans_thresh_m_, 0.003);
    nh->param<double>("my_orb_slam/vo_static_rot_thresh_deg", vo_static_rot_thresh_deg_, 0.12);
    nh->param<double>("my_orb_slam/vo_static_max_pixel_shift", vo_static_max_pixel_shift_, 0.75);
    nh->param<double>("my_orb_slam/vo_static_min_inlier_ratio", vo_static_min_inlier_ratio_, 0.6);
    int max_map_points_param = 50000;
    nh->param<int>("my_orb_slam/max_map_points", max_map_points_param, 50000);
    max_map_points_ = static_cast<size_t>(std::max(1000, max_map_points_param));

    //初始化双目相机内参和外参
    camera_intrinsic_l_cv_ = (cv::Mat_<double>(4, 4)<<
        833.8, 0.0, 481.6, 0.0,
        0.0, 835.27, 360.766, 0.0, 
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
        );
    camera_intrinsic_l_3x3_cv_ = (cv::Mat_<double>(3, 3)<<
        833.8, 0.0, 481.6, 
        0.0, 835.27, 360.766, 
        0.0, 0.0, 1.0
        );
    camera_intrinsic_r_cv_ = (cv::Mat_<double>(4, 4)<<
        832.287, 0.0, 481.66, 0.0,
        0.0, 833.94, 359.45, 0.0, 
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
        );
    camera_intrinsic_r_3x3_cv_ = (cv::Mat_<double>(3, 3)<<
        832.287, 0.0, 481.66, 
        0.0, 833.94, 359.45, 
        0.0, 0.0, 1.0
        );
    camera_Tlr_cv_ = (cv::Mat_<double>(3, 4)<<
        1.0, 0.0,0.0, 0.301922,
        0.0, 1.0, 0.0, -0.00150702, 
        0.0, 0.0, 1.0, 0.00211
        );
    camera_Tlr_3x3_cv_ = (cv::Mat_<double>(3, 3)<<
        1.0, 0.0,0.0,   
        0.0, 1.0, 0.0, 
        0.0, 0.0, 1.0
        );
    camera_Trl_cv_ = (cv::Mat_<double>(3, 4)<<
        1.0, 0.0,0.0, -0.301922,
        0.0, 1.0, 0.0, 0.00150702, 
        0.0, 0.0, 1.0, -0.00211
        );
    camera_Trl_3x3_cv_ = (cv::Mat_<double>(3, 3)<<
        1.0, 0.0,0.0,   
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
        );
    camera_Distor_l_cv_ = (cv::Mat_<double>(4, 1)<<
        -0.00170596, -0.00124899, 0.00007559, -0.00017225
        );
    camera_Distor_r_cv_ = (cv::Mat_<double>(4, 1)<<
        -0.00567839, 0.01405329, 0.00025239, -0.00037066
        );
    camera_to_NED_cv_ = (cv::Mat_<double>(4, 4)<<
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0
        );

    //创建图像传输控制句柄
    //960*480-60fov (cx=481.6 => width=960)
    it = std::make_unique<image_transport::ImageTransport>(*nh); 
    front_left_img = cv::Mat(480, 960, CV_8UC3, cv::Scalar(0));
    front_right_img = cv::Mat(480, 960, CV_8UC3, cv::Scalar(0));
    img_width = 960;
    img_height = 480;
    has_previous_frame_ = false;
    stereo_baseline_ = std::abs(camera_Tlr_cv_.at<double>(0, 3));
    pose_w_c_ = cv::Mat::eye(4, 4, CV_64F);
    local_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    local_map_cloud_->points.reserve(max_map_points_);

    // 计算映射表，用于图像矫正
    if(enable_rectification_)
    {
        stereo_correction();
    }

#if ORB_SLAM3_AVAILABLE
    if(use_orbslam3_)
    {
        const bool missing_config = vocabulary_path_.empty() || settings_path_.empty();
        if(missing_config)
        {
            ROS_WARN("ORB-SLAM3 is enabled, but vocabulary_path/settings_path is empty. Fallback VO will be used.");
            use_orbslam3_ = false;
        }
        else
        {
            std::ifstream vocab_fs(vocabulary_path_);
            std::ifstream settings_fs(settings_path_);
            if(!vocab_fs.good() || !settings_fs.good())
            {
                ROS_WARN("ORB-SLAM3 config file not found. fallback VO is used. vocab=%s, settings=%s",
                         vocabulary_path_.c_str(), settings_path_.c_str());
                use_orbslam3_ = false;
            }
            else
            {
                const ORB_SLAM3::System::eSensor sensor_mode = use_stereo_inertial_ ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO;
                slam_system_ = std::make_unique<ORB_SLAM3::System>(vocabulary_path_, settings_path_, sensor_mode, false);
                ROS_INFO("ORB-SLAM3 initialized in %s mode.", use_stereo_inertial_ ? "IMU_STEREO" : "STEREO");
            }
        }
    }
#else
    if(use_orbslam3_)
    {
        ROS_WARN("use_orbslam3=true, but current build has no ORB-SLAM3 linked. Fallback VO will be used.");
        use_orbslam3_ = false;
    }
#endif

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    imu_suber = nh->subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, std::bind(&OrbSlam::imu_cb, this, std::placeholders::_1));//imu数据
    lidar_suber = nh->subscribe<sensor_msgs::PointCloud2>("airsim_node/drone_1/lidar", 1, std::bind(&OrbSlam::lidar_cb, this, std::placeholders::_1));//imu数据
    front_left_view_suber = it->subscribe("airsim_node/drone_1/front_left/Scene", 1, std::bind(&OrbSlam::front_left_view_cb, this,  std::placeholders::_1));
    front_right_view_suber = it->subscribe("airsim_node/drone_1/front_right/Scene", 1, std::bind(&OrbSlam::front_right_view_cb, this,  std::placeholders::_1));
    front_left_view_puber = it->advertise("my_orb_slam/front_left/image_raw", 1);
    front_right_view_puber = it->advertise("my_orb_slam/front_right/image_raw", 1);
    pose_puber = nh->advertise<geometry_msgs::PoseStamped>("my_orb_slam/pose", 1);
    path_puber = nh->advertise<nav_msgs::Path>("my_orb_slam/path", 1, true);
    odom_puber = nh->advertise<nav_msgs::Odometry>("my_orb_slam/odometry", 1);
    local_map_puber = nh->advertise<sensor_msgs::PointCloud2>("my_orb_slam/local_map", 1);
    obstacle_dynamic_puber = nh->advertise<std_msgs::Bool>("my_orb_slam/dynamic_obstacle", 1);
    avoidance_cmd_puber = nh->advertise<airsim_ros::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 1);
    path_msg_.header.frame_id = "map";
    current_frame_stamp_ = ros::Time(0);
    current_left_stamp_ = ros::Time(0);
    current_right_stamp_ = ros::Time(0);
    last_imu_stamp_ = ros::Time(0);
    last_lidar_stamp_ = ros::Time(0);
    last_pose_publish_stamp_ = ros::Time(0);
    
    ros::spin();
}

OrbSlam::~OrbSlam()
{
#if ORB_SLAM3_AVAILABLE
    if(slam_system_)
    {
        slam_system_->Shutdown();
    }
#endif
}

void OrbSlam::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_data_ = *msg;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_buffer_.push_back(*msg);
        while(imu_buffer_.size() > 2000)
        {
            imu_buffer_.pop_front();
        }
    }
    last_imu_stamp_ = msg->header.stamp;

    // 读取无人机当前位姿数据, 保存到drone_data_中
    tf2::Quaternion quat;
    tf2::fromMsg(imu_data_.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    drone_data_.roll = roll;
    drone_data_.pitch = pitch;
    drone_data_.yaw = yaw;
    ROS_INFO_THROTTLE(1.0, "IMU rpy: %.3f %.3f %.3f", drone_data_.roll, drone_data_.pitch, drone_data_.yaw);
}

void OrbSlam::front_left_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_left_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(cv_front_left_ptr->image.empty())
    {
        return;
    }

    // 将ptr转化为cv::Mat格式的图像，并保存到front_left_img中
    front_left_img = cv_front_left_ptr->image;
    current_left_stamp_ = msg->header.stamp;
    if(!map1_l.empty() && !map2_l.empty())
    {
        cv::remap(front_left_img, front_left_img_undistort, map1_l, map2_l, cv::INTER_LINEAR);
        auto msg_undistort = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, front_left_img_undistort).toImageMsg();
        front_left_view_puber.publish(msg_undistort);
        return;
    }

    front_left_view_puber.publish(msg);
}

void OrbSlam::front_right_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    current_frame_stamp_ = msg->header.stamp;
    current_right_stamp_ = msg->header.stamp;
    cv_front_right_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(cv_front_right_ptr->image.empty())
    {
        return;
    }

    // 将ptr转化为cv::Mat格式的图像，并保存到front_right_img中
    front_right_img = cv_front_right_ptr->image;
    if(!map1_r.empty() && !map2_r.empty())
    {
        cv::remap(front_right_img, front_right_img_undistort, map1_r, map2_r, cv::INTER_LINEAR);
        auto msg_undistort = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, front_right_img_undistort).toImageMsg();
        front_right_view_puber.publish(msg_undistort);
        try_process_stereo_pair();
        return;
    }

    front_right_view_puber.publish(msg);
    try_process_stereo_pair();
}

void OrbSlam::try_process_stereo_pair()
{
    if(front_left_img.empty() || front_right_img.empty())
    {
        return;
    }

    if(current_left_stamp_.isZero() || current_right_stamp_.isZero())
    {
        return;
    }

    const double dt = std::abs((current_left_stamp_ - current_right_stamp_).toSec());
    if(dt > image_sync_tolerance_sec_)
    {
        ROS_WARN_THROTTLE(1.0, "Stereo pair out of sync: dt=%.4f sec", dt);
        return;
    }

    const cv::Mat left_rect = front_left_img_undistort.empty() ? front_left_img : front_left_img_undistort;
    const cv::Mat right_rect = front_right_img_undistort.empty() ? front_right_img : front_right_img_undistort;
    const ros::Time stamp = current_right_stamp_;
    if(left_rect.empty() || right_rect.empty())
    {
        return;
    }

    bool processed = false;
#if ORB_SLAM3_AVAILABLE
    if(use_orbslam3_)
    {
        processed = process_with_orbslam3(left_rect, right_rect, stamp);
    }
#endif
    if(!processed)
    {
        orb_process_fallback(left_rect, right_rect, stamp);
    }
}

void OrbSlam::stereo_correction()
{
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat R = (cv::Mat_<double>(3, 3)<<
        1.0, 0.0,0.0,   
        0.0, 1.0, 0.0, 
        0.0, 0.0, 1.0
        );
    cv::Mat T = (cv::Mat_<double>(3, 1)<<
        0.301922, -0.00150702, 0.00211
        );

    if(img_width <= 0 || img_height <= 0)
    {
        ROS_ERROR("Invalid image size for stereo correction: %d x %d", img_width, img_height);
        return;
    }

    cv::Size img_size(img_width, img_height);
    try
    {
        cv::stereoRectify(camera_intrinsic_l_3x3_cv_, camera_Distor_l_cv_, 
                          camera_intrinsic_r_3x3_cv_, camera_Distor_r_cv_, 
                          img_size, R, T,
                          R1, R2, P1, P2, Q);
    }
    catch (const cv::Exception& e)
    {
        ROS_ERROR("stereoRectify failed: %s", e.what());
        return;
    }

    cv::initUndistortRectifyMap(camera_intrinsic_l_3x3_cv_, camera_Distor_l_cv_, R1, P1, img_size, CV_32F, map1_l, map2_l);
    cv::initUndistortRectifyMap(camera_intrinsic_r_3x3_cv_, camera_Distor_r_cv_, R2, P2, img_size, CV_32F, map1_r, map2_r);
    rectified_intrinsic_l_3x3_cv_ = P1(cv::Rect(0, 0, 3, 3)).clone();
    rectified_intrinsic_r_3x3_cv_ = P2(cv::Rect(0, 0, 3, 3)).clone();
    stereo_baseline_ = std::abs(P2.at<double>(0, 3) / P2.at<double>(0, 0));
}

void OrbSlam::publish_pose(const ros::Time& stamp)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = pose_w_c_.at<double>(0, 3);
    pose_msg.pose.position.y = pose_w_c_.at<double>(1, 3);
    pose_msg.pose.position.z = pose_w_c_.at<double>(2, 3);

    tf2::Matrix3x3 rotation(
        pose_w_c_.at<double>(0, 0), pose_w_c_.at<double>(0, 1), pose_w_c_.at<double>(0, 2),
        pose_w_c_.at<double>(1, 0), pose_w_c_.at<double>(1, 1), pose_w_c_.at<double>(1, 2),
        pose_w_c_.at<double>(2, 0), pose_w_c_.at<double>(2, 1), pose_w_c_.at<double>(2, 2)
    );
    tf2::Quaternion quaternion;
    rotation.getRotation(quaternion);
    pose_msg.pose.orientation = tf2::toMsg(quaternion);

    pose_puber.publish(pose_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = "camera";
    odom_msg.pose.pose = pose_msg.pose;
    odom_puber.publish(odom_msg);

    path_msg_.header = pose_msg.header;
    path_msg_.poses.push_back(pose_msg);
    if(path_msg_.poses.size() > 2000)
    {
        path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_puber.publish(path_msg_);
    last_pose_publish_stamp_ = pose_msg.header.stamp;
}

void OrbSlam::publish_local_map(const ros::Time& stamp)
{
    if(!local_map_cloud_ || local_map_cloud_->empty())
    {
        return;
    }

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*local_map_cloud_, map_msg);
    map_msg.header.frame_id = "map";
    map_msg.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    local_map_puber.publish(map_msg);
}

void OrbSlam::append_lidar_points_to_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pts, const ros::Time& stamp)
{
    if(!pts || !local_map_cloud_)
    {
        return;
    }

    const cv::Mat rotation = pose_w_c_(cv::Rect(0, 0, 3, 3)).clone();
    const cv::Mat translation = pose_w_c_(cv::Rect(3, 0, 1, 3)).clone();

    for(const auto& point : pts->points)
    {
        if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            continue;
        }
        if(point.z < obstacle_min_z_ || point.z > obstacle_max_z_)
        {
            continue;
        }

        const double radius = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if(radius > obstacle_max_distance_)
        {
            continue;
        }

        cv::Mat p_c = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
        cv::Mat p_w = rotation * p_c + translation;
        pcl::PointXYZ world_pt;
        world_pt.x = static_cast<float>(p_w.at<double>(0));
        world_pt.y = static_cast<float>(p_w.at<double>(1));
        world_pt.z = static_cast<float>(p_w.at<double>(2));
        local_map_cloud_->points.push_back(world_pt);
    }

    if(local_map_cloud_->points.size() > max_map_points_)
    {
        const size_t remove_count = local_map_cloud_->points.size() - max_map_points_;
        local_map_cloud_->points.erase(local_map_cloud_->points.begin(), local_map_cloud_->points.begin() + static_cast<long>(remove_count));
    }

    local_map_cloud_->width = static_cast<uint32_t>(local_map_cloud_->points.size());
    local_map_cloud_->height = 1;
    local_map_cloud_->is_dense = false;
    publish_local_map(stamp);
}

bool OrbSlam::compute_left_right_clearance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pts, double& min_left, double& min_right) const
{
    min_left = std::numeric_limits<double>::infinity();
    min_right = std::numeric_limits<double>::infinity();
    if(!pts)
    {
        return false;
    }

    bool has_point = false;
    for(const auto& point : pts->points)
    {
        if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            continue;
        }
        if(point.z < obstacle_min_z_ || point.z > obstacle_max_z_ || point.x <= 0.1)
        {
            continue;
        }

        const double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if(distance > obstacle_max_distance_)
        {
            continue;
        }

        has_point = true;
        if(point.y >= 0.0)
        {
            min_left = std::min(min_left, distance);
        }
        else
        {
            min_right = std::min(min_right, distance);
        }
    }
    return has_point;
}

void OrbSlam::publish_avoidance_cmd()
{
    
}

#if ORB_SLAM3_AVAILABLE
std::vector<ORB_SLAM3::IMU::Point> OrbSlam::collect_imu_measurements(double ts_sec)
{
    std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
    std::lock_guard<std::mutex> lock(imu_mutex_);
    while(!imu_buffer_.empty() && imu_buffer_.front().header.stamp.toSec() <= ts_sec)
    {
        const sensor_msgs::Imu& imu = imu_buffer_.front();
        imu_measurements.emplace_back(
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
            imu.header.stamp.toSec());
        imu_buffer_.pop_front();
    }
    return imu_measurements;
}

bool OrbSlam::process_with_orbslam3(const cv::Mat& left_rect, const cv::Mat& right_rect, const ros::Time& stamp)
{
    if(!slam_system_)
    {
        return false;
    }

    cv::Mat left_gray;
    cv::Mat right_gray;
    if(left_rect.channels() == 3)
    {
        cv::cvtColor(left_rect, left_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        left_gray = left_rect;
    }
    if(right_rect.channels() == 3)
    {
        cv::cvtColor(right_rect, right_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        right_gray = right_rect;
    }

    const double ts_sec = stamp.toSec();
    Sophus::SE3f Tcw;
    if(use_stereo_inertial_)
    {
        std::vector<ORB_SLAM3::IMU::Point> imu_measurements = collect_imu_measurements(ts_sec);
        Tcw = slam_system_->TrackStereo(left_gray, right_gray, ts_sec, imu_measurements);
    }
    else
    {
        Tcw = slam_system_->TrackStereo(left_gray, right_gray, ts_sec);
    }

    const Eigen::Matrix4f Tcw_eigen = Tcw.matrix();
    if(Tcw_eigen.isZero(0.0f))
    {
        ROS_WARN_THROTTLE(1.0, "ORB-SLAM3 tracking lost on current frame.");
        return false;
    }

    const Eigen::Matrix4f Twc_eigen = Tcw_eigen.inverse();
    pose_w_c_ = cv::Mat::eye(4, 4, CV_64F);
    for(int row = 0; row < 4; ++row)
    {
        for(int col = 0; col < 4; ++col)
        {
            pose_w_c_.at<double>(row, col) = static_cast<double>(Twc_eigen(row, col));
        }
    }

    publish_pose(stamp);
    ROS_INFO_THROTTLE(1.0, "ORB-SLAM3 tracking ok. world_t=[%.3f %.3f %.3f]", 
                      pose_w_c_.at<double>(0, 3), pose_w_c_.at<double>(1, 3), pose_w_c_.at<double>(2, 3));
    return true;
}
#endif

// 对矫正后的图像进行ORB特征提取和匹配，计算相机位姿
// 特征提取：
//对左目图像提取ORB特征点（关键点+描述子）。ORB-SLAM3使用多尺度图像金字塔，在每一层图像上提取FAST角点，并计算ORB描述子。
//同时，根据左目图像的特征点位置，在右目图像上通过极线约束（因为已经立体校正，所以在同一水平线上）进行特征点匹配，得到右目图像中对应的特征点。这样，每一个特征点都有了在左目和右目图像中的位置，即双目匹配。
// # 特征提取过程
// 左图特征提取：
//   - 构建图像金字塔（8层尺度，1.2缩放因子）
//   - 每层计算FAST角点（根据图像大小自适应阈值）
//   - 使用灰度质心法计算方向（计算关键点角度）
//   - 生成256位BRIEF描述子
// 右图特征匹配：
//   - 基于极线约束在右图搜索匹配
//   - 使用SAD（绝对误差和）进行匹配代价计算
//   - 亚像素精度优化匹配位置
//   - 计算视差 → 深度信息
// 关键数据生成：
//     左图提取1000-2000个ORB特征点
//     右图匹配获得约70-80%的特征点对应关系
//     生成带深度信息的Frame对象
void OrbSlam::orb_process_fallback(const cv::Mat& left_rect, const cv::Mat& right_rect, const ros::Time& stamp)
{
    if(left_rect.empty() || right_rect.empty())
    {
        return;
    }

    front_left_img_orb = left_rect.clone();
    front_right_img_orb = right_rect.clone();

    cv::Mat left_gray;
    cv::Mat right_gray;
    cv::cvtColor(left_rect, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_rect, right_gray, cv::COLOR_BGR2GRAY);

    auto orb = cv::ORB::create(1500, 1.2f, 8);
    std::vector<cv::KeyPoint> keypoints_left;
    std::vector<cv::KeyPoint> keypoints_right;
    cv::Mat descriptors_left;
    cv::Mat descriptors_right;
    orb->detectAndCompute(left_gray, cv::Mat(), keypoints_left, descriptors_left);
    orb->detectAndCompute(right_gray, cv::Mat(), keypoints_right, descriptors_right);

    if(descriptors_left.empty() || descriptors_right.empty())
    {
        ROS_WARN_THROTTLE(2.0, "ORB descriptors are empty.");
        return;
    }

    cv::BFMatcher stereo_matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> stereo_knn_matches;
    stereo_matcher.knnMatch(descriptors_left, descriptors_right, stereo_knn_matches, 2);

    std::vector<cv::DMatch> stereo_matches;
    stereo_matches.reserve(stereo_knn_matches.size());
    for(const auto& pair : stereo_knn_matches)
    {
        if(pair.size() < 2)
        {
            continue;
        }
        if(pair[0].distance < 0.75f * pair[1].distance)
        {
            const cv::Point2f& pl = keypoints_left[pair[0].queryIdx].pt;
            const cv::Point2f& pr = keypoints_right[pair[0].trainIdx].pt;
            if(std::abs(pl.y - pr.y) < 2.0f)
            {
                stereo_matches.push_back(pair[0]);
            }
        }
    }

    if(stereo_matches.size() < 20)
    {
        ROS_WARN_THROTTLE(2.0, "Not enough stereo matches to build landmarks: %zu", stereo_matches.size());
        return;
    }

    const bool use_rectified_intrinsics = !rectified_intrinsic_l_3x3_cv_.empty();
    const cv::Mat active_camera_matrix = use_rectified_intrinsics ? rectified_intrinsic_l_3x3_cv_ : camera_intrinsic_l_3x3_cv_;
    const cv::Mat active_distortion = use_rectified_intrinsics ? cv::Mat::zeros(4, 1, CV_64F) : camera_Distor_l_cv_;
    const double fx = active_camera_matrix.at<double>(0, 0);
    const double fy = active_camera_matrix.at<double>(1, 1);
    const double cx = active_camera_matrix.at<double>(0, 2);
    const double cy = active_camera_matrix.at<double>(1, 2);

    std::vector<cv::Point3f> current_points_3d(keypoints_left.size(), cv::Point3f(0.0f, 0.0f, -1.0f));
    for(const auto& match : stereo_matches)
    {
        const cv::Point2f pl = keypoints_left[match.queryIdx].pt;
        const cv::Point2f pr = keypoints_right[match.trainIdx].pt;
        const double disparity = pl.x - pr.x;
        if(disparity <= 0.5)
        {
            continue;
        }

        const double depth = fx * stereo_baseline_ / disparity;
        if(depth < 0.1 || depth > 80.0)
        {
            continue;
        }

        const double x = (pl.x - cx) * depth / fx;
        const double y = (pl.y - cy) * depth / fy;
        current_points_3d[match.queryIdx] = cv::Point3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(depth));
    }

    auto store_current_frame = [&]() {
        prev_left_img_orb_ = left_rect.clone();
        prev_keypoints_left_ = keypoints_left;
        prev_descriptors_left_ = descriptors_left.clone();
        prev_points_3d_ = current_points_3d;
        has_previous_frame_ = true;
    };

    if(!has_previous_frame_ || prev_descriptors_left_.empty() || prev_points_3d_.empty())
    {
        store_current_frame();
        publish_pose(stamp);
        ROS_INFO_THROTTLE(2.0, "VO initialized with current stereo frame.");
        return;
    }

    cv::BFMatcher temporal_matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> temporal_knn_matches;
    temporal_matcher.knnMatch(prev_descriptors_left_, descriptors_left, temporal_knn_matches, 2);

    std::vector<cv::Point3f> points_3d;
    std::vector<cv::Point2f> points_2d;
    std::vector<cv::DMatch> temporal_matches;
    points_3d.reserve(temporal_knn_matches.size());
    points_2d.reserve(temporal_knn_matches.size());
    temporal_matches.reserve(temporal_knn_matches.size());

    for(const auto& pair : temporal_knn_matches)
    {
        if(pair.size() < 2)
        {
            continue;
        }
        if(pair[0].distance >= 0.75f * pair[1].distance)
        {
            continue;
        }

        const int prev_idx = pair[0].queryIdx;
        const cv::Point3f& point_3d = prev_points_3d_[prev_idx];
        if(point_3d.z <= 0.0f)
        {
            continue;
        }

        points_3d.push_back(point_3d);
        points_2d.push_back(keypoints_left[pair[0].trainIdx].pt);
        temporal_matches.push_back(pair[0]);
    }

    if(points_3d.size() < 15)
    {
        ROS_WARN_THROTTLE(2.0, "Not enough temporal 3D-2D correspondences for VO: %zu", points_3d.size());
        store_current_frame();
        return;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;
    bool ok = cv::solvePnPRansac(
        points_3d,
        points_2d,
        active_camera_matrix,
        active_distortion,
        rvec,
        tvec,
        false,
        100,
        3.0,
        0.99,
        inliers,
        cv::SOLVEPNP_ITERATIVE
    );

    if(!ok)
    {
        ROS_WARN_THROTTLE(2.0, "solvePnPRansac failed.");
        store_current_frame();
        return;
    }

    const int inlier_count = inliers.rows;
    cv::Mat rotation_prev_to_curr;
    cv::Rodrigues(rvec, rotation_prev_to_curr);
    cv::Mat transform_prev_to_curr = cv::Mat::eye(4, 4, CV_64F);
    rotation_prev_to_curr.copyTo(transform_prev_to_curr(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(transform_prev_to_curr(cv::Rect(3, 0, 1, 3)));

    cv::Mat transform_curr_to_prev = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat rotation_curr_to_prev = rotation_prev_to_curr.t();
    rotation_curr_to_prev.copyTo(transform_curr_to_prev(cv::Rect(0, 0, 3, 3)));
    cv::Mat translation_curr_to_prev = -rotation_curr_to_prev * tvec;
    translation_curr_to_prev.copyTo(transform_curr_to_prev(cv::Rect(3, 0, 1, 3)));
    const double inlier_ratio = temporal_matches.empty() ? 0.0 : static_cast<double>(inlier_count) / static_cast<double>(temporal_matches.size());
    const double rel_translation_norm = cv::norm(tvec);
    const double trace_r = rotation_prev_to_curr.at<double>(0, 0) + rotation_prev_to_curr.at<double>(1, 1) + rotation_prev_to_curr.at<double>(2, 2);
    const double cos_theta = std::max(-1.0, std::min(1.0, 0.5 * (trace_r - 1.0)));
    const double rel_rotation_deg = std::acos(cos_theta) * 180.0 / M_PI;

    std::vector<double> inlier_pixel_shifts;
    inlier_pixel_shifts.reserve(static_cast<size_t>(inlier_count));
    for(int row = 0; row < inliers.rows; ++row)
    {
        const int inlier_idx = inliers.at<int>(row, 0);
        if(inlier_idx < 0 || static_cast<size_t>(inlier_idx) >= temporal_matches.size())
        {
            continue;
        }
        const cv::DMatch& match = temporal_matches[static_cast<size_t>(inlier_idx)];
        const cv::Point2f& prev_pt = prev_keypoints_left_[match.queryIdx].pt;
        const cv::Point2f& curr_pt = keypoints_left[match.trainIdx].pt;
        inlier_pixel_shifts.push_back(cv::norm(curr_pt - prev_pt));
    }

    double median_pixel_shift = std::numeric_limits<double>::infinity();
    if(!inlier_pixel_shifts.empty())
    {
        const size_t mid = inlier_pixel_shifts.size() / 2;
        std::nth_element(inlier_pixel_shifts.begin(), inlier_pixel_shifts.begin() + static_cast<long>(mid), inlier_pixel_shifts.end());
        median_pixel_shift = inlier_pixel_shifts[mid];
    }

    const bool stationary = enable_vo_static_lock_ &&
                            inlier_count >= vo_min_inliers_ &&
                            inlier_ratio >= vo_static_min_inlier_ratio_ &&
                            rel_translation_norm <= vo_static_trans_thresh_m_ &&
                            rel_rotation_deg <= vo_static_rot_thresh_deg_ &&
                            std::isfinite(median_pixel_shift) &&
                            median_pixel_shift <= vo_static_max_pixel_shift_;

    if(!stationary)
    {
        pose_w_c_ = pose_w_c_ * transform_curr_to_prev;
    }
    publish_pose(stamp);

    ROS_INFO_THROTTLE(
        0.5,
        "VO: stereo=%zu temporal=%zu inliers=%d ratio=%.2f px_med=%.3f lock=%d rel_t=[%.3f %.3f %.3f] rel_rot=%.3fdeg world_t=[%.3f %.3f %.3f]",
        stereo_matches.size(),
        temporal_matches.size(),
        inlier_count,
        inlier_ratio,
        median_pixel_shift,
        stationary ? 1 : 0,
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2),
        rel_rotation_deg,
        pose_w_c_.at<double>(0, 3),
        pose_w_c_.at<double>(1, 3),
        pose_w_c_.at<double>(2, 3)
    );

    std::vector<char> matches_mask(temporal_matches.size(), 0);
    for(int row = 0; row < inliers.rows; ++row)
    {
        const int inlier_idx = inliers.at<int>(row, 0);
        if(inlier_idx >= 0 && static_cast<size_t>(inlier_idx) < matches_mask.size())
        {
            matches_mask[inlier_idx] = 1;
        }
    }

    cv::Mat img_matches;
    cv::drawMatches(prev_left_img_orb_, prev_keypoints_left_, left_rect, keypoints_left, temporal_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), matches_mask);

    store_current_frame();
}

void OrbSlam::lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pts);

    ObstacleState state;
    state.stamp = msg->header.stamp;
    double min_front = std::numeric_limits<double>::infinity();
    const double half_fov = 0.5 * obstacle_front_fov_deg_;

    for(const auto& point : pts->points)
    {
        if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            continue;
        }
        if(point.z < obstacle_min_z_ || point.z > obstacle_max_z_)
        {
            continue;
        }

        const double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if(distance > obstacle_max_distance_)
        {
            continue;
        }
        const double angle_deg = std::atan2(point.y, point.x) * 180.0 / M_PI;
        if(point.x > 0.0 && std::abs(angle_deg) <= half_fov)
        {
            min_front = std::min(min_front, distance);
        }
    }

    state.min_front_distance = min_front;
    state.has_obstacle = std::isfinite(min_front) && min_front < obstacle_slow_distance_;
    compute_left_right_clearance(pts, state.min_left_distance, state.min_right_distance);

    if(!prev_obstacle_state_.stamp.isZero() && std::isfinite(state.min_front_distance) && std::isfinite(prev_obstacle_state_.min_front_distance))
    {
        const double dt = (state.stamp - prev_obstacle_state_.stamp).toSec();
        if(dt > 0.02)
        {
            const double radial_speed = std::abs(prev_obstacle_state_.min_front_distance - state.min_front_distance) / dt;
            state.is_dynamic = radial_speed > obstacle_dynamic_speed_thresh_;
        }
    }

    if(!enable_dynamic_avoidance_)
    {
        state.is_dynamic = false;
    }

    obstacle_state_ = state;
    prev_obstacle_state_ = state;
    last_lidar_stamp_ = msg->header.stamp;

    std_msgs::Bool dynamic_msg;
    dynamic_msg.data = obstacle_state_.is_dynamic;
    obstacle_dynamic_puber.publish(dynamic_msg);

    append_lidar_points_to_map(pts, msg->header.stamp);
    publish_avoidance_cmd();

    ROS_INFO_THROTTLE(0.5,
                      "Lidar: size=%zu obstacle=%d dynamic=%d front=%.2f left=%.2f right=%.2f",
                      pts->size(),
                      obstacle_state_.has_obstacle ? 1 : 0,
                      obstacle_state_.is_dynamic ? 1 : 0,
                      obstacle_state_.min_front_distance,
                      obstacle_state_.min_left_distance,
                      obstacle_state_.min_right_distance);
}

#endif