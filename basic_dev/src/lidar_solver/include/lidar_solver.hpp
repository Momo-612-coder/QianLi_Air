#ifndef _LIDAR_SOLVER_HPP_
#define _LIDAR_SOLVER_HPP_

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
#include "sensor_msgs/PointCloud2.h"
#include  "sensor_msgs/Imu.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <time.h>
#include <stdlib.h>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <functional>
#include <string>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "lidar_solver/ObstacleArray.h"

// PCA结果结构体：存储障碍物本体朝向 + 包围盒尺寸 -- 用于A*路径规划中的碰撞检测
struct OBB_detected_object
{
    visualization_msgs::Marker bounding_box; // 可视化用的边界框

    Eigen::Vector3f center;       // 障碍物中心点(世界系)
    Eigen::Vector3f dir1;         // 最长主方向
    Eigen::Vector3f dir2;         // 次主方向
    Eigen::Vector3f dir3;         // 最小厚度方向
    float len1, len2, len3;       // 三个方向长度
    std::vector<pcl::PointXYZ> bbox_vertex; // OBB 8个顶点
};

// 各区间聚类参数
struct ClusterParams {
    double tolerance;      // 聚类半径
    int min_cluster_size;  // 最小点数
    int max_cluster_size;  // 最大点数（可选，过滤过大的地面/墙面）
};

class LidarSolver
{
private:
    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    ros::Subscriber lidar_suber;//lidar数据
    ros::Subscriber odom_suber;//odom数据
    ros::Publisher lidar_pub_;//雷达数据
    ros::Publisher bbox_pub_;// bounding box
    ros::Publisher front_obstacle_pub_;// 前方FOV障碍物中心和长宽高
    ros::Publisher front_obstacle_marker_pub_;// 前方FOV障碍物立方体可视化

    void lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void filter_front_fov_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& out,
                                      const Eigen::Vector3f& sensor_origin,
                                      const Eigen::Matrix3f& target_to_sensor_rotation) const;
    void voxel_grid_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& out,
                                      double leaf_size);
    void point_cloud_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& segmented_clouds,
                                      const std::vector<double>& seg_distances,
                                      const Eigen::Vector3f& sensor_origin);
    void point_cloud_clustering(std_msgs::Header header, int idx,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                      std::vector<OBB_detected_object>& detected_objects,
                                      double cluster_tolerance, int min_cluster_size, int max_cluster_size);
    void publish_front_fov_obstacles(const std_msgs::Header& target_header, const std::string& source_frame);
    bool is_in_front_fov(const OBB_detected_object& obj, const std_msgs::Header& target_header, const std::string& source_frame);
    ros::Time resolve_lidar_stamp(const ros::Time& lidar_stamp) const;
    bool is_valid_detected_object(const OBB_detected_object& obj) const;
    void smooth_detected_objects();

    // 距离阈值（米）
    std::vector<double> seg_distances_ = {2.0, 12.0, 25.0, 30.0};  // 4个阈值 → 5个区间
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_clouds_;  // 存储分割后的点云
    std::vector<std::vector<OBB_detected_object>> obb_detected_objects_; // 存储各区间聚类后的物体信息
    std::vector<OBB_detected_object> previous_smoothed_objects_; // 上一帧平滑后的障碍物，用于抑制框抖动
    std::vector<ClusterParams> cluster_params_; // 存储各区间聚类参数
    Eigen::Vector3f current_velocity_ = Eigen::Vector3f::Zero(); // 保存当前无人机线速度，用于TF降级补偿
    Eigen::Vector3f current_angular_velocity_ = Eigen::Vector3f::Zero(); // 保存角速度，用于姿态插值补偿
    Eigen::Quaternionf current_orientation_ = Eigen::Quaternionf::Identity(); // 保存无人机的姿态，用于补偿点云旋转
    Eigen::Vector3f current_position_ = Eigen::Vector3f::Zero(); // 保存无人机位置
    ros::Time current_odom_stamp_; // 保存odom时间戳

    std::string target_frame_ = "map";
    std::string source_frame_override_;
    double front_fov_deg_ = 120.0;
    double front_fov_margin_deg_ = 5.0;
    double vertical_fov_deg_ = 90.0;
    double vertical_fov_margin_deg_ = 5.0;
    bool fov_prefilter_enabled_ = true;
    bool publish_rviz_visualization_ = true;
    double fov_prefilter_deg_ = 130.0;

    double voxel_leaf_size_ = 0.1;
    int statistical_mean_k_ = 50;
    double statistical_stddev_mul_thresh_ = 1.0;
    double plane_segmentation_distance_threshold_ = 0.05;

    bool use_imu_clock_time_ = true;
    bool drop_lidar_until_clock_ = true;
    double max_lidar_imu_stamp_skew_ = 0.03;
    double min_object_extent_ = 0.05;
    double min_object_height_ = 0.05;
    double max_object_length_ = 12.0;
    double max_object_width_ = 12.0;
    double max_object_height_ = 8.0;
    double max_object_volume_ = 200.0;
    bool temporal_smoothing_enabled_ = true;
    double temporal_smoothing_alpha_ = 0.65;
    double temporal_smoothing_max_match_distance_ = 2.0;
    ros::WallTime last_front_obstacle_publish_wall_time_;
    ros::Time last_front_obstacle_publish_stamp_;
    bool have_last_front_obstacle_publish_time_ = false;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

public:
    LidarSolver(ros::NodeHandle *nh);
    ~LidarSolver();

};

#endif
