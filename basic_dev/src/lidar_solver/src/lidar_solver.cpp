#ifndef _LIDAR_SOLVER_CPP_
#define _LIDAR_SOLVER_CPP_

#include "lidar_solver.hpp"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "lidar_solver"); // 初始化ros 节点，命名为 lidar_solver
    ros::NodeHandle n; // 创建node控制句柄
    LidarSolver go(&n);
    return 0;
}

LidarSolver::LidarSolver(ros::NodeHandle *nh)
{  
    // 初始化各区间聚类参数
    cluster_params_.push_back({0.1, 5, 250});  // 0-2m // cluster_tolerance为聚类的距离阈值，单位为米， min_cluster_size和max_cluster_size分别为最小和最大簇的点数
    cluster_params_.push_back({0.50, 10, 500});  // 2-8m
    cluster_params_.push_back({1.00, 6, 1000});  // 8-20m
    cluster_params_.push_back({1.50, 3, 2000});  // 20-30m
    cluster_params_.push_back({1.50, 3, 2000});  // 预留区间，避免分割区间数量变化时越界

    nh->param<std::string>("target_frame", target_frame_, "map");
    nh->param<std::string>("source_frame_override", source_frame_override_, "lidar_link");
    nh->param<double>("front_fov_deg", front_fov_deg_, 120.0);
    nh->param<double>("front_fov_margin_deg", front_fov_margin_deg_, 5.0);
    nh->param<double>("vertical_fov_deg", vertical_fov_deg_, 90.0);
    nh->param<double>("vertical_fov_margin_deg", vertical_fov_margin_deg_, 5.0);
    nh->param<bool>("use_rviz", publish_rviz_visualization_, true);
    nh->param<bool>("fov_prefilter/enabled", fov_prefilter_enabled_, true);
    nh->param<double>("fov_prefilter/deg", fov_prefilter_deg_, 130.0);

    nh->param<double>("voxel_grid/leaf_size", voxel_leaf_size_, 0.1);
    nh->param<int>("statistical/mean_k", statistical_mean_k_, 50);
    nh->param<double>("statistical/stddev_mul_thresh", statistical_stddev_mul_thresh_, 1.0);
    nh->param<double>("plane_segmentation/distance_threshold", plane_segmentation_distance_threshold_, 0.05);

    nh->param<bool>("use_imu_clock_time", use_imu_clock_time_, true);
    nh->param<bool>("drop_lidar_until_clock", drop_lidar_until_clock_, true);
    nh->param<double>("max_lidar_imu_stamp_skew", max_lidar_imu_stamp_skew_, 0.03);
    nh->param<double>("object_filter/min_extent", min_object_extent_, 0.05);
    nh->param<double>("object_filter/min_height", min_object_height_, 0.05);
    nh->param<double>("object_filter/max_length", max_object_length_, 12.0);
    nh->param<double>("object_filter/max_width", max_object_width_, 12.0);
    nh->param<double>("object_filter/max_height", max_object_height_, 8.0);
    nh->param<double>("object_filter/max_volume", max_object_volume_, 200.0);
    nh->param<bool>("temporal_smoothing/enabled", temporal_smoothing_enabled_, true);
    nh->param<double>("temporal_smoothing/alpha", temporal_smoothing_alpha_, 0.65);
    nh->param<double>("temporal_smoothing/max_match_distance", temporal_smoothing_max_match_distance_, 2.0);
    for (size_t i = 0; i < cluster_params_.size(); ++i)
    {
        const std::string prefix = "cluster/seg" + std::to_string(i) + "/";
        nh->param<double>(prefix + "tolerance", cluster_params_[i].tolerance, cluster_params_[i].tolerance);
        nh->param<int>(prefix + "min_size", cluster_params_[i].min_cluster_size, cluster_params_[i].min_cluster_size);
        nh->param<int>(prefix + "max_size", cluster_params_[i].max_cluster_size, cluster_params_[i].max_cluster_size);
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    lidar_suber = nh->subscribe<sensor_msgs::PointCloud2>("airsim_node/drone_1/lidar", 1, std::bind(&LidarSolver::lidar_cb, this, std::placeholders::_1));//雷达数据
    odom_suber = nh->subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/drone_state", 1, std::bind(&LidarSolver::odom_cb, this, std::placeholders::_1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()); //获取当前飞行速度

    front_obstacle_pub_ = nh->advertise<lidar_solver::ObstacleArray>("front_fov_obstacles", 1);
    if (publish_rviz_visualization_) {
        lidar_pub_ = nh->advertise<sensor_msgs::PointCloud2>("visualized_point_cloud", 1);
        bbox_pub_ = nh->advertise<visualization_msgs::MarkerArray>("detected_bounding_boxes", 1);
        front_obstacle_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("front_fov_obstacle_markers", 1);
    }

    ros::spin();
}

LidarSolver::~LidarSolver()
{
}

void LidarSolver::odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 获取当前世界坐标系下的飞行速度
    current_velocity_ = Eigen::Vector3f(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);

    current_angular_velocity_ = Eigen::Vector3f(
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z);

    // 获取并保存无人机当前的世界坐标和姿态
    current_position_ = Eigen::Vector3f(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
        
    current_orientation_ = Eigen::Quaternionf(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
        
    current_odom_stamp_ = msg->header.stamp;
}

void LidarSolver::lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud_for_tf = *msg;
    const ros::Time aligned_stamp = resolve_lidar_stamp(msg->header.stamp);
    if (aligned_stamp.isZero())
    {
        return;
    }
    cloud_for_tf.header.stamp = aligned_stamp;
    if (!source_frame_override_.empty())
    {
        cloud_for_tf.header.frame_id = source_frame_override_;
    }

    // ================== 【性能优化：先FOV过滤，再TF转换】 ==================
    // 原雷达点云是在 source_frame (如 lidar_link) 坐标系下的，前方 = 正X轴方向
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_for_tf, *raw_pts);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr fov_pts_sensor(new pcl::PointCloud<pcl::PointXYZ>);
    if (fov_prefilter_enabled_) {
        const double half_fov_rad = fov_prefilter_deg_ * std::acos(-1.0) / 360.0;
        fov_pts_sensor->points.reserve(raw_pts->points.size());
        for (const auto& point : raw_pts->points) {
            // 在传感器机体坐标系下，前视就是 X 轴正方向
            if (point.x <= 0.0f) {
                continue;
            }
            const double yaw = std::atan2(point.y, point.x);
            if (std::abs(yaw) > half_fov_rad) {
                continue;
            }
            fov_pts_sensor->points.push_back(point);
        }
        fov_pts_sensor->width = fov_pts_sensor->points.size();
        fov_pts_sensor->height = 1;
        fov_pts_sensor->is_dense = false;
        fov_pts_sensor->header = raw_pts->header;
    } else {
        *fov_pts_sensor = *raw_pts;
    }
    
    ROS_INFO_THROTTLE(1.0, "After Sensor-Frame FOV prefilter, size: %lu -> %lu",
                      static_cast<unsigned long>(raw_pts->size()), static_cast<unsigned long>(fov_pts_sensor->size()));

    // 将过滤后的大幅缩减的点云重新转换为 ROS Message 以用于 TF 变换
    sensor_msgs::PointCloud2 fov_cloud_for_tf;
    pcl::toROSMsg(*fov_pts_sensor, fov_cloud_for_tf);
    fov_cloud_for_tf.header = cloud_for_tf.header;

    sensor_msgs::PointCloud2 cloud_in_target;
    Eigen::Vector3f sensor_origin_target(0.0f, 0.0f, 0.0f);
    Eigen::Matrix3f target_to_sensor_rotation = Eigen::Matrix3f::Identity();
    geometry_msgs::TransformStamped tf_stamped;
    try
    {
        tf_stamped = tf_buffer_->lookupTransform(
            target_frame_, cloud_for_tf.header.frame_id, cloud_for_tf.header.stamp, ros::Duration(0.05));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "Exact stamp transform failed, trying latest zero time fallback. %s", ex.what());
        try
        {
            tf_stamped = tf_buffer_->lookupTransform(
                target_frame_, cloud_for_tf.header.frame_id, ros::Time(0), ros::Duration(0.02));
        }
        catch (const tf2::TransformException& ex2)
        {
            ROS_WARN_THROTTLE(1.0, "Transform point cloud fallback failed totally: %s", ex2.what());
            return;
        }
    }
    
    sensor_origin_target = Eigen::Vector3f(
        tf_stamped.transform.translation.x,
        tf_stamped.transform.translation.y,
        tf_stamped.transform.translation.z);
    // 计算实际雷达点云时间与查找成功TF时间的时间戳差，如果产生降级(Fallback)，把整个被错位的时间段内对应的线速度积分为刚体平移并补回原位
    double dt = cloud_for_tf.header.stamp.toSec() - tf_stamped.header.stamp.toSec();
    if (std::abs(dt) > 0.005) { 
        Eigen::Vector3f translation_extrapolation = current_velocity_ * dt;
        tf_stamped.transform.translation.x += translation_extrapolation.x();
        tf_stamped.transform.translation.y += translation_extrapolation.y();
        tf_stamped.transform.translation.z += translation_extrapolation.z();
        
        sensor_origin_target += translation_extrapolation;
        ROS_WARN_THROTTLE(1.0, "TF Extrapolated. dt: %.3fs. Shifted: (%.2f, %.2f, %.2f)m", dt, translation_extrapolation.x(), translation_extrapolation.y(), translation_extrapolation.z());
    }
    
    // 【核心修复】：由于 TF 树可能存在延迟或未包含完整的无人机姿态（Pitch/Roll），导致点云在雷达移动时发生晃动。
    // 我们直接使用从 \`drone_state\` 订阅到的最新高频 Odom 的四元数覆盖 TF 旋转部分来进行姿态补偿：
    // 同时考虑当前雷达点云时间戳与 Odom 姿态时间戳可能存在的差异 dt，利用角速度进行旋转积分补偿对齐时间戳。
    double angular_dt = cloud_for_tf.header.stamp.toSec() - current_odom_stamp_.toSec();
    Eigen::Quaternionf aligned_orientation = current_orientation_;
    if (std::abs(angular_dt) > 0.005 && std::abs(angular_dt) < 0.2) // 仅当时间差在合理范围内时进行积分
    {
        Eigen::Vector3f angle_axis = current_angular_velocity_ * angular_dt;
        double angle = angle_axis.norm();
        if (angle > 1e-4) {
            Eigen::Quaternionf delta_q(Eigen::AngleAxisf(angle, angle_axis / angle));
            aligned_orientation = (aligned_orientation * delta_q).normalized();
        }
    }

    tf_stamped.transform.rotation.w = aligned_orientation.w();
    tf_stamped.transform.rotation.x = aligned_orientation.x();
    tf_stamped.transform.rotation.y = aligned_orientation.y();
    tf_stamped.transform.rotation.z = aligned_orientation.z();

    const Eigen::Quaternionf sensor_to_target_rotation(
            tf_stamped.transform.rotation.w,
            tf_stamped.transform.rotation.x,
            tf_stamped.transform.rotation.y,
            tf_stamped.transform.rotation.z);
        target_to_sensor_rotation = sensor_to_target_rotation.normalized().toRotationMatrix().transpose();
        // 这里改用被提前阉割了的 fov_cloud_for_tf 参与繁重的 TF 变换
        tf2::doTransform(fov_cloud_for_tf, cloud_in_target, tf_stamped);
        cloud_in_target.header.stamp = aligned_stamp;
        cloud_in_target.header.frame_id = target_frame_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_in_target, *pts);
    const size_t raw_point_count = pts->size();
    std::vector<int> valid_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr valid_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*pts, *valid_pts, valid_indices);
    pts = valid_pts;
    ROS_INFO_THROTTLE(1.0, "Get lidar data. raw time: %.3f, aligned time: %.3f, size: %lu, valid: %lu",
                      msg->header.stamp.toSec(), aligned_stamp.toSec(),
                      static_cast<unsigned long>(raw_point_count), static_cast<unsigned long>(pts->size()));

    // 因为前面已经在传感器平面做过 FOV 裁剪了，所以这里的原 X 轴剪裁就变味了而且也没必要，直接传给下一级平滑和体素滤波即可
    pcl::PointCloud<pcl::PointXYZ>::Ptr fov_pts = pts; 

    // 体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pts(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid_filter(fov_pts, filtered_pts, voxel_leaf_size_);  // 体素大小
    ROS_INFO_THROTTLE(1.0, "After voxel grid filter, size: %lu", static_cast<unsigned long>(filtered_pts->size()));
    // 统计滤波 (去除离群点)
    pcl::PointCloud<pcl::PointXYZ>::Ptr stat_filtered_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_pts);
    sor.setMeanK(statistical_mean_k_); // 设置用于计算平均距离的邻居数量
    sor.setStddevMulThresh(statistical_stddev_mul_thresh_); // 设置距离阈值，默认值为1.0，表示点与其邻居的平均距离超过全局平均距离的1倍标准差时被认为是离群点
    sor.filter(*stat_filtered_pts);
    ROS_INFO_THROTTLE(1.0, "After statistical outlier removal, size: %lu", static_cast<unsigned long>(stat_filtered_pts->size()));
    // 平面分割 (移除地面)
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_segmentation_distance_threshold_); // 设置点到模型的距离阈值，单位为米
    seg.setInputCloud(stat_filtered_pts);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty())    {
        ROS_WARN_THROTTLE(1.0, "No plane found in point cloud.");
    } else {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(stat_filtered_pts);
        extract.setIndices(inliers);
        extract.setNegative(true); // true表示提取非平面点云，false表示提取平面点云
        extract.filter(*stat_filtered_pts);
        ROS_INFO_THROTTLE(1.0, "After plane segmentation, size: %lu", static_cast<unsigned long>(stat_filtered_pts->size()));
    }
    // 点云分割
    point_cloud_segmentation(stat_filtered_pts, segmented_clouds_, seg_distances_, sensor_origin_target);
    for (size_t i = 0; i < segmented_clouds_.size(); i++)
    {
        ROS_INFO_THROTTLE(1.0, "Segmented cloud %lu, size: %lu",
                          static_cast<unsigned long>(i), static_cast<unsigned long>(segmented_clouds_[i]->size()));
    }
    // 点云聚类
    const size_t cluster_segment_count = std::min(segmented_clouds_.size(), cluster_params_.size());
    if (cluster_segment_count < segmented_clouds_.size())
    {
        ROS_WARN_THROTTLE(1.0, "Only %lu cluster parameter sets for %lu lidar segments.",
                          static_cast<unsigned long>(cluster_params_.size()),
                          static_cast<unsigned long>(segmented_clouds_.size()));
    }
    size_t total_detected_objects = 0;
    for (size_t i = 0; i < cluster_segment_count; i++)
    {
        std::vector<OBB_detected_object> obb_detected_objects;
        point_cloud_clustering(cloud_in_target.header, i, segmented_clouds_[i], obb_detected_objects, cluster_params_[i].tolerance, cluster_params_[i].min_cluster_size, cluster_params_[i].max_cluster_size);
        total_detected_objects += obb_detected_objects.size();
        obb_detected_objects_.push_back(obb_detected_objects);
        ROS_INFO_THROTTLE(1.0, "Clustered cloud %lu, obb detected objects: %lu",
                          static_cast<unsigned long>(i), static_cast<unsigned long>(obb_detected_objects.size()));
    }
    ROS_INFO("Clustered all front FOV segments. segments=%lu, total_detected_objects=%lu.",
             static_cast<unsigned long>(cluster_segment_count),
             static_cast<unsigned long>(total_detected_objects));
    smooth_detected_objects();
    
    // 处理聚类结果，例如发布检测到的物体信息，或可视化等
    // 发布降采样后的点云
    if (publish_rviz_visualization_) {
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_pts, output_msg);
        output_msg.header = cloud_in_target.header;
        lidar_pub_.publish(output_msg);

        // 汇总并发布聚类得到的边界框
        visualization_msgs::MarkerArray bbox_array;
        // 添加一个删除所有标记的指令，防止残留旧物体
        visualization_msgs::Marker delete_all_marker;
        delete_all_marker.header = cloud_in_target.header;
        delete_all_marker.action = 3; // DELETEALL (ROS Kinetic+)
        bbox_array.markers.push_back(delete_all_marker);

        int current_id = 0;
        for (size_t i = 0; i < obb_detected_objects_.size(); i++)
        {
            for (size_t j = 0; j < obb_detected_objects_[i].size(); ++j)
            {
                visualization_msgs::Marker marker = obb_detected_objects_[i][j].bounding_box;
                marker.header = cloud_in_target.header;
                marker.id = current_id++;
                bbox_array.markers.push_back(marker);
            }
        }
        bbox_pub_.publish(bbox_array);
    }
    publish_front_fov_obstacles(cloud_in_target.header, cloud_for_tf.header.frame_id);

    // 处理完成，清除分割和聚类结果，准备下一帧数据
    segmented_clouds_.clear();
    obb_detected_objects_.clear();
}

ros::Time LidarSolver::resolve_lidar_stamp(const ros::Time& lidar_stamp) const
{
    if (!use_imu_clock_time_)
    {
        if (lidar_stamp.isZero())
        {
            ROS_WARN_THROTTLE(1.0, "Ignore lidar frame with zero stamp.");
        }
        return lidar_stamp;
    }

    const ros::Time imu_clock_stamp = ros::Time::now();
    if (imu_clock_stamp.isZero())
    {
        if (drop_lidar_until_clock_)
        {
            ROS_WARN_THROTTLE(1.0, "Waiting for IMU-driven /clock before processing lidar frames.");
            return ros::Time(0);
        }

        ROS_WARN_THROTTLE(1.0, "IMU-driven /clock is zero, falling back to lidar stamp.");
        return lidar_stamp;
    }

    if (lidar_stamp.isZero())
    {
        ROS_WARN_THROTTLE(1.0, "Lidar stamp is zero, aligning this frame to IMU-driven /clock.");
        return imu_clock_stamp;
    }

    const double stamp_skew = std::abs((lidar_stamp - imu_clock_stamp).toSec());
    if (stamp_skew > max_lidar_imu_stamp_skew_)
    {
        ROS_WARN_THROTTLE(1.0,
                          "Lidar/IMU stamp skew %.3f s is larger than %.3f s, using IMU-driven /clock for lidar.",
                          stamp_skew, max_lidar_imu_stamp_skew_);
        return imu_clock_stamp;
    }

    return lidar_stamp;
}

bool LidarSolver::is_valid_detected_object(const OBB_detected_object& obj) const
{
    if (!std::isfinite(obj.center.x()) || !std::isfinite(obj.center.y()) || !std::isfinite(obj.center.z()) ||
        !std::isfinite(obj.len1) || !std::isfinite(obj.len2) || !std::isfinite(obj.len3))
    {
        return false;
    }

    if (obj.len1 < min_object_extent_ && obj.len2 < min_object_extent_)
    {
        return false;
    }
    if (obj.len3 < min_object_height_)
    {
        return false;
    }
    if (obj.len1 > max_object_length_ || obj.len2 > max_object_width_ || obj.len3 > max_object_height_)
    {
        return false;
    }

    const double volume = std::max(obj.len1, 0.0f) * std::max(obj.len2, 0.0f) * std::max(obj.len3, 0.0f);
    return volume <= max_object_volume_;
}

void LidarSolver::smooth_detected_objects()
{
    std::vector<OBB_detected_object> current_objects;
    std::vector<bool> previous_used(previous_smoothed_objects_.size(), false);
    for (auto& segment_objects : obb_detected_objects_)
    {
        for (auto& obj : segment_objects)
        {
            if (!temporal_smoothing_enabled_ || previous_smoothed_objects_.empty())
            {
                current_objects.push_back(obj);
                continue;
            }

            int best_idx = -1;
            double best_distance_sq = temporal_smoothing_max_match_distance_ * temporal_smoothing_max_match_distance_;
            for (size_t i = 0; i < previous_smoothed_objects_.size(); ++i)
            {
                if (previous_used[i])
                {
                    continue;
                }

                const OBB_detected_object& prev = previous_smoothed_objects_[i];
                const double dx = obj.center.x() - prev.center.x();
                const double dy = obj.center.y() - prev.center.y();
                const double dz = obj.center.z() - prev.center.z();
                const double distance_sq = dx * dx + dy * dy + dz * dz;
                if (distance_sq < best_distance_sq)
                {
                    best_distance_sq = distance_sq;
                    best_idx = static_cast<int>(i);
                }
            }

            if (best_idx >= 0)
            {
                previous_used[best_idx] = true;
                const OBB_detected_object& prev = previous_smoothed_objects_[best_idx];
                const double alpha = std::min(1.0, std::max(0.0, temporal_smoothing_alpha_));
                const double beta = 1.0 - alpha;

                obj.center.x() = static_cast<float>(alpha * obj.center.x() + beta * prev.center.x());
                obj.center.y() = static_cast<float>(alpha * obj.center.y() + beta * prev.center.y());
                obj.center.z() = static_cast<float>(alpha * obj.center.z() + beta * prev.center.z());
                obj.len1 = alpha * obj.len1 + beta * prev.len1;
                obj.len2 = alpha * obj.len2 + beta * prev.len2;
                obj.len3 = alpha * obj.len3 + beta * prev.len3;

                obj.bounding_box.pose.position.x = obj.center.x();
                obj.bounding_box.pose.position.y = obj.center.y();
                obj.bounding_box.pose.position.z = obj.center.z();
                obj.bounding_box.scale.x = obj.len1;
                obj.bounding_box.scale.y = obj.len2;
                obj.bounding_box.scale.z = obj.len3;
            }

            current_objects.push_back(obj);
        }
    }

    previous_smoothed_objects_ = current_objects;
}

void LidarSolver::publish_front_fov_obstacles(const std_msgs::Header& target_header, const std::string& source_frame)
{
    lidar_solver::ObstacleArray obstacle_array;
    obstacle_array.header = target_header;

    visualization_msgs::MarkerArray marker_array;
    if (publish_rviz_visualization_) {
        visualization_msgs::Marker delete_all_marker;
        delete_all_marker.header = target_header;
        delete_all_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all_marker);
    }

    uint32_t current_id = 0;
    for (size_t i = 0; i < obb_detected_objects_.size(); i++)
    {
        for (size_t j = 0; j < obb_detected_objects_[i].size(); ++j)
        {
            const OBB_detected_object& obj = obb_detected_objects_[i][j];
            if (!is_in_front_fov(obj, target_header, source_frame))
            {
                continue;
            }

            // # OBB.msg
            // std_msgs/Header header   # 时间戳和坐标系
            // uint32 id                # 障碍物跟踪ID，保持稳定
            // geometry_msgs/Pose pose  # 障碍物中心点 + 姿态 (朝向)
            // geometry_msgs/Vector3 scale # 三个轴方向的全尺寸

            // struct OBB_detected_object
            // {
            //     Eigen::Vector3f center;       // 障碍物中心点(世界系)
            //     Eigen::Vector3f dir1;         // 最长主方向
            //     Eigen::Vector3f dir2;         // 次主方向
            //     Eigen::Vector3f dir3;         // 最小厚度方向
            //     float len1, len2, len3;       // 三个方向长度
            //     std::vector<pcl::PointXYZ> bbox_vertex; // OBB 8个顶点
            // };

            lidar_solver::Obstacle obstacle;
            obstacle.header = target_header;
            obstacle.id = current_id;
            obstacle.pose.position.x = obj.center.x();
            obstacle.pose.position.y = obj.center.y();
            obstacle.pose.position.z = obj.center.z();
            // 将三个主方向 (dir1, dir2, dir3) 构造成旋转矩阵，再转成四元数
            Eigen::Matrix3f rot;
            rot.col(0) = obj.dir1.normalized(); // 长轴
            rot.col(1) = obj.dir2.normalized(); // 中轴
            rot.col(2) = rot.col(0).cross(rot.col(1)); // 直接用叉乘保证它是右手坐标系
            Eigen::Quaternionf quat(rot);
            quat.normalize();
            obstacle.pose.orientation.x = quat.x();
            obstacle.pose.orientation.y = quat.y();
            obstacle.pose.orientation.z = quat.z();
            obstacle.pose.orientation.w = quat.w();
            obstacle.scale.x = obj.len1;
            obstacle.scale.y = obj.len2;
            obstacle.scale.z = obj.len3;
            obstacle_array.obstacles.push_back(obstacle);


            if (publish_rviz_visualization_) {
                visualization_msgs::Marker marker;
                marker.header = target_header;
                marker.ns = "front_fov_obstacle_cube";
                marker.id = static_cast<int>(current_id);
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position = obstacle.pose.position;
                marker.pose.orientation = obstacle.pose.orientation;
                marker.scale.x = std::max(0.05f, obj.len1);
                marker.scale.y = std::max(0.05f, obj.len2);
                marker.scale.z = std::max(0.05f, obj.len3);
                marker.color.r = 1.0f;
                marker.color.g = 0.55f;
                marker.color.b = 0.05f;
                marker.color.a = 0.35f;
                marker.lifetime = ros::Duration(0.3);
                marker_array.markers.push_back(marker);
            }

            current_id++;
        }
    }

    front_obstacle_pub_.publish(obstacle_array);
    const ros::WallTime publish_wall_time = ros::WallTime::now();
    if (have_last_front_obstacle_publish_time_) {
        const double wall_dt = (publish_wall_time - last_front_obstacle_publish_wall_time_).toSec();
        const double stamp_dt = (!target_header.stamp.isZero() && !last_front_obstacle_publish_stamp_.isZero())
            ? (target_header.stamp - last_front_obstacle_publish_stamp_).toSec()
            : 0.0;
        ROS_INFO("Published front FOV obstacle frame. count=%lu, wall_dt=%.3f s, stamp_dt=%.3f s, stamp=%.3f.",
                 static_cast<unsigned long>(obstacle_array.obstacles.size()),
                 wall_dt, stamp_dt, target_header.stamp.toSec());
    } else {
        ROS_INFO("Published first front FOV obstacle frame. count=%lu, stamp=%.3f.",
                 static_cast<unsigned long>(obstacle_array.obstacles.size()),
                 target_header.stamp.toSec());
    }
    last_front_obstacle_publish_wall_time_ = publish_wall_time;
    last_front_obstacle_publish_stamp_ = target_header.stamp;
    have_last_front_obstacle_publish_time_ = true;

    if (publish_rviz_visualization_) {
        front_obstacle_marker_pub_.publish(marker_array);
    }
}

bool LidarSolver::is_in_front_fov(const OBB_detected_object& obj, const std_msgs::Header& target_header, const std::string& source_frame)
{
    if (source_frame.empty()) return false;

    // 确保方向向量为单位向量 (如果是构造时已保证则可省略)
    Eigen::Vector3f u1 = obj.dir1.normalized();
    Eigen::Vector3f u2 = obj.dir2.normalized();
    Eigen::Vector3f u3 = obj.dir3.normalized();

    const double half_len1 = 0.5 * std::max(0.0f, obj.len1);
    const double half_len2 = 0.5 * std::max(0.0f, obj.len2);
    const double half_len3 = 0.5 * std::max(0.0f, obj.len3);

    // 统一计算半水平FOV + 半垂直FOV
    const double half_hfov = (front_fov_deg_ + front_fov_margin_deg_) * M_PI / 360.0;   // 总角转半角
    const double half_vfov = (vertical_fov_deg_ + vertical_fov_margin_deg_) * M_PI / 360.0; // 同理增加垂直

    // 提取候选点 (中心+8角点)
    std::vector<Eigen::Vector3f> candidates;
    candidates.reserve(9);
    candidates.push_back(obj.center);
    for (int sx : {-1, 1})
        for (int sy : {-1, 1})
            for (int sz : {-1, 1})
                candidates.push_back(obj.center + (sx*half_len1)*u1 + (sy*half_len2)*u2 + (sz*half_len3)*u3);

    for (const auto& pt : candidates) {
        geometry_msgs::PointStamped pt_map;
        pt_map.header = target_header;
        pt_map.point.x = pt.x(); pt_map.point.y = pt.y(); pt_map.point.z = pt.z();

        geometry_msgs::PointStamped pt_source;
        try {
            pt_source = tf_buffer_->transform(pt_map, source_frame, ros::Duration(0.05));
        } catch (...) {
            // 变换失败时的策略：可考虑返回 true 或沿用上次状态，这里暂保持原有
            continue;
        }

        if (pt_source.point.x <= 0.0) continue;

        double yaw = std::atan2(pt_source.point.y, pt_source.point.x);
        if (std::abs(yaw) > half_hfov) continue;

        // 垂直检查
        double pitch = std::atan2(pt_source.point.z, pt_source.point.x);
        if (std::abs(pitch) > half_vfov) continue;

        return true;  // 任一点在FOV内即返回可见
    }
    return false;
}

void LidarSolver::filter_front_fov_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& out,
                                      const Eigen::Vector3f& sensor_origin,
                                      const Eigen::Matrix3f& target_to_sensor_rotation) const
{
    // ====== 【注意：此函数已被弃用，因为已经在传感器原始系下做了原生 FOV 过滤】======
    // 为了兼容旧的头文件签名，保留此占位符体
    if(in && out) {
        *out = *in;
    }
}

// 体素滤波, 将点云根据栅格大小划分在一个体素内， leaf_size为体素的边长，单位为米，需自行调整
void LidarSolver::voxel_grid_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& out,
                                      double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(in);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*out);
}

// 点云分割, 将点云根据距离划分成不同的区域， seg_distances为距离阈值，单位为米，需自行调整
void LidarSolver::point_cloud_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& segmented_clouds,
                                      const std::vector<double>& seg_distances,
                                      const Eigen::Vector3f& sensor_origin)
{   
    segmented_clouds.clear();
    for (size_t i = 0; i < seg_distances.size() + 1; i++)
    {
        segmented_clouds.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
    }

    if (seg_distances.empty())
    {
        *segmented_clouds.front() = *in;
        return;
    }

    for (const auto& point : in->points)
    {
        // 使用平方阈值来判断，减少开方运算
        const double dx = point.x - sensor_origin.x();
        const double dy = point.y - sensor_origin.y();
        const double dz = point.z - sensor_origin.z();
        const double distance_squared = dx * dx + dy * dy + dz * dz;

        if(distance_squared > seg_distances.back() * seg_distances.back())
        {
            continue;  // 超出最大距离阈值，忽略该点
        }

        size_t segment_idx = 0;
        while (segment_idx < seg_distances.size() &&
               distance_squared >= seg_distances[segment_idx] * seg_distances[segment_idx])
        {
            ++segment_idx;
        }
        segmented_clouds[segment_idx]->points.push_back(point);
    }
}

// 点云聚类, 将点云根据距离划分成不同的簇， cluster_tolerance为聚类的距离阈值，单位为米， min_cluster_size和max_cluster_size分别为最小和最大簇的点数，需自行调整
void LidarSolver::point_cloud_clustering(std_msgs::Header header, int idx,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                      std::vector<OBB_detected_object>& detected_objects,
                                      double cluster_tolerance, int min_cluster_size, int max_cluster_size)
{
    detected_objects.clear();
    if (!in || in->empty())
    {
        return;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in);
    ec.extract(cluster_indices);

    for (const auto& indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& index : indices.indices)
        {
            cluster->points.push_back(in->points[index]);
        }

        // 使用 PCL 的 PCA 类
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cluster);
        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors(); // 每列是一个特征向量，按特征值大小排序
        Eigen::Vector3f eigen_values = pca.getEigenValues();
        Eigen::Vector4f centroid = pca.getMean().head<4>(); // (x, y, z, 1)

        // 将点云变换至PCA空间（以质心为原点，主轴对齐坐标轴）
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_projected(new pcl::PointCloud<pcl::PointXYZ>);
        pca.project(*cluster, *cluster_projected);

        // 在投影空间求轴对齐包围盒的半尺寸
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster_projected, min_pt, max_pt);
        float half_x = std::max(std::abs(max_pt.x), std::abs(min_pt.x));
        float half_y = std::max(std::abs(max_pt.y), std::abs(min_pt.y));
        float half_z = std::max(std::abs(max_pt.z), std::abs(min_pt.z));

        Eigen::Matrix3f rotation = eigen_vectors; // 直接使用 PCA 的特征向量作为旋转矩阵
        // 保证旋转矩阵是右手坐标系 (判定行列式，如果小于0则反转第三轴)
        if (rotation.determinant() < 0.0f) {
            rotation.col(2) *= -1.0f;
        }
        
        // 转为四元数来可视化
        Eigen::Quaternionf quat(rotation);
        quat.normalize();

        // 全尺寸
        float len1 = 2.0f * half_x;
        float len2 = 2.0f * half_y;
        float len3 = 2.0f * half_z;

        // obj输入
        OBB_detected_object obj;
        obj.center = centroid.head<3>();
        obj.dir1 = rotation.col(0); // 主方向
        obj.dir2 = rotation.col(1); // 次方向
        obj.dir3 = rotation.col(2); // 垂直方向
        obj.len1 = len1;
        obj.len2 = len2;
        obj.len3 = len3;
        obj.bounding_box.header = header;
        obj.bounding_box.ns = "obb_detected_objects";
        obj.bounding_box.id = static_cast<int>(detected_objects.size());
        obj.bounding_box.type = visualization_msgs::Marker::CUBE;
        obj.bounding_box.action = visualization_msgs::Marker::ADD;
        obj.bounding_box.pose.position.x = obj.center.x();
        obj.bounding_box.pose.position.y = obj.center.y();
        obj.bounding_box.pose.position.z = obj.center.z();
        obj.bounding_box.pose.orientation.x = quat.x();
        obj.bounding_box.pose.orientation.y = quat.y();
        obj.bounding_box.pose.orientation.z = quat.z();
        obj.bounding_box.pose.orientation.w = quat.w();
        obj.bounding_box.scale.x = std::max(0.05f, len1);
        obj.bounding_box.scale.y = std::max(0.05f, len2);
        obj.bounding_box.scale.z = std::max(0.05f, len3);
        obj.bounding_box.color.r = 0.0f;
        obj.bounding_box.color.g = 1.0f;
        obj.bounding_box.color.b = 0.0f;
        obj.bounding_box.color.a = 0.5f;

        // 过滤掉不合理的检测结果
        if (!is_valid_detected_object(obj))
        {            
            continue;
        }

        detected_objects.push_back(obj);
    }
}

#endif
