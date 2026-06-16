#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// [新增] 引入 TF2 相关的头文件
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace {

class PointCloud2ToLivox {
 public:
  // ==========================================
  // 函数功能: 构造函数，初始化 ROS 参数、订阅器和发布器、TF 监听器
  // ==========================================
  PointCloud2ToLivox(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tf_listener_(tf_buffer_) {
    // 读取原始参数
    pnh.param<std::string>("input_topic", input_topic_, std::string("/airsim_node/drone_1/lidar"));
    pnh.param<std::string>("output_topic", output_topic_, std::string("/livox/lidar"));
    
    // 读取 lidar_odom 的发布话题
    pnh.param<std::string>("output_odom_topic", output_odom_topic_, std::string("/airsim_node/drone_1/lidar_odom"));
    
    pnh.param<int>("queue_size", queue_size_, 10);
    pnh.param<double>("scan_period", scan_period_, 0.1);
    pnh.param<int>("default_line", default_line_, 0);
    pnh.param<int>("default_tag", default_tag_, 0x10);
    pnh.param<int>("lidar_id", lidar_id_, 1);
    pnh.param<bool>("filter/enable", filter_enable_, true);
    pnh.param<double>("filter/min_range", filter_min_range_, 0.8);
    pnh.param<double>("filter/max_range", filter_max_range_, 24.0);
    pnh.param<double>("filter/voxel_leaf_size", filter_voxel_leaf_size_, 0.4);
    pnh.param<double>("filter/radius_search", filter_radius_search_, 1.0);
    pnh.param<int>("filter/min_neighbors", filter_min_neighbors_, 2);
    pnh.param<bool>("filter/log_stats", filter_log_stats_, true);

    if (input_topic_ == output_odom_topic_) {
      ROS_FATAL_STREAM("input_topic and output_odom_topic must be different to avoid loopback. input="
                       << input_topic_ << ", output_odom_topic=" << output_odom_topic_);
      ros::shutdown();
      return;
    }

    if (scan_period_ <= 0.0) {
      ROS_WARN("scan_period <= 0 is invalid, fallback to 0.1s");
      scan_period_ = 0.1;
    }
    if (filter_min_range_ < 0.0) {
      ROS_WARN("filter/min_range < 0 is invalid, fallback to 0.0m");
      filter_min_range_ = 0.0;
    }
    if (filter_max_range_ <= filter_min_range_) {
      ROS_WARN("filter/max_range <= filter/min_range is invalid, fallback to 24.0m");
      filter_max_range_ = 24.0;
    }
    if (filter_voxel_leaf_size_ <= 0.0) {
      ROS_WARN("filter/voxel_leaf_size <= 0 is invalid, fallback to 0.4m");
      filter_voxel_leaf_size_ = 0.4;
    }
    if (filter_radius_search_ <= 0.0) {
      ROS_WARN("filter/radius_search <= 0 is invalid, fallback to 1.0m");
      filter_radius_search_ = 1.0;
    }
    filter_min_neighbors_ = std::max(filter_min_neighbors_, 1);

    const int safe_queue = std::max(queue_size_, 1);
    
    // 初始化订阅器：接收原始点云
    sub_ = nh.subscribe(input_topic_, safe_queue, &PointCloud2ToLivox::pointcloudCallback, this);
    
    // 初始化发布器 1：发布转换后的 Livox 格式数据
    pub_ = nh.advertise<livox_ros_driver::CustomMsg>(output_topic_, safe_queue);
    
    // 发布器 2：发布转换到 camera_init 且旋转后的点云
    odom_pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_odom_topic_, safe_queue);

    ROS_INFO_STREAM("pointcloud2_to_livox started. input=" << input_topic_
                    << ", output(livox)=" << output_topic_ 
                    << ", output(lidar_odom)=" << output_odom_topic_ 
                    << ", scan_period=" << scan_period_
                    << ", filter_enable=" << filter_enable_
                    << ", filter_range=[" << filter_min_range_ << ", " << filter_max_range_ << "]"
                    << ", voxel_leaf_size=" << filter_voxel_leaf_size_
                    << ", radius_search=" << filter_radius_search_
                    << ", min_neighbors=" << filter_min_neighbors_);
  }

 private:
  static bool hasField(const sensor_msgs::PointCloud2& msg, const std::string& field_name) {
    for (const auto& field : msg.fields) {
      if (field.name == field_name) {
        return true;
      }
    }
    return false;
  }

  static uint8_t toUint8Clamped(float value) {
    if (!std::isfinite(value)) {
      return 0;
    }
    const float clamped = std::max(0.0f, std::min(255.0f, value));
    return static_cast<uint8_t>(std::lround(clamped));
  }

  static uint8_t toUint8Clamped(int value) {
    return static_cast<uint8_t>(std::max(0, std::min(255, value)));
  }

  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  size_t copyFiniteRangePoints(const sensor_msgs::PointCloud2& msg, PointCloudXYZ::Ptr& cloud) const {
    cloud->clear();
    const size_t estimated_points = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    cloud->reserve(estimated_points);

    const double min_range_sq = filter_min_range_ * filter_min_range_;
    const double max_range_sq = filter_max_range_ * filter_max_range_;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const double range_sq = static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z;
      if (range_sq < min_range_sq || range_sq > max_range_sq) {
        continue;
      }

      cloud->push_back(pcl::PointXYZ(x, y, z));
    }

    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud->size();
  }

  void publishEgoCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    if (cloud_msg->width == 0 || cloud_msg->height == 0) {
      odom_pub_.publish(*cloud_msg);
      return;
    }

    if (!hasField(*cloud_msg, "x") || !hasField(*cloud_msg, "y") || !hasField(*cloud_msg, "z")) {
      ROS_WARN_THROTTLE(1.0, "skip ego cloud: missing x/y/z fields");
      return;
    }

    try {
      geometry_msgs::TransformStamped transform_stamped =
          tf_buffer_.lookupTransform("camera_init", cloud_msg->header.frame_id,
                                     cloud_msg->header.stamp, ros::Duration(0.02));

      if (!filter_enable_) {
        sensor_msgs::PointCloud2 cloud_transformed;
        tf2::doTransform(*cloud_msg, cloud_transformed, transform_stamped);
        odom_pub_.publish(cloud_transformed);
        return;
      }

      const size_t raw_count = static_cast<size_t>(cloud_msg->width) * static_cast<size_t>(cloud_msg->height);
      PointCloudXYZ::Ptr range_cloud(new PointCloudXYZ);
      const size_t range_count = copyFiniteRangePoints(*cloud_msg, range_cloud);

      sensor_msgs::PointCloud2 range_msg;
      pcl::toROSMsg(*range_cloud, range_msg);
      range_msg.header = cloud_msg->header;

      sensor_msgs::PointCloud2 cloud_transformed_msg;
      tf2::doTransform(range_msg, cloud_transformed_msg, transform_stamped);

      PointCloudXYZ::Ptr transformed_cloud(new PointCloudXYZ);
      pcl::fromROSMsg(cloud_transformed_msg, *transformed_cloud);

      PointCloudXYZ::Ptr voxel_cloud(new PointCloudXYZ);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(transformed_cloud);
      voxel_filter.setLeafSize(filter_voxel_leaf_size_, filter_voxel_leaf_size_, filter_voxel_leaf_size_);
      voxel_filter.filter(*voxel_cloud);

      PointCloudXYZ::Ptr radius_cloud(new PointCloudXYZ);
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
      radius_filter.setInputCloud(voxel_cloud);
      radius_filter.setRadiusSearch(filter_radius_search_);
      radius_filter.setMinNeighborsInRadius(filter_min_neighbors_);
      radius_filter.filter(*radius_cloud);

      sensor_msgs::PointCloud2 output_msg;
      pcl::toROSMsg(*radius_cloud, output_msg);
      output_msg.header = cloud_transformed_msg.header;
      odom_pub_.publish(output_msg);

      if (filter_log_stats_) {
        ROS_INFO_THROTTLE(1.0,
                          "ego cloud filter: raw=%zu range=%zu voxel=%zu radius=%zu output=%zu",
                          raw_count, range_count, voxel_cloud->size(), radius_cloud->size(), radius_cloud->size());
      }
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0, "TF Transform to camera_init failed: %s", ex.what());
    }
  }

  // ==========================================
  // 函数功能: 核心回调函数，处理输入点云并进行双路分发
  // ==========================================
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // ---------------------------------------------------------
    // [原功能区]：转换为 Livox CustomMsg 并发布 (直接读取原始 cloud_msg)
    // ---------------------------------------------------------
    livox_ros_driver::CustomMsg out_msg;
    out_msg.header = cloud_msg->header;
    out_msg.timebase = static_cast<uint64_t>(cloud_msg->header.stamp.toNSec());
    out_msg.lidar_id = toUint8Clamped(lidar_id_);
    out_msg.rsvd[0] = 0;
    out_msg.rsvd[1] = 0;
    out_msg.rsvd[2] = 0;

    if (cloud_msg->width == 0 || cloud_msg->height == 0) {
      out_msg.point_num = 0;
      pub_.publish(out_msg);
      publishEgoCloud(cloud_msg);
      return;
    }

    if (!hasField(*cloud_msg, "x") || !hasField(*cloud_msg, "y") || !hasField(*cloud_msg, "z")) {
      ROS_WARN_THROTTLE(1.0, "skip cloud: missing x/y/z fields");
      out_msg.point_num = 0;
      pub_.publish(out_msg);
      publishEgoCloud(cloud_msg);
      return;
    }

    const bool has_intensity = hasField(*cloud_msg, "intensity");
    const bool has_ring = hasField(*cloud_msg, "ring");
    const bool has_line = hasField(*cloud_msg, "line");
    const bool has_tag = hasField(*cloud_msg, "tag");

    const size_t estimated_points = static_cast<size_t>(cloud_msg->width) * static_cast<size_t>(cloud_msg->height);
    out_msg.points.reserve(estimated_points);

    // 读取原始点云的迭代器
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_intensity;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> iter_ring_u16;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_ring_u8;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_line;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_tag;

    if (has_intensity) {
      iter_intensity.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*cloud_msg, "intensity"));
    }
    if (has_ring) {
      for (const auto& f : cloud_msg->fields) {
        if (f.name == "ring") {
          if (f.datatype == sensor_msgs::PointField::UINT16) {
            iter_ring_u16.reset(new sensor_msgs::PointCloud2ConstIterator<uint16_t>(*cloud_msg, "ring"));
          } else {
            iter_ring_u8.reset(new sensor_msgs::PointCloud2ConstIterator<uint8_t>(*cloud_msg, "ring"));
          }
          break;
        }
      }
    }
    if (has_line) {
      iter_line.reset(new sensor_msgs::PointCloud2ConstIterator<uint8_t>(*cloud_msg, "line"));
    }
    if (has_tag) {
      iter_tag.reset(new sensor_msgs::PointCloud2ConstIterator<uint8_t>(*cloud_msg, "tag"));
    }

    const uint64_t total_points = estimated_points;
    const uint64_t frame_ns = static_cast<uint64_t>(scan_period_ * 1e9);

    for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        if (iter_intensity) ++(*iter_intensity);
        if (iter_ring_u16) ++(*iter_ring_u16);
        if (iter_ring_u8) ++(*iter_ring_u8);
        if (iter_line) ++(*iter_line);
        if (iter_tag) ++(*iter_tag);
        continue;
      }

      livox_ros_driver::CustomPoint pt;
      pt.x = *iter_x;
      pt.y = *iter_y;
      pt.z = *iter_z;

      if (iter_intensity) {
        pt.reflectivity = toUint8Clamped(**iter_intensity);
        ++(*iter_intensity);
      } else {
        pt.reflectivity = 0;
      }

      if (iter_line) {
        pt.line = **iter_line;
        ++(*iter_line);
      } else if (iter_ring_u16) {
        pt.line = toUint8Clamped(static_cast<int>(**iter_ring_u16));
        ++(*iter_ring_u16);
      } else if (iter_ring_u8) {
        pt.line = **iter_ring_u8;
        ++(*iter_ring_u8);
      } else {
        pt.line = toUint8Clamped(default_line_);
      }

      if (iter_tag) {
        pt.tag = **iter_tag;
        ++(*iter_tag);
      } else {
        pt.tag = toUint8Clamped(default_tag_);
      }

      if (total_points > 1) {
        const uint64_t offset = (static_cast<uint64_t>(i) * frame_ns) / (total_points - 1);
        // pt.offset_time = static_cast<uint32_t>(std::min<uint64_t>(offset, std::numeric_limits<uint32_t>::max()));
        pt.offset_time = 0.0;
      } else {
        pt.offset_time = 0;
      }

      out_msg.points.push_back(pt);
    }

    out_msg.point_num = static_cast<uint32_t>(out_msg.points.size());
    pub_.publish(out_msg);

    // ---------------------------------------------------------
    // [EGO 规划点云]：转换至 camera_init，并按配置过滤噪点
    // ---------------------------------------------------------
    publishEgoCloud(cloud_msg);
  }

  // ROS 参数
  std::string input_topic_;
  std::string output_topic_;
  std::string output_odom_topic_;
  int queue_size_ = 10;
  double scan_period_ = 0.1;
  int default_line_ = 0;
  int default_tag_ = 0x10;
  int lidar_id_ = 1;
  bool filter_enable_ = true;
  double filter_min_range_ = 0.8;
  double filter_max_range_ = 24.0;
  double filter_voxel_leaf_size_ = 0.4;
  double filter_radius_search_ = 1.0;
  int filter_min_neighbors_ = 2;
  bool filter_log_stats_ = true;

  // [新增] TF 变换组件
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher odom_pub_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud2_to_livox");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PointCloud2ToLivox converter(nh, pnh);
  ros::spin();
  return 0;
}
