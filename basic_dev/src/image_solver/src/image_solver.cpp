#ifndef _IMAGE_SOLVER_CPP_
#define _IMAGE_SOLVER_CPP_

#include "image_solver.hpp"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "image_solver"); // 初始化ros 节点，命名为 image_solver
    ros::NodeHandle n; // 创建node控制句柄
    ImageSolver go(&n);
    return 0;
}

ImageSolver::ImageSolver(ros::NodeHandle *nh)
{  
    init_camera_params(); // 初始化相机内参和外参
    init_cuda_detector(); // 初始化神经网络模型

    stereo_correction(); // 计算双目矫正映射表

    //创建图像传输控制句柄
    it = std::make_unique<image_transport::ImageTransport>(*nh); 
    front_left_img = cv::Mat(720, 960, CV_8UC3, cv::Scalar(0));
    front_right_img = cv::Mat(720, 960, CV_8UC3, cv::Scalar(0));

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    front_left_view_suber = it->subscribe("airsim_node/drone_1/front_left/Scene", 1, std::bind(&ImageSolver::front_left_view_cb, this,  std::placeholders::_1));
    front_right_view_suber = it->subscribe("airsim_node/drone_1/front_right/Scene", 1, std::bind(&ImageSolver::front_right_view_cb, this,  std::placeholders::_1));
    odom_suber = nh->subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/drone_state", 1, std::bind(&ImageSolver::odom_cb, this, std::placeholders::_1));//imu与gps数据融合后的位姿数据

    front_left_pub = it->advertise("image_solver/front_left/image", 1);
    front_right_pub = it->advertise("image_solver/front_right/image", 1);
    score_gate_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("image_solver/score_gate_target_marker", 1, true);
    target_point_pub_ = nh->advertise<geometry_msgs::PointStamped>("image_solver/score_gate_target_point", 1, true);
    target_yaw_pub_ = nh->advertise<std_msgs::Float64>("image_solver/score_gate_target_yaw", 1, true);

    nh->param<std::string>("target_frame", target_frame_, "map");
    nh->param<std::string>("front_left_frame_override", front_left_frame_override_, "front_left_camera_link");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    ros::spin();
}

ImageSolver::~ImageSolver()
{
}

void ImageSolver::front_left_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_left_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(!cv_front_left_ptr->image.empty())
    {
        //ROS_INFO("Get front left image.: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
        //ROS_INFO("Image size: %d x %d", cv_front_left_ptr->image.cols, cv_front_left_ptr->image.rows);
        front_left_img = cv_front_left_ptr->image.clone();
        // // 100ms记录一张图片
        // if (msg->header.stamp.sec + msg->header.stamp.nsec*1e-9 - front_left_img_timestamp > 0.1)
        // {
        //     front_left_img_timestamp = msg->header.stamp.sec + msg->header.stamp.nsec*1e-9;
        //     std::string filename = "/basic_dev/score_pictures/front_left_" + std::to_string(front_left_img_timestamp) + ".jpg";
        //     cv::imwrite(filename, front_left_img);
        // }
        //find_score_gate_center(front_left_img);

        if(!map1_l.empty() && !map2_l.empty())
        {
            cv::remap(front_left_img, front_left_img, map1_l, map2_l, cv::INTER_LINEAR);
        }

        left_cuda_detector_.infer(front_left_img);
        if (left_cuda_detector_.score_gate_.is_found)
        {
            // 显示检测结果
            cv::rectangle(front_left_img, left_cuda_detector_.score_gate_.rect, cv::Scalar(0, 255, 0), 2);  // 绿色矩形框
            cv::circle(front_left_img, left_cuda_detector_.score_gate_.center, 5, cv::Scalar(0, 0, 255), -1);  // 红色中心点
            for (const auto& pt : left_cuda_detector_.score_gate_.four_points)
            {
                cv::circle(front_left_img, pt, 5, cv::Scalar(255, 0, 0), -1);  // 蓝色关键点
            }
            // 类别➕置信度
            std::string label = "Score Gate" + std::to_string(left_cuda_detector_.score_gate_.confidence);
            cv::putText(front_left_img, label, cv::Point(left_cuda_detector_.score_gate_.rect.x, left_cuda_detector_.score_gate_.rect.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            left_cuda_detector_.score_gate_.timestamp = msg->header.stamp.sec + msg->header.stamp.nsec*1e-9; // 记录时间戳
            if (get_score_gate_depth())
            {
                double depth = target_score_gate_.depth;
                target_score_gate_ = left_cuda_detector_.score_gate_; // 保存检测结果，后续可以进行匹配和深度计算
                target_score_gate_.depth = depth; // 恢复深度信息
                target_score_gate_.camera_pixel = left_cuda_detector_.score_gate_.center;

                if (get_score_gate_map_point(msg->header))
                {
                    publish_score_gate_marker(msg->header.stamp);
                }

                // 在图像中心显示深度信息
                std::string depth_label = "Depth: " + std::to_string(target_score_gate_.depth) + " m";
                cv::putText(front_left_img, depth_label, left_cuda_detector_.score_gate_.center, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

                std::string map_label = "Map: [" + std::to_string(target_score_gate_.map_point.x) + ", " +
                                        std::to_string(target_score_gate_.map_point.y) + ", " +
                                        std::to_string(target_score_gate_.map_point.z) + "]";
                cv::putText(front_left_img, map_label, cv::Point(left_cuda_detector_.score_gate_.rect.x, left_cuda_detector_.score_gate_.rect.y - 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            }

            // cv::imshow("front_left_img", front_left_img);
            // cv::waitKey(1);
            front_left_pub.publish(cv_bridge::CvImage(cv_front_left_ptr->header, cv_front_left_ptr->encoding, front_left_img).toImageMsg());
            left_cuda_detector_.score_gate_.is_found = false; // 重置状态，准备下一次检测
        }
        else
        {
            // cv::imshow("front_left_img", front_left_img);
            // cv::waitKey(1);
            front_left_pub.publish(cv_bridge::CvImage(cv_front_left_ptr->header, cv_front_left_ptr->encoding, front_left_img).toImageMsg());
        }
    }
}

void ImageSolver::front_right_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_right_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(!cv_front_right_ptr->image.empty())
    {
        //ROS_INFO("Get front right image.%f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
        front_right_img = cv_front_right_ptr->image.clone();

        if(!map1_r.empty() && !map2_r.empty())
        {
            cv::remap(front_right_img, front_right_img, map1_r, map2_r, cv::INTER_LINEAR);
        }

        right_cuda_detector_.infer(front_right_img);
        if (right_cuda_detector_.score_gate_.is_found)
        {
            // 保留最近5个检测结果
            if (right_score_gates_.size() >= 5)
            {
                right_score_gates_.erase(right_score_gates_.begin());
            }
            right_cuda_detector_.score_gate_.timestamp = msg->header.stamp.sec + msg->header.stamp.nsec*1e-9; // 记录时间戳
            right_score_gates_.push_back(right_cuda_detector_.score_gate_); // 将检测结果保存到列表中，后续可以进行匹配和深度计算
            // 显示检测结果
            cv::rectangle(front_right_img, right_cuda_detector_.score_gate_.rect, cv::Scalar(0, 255, 0), 2);  // 绿色矩形框
            cv::circle(front_right_img, right_cuda_detector_.score_gate_.center, 5, cv::Scalar(0, 0, 255), -1);  // 红色中心点
            for (const auto& pt : right_cuda_detector_.score_gate_.four_points)
            {
                cv::circle(front_right_img, pt, 5, cv::Scalar(255, 0, 0), -1);  // 蓝色关键点
            }
            // 类别➕置信度
            std::string label = "Score Gate" + std::to_string(right_cuda_detector_.score_gate_.confidence);
            cv::putText(front_right_img, label, cv::Point(right_cuda_detector_.score_gate_.rect.x, right_cuda_detector_.score_gate_.rect.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            // cv::imshow("front_right_img", front_right_img);
            // cv::waitKey(1);
            // 发布front_right_img
            front_right_pub.publish(cv_bridge::CvImage(cv_front_right_ptr->header, cv_front_right_ptr->encoding, front_right_img).toImageMsg());
            right_cuda_detector_.score_gate_.is_found = false; // 重置状态，准备下一次检测
        }
        else
        {
            // cv::imshow("front_right_img", front_right_img);
            // cv::waitKey(1);
            front_right_pub.publish(cv_bridge::CvImage(cv_front_right_ptr->header, cv_front_right_ptr->encoding, front_right_img).toImageMsg());
        }
    }
}

void ImageSolver::odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Point drone_position;
    drone_position.x = msg->pose.pose.position.x;
    drone_position.y = msg->pose.pose.position.y;
    drone_position.z = msg->pose.pose.position.z;

    path_planner_.update_drone_position(drone_position);

    if (path_planner_.has_target())
    {
        geometry_msgs::Point current_target = path_planner_.current_target();
        // 发布当前路径目标点
        geometry_msgs::PointStamped target_point_msg;
        target_point_msg.header.stamp = ros::Time::now();
        target_point_msg.header.frame_id = target_frame_;
        target_point_msg.point.x = current_target.x;
        target_point_msg.point.y = current_target.y;
        target_point_msg.point.z = current_target.z;
        target_point_pub_.publish(target_point_msg);
    }
}

void ImageSolver::init_cuda_detector()
{
    DL_INIT_PARAM init_param;
    init_param.modelPath = "/basic_dev/model/gate.onnx";
    init_param.imgSize = {480, 480};
    init_param.rectConfidenceThreshold = 0.5;
    init_param.cudaEnable = true;
    init_param.logSeverityLevel = 3;
    init_param.intraOpNumThreads = 1;

    const char* ret = left_cuda_detector_.CreateSession(init_param);
    if (ret != RET_OK)
    {        
        std::cout << "Failed to initialize LeftCudaDetector: " << ret << std::endl;
    }
    const char* ret_r = right_cuda_detector_.CreateSession(init_param);
    if (ret_r != RET_OK)    {        
        std::cout << "Failed to initialize RightCudaDetector: " << ret_r << std::endl;
    }   
}

void ImageSolver::init_camera_params()
{
    img_width_ = 960;
    img_height_ = 720;
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
}

ScoreGate ImageSolver::find_score_gate_center(const cv::Mat& img)
{
    ScoreGate result = {false, cv::Point2f(0, 0)};

    // 将较暗的部分设置为黑色, 用rgb法
    cv::Mat mask;
    mask = img.clone();
    int h = img.rows;
    int w = img.cols;
    
    // 遍历每个像素（RGB判断）
    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < w; x++)
        {
            cv::Vec3b pixel = img.at<cv::Vec3b>(y, x);  // 获取像素的BGR值
            int b = pixel[0];
            int g = pixel[1];
            int r = pixel[2];
            
            // 计算亮度（RGB法）
            int brightness = (r + g + b) / 3;
            
            // 较暗 → 变黑
            if (brightness < 100)  // 亮度阈值（可调整）
            {
                mask.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);  // BGR 黑色
            }
        }
    }

    // 转成灰度图
    cv::Mat gray_mask;
    cv::cvtColor(mask, gray_mask, cv::COLOR_BGR2GRAY);

    // 在mask上寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 找到最大的轮廓，且符合面积要求和长宽比要求
    double max_area = 0.0;
    cv::Rect best_rect;
    for (const auto& contour : contours)
    {
        // 画出所有轮廓
        cv::drawContours(img, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255, 0, 0), 2);  // 计算轮廓面积
        double area = cv::contourArea(contour);
        if (area > max_area && area > 500)  // 面积阈值（可调整）
        {
            cv::Rect rect = cv::boundingRect(contour);
            float aspect_ratio = static_cast<float>(rect.width) / rect.height;
            if (aspect_ratio < 0.8)  // 长宽比要求（可调整）
            {
                max_area = area;
                best_rect = rect;
            }
        }
    }

    // 如果找到了符合要求的轮廓，计算其中心点
    if (max_area > 0.0)
    {
        result.is_found = true;
        result.center = cv::Point2f(best_rect.x + best_rect.width / 2.0f, best_rect.y + best_rect.height / 2.0f);
        // 在原图上画出找到的轮廓和中心点
        cv::rectangle(img, best_rect, cv::Scalar(0, 255, 0), 2);  // 绿色矩形框
        cv::circle(img, result.center, 5, cv::Scalar(0, 0, 255), -1);  // 红色中心点
    }

    cv::imshow("img", img);
    cv::waitKey(1);

    return result;
}

void ImageSolver::stereo_correction()
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

    if(img_width_ <= 0 || img_height_ <= 0)
    {
        ROS_ERROR("Invalid image size for stereo correction: %d x %d", img_width_, img_height_);
        return;
    }

    cv::Size img_size(img_width_, img_height_);
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

// 以左相机为基准，根据双目检测到的目标在左右图像中的位置差，计算目标的深度信息，并保存在ScoreGate结构体中
bool ImageSolver::get_score_gate_depth()
{
    // 在右图像的检测结果列表中找到与左图像当前检测结果时间戳最接近的一个， 并设定一个时间戳阈值（例如0.1秒）来判断是否匹配成功
    double timestamp_threshold = 0.2; // 200ms
    ScoreGate* matched_right_gate = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();
    for (auto& right_gate : right_score_gates_)
    {
        double time_diff = std::abs(right_gate.timestamp - left_cuda_detector_.score_gate_.timestamp);
        if (time_diff < min_time_diff && time_diff < timestamp_threshold)
        {
            min_time_diff = time_diff;
            matched_right_gate = &right_gate;
        }
    }
    if (matched_right_gate != nullptr)
    {
        float disparity = left_cuda_detector_.score_gate_.center.x - matched_right_gate->center.x;
        float fx = rectified_intrinsic_l_3x3_cv_.at<double>(0, 0); // 左相机的焦距
        if (disparity > 0) // 避免除以零或负数
        {
            double depth = (fx * stereo_baseline_) / disparity;
            target_score_gate_.depth = depth;
            ROS_INFO("Estimated depth to score gate: %.2f meters", depth);
            return true;
        }
        else
        {
            ROS_WARN("Invalid disparity: %.2f. Cannot compute depth.", disparity);
            target_score_gate_.depth = -1.0; // 表示深度不可用
            return false;
        }
    }
    else
    {
        target_score_gate_.depth = -1.0; // 表示深度不可用
        return false;
    }
}

bool ImageSolver::get_score_gate_map_point(const std_msgs::Header& img_header)
{
    if (target_score_gate_.depth <= 0.0)
    {
        return false;
    }

    const double fx = rectified_intrinsic_l_3x3_cv_.at<double>(0, 0);
    const double fy = rectified_intrinsic_l_3x3_cv_.at<double>(1, 1);
    const double cx = rectified_intrinsic_l_3x3_cv_.at<double>(0, 2);
    const double cy = rectified_intrinsic_l_3x3_cv_.at<double>(1, 2);

    const double u = target_score_gate_.camera_pixel.x;
    const double v = target_score_gate_.camera_pixel.y;
    const double z = target_score_gate_.depth;

    // Pinhole camera projection inverse: pixel + depth -> 3D point in camera frame.
    geometry_msgs::PointStamped point_camera;
    point_camera.header.stamp = img_header.stamp;
    point_camera.header.frame_id = "front_left_camera_optical_link";
    point_camera.point.x = (u - cx) * z / fx;
    point_camera.point.y = (v - cy) * z / fy;
    point_camera.point.z = z;

    geometry_msgs::PointStamped point_map;
    try
    {
        point_map = tf_buffer_->transform(point_camera, target_frame_, ros::Duration(0.05));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "Transform score gate point %s -> %s failed: %s",
                          point_camera.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
        return false;
    }

    target_score_gate_.map_point = cv::Point3f(point_map.point.x, point_map.point.y, point_map.point.z);

    geometry_msgs::Point observed_point;
    observed_point.x = point_map.point.x;
    observed_point.y = point_map.point.y;
    observed_point.z = point_map.point.z;
    const bool accepted_to_path = path_planner_.add_observation(observed_point);

    if (path_planner_.has_target())
    {
        geometry_msgs::Point current_target = path_planner_.current_target();
        // 发布当前路径目标点
        geometry_msgs::PointStamped target_point_msg;
        target_point_msg.header.stamp = ros::Time::now();
        target_point_msg.header.frame_id = target_frame_;
        target_point_msg.point.x = current_target.x;
        target_point_msg.point.y = current_target.y;
        target_point_msg.point.z = current_target.z;
        target_point_pub_.publish(target_point_msg);
    }

    if (accepted_to_path)
    {
        ROS_INFO_THROTTLE(1.0, "PathPlanner accepted point, current path size=%zu", path_planner_.target_path_.size());
    }

    return true;
}

void ImageSolver::publish_score_gate_marker(const ros::Time& stamp)
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = target_frame_;
    marker.ns = "score_gate_target";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = target_score_gate_.map_point.x;
    marker.pose.position.y = target_score_gate_.map_point.y;
    marker.pose.position.z = target_score_gate_.map_point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.9;
    marker.lifetime = ros::Duration(0.0);
    marker_array.markers.push_back(marker);

    visualization_msgs::Marker text;
    text.header = marker.header;
    text.ns = "score_gate_target_text";
    text.id = 1;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.x = target_score_gate_.map_point.x;
    text.pose.position.y = target_score_gate_.map_point.y;
    text.pose.position.z = target_score_gate_.map_point.z + 0.6;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.35;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 0.2;
    text.color.a = 1.0;
    text.text = "score_gate_target";
    text.lifetime = ros::Duration(0.0);
    marker_array.markers.push_back(text);

    score_gate_marker_pub_.publish(marker_array);
    ROS_INFO_THROTTLE(1.0,
                      "Published score gate marker in frame=%s at [%.2f, %.2f, %.2f]",
                      target_frame_.c_str(),
                      target_score_gate_.map_point.x,
                      target_score_gate_.map_point.y,
                      target_score_gate_.map_point.z);
}

#endif