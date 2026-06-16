#ifndef CUDA_DETECTOR_HPP
#define CUDA_DETECTOR_HPP

// 神经网络相关
#define    USE_CUDA
#define    RET_OK nullptr

#ifdef _WIN32
#include <Windows.h>
#include <direct.h>
#include <io.h>
#endif

#include <string>
#include <vector>
#include <cstdio>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc/imgproc_c.h"
#include <opencv2/imgproc/types_c.h>
#include "onnxruntime_cxx_api.h"

#ifdef USE_CUDA
#include <cuda_fp16.h>
#endif

typedef struct _DL_INIT_PARAM
{
    std::string modelPath;
    std::vector<int> imgSize = { 480, 480 };
    float rectConfidenceThreshold = 0.6;
    bool cudaEnable = true;
    int logSeverityLevel = 3;
    int intraOpNumThreads = 1;
} DL_INIT_PARAM;

struct ScoreGate
{
    bool is_found = false;
    cv::Point2f center;
    cv::Rect rect; // 包围盒
    std::vector<cv::Point2f> four_points; // 四个关键点坐标，顺序为左上、 左下、 右下、 右上
    float confidence; // 置信度
    // 时间戳
    double timestamp = 0.0;
    double depth; // 深度信息 -- 相对于相机的距离
    cv::Point2f camera_pixel; // 目标在图像中的像素坐标
    cv::Point3f map_point; // 转换到NED坐标系下的三维坐标
    double gate_yaw_map = std::numeric_limits<double>::quiet_NaN(); // 门平面法线在world坐标系的偏航角(rad)，NaN表示未计算
};

class CudaDetector
{
public:
    CudaDetector();
    ~CudaDetector();

    const char* CreateSession(DL_INIT_PARAM& iParams);
    void infer(const cv::Mat &input);

    Ort::Env env;
    Ort::Session* session;
    bool cudaEnable;
    Ort::RunOptions options;
    std::vector<const char*> inputNodeNames;
    std::vector<const char*> outputNodeNames;

    std::vector<int> imgSize;
    float rectConfidenceThreshold;
    float resizeScales;//letterbox scale

    const float IMAGE_WIDTH_ = 480;
    const float IMAGE_HEIGHT_ = 480;
    const float CONFIDENCE_THRESHOLD_ = 0.6;
    const float SCORE_THRESHOLD_ = 0.6;
    const std::vector<std::string> class_names_ = {"score_gate"};
    ScoreGate score_gate_;
};

#endif // CUDA_DETECTOR_HPP