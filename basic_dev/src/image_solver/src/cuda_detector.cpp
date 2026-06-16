#include "cuda_detector.hpp"
#include <regex>

#define benchmark

// cv::Mat letterbox(const cv::Mat &input)
// {
//     int col = input.cols;
//     int row = input.rows;
//     int _max = std::max(col, row);
//     cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
//     input.copyTo(result(cv::Rect(0, 0, col, row)));
//     return result;
// }

cv::Mat letterbox(const cv::Mat &input, cv::Size target_size, float& ratio, int& dw, int& dh)
{
    ratio = std::min((float)target_size.width / input.cols, (float)target_size.height / input.rows);
    int new_w = std::round(input.cols * ratio);
    int new_h = std::round(input.rows * ratio);

    dw = (target_size.width - new_w) / 2;
    dh = (target_size.height - new_h) / 2;

    cv::Mat resized;
    cv::resize(input, resized, cv::Size(new_w, new_h));
    
    // 使用标准YOLO的114灰色填充
    cv::Mat result(target_size.height, target_size.width, CV_8UC3, cv::Scalar(114, 114, 114)); 
    resized.copyTo(result(cv::Rect(dw, dh, new_w, new_h)));
    return result;
}

CudaDetector::CudaDetector()
{
    std::cout << "CudaDetector constructed." << std::endl;
}

CudaDetector::~CudaDetector()
{
    std::cout << "CudaDetector destructed." << std::endl;
    delete session;
}

const char* CudaDetector::CreateSession(DL_INIT_PARAM& iParams)
{
    const char* Ret = RET_OK;
    std::regex pattern("[\u4e00-\u9fa5]");
    bool result = std::regex_search(iParams.modelPath, pattern);
    if (result)
    {
        Ret = "[YOLO_V26]:Your model path is error.Change your model path without chinese characters.";
        std::cout << Ret << std::endl;
        return Ret;
    }
    try
    {
        rectConfidenceThreshold = iParams.rectConfidenceThreshold;
        imgSize = iParams.imgSize;
        cudaEnable = iParams.cudaEnable;
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "Yolo");
        Ort::SessionOptions sessionOption;
        if (iParams.cudaEnable)
        {
            OrtCUDAProviderOptions cudaOption;
            cudaOption.device_id = 0;
            sessionOption.AppendExecutionProvider_CUDA(cudaOption);
        }
        sessionOption.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        sessionOption.SetIntraOpNumThreads(iParams.intraOpNumThreads);
        sessionOption.SetLogSeverityLevel(iParams.logSeverityLevel);

#ifdef _WIN32
        int ModelPathSize = MultiByteToWideChar(CP_UTF8, 0, iParams.modelPath.c_str(), static_cast<int>(iParams.modelPath.length()), nullptr, 0);
        wchar_t* wide_cstr = new wchar_t[ModelPathSize + 1];
        MultiByteToWideChar(CP_UTF8, 0, iParams.modelPath.c_str(), static_cast<int>(iParams.modelPath.length()), wide_cstr, ModelPathSize);
        wide_cstr[ModelPathSize] = L'\0';
        const wchar_t* modelPath = wide_cstr;
#else
        const char* modelPath = iParams.modelPath.c_str();
#endif // _WIN32

        session = new Ort::Session(env, modelPath, sessionOption);
        Ort::AllocatorWithDefaultOptions allocator;
        size_t inputNodesNum = session->GetInputCount();
        for (size_t i = 0; i < inputNodesNum; i++)
        {
            Ort::AllocatedStringPtr input_node_name = session->GetInputNameAllocated(i, allocator);
            char* temp_buf = new char[50];
            strcpy(temp_buf, input_node_name.get());
            inputNodeNames.push_back(temp_buf);
        }
        size_t OutputNodesNum = session->GetOutputCount();
        for (size_t i = 0; i < OutputNodesNum; i++)
        {
            Ort::AllocatedStringPtr output_node_name = session->GetOutputNameAllocated(i, allocator);
            char* temp_buf = new char[10];
            strcpy(temp_buf, output_node_name.get());
            outputNodeNames.push_back(temp_buf);
        }
        options = Ort::RunOptions{ nullptr };
        //WarmUpSession();
        return RET_OK;
    }
    catch (const std::exception& e)
    {
        const char* str1 = "[YOLO_V26]:";
        const char* str2 = e.what();
        std::string result = std::string(str1) + std::string(str2);
        char* merged = new char[result.length() + 1];
        std::strcpy(merged, result.c_str());
        std::cout << merged << std::endl;
        delete[] merged;
        return "[YOLO_V26]:Create session failed.";
    }

}

void CudaDetector::infer(const cv::Mat &input)
{
    // cv::Mat blob_img;
    // cv::Mat letterbox_img = letterbox(input);
    // cv::dnn::blobFromImage(letterbox_img, blob_img, 1.0 / 255.0, cv::Size(IMAGE_WIDTH_, IMAGE_HEIGHT_), cv::Scalar(), true, false);
    // // 获取 blob 的数据指针
	// float* blob_data = reinterpret_cast<float*>(blob_img.data);
    // // 计算 blob 的大小
	// size_t blob_size = blob_img.total() * blob_img.channels();
	// float x_scale = (float)letterbox_img.cols / IMAGE_WIDTH_;
	// float y_scale = (float)letterbox_img.rows / IMAGE_HEIGHT_;

    float ratio;
    int dw, dh;
    cv::Mat blob_img;
    cv::Mat letterbox_img = letterbox(input, cv::Size(IMAGE_WIDTH_, IMAGE_HEIGHT_), ratio, dw, dh);
    
    // 此时 letterbox_img 已经是 480x480 的了，不需要让 blobFromImage 帮你 resize 了！
    cv::dnn::blobFromImage(letterbox_img, blob_img, 1.0 / 255.0, cv::Size(IMAGE_WIDTH_, IMAGE_HEIGHT_), cv::Scalar(), true, false);
    // 获取 blob 的数据指针
	float* blob_data = reinterpret_cast<float*>(blob_img.data);
    // 计算 blob 的大小
	size_t blob_size = blob_img.total() * blob_img.channels();

    std::vector<int64_t> inputNodeDims = { 1, 3, imgSize.at(0), imgSize.at(1) };
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU), blob_data, blob_size,
        inputNodeDims.data(), inputNodeDims.size());
    
    //处理推理数据begin
    auto outputTensor = session->Run(options, inputNodeNames.data(), &inputTensor, 1, outputNodeNames.data(),outputNodeNames.size());
    float* pdata = outputTensor.front().GetTensorMutableData<float>();
    
    // YoloV26 pose 输出纬度 1x300x18
    int dimensions = outputTensor.front().GetTypeInfo().GetTensorTypeAndShapeInfo().GetShape().at(2);
    int num_anchors = outputTensor.front().GetTypeInfo().GetTensorTypeAndShapeInfo().GetShape().at(1);
    int start_kpt_index = dimensions - 12; // 倒数12维是四个关键点的 x, y, conf

	cv::Mat output_buffer(num_anchors, dimensions, CV_32FC1, pdata);

    // 输出结果进行调试
    std::cout << "output buffer size: " << output_buffer.size() << std::endl;
    //std::cout << "x_scale: " << x_scale << ", y_scale: " << y_scale << std::endl;
    // 输出关键点坐标和置信度进行调试
    for (int i = 0; i < output_buffer.rows; i++)
    {        float maxNumberScore = output_buffer.at<float>(i, 4); // index 4 通常为置信度
        if(maxNumberScore < rectConfidenceThreshold)
        {            continue;
        }
        std::cout << "Detection " << i << ": x1=" << output_buffer.at<float>(i, 0) << ", y1=" << output_buffer.at<float>(i, 1) << ", x2=" << output_buffer.at<float>(i, 2) <<
            ", y2=" << output_buffer.at<float>(i, 3) << ", confidence=" << maxNumberScore << std::endl;
        for (int k = 0; k < 4; ++k)        {
            float kpt_x = output_buffer.at<float>(i, start_kpt_index + k * 3 + 0); // 乘以缩放系数还原到原图坐标
            float kpt_y = output_buffer.at<float>(i, start_kpt_index + k * 3 + 1); // 乘以缩放系数还原到原图坐标
            float kpt_conf = output_buffer.at<float>(i, start_kpt_index + k * 3 + 2); // 关键点置信度
            std::cout << "  Keypoint " << k << ": x=" << kpt_x << ", y=" << kpt_y << ", confidence=" << kpt_conf << std::endl;
        }
    }   

    // 若有多个结果则根据最大面积来筛选检测结果
    double max_area = -1000.0;
    for(int i = 0; i < output_buffer.rows; i++)
	{
		float maxNumberScore = output_buffer.at<float>(i, 4); // index 4 通常为置信度
        if(maxNumberScore < CONFIDENCE_THRESHOLD_)
        {
            continue;
        }

        // 如果输出是 x1, y1, x2, y2 格式 (常见的导出带 NMS 的模型格式)
        float x1_model = output_buffer.at<float>(i, 0); // x1
        float y1_model = output_buffer.at<float>(i, 1); // y1
        float x2_model = output_buffer.at<float>(i, 2); // x2
        float y2_model = output_buffer.at<float>(i, 3); // y2
        
        // float x1 = x1_model * x_scale;
        // float y1 = y1_model * y_scale;
        // float x2 = x2_model * x_scale;
        // float y2 = y2_model * y_scale;
        float x1 = (x1_model - dw) / ratio; // 先减去 letterbox 的 padding，再除以缩放系数
        float y1 = (y1_model - dh) / ratio; // 先减去 letterbox 的 padding，再除以缩放系数
        float x2 = (x2_model - dw) / ratio; // 先减去 letterbox 的 padding，再除以缩放系数
        float y2 = (y2_model - dh) / ratio; // 先减去 letterbox 的 padding，再除以缩放系数
        float rect_w = x2 - x1;
        float rect_h = y2 - y1;
        cv::Rect box(x1, y1, rect_w, rect_h);
        
        ScoreGate score_gate;
        score_gate.is_found = true;
        // 中心点也应是矩形框的中心
        score_gate.center = cv::Point2f(x1 + rect_w / 2.0f, y1 + rect_h / 2.0f);
        score_gate.rect = box;
        score_gate.confidence = maxNumberScore;
        
        // 提取 keypoints
        for (int k = 0; k < 4; ++k)
        {
            float kpt_x_model = output_buffer.at<float>(i, start_kpt_index + k * 3 + 0); // 乘以缩放系数还原到原图坐标
            float kpt_y_model = output_buffer.at<float>(i, start_kpt_index + k * 3 + 1); // 乘以缩放系数还原到原图坐标
            float kpt_x = (kpt_x_model - dw) / ratio; // 先减去 letterbox 的 padding，再除以缩放系数
            float kpt_y = (kpt_y_model - dh) / ratio; // 先减去 letterbox 的 padding，再除以缩放系数
            float kpt_conf = output_buffer.at<float>(i, start_kpt_index + k * 3 + 2); // 关键点置信度
            // 如果需要使用 keypoint 的置信度，这行是: output_buffer.at<float>(i, start_kpt_index + k * 3 + 2);
            score_gate.four_points.push_back(cv::Point2f(kpt_x, kpt_y));
        }
        
        double area = rect_w * rect_h;
        if (area > max_area)        {
            max_area = area;
            score_gate_ = score_gate; // score_gate_ 是 CudaDetector 类的成员变量，存储最终的检测结果
        }
    }
}
