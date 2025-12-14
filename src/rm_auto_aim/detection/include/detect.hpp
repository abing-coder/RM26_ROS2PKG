#ifndef DETECT_HPP_
#define DETECT_HPP_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "yolo_detection.hpp"
#include "traditional_detector.hpp"

namespace detection
{

/**
 * @brief 统一装甲板检测入口，封装 YOLO + 传统视觉流程
 */
class Detect
{
public:
    /**
     * @brief 构造检测器
     * @param model_path OpenVINO 模型路径
     * @param binary_threshold 传统检测二值化阈值（默认 100）
     * @param l_params 传统检测灯条参数
     * @param a_params 传统检测装甲板参数
     */
    Detect(const std::string& model_path,
           int binary_threshold = 100,
           const TraditionalDetector::LightParams& l_params = {},
           const TraditionalDetector::ArmorParams& a_params = {});

    /**
     * @brief 运行检测并返回装甲板结果
     * @param image 输入图像（BGR）
     * @return 检测出的装甲板数据
     */
    std::vector<ArmorData> detect(const cv::Mat& image);

    /**
     * @brief 设置识别颜色（0: 红，1: 蓝）
     */
    void set_detect_color(int color) { DetectionArmor::detect_color = color; }

private:
    std::string model_path_copy_;
    DetectionArmor yolo_detector_;
    TraditionalDetector traditional_detector_;
};

}  // namespace detection

#endif  // DETECT_HPP_
