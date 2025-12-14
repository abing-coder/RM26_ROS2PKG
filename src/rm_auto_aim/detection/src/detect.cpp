#include "detect.hpp"

namespace detection
{

Detect::Detect(const std::string& model_path,
               int binary_threshold,
               const TraditionalDetector::LightParams& l_params,
               const TraditionalDetector::ArmorParams& a_params)
    : model_path_copy_(model_path),
      yolo_detector_(model_path_copy_, false),
      traditional_detector_(binary_threshold, l_params, a_params)
{
}

std::vector<ArmorData> Detect::detect(const cv::Mat& image)
{
    if (image.empty()) {
        return {};
    }

    // 先进行 YOLO 检测
    const auto& yolo_armors = yolo_detector_.detect(image);

    // 传统检测器根据 YOLO 结果进行融合/回退
    return traditional_detector_.detect(image, yolo_armors);
}

}  // namespace detection
