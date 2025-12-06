#include "yolo_detection.hpp"
#include <algorithm>
#include <iomanip>
#include <thread>
#include <cmath>

// 来自 traditional_detector.cpp，用于在传统视觉流程前同步阵营颜色
extern int detect_color;

using namespace detection;

namespace {

// 将传统视觉的 Armor 结果转换为通用 ArmorData
ArmorData convertTraditionalArmor(const Armor& armor, int current_color)
{
    ArmorData data;

    cv::Point2f left_pts[4];
    cv::Point2f right_pts[4];
    armor.left_light.points(left_pts);
    armor.right_light.points(right_pts);

    float min_x = std::min({left_pts[0].x, left_pts[1].x, left_pts[2].x, left_pts[3].x,
                            right_pts[0].x, right_pts[1].x, right_pts[2].x, right_pts[3].x});
    float min_y = std::min({left_pts[0].y, left_pts[1].y, left_pts[2].y, left_pts[3].y,
                            right_pts[0].y, right_pts[1].y, right_pts[2].y, right_pts[3].y});
    float max_x = std::max({left_pts[0].x, left_pts[1].x, left_pts[2].x, left_pts[3].x,
                            right_pts[0].x, right_pts[1].x, right_pts[2].x, right_pts[3].x});
    float max_y = std::max({left_pts[0].y, left_pts[1].y, left_pts[2].y, left_pts[3].y,
                            right_pts[0].y, right_pts[1].y, right_pts[2].y, right_pts[3].y});

    data.p1 = cv::Point(static_cast<int>(min_x), static_cast<int>(min_y));
    data.p2 = cv::Point(static_cast<int>(max_x), static_cast<int>(min_y));
    data.p3 = cv::Point(static_cast<int>(max_x), static_cast<int>(max_y));
    data.p4 = cv::Point(static_cast<int>(min_x), static_cast<int>(max_y));

    data.center_point = cv::Point(static_cast<int>(armor.center.x), static_cast<int>(armor.center.y));
    data.ID = 0;  // 传统视觉当前未识别数字，设为0占位
    data.color = (current_color == 0) ? Color::RED : Color::BLUE;

    return data;
}

// 简单的中心点去重，防止与 YOLO 结果重复返回
bool isDuplicate(const std::vector<ArmorData>& existing, const cv::Point2f& center, double threshold = 15.0)
{
    for (const auto& item : existing) {
        if (cv::norm(center - cv::Point2f(static_cast<float>(item.center_point.x),
                                          static_cast<float>(item.center_point.y))) < threshold) {
            return true;
        }
    }
    return false;
}

}  // namespace

// 静态成员变量定义（共享变量，其他节点可能使用）
int detection::DetectionArmor::detect_color = 0;  // 0: 红色，1: 蓝色

// 常量定义
constexpr int INPUT_SIZE = 640;
constexpr float CONFIDENCE_THRESHOLD = 0.6f;
constexpr float NMS_THRESHOLD = 0.4f;
constexpr int INFERENCE_THREADS = 14;

DetectionArmor::DetectionArmor(std::string& model_path, bool if_count_time, std::string video_path)
    : m_inference_engine(model_path, INPUT_SIZE, INFERENCE_THREADS),
      m_if_count_time(if_count_time), fps(0.0), m_profiler("openvino_performance.log")
{
    // 初始化传统视觉检测器（使用默认参数）
    Detector::LightParams light_params;
    Detector::ArmorParams armor_params;
    int binary_threshold = 100;  // 默认二值化阈值
    m_traditional_detector = std::make_unique<Detector>(binary_threshold, light_params, armor_params);

    // 如果提供了视频路径，则初始化视频捕获
    if (!video_path.empty()) {
        m_cap = cv::VideoCapture(video_path);
    }
}

/**
 * @brief 计算目标点的角度（偏航和俯仰）
 * @param point 目标点坐标
 * @param yaw 输出的偏航角
 * @param pitch 输出的俯仰角
 * @param cx 相机光心x坐标
 * @param cy 相机光心y坐标
 * @param fx 相机焦距x
 * @param fy 相机焦距y
 */
void getAngle(cv::Point2f point, float &yaw, float &pitch, 
              const double &cx, const double &cy, 
              const double &fx, const double &fy)
{
    float x_pos = (point.x - cx) / fx;  // 光心点归一化
    float y_pos = (point.y - cy) / fy; 
    float z_pos = 1.0f;
    
    // 在相机系下枪管和相机的偏移补偿，单位m
    x_pos -= GUN_CAM_DISTANCE_X;
    y_pos -= GUN_CAM_DISTANCE_Y;
    z_pos -= GUN_CAM_DISTANCE_Z;
    
    // 转角转换
    float tan_pitch = y_pos / std::sqrt(x_pos * x_pos + z_pos * z_pos);
    float tan_yaw = x_pos / z_pos;
    pitch = std::atan(tan_pitch);
    yaw = std::atan(tan_yaw);
}

DetectionArmor::~DetectionArmor() 
{
    // std::cout << "quit from detection" << std::endl;
    clearHeap();
}

void DetectionArmor::clearHeap()
{
    m_cap.release();
    cv::destroyAllWindows();
}

/**
 * @brief 获取图像中的感兴趣区域(ROI)
 * @param image 输入图像
 * @param points 四个角点坐标
 * @param ROI 输出的感兴趣区域
 */
void get_roi(cv::Mat& image, std::vector<cv::Point>& points, cv::Mat& ROI)
{ 
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    
    cv::Point lt = points[0];  // 左上角
    cv::Point rb = points[2];  // 右下角
    cv::Point lb = points[1];  // 左下角
    cv::Point rt = points[3];  // 右上角
    
    cv::polylines(image, std::vector<cv::Point>{lt, rt, rb, lb}, true, cv::Scalar(255, 0, 0), 2);
    std::vector<cv::Point> roi_points = {lt, rt, rb, lb};
    cv::fillConvexPoly(mask, roi_points, cv::Scalar(255, 0, 0));
    cv::bitwise_and(image, image, ROI, mask);
}

void DetectionArmor::drawObject(cv::Mat& image, std::vector<ArmorData>& datas)
{
    // 绘制装甲板的边界框
    // 计算并绘制光心点（图像中心）
    cv::Point optical_center(image.cols / 2, image.rows / 2);

    for (ArmorData& d : datas)
    {
        // 使用YOLO检测的中心点，不要覆盖它！
        // d.center_point 已经在 infer() 中计算好了

        d.optical_center = optical_center;
        d.delta_x = (d.center_point.x - d.optical_center.x) + GUN_CAM_DISTANCE_X;
        d.delta_y = (d.optical_center.y - d.center_point.y) + GUN_CAM_DISTANCE_Y;

        // 绘制四个角点
        std::vector<cv::Point> armor_points = {d.p1, d.p2, d.p3, d.p4};
        cv::polylines(image, armor_points, true, cv::Scalar(0, 255, 0), 2);

        // 绘制中心点
        cv::circle(image, d.center_point, 3, cv::Scalar(0, 0, 255), -1);

        // 调试信息
        cv::circle(image, optical_center, 5, cv::Scalar(255, 0, 0), -1);
        std::string delta_text = "dx:" + std::to_string(static_cast<int>(d.delta_x)) +
                                 " dy:" + std::to_string(static_cast<int>(d.delta_y));
        cv::putText(image, delta_text, d.p1, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 0), 1);

        std::string id_text = "ID:" + std::to_string(d.ID);
        cv::putText(image, id_text, cv::Point(d.p1.x, d.p1.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
    }

    cv::putText(image, "fps:" + std::to_string(static_cast<int>(fps)), cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

    cv::imshow("Detection", image);
}

inline double DetectionArmor::sigmoid(double x) 
{
    return (1 / (1 + std::exp(-x)));
}


void DetectionArmor::run()
{
    size_t frame_count = 0;

    while (true)
    {
        m_armors_datas.clear();  // 清空当前帧的装甲板数据
        m_cap >> m_frame;
        m_img = m_frame;  // 直接使用原始图像，PPP自动处理BGR→RGB、u8→f32、归一化
        auto start = std::chrono::high_resolution_clock::now();

        // 推理
        infer();

        frame_count += 1;
        if (frame_count == static_cast<size_t>(m_cap.get(cv::CAP_PROP_FRAME_COUNT)))
        {
            frame_count = 0;
            m_cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        
        // fps 计算
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        if (duration > 0) {
            fps = 1000000.0 / duration;  // 使用微秒计算，更精确
        } else {
            fps = 0.0;  // 避免除以0
        }
        
        std::cout << "FPS: " << fps << std::endl;
        
        #ifdef TEST_MODE
        showImage();
        #endif

        if (cv::waitKey(1) == 27)  // 按下ESC键退出
        {
            clearHeap();
            break;
        }
    }
}

void DetectionArmor::parseDetections(const cv::Mat& output_buffer,
                                    std::vector<cv::Rect>& boxes,
                                    std::vector<int>& num_class,
                                    std::vector<int>& color_class,
                                    std::vector<float>& confidences,
                                    std::vector<std::vector<cv::Point>>& fourPointModel)
{
    // 遍历所有的网络输出
    for (int i = 0; i < output_buffer.rows; ++i)
    {
        // 获取当前的置信度
        float confidence = output_buffer.at<float>(i, 8);

        // 激活到0-1之间
        confidence = sigmoid(confidence);

        // 过滤低置信度检测框
        if (confidence < CONFIDENCE_THRESHOLD) continue;

        // 检查出颜色和数字的类别
        cv::Mat color_scores = output_buffer.row(i).colRange(9, 13);   // 颜色概率（红/蓝等）
        cv::Mat classes_scores = output_buffer.row(i).colRange(13, 22);// 类别概率
        cv::Point class_id, color_id;
        cv::minMaxLoc(classes_scores, nullptr, nullptr, nullptr, &class_id);
        cv::minMaxLoc(color_scores, nullptr, nullptr, nullptr, &color_id);

        // 加入预测出来的数字和颜色
        num_class.push_back(class_id.x);
        color_class.push_back(color_id.x);

        // 检测颜色过滤
        if ((detect_color == 0 && color_id.x == 1) || (detect_color == 1 && color_id.x == 0)) continue;

        // 获取第一个输出向量的指针
        const float* f_ptr = output_buffer.ptr<float>(i);

        // 提取四个角点
        std::vector<cv::Point> box_point(4);
        box_point[0].x = f_ptr[0];
        box_point[0].y = f_ptr[1];
        box_point[1].x = f_ptr[2];
        box_point[1].y = f_ptr[3];
        box_point[2].x = f_ptr[4];
        box_point[2].y = f_ptr[5];
        box_point[3].x = f_ptr[6];
        box_point[3].y = f_ptr[7];

        fourPointModel.push_back(box_point);

        // 创建边界矩形
        cv::Rect rect(
            f_ptr[0], // x
            f_ptr[1], // y
            f_ptr[4] - f_ptr[0], // width
            f_ptr[5] - f_ptr[1]  // height
        );

        boxes.push_back(rect);
        confidences.push_back(confidence);
    }
}

void DetectionArmor::applyNMS(const std::vector<cv::Rect>& boxes,
                             const std::vector<float>& confidences,
                             std::vector<int>& indices)
{
    cv::dnn::NMSBoxes(
        boxes,                // 输入边界框
        confidences,          // 输入置信度
        CONFIDENCE_THRESHOLD, // 得分阈值
        NMS_THRESHOLD,        // NMS 阈值
        indices               // 输出索引
    );
}

void DetectionArmor::buildArmorData(const std::vector<int>& indices,
                                    const std::vector<std::vector<cv::Point>>& fourPointModel,
                                    const std::vector<int>& num_class,
                                    const std::vector<int>& color_class)
{
    // 保留最终的数据
    for (int valid_index = 0; valid_index < indices.size(); ++valid_index)
    {
        ArmorData d;

        // 设置四个角点
        d.p1 = fourPointModel[indices[valid_index]][0];
        d.p2 = fourPointModel[indices[valid_index]][1];
        d.p3 = fourPointModel[indices[valid_index]][2];
        d.p4 = fourPointModel[indices[valid_index]][3];

        // 计算中心点
        d.center_point.x = (d.p1.x + d.p2.x + d.p3.x + d.p4.x) / 4;
        d.center_point.y = (d.p1.y + d.p2.y + d.p3.y + d.p4.y) / 4;

        // 设置ID
        d.ID = num_class[indices[valid_index]];

        // 设置颜色
        int color = color_class[indices[valid_index]];
        if (color == 0) {
            d.color = Color::RED;
        } else if (color == 1) {
            d.color = Color::BLUE;
        } else {
            d.color = Color::NONE;
        }

        m_armors_datas.push_back(d);
    }
}

void DetectionArmor::infer()
{
    Timer t(m_counter);

    // 1. 使用推理引擎执行推理
    cv::Mat output_buffer = m_inference_engine.infer(m_img);

    // 2. 存储临时结果
    std::vector<cv::Rect> boxes;
    std::vector<int> num_class;
    std::vector<int> color_class;
    std::vector<float> confidences;
    std::vector<int> indices;
    std::vector<std::vector<cv::Point>> fourPointModel;

    // 3. 解析检测结果
    parseDetections(output_buffer, boxes, num_class, color_class, confidences, fourPointModel);

    // 4. 应用非极大值抑制
    applyNMS(boxes, confidences, indices);

    // 5. 构建装甲板数据
    buildArmorData(indices, fourPointModel, num_class, color_class);


    std::vector<Armor> traditional_armors = m_traditional_detector->detect(m_img);

}



std::vector<ArmorData>& DetectionArmor::getdata()
{
    return m_armors_datas;  // 返回当前帧的装甲板数据
}

void DetectionArmor::start_detection()
{
    run();
}

void DetectionArmor::start_detection(const cv::Mat& input_image)
{
    if (input_image.empty())
    {
        m_armors_datas.clear();
        return;
    }

    m_armors_datas.clear();
    m_img = input_image;  // 直接使用输入图像，PPP自动处理BGR→RGB、u8→f32、归一化

    auto start = std::chrono::high_resolution_clock::now();
    infer();
    auto end = std::chrono::high_resolution_clock::now();
    
    // fps 计算
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    if (duration > 0) {
        fps = 1000000.0 / duration;  
    } else {
        fps = 0.0;  
    }
    
    drawObject(m_img, m_armors_datas);
}

#ifdef TEST_MODE
void DetectionArmor::showImage()
{
    if (!m_img.empty())
    {
        drawObject(m_img, getdata());
    }
}

void DetectionArmor::format_print_data_test()
{
    std::cout << "armor Num: " << getdata().size() << std::endl;
    for (auto d : getdata())
    {
        // std::cout << "center X: " << d.center_x << " ";
        // std::cout << "center Y: " << d.center_y << std::endl;
    }
}
#endif
