#include "yolo_detection.hpp"
#include <iomanip>
#include <thread>

using namespace detection;

// 静态成员变量定义（共享变量，其他节点可能使用）
int detection::DetectionArmor::detect_color = 0;  // 0: 红色，1: 蓝色

// 常量定义
constexpr int INPUT_SIZE = 640;
constexpr float CONFIDENCE_THRESHOLD = 0.6f;
constexpr float NMS_THRESHOLD = 0.4f;
constexpr int INFERENCE_THREADS = 14;

DetectionArmor::DetectionArmor(std::string& model_path, bool if_count_time, std::string video_path)
    : m_if_count_time(if_count_time), fps(0.0), m_profiler("openvino_performance.log")
{
    // 如果提供了视频路径，则初始化视频捕获
    if (!video_path.empty()) {
        m_cap = cv::VideoCapture(video_path);
    }
    
    // 统一使用优化的配置参数
    ov::AnyMap config = {
        {ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT)},  // 吞吐量模式，提高帧率
        {ov::inference_num_threads(INFERENCE_THREADS)},                       // 动态线程配置
        {ov::num_streams(1)},                                                 // 多流推理，提高并行性
        {ov::hint::enable_cpu_pinning(true)},                                 // 启用CPU固定，减少线程迁移开销
        {ov::hint::execution_mode(ov::hint::ExecutionMode::PERFORMANCE)},     // 性能执行模式
        {ov::log::level(ov::log::Level::WARNING)}                             // 减少日志输出，提高性能
    };

    auto network = m_core.read_model(model_path);
    m_compiled = m_core.compile_model(network, "CPU", config);
    m_infer_request = m_compiled.create_infer_request();
    m_input_port = m_compiled.input();

    m_input_blob = cv::Mat(INPUT_SIZE, INPUT_SIZE, CV_32F, cv::Scalar(0));  // 初始化输入blob
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
        cv::resize(m_frame, m_img, cv::Size(INPUT_SIZE, INPUT_SIZE));
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

void DetectionArmor::infer()
{
    Timer t(m_counter);

    // 归一化
    m_input_blob = cv::dnn::blobFromImage(
        m_img, 
        1 / 255.0
    );

    // 固定八股
    ov::Tensor input_tensor(m_input_port.get_element_type(), m_input_port.get_shape(), m_input_blob.data);
    m_infer_request.set_input_tensor(input_tensor);
    m_infer_request.infer();    
    auto outputs = m_compiled.outputs();
    ov::Tensor output = m_infer_request.get_tensor(outputs[0]);
    ov::Shape output_shape = output.get_shape();
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, output.data());
    
    // 存储临时结果
    std::vector<cv::Rect> boxes;
    std::vector<int> num_class;
    std::vector<int> color_class;
    std::vector<float> confidences;     
    std::vector<int> indices;

    // 临时的四点
    std::vector<std::vector<cv::Point>> fourPointModel;

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

        // 检测颜色
        if ((detect_color == 0 && color_id.x == 1) || (detect_color == 1 && color_id.x == 0)) continue;
        
        // 获取第一个输出向量的指针
        float* f_ptr = output_buffer.ptr<float>(i);

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

        // 创建rect
        cv::Rect rect(
            f_ptr[0], // x
            f_ptr[1], // y
            f_ptr[4] - f_ptr[0], // width
            f_ptr[5] - f_ptr[1]  // height
        );
        
        // 加入
        boxes.push_back(rect);
        confidences.push_back(confidence);
    }

    // 非极大值抑制
    cv::dnn::NMSBoxes(
        boxes,                // 输入边界框（std::vector<cv::Rect>）
        confidences,          // 输入置信度（std::vector<float>）
        CONFIDENCE_THRESHOLD, // 得分阈值
        NMS_THRESHOLD,        // NMS 阈值
        indices               // 输出索引（必须传入引用）
    );

    // 保留最终的数据
    for (int valid_index = 0; valid_index < indices.size(); ++valid_index) 
    {
        ArmorData d;

        d.p1 = fourPointModel[indices[valid_index]][0];
        d.p2 = fourPointModel[indices[valid_index]][1];
        d.p3 = fourPointModel[indices[valid_index]][2];
        d.p4 = fourPointModel[indices[valid_index]][3];

        d.center_point.x = (d.p1.x + d.p2.x + d.p3.x + d.p4.x) / 4;
        d.center_point.y = (d.p1.y + d.p2.y + d.p3.y + d.p4.y) / 4;

        d.ID = num_class[indices[valid_index]];

        int color = color_class[indices[valid_index]];
        if (color == 0){ d.color = Color::RED; }
        else if (color == 1){ d.color = Color::BLUE; }
        else { d.color = Color::NONE; }

        m_armors_datas.push_back(d);
    }
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
    cv::resize(input_image, m_img, cv::Size(INPUT_SIZE, INPUT_SIZE));

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
        std::cout << getdata().size() << std::endl;
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
