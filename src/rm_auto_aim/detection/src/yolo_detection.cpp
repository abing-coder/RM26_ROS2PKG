#include "yolo_detection.hpp"
#include <iomanip>
#include <thread>

using namespace detection;

int detection::DetectionArmor::detect_color = 0; // 0: 红色，1: 蓝色

DetectionArmor::DetectionArmor(string& model_path, bool ifcountTime, string video_path)
    : ifCountTime(ifcountTime), fps(0.0), profiler_("openvino_performance.log")
{
    // 如果提供了视频路径，则初始化视频捕获
    if (!video_path.empty()) {
        cap = VideoCapture(video_path);
    }
    
    // 统一使用优化的配置参数
    ov::AnyMap config = {
        {ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT)}, // 吞吐量模式，提高帧率
        {ov::inference_num_threads(14)}, // 动态线程配置
        {ov::num_streams(1)}, // 多流推理，提高并行性
        {ov::hint::enable_cpu_pinning(true)}, // 启用CPU固定，减少线程迁移开销
        {ov::hint::execution_mode(ov::hint::ExecutionMode::PERFORMANCE)}, // 性能执行模式
        {ov::log::level(ov::log::Level::WARNING)} // 减少日志输出，提高性能
    };

    auto network = core.read_model(model_path);
    compiled = core.compile_model(network, "CPU", config);
    infer_request = compiled.create_infer_request();
    input_port = compiled.input();

    input_blob = Mat(640, 640, CV_32F, Scalar(0)); // 初始化输入blob
}

float yaw = 0;
float pitch = 0;
void getAngle(Point2f point, float &yaw, float &pitch, 
                            const double &cx, const double &cy, 
                            const double &fx, const double &fy ){

    float x_pos = (point.x-cx)/fx; // 光心点归一化
    float y_pos = (point.y-cy)/fy; 
    float z_pos = 1;
    // 在相机系下枪管和相机的偏移补偿，单位m
    x_pos -= GUN_CAM_DISTANCE_X;
    y_pos -= GUN_CAM_DISTANCE_Y;
    z_pos -= GUN_CAM_DISTANCE_Z;
    // 转角转换
    float tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
    float tan_yaw = x_pos / z_pos;
    pitch = atan(tan_pitch);
    yaw = atan(tan_yaw);
}

DetectionArmor::~DetectionArmor() 
{
    // std::cout << "quit from detection" << std::endl;
    clearHeap();
}

void DetectionArmor::clearHeap()
{
    cap.release();
    cv::destroyAllWindows();
}

Detector::LightParams l_params; 
Detector::ArmorParams a_params;
Detector detector(80,l_params,a_params); 

void get_roi(Mat& image,vector<Point>& points,Mat& ROI)
{ 
    cv::Mat mask = Mat::zeros(image.size(), CV_8UC1);
    
    Point lt = points[0];  // 左上角
    Point rb = points[2];  // 右下角
    Point lb = points[1];  // 左下角
    Point rt = points[3];  // 右上角
    cv::polylines(image, std::vector<Point>{lt, rt, rb, lb}, true, Scalar(255, 0, 0), 2);
    std::vector<cv::Point> roi_points = {lt, rt, rb, lb};
    cv::fillConvexPoly(mask, roi_points,Scalar(255,0,0));
    cv::bitwise_and(image, image,ROI, mask);
}
void DetectionArmor::drawObject(Mat& image, vector<ArmorData>& datas)
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
        std::vector<Point> armor_points = {d.p1, d.p2, d.p3, d.p4};
        cv::polylines(image, armor_points, true, Scalar(0, 255, 0), 2);

        // 绘制中心点
        cv::circle(image, d.center_point, 3, Scalar(0, 0, 255), -1);

        // 调试信息
        cv::circle(image, optical_center, 5, Scalar(255, 0, 0), -1);
        std::string delta_text = "dx:" + std::to_string(static_cast<int>(d.delta_x)) +
                                 " dy:" + std::to_string(static_cast<int>(d.delta_y));
        cv::putText(image, delta_text, d.p1, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    Scalar(0, 255, 0), 1);

        std::string id_text = "ID:" + std::to_string(d.ID);
        cv::putText(image, id_text, cv::Point(d.p1.x, d.p1.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
    }

    cv::putText(image,"fps:"+to_string(static_cast<int>(fps)),cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX,1, Scalar(0, 255, 0),2);

    cv::imshow("Detection", image);
}

inline double DetectionArmor::sigmoid(double x) 
{
    return (1 / (1 + exp(-x)));
}


void DetectionArmor::run()
{
    size_t frame_count = 0;

    while (1) 
    {
        armorsDatas.clear(); // 清空当前帧的装甲板数据
        cap >> frame;
        resize(frame, img, Size(640, 640));
        auto start = std::chrono::high_resolution_clock::now();
        // 推理
        infer();

        frame_count += 1;
        if (frame_count == int(cap.get(cv::CAP_PROP_FRAME_COUNT)))
        {
            frame_count = 0;
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
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
        showImage();

        if (cv::waitKey(1) == 27)
        {
            clearHeap();
            break;
        } // 按下ESC键退出

    }
}

void DetectionArmor::infer()
{

    Timer t(counter);

    // 归一化
    input_blob = blobFromImage(
        img, 
        1 / 255.0
    );

    // 固定八股
    Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), input_blob.data);
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();    
    auto outputs = compiled.outputs();
    Tensor output = infer_request.get_tensor(outputs[0]);
    ov::Shape output_shape = output.get_shape();
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, output.data());
    
    float conf_threshold = 0.6;   
    float nms_threshold = 0.4;  
    
    // 存储临时结果
    std::vector<Rect> boxes;
    std::vector<int> num_class;
    std::vector<int> color_class;
    std::vector<float> confidences;     
    std::vector<int> indices;

    // 临时的四点
    std::vector<vector<Point>> fourPointModel;

    // 遍历所有的网络输出
    for (int i = 0; i < output_buffer.rows; ++i) 
    {
        // 获取当前的置信度
        float confidence = output_buffer.at<float>(i, 8);

        // 激活到0-1之间
        confidence = sigmoid(confidence);

        // 过滤低置信度检测框
        if (confidence < conf_threshold) continue;  

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

        vector<Point> box_point(4);

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

    // 非什么几把极大值抑制
    cv::dnn::NMSBoxes(
        boxes,                // 输入边界框（std::vector<cv::Rect>）
        confidences,          // 输入置信度（std::vector<float>）
        conf_threshold,       // 得分阈值（如 0.5f）
        nms_threshold,        // NMS 阈值（如 0.4f）
        indices               // 输出索引（必须传入引用）
    );

    // 保留最终的数据
    std::vector<ArmorData> data;
    for (int valid_index = 0; valid_index < indices.size(); ++valid_index) 
    {
        ArmorData d;

        d.p1 = fourPointModel[valid_index][0];
        d.p2 = fourPointModel[valid_index][1];
        d.p3 = fourPointModel[valid_index][2];
        d.p4 = fourPointModel[valid_index][3];

        d.center_point.x = (d.p1.x + d.p2.x + d.p3.x + d.p4.x) / 4;
        d.center_point.y = (d.p1.y + d.p2.y + d.p3.y + d.p4.y) / 4;

        d.ID = num_class[indices[valid_index]];

        int color = color_class[indices[valid_index]];
        if (color == 0){ d.color = Color::RED; }
        else if (color == 1){ d.color = Color::BLUE; }
        else { d.color = Color::NONE; }

        armorsDatas.push_back(d);
    }
}



inline vector<ArmorData>& DetectionArmor::getdata()
{
    return armorsDatas; // 返回当前帧的装甲板数据
}

void DetectionArmor::start_detection()
{
    run();
}

void DetectionArmor::start_detection(const cv::Mat& input_image)
{
    if (input_image.empty())
    {
        armorsDatas.clear();
        return;
    }

    armorsDatas.clear();
    cv::resize(input_image, img, cv::Size(640, 640));

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
    
    drawObject(img, armorsDatas);
}


void __TEST__ DetectionArmor::showImage()
{
    if (!img.empty()) 
    {
        std::cout << getdata().size() << std::endl;
        drawObject(img, getdata());

        //cv::imshow("Detection Armor", img); // 显示图像
        // format_print_data_test();
    }
}

void __TEST__ DetectionArmor::format_print_data_test()
{
    cout << "armor Num: " << getdata().size() << endl;
    for (auto d : getdata())
    {
        // cout << "center X: " << d.center_x << " ";
        // cout << "center Y: " << d.center_y << endl;
    }
}
