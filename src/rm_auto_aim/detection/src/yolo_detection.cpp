#include "yolo_detection.hpp"

using namespace detection;

int detection::DetectionArmor::detect_color = 0;//0: 红色，1: 蓝色

bool setThreadPriority(std::thread& thread, int priority) {
    pthread_t pthread = thread.native_handle();
    
    // 获取当前调度策略
    int policy;
    struct sched_param param;
    if (pthread_getschedparam(pthread, &policy, &param) != 0) {
        std::cerr << "获取线程调度参数失败" << std::endl;
        return false;
    }
    
    // 设置新优先级（macOS 和 Linux 通用）
    // 注意：优先级范围通常为 1-99，值越大优先级越高
    param.sched_priority = priority;
    
    // 应用新参数（使用 SCHED_RR 实时调度策略）
    if (pthread_setschedparam(pthread, SCHED_RR, &param) != 0) {
        std::cerr << "设置线程优先级失败，可能需要 root 权限" << std::endl;
        return false;
    }
    
    return true;
}

DetectionArmor::DetectionArmor(string& model_path, bool ifcountTime, string video_path)
    : ifCountTime(ifcountTime), fps(0.0)
{
    cap = VideoCapture(video_path);

    ov::AnyMap config = {
        {ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)}, // 设置性能模式为延迟优化
        {ov::inference_num_threads(4)},//使用4个线程进行推理
        {ov::num_streams(1)}, // 允许同时执行1个推理流
        {ov::hint::scheduling_core_type(ov::hint::SchedulingCoreType::PCORE_ONLY)}, // 性能核心绑定
        {ov::hint::enable_hyper_threading(false)}, // 关闭超线程
        {ov::hint::enable_cpu_pinning(false)} // 关闭CPU固定
    };

    auto network = core.read_model(model_path);
    compiled = core.compile_model(network, "CPU", config);
    infer_request = compiled.create_infer_request();
    input_port = compiled.input();

    input_blob = Mat(640, 640, CV_32F, Scalar(0)); // 初始化输入blob

    //tracker = BYTETracker(10, 10); // 初始化BYTETracker

    // armorsDatas = new ArmorData[20]; // 最多装20个装甲板
}

DetectionArmor::DetectionArmor(string& model_path, bool ifcountTime)
    : ifCountTime(ifcountTime), fps(0.0)
{

    ov::AnyMap config = {
        {ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)}, // 设置性能模式为延迟优化
        {ov::inference_num_threads(4)}, // 使用4个线程进行推理
        {ov::num_streams(1)}, // 允许同时执行1个推理流
        {ov::hint::scheduling_core_type(ov::hint::SchedulingCoreType::PCORE_ONLY)}, // 性能核心绑定
        {ov::hint::enable_hyper_threading(false)}, // 关闭超线程
        {ov::hint::enable_cpu_pinning(false)} // 关闭CPU固定
    };
    cout<<"dfdsfdsfdsfsdfsdfsdfsdfsdfsdfsdf";
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
Detector detector(70,l_params,a_params); 

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

// 使用不同名称以避免与 OpenCV 的 cv::Rect 冲突
struct RectF {
    float x1, y1, x2, y2;  // 左下角和右上角坐标
};

// 定义圆结构体（浮点）
struct CircleF {
    float cx, cy, r;       // 圆心坐标和半径
};

bool isOverlap(const CircleF& c, const RectF& r) {
    // 计算矩形最近点（将圆心的x,y坐标限制在矩形边界内）
    float closestX = std::max(r.x1, std::min(c.cx, r.x2));
    float closestY = std::max(r.y1, std::min(c.cy, r.y2));
    
    // 计算最近点到圆心的距离平方
    float dx = c.cx - closestX;
    float dy = c.cy - closestY;
    float distanceSquared = dx * dx + dy * dy;
    
    // 如果距离平方小于等于半径平方，则表示重叠
    return distanceSquared <= (c.r * c.r);
}

// 计算交集面积（简化版，仅处理完全包含情况）
// 计算交集面积（更完整的实现）
// 采用：先判断完全包含的简单情况；否则在 x 方向上对重叠长度做数值积分（Simpson）
float intersectionArea(const CircleF& c, const RectF& r) {
    // 第一步：规范化矩形坐标
    // 确保left < right, top < bottom
    float left = std::min(r.x1, r.x2);
    float right = std::max(r.x1, r.x2);
    float top = std::min(r.y1, r.y2);
    float bottom = std::max(r.y1, r.y2);
    
    // 第二步：检查矩形是否完全在圆内
    // 定义一个lambda函数，检查点是否在圆内（包含误差容限1e-6）
    auto insideCircle = [&](float x, float y) {
        float dx = x - c.cx;
        float dy = y - c.cy;
        return dx*dx + dy*dy <= c.r * c.r + 1e-6f;
    };
    
    // 如果矩形四个角点都在圆内，矩形完全在圆内
    if (insideCircle(left, top) && insideCircle(right, top) && 
        insideCircle(left, bottom) && insideCircle(right, bottom)) {
        return (right - left) * (bottom - top);  // 交集面积 = 矩形面积
    }
    
    // 第三步：检查圆是否完全在矩形内
    // 判断圆的外接正方形是否被矩形包含
    if (c.cx - c.r >= left && c.cx + c.r <= right && 
        c.cy - c.r >= top && c.cy + c.r <= bottom) {
        return static_cast<float>(M_PI * c.r * c.r);  // 交集面积 = 圆面积
    }
    
    // 第四步：部分重叠情况 - 使用数值积分计算重叠面积
    
    // 4.1 将坐标系平移到以圆心为原点
    // 这样圆变成中心在原点的单位圆（半径为r）
    double rx1 = left - c.cx;
    double rx2 = right - c.cx;
    double ry1 = top - c.cy;
    double ry2 = bottom - c.cy;
    
    // 4.2 限制积分区间到圆在x方向的投影[-r, r]
    // 只在圆和矩形在x方向有重叠的区域进行积分
    double x_low = std::max(rx1, -static_cast<double>(c.r));
    double x_high = std::min(rx2, static_cast<double>(c.r));
    
    // 如果x方向没有重叠，交集面积为0
    if (x_low >= x_high) return 0.0f;
    
    // 4.3 定义被积函数L(x)
    // 对于给定的x，计算圆在该x处的垂直高度，并与矩形的y区间求交
    auto L = [&](double x) -> double {
        // 计算圆在x处的y坐标（上半圆）
        double y_circle = sqrt(std::max(0.0, (double)c.r * c.r - x*x));
        
        // 圆在x处的垂直区间为[-y_circle, y_circle]
        double cy_low = -y_circle;
        double cy_high = y_circle;
        
        // 计算圆和矩形在y方向的重叠区间
        double overlap_low = std::max(cy_low, ry1);
        double overlap_high = std::min(cy_high, ry2);
        double len = overlap_high - overlap_low;
        
        // 如果长度为负，说明没有重叠
        return len > 0.0 ? len : 0.0;
    };
    
    // 4.4 使用辛普森法则数值积分
    double interval = x_high - x_low;  // 积分区间长度
    
    // 确定采样点数量（步长约0.5个单位，确保精度）
    int n = std::max(2, static_cast<int>(std::ceil(interval / 0.5)));
    if (n % 2 == 1) ++n;  // 辛普森法则要求n为偶数
    double h = interval / n;  // 步长
    
    // 计算积分：Simpson's rule公式
    double sum = L(x_low) + L(x_high);
    for (int i = 1; i < n; ++i) {
        double x = x_low + i * h;
        // 系数交替为4和2
        sum += (i % 2 == 0) ? 2.0 * L(x) : 4.0 * L(x);
    }
    double area = (h / 3.0) * sum;
    
    return static_cast<float>(area);
}

// 计算IoU
float calculateInterArea(const CircleF& c, const RectF& r) {
    // 计算矩形面积
    float rectArea = (r.x2 - r.x1) * (r.y2 - r.y1);

    // 计算圆面积
    float circleArea = M_PI * c.r * c.r;

    // 判断是否相交
    if (!isOverlap(c, r)) {
        return 0.0f;
    }
    
    
    // 计算交集面积
    float interArea = intersectionArea(c, r);

    // 计算并集面积

    // 返回IoU
    return interArea;
}

//相机内参矩阵
std::array<double, 9> cam = {
    2066.9141, 0.0,     667.3090,   // fx, 0, cx
    0.0,       2071.3661, 507.0521, // 0, fy, cy
    0.0,       0.0,     1.0         // 0, 0, 1
};
//畸变系数
std::vector<double> dist = {
    -0.054763, 0.105308, 0.002309, -0.000850, 0.000000
};
PnPSolver solver(cam, dist);
double fx = cam[0];
double fy = cam[4];
cv::Mat rev;
cv::Mat tvc;


void calculateGimbalAngles(double delta_x, double delta_y, double fx, double fy,double& yaw, double& pitch) {
    yaw = atan(delta_x / fx) * 180.0 / M_PI;
    pitch = atan(delta_y / fy) * 180.0 / M_PI;
}

void DetectionArmor::drawObject(Mat& image, vector<ArmorData>& datas)
{
    
    // 绘制装甲板的边界框
    //std::vector<Point> points = {d.p1, d.p2, d.p3, d.p4};
   
    // 计算并绘制光心点（图像中心）
    cv::Point optical_center(image.cols / 2, image.rows / 2);
    cv::Point2f left_top, right_bottom, left_bottom, right_top;
    // 使用之前定义的浮点结构体
    CircleF c = {image.cols / 2.0f, image.rows / 2.0f, 20};
    for (ArmorData& d : datas)
    {
        
        cv::Point lt = cv::Point(d.p1.x-20, d.p1.y-20);  // 左上角
        cv::Point rb = cv::Point(d.p3.x+20, d.p3.y+20);  // 右下角
        cv::Point lb = cv::Point(d.p2.x-20, d.p2.y+20);  // 左下角
        cv::Point rt = cv::Point(d.p4.x+20, d.p4.y-20);  // 右上角
        std::vector<Point> points = {lt, rt, rb, lb};
        Mat ROI;
        get_roi(image,points,ROI);
        // cv::imshow("ROI", ROI);
        auto armor = detector.detect(ROI);   
        cv::Point2f detected_center;
        detector.drawResults(image, detected_center);

        if (armor.empty()) {
            continue; 
        }
        
        //cout << d.flag << endl;
        d.center_point = detected_center;
        std::cout << "Detected center: (" << detected_center.x << ", " << detected_center.y << ")" << std::endl;
        // 计算差值
        d.optical_center = optical_center;
        d.delta_x = ((d.center_point.x - d.optical_center.x) + GUN_CAM_DISTANCE_X);
        d.delta_y = ((d.optical_center.y - d.center_point.y) + GUN_CAM_DISTANCE_Y);

        if(!datas.empty())
        {
            d.flag = 1;
            cout << "flag" <<"1" << endl;
        }
        else{
            d.flag = 0;
            cout << "flag" << "0" << endl;
        }
        // pnp
        left_top = armor[0].left_light.top;
        right_bottom = armor[0].right_light.bottom;
        left_bottom = armor[0].left_light.bottom;
        right_top = armor[0].right_light.top;
        solver.solvePnP(armor[0],rev,tvc);
        // cout << "rev :" << rev << endl;
        // cout << "tev :"  << tvc << endl;

        // 获取装甲板位姿欧拉角
        double yaw, pitch, roll;
        solver.rvecToEuler(rev, pitch, yaw, roll);
        //cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << endl;
        cout << "Distance: " << solver.calculateDistance(tvc) << "m" << endl;

        // 计算云台角度
        calculateGimbalAngles(d.delta_x, d.delta_y, fx, fy, d.target_yaw, d.target_pitch);
        
        // 计算IoU
        RectF r = {static_cast<float>(left_top.x), static_cast<float>(left_top.y), static_cast<float>(right_bottom.x), static_cast<float>(right_bottom.y)};
        float inter_area = calculateInterArea(c, r);
        if(inter_area>0.5)
        {
            d.fire_control = 1;
            cv::putText(image,"on target " ,cv::Point(10, 60),cv::FONT_HERSHEY_SIMPLEX,1,
                        Scalar(0, 255, 0),1);
        }
        else
        {
            d.fire_control = 0;
            cout<<"no target"<<endl;
            cv::putText(image,"no target " ,cv::Point(10, 60),cv::FONT_HERSHEY_SIMPLEX,1,
                        Scalar(0, 255, 0),1);
        }

        // 调试信息
        cv::circle(image, optical_center, 5, Scalar(255, 0, 0), -1); 
        cv::circle(image,cv::Point(static_cast<int>(c.cx), static_cast<int>(c.cy)),static_cast<int>(c.r),cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
        std::string delta_text = "dx:" + std::to_string(static_cast<int>(d.delta_x)) +
                                 " dy:" + std::to_string(static_cast<int>(d.delta_y));
        cv::putText(image, delta_text, rt, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    Scalar(0, 255, 0), 1);
        cv::putText(image,"fps"+to_string(fps),cv::Point(10, 30),cv::FONT_HERSHEY_SIMPLEX,1,
                    Scalar(0, 255, 0),1);
                    

    }
    

    // cv::imshow("Detection", image);

    // cv::rectangle(image, lt, rb, Scalar(0, 255, 0), 2);

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
            isRunning = false; // 设置线程停止标志
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
    
    float conf_threshold = 0.8;   
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
    for (size_t valid_index = 0; valid_index < indices.size(); ++valid_index) 
    {
        const int keep = indices[valid_index];
        ArmorData d;

        d.p1 = fourPointModel[keep][0];
        d.p2 = fourPointModel[keep][1];
        d.p3 = fourPointModel[keep][2];
        d.p4 = fourPointModel[keep][3];

        d.center_point.x = (d.p1.x + d.p2.x + d.p3.x + d.p4.x) / 4;
        d.center_point.y = (d.p1.y + d.p2.y + d.p3.y + d.p4.y) / 4;
        // d.length = boxes[indices[valid_index]].width;
        // d.width = boxes[indices[valid_index]].height;
        d.ID = num_class[keep];

        int color = color_class[keep];
        if (color == 0){ d.color = Color::RED; }
        else if (color == 1){ d.color = Color::BLUE; }
        else { d.color = Color::NONE; }

        armorsDatas.push_back(d);

        // // 创建对象用于跟踪器
        // Object dog;
        // dog.rect = cv::Rect_<float>(
        //     boxes[keep].x,
        //     boxes[keep].y,

        //     boxes[keep].width,
        //     boxes[keep].height
        // );
        // dog.label = num_class[keep];  //从类别里面取
        // dog.prob = confidences[keep];
        // detection_objects.push_back(dog);

        
    }

    // tracks_objects = tracker.update(detection_objects);
    detection_objects.clear();
}

void DetectionArmor::drawTracks(Mat& image)
{
    // 绘制跟踪轨迹
    for (const auto& track : tracks_objects) {
        if (track.is_activated) {  // 绘制已激活的轨迹
            auto tlwh = track.tlwh;
            Scalar color = tracker.get_color(track.track_id); // 获取跟踪ID对应的颜色
            
            // 绘制跟踪框
            Point tl = Point(tlwh[0], tlwh[1]); // 左上角   1
            Point br = Point(tlwh[0] + tlwh[2], tlwh[1] + tlwh[3]); // 右下角  3

            Point center = Point(tlwh[0] + tlwh[2] / 2, tlwh[1] + tlwh[3] / 2);

            circle(image, center, 5, Scalar(0,255,0), -1);

            //rectangle(image, tl, br, color, 2);
            //rectangle(image, tl, br,Scalar(0, 255, 0), 2);
            
            // 绘制跟踪ID
            // putText(image, 
            //     "Track " + to_string(track.track_id), 
            //     Point(tlwh[0], tlwh[1] - 5), 
            //     FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        }
    }
}


inline vector<ArmorData>& DetectionArmor::getdata()
{
    return armorsDatas; // 返回当前帧的装甲板数据
}

void DetectionArmor::start_detection()
{
    this->isRunning = true; // 设置线程运行标志
    run();

}

void DetectionArmor::start_detection(const cv::Mat& input_image)
{
    this->isRunning = true; // 设置线程运行标志
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
        //drawTracks(img); // 绘制跟踪轨迹

        //cv::imshow("Detection Armor", img); // 显示图像
        // format_print_data_test();
    }

    // std::lock_guard<std::mutex> lock(_mtx);
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
