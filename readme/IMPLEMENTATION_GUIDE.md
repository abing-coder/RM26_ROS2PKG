# 视觉自瞄节点实现路线

## 一、技术栈总览

| 组件 | 版本/规格 | 用途 |
|-----|----------|------|
| **OpenVINO** | 2025.0.0.17942 | 神经网络推理加速 |
| **OpenCV** | 4.x (core, imgproc, highgui, videoio, dnn) | 图像处理、NMS |
| **C++** | C++14 | 编程语言 |
| **ROS2** | Humble | 节点通信 |
| **模型格式** | ONNX | YOLO 检测模型 |
| **推理设备** | CPU | Intel 处理器 |

---

## 二、OpenVINO 配置详情

### 2.1 安装路径

```
/home/ubuntu/桌面/Robomaster/openvino_toolkit_ubuntu22_2025.0.0.17942.1f68be9f594_x86_64/
```

### 2.2 CMake 配置

```cmake
set(OpenVINO_DIR /home/ubuntu/桌面/Robomaster/openvino_toolkit_ubuntu22_2025.0.0.17942.1f68be9f594_x86_64/runtime/cmake)
find_package(OpenVINO REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE openvino::runtime)
```

### 2.3 推理优化配置

```cpp
ov::AnyMap config = {
    // 低延迟模式 - 适合实时推理
    {ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)},
    // 性能执行模式
    {ov::hint::execution_mode(ov::hint::ExecutionMode::PERFORMANCE)},
    // 推理线程数
    {ov::inference_num_threads(10)},
    // 单流模式 - 避免多流调度开销
    {ov::num_streams(1)},
    // CPU 核心绑定 - 减少线程迁移
    {ov::hint::enable_cpu_pinning(true)},
    // 日志级别
    {ov::log::level(ov::log::Level::WARNING)}
};

// 编译模型到 CPU
m_compiled_model = m_core.compile_model(network, "CPU", config);
```

---

## 三、模型规格

### 3.1 输入规格

| 属性 | 值 |
|-----|-----|
| 尺寸 | 640 x 640 |
| 通道 | RGB (3通道) |
| 归一化 | 像素值 / 255.0 |
| 数据类型 | float32 |

### 3.2 输出规格

| 属性 | 值 |
|-----|-----|
| 形状 | [1, N, 22] |
| N | 检测框数量（动态） |

**输出列含义**：

| 列索引 | 内容 |
|-------|------|
| 0-7 | 四角点坐标 (x1,y1,x2,y2,x3,y3,x4,y4) |
| 8 | 置信度（需 Sigmoid 激活） |
| 9-12 | 颜色概率（4类：红/蓝/...） |
| 13-21 | 数字概率（9类：0-8） |

### 3.3 当前模型

```
路径: src/rm_auto_aim/detection/model/new2.onnx
格式: ONNX
```

---

## 四、核心参数配置

```cpp
// yolo_detection.cpp
constexpr int INPUT_SIZE = 640;              // 模型输入尺寸
constexpr float CONFIDENCE_THRESHOLD = 0.6f; // 置信度阈值
constexpr float NMS_THRESHOLD = 0.4f;        // NMS IoU 阈值
constexpr int INFERENCE_THREADS = 14;        // 推理线程数
```

---

## 五、实现流程

### 5.1 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                      检测流水线                              │
└─────────────────────────────────────────────────────────────┘

[输入图像]
    │
    ▼
┌─────────────────┐
│ 1. 预处理        │  cv::resize → 640x640
│    Preprocess   │  cv::dnn::blobFromImage → 归一化 1/255
└─────────────────┘
    │
    ▼
┌─────────────────┐
│ 2. 推理          │  OpenVINO InferRequest::infer()
│    Inference    │  输出: cv::Mat [N, 22]
└─────────────────┘
    │
    ▼
┌─────────────────┐
│ 3. 解析          │  parseDetections()
│    Parse        │  - Sigmoid 激活置信度
│                 │  - 过滤 conf < 0.6
│                 │  - 颜色过滤
│                 │  - 提取四点坐标
└─────────────────┘
    │
    ▼
┌─────────────────┐
│ 4. NMS          │  cv::dnn::NMSBoxes()
│    抑制重叠框    │  IoU > 0.4 去重
└─────────────────┘
    │
    ▼
┌─────────────────┐
│ 5. 构建输出      │  buildArmorData()
│    Build        │  - 计算中心点
│                 │  - 关联颜色/ID
│                 │  - 生成 ArmorData
└─────────────────┘
    │
    ▼
[std::vector<ArmorData>]
```

### 5.2 关键代码路径

```
start_detection(cv::Mat)
    │
    ├─► cv::resize(input, m_img, 640x640)
    │
    └─► infer()
            │
            ├─► m_inference_engine.infer(m_img)
            │       │
            │       ├─► cv::dnn::blobFromImage(img, 1/255.0)
            │       ├─► set_input_tensor()
            │       ├─► infer_request.infer()
            │       └─► 返回 cv::Mat output_buffer
            │
            ├─► parseDetections(output_buffer, ...)
            │       │
            │       ├─► 遍历每行输出
            │       ├─► sigmoid(conf) → 过滤 < 0.6
            │       ├─► argmax(color_scores) → 颜色
            │       ├─► argmax(class_scores) → 数字ID
            │       ├─► 颜色过滤（红/蓝匹配）
            │       └─► 提取四点坐标
            │
            ├─► applyNMS(boxes, confidences, indices)
            │       └─► cv::dnn::NMSBoxes()
            │
            └─► buildArmorData(indices, fourPoints, ...)
                    │
                    ├─► 设置四角点 p1,p2,p3,p4
                    ├─► 计算中心点
                    ├─► 设置 ID 和颜色
                    └─► push_back 到 m_armors_datas
```

---

## 六、数据结构

### 6.1 ArmorData 输出结构

```cpp
struct ArmorData {
    // 位置信息
    cv::Point center_point;    // 装甲板中心（像素）
    cv::Point optical_center;  // 图像光心
    float delta_x, delta_y;    // 中心与光心偏移

    // 四角点（YOLO 输出）
    cv::Point p1, p2, p3, p4;

    // 分类信息
    int ID;        // 数字 0-8
    Color color;   // RED / BLUE / NONE
};
```

### 6.2 颜色枚举

```cpp
enum class Color { RED, BLUE, NONE };

// 检测目标颜色（静态变量）
static int detect_color;  // 0=红色, 1=蓝色
```

---

## 七、ROS2 集成

### 7.1 节点封装 (recive_pkg)

```cpp
class ImageSubscriber : public rclcpp::Node {
    detection::DetectionArmor detectionArmor_;  // 检测器实例

    // 订阅
    Subscription<sensor_msgs::msg::Image> subscription_;  // /image_raw

    // 发布
    Publisher<geometry_msgs::msg::Point> delta_publisher_;           // target_delta
    Publisher<std_msgs::msg::Float32MultiArray> target_info_publisher_; // target_info
};
```

### 7.2 回调处理

```cpp
void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 1. ROS Image → cv::Mat
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // 2. 执行检测
    detectionArmor_.start_detection(frame);

    // 3. 获取结果
    auto& results = detectionArmor_.getdata();

    // 4. 发布
    if (!results.empty()) {
        // 发布 target_delta
        // 发布 target_info
    }
}
```

### 7.3 QoS 配置

```cpp
rclcpp::QoS qos(10);
qos.reliable();          // 可靠传输
qos.transient_local();   // 持久化最新消息
```

---

## 八、性能指标

| 指标 | 值 |
|-----|-----|
| 推理帧率 | ~30 FPS |
| 单帧延迟 | ~33ms |
| 线程数 | 10-14 |
| 内存占用 | 取决于模型 |

---

## 九、编译依赖

### 9.1 CMakeLists.txt 关键配置

```cmake
cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 14)

# OpenVINO
set(OpenVINO_DIR /path/to/openvino/runtime/cmake)
find_package(OpenVINO REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio dnn)

# 链接
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    openvino::runtime
)
```

### 9.2 ROS2 依赖 (recive_pkg)

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge
)
```

---

## 十、文件清单

| 文件 | 作用 |
|-----|------|
| `include/yolo_detection.hpp` | DetectionArmor 类定义 |
| `src/yolo_detection.cpp` | 检测逻辑实现 |
| `include/inference_engine.hpp` | OpenVINO 引擎封装 |
| `src/inference_engine.cpp` | 推理实现 |
| `include/armor.hpp` | Light/Armor 数据结构 |
| `model/new2.onnx` | YOLO 检测模型 |

---

## 十一、使用示例

```cpp
#include "yolo_detection.hpp"

int main() {
    // 初始化
    std::string model = "model/new2.onnx";
    detection::DetectionArmor detector(model, true);
    detection::DetectionArmor::detect_color = 0;  // 检测红色

    // 单帧检测
    cv::Mat frame = cv::imread("test.jpg");
    detector.start_detection(frame);

    // 获取结果
    for (auto& armor : detector.getdata()) {
        printf("ID=%d, Center=(%d,%d), Delta=(%.1f,%.1f)\n",
               armor.ID,
               armor.center_point.x, armor.center_point.y,
               armor.delta_x, armor.delta_y);
    }
    return 0;
}
```

---

## 十二、调试选项

```cpp
// yolo_detection.hpp
#define TEST_MODE  // 启用后可调用 showImage() 和 format_print_data_test()

// 可视化
detector.showImage();  // 显示检测框、中心点、FPS

// 性能日志
OpenVINOProfiler profiler("openvino_performance.log");
timeCounter counter("run a frame");
```
