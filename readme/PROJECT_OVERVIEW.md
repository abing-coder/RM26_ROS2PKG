# RM26 ROS2 自瞄系统 - 项目概述

## 一、项目总览

这是一个 Robomaster 机器人自动瞄准系统，基于 ROS2 Humble 构建。

### 项目结构

```
RM26_ROS2PKG/
├── src/
│   ├── rm_auto_aim/detection/     # 核心：YOLO装甲板检测（重点）
│   ├── recive_pkg/                # ROS2图像订阅节点
│   ├── ros2_armor_can/            # CAN电机控制
│   └── ros2_hik_camera-main/      # 海康相机驱动
├── build/                         # 编译输出
├── install/                       # 安装目录
└── *.sh                           # 启动脚本
```

### 系统数据流

```
海康相机 → /image_raw → 检测节点 → target_delta/target_info → CAN电机控制
```

---

## 二、其他节点（简述）

| 包名 | 功能 | 核心技术 |
|-----|------|---------|
| `ros2_hik_camera-main` | 相机驱动，发布 `/image_raw` | 海康SDK |
| `ros2_armor_can` | 订阅目标信息，CAN发送电机命令 | BMcan库 |
| `recive_pkg` | 图像订阅+检测封装，发布检测结果 | cv_bridge |

---

## 三、核心：rm_auto_aim/detection 检测包（重点）

### 3.1 目录结构

```
detection/
├── include/
│   ├── yolo_detection.hpp         # 主检测类 DetectionArmor
│   ├── inference_engine.hpp       # OpenVINO推理引擎封装
│   ├── armor.hpp                  # ArmorData 数据结构
│   ├── traditional_detector.hpp   # 传统灯条检测（备用）
│   └── performance_logger.hpp     # 性能日志
├── src/
│   ├── yolo_detection.cpp         # 检测实现（372行）
│   ├── inference_engine.cpp       # 推理实现
│   └── traditional_detector.cpp
├── model/
│   └── new2.onnx                  # 当前使用的YOLO模型
└── TimeCounter/
    └── timeCounter.hpp            # 帧计时工具
```

### 3.2 核心类：DetectionArmor

**文件**：`include/yolo_detection.hpp`

```cpp
namespace detection {
    class DetectionArmor {
    public:
        static int detect_color;   // 0=红色, 1=蓝色
        double fps;

        // 构造
        DetectionArmor(std::string& model_path, bool if_count_time, std::string video_path = "");

        // 核心接口
        void start_detection(const cv::Mat& input);  // 单帧检测
        std::vector<ArmorData>& getdata();           // 获取结果
        void drawObject(cv::Mat& image, std::vector<ArmorData>& datas);

    private:
        InferenceEngine m_inference_engine;
        std::vector<ArmorData> m_armors_datas;

        void infer();                    // 推理
        void parseDetections(...);       // 解析输出
        void applyNMS(...);              // NMS抑制
        void buildArmorData(...);        // 构建输出
    };
}
```

**关键常量**：
```cpp
constexpr int INPUT_SIZE = 640;
constexpr float CONFIDENCE_THRESHOLD = 0.6f;
constexpr float NMS_THRESHOLD = 0.4f;
constexpr int INFERENCE_THREADS = 14;
```

### 3.3 数据结构：ArmorData

**文件**：`include/armor.hpp`

```cpp
struct ArmorData {
    cv::Point center_point;      // 目标中心（像素坐标）
    cv::Point optical_center;    // 光心（图像中心）
    float delta_x, delta_y;      // 中心与光心的偏移

    cv::Point p1, p2, p3, p4;    // 四角点坐标
    int ID;                      // 装甲板数字 (0-9)
    Color color;                 // RED/BLUE/NONE
};
```

### 3.4 推理引擎：InferenceEngine

**文件**：`include/inference_engine.hpp`

```cpp
class InferenceEngine {
public:
    InferenceEngine(const std::string& model_path, int input_size = 640, int threads = 14);
    cv::Mat infer(const cv::Mat& input_image);  // 返回模型输出

private:
    ov::Core m_core;
    ov::CompiledModel m_compiled_model;
    ov::InferRequest m_infer_request;
};
```

**OpenVINO 配置**：
```cpp
ov::AnyMap config = {
    {ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)},
    {ov::inference_num_threads(10)},
    {ov::num_streams(1)},
    {ov::hint::enable_cpu_pinning(true)}
};
```

### 3.5 检测流程

```
输入图像 (cv::Mat)
    ↓
[归一化] 1/255.0
    ↓
[OpenVINO推理] → 输出 [1, N, 22]
    ├─ 列0-7: 四角点坐标 (x1,y1,x2,y2,x3,y3,x4,y4)
    ├─ 列8: 置信度 (需Sigmoid)
    ├─ 列9-12: 颜色概率 (4类)
    └─ 列13-21: 数字概率 (9类)
    ↓
[解析] parseDetections
    ├─ Sigmoid激活置信度
    ├─ 过滤 < 0.6
    └─ 颜色过滤
    ↓
[NMS] IoU > 0.4 的去重
    ↓
[构建] ArmorData数组
    ↓
输出: std::vector<ArmorData>
```

### 3.6 模型信息

- **路径**：`model/new2.onnx`
- **输入**：640x640 RGB，归一化
- **输出**：`[1, N, 22]`
  - 四点坐标 + 置信度 + 颜色(4) + 数字(9)

---

## 四、ROS2 话题通信

### 发布话题（recive_pkg 节点）

| 话题 | 类型 | 内容 |
|-----|------|------|
| `target_delta` | `geometry_msgs/Point` | (delta_x, delta_y, 0) |
| `target_info` | `std_msgs/Float32MultiArray` | [cx, cy, ox, oy, dx, dy] |

### 订阅话题

| 话题 | 类型 | 订阅者 |
|-----|------|-------|
| `/image_raw` | `sensor_msgs/Image` | recive_pkg |
| `target_delta` | `geometry_msgs/Point` | ros2_armor_can |

### QoS 策略

```cpp
// 可靠传输 + 持久化
rclcpp::QoS qos(10);
qos.reliable();
qos.transient_local();
```

---

## 五、依赖

| 依赖 | 版本 | 用途 |
|-----|------|------|
| OpenVINO | 2025.0 | 神经网络推理 |
| OpenCV | 4.x | 图像处理 |
| ROS2 | Humble | 中间件 |
| C++ | C++14 | 语言标准 |

**OpenVINO 路径**：
```
/home/ubuntu/桌面/Robomaster/openvino_toolkit_ubuntu22_2025.0.0.17942.1f68be9f594_x86_64/
```

---

## 六、关键代码位置

| 功能 | 文件 |
|-----|-----|
| 检测主类 | `src/rm_auto_aim/detection/include/yolo_detection.hpp` |
| 检测实现 | `src/rm_auto_aim/detection/src/yolo_detection.cpp` |
| 推理引擎 | `src/rm_auto_aim/detection/include/inference_engine.hpp` |
| 数据结构 | `src/rm_auto_aim/detection/include/armor.hpp` |
| ROS2节点 | `src/recive_pkg/include/recive_pkg/image_subscriber_node.hpp` |
| 电机控制 | `src/ros2_armor_can/src/armor_send.cpp` |

---

## 七、使用示例

```cpp
// 初始化检测器
std::string model_path = "model/new2.onnx";
detection::DetectionArmor detector(model_path, true);
detection::DetectionArmor::detect_color = 0; // 检测红色

// 单帧检测
cv::Mat frame = ...;
detector.start_detection(frame);

// 获取结果
auto& results = detector.getdata();
for (auto& armor : results) {
    std::cout << "ID: " << armor.ID
              << " Center: " << armor.center_point
              << " Delta: (" << armor.delta_x << ", " << armor.delta_y << ")"
              << std::endl;
}
```

---

## 八、编译

```bash
cd /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG
colcon build --packages-select detection recive_pkg
source install/setup.bash
```
