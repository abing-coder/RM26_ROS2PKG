# RM26 ROS2 自动瞄准系统

## 项目简介

本项目是基于 ROS2 的 RoboMaster 自动瞄准系统，集成了视觉检测、目标跟踪和电机控制功能。系统使用 YOLO 深度学习模型进行装甲板检测，通过 CAN 总线与电机通信，实现自动瞄准功能。

## 系统架构

项目采用模块化设计，包含以下核心功能包：

```
RM26_ROS2PKG/
├── src/
│   ├── recive_pkg/              # 图像接收与检测节点
│   ├── rm_auto_aim/             # 自动瞄准核心算法
│   │   └── detection/           # YOLO 检测模块
│   ├── ros2_hik_camera-main/    # 海康威视相机驱动
│   └── ros2_armor_can/          # CAN 通信与电机控制
└── README.md
```

## 功能模块详解

### 1. recive_pkg - 图像接收与检测节点

**功能描述：**
- 订阅相机发布的原始图像流 (`/image_raw`)
- 调用 YOLO 检测模块进行装甲板识别
- 计算目标中心点与光心的偏移量
- 发布目标偏移信息供电机控制使用

**主要话题：**
- 订阅：`/image_raw` (sensor_msgs/Image) - 原始图像数据
- 发布：`target_delta` (geometry_msgs/Point) - 目标偏移量 (x, y)
- 发布：`target_info` (std_msgs/Float32MultiArray) - 完整目标信息

**关键特性：**
- 10Hz 发布频率控制，避免过载
- QoS 配置：使用 reliable + transient_local 保证消息可靠性
- 实时图像显示（调试模式）

### 2. rm_auto_aim/detection - YOLO 检测模块

**功能描述：**
- 基于 OpenVINO 推理引擎的 YOLO 目标检测
- 支持红蓝装甲板识别
- 传统图像处理辅助检测
- 性能计时与优化

**核心类：**
- `DetectionArmor`: 主检测类
  - 模型加载与推理
  - 装甲板数据提取
  - 坐标计算与转换

**数据结构：**
```cpp
struct ArmorData {
    cv::Point center_point;    // 目标中心点
    cv::Point optical_center;  // 光心点
    float delta_x;             // X 轴偏移
    float delta_y;             // Y 轴偏移
    cv::Point p1, p2, p3, p4;  // 四个角点
    int ID;                    // 装甲板 ID
    Color color;               // 颜色（红/蓝）
};
```

**依赖项：**
- OpenVINO 2024.6.0
- OpenCV (core, imgproc, highgui, videoio, dnn)
- Eigen (线性代数库)

### 3. ros2_hik_camera-main - 海康威视相机驱动

**功能描述：**
- 海康威视工业相机 SDK 封装
- 图像采集与发布
- 相机参数配置与管理
- 相机标定支持

**配置参数：** (camera_params.yaml)
```yaml
camera_name: narrow_stereo
exposure_time: 8500.0  # 曝光时间 (μs)
gain: 32.0             # 增益
width: 1280            # 图像宽度
height: 720            # 图像高度
```

**发布话题：**
- `/image_raw` (sensor_msgs/Image) - 原始图像流
- `/camera_info` (sensor_msgs/CameraInfo) - 相机标定信息

### 4. ros2_armor_can - CAN 通信与电机控制

**功能描述：**
- 通过 CAN 总线与电机控制板通信
- 接收目标偏移量并转换为电机控制指令
- 支持 BM-CANFD-X1 设备

**通信协议：**
- CAN ID: 0x520 (发送)
- CAN ID: 0x101 (接收)
- 数据格式：8 字节
  - Byte 0-1: motor1 (X 轴偏移，高低字节)
  - Byte 2-3: motor2 (Y 轴偏移，高低字节)
  - Byte 4-7: 保留

**订阅话题：**
- `target_delta` (geometry_msgs/Point) - 目标偏移量
- `target_info` (std_msgs/Float32MultiArray) - 完整目标信息

**关键特性：**
- 异步发送机制，避免阻塞 ROS2 回调
- QoS 配置与发布方对齐 (reliable + transient_local)
- 错误处理与日志记录

## 系统工作流程

```
[海康相机] → /image_raw → [检测节点] → target_delta → [CAN 节点] → [电机控制]
                              ↓
                         target_info
                              ↓
                          [日志输出]
```

1. **图像采集**：海康相机以配置的分辨率和帧率采集图像
2. **图像发布**：相机节点将图像发布到 `/image_raw` 话题
3. **目标检测**：检测节点接收图像，使用 YOLO 模型识别装甲板
4. **偏移计算**：计算目标中心与图像光心的偏移量
5. **数据发布**：发布偏移量到 `target_delta` 话题
6. **电机控制**：CAN 节点接收偏移量，转换为电机指令并通过 CAN 总线发送
7. **闭环控制**：电机调整云台姿态，实现自动瞄准

## 环境依赖

### 系统要求
- Ubuntu 22.04 LTS
- ROS2 Humble
- Linux Kernel 6.8.0-87-generic

### 核心依赖
- **OpenVINO**: 2024.6.0 (安装路径: `/opt/intel/openvino_2024.6.0`)
- **OpenCV**: 4.x (需包含 core, imgproc, highgui, videoio, dnn 模块)
- **ROS2 包**:
  - rclcpp
  - sensor_msgs
  - geometry_msgs
  - std_msgs
  - cv_bridge
  - image_transport
  - camera_info_manager
- **硬件驱动**:
  - 海康威视 MVS SDK
  - BM-CANFD-X1 驱动

## 编译与安装

### 1. 安装依赖

```bash
# 安装 ROS2 依赖
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-image-transport \
                 ros-humble-camera-info-manager

# 安装 OpenVINO (如未安装)
# 参考: https://docs.openvino.ai/latest/openvino_docs_install_guides_installing_openvino_linux.html

# 安装 OpenCV
sudo apt install libopencv-dev
```

### 2. 编译项目

```bash
cd /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG

# 编译所有包
colcon build

# 或编译特定包
colcon build --packages-select recive_pkg
colcon build --packages-select hik_camera
colcon build --packages-select ros2_armor_can
```

### 3. 配置环境

```bash
# 加载 OpenVINO 环境
source /opt/intel/openvino_2024.6.0/setupvars.sh

# 加载 ROS2 工作空间
source install/setup.bash
```

## 使用方法

### 启动完整系统

```bash
# 终端 1: 启动相机节点
ros2 launch hik_camera hik_camera.launch.py

# 终端 2: 启动检测节点
ros2 run recive_pkg recive_pkg

# 终端 3: 启动 CAN 通信节点
ros2 run ros2_armor_can armor_send
```

### 使用 Launch 文件（推荐）

```bash
# 启动完整系统
ros2 launch ros2_armor_can motor_all.launch.py
```

### 查看话题

```bash
# 查看所有话题
ros2 topic list

# 查看图像话题
ros2 topic echo /image_raw

# 查看目标偏移
ros2 topic echo target_delta

# 查看目标详细信息
ros2 topic echo target_info
```

### 调试模式

检测节点会自动显示检测结果窗口（需要 GUI 环境）：
- 原始图像窗口：显示实时图像
- 检测结果：在图像上绘制装甲板位置

## 配置说明

### 相机参数配置

编辑 `src/ros2_hik_camera-main/config/camera_params.yaml`：

```yaml
/hik_camera:
  ros__parameters:
    camera_name: narrow_stereo
    exposure_time: 8500.0  # 根据光照条件调整
    gain: 32.0             # 增益值
    width: 1280            # 分辨率
    height: 720
```

### 检测颜色配置

在 `src/rm_auto_aim/detection/include/yolo_detection.hpp` 中：

```cpp
// 修改 detect_color 静态变量
// 0: 红色，1: 蓝色
static int detect_color = 0;
```

### CAN 设备配置

在 `src/ros2_armor_can/src/armor_send.cpp` 中：

```cpp
const int cantx_id = 0x520;  // 发送 ID
const int canrx_id = 0x101;  // 接收 ID
```

## 模型文件

YOLO 模型位于：`src/rm_auto_aim/detection/model/0526.onnx`

**模型信息：**
- 格式：ONNX
- 推理引擎：OpenVINO
- 输入：BGR 图像
- 输出：装甲板检测框与分类

## 性能优化

### 1. 检测频率控制
检测节点默认以 10Hz 发布，可在 `image_subscriber_node.cpp` 中调整：

```cpp
const std::chrono::milliseconds publish_interval_{100};  // 100ms = 10Hz
```

### 2. OpenVINO 优化
- 使用 GPU 推理（如有）
- 模型量化（INT8）
- 批处理优化

### 3. 图像传输优化
- 使用 `image_transport` 压缩
- 调整 QoS 策略
- 降低图像分辨率（如不影响检测精度）

## 常见问题

### 1. 相机无法打开
- 检查 USB 连接
- 确认海康 SDK 已正确安装
- 检查设备权限：`sudo chmod 666 /dev/bus/usb/*/*`

### 2. CAN 设备打开失败
- 确认 BM-CANFD-X1 驱动已安装
- 检查设备连接：`lsusb`
- 查看内核日志：`dmesg | grep can`

### 3. OpenVINO 找不到
- 确认安装路径正确
- 执行 `source /opt/intel/openvino_2024.6.0/setupvars.sh`
- 检查 CMakeLists.txt 中的路径配置

### 4. 检测无输出
- 检查模型文件是否存在
- 确认 `detect_color` 设置正确
- 查看日志输出：`ros2 topic echo /rosout`

### 5. 消息接收不到
- 检查 QoS 配置是否匹配
- 使用 `ros2 topic info <topic_name> -v` 查看详情
- 确认节点正常运行：`ros2 node list`

## 开发说明

### 代码结构

```
src/
├── recive_pkg/
│   ├── include/
│   │   └── recive_pkg/
│   │       └── image_subscriber_node.hpp
│   ├── src/
│   │   └── image_subscirber_node.cpp
│   └── main.cpp
│
├── rm_auto_aim/detection/
│   ├── include/
│   │   ├── yolo_detection.hpp
│   │   ├── traditional_detector.hpp
│   │   ├── armor.hpp
│   │   └── performance_logger.hpp
│   ├── src/
│   │   ├── yolo_detection.cpp
│   │   └── traditional_detector.cpp
│   └── TimeCounter/
│       └── timeCounter.hpp
│
├── ros2_hik_camera-main/
│   ├── src/
│   │   └── hik_camera_node.cpp
│   ├── config/
│   │   └── camera_params.yaml
│   └── launch/
│       └── hik_camera.launch.py
│
└── ros2_armor_can/
    ├── include/
    │   └── robotcontrol/
    │       └── bmcan_bus.hpp
    ├── src/
    │   ├── armor_send.cpp
    │   └── bmcan_bus.cpp
    └── launch/
        └── motor_all.launch.py
```

### 添加新功能

1. **添加新的检测算法**：
   - 在 `rm_auto_aim/detection/include` 中添加头文件
   - 在 `rm_auto_aim/detection/src` 中实现算法
   - 在 `DetectionArmor` 类中集成

2. **修改通信协议**：
   - 编辑 `armor_send.cpp` 中的 `send_motor_cmd` 函数
   - 调整数据打包格式

3. **添加新的传感器**：
   - 创建新的 ROS2 包
   - 实现传感器驱动节点
   - 在 launch 文件中集成

## Git 分支说明

- **main**: 主分支，稳定版本
- **TX**: 当前开发分支

## 最近更新

- 移除 Eigen 库的本地副本，改用系统安装
- 移除 BYTETracker 跟踪模块
- 优化 CMakeLists.txt 配置
- 添加性能日志记录功能
- 改进 QoS 配置以提高消息可靠性

## 许可证

Apache License 2.0

## 维护者

- 邮箱：18588995616@163.com
- 邮箱：1258124282@qq.com

## 参考资料

- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [OpenVINO 文档](https://docs.openvino.ai/)
- [海康威视 MVS SDK](https://www.hikrobotics.com/cn/machinevision/service/download)
- [RoboMaster 官方论坛](https://bbs.robomaster.com/)

## 致谢

感谢 RoboMaster 社区和所有贡献者的支持。
