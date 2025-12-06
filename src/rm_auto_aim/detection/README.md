# Detection Package

ROS2 装甲板检测包，基于 YOLO 和 OpenVINO 推理引擎。

## 功能特性

- YOLO 装甲板检测
- OpenVINO INT8 量化推理优化
- 零拷贝预处理管道（PPP）
- 自适应输入尺寸支持

## 构建

```bash
colcon build --packages-select detection --cmake-args -DCMAKE_CXX_FLAGS="-march=native"
```

## INT8 推理优化配置

检测器使用 OpenVINO 针对 INT8 量化模型进行了优化配置：

### CPU 配置参数

- `performance_mode`: LATENCY - 优化单帧延迟
- `execution_mode`: PERFORMANCE - 允许混合精度层以获得最佳性能
- `inference_precision`: INT8 - 使用 INT8 推理
- `num_requests`: 1 - 单帧推理
- `enable_cpu_mem_pinning`: true - 启用 CPU 内存固定
- `denormals_optimization`: true - 启用反规范化优化

### OV_CPU_CAPABILITIES 环境变量

检测器会自动设置 `OV_CPU_CAPABILITIES` 环境变量以优化 ISA 选择：

**默认值**: `AMX_INT8,AVX512_CORE_VNNI,BF16`

#### 运行时覆盖

如果需要自定义 CPU 指令集，可以通过以下方式之一：

1. **环境变量**（运行前设置）:
```bash
export OV_CPU_CAPABILITIES="AMX_INT8,AVX512_CORE_VNNI"
source install/setup.bash
./build/detection/test_aim_node
```

2. **ROS 参数覆盖**（适用于 `recive_pkg` 中的 `image_subscriber` 节点）：
```bash
ros2 run recive_pkg recive_pkg --ros-args -p cpu_capabilities_override:="AMX_INT8,AVX512_CORE_VNNI,BF16"
```
默认值为空字符串（继承已有环境变量或包内默认值）；仅当参数非空时才会覆盖 `OV_CPU_CAPABILITIES`。

3. **构造函数参数**（代码中传递）:
```cpp
std::string custom_caps = "AVX2";
detection::DetectionArmor detector(model_path, true, video_path, custom_caps);
```

#### 调试输出

引擎初始化时会输出实际使用的 `OV_CPU_CAPABILITIES` 值：
```
[InferenceEngine] OV_CPU_CAPABILITIES set to (default): AMX_INT8,AVX512_CORE_VNNI,BF16
```

## 模型路径

当前使用的 INT8 量化模型：
```
src/rm_auto_aim/detection/model/IR_MODEL/3_int8_new.xml
```

## 使用示例

### 基本用法
```cpp
#include "yolo_detection.hpp"

std::string model_path = "model/IR_MODEL/3_int8_new.xml";
detection::DetectionArmor detector(model_path, true);

cv::Mat image = cv::imread("test.jpg");
detector.start_detection(image);
auto results = detector.getdata();
```

### 自定义 CPU 优化
```cpp
std::string model_path = "model/IR_MODEL/3_int8_new.xml";
std::string cpu_caps = "AMX_INT8,AVX512_CORE_VNNI";
detection::DetectionArmor detector(model_path, true, "", cpu_caps);
```

## 测试

运行测试节点：
```bash
source install/setup.bash
./build/detection/test_aim_node
```

或使用测试脚本：
```bash
./test_aim.sh
```

## 性能调优建议

1. **确认 CPU 支持**: 使用 `lscpu` 检查 CPU 支持的指令集
2. **INT8 模型**: 确保使用 INT8 量化模型以获得最佳性能
3. **编译优化**: 使用 `-march=native` 编译以启用所有 CPU 特性
4. **线程数调整**: 根据 CPU 核心数调整 `INFERENCE_THREADS` (默认 14)

## API 参考

### DetectionArmor 构造函数
```cpp
DetectionArmor(std::string& model_path,
               bool if_count_time,
               std::string video_path = "",
               const std::string& cpu_capabilities_override = "");
```

参数：
- `model_path`: OpenVINO IR 模型路径（.xml）
- `if_count_time`: 是否启用性能计时
- `video_path`: 视频文件路径（可选，用于测试）
- `cpu_capabilities_override`: OV_CPU_CAPABILITIES 覆盖值（空字符串使用默认值）

### InferenceEngine 构造函数
```cpp
InferenceEngine(const std::string& model_path,
                int input_size = 640,
                int inference_threads = 14,
                const std::string& cpu_capabilities_override = "");
```

参数：
- `model_path`: 模型文件路径
- `input_size`: 输入尺寸（默认 640）
- `inference_threads`: 推理线程数（默认 14）
- `cpu_capabilities_override`: CPU 能力覆盖值

## 故障排除

### 性能问题
- 检查是否使用了正确的 INT8 模型
- 确认 CPU 支持 AVX512/AMX 指令集
- 查看初始化日志确认使用的 ISA

### 编译错误
- 确保 OpenVINO 2025.0+ 已正确安装
- 检查 CMakeLists.txt 中的 OpenVINO_DIR 路径

### 运行时错误
- 确认模型文件存在且路径正确
- 检查 OpenVINO 运行时库是否在 LD_LIBRARY_PATH 中
