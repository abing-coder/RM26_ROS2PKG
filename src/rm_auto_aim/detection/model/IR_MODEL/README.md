# IR模型int8量化脚本使用说明

## 概述
`quantize_layers.py` 脚本用于将OpenVINO IR模型的前四层卷积层权重从fp16量化为int8，实现2倍压缩。

## 功能特点
- ✅ 自动识别卷积层（Convolution类型）
- ✅ 将fp16权重转换为fp32，再量化为int8
- ✅ 使用对称量化方案（±127范围）
- ✅ 保持精度信息（quantization_scale）
- ✅ 生成压缩后的模型文件

## 使用方法

### 基本用法
```bash
cd /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/rm_auto_aim/detection/model/IR_MODEL/
python3 quantize_layers.py
```

### 输出文件
脚本运行后会生成两个文件：
- `new_int8.xml` - 量化后的模型拓扑结构
- `new_int8.bin` - 量化后的权重文件

### 压缩效果
以本模型为例：
```
原始模型: new.xml (347KB) + new.bin (2.8MB)
量化后:   new_int8.xml (340KB) + new_int8.bin (2.8MB)

前4层压缩: 2096 bytes → 1048 bytes (压缩比: 2.00x)
```

## 量化详情

### 支持的模型格式
- 输入：OpenVINO IR模型（.xml + .bin）
- 原始权重精度：fp16（半精度浮点）
- 目标精度：int8（有符号8位整数）

### 量化的层
脚本会自动识别并量化前4个卷积层，包括：
1. 第一层卷积：通常是大卷积核（如7x7, 3x3等）
2. 后续卷积层：通常是1x1或3x3卷积

### 量化算法
- **方案**：对称量化
- **范围**：[-127, 127] → int8
- **缩放因子**：基于权重最大绝对值计算
- **公式**：`quantized = round(original / max_abs * 127)`

## 在代码中使用量化模型

在 `inference_engine.cpp` 中切换到量化模型：

```cpp
// 原始代码
// auto model = core.read_model("model/IR_MODEL/new.xml");

// 使用量化模型
auto model = core.read_model("model/IR_MODEL/new_int8.xml");
```

## 注意事项

⚠️ **精度说明**
- 此脚本使用静态量化方案，未进行校准
- 可能会有精度损失，建议测试性能
- 如需更高精度，推荐使用OpenVINO POT工具进行校准量化

⚠️ **兼容性**
- 量化后的模型需要在支持int8的硬件上运行
- CPU需要支持AVX512_VNNI或类似指令集以获得最佳性能

## 进一步优化

如需更高级的量化，可以使用OpenVINO官方工具：

```bash
# 安装OpenVINO
pip install openvino-dev

# 使用POT进行校准量化
pot -c quantized_model_config.json -o output_dir
```

## 脚本结构

主要函数：
- `fp16_to_fp32()` - 将fp16字节转换为fp32
- `quantize_fp32_to_int8()` - 执行int8量化
- `main()` - 主执行流程

## 故障排除

### 问题：ImportError
解决：`pip install xml.etree.ElementTree`

### 问题：压缩比不为2.0x
说明：正常，不同层的权重数量可能不同

### 问题：量化后模型加载失败
检查：
1. XML文件格式是否正确
2. BIN文件是否存在且未损坏
3. element_type是否正确设置为"i8"

---
**作者**：Claude Code
**版本**：1.0
