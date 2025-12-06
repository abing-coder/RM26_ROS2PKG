# detection / test_aim_node 说明

- **作用**：封装装甲板检测流水线（OpenVINO + OpenCV）。`detection` 可执行体用于离线/调试；`test_aim_node` 用于回放测试，核心类 `detection::DetectionArmor` 被 `recive_pkg` 节点复用。
- **主要入口**：  
  - `src/rm_auto_aim/detection/main.cpp:1-17` 读取默认视频与模型路径，构造 `DetectionArmor` 进行本地检测。  
  - `src/rm_auto_aim/detection/test_aim_node.cpp`（同样链接所有检测源文件）用于 `test_aim.sh` 离线回放。
- **核心结构**：  
  - 头文件：`include/yolo_detection.hpp`（`DetectionArmor` 类、`ArmorData` 结构、颜色枚举）、`inference_engine.hpp`、`traditional_detector.hpp`、`openvino_profiler.hpp`、`performance_logger.hpp`、`armor.hpp`。  
  - 源文件：`src/*.cpp` 实现预处理、推理、NMS、性能计数；`TimeCounter` 提供耗时统计；`model/IR_MODEL` 存放 OpenVINO IR 模型（默认 `new.xml`）。  
  - 辅助资源：`video/` 放样例视频；`log/` 存性能日志。
- **与其它节点关系**：  
  - `recive_pkg` 在 `CMakeLists.txt` 中将本包所有检测源文件编入 `recive_pkg` 节点，实现在线检测。  
  - 检测颜色全局变量 `DetectionArmor::detect_color` 在 `recive_pkg` 内设定，需与战队阵营同步。  
  - 输出的 `ArmorData.delta_x/delta_y` 最终经 `target_delta` 话题送入 `armor_send_node`。
- **使用提醒**：更新模型/预处理时需同步调整 `recive_pkg` 的模型路径；若单独跑离线测试，可执行 `colcon build --packages-select detection` 后运行 `./install/detection/lib/detection/test_aim_node` 或自定义视频路径。
