# image_subscriber 节点说明

- **作用**：订阅摄像头话题 `/image_raw`，调用 OpenVINO 装甲板检测（`detection::DetectionArmor`）生成偏移量；以 10 Hz 发布 `target_delta`（`geometry_msgs/Point`）和 `target_info`（`std_msgs/Float32MultiArray`）供下游电机/弹道控制使用。
- **入口**：`src/recive_pkg/main.cpp:3-17` 直接构造 `armor_detection::ImageSubscriber`，默认模型路径为 `src/rm_auto_aim/detection/model/IR_MODEL/new.xml`。
- **核心逻辑**：`src/recive_pkg/src/image_subscirber_node.cpp:6-103`  
  - 订阅 `/image_raw`（sensor_data QoS，best_effort）。  
  - 将 ROS 图像转为 OpenCV，调用 `detectionArmor_.start_detection` 并取首个目标。  
  - 计算光心差值并发布 `target_delta`、`target_info`（QoS：reliable + transient_local）。  
  - 内置 100 帧 FPS 统计与 10 Hz 发布节流。
- **依赖与结构**：头文件在 `include/recive_pkg/image_subscriber_node.hpp`，检测库代码复用 `src/rm_auto_aim/detection/src/*.cpp` 与对应头文件，需事先安装 OpenCV/OpenVINO（CMake 引用 `openvino::runtime`）。
- **与其它节点关系**：  
  - 上游：`hik_camera_node` 发布 `/image_raw`。  
  - 下游：`armor_send_node` / `test_send_node` 订阅 `target_delta` 和 `target_info`，通过 CAN 下发电机指令。  
  - 模型与预处理参数与 `detection` 包保持一致（调整模型路径时同步 `test_aim_node`）。
- **运行**：在工作区完成 `colcon build` 后，`source install/setup.bash && ros2 run recive_pkg recive_pkg`（记得先启动摄像头节点或播放 bag）。
