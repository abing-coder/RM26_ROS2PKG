# hik_camera_node 节点说明

- **作用**：通过海康 MVS SDK 采集相机帧，转换为 BGR8 并发布 `/image_raw` 与 `/camera_info`，为自瞄检测提供实时图像源。
- **入口与实现**：`src/ros2_hik_camera-main/src/hik_camera_node.cpp:16-182`  
  - 初始化相机句柄，声明参数并加载标定（`camera_info_url`）。  
  - 创建 `image_transport` 发布者，QoS 基于 sensor_data，可靠性被设为 Reliable。  
  - 独立采集线程抓取、像素转换、发布 `sensor_msgs/Image` 与配套 `CameraInfo`。
- **配置文件**：`config/camera_params.yaml` 设置曝光、增益、分辨率、帧率；`config/camera_info.yaml` 提供内参，用于 launch 中的 `camera_info_url`。
- **与其它节点关系**：  
  - 下游订阅者：`recive_pkg` 的 `image_subscriber` 消费 `/image_raw`。  
  - 发布的 `camera_info` 可被 RViz/标定工具或其他视觉节点使用。
- **启动**：`ros2 launch hik_camera hik_camera.launch.py`（可通过 `params_file` 与 `camera_info_url` 覆盖默认配置）。
