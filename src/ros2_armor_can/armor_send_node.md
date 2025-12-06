# armor_send_node 节点说明

- **作用**：订阅自瞄检测输出 `target_delta`（偏移量）和 `target_info`（目标/光心信息），通过 BM-CANFD 设备将偏移量打包成 0x520 CAN 帧下发给电机/云台控制。
- **实现位置**：`src/ros2_armor_can/src/armor_send.cpp:12-144`  
  - 初始化时打开 CAN 通道；若失败会打印错误。  
  - 订阅 `target_delta` 和 `target_info`，QoS 设为 `reliable + transient_local` 与上游一致。  
  - 在回调内启动后台线程，调用 `send_motor_cmd` 发送 16 位整数偏移量，避免阻塞 DDS 回调。
- **依赖与库**：依赖 `robotcontrol/bmcan_bus` 封装，链接 `bmapi64`（或 `bmapi`，取决于架构）；包含路径在 `include/robotcontrol` 与 `lib/bin` 下。
- **与其它节点关系**：  
  - 上游：`recive_pkg` 发布 `target_delta`、`target_info`。  
  - 同包中的 `test_send_node` 可在无视觉输入时验证 CAN 通路。  
  - 下游为物理电机控制器（非 ROS 节点）。
- **运行**：`source install/setup.bash && ros2 run ros2_armor_can armor_send_node`；确保 CAN 设备型号字符串与硬件一致（默认 `"BM-CANFD-X1(5850)"`）。
