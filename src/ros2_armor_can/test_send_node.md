# test_send_node 节点说明

- **作用**：在未接入视觉链路时，周期性向 CAN 总线发送递增测试数据，验证 BM-CANFD 通道与下游电机接收是否正常。
- **实现位置**：`src/ros2_armor_can/src/test.cpp:1-79`  
  - 初始化打开 CAN 设备，定时器每 100 ms 调用 `send_test_data`，帧 ID 默认 0x520，数据第 1 字节自增。  
  - 析构时关闭 CAN 通道。
- **依赖**：与 `armor_send_node` 共用 `robotcontrol/bmcan_bus` 和 `bmapi64/bmapi` 库。
- **与其它节点关系**：独立于视觉链路运行；常用于联调 BM-CANFD 接口或电机侧固件。可与 `armor_send_node` 互斥使用以避免总线写冲突。
- **运行**：`source install/setup.bash && ros2 run ros2_armor_can test_send_node`；如需修改发送频率或 ID，可调整 `timer_callback` 中的周期与 `cantx_id`。
