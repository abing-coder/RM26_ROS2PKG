#!/bin/bash

# 启动test.launch.py的脚本

echo "=== 启动 test.launch.py ==="

# 设置环境
source /opt/ros/humble/setup.bash
source /home/ubuntu/桌面/RM26/install/setup.bash

# 手动添加ros2_armor_can包到AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH="/home/ubuntu/桌面/RM26/install/ros2_armor_can:$AMENT_PREFIX_PATH"

echo "环境设置完成，启动launch文件..."

# 检查包是否可用
if ros2 pkg list | grep -q "ros2_armor_can"; then
    echo "✓ ros2_armor_can 包已找到"
else
    echo "✗ ros2_armor_can 包未找到"
    exit 1
fi

# 启动launch文件
ros2 launch ros2_armor_can test.launch.py