#!/bin/bash

# 终止所有ROS2节点的脚本

echo "正在查找并终止所有ROS2节点..."

# 方法1: 使用pkill终止常见的ROS2进程
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "_node" 2>/dev/null || true

# 方法2: 查找并终止剩余的ROS2相关进程
ros2_pids=$(pgrep -f "ros2|_node|detection|recive_pkg")

if [ -z "$ros2_pids" ]; then
    echo "所有ROS2节点已终止"
    exit 0
fi

echo "终止剩余进程..."
for pid in $ros2_pids; do
    kill -9 $pid 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "已终止进程: $pid"
    fi
done

echo ""
echo "所有ROS2节点已终止"
