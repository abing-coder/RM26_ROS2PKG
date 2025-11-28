#!/bin/bash

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

# 先执行增量编译
echo "正在执行增量编译..."
bash "$WORKSPACE_DIR/incremental_build.sh"

# 检查编译是否成功
if [ $? -ne 0 ]; then
    echo "编译失败，停止启动节点"
    exit 1
fi

# 检查 setup.bash 是否存在
SETUP_FILE="$WORKSPACE_DIR/install/setup.bash"
if [ ! -f "$SETUP_FILE" ]; then
    echo "错误: 找不到 $SETUP_FILE"
    echo "请确保已经编译了工作空间"
    exit 1
fi

echo "正在启动 ROS2 节点..."
echo "工作空间目录: $WORKSPACE_DIR"

# Source 环境
echo "正在加载 ROS2 环境..."
source "$SETUP_FILE"

echo "环境加载完成，开始启动节点..."

# 使用后台运行的方式启动两个节点
ros2 run hik_camera hik_camera_node &
echo "启动 hik_camera_node (后台运行)..."
HIK_PID=$!

sleep 1

echo "启动 recive_pkg (后台运行)..."
ros2 run recive_pkg recive_pkg &
RECIVE_PID=$!

echo "所有节点已启动完成!"
echo "进程 ID:"
echo "  hik_camera_node: $HIK_PID"
echo "  recive_pkg: $RECIVE_PID"
echo ""
echo "要停止节点，请按 Ctrl+C 或运行:"
echo "  kill $HIK_PID $RECIVE_PID"

# 等待用户中断
trap "echo '正在停止节点...'; kill $HIK_PID $RECIVE_PID 2>/dev/null; echo '节点已停止'; exit 0" INT

# 保持脚本运行
wait
