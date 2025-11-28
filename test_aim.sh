#!/bin/bash

# 测试瞄准节点启动脚本
# 自动构建并启动测试瞄准节点

echo "=== 测试瞄准节点启动脚本 ==="

# 切换到工作目录
cd /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG

# 检查是否在正确目录
if [ ! -f "src/rm_auto_aim/detection/test_aim_node.cpp" ]; then
    echo "错误: 不在正确的项目目录中"
    exit 1
fi

echo "1. 构建检测包..."
colcon build --packages-select detection

if [ $? -ne 0 ]; then
    echo "构建失败，请检查错误信息"
    exit 1
fi

echo ""
echo "2. 启动测试瞄准节点..."
echo "   节点将自动检测视频目录中的文件"
echo "   请根据提示选择要测试的视频"
echo "   按 'q' 键退出程序"
echo ""

# 运行测试节点
./build/detection/test_aim_node

echo ""
echo "测试完成"
