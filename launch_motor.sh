#!/usr/bin/env bash
set -e

# 用法：
#  - 默认: 启动 launch 文件
#  - 直接运行节点: ./launch_motor.sh node    # armor_send_node
#  - 直接运行测试: ./launch_motor.sh test    # test_send_node

BASE_DIR="/home/ubuntu/桌面/RM26"
PKG_NAME="ros2_armor_can"

echo "=== 启动 ${PKG_NAME} ==="

# 固定中间件与域（两端保持一致）
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
: "${ROS_DOMAIN_ID:=0}"
export RMW_IMPLEMENTATION ROS_DOMAIN_ID

# Source 环境
source /opt/ros/humble/setup.bash
source "${BASE_DIR}/install/setup.bash"

# 运行时库路径（BMAPI 动态库）
export LD_LIBRARY_PATH="${BASE_DIR}/src/${PKG_NAME}/lib/bin/unix64/release:${LD_LIBRARY_PATH}"

# 可选：加强 AMENT 前缀（通常仅 source 即可，这里保留以兼容旧环境）
export AMENT_PREFIX_PATH="${BASE_DIR}/install/${PKG_NAME}:${AMENT_PREFIX_PATH}"

echo "环境设置完成"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# 诊断包可见性
if ros2 pkg list | grep -q "^${PKG_NAME}$"; then
    echo "✓ 包已找到: ${PKG_NAME}"
    ros2 pkg executables ${PKG_NAME} || true
else
    echo "✗ 未找到包: ${PKG_NAME}"
    echo "请确认已完成 colcon build 并且已 source: ${BASE_DIR}/install/setup.bash"
    exit 1
fi

mode="$1"
case "${mode}" in
    node)
        echo "→ 直接运行节点: armor_send_node"
        exec ros2 run ${PKG_NAME} armor_send_node
        ;;
    test)
        echo "→ 直接运行节点: test_send_node"
        exec ros2 run ${PKG_NAME} test_send_node
        ;;
    * )
        echo "→ 启动 launch: motor_all.launch.py"
        exec ros2 launch ${PKG_NAME} motor_all.launch.py
        ;;
esac