#!/usr/bin/env bash
# 启动摄像头、识别节点 + 调试显示节点（不影响主流程）
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

# 路径与环境
OPENVINO_DIR="/home/ubuntu/桌面/Robomaster/openvino_toolkit_ubuntu22_2025.0.0.17942.1f68be9f594_x86_64"
ROS_LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${ROS_LOG_DIR}"

echo "=== 启动带显示的调试流程 ==="

# 加载 OpenVINO（若存在）
if [ -f "${OPENVINO_DIR}/setupvars.sh" ]; then
  echo "[env] source OpenVINO"
  # shellcheck disable=SC1090
  source "${OPENVINO_DIR}/setupvars.sh"
else
  echo "[warn] OpenVINO setupvars.sh 未找到，可能影响推理性能"
fi

echo "[env] incremental build"
bash "${WORKSPACE_DIR}/incremental_build.sh"

# 检查 build 结果
if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  echo "[error] install/setup.bash 不存在，可能构建失败"
  exit 1
fi

echo "[env] source workspace"
# shellcheck disable=SC1090
source "${WORKSPACE_DIR}/install/setup.bash"

# 使用最少反压的显示：best-effort 深度1
IMAGE_TOPIC_DEFAULT="/debug/detection_image"
IMAGE_TOPIC="${1:-${IMAGE_TOPIC_DEFAULT}}"

echo "[info] IMAGE_TOPIC=${IMAGE_TOPIC}"
echo "[info] ROS_LOG_DIR=${ROS_LOG_DIR}"

# 启动摄像头与识别节点
echo "[run] launch hik_camera"
ROS_LOG_DIR="${ROS_LOG_DIR}" ros2 launch hik_camera hik_camera.launch.py &
HIK_PID=$!

sleep 1

echo "[run] start recive_pkg (检测)"
ROS_LOG_DIR="${ROS_LOG_DIR}" ros2 run recive_pkg recive_pkg &
RECIVE_PID=$!

sleep 1

echo "[run] start debug_display_node (仅显示)"
ROS_LOG_DIR="${ROS_LOG_DIR}" ros2 run recive_pkg debug_display_node --ros-args -p image_topic:="${IMAGE_TOPIC}" &
DISPLAY_PID=$!

echo "进程 ID:"
echo "  hik_camera_node: ${HIK_PID}"
echo "  recive_pkg:      ${RECIVE_PID}"
echo "  debug_display:   ${DISPLAY_PID}"
echo "按 Ctrl+C 结束所有进程"

cleanup() {
  echo "Stopping..."
  kill ${DISPLAY_PID} ${RECIVE_PID} ${HIK_PID} 2>/dev/null || true
  wait ${DISPLAY_PID} ${RECIVE_PID} ${HIK_PID} 2>/dev/null || true
  echo "Done."
}

trap cleanup INT TERM
wait
