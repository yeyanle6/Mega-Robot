#!/bin/bash
# T-Robot SLAM System - Bringup Test Script
# Tests: Sensors → State Estimation → TF Tree

set -e

cleanup() {
  if [ -n "$BRINGUP_PID" ] && kill -0 "$BRINGUP_PID" 2>/dev/null; then
    kill "$BRINGUP_PID" 2>/dev/null || true
    wait "$BRINGUP_PID" 2>/dev/null || true
  fi

  if [ "${MICRO_ROS_STARTED}" = "1" ] && [ -n "$MICRO_ROS_PID" ]; then
    if kill -0 "$MICRO_ROS_PID" 2>/dev/null; then
      kill "$MICRO_ROS_PID" 2>/dev/null || true
      wait "$MICRO_ROS_PID" 2>/dev/null || true
    fi
  fi

  ros2 daemon stop 2>/dev/null || true
  ros2 daemon start >/dev/null 2>&1 || true
}

trap cleanup EXIT

echo "=========================================="
echo "  T-Robot SLAM Bringup Test"
echo "=========================================="
echo ""

# Source base ROS 2 workspace（保证 ros2 命令可用）
source /opt/ros/humble/setup.bash

# Source 主项目工作空间
if [ -f "install/setup.bash" ]; then
  source install/setup.bash
else
  echo "❌ 未找到 install/setup.bash，请先编译工作空间。"
  exit 1
fi

# 启动 micro-ROS Agent（用于 Megarover3 有线底盘通讯）
MICRO_ROS_AGENT_DEV=${MICRO_ROS_AGENT_DEV:-/dev/ttyUSB0}
MICRO_ROS_AGENT_ARGS=${MICRO_ROS_AGENT_ARGS:-"serial --dev ${MICRO_ROS_AGENT_DEV} -v4"}
MICRO_ROS_AGENT_LOG=${MICRO_ROS_AGENT_LOG:-/tmp/micro_ros_agent.log}
MICRO_ROS_PID=""
MICRO_ROS_STARTED=0

if ros2 node list 2>/dev/null | grep -q micro_ros_agent; then
  echo "[0/6] micro-ROS Agent 已在运行，跳过自动启动。"
else
  if [ -c "$MICRO_ROS_AGENT_DEV" ]; then
    if [ -f "/home/wang/uros_ws/install/setup.bash" ]; then
      echo "[0/6] 启动 micro-ROS Agent (${MICRO_ROS_AGENT_ARGS})..."
      (
        source /opt/ros/humble/setup.bash
        source /home/wang/uros_ws/install/setup.bash
        ros2 run micro_ros_agent micro_ros_agent ${MICRO_ROS_AGENT_ARGS}
      ) >"${MICRO_ROS_AGENT_LOG}" 2>&1 &
      MICRO_ROS_PID=$!
      MICRO_ROS_STARTED=1
      echo "  micro-ROS Agent PID: $MICRO_ROS_PID"
      echo "  日志: ${MICRO_ROS_AGENT_LOG}"
      sleep 3
    else
      echo "⚠️ 未找到 /home/wang/uros_ws/install/setup.bash，无法自动启动 micro-ROS Agent。"
      echo "   请手动执行: source /home/wang/uros_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent ${MICRO_ROS_AGENT_ARGS}"
    fi
  else
    echo "⚠️ 未检测到 ${MICRO_ROS_AGENT_DEV} 设备，micro-ROS Agent 将不会自动启动。"
  fi
fi

echo "[1/6] Launching bringup system..."
echo "  - Base chassis driver"
echo "  - MID360 LiDAR + IMU"
echo "  - Static TF publishers"
echo "  - EKF state estimation"
echo ""

# 启动 bringup（后台运行，输出到日志）
ros2 launch t_robot_bringup bringup.launch.py > /tmp/bringup_test.log 2>&1 &
BRINGUP_PID=$!

echo "Bringup PID: $BRINGUP_PID"
echo "等待 10 秒初始化..."
sleep 10

echo ""
echo "[2/6] Checking active nodes..."
ros2 node list

echo ""
echo "[3/6] Checking published topics..."
ros2 topic list | grep -E "mid360|livox|odom|tf" || echo "  ⚠️ 未找到关键话题"

echo "" 
echo "[4/6] Testing topic data + frequencies..." 

check_topic() {
  local topic="$1"
  local qos_flag="$2"
  local echo_cmd=(ros2 topic echo --once)
  if [ -n "$qos_flag" ]; then
    echo_cmd+=(--qos-reliability "$qos_flag")
  fi
  echo_cmd+=("$topic")

  echo "  Checking $topic ..."
  if timeout 5 "${echo_cmd[@]}" >/tmp/bringup_topic.$$ 2>&1; then
    echo "    ✓ Data detected"
    local hz_cmd=(ros2 topic hz "$topic" --window 10)
    if [ -n "$qos_flag" ]; then
      hz_cmd+=(--qos-reliability "$qos_flag")
    fi
    local hz_output
    if hz_output=$(timeout 5 "${hz_cmd[@]}" 2>&1); then
      if grep -q "average rate" <<<"$hz_output"; then
        echo "$hz_output" | head -n 3 | sed 's/^/    /'
      else
        echo "$hz_output" | sed 's/^/    /'
        echo "    (no average rate line detected, command succeeded)"
      fi
    else
      echo "    ⚠️ Unable to compute frequency"
    fi
  else
    echo "    ❌ No data (echo timed out)"
    if [ -n "$qos_flag" ] && [ "$qos_flag" != "reliable" ]; then
      echo "    ℹ️ 提示：确认发布端是否使用 $qos_flag QoS"
    fi
  fi
  rm -f /tmp/bringup_topic.$$ 2>/dev/null || true
}

check_topic /mid360/lidar best_effort
check_topic /mid360/imu reliable
check_topic /odom reliable
check_topic /odometry/filtered reliable

echo "" 
echo "[QoS] Verifying relay QoS settings..." 
ros2 topic info -v /mid360/imu 2>/dev/null | sed 's/^/  /' || echo "  ⚠️ 无法查询 /mid360/imu QoS" 
if ros2 topic list 2>/dev/null | grep -q '^/imu/data$'; then
  ros2 topic info -v /imu/data 2>/dev/null | sed 's/^/  /'
else
  echo "  ℹ️ /imu/data 未发布（如需旧接口，可启用 imu_relay publish_legacy_topic 参数）"
fi 

echo "" 
echo "[5/6] Generating TF tree visualization..." 

if ! ros2 pkg executables tf2_tools 2>/dev/null | grep -q view_frames; then
  echo "  ⚠️ 未找到 tf2_tools/view_frames，可通过 'sudo apt install ros-humble-tf2-tools graphviz' 安装"
else
  TF_TARGET=${TF_OUTPUT_FILE:-frames.pdf}
  TF_BASE=${TF_TARGET%.*}
  timeout 10 ros2 run tf2_tools view_frames --output "$TF_BASE" >/tmp/bringup_tf.$$ 2>&1 || true

  TF_PDF="${TF_BASE}.pdf"
  if [ -f "$TF_PDF" ]; then
    if [ "$TF_PDF" != "$TF_TARGET" ]; then
      mv -f "$TF_PDF" "$TF_TARGET" 2>/dev/null || cp -f "$TF_PDF" "$TF_TARGET"
    fi
    echo "  ✅ TF tree saved to $TF_TARGET"
    rm -f "${TF_BASE}.gv" 2>/dev/null || true
    echo "  Converting to text summary..."
    timeout 3 ros2 run tf2_ros tf2_echo base_link mid360_lidar 2>/dev/null | head -10
  else
    echo "  ⚠️ view_frames 未生成 $TF_PDF，原始输出如下："
    sed 's/^/    /' /tmp/bringup_tf.$$ || true
    echo "  ❌ Failed to generate TF tree"
  fi
  rm -f /tmp/bringup_tf.$$ 2>/dev/null || true
fi

echo ""
echo "[6/6] micro-ROS Agent 状态"
if [ -n "$MICRO_ROS_PID" ]; then
  if ps -p $MICRO_ROS_PID > /dev/null; then
    echo "  ✅ micro-ROS Agent 仍在运行 (PID: $MICRO_ROS_PID)"
  else
    echo "  ⚠️ micro-ROS Agent 已退出，请查看日志 ${MICRO_ROS_AGENT_LOG}"
  fi
else
  echo "  ℹ️ 若需底盘里程计，请确认 micro-ROS Agent 已手动启动。"
fi

echo ""
echo "=========================================="
echo "  Test Summary"
echo "=========================================="
echo "Bringup log: /tmp/bringup_test.log"
echo "TF tree: frames.pdf (if generated)"
if [ -n "$MICRO_ROS_PID" ]; then
  echo "micro-ROS Agent log: ${MICRO_ROS_AGENT_LOG}"
fi
echo ""
echo "保持窗口，按 Ctrl+C 停止 bringup 测试，脚本将自动清理节点。"
echo "在另一终端继续执行 ./test_mapping.sh 或 ./test_navigation.sh。"
echo "=========================================="

wait "$BRINGUP_PID" 2>/dev/null || true

echo "Bringup 测试结束，资源已清理。"
