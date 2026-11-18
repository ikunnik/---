#!/usr/bin/env bash
# Rover X1 一键启动：自动进入 distrobox、配置 ROS2 / RMW / CycloneDDS，再跑 main.py
#   ./start.sh                 # 正常启动
#   ./start.sh --help          # 看参数
#   ./start.sh --no-mission    # 透传任意 main.py 参数
# 注意：不要用 set -u，ROS2 的 setup.bash 里有未定义变量
set -eo pipefail

BOX="${BOX_NAME:-dobot-quad}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 步骤 1：不在容器里就先进容器（宿主机直接跑本脚本即可）
if [ -z "${CONTAINER_ID:-}" ]; then
  args=""
  for a in "$@"; do args="$args $(printf '%q' "$a")"; done
  exec distrobox enter "$BOX" -- bash -lc "cd '$DIR' && exec bash ./start.sh$args"
fi

# 步骤 2：ROS 2 Humble（提供 rclpy / geometry_msgs）
source /opt/ros/humble/setup.bash

# 步骤 3：中间件与 DDS 配置
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [ -z "${CYCLONEDDS_URI:-}" ] || [ ! -f "${CYCLONEDDS_URI#file://}" ]; then
  source /usr/local/bin/quad-dds-env >/dev/null 2>&1 || true   # 生成含 EnableTopicDiscoveryEndpoints 的配置
fi

# 步骤 4：启动
cd "$DIR"
exec python3 main.py "$@"
