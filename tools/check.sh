#!/usr/bin/env bash
# 本地/CI 最小门禁：语法编译 + 纯逻辑回归；容器里有 grpc 时再跑全量自检。
set -euo pipefail
cd "$(dirname "$0")/.."

keep_going=false
require_full=false
failures=0
for option in "$@"; do
  case "$option" in
    --keep-going) keep_going=true ;;
    --require-full) require_full=true ;;
    *) echo "未知参数: $option" >&2; exit 2 ;;
  esac
done
run_check() {
  if "$@"; then
    return 0
  else
    local code=$?
    failures=$((failures + 1))
    echo "[check] 失败 ($code): $*" >&2
    if ! "$keep_going"; then return "$code"; fi
  fi
}

echo "[check] 语法编译"
# __pycache__ 目录可能属于其他用户，把字节码缓存重定向到 /tmp
export PYTHONPYCACHEPREFIX="${TMPDIR:-/tmp}/rover_x1_pycache"
python3 -m compileall -q main.py robot_factory.py config.py messages.py actions.py comm.py robot_link.py ros2_robot_link.py motion app runtime decisions workers tools rover_x1 launch setup.py

echo "[check] 默认配置一致性"
run_check python3 tools/generate_ros_config.py --check

echo "[check] 纯逻辑回归（无外部依赖）"
run_check python3 tools/test_ros_package.py
run_check python3 tools/test_ros_contracts.py
run_check python3 tools/test_ros_camera.py
run_check python3 tools/test_migration_baseline.py
run_check python3 tools/test_decisions.py
run_check python3 tools/test_traffic_profile.py
run_check python3 tools/test_mission.py
run_check python3 tools/test_mission_flow.py
run_check python3 tools/test_lane_no_line.py
run_check python3 tools/test_ros2_link.py
run_check python3 tools/test_motion_api.py
run_check python3 tools/test_path_return.py
run_check python3 tools/test_complex_avoidance.py
run_check python3 tools/test_stop_line_align.py
echo "[check] 掉头仿真（正常/补转）"
run_check python3 tools/uturn_probe.py --simulate >/dev/null
run_check python3 tools/uturn_probe.py --simulate --yaw-error 15 >/dev/null

if python3 -c "import grpc, numpy" >/dev/null 2>&1; then
  echo "[check] 全量自检（需要 grpc/numpy）"
  run_check python3 tools/test_actions.py
  run_check python3 tools/test_mangdao.py
  run_check python3 tools/test_bypass_distance.py
  run_check python3 tools/selftest.py
else
  echo "[check] 跳过 actions/selftest：当前 Python 缺少 grpc/numpy（请在 roverx1-dev 容器执行）"
  if "$require_full"; then failures=$((failures + 1)); fi
fi

if (( failures > 0 )); then
  echo "[check] $failures 个检查命令失败" >&2
  exit 1
fi
echo "[check] 通过"
