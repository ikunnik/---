#!/usr/bin/env bash
# 搬迁门禁：证明「结构变了，行为没变」。
#
#   tools/move_gate.sh structure          拆包后的结构断言（纯静态，无需 ROS）
#   tools/move_gate.sh capture <name>     记录当前行为基线（需要 numpy/grpc，请在容器内跑）
#   tools/move_gate.sh compare <a> <b>    比对两份基线，逐用例差异
#
# 铁律：capture 与 compare 之间，tools/ 下的测试文件一个字节都不许改。
#      一旦为适配新结构去改测试，等价性证明就失效了 —— 改的是标准答案。
#
# 依赖方向约定（见 docs/ros2_layout.md）：
#   interfaces <- core <- {perception, control, voice} <- bringup
#   tools 平级，只允许依赖 core。
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

BASELINE_DIR="docs/behavior"
FAILS=0

pass() { printf '  PASS  %s\n' "$*"; }
fail() { printf '  FAIL  %s\n' "$*"; FAILS=$((FAILS + 1)); }
skip() { printf '  SKIP  %s\n' "$*"; }

# ---------------------------------------------------------------------
# L0 结构断言：只读静态检查，不需要 ROS、不需要 numpy
# ---------------------------------------------------------------------
check_structure() {
  echo "[structure] 包拓扑"
  if command -v colcon >/dev/null 2>&1 && [[ -d src ]]; then
    # 包数量应与 docs/ros2_layout.md 的约定一致；多一个少一个都说明切分错了
    colcon list 2>/dev/null | awk '{printf "        %-28s %s\n", $1, $2}'
  else
    skip "src/ 或 colcon 不可用，跳过包拓扑"
  fi

  if [[ ! -d src/rover_x1_core ]]; then
    skip "src/rover_x1_core 不存在（第 1 步尚未开始）"
    return
  fi

  echo "[structure] core 纯度（本次重构的核心验收点）"
  local hits
  hits=$(grep -rEn '^[[:space:]]*(import|from)[[:space:]]+(rclpy|cv2|grpc)([.[:space:]]|$)' \
           src/rover_x1_core --include='*.py' || true)
  if [[ -n "$hits" ]]; then
    fail "core 引入了 ROS/图像/网络依赖，core 必须能在裸 Python 里导入："
    printf '%s\n' "$hits" | sed 's/^/        /'
  else
    pass "core 不依赖 rclpy / cv2 / grpc"
  fi

  echo "[structure] 依赖方向"
  # 反向依赖：core 引用上层包，或 interfaces 引用任何本仓包
  hits=$(grep -rEn '^[[:space:]]*(import|from)[[:space:]]+rover_x1_(perception|control|voice|bringup)' \
           src/rover_x1_core src/rover_x1_interfaces --include='*.py' 2>/dev/null || true)
  if [[ -n "$hits" ]]; then
    fail "core/interfaces 反向依赖了上层包："
    printf '%s\n' "$hits" | sed 's/^/        /'
  else
    pass "无反向依赖"
  fi

  echo "[structure] interfaces 依赖面"
  local deps
  deps=$(grep -oE '<(build|exec|test)_depend>[^<]+</[a-z_]+_depend>' \
           src/rover_x1_interfaces/package.xml 2>/dev/null |
         sed -E 's/<[^>]+>//g' | sort -u || true)
  if [[ -z "$deps" ]]; then
    skip "package.xml 不可读，跳过"
  else
    local bad
    bad=$(printf '%s\n' "$deps" |
          grep -vE '^(std_msgs|rosidl_default_generators|rosidl_default_runtime|ament_cmake|builtin_interfaces|action_msgs)$' || true)
    if [[ -n "$bad" ]]; then
      fail "interfaces 混入了非必要依赖：$(printf '%s' "$bad" | tr '\n' ' ')"
    else
      pass "interfaces 只依赖 std_msgs / rosidl 默认组"
    fi
  fi

  echo "[structure] 旧路径 shim（决定 selftest 能否零改动跑）"
  local pkg
  for pkg in decisions workers motion runtime app; do
    if [[ -f "$pkg/__init__.py" ]] && [[ -d src/rover_x1_core/rover_x1_core/$pkg || -d "src/rover_x1_control/rover_x1_control/$pkg" ]]; then
      if grep -qE 'from (rover_x1_core|rover_x1_control)' "$pkg/__init__.py"; then
        pass "$pkg/ 已降级为 re-export shim"
      else
        fail "$pkg/ 与新位置并存但未降级为 shim，存在两份实现的风险"
      fi
    fi
  done

  if (( FAILS > 0 )); then
    echo "[structure] 未通过：$FAILS 项"
    return 1
  fi
  echo "[structure] 通过"
}

# ---------------------------------------------------------------------
# 行为基线：把 check.sh 全量输出规范化后落盘
# 规范化是为了剔除 UUID / 时间戳 / 耗时 / 绝对路径这类必然变化的噪声
# ---------------------------------------------------------------------
normalize() {
  LC_ALL=C sed -E \
    -e 's/[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}/<UUID>/g' \
    -e 's/[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9:.]+([+-][0-9]{2}:?[0-9]{2}|Z)/<TIMESTAMP>/g' \
    -e 's/0x[0-9a-fA-F]+/<ADDR>/g' \
    -e 's#(/home|/tmp|/root)/[^ "]+#<PATH>#g' \
    -e 's/([0-9]+\.[0-9]+)[[:space:]]*(ms|s)\b/<DURATION>/g' \
    -e 's/[[:space:]]+$//'
}

capture() {
  local name="${1:-}"
  if [[ -z "$name" ]]; then
    echo "用法: tools/move_gate.sh capture <基线名>  例如 before-move" >&2
    return 2
  fi
  local out="$BASELINE_DIR/${name}.txt"
  mkdir -p "$BASELINE_DIR"
  echo "[capture] 运行 tools/check.sh --require-full → $out"
  # 先记退出码，再规范化；PIPESTATUS 必须在管道后立刻取
  tools/check.sh --require-full 2>&1 | normalize > "$out"
  local code=${PIPESTATUS[0]}
  local cases
  cases=$(grep -cE '(^|[[:space:]])(PASS|FAIL|ok|OK|FAILED)([[:space:]]|$)' "$out" || true)
  echo "[capture] 退出码=$code，可识别用例行=$cases"
  if [[ "$code" -ne 0 ]]; then
    echo "[capture] 注意：基线本身未全绿。搬迁只能让 diff 为空，不能让它变绿。" >&2
  fi
  echo "[capture] 已记录。搬迁后请用 compare 与它逐行比对。"
}

compare() {
  local a="${1:-}" b="${2:-}"
  if [[ -z "$a" || -z "$b" ]]; then
    echo "用法: tools/move_gate.sh compare <基线A> <基线B>" >&2
    return 2
  fi
  local fa="$BASELINE_DIR/${a}.txt" fb="$BASELINE_DIR/${b}.txt"
  [[ -f "$fa" ]] || { echo "基线不存在: $fa" >&2; return 2; }
  [[ -f "$fb" ]] || { echo "基线不存在: $fb" >&2; return 2; }

  echo "[compare] $a  vs  $b"
  if diff -u "$fa" "$fb" > /tmp/move_gate_diff.$$ 2>&1; then
    pass "行为完全等价（diff 为空）"
    rm -f /tmp/move_gate_diff.$$
    return 0
  fi

  echo
  echo "  出现行为差异 —— 搬迁不允许有「这个差异是预期的」这种解释。"
  echo "  差异行数：$(grep -cE '^[+-]' /tmp/move_gate_diff.$$ || true)"
  echo
  grep -E '^[+-]' /tmp/move_gate_diff.$$ | grep -vE '^(\+\+\+|---)' | head -40 | sed 's/^/    /'
  echo
  echo "  完整差异：/tmp/move_gate_diff.$$"
  return 1
}

case "${1:-}" in
  structure) check_structure ;;
  capture)   shift; capture "$@" ;;
  compare)   shift; compare "$@" ;;
  *)
    echo "用法: tools/move_gate.sh {structure | capture <name> | compare <A> <B>}" >&2
    exit 2
    ;;
esac
