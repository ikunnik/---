#!/usr/bin/env bash
# 与真机代码同步：默认只比较（不做任何修改）。
#
#   bash tools/sync_robot.sh --diff      # 比较本地与真机源码
#   bash tools/sync_robot.sh --push      # 备份远端 → 推送 → 校验
#   bash tools/sync_robot.sh --pull      # 拉回 robot_snapshot/ 供 diff
#
# 真机默认：robot@192.168.1.20:/home/robot/rover_x1_ws4
# 需要 ssh 可用（密钥或交互密码）；--batch 只允许免密登录。
set -euo pipefail
cd "$(dirname "$0")/.."

HOST="robot@192.168.1.20"
DEST="/home/robot/rover_x1_ws4"
MODE="diff"
FORCE=0
YES=0
BATCH=0
PRUNE_REMOTE=0

usage() {
  sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --diff) MODE="diff"; shift ;;
    --push) MODE="push"; shift ;;
    --pull) MODE="pull"; shift ;;
    --host) HOST="$2"; shift 2 ;;
    --dest) DEST="$2"; shift 2 ;;
    --force) FORCE=1; shift ;;
    --yes) YES=1; shift ;;
    --batch) BATCH=1; shift ;;
    --prune) PRUNE_REMOTE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

SSH=(ssh)
# 某些机器的 /etc/ssh/ssh_config.d 属主异常，ssh 会直接拒绝启动；
# 此时回退到 -F /dev/null（仍然使用默认密钥和 known_hosts）。
if ! ssh -G "$HOST" >/dev/null 2>&1; then
  echo "[sync] ssh 系统配置不可读，回退 -F /dev/null" >&2
  SSH+=(-F /dev/null)
fi
if [[ $BATCH -eq 1 ]]; then
  SSH+=(-o BatchMode=yes -o ConnectTimeout=5)
fi
SSH+=("$HOST")

# 只同步源码/配置/语音包；不碰缓存、标定图片和旧版本目录。
PRUNE=( -name '__pycache__' -o -name '*.pyc' -o -name 'rgb_images'
        -o -name 'depth_images' -o -name 'legacy'
        -o -name 'decisions.old_*' -o -name '.vscode' -o -name 'robot_snapshot'
        -o -name '.sync_*' )
KEEP=( -name '*.py' -o -name '*.md' -o -name '*.json'
       -o -name '*.txt' -o -name '*.wav' )

local_list() {
  find . \( "${PRUNE[@]}" \) -prune -o -type f \( "${KEEP[@]}" \) -print \
    | sed 's|^\./||' | LC_ALL=C sort
}

local_manifest() {
  local_list | while IFS= read -r f; do sha256sum -- "$f"; done \
    | sed 's|  | |' | LC_ALL=C sort -k2
}

remote_manifest() {
  "${SSH[@]}" bash -s -- "$DEST" <<'REMOTE' 2>/tmp/rover_sync_err.$$ \
    | sed 's|  | |' | LC_ALL=C sort -k2
set -e
cd "$1" 2>/dev/null || { echo "__NO_DEST__" >&2; exit 3; }
find . \( -name '__pycache__' -o -name '*.pyc' -o -name 'rgb_images' \
          -o -name 'depth_images' -o -name 'legacy' \
          -o -name 'decisions.old_*' -o -name '.vscode' -o -name 'robot_snapshot' \
          -o -name '.sync_*' \) -prune -o \
  -type f \( -name '*.py' -o -name '*.md' -o -name '*.json' \
             -o -name '*.txt' -o -name '*.wav' \) -print \
| sed 's|^\./||' | LC_ALL=C sort \
| while IFS= read -r f; do sha256sum -- "$f"; done
REMOTE
  local rc=$?
  if [[ $rc -ne 0 ]]; then
    echo "[sync] 读取远端失败（exit=$rc）" >&2
    grep -v '__NO_DEST__' /tmp/rover_sync_err.$$ 2>/dev/null | head -5 >&2 || true
    echo "[sync] 提示：先在终端执行 ssh $HOST 确认可登录；免密需先 ssh-copy-id" >&2
  fi
  rm -f /tmp/rover_sync_err.$$
  return $rc
}

run_diff() {
  local tmp; tmp=$(mktemp -d); trap 'rm -rf "$tmp"' RETURN
  local_manifest > "$tmp/local"
  if ! remote_manifest > "$tmp/remote"; then
    return 1
  fi
  local n; n=$(wc -l < "$tmp/local")
  if cmp -s "$tmp/remote" "$tmp/local"; then
    echo "[sync] 两端一致（$n 个源文件）"
    return 0
  fi
  echo "[sync] 差异（+ 仅本地 / ~ 内容不同 / - 仅真机）："
  awk 'NR==FNR{h[$2]=$1; next}
       {if (!($2 in h)) print "+ " $2;
        else {if (h[$2] != $1) print "~ " $2; delete h[$2]}}
       END{for (p in h) print "- " p}' "$tmp/remote" "$tmp/local" \
    | LC_ALL=C sort -k2 | sed -n '1,140p'
  echo "[sync] 本地 $n 个，真机 $(wc -l < "$tmp/remote") 个"
  return 1
}

run_push() {
  if ! "${SSH[@]}" true; then
    echo "[sync] 无法 ssh 到 $HOST" >&2
    return 1
  fi
  local running
  running=$("${SSH[@]}" "pgrep -af '[p]ython3 .*main\.py' || true" || true)
  if [[ -n "$running" && $FORCE -eq 0 ]]; then
    echo "[sync] 远端仍有程序在跑，拒绝推送：" >&2
    echo "$running" >&2
    echo "[sync] 先在该终端 Ctrl+C 停止程序；确认安全后可加 --force" >&2
    return 1
  fi

  local ts backup
  ts=$(date +%Y%m%d%H%M%S)
  backup="/home/robot/$(basename "$DEST").bak.$ts.tgz"
  if "${SSH[@]}" "test -d '$DEST'"; then
    echo "[sync] 备份远端 -> $backup"
    "${SSH[@]}" "cd '$DEST' && tar czf '$backup' ." \
      || { echo "[sync] 远端备份失败，已中止" >&2; return 1; }
  else
    echo "[sync] 远端 $DEST 不存在，将新建"
  fi

  local n; n=$(local_list | wc -l)
  echo "[sync] 将推送 $n 个源文件到 $HOST:$DEST"
  if [[ $YES -eq 0 ]]; then
    local ans
    read -r -p "确认安全、机器人已停止后输入 PUSH：" ans
    [[ "$ans" == "PUSH" ]] || { echo "[sync] 已取消"; return 2; }
  fi

  tar czf - \
    --exclude='__pycache__' --exclude='*.pyc' \
    --exclude='rgb_images' --exclude='depth_images' \
    --exclude='legacy' --exclude='decisions.old_*' --exclude='.vscode' \
    --exclude='robot_snapshot' --exclude='.sync_*' . | "${SSH[@]}" "mkdir -p '$DEST' && tar xzf - -C '$DEST'"

  if [[ $PRUNE_REMOTE -eq 1 ]]; then
    local tmp; tmp=$(mktemp -d); trap 'rm -rf "$tmp"' RETURN
    local_manifest | awk '{print $2}' | LC_ALL=C sort > "$tmp/local_paths"
    remote_manifest | awk '{print $2}' | LC_ALL=C sort > "$tmp/remote_paths"
    awk 'NR==FNR{l[$1]=1; next} !($1 in l)' "$tmp/local_paths" "$tmp/remote_paths" > "$tmp/remote_only"
    if [[ -s "$tmp/remote_only" ]]; then
      echo "[sync] 清理真机多余源文件（已备份）："
      sed 's/^/  - /' "$tmp/remote_only"
      if ! "${SSH[@]}" "cd '$DEST' && xargs -r rm -f --" < "$tmp/remote_only"; then
        echo "[sync] 删除真机多余文件失败" >&2
        return 1
      fi
    else
      echo "[sync] 真机没有多余源文件"
    fi
  fi

  echo "[sync] 推送完成，校验哈希..."
  if run_diff >/dev/null; then
    echo "[sync] 校验一致"
  else
    echo "[sync] 校验不一致，请执行 --diff 查看" >&2
    return 1
  fi

  if "${SSH[@]}" "cd '$DEST' && python3 -m compileall -q main.py decisions workers tools"; then
    echo "[sync] 远端语法检查通过"
  else
    echo "[sync] 远端语法检查失败（代码已推送，未删除；请检查 Python 版本）" >&2
    return 1
  fi
  echo "[sync] 完成。远端备份：$backup"
}

run_pull() {
  mkdir -p robot_snapshot
  "${SSH[@]}" "cd '$DEST' && tar czf - \
    --exclude='__pycache__' --exclude='*.pyc' \
    --exclude='rgb_images' --exclude='depth_images' \
    --exclude='legacy' --exclude='decisions.old_*' --exclude='robot_snapshot' ." \
    | tar xzf - -C robot_snapshot
  echo "[sync] 已拉取到 robot_snapshot/；用 diff -r 与本项目对比"
}

case "$MODE" in
  diff) run_diff ;;
  push) run_push ;;
  pull) run_pull ;;
esac
