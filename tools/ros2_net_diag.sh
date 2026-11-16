#!/usr/bin/env bash
# ROS 2 多机通信分层自检（只读）
# 用法：两端各跑一次，把输出并排 diff，差异就是故障层。
#   bash tools/ros2_net_diag.sh            # 默认检查 /guide/perception 命名空间
#   bash tools/ros2_net_diag.sh /my/ns     # 指定命名空间
# 不启停任何节点、不改任何环境变量、不发送运动指令。
set -u

NS="${1:-/guide/perception}"
RESULTS=()

section() { printf '\n===== %s =====\n' "$1"; }
item() { RESULTS+=("$1=$2"); printf '%-28s %s\n' "$1" "$2"; }

section "0 主机与网络（L0 环境层）"
item host "$(uname -n 2>/dev/null || cat /etc/hostname 2>/dev/null || echo unknown)"
item time "$(date -Is)"
printf '%-28s %s\n' "IPv4 地址" "$(ip -4 -o addr show 2>/dev/null | awk '{printf "%s=%s ", $2, $4}')"
printf '%-28s %s\n' "默认路由" "$(ip route 2>/dev/null | awk '/^default/{print $3; exit}')"
printf '%-28s %s\n' "多网卡同网段冲突" "$(ip -4 -o addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | cut -d. -f1-3 | sort | uniq -d | tr '\n' ' ')"
printf '%-28s %s\n' "rp_filter(all)" "$(sysctl -n net.ipv4.conf.all.rp_filter 2>/dev/null)"

section "1 环境三件套（L1 域与中间件层）"
item ROS_DISTRO "${ROS_DISTRO:-<未设置>}"
# 未设置时 rclpy 默认 0，但两端必须"实际值"一致，这里显式打印
item ROS_DOMAIN_ID "${ROS_DOMAIN_ID:-<未设置，默认 0>}"
item RMW_IMPLEMENTATION "${RMW_IMPLEMENTATION:-<未设置，默认 rmw_fastrtps_cpp>}"
item ROS_LOCALHOST_ONLY "${ROS_LOCALHOST_ONLY:-<未设置，默认 0>}"
item ROS_AUTOMATIC_DISCOVERY_RANGE "${ROS_AUTOMATIC_DISCOVERY_RANGE:-<未设置>}"
item CYCLONEDDS_URI "${CYCLONEDDS_URI:-<未设置>}"

# CycloneDDS 配置四项：网卡钉死、组播、主题发现、participant 上限
DDS_TEXT=""
case "${CYCLONEDDS_URI:-}" in
  file://*) path="${CYCLONEDDS_URI#file://}"; [ -r "$path" ] && DDS_TEXT="$(cat "$path")" || echo "  ! 配置指向的文件不可读: $path" ;;
  '') : ;;
  *) DDS_TEXT="$CYCLONEDDS_URI" ;;
esac
if [ -n "$DDS_TEXT" ]; then
  for key in NetworkInterface AllowMulticast EnableTopicDiscoveryEndpoints MaxAutoParticipantIndex; do
    if printf '%s' "$DDS_TEXT" | grep -qi "$key"; then
      item "dds.$key" "$(printf '%s' "$DDS_TEXT" | grep -oi "<$key>[^<]*</$key>" | head -1)"
    else
      item "dds.$key" "缺失"
    fi
  done
else
  echo "  ! 未设置 CYCLONEDDS_URI：CycloneDDS 会自动选网卡，多网卡机器上常选错"
fi

section "2 ROS 图（L2 发现层）"
if ! command -v ros2 >/dev/null 2>&1; then
  item ros2 "<未安装或未 source /opt/ros/humble/setup.bash>"
  echo "  ! 本机没有任何 ROS 2 命令，本层的节点/话题检查无法进行"
else
  item ros2 "$(command -v ros2)"
  item daemon "$(ros2 daemon status 2>&1 | tr '\n' ' ')"
  echo "-- node list --"; ros2 node list 2>&1 | head -20
  echo "-- topic list（前20）--"; ros2 topic list 2>&1 | head -20
  echo "-- 目标命名空间 --"; ros2 topic list 2>&1 | grep -F "$NS" || echo "  (无匹配，命名空间不一致或对端未发布)"
fi

section "3 QoS 对拍（L3 兼容层）"
if command -v ros2 >/dev/null 2>&1; then
  for t in $(ros2 topic list 2>/dev/null | grep -F "$NS"); do
    echo "-- $t --"
    # 关键：发布端 best_effort / 订阅端 reliable 时不兼容，这里两种都试
    ros2 topic info -v "$t" 2>&1 | sed -n '1,12p'
    timeout 3 ros2 topic hz --qos-reliability best_effort "$t" 2>&1 | tail -2
  done
  [ -z "$(ros2 topic list 2>/dev/null | grep -F "$NS")" ] && echo "  (命名空间下无话题，跳过)"
fi

section "4 TF 与时间（L4 使用层）"
if command -v ros2 >/dev/null 2>&1; then
  item tf_topic "$(ros2 topic list 2>/dev/null | grep -c '^/tf$\|^/tf_static$')"
  echo "-- tf 帧（2s 采样）--"
  timeout 3 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id" | head -6 \
    || echo "  (无 /tf_static，静态变换缺失)"
  item use_sim_time "$(ros2 param get /$(ros2 node list 2>/dev/null | head -1 | tr -d '/') use_sim_time 2>&1 | tail -1)"
fi

section "汇总（用于两端 diff）"
printf '%s\n' "${RESULTS[@]}" | sort
