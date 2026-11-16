"""ROS 2 运动后端：实时运动走 `/planner/vel_cmd`，姿态仍走 SDK。

设计要点：
    - 实时运动不碰 gRPC：gRPC 是串行队列，长动作会阻塞起立/蹲下。
    - 起立/切 WALK/读 SDK 姿态仍复用 RobotLink；运动只发 Twist。
    - 20 Hz 定时器持续发布，既是控制也是保活；取消时立刻发零速。
    - 距离/角度用 `/foot_odom/data` 闭环，不依赖 SDK 的阻塞动作。
    - 退出/取消/异常统一先发零速，再交给 RobotLink 收尾。

依赖（ROS 2 Humble）：
    rclpy、geometry_msgs、nav_msgs
    运行前要 source /opt/ros/humble/setup.bash，并设置 ROS_DOMAIN_ID=0、
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp；DDS 网卡最好钉到 192.168.5.x。
"""
import logging
import math
import os
import re
import subprocess
import threading
import time

from messages import Motion, Pose, Velocity, VelocityPlan
from motion.api import MotionCancelled, MotionProgress

STOP_REPEAT = 3
STOP_INTERVAL = 0.1


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _quat_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def _clamp_vx(vx: float, min_vx: float) -> float:
    """速度死区：0 保持 0，非零但过小则抬到 min_vx（保留符号）。"""
    if abs(vx) < 1e-6:
        return 0.0
    if abs(vx) < min_vx:
        return math.copysign(min_vx, vx)
    return vx


def _ensure_ros2_dds():
    """没有设置 CYCLONEDDS_URI 时，自动钉到 192.168.5.x 网卡地址。

    本机直连时 enp0s20f0u4 同时有 192.168.1.100/192.168.5.100，
    CycloneDDS 自动选择经常选错，表现为“话题一个都发现不了”。
    """
    if not os.environ.get("RMW_IMPLEMENTATION"):
        os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        logging.info("[ROS2] 未设置 RMW_IMPLEMENTATION，使用 rmw_cyclonedds_cpp")

    if os.environ.get("CYCLONEDDS_URI"):
        return
    try:
        out = subprocess.check_output(
            ["ip", "-4", "-o", "addr"], text=True, timeout=2.0
        )
    except Exception:
        return
    for match in re.finditer(r"inet (192\.168\.5\.\d+)/", out):
        address = match.group(1)
        os.environ["CYCLONEDDS_URI"] = (
            "<CycloneDDS><Domain><General><Interfaces>"
            f'<NetworkInterface address="{address}"/>'
            "</Interfaces><AllowMulticast>spdp</AllowMulticast></General>"
            "<Discovery><EnableTopicDiscoveryEndpoints>true"
            "</EnableTopicDiscoveryEndpoints></Discovery></Domain></CycloneDDS>"
        )
        logging.info("[ROS2] 未设置 CYCLONEDDS_URI，已钉到 %s", address)
        return
    logging.warning(
        "[ROS2] 未找到 192.168.5.x 地址，DDS 可能选错网卡；"
        "请手动 export CYCLONEDDS_URI"
    )


class _Ros2Call:
    """一次动作流，接口与 gRPC 的流式 call 对齐：可迭代、可 cancel。"""

    def __init__(self, link, cmd):
        self._link = link
        self._cmd = cmd
        self._cancel = threading.Event()

    def cancel(self):
        # 不立刻归零：盲道重规划会紧接着发新目标，立刻归零会每 2~3 秒顿一下。
        # 这里只登记“取消时刻/版本”；_tick 发现没有新目标时再补零速。
        self._cancel.set()
        self._link._mark_cancel()

    def __iter__(self):
        self._link._run_cmd(self._cmd, self._cancel)
        yield MotionProgress(True, True, "")


class Ros2MotionBackend:
    """ROS 2 实时运动 + SDK 姿态的组合后端。"""

    def __init__(self, ip: str, speed_ratio: int = 60,
                 pose_timeout_s: float = 0.30,
                 pose_cache_s: float = 0.05, cfg=None):
        self.ip = ip
        self.speed_ratio = speed_ratio
        self.pose_timeout_s = pose_timeout_s
        self.pose_cache_s = pose_cache_s
        self.cfg = cfg
        self._sdk = None
        self._node = None
        self._pub = None
        self._spin_thread = None
        self._target = (0.0, 0.0, 0.0)
        self._target_lock = threading.Lock()
        self._target_version = 0
        self._cancel_pending = None   # (取消时刻, 取消时的目标版本)
        self._odom = None          # (x, y, yaw, ts)
        self._odom_lock = threading.Lock()
        self._closed = False

    # ------------------------------------------------------------ 生命周期
    def connect(self):
        from motion.posture_sdk import SdkPosture
        self._sdk = SdkPosture(
            self.ip, self.speed_ratio,
            pose_timeout_s=self.pose_timeout_s,
            pose_cache_s=self.pose_cache_s,
        )
        self._sdk.connect()
        try:
            self._init_ros()
        except Exception:
            # 起立前失败：把 SDK 改过的内置避障/速度恢复掉
            try:
                self._sdk.close()
            finally:
                self._sdk = None
            raise

    def _init_ros(self):
        _ensure_ros2_dds()
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        except Exception:
            rclpy.init(args=[])

        self._node = Node("rover_x1_motion", use_global_arguments=False)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self._node.create_publisher(
            Twist, str(self.cfg.ros2_vel_topic), qos
        )
        sensor_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self._node.create_subscription(
            Odometry, str(self.cfg.ros2_odom_topic), self._on_odom, sensor_qos
        )
        rate = max(1.0, float(self.cfg.ros2_rate_hz))
        self._node.create_timer(1.0 / rate, self._tick)

        self._spin_thread = threading.Thread(
            target=self._spin, name="ros2_spin", daemon=True
        )
        self._spin_thread.start()

        deadline = time.monotonic() + float(self.cfg.ros2_startup_timeout_s)
        while time.monotonic() < deadline:
            if self._node.count_subscribers(str(self.cfg.ros2_vel_topic)) > 0:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(
                f"ROS 2 未发现 {self.cfg.ros2_vel_topic} 订阅者；"
                "检查 ROS_DOMAIN_ID / RMW_IMPLEMENTATION / CYCLONEDDS_URI"
            )
        logging.info(
            "[ROS2] 已连接 %s，发布 %s @ %.0fHz",
            self.cfg.ros2_vel_topic, self.cfg.ros2_vel_topic, rate,
        )

    def _spin(self):
        import rclpy
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def stand(self):
        self._sdk.stand()

    def walk(self):
        self._sdk.walk()

    # ------------------------------------------------------------ 发布
    def _tick(self):
        if self._pub is None or self._closed:
            return
        from geometry_msgs.msg import Twist
        with self._target_lock:
            pending = self._cancel_pending
            if pending is not None:
                cancel_ts, cancel_version = pending
                if self._target_version != cancel_version:
                    self._cancel_pending = None  # 新目标已接上，无需补零
                elif time.monotonic() - cancel_ts >= float(
                    self.cfg.ros2_cancel_zero_delay_s
                ):
                    self._target = (0.0, 0.0, 0.0)
                    self._cancel_pending = None
            vx, vy, vyaw = self._target
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(vyaw)
        self._pub.publish(msg)

    def _set_target(self, vx, vy=0.0, vyaw=0.0):
        with self._target_lock:
            self._target = (float(vx), float(vy), float(vyaw))
            self._target_version += 1
            self._cancel_pending = None

    def _mark_cancel(self):
        with self._target_lock:
            self._cancel_pending = (time.monotonic(), self._target_version)

    def _publish_zero(self):
        if self._pub is None:
            return
        self._set_target(0.0, 0.0, 0.0)
        from geometry_msgs.msg import Twist
        for _ in range(STOP_REPEAT):
            self._pub.publish(Twist())
            time.sleep(STOP_INTERVAL)

    # ------------------------------------------------------------ 里程计
    def _on_odom(self, msg):
        pose = msg.pose.pose
        p, q = pose.position, pose.orientation
        with self._odom_lock:
            self._odom = (float(p.x), float(p.y),
                          _quat_yaw(q.x, q.y, q.z, q.w), time.time())

    def _odom_fresh(self):
        with self._odom_lock:
            odom = self._odom
        if odom is None:
            return None
        if time.time() - odom[3] > float(self.cfg.ros2_odom_stale_s):
            return None
        return odom

    def _yaw(self):
        odom = self._odom_fresh()
        if odom is not None:
            return odom[2]
        pose = self._sdk.get_pose(refresh=True) if self._sdk else None
        return pose.yaw_rad if pose is not None else None

    def get_pose(self, refresh: bool = False):
        odom = self._odom_fresh()
        if odom is not None:
            return Pose(yaw_rad=odom[2], ts=odom[3], x=odom[0], y=odom[1])
        return self._sdk.get_pose(refresh=refresh) if self._sdk else None

    # ------------------------------------------------------------ 动作流
    def execute_stream(self, cmd):
        return _Ros2Call(self, cmd)

    def _run_cmd(self, cmd, cancel):
        cancelled = False
        try:
            if isinstance(cmd, Velocity):
                self._run_velocity(cmd, cancel)
            elif isinstance(cmd, VelocityPlan):
                self._run_plan(cmd, cancel)
            elif isinstance(cmd, Motion):
                self._run_motion(cmd, cancel)
            else:
                raise TypeError(f"ROS2 不支持指令: {type(cmd).__name__}")
        except MotionCancelled:
            cancelled = True
            raise
        finally:
            # 被替换时不归零，让新计划无缝接上；没有新目标时 _tick 会补零。
            if not cancelled:
                self._set_target(0.0, 0.0, 0.0)

    def _wait(self, seconds, cancel):
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise MotionCancelled()
            time.sleep(0.02)
        if cancel.is_set():
            raise MotionCancelled()

    def _run_velocity(self, cmd: Velocity, cancel):
        vx = _clamp_vx(cmd.vx, float(self.cfg.ros2_min_vx))
        if abs(vx) > 1e-6 and abs(vx) < abs(cmd.vx):
            logging.warning("[ROS2] vx=%.2f 低于死区，抬到 %.2f",
                            cmd.vx, vx)
        self._set_target(vx, cmd.vy, cmd.vyaw)
        self._wait(cmd.duration, cancel)

    def _run_plan(self, plan: VelocityPlan, cancel):
        while not cancel.is_set():
            for step in plan.steps:
                if cancel.is_set():
                    raise MotionCancelled()
                self._run_velocity(step, cancel)
            if not plan.loop:
                return
        raise MotionCancelled()

    def _run_motion(self, cmd: Motion, cancel):
        if cmd.motion_id == "rotation":
            self._run_rotation(cmd, cancel)
        elif cmd.motion_id == "line_walk":
            self._run_line(cmd, cancel)
        else:
            raise RuntimeError(f"ROS2 未实现动作: {cmd.motion_id}")

    def _run_rotation(self, cmd: Motion, cancel):
        params = dict(cmd.params)
        direction = int(params.get("direction", 0))
        angle = math.radians(float(params.get("angle", 0.0)))
        if angle <= 1e-6:
            return
        yaw0 = self._yaw()
        if yaw0 is None:
            raise RuntimeError("ROS2 旋转失败：没有里程计/SDK 姿态")
        yaw_rate = abs(float(self.cfg.ros2_yaw_rate))
        sign = 1.0 if direction == 0 else -1.0
        self._set_target(0.0, 0.0, sign * yaw_rate)
        timeout = max(5.0, angle / yaw_rate + 3.0)
        deadline = time.monotonic() + timeout
        yaw_prev = yaw0
        turned = 0.0
        try:
            while True:
                if cancel.is_set():
                    raise MotionCancelled()
                yaw = self._yaw()
                if yaw is not None:
                    # 逐帧累加，支持 circle() 的 360°（单次差值会被 wrap 成 0）
                    turned += _wrap_pi(yaw - yaw_prev)
                    yaw_prev = yaw
                    if abs(turned) >= angle:
                        return
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"ROS2 旋转超时：目标 {math.degrees(angle):.0f}°，"
                        f"已转 {math.degrees(abs(turned)):.0f}°"
                    )
                time.sleep(0.02)
        finally:
            self._set_target(0.0, 0.0, 0.0)

    def _run_line(self, cmd: Motion, cancel):
        params = dict(cmd.params)
        direction = int(params.get("direction", 0))
        distance = max(0.0, float(params.get("distance", 0.0)))
        if distance <= 1e-6:
            return
        odom = self._odom_fresh()
        if odom is None:
            raise RuntimeError("ROS2 距离移动失败：没有新鲜里程计")
        speed = max(0.3, float(self.cfg.ros2_line_vx))
        if direction == 0:
            vx, vy = speed, 0.0
        elif direction == 1:
            vx, vy = -speed, 0.0
        elif direction == 2:
            vx, vy = 0.0, speed
        else:
            vx, vy = 0.0, -speed
        x0, y0 = odom[0], odom[1]
        self._set_target(vx, vy, 0.0)
        timeout = max(5.0, distance / speed + 3.0)
        deadline = time.monotonic() + timeout
        moved = 0.0
        last_log = 0.0
        try:
            while True:
                if cancel.is_set():
                    raise MotionCancelled()
                now = self._odom_fresh()
                if now is None:
                    raise RuntimeError("ROS2 距离移动中里程计失效，停车")
                moved = math.hypot(now[0] - x0, now[1] - y0)
                if moved >= distance:
                    return
                if time.monotonic() - last_log >= 0.5:
                    last_log = time.monotonic()
                    logging.info(
                        "[ROS2] line_walk 目标 %.2fm，已走 %.2fm", distance, moved
                    )
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"ROS2 距离移动超时：目标 {distance:.2f}m，"
                        f"实际 {moved:.2f}m（检查是否被挡/里程计不动）"
                    )
                time.sleep(0.02)
        finally:
            self._set_target(0.0, 0.0, 0.0)

    # ------------------------------------------------------------ 收尾
    def brake(self):
        self._publish_zero()
        if self._sdk is not None:
            try:
                self._sdk.brake()
            except Exception as e:
                logging.warning("[ROS2] SDK 刹车失败: %s", e)

    def close(self):
        self._closed = True
        try:
            self._publish_zero()
        except Exception:
            pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._sdk is not None:
            self._sdk.close()
            self._sdk = None


# 兼容旧导入名（tools/tests 仍在用）
Ros2RobotLink = Ros2MotionBackend
