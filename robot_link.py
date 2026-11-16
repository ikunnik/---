"""
和机器人说话的唯一入口。

两个实现：
    RobotLink   —— 真机，gRPC
    DryRunRobot —— 假机器人，只睡觉不动作，用于没接机器人时跑通整个框架

为什么执行器要自己拼 ExecuteSequence？
    SDK 的 velocity_sequence() / rotate() 都是阻塞的，拿不到"这次调用"的句柄，
    也就没法从别的线程 cancel()。所以这里直接用 stub，把 call 对象返回出去。

【谁在什么时候调用】
    main 线程（启动）: connect() -> stand() -> walk()
    main 线程（退出）: brake() -> close()（close 会恢复内置避障）
    executor 线程：    execute_stream()，返回的 call 可以被决策线程 cancel()
    决策线程：         get_pose()，读 yaw 做航向校验/回正
"""
import logging
import math
import threading
import time

import grpc
from dobot_quad.robot_client import RobotClient
from dobot_quad.proto import grpc_service_pb2 as pb2
from dobot_quad.proto.grpc_service_pb2_grpc import gRPCServiceStub

from actions import stop
from messages import Motion, Pose, Velocity, VelocityPlan
from motion.api import MotionCancelled, MotionProgress


class _GrpcMotionCall:
    """把 gRPC 流适配成 MotionCall：核心不再依赖 grpc.RpcError。"""

    def __init__(self, call):
        self._call = call

    def cancel(self):
        self._call.cancel()

    def __iter__(self):
        try:
            for progress in self._call:
                yield MotionProgress(
                    bool(getattr(progress, "is_final", True)),
                    bool(getattr(progress, "success", False)),
                    str(getattr(progress, "message", "")),
                )
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.CANCELLED:
                raise MotionCancelled(str(e)) from None
            raise


class RobotLink:
    """真机连接。一个实例 = 一条 gRPC 通道 + 一个高层客户端。"""

    def __init__(
        self,
        ip: str,
        speed_ratio: int = 60,
        pose_timeout_s: float = 0.30,
        pose_cache_s: float = 0.05,
        client_factory=RobotClient,
    ):
        self.ip = ip
        self.speed_ratio = speed_ratio
        self.pose_timeout_s = pose_timeout_s
        self.pose_cache_s = pose_cache_s
        self._client_factory = client_factory
        self._client = None
        self._channel = None
        self._stub = None
        self._oa_before = None
        self._speed_ratio_before = None
        self._pose = None
        self._pose_ts = 0.0
        self._pose_lock = threading.Lock()
        self._warn_ts = 0.0

    # ---------------- 生命周期 ----------------
    def connect(self):
        # main 线程第 [2] 步调用；这里建立两条连接：
        #   1) 高层客户端：起立、切模式、刹车这些一次性动作
        #   2) 底层 stub：跑速度/动作流，需要 call 句柄才能 cancel
        self._client = self._client_factory(self.ip)

        # 先记下进入前的内置避障/速度档位，退出时恢复，避免影响其他程序。
        # 读不到就失败退出：不知道原来的状态，不能假装“关闭过”。
        state = self._client.get_state()
        if not getattr(state, "success", False):
            raise RuntimeError(
                "读取机器人状态失败，无法确认内置避障/速度档位"
            )
        self._oa_before = bool(state.obstacle_avoidance_enabled)
        self._speed_ratio_before = int(state.current_speed_ratio)

        resp = self._client.set_obstacle_avoidance(False)   # 运行期间关闭，用我们自己的
        if not getattr(resp, "success", False) or bool(
            getattr(resp, "current_enabled", True)
        ):
            raise RuntimeError("关闭机器人内置避障失败")

        resp = self._client.set_speed_ratio(self.speed_ratio)
        if not getattr(resp, "success", False):
            raise RuntimeError(
                f"设置机器人速度档位 {self.speed_ratio} 失败"
            )

        # 底层通道：跑速度/动作流，需要拿到 call 句柄才能 cancel
        self._open_channel()

    def _open_channel(self):
        self._channel = grpc.insecure_channel(self.ip)
        grpc.channel_ready_future(self._channel).result(timeout=5.0)
        self._stub = gRPCServiceStub(self._channel)

    @staticmethod
    def _require_success(result, action: str):
        if result is None or not bool(getattr(result, "success", False)):
            message = "" if result is None else getattr(result, "message", "")
            raise RuntimeError(
                f"{action}失败" + (f": {message}" if message else "")
            )

    def stand(self):
        # main 线程第 [2] 步调用，阻塞到起立完成
        self._require_success(self._client.balance_stand(), "起立")

    def walk(self):
        # main 线程第 [2] 步调用，切到行走模式
        self._require_success(self._client.walk(), "切换到行走模式")

    # ---------------- 指令流 ----------------
    @staticmethod
    def _add_motion(sequence, motion_id: str, params):
        """把 motion_id 和参数写进 protobuf；类型必须和 proto 的 oneof 对上。"""
        motion = sequence.motions.add()
        motion.motion_id = motion_id
        for key, value in params:
            param = motion.parameters.add()
            param.key = key
            if isinstance(value, bool):          # bool 要在 int 前面判断
                param.bool_value = value
            elif isinstance(value, int):
                param.int_value = value
            elif isinstance(value, float):
                param.float_value = value
            elif isinstance(value, str):
                param.string_value = value
            else:
                raise TypeError(f"参数 {key} 类型不支持: {type(value).__name__}")

    def build_request(self, cmd):
        """把 Velocity 或 Motion 转成一次 ExecuteSequence 请求。"""
        seq_id = f"cmd_{time.time_ns()}"   # 每次唯一，避免服务端去重
        req = pb2.ExecuteSequenceRequest()
        req.immediate_start = True
        req.sequence.sequence_id = seq_id
        req.sequence.sequence_name = seq_id
        req.sequence.loop = False

        if isinstance(cmd, (Velocity, VelocityPlan)):
            req.sequence.loop = isinstance(cmd, VelocityPlan) and cmd.loop
            self._add_motion(
                req.sequence,
                "walk_velocity_seq",
                (("velocity_sequence", cmd.to_string()),),
            )
        elif isinstance(cmd, Motion):
            self._add_motion(req.sequence, cmd.motion_id, cmd.params)
        else:
            raise TypeError(f"不支持的指令类型: {type(cmd).__name__}")
        return req

    def execute_stream(self, cmd):
        """发起一次流式调用，返回可以 .cancel() 的迭代器。"""
        # executor 线程调用；决策线程拿到同一个 call 后可以 cancel()
        return _GrpcMotionCall(self._stub.ExecuteSequence(self.build_request(cmd)))

    # ---------------- 姿态 ----------------
    def get_pose(self, refresh: bool = False):
        """返回 Pose 或 None。

        平时用 50ms 缓存，避免决策 50Hz 打爆 GetRobotState；
        动作完成需要校验航向时传 refresh=True，强制读一次新姿态。
        """
        now = time.time()
        with self._pose_lock:
            if (
                not refresh
                and self._pose is not None
                and (now - self._pose_ts) < self.pose_cache_s
            ):
                return self._pose

        if self._stub is None:
            return None

        try:
            res = self._stub.GetRobotState(
                pb2.GetRobotStateRequest(), timeout=self.pose_timeout_s
            )
        except Exception as e:
            if now - self._warn_ts >= 1.0:
                logging.warning("[机器人] 读取姿态失败: %s", e)
                self._warn_ts = now
            return None

        if not res.success:
            return None
        ori = res.robot_state.ori_body
        if len(ori) < 3:
            return None

        pos = getattr(res.robot_state, "pos_body", ())
        x = float(pos[0]) if len(pos) >= 2 else float("nan")
        y = float(pos[1]) if len(pos) >= 2 else float("nan")
        pose = Pose(yaw_rad=float(ori[2]), ts=now, x=x, y=y)
        with self._pose_lock:
            self._pose = pose
            self._pose_ts = now
        return pose

    # ---------------- 收尾 ----------------
    def brake(self):
        # main 线程退出前调用：发一条零速，防止机器人还在动
        if self._client is None:
            return
        try:
            self._client.velocity_sequence(
                stop(0.5).to_string(), stand_down_after=False, show_progress=False
            )
        except Exception as e:
            print(f"[robot] 刹车失败: {e}")

    def close(self):
        # 恢复进入前的速度档位和内置避障状态；失败也不阻塞退出。
        if self._client is not None and self._speed_ratio_before is not None:
            try:
                resp = self._client.set_speed_ratio(self._speed_ratio_before)
                if getattr(resp, "success", False):
                    logging.info(
                        "[机器人] 已恢复速度档位: %s",
                        self._speed_ratio_before,
                    )
                else:
                    logging.warning("[机器人] 恢复速度档位失败")
            except Exception as e:
                logging.warning("[机器人] 恢复速度档位失败: %s", e)

        if self._client is not None and self._oa_before is not None:
            try:
                resp = self._client.set_obstacle_avoidance(self._oa_before)
                if getattr(resp, "success", False):
                    logging.info("[机器人] 已恢复内置避障: %s", self._oa_before)
                else:
                    logging.warning("[机器人] 恢复内置避障失败")
            except Exception as e:
                logging.warning("[机器人] 恢复内置避障失败: %s", e)

        if self._client is not None:
            try:
                self._client.channel.close()
            except Exception:
                pass
            self._client = None

        if self._channel is not None:
            self._channel.close()
            self._channel = None


# =====================================================================
# 兼容旧导入：实现已移到 motion.dry_run
# =====================================================================
from motion.dry_run import DryRunRobot, _FakeCall  # noqa: E402,F401
