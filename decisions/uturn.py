"""三段式掉头：左转90° → 前进配置距离 → 左转90°；逐段等回执和新感知。"""
import math
from dataclasses import replace

from actions import Actions
from decisions.rules import wrap_pi
from decisions.types import StepResult


class UTurnController:
    def __init__(self, cfg):
        self.cfg = cfg
        self.actions = Actions(cfg)
        self.index = 0
        self.pending = None
        self.started_ts = 0.0
        self.start_yaw = None
        self.entry_yaw = None
        self.finished_ts = None
        self.completed = False
        self.failed = False
        self.reason = ''
        self.last_stop = -math.inf
        self.ignore_obstacles = bool(
            getattr(cfg, "uturn_ignore_obstacles", False)
        )
        # 掉头分级：本段补转次数、补转目标角与当前段期望角
        self.retries = 0
        self._correction = None
        self._expected_turn_deg = 90.0

    @property
    def has_pending(self):
        return self.pending is not None

    def _stop(self, now, *, interrupt=False):
        request = None
        if interrupt or now-self.last_stop >= self.cfg.stop_refresh_s:
            self.last_stop = now
            request = self.actions.stop()
        return StepResult('UTURN_FAILED' if self.failed else 'UTURN_DONE', request,
                          interrupt=interrupt, log=self.reason or None)

    def _fail(self, now, reason):
        self.failed = True
        self.reason = reason
        self.pending = None
        return self._stop(now, interrupt=True)

    def step(self, data):
        now = data.now
        if self.failed or self.completed:
            return self._stop(now)
        per, pose = data.perception, data.pose
        if not self.ignore_obstacles:
            if (per is None or not per.valid or not math.isfinite(data.perception_ts)
                    or not 0 <= now-data.perception_ts < self.cfg.stale_s):
                return self._fail(now, '掉头时深度失效，锁停')
        if (pose is None or not math.isfinite(pose.yaw_rad) or not math.isfinite(pose.ts)
                or not 0 <= now-pose.ts <= self.cfg.uturn_pose_stale_s):
            return self._fail(now, '掉头时姿态失效，锁停')
        # 转向检查周围净空；直行必须留出命令距离和危险距离。
        # ignore_obstacles=True 时跳过这些深度检查，只保留姿态校验。
        if not self.ignore_obstacles:
            if self.index in (0, 2):
                if not math.isfinite(per.min_m) or per.min_m < self.cfg.rotate_clear_m:
                    return self._fail(
                        now,
                        f'掉头转向净空不足（min={per.min_m:.2f}m < '
                        f'{self.cfg.rotate_clear_m:.2f}m），锁停',
                    )
            elif self.cfg.uturn_check_forward_space:
                # 默认不检查直行空间：掉头只要求转向净空，
                # 直行 1m 由操作员确认场地空旷（可用
                # --uturn-check-forward-space 恢复检查）。
                distance = per.distance_m
                if not math.isfinite(distance) or distance < self.cfg.danger_m:
                    return self._fail(
                        now,
                        f'掉头直行前方受阻（{distance:.2f}m < '
                        f'danger {self.cfg.danger_m:.2f}m），锁停',
                    )
                required = (self.cfg.danger_m + self.cfg.uturn_forward_m
                            + self.cfg.safety_buffer_m)
                if self.pending is None and distance < required:
                    return self._fail(
                        now,
                        f'掉头直行空间不足（{distance:.2f}m < '
                        f'{required:.2f}m），锁停；'
                        f'可用 --uturn-ignore-obstacles 跳过深度检查',
                    )
        if self.entry_yaw is None:
            self.entry_yaw = pose.yaw_rad
        if self.pending is not None:
            result = data.result
            if (result is not None and result.tag == self.pending.cmd.tag
                    and result.ts >= self.started_ts and result.status != 'running'):
                if result.status != 'ok':
                    return self._fail(now, f'掉头动作{result.status}，锁停')
                if pose.ts < result.ts:
                    # 需要动作结束后重新读取的姿态，不能拿旧姿态确认成功。
                    if now-self.started_ts >= self.pending.timeout_s:
                        return self._fail(now, '掉头完成后姿态未刷新，锁停')
                    return StepResult('UTURN_WAIT_POSE')
                if self.index in (0, 2):
                    delta = math.degrees(wrap_pi(pose.yaw_rad-self.start_yaw))
                    error = delta - self._expected_turn_deg
                    tolerance = max(self.cfg.yaw_tol_deg,
                                    abs(self._expected_turn_deg) * 0.10)
                    if abs(error) > tolerance:
                        # 小偏差先补转一次；补转仍不准则锁停。
                        can_retry = (
                            abs(error) <= self.cfg.uturn_retry_tol_deg
                            and self.retries < self.cfg.uturn_max_retries
                        )
                        if can_retry:
                            self.retries += 1
                            self._correction = -error
                            self.pending = None
                            return StepResult(
                                'UTURN_RETRY',
                                log=(f'掉头角度补转 {self._correction:+.1f}°'
                                     f'（实际 {delta:.1f}°，期望 {self._expected_turn_deg:.1f}°）'),
                            )
                        return self._fail(
                            now, f'掉头左转角度异常：{delta:.1f}°，锁停'
                        )
                else:
                    drift = math.degrees(wrap_pi(pose.yaw_rad-self.start_yaw))
                    if abs(drift) > self.cfg.yaw_tol_deg:
                        return self._fail(now, '掉头直行航向偏差过大，锁停')
                self.pending = None
                self.index += 1
                self.retries = 0
                self.finished_ts = now
                if self.index == 3:
                    error = abs(math.degrees(wrap_pi(pose.yaw_rad-self.entry_yaw-math.pi)))
                    if error > self.cfg.yaw_tol_deg:
                        return self._fail(now, '掉头总航向未达到180°，锁停')
                    self.completed = True
                    self.reason = '两次左转和短距离直行均完成'
                    return self._stop(now, interrupt=True)
                return StepResult('UTURN_SETTLE')
            if now-self.started_ts >= self.pending.timeout_s:
                return self._fail(now, '掉头动作超时，锁停')
            return StepResult('UTURN_WAIT')
        if self.finished_ts is not None:
            if now-self.finished_ts < self.cfg.settle_s or data.perception_ts <= self.finished_ts:
                if now-self.finished_ts > self.cfg.uturn_sensor_wait_s:
                    return self._fail(now, '掉头下一步等待新感知超时，锁停')
                return StepResult('UTURN_SETTLE')
        if data.executor_busy:
            return StepResult('UTURN_WAIT_IDLE')
        if self._correction is not None:
            delta = self._correction
            self._correction = None
            self._expected_turn_deg = delta
            request = self.actions.turn(delta, purpose='掉头角度补正')
            state = 'UTURN_RETRY'
        elif self.index in (0, 2):
            self._expected_turn_deg = 90.0
            request = self.actions.turn(90, purpose='掉头左转90°')
            state = ('UTURN_LEFT_1', 'UTURN_FORWARD', 'UTURN_LEFT_2')[self.index]
        else:
            request = self.actions.move(
                'forward', self.cfg.uturn_forward_m, purpose='掉头短距离直行')
            state = 'UTURN_FORWARD'
        # 每段（含补转）使用不同tag，拒绝上一段的迟到结果。
        cmd = replace(
            request.cmd,
            tag=f'uturn_{self.index}_{self.retries}_{request.cmd.tag}',
        )
        self.pending = replace(request, cmd=cmd)
        self.started_ts = now
        self.start_yaw = pose.yaw_rad
        return StepResult(state, self.pending)
