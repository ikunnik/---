# Phase 3：决策层拆分

## Traffic

```text
decisions/traffic/
  __init__.py      导出 TrafficSequence + 阶段常量
  constants.py     LINE / AT_LINE / ALIGN / SIGNAL / PASS / WAIT / DONE
  sequence.py      step() 门面：语音、守卫生效、阶段分发
  core.py          初始化、maybe_start、guard_sensor、动作结果、停车/锁存
  line.py          LINE_APPROACH
  align.py         AT_LINE + 停止线倾角对齐
  signal_.py       SIGNAL
  pass_.py         PASS + 2m/0.7m
```

## Avoidance

```text
decisions/avoidance/
  __init__.py      导出 AvoidStateMachine + 常量 + rules
  machine.py       组合 mixin 的门面
  base.py          __init__ / reset / step / context
  pending.py       pending 结果、取消、超时、航向校验、advance/realign
  transitions.py   HOLD 切换、BLOCKED、停车/锁定、历史/稳定窗口
  go.py            GO
  hold.py          HOLD
  plan.py          PLAN
  bypass.py        TURN / ADVANCE / CHECK
  path_return.py   RETURN / 回原路线
```

## 保持不变的接口

- `from decisions.traffic import TrafficSequence, WAIT_STOP_LINE`
- `from decisions.avoidance import AvoidStateMachine, GO, HOLD, ...`
- `StepResult` / `ActionRequest` / `PendingAction` 字段不变；
- `DecisionEngine` 不需要改。

## 旧文件

原文件已移到：

```
docs/refactor/legacy/traffic.py
docs/refactor/legacy/avoidance.py
```

仅作对照，不参与运行。

## 验证

```text
bash tools/check.sh                 通过
tools/test_decisions.py             19 tests OK
tools/test_mission_flow.py          OK
tools/test_path_return.py           5 tests OK
tools/test_stop_line_align.py       4 tests OK
tools/selftest.py                   全部通过（桩环境）
tools/test_mangdao.py               8 tests OK（桩环境）
```

真机验证仍需执行完整流程，重点：

- 停止线停车 + 倾角对齐；
- 绿灯 2m / 0.7m 提前停；
- 掉头；
- 绕障并肩 2m + 回原线；
- 盲道。
