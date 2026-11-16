# Phase 1–2 重构记录

## Phase 1：运动后端收口

新增：

```
motion/
  api.py            MotionBackend / MotionCall / MotionProgress / MotionCancelled
  ros2_backend.py   Twist、里程计闭环、取消（原 ros2_robot_link.py）
  posture_sdk.py    只做起立/切 WALK/姿态/刹车
  dry_run.py        假机器人
  factory.py        build_robot
```

改动：

- `workers/executor.py` 不再 import `grpc`，只捕获 `MotionCancelled`；
- `robot_link.py` 用 `_GrpcMotionCall` 把 gRPC CANCELLED 翻译成 `MotionCancelled`；
- `ros2_robot_link.py` 变成兼容 shim，工具/测试旧导入不破坏；
- `robot_factory.py` 变成 `motion.factory` shim；
- 默认 `motion_backend=ros2`。

## Phase 2：main / runtime 拆分

```
app/
  cli.py            参数与 Config 生成
runtime/
  assembly.py       队列/事件/worker/decision 装配
  lifecycle.py      连接、启动、监视、逆序停止
  readiness.py      检测器第一帧握手
  dds.py            CYCLONEDDS_URI 修正
  app.py            装配 → 连接 → 启动 → 就绪 → 监视
main.py             兼容入口（约 50 行）
```

兼容层：

- `main.CFG`、`main.build_robot`、`main._parse_args`、`main._build_cfg`、
  `main._wait_detector_ready`、`main._build_runtime`、`main._shutdown` 保留；
- `tools/test_mangdao.py` 改为 patch `runtime.assembly` / `runtime.readiness`。

## 验证

```bash
bash tools/check.sh
python3 tools/test_motion_api.py
python3 tools/test_ros2_link.py
python3 tools/test_mangdao.py     # 容器内（需要 grpc/audioop）
```

## 回滚

`docs/refactor/baseline.sha256` 记录了拆分前关键文件哈希。
