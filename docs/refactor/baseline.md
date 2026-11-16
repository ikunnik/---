# 重构基线（Phase 0）

记录时间：2026-11-13
基线版本：默认运动后端 `ros2`，完整串联 `红绿灯 → 掉头 → 盲道`。

## 1. 基线命令

```bash
# 纯逻辑门禁
bash tools/check.sh

# 不接机器人跑完整决策链
python3 main.py --dry-run

# 真机完整流程（需 ROS2 环境与机器人）
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 tools/ros2_check.py --seconds 5
python3 main.py
```

## 2. 行为基线

| 环节 | 预期 |
|---|---|
| 启动 | 起立 → WALK → 停止线第一帧 → 盲道第一帧 → 执行器/调度/决策 |
| 停止线 | `row >= 0.83` 停车；停稳 0.25s；按黄线倾角对齐 |
| 红绿灯 | 绿灯稳定后 `line_walk 2.0m`；途中前方 `<0.7m` 提前停 |
| 掉头 | 左转 90° → 直行 1.0m → 左转 90°；失败锁停 |
| 盲道 | `VelocityPlan(loop=True)` 持续运行；无盲道直行 |
| 绕障 | 转向 → 至少并肩 2.0m → 回正 → 横向回原路线 |
| 退出 | Ctrl+C → 零速 → 逆序停 worker → 恢复速度档位/内置避障 |

## 3. 基线文件哈希

重构前请保存：

```bash
sha256sum main.py config.py actions.py robot_link.py ros2_robot_link.py \
  decisions/*.py workers/*.py > docs/refactor/baseline.sha256
```

当前关键文件行数（重构前）：

| 文件 | 行数 |
|---|---:|
| `main.py` | ~801 |
| `decisions/avoidance.py` | ~1168 |
| `decisions/traffic.py` | ~688 |
| `ros2_robot_link.py` | ~474 |
| `robot_link.py` | ~337 |
| `workers/executor.py` | ~238 |

## 4. 回滚

项目不是 git 仓库。每阶段开始前打包：

```bash
tar czf /tmp/rover_x1_pre_phaseN.tgz \
  --exclude='assets/voice' --exclude='__pycache__' .
```

Phase 1/2 只改代码与文档，不改 `assets/` 与机器人文件。
