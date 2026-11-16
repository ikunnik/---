# ROS 迁移阶段1记录

日期：2026-11-16。状态：配置、首批接口与候选基线已建立；完整算法回归存在失败，阶段1尚未验收。

## 已实施

- `config.py` 作为默认配置的唯一来源，`tools/generate_ros_config.py` 生成 `config/rover.yaml`。生成前已核对两者全部现有参数一致，没有修改实际调参值。
- ROS默认文件漂移检查已纳入测试。部署调参使用外部YAML，通过 `params_file` 指定。
- 原掉头/太近语音用例按 `rotate_clear_m` 构造实际近障碍输入；通过距离断言改为比较 `traffic_pass_m`。保留停止、失败原因、动作距离及完成断言。
- 新建可编译消息包、感知/语音/结果转换函数和时间/任务身份检查器，详见 [接口契约](ros2_interfaces.md)。运行时尚未切换到新接口。
- `tools/check.sh --keep-going --require-full` 汇总所有检查，即使某套测试失败也继续后续检查，并在末尾返回非零；不隐藏缺少完整测试依赖的情况。
- `tools/migration_baseline.py` 保存源代码、配置、标定文件和语音资源的SHA-256以及环境信息。快照不代表验收通过。

## 本次验证结果

- 两个ROS包均构建成功。
- ROS包测试32项通过，无失败、错误或跳过。
- 完整离线检查已运行到底：仅 `test_complex_avoidance.py` 失败，其余检查命令通过；该文件有下述3条失败记录。
- 检查默认参数一致性通过；语音资源/配置/代码指纹已保存到候选快照。

## 待解决的回归问题

`tools/test_complex_avoidance.py` 共32个测试方法，出现3条失败记录，其中2条来自同一方法的不同子场景：

1. `test_compact_route_reduces_longitudinal_space_without_widening`：简单障碍场景的终点纵向距离约1.981m，旧比例要求小于1.647m。该比例来自早期参数/策略对比，需重新明确整段绕障的空间验收指标，不能直接降低断言以换取通过。
2. 同一方法的连续障碍子场景：路径进入了仿真要求的0.2m几何间隔。
3. `test_second_obstacle_replans_and_keeps_original_route`：当前默认参数下，连续障碍场景同样进入0.2m几何间隔。

后两项是离线射线/矩形模型中的间隔失败，不等于已经发生实机碰撞。该模型没有完整机身外形，但现有安全间隔断言仍须保留；需要排查整段运动期间的侧向感知覆盖、制动时序和路径净空。当前生产算法及阈值保持原样，相关用例未删除、跳过或标为预期失败。

消息编译与接口测试通过不能替代绕障验收。后续用户已调整优先级：连续绕障暂缓修复，先推进 [相机ROS输入迁移](ros2_camera.md)。本页保留阶段1当时的失败记录。

## 场景与测试入口

| 场景 | 当前回归入口 |
|---|---|
| 左右绕障、旋转原路线、长障碍、连续障碍、回原线 | `test_complex_avoidance.py`、`test_path_return.py` |
| 无路、停止失败、返回失败、语音等待中新障碍 | `test_complex_avoidance.py` |
| 深度断帧、旧帧、里程计过期 | `test_complex_avoidance.py`、`test_ros_contracts.py` |
| 停止线丢失、红灯等待、停止线回正 | `test_decisions.py`、`test_stop_line_align.py`、`selftest.py` |
| 盲道恢复、模式切换、串联任务 | `test_decisions.py`、`test_mission.py`、`test_mission_flow.py` |
| 退出与初始化失败清理 | `test_ros_runtime.py` |
| 跨会话迟到回执、字段无损转换 | `test_ros_conversions.py` |

这些是逻辑/模拟验收，不能替代后续故障注入和受控实机验收。当前阶段没有启动真实运动、相机或语音。

## 复现

```bash
source /opt/ros/humble/setup.bash
python3 tools/generate_ros_config.py --check
bash tools/check.sh --keep-going --require-full
# 上条命令当前预期返回1；后续命令可分开执行以完成全部检查。
colcon build --base-paths . ros2/rover_x1_interfaces --packages-up-to rover_x1
source install/setup.bash
colcon test --base-paths . ros2/rover_x1_interfaces --packages-select rover_x1_interfaces rover_x1
colcon test-result --verbose
```

候选快照位于 `docs/baselines/ros2_phase1.json`。检查当前源码/参数/资产是否仍与快照一致：

```bash
python3 tools/migration_baseline.py --check docs/baselines/ros2_phase1.json
```

算法或参数修复后应重新完成检查，并另存新快照，保留这份失败基线用于比较：

```bash
python3 tools/migration_baseline.py --output /tmp/rover_next_candidate.json
```

文档编辑、构建输出和日志不计入源文件指纹；Python代码、测试脚本、包元数据、ROS消息、默认参数与全部资产计入。由于原项目没有Git版本记录，快照中的输入总哈希用于识别这份候选基线。
