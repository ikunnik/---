# Rover X1 深度避障

DOBOT Rover X1 四足机器人的最小闭环：
**深度相机多区域测距 → 避障状态机 → ROS 2 速度控制（默认）/ gRPC 回退 → E7 语音提示**。

现在的行为：

- 前方安全时持续前进（`forward_s=5s` 一条，执行器空闲自动补发）。
- 危险时停车；障碍稳定后固定按 `±45°` 选一侧绕行（角度写死）。
- 斜向绕行目标 0.8m，回正后沿原方向绕出 0.9m；各阶段一次执行目标距离，按实时净空缩短。
- 回切方向清空后再沿原方向确认0.4m，以60°切回；受阻时继续绕出并重新观察；只有实测横向偏差 ≤0.12m、航向误差 ≤10° 且前方安全才恢复前进。
- 动态障碍（距离持续变化/不稳定）只停车等待，不绕行。
- 执行失败、超时、位置失效、无进展或预算耗尽 → 锁存 `BLOCKED`；普通路径受阻则先确认停车再重规划。
- 感知失效立即中断动作并停车。
- 通过机器人 SDK 的低层 E7 语音接口播报绕行方向、绕行完成和具体停车原因；
  默认使用预生成自然女声，播报期间机器人保持停车，播完再继续动作。
- 运行期间关闭机器人内置避障，退出时恢复原状态和速度档位；读不到原状态会直接拒绝启动。

## ROS 2 统一启动

项目已提供 `ament_python` 包 `rover_x1`。构建后可统一启动原有控制流程：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths . ros2/rover_x1_interfaces --packages-up-to rover_x1
source install/setup.bash
ros2 launch rover_x1 rover.launch.py check_config:=true  # 仅检查配置，不连接机器人
ros2 launch rover_x1 rover.launch.py mode:=avoidance    # 实际运行普通绕障
```

模式支持 `mission`（默认串联）、`avoidance`、`traffic`、`lane`。启动参数文件为 `config/rover.yaml`，可通过 `params_file:=/绝对路径/配置.yaml` 替换；参数修改后重启生效。`dry_run:=true` 只模拟运动，仍使用实际相机和语音。

构建、设备依赖、参数优先级与退出方式见 [ROS 2 包化说明](docs/ros2.md)。原 `python3 main.py` 入口继续可用。 后续节点拆分顺序见 [ROS 2 迁移计划](docs/ros2_migration_plan.md)。

## 代码结构（Phase 1–2 后）

```
main.py                  兼容入口（约 50 行）
app/cli.py               CLI + Config 生成
runtime/assembly.py      队列/worker/decision 装配
runtime/lifecycle.py     连接、启动、监视、逆序停止
runtime/readiness.py     相机第一帧握手
runtime/dds.py           CYCLONEDDS_URI 修正
runtime/app.py           启动骨架
motion/api.py            MotionBackend / MotionCall / MotionCancelled
motion/ros2_backend.py   ROS2 Twist + 里程计闭环
motion/posture_sdk.py    只做起立/切 WALK/姿态
motion/dry_run.py        假机器人
motion/factory.py        后端选择
```

## 架构

```
 [进程] depth_camera.py ── mp.Queue(maxsize=1) ──► [线程] perception.py
   DDS 订阅 + 中心/左/右剖面      只放最新一帧             搬运到 Latest
                                                               │
                                                         Latest（单值白板）
                                                               ▼
                                                      [线程] decision.py
                                              50Hz 调用 decisions/engine.py 统一判断
                                                         │        ▲
                                                         │        │ Pose（yaw）
                                                         │   robot_link.get_pose()
                                                         │
                                                   Mailbox（单槽信箱）
                                                         ▼
                                                      [线程] executor.py
                                               同步 gRPC 流 + cancel + CommandResult
                                                         │
                                                   robot_link.py ──► :50051

 [进程] voice.py ◄── mp.Queue(maxsize=1, VoiceCue) ── [线程] decision.py
   女声 WAV + 24kHz PCM ── DDS rt/voice/cmd ──► 机器人 E7 扬声器
        └── VoiceDone 回执 ──► decision.py（播完才继续运动）

 [进程] rgb_camera.py ── mp.Queue(maxsize=1) ──► [线程] rgb_perception.py
   DDS rt/camera/camera0/image_compressed          只留最新一帧
   （只解析，不解码）                                    └──► Latest[RgbFrame]
                                                              （以后：预览/检测）
```

- **深度/RGB 相机和语音是进程**：都依赖原生 DDS 库，隔离出去，崩了不影响主控。
- **RGB 只采集/转发**：`RgbCameraProcess` 只做 `CompressedImage -> RgbFrame`，
  `RgbPerceptionWorker` 把它搬到 `Latest`；不解码、不参与避障。
- **其余都是线程**：共享内存，传对象即可，不需要序列化。
- **通信工具**（`comm.py`）：`Latest` 白板、`Mailbox` 信箱、`put_latest()`。
- **动作结果**：执行器把每条指令的最终状态写成 `CommandResult`，决策层只有
  收到 `ok` 才认为动作完成；`failed / cancelled / 超时` 都进入 `BLOCKED`。
- **语音事件**：状态机只产生 `VoiceCue(kind, side, reason)`，`workers/voice.py`
  负责翻译成中文、读取女声 WAV（不回退机器声）和 DDS 发布；播报完成后
  通过 `VoiceDone` 回执，决策层才执行转向/恢复前进。

## 目录

| 文件 | 作用 |
|---|---|
| `main.py` | 入口：建对象 → 启动 → 等 Ctrl+C → 逆序停止 |
| `config.py` | 所有参数（距离、速度、ROI、绕障阈值、超时） |
| `messages.py` | `Perception` / `RgbFrame` / `ActionRequest` / `Velocity` / `VelocityPlan` / `Motion` / `CommandResult` / `Pose` / `VoiceCue` / `VoiceDone` |
| `actions.py` | 统一 `Actions`：速度、距离、转向、停车及盲道/停止线参数预设，不碰网络 |
| `comm.py` | `Latest` / `Mailbox` / `put_latest` |
| `robot_link.py` | gRPC 连接、起立、walk、速度/动作请求、`get_pose()`；含 `DryRunRobot` |
| `workers/depth_camera.py` | 进程：DDS 订阅 + `compute_profile()` |
| `workers/rgb_camera.py` | 进程：DDS 订阅 camera0 的 `CompressedImage` + `decode_compressed()` |
| `workers/rgb_perception.py` | 线程：RGB 队列 → `Latest[RgbFrame]`，日志限流 |
| `workers/traffic_light.py` | 进程：camera0 RGB → ROI/HSV → 时序投票 → `TrafficLightState`（默认序列消费） |
| `workers/stop_line.py` | 进程：camera2 RGB → 下方 ROI/黄线 → `StopLineState`（默认序列消费并控车） |
| `workers/lane_camera.py` | 独立 camera2 盲道检测进程，只输出 LaneState |
| `workers/lane.py` | 单线/双线图像检测、LaneState 队列转发 |
| `workers/voice.py` | 进程：`VoiceCue` → 女声 WAV → DDS `rt/voice/cmd`（无机器声回退） |
| `workers/perception.py` | 线程：队列 → Latest，日志限流 |
| `workers/decision.py` | 线程：收集快照、调用统一判断、匹配回执、提交动作和语音 |
| `decisions/engine.py` | 唯一行为入口 `DecisionEngine.evaluate()`：模式接管、切换和回执跟踪策略 |
| `decisions/safety.py` | 公共感知有效性、障碍和停止线检测器健康判断 |
| `decisions/avoidance.py` | 避障状态机：前进、等待、转向、绕行、回正、锁停 |
| `decisions/rules.py` | 几何、动态步长和左右候选方向排序 |
| `decisions/lane.py` | 盲道方案 4、无盲道直行、测试时限与失败停车 |
| `decisions/traffic.py` | 停止线接近、停车、信号等待、通过及失效锁停 |
| `decisions/voice.py` | 播报去重、等待回执/超时和延迟动作判断 |
| `decisions/types.py` | 状态名、候选方向、StepResult、PendingAction、StepContext |
| `workers/executor.py` | 统一 `MotionExecutor`：`submit()`、取消、循环看门狗、发布 `CommandResult` |
| `tools/test_decisions.py` | 统一入口回归：模式切换、打断、感知失效、动作失败 |
| `tools/selftest.py` | 不接机器人，自检剖面 / 状态机 / 失败动作 / 回正 / 语音 / 联调 |
| `tools/depth_probe.py` | 深度相机体检：零值占比和各区域距离 |
| `tools/rgb_probe.py` | 前置 RGB 体检：只订阅 camera0，打印格式/帧率并保存首帧 |
| `tools/traffic_light_calibrate.py` | 红绿灯 HSV/ROI 标定，生成 `traffic_light_profile.json` |
| `tools/voice_check.py` | 语音链路自检：只合成/读 WAV，或通过 DDS 播到机器人扬声器 |
| `tools/voice_generate.py` | 开发时用 Edge TTS 重新生成 `assets/voice/*.wav` |
| `tools/robot_check.py` | 机器人只读体检：状态 / 档位 / 姿态（不改状态） |
| `tools/rotate_once.py` | 手动执行一次旋转，不启动相机/决策 |
| `tools/uturn_probe.py` | 单独测试三段式掉头：`--simulate` 离线 / `--real` 真机 |
| `tools/sync_robot.sh` | 与真机同步：`--diff` 比较 / `--push` 推送 / `--pull` 拉回 |

## 运行

项目依赖装在 **distrobox 容器 `roverx1-dev`** 里（宿主机是 Python 3.14，跑不了）。

```bash
distrobox enter roverx1-dev
cd /home/admin/rover_x1_ws4

# 女声语音包已随项目放在 assets/voice/，运行时不需要额外安装或联网
# 语音只读 assets/voice/*.wav；缺文件会报错静音，不会回退机器声
# 默认运动后端是 ROS2：先 source /opt/ros/humble/setup.bash 并设置 DDS 网卡；
# 没有 ROS2 环境时加 --motion-backend sdk 回退。

python3 main.py --dry-run   # 默认含红绿灯/停止线序列，不连机器人
python3 main.py             # 真机默认：红绿灯 → 掉头 → 盲道 完整串联
python3 main.py --no-mission           # 关闭串联：只跑红绿灯/避障
python3 main.py --no-traffic-sequence  # 只跑普通避障（自动关闭串联）
python3 main.py --no-traffic-sequence --traffic-light --traffic-light-now --stop-line  # 只启动检测/监控
python3 main.py --traffic-light-now  # 调试：红绿灯立即处理
python3 main.py --vx 0.2    # 真机首次建议低速
python3 main.py --vx 0.2 --no-voice   # 真机，只避障不播报
python3 main.py --mangdao --no-voice  # 原分段模式（camera2，控方向）
python3 main.py --lane-follow --lane-vx 0.5 --no-voice  # 别名，等价 --mangdao
python3 main.py --mission              # 显式串联（默认已启用，可省略）
python3 main.py --dry-run --mangdao --no-voice  # 不接机器人看车道日志
```

### ROS 2 运动后端（默认）

实时运动走 ROS 2，起立/蹲下仍走 SDK：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 直连时建议显式钉网卡；未设置时后端会自动找 192.168.5.x 地址
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.5.100"/></Interfaces><AllowMulticast>spdp</AllowMulticast></General><Discovery><EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints></Discovery></Domain></CycloneDDS>'

python3 tools/ros2_check.py --seconds 5   # 只读：确认订阅者/里程计/雷达
python3 main.py
```

- 速度话题：`/planner/vel_cmd`（`geometry_msgs/Twist`，`BEST_EFFORT`，20Hz）；
- 位姿/看门狗：`/foot_odom/data`；
- 起立/切 WALK：SDK `192.168.5.2:50051`；
- 距离移动、90° 转向、`VelocityPlan`（盲道循环）都由里程计闭环；
- 取消/退出先发零速 ×3，再恢复内置避障和速度档位。

> 默认 `motion_backend=ros2`；需要退回 gRPC 时加 `--motion-backend sdk`。
> 两种后端共用同一套 `Actions → MotionExecutor`，切换只影响底层运动。
>
> 若 `/planner/vel_cmd` 已有订阅者但狗不动：先运行机器狗解锁/使能节点
> （`/enable/motors` 等），并确认 `/set/auto/intelligence` 已关闭。

DDS 收不到数据时，先配置网卡：

```bash
source /home/admin/dobot/dobot_quad_sdk-main/setup_cyclonedds_env.sh
```

前置 RGB 体检（只订阅，不移动机器人）：

```bash
python3 tools/rgb_probe.py                 # camera0，采样 5s，保存首帧到 rgb_images/
python3 tools/rgb_probe.py --seconds 10
python3 tools/rgb_probe.py --save rgb_images/camera0.jpg
```

离线调阈值（不需要机器人）：

```bash
python3 tools/test_decisions.py  # 统一判断入口的离线回归
python3 tools/selftest.py        # 剖面 + 状态机 + 执行器 + 语音 + 联调自检
python3 tools/depth_probe.py     # 相机体检：零值占比 / 左中右距离
python3 tools/voice_check.py --dry-run   # 读取默认 lane.wav，检查 PCM
python3 tools/voice_check.py --dry-run --wav assets/voice/bypass_left.wav  # 检查女声包
python3 tools/voice_check.py --play --wav assets/voice/bypass_left.wav     # 播女声（不移动）
python3 tools/robot_check.py     # 机器人体检：只读，不动作
python3 tools/rotate_once.py --dry-run --left 90   # 假机器人转 90°
python3 tools/rotate_once.py --left 90             # 真机左转 90°
python3 tools/rotate_once.py --circle right --turns 2
python3 tools/uturn_probe.py --simulate                  # 离线仿真掉头
python3 tools/uturn_probe.py --simulate --yaw-error 15   # 模拟打滑，验证补转
python3 tools/uturn_probe.py --real --forward 0.4        # 真机掉头（需确认）
```

> 机器人趴着（`READY`）时，前置深度相机离地只有约 7 cm，深度图 98% 为 0
> （低于最小量程），`valid=False` 是正常现象。**必须先 `balance_stand` 站起来**
> 再看深度数据，否则会误以为相机坏了。

## 自动避障流程

```text
GO → HOLD（停车、等待不同相机帧稳定、记录原路线）
       ↓
PLAN → TURN（优先保持绕行侧，±45°）→ CHECK（新深度帧）
       ↓
ADVANCE（整段斜走，目标0.8m）→ REALIGN（回原航向）
       ↓
ADVANCE（沿原方向整段绕出，初始0.9m）
       ↓
RETURN（检查回切方向、转60°、按实测偏差计算距离后整段回线）
       ├─ 回切受阻 → 回正后继续绕出；前方也受阻则重新规划
       ├─ 运动中新障碍 → 取消 + 停车成功回执 → 等待稳定 → 重新规划
       └─ 横向偏差≤0.12m → 回正 → 前方≥1.0m → 完成提示 → GO
```

原路线由本次障碍事件开始时的 `Pose.x/y/yaw` 定义，后续遇到第二个障碍或回切失败时不重置。
回切侧首次清空后再实测前进0.4m确认拐角净空，随后以60°回线；回切侧重新受阻会重置该确认。
首次绕行选较空一侧，后续优先保持同侧；某方向转过去仍无净空时先回正，记住失败方向，尝试另一侧。
各阶段不再按固定0.4m拆分；动作距离取阶段剩余目标、前方可用净空和底层单次上限3m的最小值。
前方保留 `max(danger_m, emergency_m)+safety_buffer_m` 的余量；净空不足时缩短动作或重新规划。
动作结束后等待 `settle_s` 和新源帧，再判断下一阶段；执行期间持续检查危险距离、感知与位姿。

- 默认回线要求新鲜且有限的平面位置，不能读取位置、回线尝试耗尽或没有实际位移都不会报告完成。
- 绕行与回切途中均检查 `danger_m=0.7m`；进入该范围先打断并等待停车回执。
  低于 `emergency_m`、动作失败或感知失效则锁存停车。
- 转身前及期间持续检查左/中/右区域净空；语音结束后重新规划，不直接执行播报前保存的转向。
- 回切距离按实际剩余偏差和当前航向计算，超时使用统一 `Actions` 给出的距离超时。
- 总预算包含回切与中断位移：6m、24次距离动作、8次重规划、120s；回切最多6次，回正最多12次。
- 仅因净空不足停车可在前方持续安全2s后恢复检查；执行故障、无进展和预算耗尽需重启或显式重置。

算法与离线验证见 [整段绕障与回原路线](docs/avoidance.md)。

## 语音提示

语音走机器人 SDK 的低层 E7 接口，不经过主控电脑扬声器：

```text
VoiceCue -> assets/voice/*.wav（自然女声，24kHz/mono/16bit）
         -> VoiceCmd(type="streaming") -> DDS rt/voice/cmd -> 机器人 E7 扬声器
```

- 默认音色：`zh-CN-XiaoxiaoNeural` 预生成自然女声；**没有 espeak 回退**，
  语音包缺失时只报错并静音，不会再出现机器声。
- **播报期间停车**：绕行提示、完成提示、动态障碍、等待/失败提示都会先把
  机器人停住；`workers/voice.py` 播完（或失败/超时）后回传 `VoiceDone`，
  状态机才继续转向或恢复 GO。
- 规划绕行时播报：“前方有障碍，请从左侧绕过。” / “……右侧……”
  （60° 时播报“向左前方/右前方绕过”）。
- 回正/绕回原线完成且累计绕出 ≥ `min_bypass_advance_m` 后播报：“已绕过障碍，继续前进。”
- 动态障碍播报：“检测到移动障碍，停车等待。”
- 停车/锁定按原因播报具体短句，例如：
  - 太近：“距离障碍太近，旋转空间不足，已停车。”
  - 无空间：“左右两侧空间不足，无法绕行，已停车。”
  - 无姿态：“无法获取机身方向，已停车。”
  - 转向失败/超时/角度不符：“转向失败/转向超时/转向角度不符合要求，已停车。”
  - 深度丢失：“深度数据丢失，已停车。”
  - 前进中过近：“前方距离过近，已紧急停车。”
- 停车/动态障碍用 `kWarning` 优先级，绕行方向用 `kPROMPT`，完成/等待用 `kNormal`。
- 语音进程启动后需要约 1s 做 DDS 发现；第一句可能略晚。
- 临时关闭播报：`python3 main.py --no-voice`；只验证语音：
  `python3 tools/voice_check.py --play --wav assets/voice/bypass_left.wav`
  （不会让机器人移动）。
- 重新生成女声包：见 `assets/voice/README.md`；生成阶段才需要
  `edge-tts` 和 `miniaudio`，运行阶段完全离线。

## 前置 RGB（camera0）

`camera0` 是机身**前置纯 RGB**，话题 `rt/camera/camera0/image_compressed`，
消息类型 `CompressedImage`（JPEG）。注意编号：

| 话题 | 位置 |
|---|---|
| `rt/camera/camera0/image_compressed` | 前置纯 RGB |
| `rt/camera/camera1/image_compressed` | 后置纯 RGB |
| `rt/camera/camera2/image_compressed` | 前置深度相机的 RGB（与深度同轴） |
| `rt/camera/camera3/image_compressed` | 后置深度相机的 RGB |

数据链（只采集/通信，不解码）：

```text
[进程] rgb_camera.py --mp.Queue(maxsize=1)--> [线程] rgb_perception.py --Latest--> RgbFrame
   DDS CompressedImage -> bytes(msg.data())        只搬运，不判断
```

- 默认关闭，`--rgb` 开启；`--rgb-topic` 可以切到 `camera2/image_compressed`。
- 进程只把 `data()` 转成 `bytes`（DDS 返回 `list[int]`），不解码、不重编码。
- 日志：
  - `[RGB进程] 已订阅 rt/camera/camera0/image_compressed`
  - `[RGB进程] 20.0 FPS，45.0 KB/帧`
  - `[RGB感知] 第一帧: jpeg 45.0KB seq=1`
  - 5 秒没有帧：`[RGB进程] 还没有收到帧（检查 DDS 网卡/话题 ...）`
- 体检：`python3 tools/rgb_probe.py`（只订阅，不移动机器人；首帧存 `rgb_images/`）。
- 采集链路只做 `CompressedImage -> RgbFrame`；预览/落盘/检测都从 `Latest[RgbFrame]`
  或 fan-out 队列接，不在采集进程里解码。

## 红绿灯检测（camera0，等待停止线后处理）

- 标定 profile：`assets/traffic_light_profile.json`（camera0 1920×1080 实拍；
  ROI `(0.37, 0.40, 0.32, 0.26)`，覆盖红框 x 0.382~0.665 / y 0.435~0.631；
  红 H 341~358、绿 H 131~160）。
- camera0 **采集常开**；是否处理由跨进程布尔 `traffic_active` 控制：
  - `False`：只采集、只更新 `Latest[RgbFrame]`，不 fan-out、不解码、不识别；
  - `True`：开始 ROI/HSV/投票，输出 `TrafficLightState`。
- 启动方式（默认 `main.py` 已启用停止线序列）：
  - `python3 main.py`：默认序列，到线停稳后自动 `traffic_active.set()`；
  - `python3 main.py --no-traffic-sequence --traffic-light --traffic-light-now`：
    只做红绿灯检测/调试，不启动停止线序列；
  - `python3 main.py --traffic-light-now`：调试用，启动即处理；默认序列仍启用。
- 数据链：

  ```text
  camera0 RGB（常开）→ RGB 感知线程 ──traffic_active=True──► fan-out
        → [进程] traffic_light.py → TrafficLightState
        → [线程] traffic_light_worker → Latest（以后红灯门控）
  ```

- 日志：`[红绿灯] traffic_active=True，开始处理`；之后
  `[红绿灯] red stable=True area=1.2% bbox=(...)`；每 5s 打一次检测 FPS。
- **红绿同时出现时判绿**：红灯反光/背景红色块与绿灯并存时优先绿灯；
  只有红/黄没有绿时仍按面积最大颜色处理。
- 绿灯阈值已放宽到 H 115~170、S≥0.25、V≥0.18，`min_area_ratio=0.0003`，
  `confirm_frames=4`、`confirm_s=0.3`：实拍约 5m 外的绿灯光斑在 960 宽下
  只有约 60~90px（旧阈值 0.001 会丢弃），现在可稳定识别。
- 连续 `confirm_frames` 帧、且持续 `confirm_s` 才 `stable=True`；丢帧/未知时
  保持上一稳定颜色到 `stale_s`，避免一帧闪烁就丢掉红灯。
- **默认停止线序列会消费红绿灯状态并控车**；只检测/调试用
  `--no-traffic-sequence --traffic-light --traffic-light-now`。红/黄/绿语音已接。
- 依赖 `cv2` 解码 JPEG；容器里通常已有（legacy 用过），没有就
  `pip install opencv-python-headless`。检测进程独立，`cv2`/检测崩了不影响避障。
- 标定工具：`python3 tools/traffic_light_calibrate.py --dir rgb_images/calib --roi ...`；
  换相机（camera0 ↔ camera2）必须重新标定，ROI/颜色不通用。

## 黄线停止线检测（camera2；默认序列会控车）

camera0 看不到地面黄线，camera2（深度相机的 RGB）能看到；两个相机分工。

- profile：`assets/stop_line_profile.json`；camera2 640×480；ROI
  `(0.0, 0.60, 1.0, 0.35)`；黄线偏橙黄 **H 18~48、S≥0.35、V≥0.45**；
  `stop_row = 0.844`（`front_camera204` 停车位的黄线底边）。
  **注意：`rgb_to_hsv()` 返回 H=0~360，不是 OpenCV 的 0~180。**
  实拍 201~207 的 H 中位数为 31~37、p99≤43，`18~48` 已经覆盖；
  不要再按 OpenCV 单位乘 2 改成 `50~84`，那样会完全看不到这条线。
- 启动：默认 `python3 main.py` 会同时启动停止线检测和序列；
  `--stop-line` 只是显式启用检测（默认序列已包含），不影响 camera0/避障链路。
- 与红绿灯一起：默认 `python3 main.py` 就会同时启动；只监控用
  `--no-traffic-sequence --traffic-light --traffic-light-now --stop-line`。
- 日志：`[停止线] visible=True row=...`；现在还会**每秒**打一条
  `[停止线] 状态 visible=... row=... area=...`，行号不变时也能看出机器狗是否在接近。
- 连续 `confirm_frames=3` 帧可见才 `visible=True`；短暂丢帧保持到 `stale_s=0.3`。
- **`--no-traffic-sequence --stop-line` 只检测/日志，不控制运动**；默认序列才会
  在到线时停车并 set `traffic_active`。
- 实测：`front_camera204`（停车位）row≈0.846；`front_camera201`（远处）row≈0.717。
  触发停车建议取 `row >= 0.83`（留 0.54s 刹车余量），跑几次量实际位置再微调。

## 停止线序列（默认启用，会控制运动）

```bash
python3 main.py                 # 默认：红绿灯 → 掉头 → 盲道（完整串联）
python3 main.py --no-mission             # 关闭串联，只跑红绿灯/避障
python3 main.py --no-traffic-sequence  # 关闭序列+串联，只跑普通避障
python3 main.py --traffic-sequence     # 强制启用（兼容旧命令）
# 速度/触发行号可现场调：
python3 main.py --traffic-vx 0.4 --stop-line-trigger-row 0.83
```

默认自动启用 camera0 红绿灯 + camera2 黄线，流程：

```text
看到黄线 & 避障空闲（GO/HOLD）
    → LINE_APPROACH：0.40 m/s 低速接近（1.0s 一条前进）
    → row >= stop_line_trigger_row（默认 0.83）：打断 + 停车
    → AT_LINE：停稳 0.25s
         黄线倾角 |angle| >= 2° → 原地微调 yaw（最多 12°，最多 2 次）
         对齐完成 → traffic_active.set()
    → SIGNAL：camera0 识别
         红灯/黄灯稳定 → 停车等待（播报一次）
         绿灯稳定 → 播报绿灯 → PASS（保持检测）
    → PASS：line_walk 前进 traffic_pass_m（默认 2m）
           起步 traffic_pass_guard_s 后，前方 distance <= traffic_pass_stop_m（0.7m）
           提前打断并停车掉头；2m 走完也直接掉头
         途中红灯/黄灯重新稳定 → 立刻停车，退回 SIGNAL 等绿灯
    → DONE：停车 + clear + sm.reset()，交回普通避障
异常：LINE_APPROACH 中丢失停止线 / 行号无进展超过 traffic_stall_s
    → WAIT_STOP_LINE：锁存停车（播报原因，不交回避障，需人工处理）
```

- **真机 0.15/0.30 m/s 都低于步态有效速度**：日志里会出现 `traffic_forward`
  一直“完成”、但黄线 `row` 不变；默认已改成 **0.40 m/s**，`main.py` 会拒绝更低速度。
- **无进展/丢失锁存**：黄线 `row` 连续 `traffic_stall_s=4s` 没有增加
  `0.02`，或停止线持续丢失 `traffic_stall_s=4s`，序列会进入
  `WAIT_STOP_LINE` **锁存停车**，播报 `wait_stop_line_stall_wait.wav` /
  `wait_stop_line_lost_wait.wav`。锁存后不再交回避障（交回会立刻恢复前进，
  可能冲过停止线），只能人工处理并重启程序。
- **数据新鲜度保护**：停止线超过 `stop_line_stale_s=0.5s` 没更新、深度帧
  超过 `stale_s=0.5s` 没更新或 `valid=False`，序列都会立刻停车；深度持续
  失效超过 `traffic_stall_s` 仍会退出序列交回普通避障（这是深度链路故障，
  与停止线锁存不同）。锁存状态下则保持停车，不再交回。
- **动作结果保护**：序列发出的 `traffic_forward` / `stop` 都会等执行器最终
  结果；前进失败/取消/超时立即停车并退出，停车失败则继续补发停车，
  不会因为“结果丢了”就继续前进。
- **语音互斥**：序列启动前检查 `sm.voice_pending`，播报未结束时不会接管；
  序列运行中如果语音开始播报，会立即打断前进并停车，收到 `VoiceDone`
  或等待超时才继续。
- **红绿灯语音**：红灯稳定时播报 `traffic_red.wav`（“红灯，请停车等待。”），
  黄灯播报 `traffic_yellow.wav`，绿灯播报 `traffic_green.wav` 后才开始通过；
  同一颜色只播一次，播报期间保持停车。
- **黄灯/未知颜色**：一律停车等待，只有稳定绿灯才通过。
- **通过中红灯复检**：`PASS` 期间保持 `traffic_active`，红灯/黄灯重新稳定时
  立刻打断前进、退回 `SIGNAL`，不会盲走完 2m。
- **信号丢失保护**：红绿灯数据持续缺失 `traffic_signal_timeout_s=10s` 时只
  告警并继续停车 **不交回避障**，避免在红灯前恢复前进；需要人工处理。
- 序列期间 **避障状态机暂停**，不绕行其他障碍；除上述深度/距离保护外，
  第一次真机测试仍请用空旷场地、手放急停。
- 序列本身不产生“开始接近/开始绕行”这类提示，只在红灯/黄灯/绿灯时播报；
  任何其他语音播报期间序列都会保持停车。
- 一次运行只执行一次序列（`done` 锁存）；重新跑要重启程序。
- `--traffic-light-now` 不建议和默认序列一起用（会让识别提前开始）。

## 独立盲道模式（`--mangdao`）

只保留稳定循环纠偏（原方案4），旧的方案选择、分段和预测模式已经删除。

```bash
python3 main.py --mangdao
python3 main.py --mangdao --lane-vx 0.40 --lane-trial-seconds 10 --no-voice
```

- `main.py` 默认：深度避障 + 停止线/红绿灯流程。
- `main.py --mangdao`：深度避障 + 独立盲道相机进程，仅加载 `assets/lane_profile.json`；不创建停止线、停止线感知或红绿灯进程。
- 显式同时传入 `--mangdao` 与 `--stop-line`、`--traffic-sequence`、`--traffic-light` 或 `--traffic-light-now` 会在连接机器人前报错。
- 盲道进程独立订阅 camera2，通过自己的订阅/第一帧握手启动；不等待停止线数据。进程退出时主程序停止运动并退出。
- 速度默认0.40m/s，每条RPC循环两个1.2秒步骤。转向目标变化达到0.12rad/s并持续0.25秒，且当前循环已运行至少2.5秒时，取消旧序列、下发新目标。重新下发时仍可能顿挫。
- 相机正常但没有盲道时零转向直行；识别到可信盲道后恢复纠偏。相机无数据/过期、深度失效仍停车；障碍由避障状态机处理。用 `--lane-stop-on-loss` 可恢复无盲道停车。
- 默认限时6秒，可用 `--lane-trial-seconds 10` 测10秒。切换有线/无线不重置计时，循环运动仍受执行器1秒心跳看门狗保护。
- `--mangdao` 本身即选择循环纠偏，不再需要方案选择或额外循环确认参数；本次代码调整不会自动启动真机。

机器人已验证的DDS配置绑定192.168.5.20：

```bash
export CYCLONEDDS_URI=file://$HOME/dobot_quad_sdk-main/cyclonedds.xml
python3 main.py --mangdao --no-voice
```

离线检查：`python3 tools/test_mangdao.py`、`python3 tools/test_lane_no_line.py`、`python3 tools/selftest.py`。

## 已知限制

- **盲道为主 + 避障接管**：`--mangdao` 正常沿黄线走；前方 < `danger_m`
  会切 `AvoidStateMachine` 绕行，绕完回到盲道。新鲜图像中没有盲道时默认直行，数据过期或低置信度时停车。
  第一次真机先 `--dry-run` 看 `[车道]` 日志，再空旷场地低速试。

- **依赖位置反馈**：ROS2 使用 `/foot_odom/data`，SDK 使用 `pos_body`。
  绕障累计各段实测位移，动作成功仍需检查前进进展；位置失效或明显跳变会停车。
  回线精度依赖里程计，无法纠正持续累积的定位漂移。
- **没有后置/侧置深度相机**：只能靠前置相机的左/中/右三个区域判断转身净空，
  窄通道或侧后方突然来人时仍需要人工急停。
- **航向来自 ROS2 里程计或 SDK `GetRobotState`**：走路时 yaw 有噪声，`yaw_tol_deg=10°` 是折中值；
  真机如果频繁误判 `BLOCKED`，先看姿态曲线再调大容差或 `pose_cache_s`。
- **语音播报会主动停车**：女声短句约 2.5~4.5 秒，停车等待时间会相应增加；
  语音队列只保留最新一句。语音进程异常时最多等待 `voice_wait_timeout_s`
  （默认 10 秒）后继续，避免永久停死。
- **语音进程启动有 DDS 发现延迟**：约 1 秒；启动后第一句提示可能比停车动作晚。
- **RGB/红绿灯目前只采集/检测**：`--rgb` 只把 camera0 的 JPEG 放进
  `Latest[RgbFrame]`；红绿灯默认等 `traffic_active=True` 才处理。默认的停止线
  序列会在到线时 set；只调试红绿灯可用 `--traffic-light-now`。
- **停止线序列已默认启用，且仍不是量产级**：`python3 main.py` 就会控制运动
  （接近/停/红绿灯/通过）；只跑普通避障用 `--no-traffic-sequence`。序列会检查
  深度新鲜度、停止线时间戳、动作结果、语音互斥、黄灯/未知颜色、通过中红灯复检；
  但停止线投票长期统计、红灯持续时间建模、路口内障碍绕行等仍是后续项。
  真机首次务必低速、空旷场地、手放急停。
- **默认序列会等停止线检测器就绪**：若停止线进程 8s 内没完成 DDS 订阅，主进程会
  拒绝启动运动（机器人不会收到前进指令）；此时先用 `--dry-run --stop-line`
  或 `--no-traffic-sequence` 排查，不要绕过启动握手。
- **停止线锁存后不会自动恢复**：进入 `WAIT_STOP_LINE` 后会一直保持停车，
  不交回避障，也不重新尝试接近；确认现场安全后需要人工重启程序。
- **只检测的组合**：`--no-traffic-sequence --traffic-light --traffic-light-now --stop-line`
  只启动检测，不会在 `AT_LINE` 设置 `traffic_active`；默认序列才会自动触发红绿灯处理。
- **绿灯播报不阻塞通过**：短绿灯下等 2~3 秒语音会直接错过绿灯，因此绿灯播报与
  `PASS` 前进并行；红灯/黄灯仍然“播报期间停车”。
- **绿灯通过 2m，可提前掉头**：`line_walk` 走 `traffic_pass_m`（默认 2m）；
  起步 `traffic_pass_guard_s`（默认 2s，避开地面/停止线）后，若前方
  `distance <= traffic_pass_stop_m`（默认 0.7m）则提前打断、停车掉头；
  2m 正常走完也直接交给掉头，不再额外“寻找障碍”。
- **真机移动尚未完整验证**：语音发布链路已用 `tools/voice_check.py --play` 单独验证
  （未触发运动）；完整避障第一次运行请用 `--vx 0.2`，空旷场地，手放在急停上，
  并准备随时 Ctrl+C。

## 统一行为判断

判断代码统一维护在 `decisions/`。主线程只调用一个入口：

```text
workers/decision.py 读取感知、姿态、动作结果
    → DecisionInput（同一轮快照）
    → DecisionEngine.evaluate()
        ├─ safety.py：公共有效性和障碍判断
        ├─ avoidance.py + rules.py：普通行走、避障、回正
        ├─ lane.py：盲道纠偏、无盲道直行
        ├─ traffic.py：停止线和红绿灯流程
        └─ voice.py：播报等待和超时
    → DecisionOutput（动作、是否打断、回执跟踪、播报意图）
    → workers/decision.py 提交给 MotionExecutor
```

模式选择和控制权切换只在 `engine.py` 中维护：

- 普通模式使用避障状态机。
- `--mangdao` 使用盲道判断；遇障碍先打断旧动作并切入避障，完成后恢复盲道。
- 停止线模式先检查检测器是否健康；未接管时使用普通避障，接管后使用交通序列。
- 切换控制器时清除旧动作的回执跟踪，所有打断统一通过输出意图交给执行器。
- 各模式原有的失效停车、动作结果、语音等待、测试时限和锁停规则保持在所属类别内。
- `--mission`（默认启用，`--no-mission` 关闭）：绿灯后按 `traffic_pass_m`（默认 2m）前进，途中前方
  `traffic_pass_stop_m`（默认 0.7m）有障碍则提前停车 → **关闭停止线检测进程/线程**
  → 掉头（左转90° → 直行 `uturn_forward_m` → 左转90°）
  → 再由 `ControllerSupervisor` 切到盲道；`lane_trial_seconds` 默认 0（不限时），
  盲道会一直运行到 Ctrl+C 或异常；设为 >0 才限时停车并锁定。掉头航向偏差 ≤ `uturn_retry_tol_deg` 时允许补转一次，
  超过则锁停。**掉头直行默认不检查前方 1.9m 空间**，只看转向净空；
  需要恢复检查时加 `--uturn-check-forward-space`。

`workers/lane.py` 只负责图像识别和状态转发。
修改行为请编辑 `decisions/`，修改运动参数请编辑 `config.py`。

离线验证：

```bash
python3 tools/test_decisions.py
python3 tools/selftest.py
```

## 统一动作控制

所有自动模式采用同一条调用链：

```text
DecisionEngine → 避障 / 盲道 / 停止线判断
    → actions.Actions（统一构造 ActionRequest）
    → MotionExecutor.submit(request, interrupt=...)
    → RobotLink（适配 SDK）
    → 机器狗
```

| 类别 | 统一接口 | 单位 / 含义 |
|---|---|---|
| 速度行走 | `velocity(vx, vy, vyaw, duration=...)` | 前后、横移速度 m/s；转向 rad/s；时长 s |
| 普通前进 | `forward()` | 使用 `config.py` 的前进速度和时长 |
| 距离移动 | `move(direction, distance)` | 前/后/左/右，距离 m |
| 绕障前进 | `advance(distance)` | 复用距离移动 |
| 角度转向 | `turn(delta)` | 正数左转，负数右转，单位度 |
| 航向回正 | `realign(delta)` | 复用转向，保留回正超时和航向校验 |
| 转圈 | `circle(direction, turns)` | 左/右转指定圈数 |
| 停车 | `stop()` | 零速；立即停车用 `submit(..., interrupt=True)` |
| 盲道纠偏 | `lane(vx, vyaw, sequence)` | 复用速度控制，使用方案 4 的循环参数 |
| 停止线前进 | `traffic_forward(purpose)` | 复用速度控制，使用交通模式参数 |

状态机负责安全判断、动作超时与完成后的航向校验；执行器负责提交、取消、
失效指令过滤和结果回传。循环运动都受决策心跳看门狗保护，不依赖动作标签。
速度、距离、角度仍分别对应 SDK 的 `walk_velocity_seq`、`line_walk`、`rotation`，
不把距离和角度换算为估计运动时间。

起立和切换行走模式属于启动动作，集中在 `RobotLink.stand()` / `walk()`；
退出时 `RobotLink.brake()` 复用同一零速定义，通过同步 SDK 调用完成收尾。

以下为已有执行器上的独立调用示例，每行是一个单独动作，需等待结果后再发下一条：

```python
from actions import Actions
from config import CFG

actions = Actions(CFG)
executor.submit(actions.forward())
executor.submit(actions.turn(30))                   # 左转 30°
executor.submit(actions.turn(-30))                  # 右转 30°
executor.submit(actions.move("left", 0.2))          # 左横移 0.2 m
executor.submit(actions.stop(), interrupt=True)     # 取消当前动作并停车
```

手动旋转工具 `tools/rotate_once.py` 也使用 `Actions + submit()`，不会启动相机或自动决策。
执行器统一使用 `MotionExecutor`；行为模块直接导入 `decisions/` 中的实现。

## 绕障角度与斜向目标

```python
# decisions/avoidance/constants.py
BYPASS_ANGLE_DEG = 45.0
BYPASS_DIAGONAL_M = 0.8
```

平行绕出目标由 `config.py` 的 `min_bypass_advance_m=0.9` 控制。
上述距离为阶段目标；净空足够时一次执行完整目标距离，`bypass_step_max_m` 已不再参与绕障控制。

左右空间判断仍使用 `min_passage_m`、`wall_near_ratio`、`wall_extra_m`。

## 参数（`config.py`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `danger_m` | 0.7 | 低于它立即停车并进入避障 |
| `clear_m` | 1.0 | 允许前进 / 回正的最小中心距离 |
| `stale_s` | 0.5 | 多久没感知算失效 |
| `forward_vx` | 0.5 | 前进速度 m/s |
| `forward_s` | 5.0 | 每条前进指令 5 秒，执行器空闲就自动补发 |
| `motion_backend` | `ros2` | 运动后端：`ros2`=`/planner/vel_cmd`，`sdk`=gRPC 回退 |
| `ros2_vel_topic` | `/planner/vel_cmd` | ROS 2 速度话题 |
| `ros2_odom_topic` | `/foot_odom/data` | ROS 2 里程计（距离/角度闭环） |
| `ros2_rate_hz` | 20.0 | Twist 发布频率 |
| `ros2_line_vx` | 0.40 | ROS 2 距离移动速度 m/s |
| `ros2_yaw_rate` | 1.0 | ROS 2 旋转角速度 rad/s |
| `ros2_min_vx` | 0.30 | 速度死区（低于它抬到 0.3） |
| `ros2_odom_stale_s` | 0.5 | 里程计过期时间 |
| `ros2_cancel_zero_delay_s` | 0.15 | 取消后无新目标才补零速 |
| `ros2_startup_timeout_s` | 8.0 | 等 `/planner/vel_cmd` 订阅者 |
| `stop_s` | 0.3 | 零速指令持续时间 |
| `stop_refresh_s` | 1.0 | STOP/BLOCKED 状态下每秒补发一次零速 |
| `max_forward_vx` | 0.6 | `--vx` 允许的最大值 |
| `rgb_topic` | `rt/camera/camera0/image_compressed` | 前置纯 RGB（CompressedImage/JPEG） |
| `rgb_enabled` | False | 是否启用 RGB 采集（`--rgb`） |
| `rgb_log_s` | 5.0 | RGB FPS / 无帧 / 转发日志间隔 |
| `traffic_light_enabled` | False | 红绿灯检测（默认序列已包含；`--traffic-light` 显式启用） |
| `traffic_light_profile` | `assets/traffic_light_profile.json` | 红绿灯标定 profile |
| `stop_line_enabled` | False | 停止线检测（默认序列已包含；`--stop-line` 显式启用） |
| `stop_line_topic` | `rt/camera/camera2/image_compressed` | camera2 RGB 话题 |
| `stop_line_profile` | `assets/stop_line_profile.json` | 停止线标定 profile |
| `stop_line_stale_s` | 0.5 | 停止线状态超过该时间未更新就算过期 |
| `traffic_sequence_enabled` | True | 停止线序列默认启用；`--no-traffic-sequence` 关闭 |
| `traffic_vx` | 0.40 | 交通灯路段速度 m/s（0.15/0.30 可能不走） |
| `traffic_forward_s` | 1.0 | 序列前进指令时长（太短会被加减速吃掉） |
| `traffic_stall_s` | 4.0 | 黄线无进展/丢失多久后锁存停车；深度持续失效多久后交回避障 |
| `traffic_stall_row` | 0.02 | 认为“有进展”的最小行号增量 |
| `stop_line_trigger_row` | 0.83 | 黄线底边到该行号触发停车 |
| `stop_settle_s` | 0.25 | 停稳后再开红绿灯识别 |
| `stop_line_align_enabled` | True | 到停止线后按黄线倾角微调 yaw |
| `stop_line_align_threshold_deg` | 2.0 | 倾角超过它才对齐 |
| `stop_line_align_max_deg` | 12.0 | 单次对齐最大转角 |
| `stop_line_align_gain` | 1.5 | 倾角转角的放大倍数 |
| `stop_line_align_max_attempts` | 2 | 最多对齐次数 |
| `stop_line_align_sign` | 1.0 | 方向反了就改 -1 |
| `traffic_pass_m` | 2.0 | 绿灯后先走的 line_walk 距离（0=直接找障碍） |
| `traffic_pass_timeout_s` | 10.0 | 2m 动作等待结果超时 |
| `traffic_pass_stop_m` | 0.7 | 2m 途中前方小于它就提前停车掉头 |
| `traffic_pass_guard_s` | 2.0 | 起步后忽略多久（避开地面/停止线） |
| `traffic_state_stale_s` | 0.5 | 红绿灯状态过期时间 |
| `lane_follow_enabled` | False | 由 `--mangdao` 显式开启 |
| `lane_profile` | `assets/lane_profile.json` | 盲道独立标定 |
| `lane_topic` | `rt/camera/camera2/image_compressed` | 独立盲道订阅话题 |
| `lane_forward_vx` | 0.40 | 盲道及无盲道直行速度 m/s |
| `lane_control_duration_s` | 1.20 | 循环序列内每步时长 |
| `lane_batch_steps` | 2 | 循环序列内步数 |
| `lane_loop_update_threshold` | 0.02 | 转向目标更新阈值 rad/s（0.12 等于 33cm/10° 死区） |
| `lane_loop_min_update_s` | 0.22 | 重新规划最小间隔 s（纠偏环 0.4rad/s，间隔过长等于开环走斜） |
| `lane_loop_heartbeat_timeout_s` | 1.0 | 决策心跳超时停车 |
| `lane_trial_seconds` | 0.0 | 盲道运行时限；0=不限时持续运行 |
| `lane_no_line_straight` | True | 新鲜图像无盲道时直行 |
| `lane_steer_sign` | -1.0 | 转向符号 |
| `lane_stale_s` | 0.8 | 盲道状态过期时间（8FPS 下留余量） |
| `lane_reacquire_enabled` | True | 丢线后按最后线侧搜索 |
| `lane_reacquire_yaw_rate` | 0.12 | 搜索转向角速度 rad/s |
| `lane_reacquire_seconds` | 3.0 | 最长搜索时间 |
| `mission_enabled` | True | 默认串联：红绿灯 → 掉头 → 盲道；`--no-mission` 关闭 |
| `uturn_forward_m` | 1.00 | 掉头两次左转之间的直行距离 m |
| `uturn_ignore_obstacles` | False | 掉头时跳过所有深度净空检查（仅空旷场地） |
| `uturn_check_forward_space` | False | 掉头直行前是否要求 1.9m 净空（默认不检查） |
| `uturn_pose_stale_s` | 0.5 | 掉头姿态过期时间 |
| `uturn_sensor_wait_s` | 2.0 | 掉头段间等待新感知的上限 |
| `uturn_retry_tol_deg` | 20.0 | 掉头航向偏差在此范围内允许补转 |
| `uturn_max_retries` | 1 | 每段掉头最多补转次数 |
| `supervisor_period_s` | 0.05 | 调度器 tick 周期 |
| `supervisor_timeout_s` | 1.0 | 调度心跳超时（超时锁停） |
| `controller_switch_timeout_s` | 4.0 | 控制器切换停车确认超时 |
| `traffic_signal_timeout_s` | 10.0 | 红绿灯数据持续缺失多久告警（仍保持停车） |
| `traffic_action_timeout_s` | 3.0 | 序列动作等执行器结果的最长时间 |
| `voice_enabled` | True | 是否启用语音提示（`--no-voice` 可关） |
| `voice_topic` | `rt/voice/cmd` | SDK E7 语音播放 DDS 话题 |
| `voice_pack_dir` | `assets/voice` | 预生成自然女声 WAV 目录 |
| `voice_chunk_ms` | 100 | 每包 PCM 时长 |
| `voice_startup_s` | 1.0 | 语音进程 DDS 发现等待 |
| `voice_cache_size` | 32 | 固定短句 PCM 缓存条数 |
| `voice_tail_s` | 0.30 | 最后一包后等尾音结束 |
| `voice_wait_timeout_s` | 10.0 | 等语音回执的最长时间 |
| `max_heading_dev` | 90 | 单次避障允许偏离 `entry_yaw` 的最大角度 |
| `min_passage_m` | 0.70 | 候选方向的最低侧向空间 |
| `wall_near_ratio` | 0.60 | 中心近处占比高于它 → 当成整面墙 |
| `wall_extra_m` | 0.20 | 墙面情况下额外要求的侧向空间 |
| `rotate_clear_m` | 0.40 | 原地转身需要的最小净空 |
| `safety_buffer_m` | 0.10 | 动态步长保留的安全余量 |
| `max_avoid_cycles` | 24 | 单次绕障距离动作上限，含回切与中断 |
| `max_bypass_distance_m` | 6.0 | 累计各段实测路程上限，含回切 |
| `min_bypass_advance_m` | 0.90 | 回正后沿原方向绕出的初始目标距离 |
| `bypass_step_max_m` | 0.40 | 旧配置兼容保留，已不再用于绕障 |
| `bypass_step_min_m` | 0.10 | 净空不足最小步长时重规划 |
| `bypass_timeout_s` | 120.0 | 本次绕障总时限，含等待与播报 |
| `bypass_max_replans` | 8 | 重规划上限 |
| `bypass_pose_stale_s` | 0.50 | 平面位置及航向新鲜度 |
| `bypass_min_progress_ratio` | 0.50 | 成功动作沿指令方向的最小实测进展比例 |
| `path_return_enabled` | True | 绕障后横向回原路线 |
| `path_return_tolerance_m` | 0.12 | 横向误差小于它认为已回原线 |
| `path_return_angle_deg` | 60.0 | 绕回时的切入角 |
| `path_return_clearance_m` | 0.40 | 回切侧清空后按实测位移确认越过拐角 |
| `path_return_min_step_m` | 0.10 | 回切最小步长（同时受绕障最小步长限制） |
| `path_return_max_step_m` | 1.50 | 单次回切目标上限，不再受0.4m分段限制 |
| `path_return_max_attempts` | 6 | 最多回切尝试；耗尽仍偏线则停车 |
| `max_realign_attempts` | 12 | 含受阻回切的总回正次数上限 |
| `settle_s` | 0.25 | 动作结束后等新帧的时间 |
| `confirm_s` | 0.50 | 障碍/安全状态稳定确认时间 |
| `stable_delta_m` | 0.05 | 中心距离波动上限 |
| `stable_near_delta` | 0.05 | `near_ratio` 波动上限 |
| `approach_m` | 1.20 | 距离持续减小且低于它 → 判定为接近中的动态障碍 |
| `turn_timeout_s` | 5.0 | 转身超时 |
| `step_timeout_s` | 3.0 | 短距离前进超时 |
| `realign_timeout_s` | 5.0 | 回正超时 |
| `yaw_tol_deg` | 10.0 | 航向校验容差 |
| `blocked_recheck_s` | 2.0 | BLOCKED 后连续安全多久才恢复检查 |
| `roi_*` | 5%~55%, 30%~70% | 中心 ROI |
| `roi_side_*` | 5%~30%, 70%~95% | 左/右区域 |
| `percentile` | 5 | 取区域里最近的 5% 像素 |
| `min_zone_pixels` | 30 | 左/右区域最少有效像素 |

> `roi_bottom` 不要超过 0.60，否则会看到自己脚下的地面（约 0.5 m），导致一直报警。

> 启动时会先发一条停车指令；进入 HOLD/BLOCKED 后不会只发一次，而是每
> `stop_refresh_s` 补发一次零速。即使某次停车 RPC 失败或卡死，下一次心跳也会自动纠正。

## 实测（真机 miniQuad）

> 下表是旧版 `danger_m=0.6`、`safe_m=1.0`、左转 45° 时期的历史数据；
> 当前参数已改为 `danger_m=0.7`、`clear_m=1.0`、`forward_s=5.0`，
> 新的触发/恢复距离、绕障角度和回正误差需要重新实测。

| 指标 | 实测 |
|---|---|
| 深度帧率 | 19.4 ~ 20.5 FPS |
| 深度零值占比 | 趴着 98%（低于量程）→ 站立后 16.1% |
| 检测到危险 → 打断 | 11 ms |
| 打断 → 零速指令完成 | 0.54 s |
| 前进补发间隔 | 旧版 2.20 ~ 2.30 s；当前 `forward_s=5.0` 约 5.2 ~ 5.3 s |
| 危险线触发 | 0.48 ~ 0.60 m（旧参数） |
| 恢复 | 1.36 m（旧参数，回差生效） |
| Ctrl+C → 进程退出 | 0.86 s（含 0.5 s 刹车） |

> 前进流实际比 `forward_s` 长 0.2~0.3 s，是服务端的加减速时间。

## 真机首次验证建议

1. 先 `python3 tools/robot_check.py` 确认在线、站立、内置避障状态。
2. `python3 tools/voice_check.py --dry-run --wav assets/voice/bypass_left.wav`
   确认女声包；再 `--play --wav assets/voice/bypass_left.wav` 确认机器人扬声器
   有声音（只播语音，不移动）。
3. `python3 main.py --vx 0.2 --dry-run` 跑通框架；再上真机低速。
4. 在空旷场地放一个 0.5 m 左右的箱子，站在机器人侧后方，手放在急停上。
5. 观察日志：`HOLD → PLAN → TURN → CHECK → ADVANCE → REALIGN → GO`。
6. 重点确认：
   - 转向失败/被挡住时进入 `BLOCKED`，不会继续前进；
   - 绕完后机身朝向回到避障前（允许横向偏移）；
   - 人走近时机器人停车等待，人离开并稳定后恢复；
   - 语音先报绕行方向，绕完后报“已绕过障碍”，失败时报具体原因；
   - Ctrl+C 后内置避障和速度档位恢复原状态。

> RGB 可选体检：`python3 tools/rgb_probe.py`（只订阅 camera0，不移动机器人）。

## DDS 环境（相机收不到帧时先看这里）

`CYCLONEDDS_URI` 必须包含 `EnableTopicDiscoveryEndpoints`，否则后启动的 reader
（例如盲道相机）可能发现不了相机 publisher，表现为“已订阅但 0 帧”。

`main.py` 启动时会检查：

- 已设置，且内联 XML 或其指向的文件包含该选项 → 不动；
- 未设置，或内容缺少该选项，且 `$HOME/dobot_quad_sdk-main/cyclonedds.xml` 存在
  → 自动改用 `file://$HOME/dobot_quad_sdk-main/cyclonedds.xml` 并打印日志。

手动指定：

```bash
export CYCLONEDDS_URI=file://$HOME/dobot_quad_sdk-main/cyclonedds.xml
```

### 连续启动失败：participant index 耗尽

真机 `cyclonedds.xml` 还要设置 `MaxAutoParticipantIndex`（建议 32）。默认是 9，
一次运行会创建 5 个 participant（深度/语音/RGB/停止线/盲道），连续启动或强杀后
残留的 participant 会占住 index，表现为：

- 某个检测器只打印“已订阅”，但 0 帧；
- 启动握手 8s 超时（`停止线等待DDS订阅超时` / `盲道等待DDS订阅超时`）；
- 等约 1 分钟（旧 participant 租约过期）后又能启动。

真机已改为：

```xml
<Discovery>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
    <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints>
</Discovery>
```

备份在同目录 `cyclonedds.xml.bak.<时间>`。

## 同步到真机

开发在笔记本/容器里，真机运行的是 `robot@192.168.1.20:/home/robot/rover_x1_ws4`。
真机没装 rsync，`tools/sync_robot.sh` 用 `tar | ssh` 同步；**默认只比较，不改任何东西**。

```bash
bash tools/sync_robot.sh --diff      # 只读：比较两端源码 sha256
bash tools/sync_robot.sh --push      # 备份远端 → 推送 → 哈希校验 → 远端语法检查
bash tools/sync_robot.sh --pull      # 拉回 robot_snapshot/，供 diff -r 对比
```

规则：

- 需要 ssh 可用：先在终端 `ssh robot@192.168.1.20` 确认；免密用 `ssh-copy-id`。
- `--batch` 只允许免密登录（脚本/CI 用）；交互密码模式不要加。
- 推送前检查远端是否有 `python3 main.py` 在跑；有则拒绝，确认安全后加 `--force`。
- 推送前自动备份到 `/home/robot/rover_x1_ws4.bak.<时间>.tgz`。
- 只同步 `*.py` / `*.md` / `*.json` / `*.txt` / `*.wav`；
  排除 `__pycache__`、`rgb_images`、`depth_images`、`legacy`、`decisions.old_*`、`robot_snapshot`。
- 推送后用 sha256 清单逐文件校验，再在远端跑 `compileall`。
- 真机 Python 3.10 缺运行时依赖也能 `compileall`；运行前仍需 grpc/numpy/cv2 就绪。

## 加一个新的 worker（模板）

```python
import threading, time

class MyWorker(threading.Thread):
    def __init__(self, cfg, 输入信箱, 输出信箱):
        super().__init__(name="my_worker", daemon=True)
        ...
        self._stop_event = threading.Event()

    def run(self):                       # 线程入口，只有一个 while
        while not self._stop_event.is_set():
            ...
            time.sleep(0.02)

    def stop(self):
        self._stop_event.set()
```

规则：

1. 依赖全部从构造函数传进来，**不要 import 另一个 worker 的全局变量**；
2. 文件顶层不启动任何东西，`start()` 只在 `main.py` 里调；
3. 纯计算写成独立函数（如 `make_candidates()` / `compute_profile()` / `step()`），
   线程类只做外壳，这样离线回放和单元测试都能直接调用。

## 两个踩过的坑（别再踩）

**1. 属性不要叫 `_stop`**

`threading.Thread` 内部有个 `_stop()` 方法。子类里写 `self._stop = Event()`
会把它覆盖掉，`fork()` 之后解释器调用 `thread._stop()` 就会报
`TypeError: 'Event' object is not callable`。所以本项目统一用 `_stop_event`。

**2. DDS 的 `msg.data()` 返回的是 `list[int]`**

61 万个元素的 list 直接 `np.asarray()` 要 **35 ms/帧**，20 FPS 下 CPU 70%+。
改成 `bytes(msg.data())` + `np.frombuffer()` 只要 4 ms，CPU 降到 ~25%。

## 和旧版的区别

| 旧版 | 新版 |
|---|---|
| asyncio + 线程 + 进程（3 套模型） | 线程 + 进程（2 套；深度/RGB/语音各 1 个进程） |
| `last_vel` 去重，10 秒流跑完不再发 | `executor.busy`，空闲即补发 |
| 动作失败也当成成功，继续前进 | `CommandResult` + 超时 + yaw 校验，失败即 `BLOCKED` |
| 只左转 45°，固定前进 0.2 m | 左右 ±45°，各阶段整段执行，实测位置回原线 |
| 绕完不关心朝向 | 回到本次避障的 `entry_yaw`（允许横向偏移） |
| 启动即障碍不会触发避障 | `HOLD` 稳定后正常进入 `PLAN` |
| 动态障碍也可能被绕 | 不稳定/接近中的障碍只停车等待 |
| 内置避障关掉后不恢复 | 退出时恢复进入前的内置避障/速度档位；读不到原状态拒绝启动 |
| RPC 出错后不恢复 | `finally` 清 busy，下一拍自动重发，失败退避 |
| 无效深度画成红色，ROI 被带偏 | 感知只认 `>0` 像素，不再录制/可视化 |
| 每帧 35 ms 转换，深度进程 90% CPU | `bytes()` + `frombuffer`，深度进程 ~25% CPU |
| 退出卡 5 秒等流结束 | `stop()` 里先 `cancel()` |
| 单文件 450 行 | 每个 worker 60~150 行，避障逻辑独立在 `avoid.py` |



1. 纯避障完整程序（会走，默认 0.5 m/s）
ssh robot@192.168.1.20
cd /home/robot/rover_x1_ws4
export CYCLONEDDS_URI=file://$HOME/dobot_quad_sdk-main/cyclonedds.xml
python3 main.py
做什么：连接 gRPC → 起立 → 切 WALK → 50Hz 深度避障（GO→HOLD→PLAN→TURN→CHECK→ADVANCE→REALIGN）→ 语音播报障碍/绕行方向/完成 → Ctrl+C 停车刹车、恢复厂商避障和速度档位。
常用变体：
python3 main.py --vx 0.2                 # 首次真机降速
python3 main.py --no-voice               # 关播报
python3 main.py --dry-run                # 假机器人：相机+识别+状态机全跑，但不动机器人
2. 完整避障 + 停止线/红绿灯识别（会走，停止线不控车）
cd /home/robot/rover_x1_ws4
export CYCLONEDDS_URI=file://$HOME/dobot_quad_sdk-main/cyclonedds.xml
python3 main.py --no-traffic-sequence --traffic-light --traffic-light-now --stop-line 2>&1 | tee /tmp/run.log
- camera0 红绿灯立即识别（--no-traffic-sequence 关闭默认序列，只做检测）
- camera2 黄线每秒打 [停止线] 状态 visible=... row=...
- 运动仍由普通避障决定，看到停止线不会停
3. 完整流程：到停止线停车 → 红绿灯 → 通过（默认启用，会走）
cd /home/robot/rover_x1_ws4
export CYCLONEDDS_URI=file://$HOME/dobot_quad_sdk-main/cyclonedds.xml
python3 main.py --traffic-vx 0.4 --stop-line-trigger-row 0.83 2>&1 | tee /tmp/seq.log
python3 main.py --traffic-vx 0.4 --stop-line-trigger-row 0.83 2>&1 | tee /tmp/seq3.log
默认 main.py 自动开 camera0 红绿灯 + camera2 停止线，跑完一次序列后交回普通避障；中途丢失/无进展会进 WAIT_STOP_LINE 永久停车。
4. 不动机器人的完整验证（推荐第一次用这个）
cd /home/robot/rover_x1_ws4
export CYCLONEDDS_URI=file://$HOME/dobot_quad_sdk-main/cyclonedds.xml
python3 main.py --dry-run --traffic-light --traffic-light-now --stop-line 2>&1 | tee /tmp/dry.log
真实相机、真实识别、真实状态机，所有动作只打印 [dry-run] 执行 ...，机器人不会动。先确认日志里 camera0/camera2 有帧、[停止线] visible=True、红绿灯有 red/green/yellow，再上第 1/2/3 条。
启动前必查（两台相机都可能有 0 帧）
python3 tools/rgb_probe.py --topic rt/camera/camera0/image_compressed --seconds 3
python3 tools/rgb_probe.py --topic rt/camera/camera2/image_compressed --seconds 3
python3 tools/depth_probe.py
0 帧就先起厂商相机节点（另开终端，ROS_DOMAIN_ID=0），再回来跑程序。
停止方式
任何一条都是 Ctrl+C：会先停决策 → 打断执行器 → 停感知/相机/语音 → brake() → 关闭 gRPC，并恢复厂商避障开关和速度档位。不要直接关终端或拔电。



## 加长避障绕行

当前按不超过0.4m的短段运动；长障碍导致回切方向受阻时，沿原方向继续绕出并重新观察。总路程含回切按实测位移累计，默认上限6m、24次距离动作和120s。具体参数见 `config.py` 与 [绕障说明](docs/avoidance.md)。


# 终极版运行代码
cd /home/robot/rover_x1_ws4
export CYCLONEDDS_URI=file://$HOME/dobot_quad_sdk-main/cyclonedds.xml
python3 main.py --mission --vx 0.5


ROS 后续迁移进展：[阶段1记录](docs/ros2_phase1.md) · [接口契约](docs/ros2_interfaces.md)。

可选ROS相机输入及盘点命令见 [ROS相机输入迁移](docs/ros2_camera.md)。
