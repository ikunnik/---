# ROS 接口契约 v1

本阶段新增 `rover_x1_interfaces` 消息包和 `rover_x1/conversions.py` 转换函数。它们尚未接入运行中的 worker；现有控制链保持原样。接口版本为 0.1.0 / 契约 v1。

## 构建与发现

消息包源码暂放在 `ros2/rover_x1_interfaces`。当前仓库根目录本身已经是一个包，因此必须显式列出两个搜索入口，不能依赖 colcon 自动递归发现嵌套包：

```bash
source /opt/ros/humble/setup.bash
cd /home/admin/rover_x1_v7.5
colcon list --base-paths . ros2/rover_x1_interfaces
colcon build --base-paths . ros2/rover_x1_interfaces --packages-up-to rover_x1
source install/setup.bash
ros2 interface show rover_x1_interfaces/msg/ObstacleProfile
colcon test --base-paths . ros2/rover_x1_interfaces --packages-select rover_x1_interfaces rover_x1
colcon test-result --verbose
```

以后迁入标准工作空间时，可把两个包分别放入 `src/rover_x1` 和 `src/rover_x1_interfaces`，不需要修改包内部定义。新接口是包测试依赖，当前控制器入口还不依赖这些消息。

## 数据映射

| 原类型 | ROS 消息 | 说明 |
|---|---|---|
| `Perception` | `ObstacleProfile` | 中/左/右距离、有效性、最小距离、近处像素比例 |
| `LaneState` | `LaneObservation` | 保留中心、倾角、置信度、双线/单线信息 |
| `StopLineState` | `StopLineObservation` | 可见性、归一化行号、面积、ROI边框、图像倾角 |
| `TrafficLightState` | `TrafficLightObservation` | 颜色、稳定性、ROI边框及面积 |
| `CommandResult` | `MotionResult` | 增加完整任务标识，保留运行/成功/失败/取消结果 |
| `VoiceCue` | `VoiceEvent` | 语义事件、方向、原因及原序号 |
| `VoiceDone` | `VoiceResult` | 保留本地发送结果，增加完整任务标识 |

相机图像将使用标准 `sensor_msgs/Image` / `CompressedImage`；姿态输入继续使用 `nav_msgs/Odometry`。`Velocity / VelocityPlan / Motion / ActionRequest` 的类型化动作接口在执行器阶段落实，本阶段不提供运动 Action Server、语音 Action Server 或新的控制话题。

`state_to_ros(value, frame)` 和 `state_from_ros(msg)` 支持四类感知状态；后者同时返回原 dataclass 与 `FrameContext`。调用方必须检查上下文健康、新鲜度及状态的有效标记后才能将结果交给决策。

`event_to_ros(value, task)` 和 `event_from_ros(msg, expected_task=...)` 支持结果和语音事件；读取时强制匹配会话、代数和目标三项。转换过程不初始化 ROS 节点或连接硬件。

## 单位、坐标和无效值

- 距离单位米，归一化图像位置和面积比例无量纲；边框为 ROI 内 `(x, y, width, height)` 像素，未检测到目标时为空数组。
- 过渡接口保留现有显式命名的 `heading_deg / angle_deg`，单位度。盲道正倾角表示远端中心向图像右侧偏；停止线正倾角表示右端向下。它们不是世界坐标系中的机身航向。
- `source.frame_id` 必须来自采集源；未知可为空，不能猜测真实 TF。障碍距离是既有相机 ROI 深度统计，不等同于机身碰撞体距离。
- 无效深度使用对应有效标志为 false，原有 `inf` 可无损通过序列化；valid=true 的区域距离必须有限且非负。
- `visible=false` 或 `color=unknown` 可以是正常检测结果；`observation.detector_ok=false` 表示检测失败/输入不可用。消费者不得将后者解释为道路畅通。
- `MotionResult.status=ok` 仅保留现有动作成功语义，不单独证明机身停稳。`VoiceResult.ok` 仅是本地发送加尾音等待结果，不代表扬声器确认播放完成。

## 时间与来源

`ObservationHeader` 字段：

| 字段 | 约定 |
|---|---|
| `source.stamp` | 原始采集时间，使用整数秒/纳秒保存，不由检测时间补造 |
| `source_stamp_valid` | 显式说明是否具有采集时间；false时stamp为0 |
| `clock_domain` | UNKNOWN=0、SYSTEM=1、ROS=2、DEVICE=3；不能跨时钟直接相减 |
| `stream_id` | 采集会话的规范非零UUID；采集进程重启后换UUID |
| `sequence` | 同一采集会话中递增的帧号，各检测器保留原帧号 |
| `detector_ok` | 该结果是否经过有效检测；不等于目标可见性 |

`legacy_ts` 无损保留旧 dataclass 的系统秒数，仅供现有算法适配，不能因此声称已获得真实采集时间。已知采集时间为0与缺失采集时间不同。

每个消费者在收到消息时自行记录 `time.monotonic()`；不通过网络传递某个进程的单调时钟值。存活、等待、执行超时使用本机单调时钟；当前旧决策的时间模型尚未切换，也未启用 `use_sim_time`。

`FrameFreshness` 是每个输入流独立使用的本地进度检查器：重复/倒序帧、同源时间倒退或不变、时钟域突变和不同采集会话均不刷新新鲜度。新会话需明确重新初始化检查器。超过接收年龄阈值或检测失败时不报告健康。

此检查器目前仅被离线测试使用。它无法仅靠接收时间判断首帧在网络中已滞留多久；没有采集时间时也不能识别内容相同但帧号被错误递增的旧帧。后续采集适配必须验证源序号/时间，并对可比较的源时钟增加传输年龄检查。

## 任务标识

`TaskIdentity = session_id + generation + goal_id`：协调器启动时生成新会话UUID；每次取消或撤销控制权增加generation；每个新任务使用新的goal UUID。原有tag、语音seq只用于业务关联，不能代替全局身份。

取消后的迟到结果即使tag相同也必须拒收。后续执行节点需要在接收目标、取消、排队和回执各环节落实这些规则；仅有消息字段和转换检查不等于当前运行时已经实现跨进程控制权保护。

## 验证

`tools/test_ros_contracts.py` 验证时间/任务边界；`tools/test_ros_conversions.py` 使用实际生成的消息与 ROS 序列化验证字段往返及错误回执拒绝。测试不创建节点，不启动DDS相机，不发送运动指令。
