# ROS 相机输入迁移

状态：标准ROS图像输入适配、共享采集和离线验证已实现。默认仍为厂家DDS入口。2026-11-16在当前环境执行5秒只读盘点，未发现标准ROS图像话题；尚未验证设备的真实编码、时钟、帧率和端到端延迟。

按当前迁移优先级，连续绕障既有失败保留记录，暂不修复，不再阻塞相机迁移开发；这不代表完整导盲算法已经验收通过。

## 已接入的链路

```text
sensor_msgs/Image（深度） ─────────────→ 原 compute_profile → 原感知队列
sensor_msgs/CompressedImage（camera0） → 原 RGB/红绿灯处理链
sensor_msgs/CompressedImage（camera2） → 共享解码 → 原停止线/盲道检测函数与队列
```

`camera_backend=ros2` 时，相机进程为 `rover_x1_cameras` 节点，每个话题只有一个订阅。camera2同一帧只解码一次，标定要求不同宽度时分别缩放；不再同时启动厂家的深度、RGB、停止线和盲道相机订阅进程。感知结果仍进入现有队列，决策算法和运动出口保持原样。

camera0与camera2实际为同一压缩话题时也能复用订阅。红绿灯的独立检测进程仍保留自己的解码，等待后续感知节点化迁移。

## 盘点并配置真实话题

先使用与设备一致的ROS域、RMW及DDS网卡配置，然后构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths . ros2/rover_x1_interfaces --packages-up-to rover_x1
source install/setup.bash
ros2 run rover_x1 camera_inventory --seconds 5
```

盘点命令只读取ROS图像话题，不创建机器人对象、不调用起立、运动或语音。输出消息类型、发布者QoS、接收帧数与频率、编码/尺寸、源时间戳及其年龄。空结果表示当前ROS图中未发现标准图像话题，不能据此断言厂家没有相机数据。

厂家的 `rt/camera/...` 名称不能直接假定为ROS话题。如果设备只有厂家DDS消息，下一步需依据真实协议实现桥接；本次未构造未经验证的DDS桥接，也未发布CameraInfo或TF。

复制 `config/ros_camera.example.yaml` 为部署文件，按盘点结果替换三个占位话题：

| 参数 | 内容 |
|---|---|
| `camera_backend` | `dds`（默认）或 `ros2` |
| `ros2_depth_topic` | `sensor_msgs/msg/Image`，所有模式必需 |
| `ros2_rgb_topic` | camera0 `sensor_msgs/msg/CompressedImage`，启用RGB/交通任务时必需 |
| `ros2_camera2_topic` | camera2 `sensor_msgs/msg/CompressedImage`，盲道/停止线/串联任务必需 |
| `ros2_camera_startup_timeout_s` | ROS深度订阅及首帧各自的等待上限，默认8秒 |

话题必须为明确的绝对名称；深度与压缩RGB不能使用同一名称。绕障模式且未启用RGB时只需配置深度话题。camera2分支继续使用原有停止线/盲道的启动和首帧等待参数。

先只检查参数与资源，不启动设备：

```bash
ros2 launch rover_x1 rover.launch.py mode:=mission \
  params_file:=/absolute/path/ros_cameras.yaml check_config:=true
```

确认设备话题后，使用同一命令并去掉 `check_config:=true` 才进入原运行流程，包括连接、起立和运动。`dry_run` 仍仅模拟运动，不能用于声称完全离线。

回退时将部署文件中的 `camera_backend` 改为 `dds` 后重启；原有厂家 `depth_topic / rgb_topic / lane_topic / stop_line_topic` 配置继续有效。不会在ROS输入故障后自动切换数据源。

## 图像与时间约定

- 深度支持 `16UC1` 毫米与 `32FC1` 米；统一转换为原算法所用毫米，处理大小端和每行padding。浮点深度中的NaN、Inf、负数及零作为无效深度。
- 尺寸、步长、字节数或编码错误时丢弃；JPEG/PNG压缩彩色图使用原有OpenCV解码与缩放规则。无目标检测是正常输出，坏图不会伪装成“未检测到目标”。
- 输入QoS为 `best_effort + volatile + keep_last(depth=1)`，每次回调处理最新可用帧；不会为补齐旧帧而增加应用侧缓存。
- 当前适配要求源header为与主机同步的系统时间。没有时间戳、零时间戳、重复/倒序、未来或超过对应过期阈值的帧均拒绝，不用接收时间替换采集时间。
- 相同时间戳不能增加停止线或红绿灯稳定票数。源时间倒退后需修复时钟并重启入口；尚不支持设备相对时间或 `use_sim_time`。
- 感知结果和RGB帧保留采集时间；解码/计算结束时再次检查年龄及本机单调时钟耗时。过期结果不会因计算完成而刷新健康状态。
- 数据流失效后由现有决策过期保护停止；ROS相机进程退出时，主流程清空感知状态并退出。首帧握手要求有效深度和相应检测结果已进入决策白板。
- 进入掉头/盲道阶段时只停用停止线处理，不停止共享相机和盲道输入。原就绪检查发生在现有连接/起立流程之后、决策线程启动之前。

## 验证与剩余工作

本次双包构建成功，ROS包测试47项通过；安装后从源码目录外使用示例参数执行 `check_config` 通过。完整离线回归仍只有此前记录的3条复杂绕障失败，未新增失败。候选输入快照保存于 `docs/baselines/ros2_camera_candidate.json`，不覆盖阶段1快照。

```bash
python3 tools/test_ros_camera.py
source /opt/ros/humble/setup.bash
python3 tools/test_ros_camera_transport.py
colcon test --base-paths . ros2/rover_x1_interfaces --packages-select rover_x1_interfaces rover_x1
colcon test-result --verbose
```

单元测试覆盖深度单位/字节序/padding、同帧新旧检测结果一致、共享解码、坏帧/旧帧/慢解码拒绝、模式装配、共享相机存活与停止线分支关闭。真实传输测试使用本机隔离ROS域232、spawn子进程和合成图像，验证标准消息订阅、共享分发和正常退出，不构造机器人控制对象。

下一步盘点设备真实图像入口并测量延迟；已有标准话题则配置接入，只有厂家DDS则补桥接。随后将感知输出迁移到 `rover_x1_interfaces` 消息和独立节点。现阶段尚未完成真实相机验收。
