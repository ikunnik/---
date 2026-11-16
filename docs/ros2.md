# ROS 2 包化与统一启动

包名 `rover_x1`，面向当前 ROS 2 Humble / Python 3.10 环境。复用现有感知、决策、执行器和进程退出流程；此阶段未将所有 worker 拆成独立 ROS 节点。

## 构建

在已有 ROS 2 Humble 和 colcon 的开发环境中执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/admin/rover_x1_v7.5
colcon build --base-paths . ros2/rover_x1_interfaces --packages-up-to rover_x1
source install/setup.bash
```

完整运行仍需要厂家 `dds_middleware_python` 和 `dobot_quad` SDK，以及原有 DDS 配置文件。它们不会被打进本包，也不会被构建过程自动安装。普通 ROS/Python 依赖列在 `package.xml`，厂家依赖安装说明见 `requirements.txt`。

语音 WAV 和三份标定 JSON 随包安装到 `share/rover_x1/assets`。从其他目录启动时也能找到它们，不依赖当前工作目录。包保留原有 Python 模块布局，避免改变现有脚本与工具的导入方式。

## 只检查配置

下面两条命令不会连接机器人、启动相机或播放语音；仅创建 ROS 参数节点、校验参数和资源路径，输出实际配置后退出：

```bash
ros2 launch rover_x1 rover.launch.py check_config:=true
ros2 run rover_x1 controller --ros-args -p check_config:=true -p mode:=avoidance
```

## 统一启动

配置实际网络的 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION` 和 DDS 网卡后启动。网络设置沿用 README 的 ROS 2 运动后端说明。以下命令会运行机器人控制流程，包括起立：

```bash
# 原有串联任务：红绿灯 → 掉头 → 盲道
ros2 launch rover_x1 rover.launch.py

# 普通绕障
ros2 launch rover_x1 rover.launch.py mode:=avoidance

# 停止线与红绿灯
ros2 launch rover_x1 rover.launch.py mode:=traffic

# 独立盲道
ros2 launch rover_x1 rover.launch.py mode:=lane
```

`Ctrl+C` 沿用原有取消、停止和关闭进程流程；统一入口不自动重启控制进程。`dry_run:=true` 仅模拟运动，仍会使用实际 DDS 相机和语音，纯配置检查请用 `check_config:=true`。

## YAML 参数

源码默认文件为 `config/rover.yaml`，由 `config.py` 自动生成。修改通用默认值后运行 `python3 tools/generate_ros_config.py` 并重新构建；`python3 tools/generate_ros_config.py --check` 检查是否漂移。部署调参请另存覆盖文件，在启动时传入绝对路径：

```bash
ros2 launch rover_x1 rover.launch.py mode:=avoidance params_file:=/absolute/path/my_rover.yaml
```

文件结构：

```yaml
/**:
  ros__parameters:
    min_bypass_advance_m: 0.9
    path_return_angle_deg: 60.0
    path_return_clearance_m: 0.4
    danger_m: 0.7
    voice_enabled: true
```

小数参数请保留小数点，例如 `60.0`；非法类型、非有限数、未知参数和不合法的距离阈值会在连接机器人前报错。`mode` 决定任务与检测器开关，优先于单独的 `mission_enabled`、`lane_follow_enabled`、`traffic_sequence_enabled`、`traffic_light_enabled`、`stop_line_enabled`；参数查询显示的是归一化后的实际值。

配置优先级：`config.py` 后备值 → YAML → 显式 ROS 参数覆盖 → 运行模式决定关联开关。launch 的 `mode`、`dry_run`、`check_config` 由同名启动参数覆盖。

参数是启动时快照，不支持运行中调参。修改部署覆盖 YAML 后重启生效；运行中的 `ros2 param set` 会被拒绝：

```bash
ros2 param get /rover_x1_controller min_bypass_advance_m
ros2 param dump /rover_x1_controller
```

相对资源路径以包的 share 目录为基准；自定义外部资源建议使用绝对路径。`dds_config`、`dds_uri_file` 仍指向设备上的厂家配置，请按实际安装位置修改。

## 范围

`/rover_x1_controller` 提供参数服务；运动后端保留独立的 `/rover_x1_motion` 节点。两者使用独立 ROS 上下文，launch 对主节点的重命名与参数不会污染运动节点。相机默认通过厂家 DDS，可通过 `camera_backend=ros2` 切换到标准图像订阅；语音仍通过厂家 DDS，其他 worker 仍在同一控制进程体系内。现有逻辑使用系统时间，暂不支持 `use_sim_time`。

原有 `python3 main.py ...` 入口继续可用，其参数来源仍为 `config.py` 和 CLI，不读取 ROS YAML。

## 验证

```bash
colcon test --packages-select rover_x1
colcon test-result --verbose
```

包测试覆盖模式映射、YAML 与路径、实际 ROS 参数只读、节点名称及上下文隔离、原运行时退出流程；ROS 测试使用本机隔离域231与专用测试话题，不调用厂家SDK。原有完整算法检查仍使用 `bash tools/check.sh`。

包元数据中的维护者邮箱是占位符，公开分发前应填写实际维护信息并确认许可证。

后续节点化的阶段、接口和验收计划见 [ROS 2 后续迁移计划](ros2_migration_plan.md)。

阶段1进度与已知回归失败见 [阶段1记录](ros2_phase1.md)，消息定义见 [接口契约](ros2_interfaces.md)。

相机输入切换、只读话题盘点和时间戳要求见 [ROS相机输入迁移](ros2_camera.md)。
