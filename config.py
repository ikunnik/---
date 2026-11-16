"""
集中配置：所有可调参数都在这里，改阈值不用翻代码。

单位约定：
    距离 = 米(m)      时间 = 秒(s)      频率 = 赫兹(Hz)      角度 = 度(deg)
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    # ---------------- 网络 / 设备 ----------------
    robot_ip: str = "192.168.5.2:50051"
    dds_config: str = (
        "/home/admin/dobot/dobot_quad_sdk-main/low_level/python/config/dds_config.yaml"
    )
    # CYCLONEDDS_URI 缺 EnableTopicDiscoveryEndpoints 时，改用这个文件（存在才用）
    dds_uri_file: str = "~/dobot_quad_sdk-main/cyclonedds.xml"
    # 运动后端：sdk=gRPC 50051（默认）；ros2=/planner/vel_cmd（实时运动）
    motion_backend: str = "ros2"       # 默认 ROS2；sdk 仅保留回退
    ros2_vel_topic: str = "/planner/vel_cmd"
    ros2_odom_topic: str = "/foot_odom/data"
    ros2_rate_hz: float = 20.0          # Twist 发布频率
    ros2_line_vx: float = 0.40          # line_walk 距离移动速度 m/s
    ros2_yaw_rate: float = 1.0          # 旋转角速度 rad/s
    ros2_min_vx: float = 0.30           # 速度死区，低于它抬到 0.3
    ros2_odom_stale_s: float = 0.50     # 里程计过期时间
    ros2_startup_timeout_s: float = 8.0 # 等 /planner/vel_cmd 订阅者
    ros2_cancel_zero_delay_s: float = 0.15  # 取消后无新目标才补零速
    # 相机输入：dds 保留厂家入口；ros2 使用显式配置的标准 ROS 图像话题。
    camera_backend: str = "dds"
    perception_backend: str = "queue"  # queue / ros2；ros2 需要 ROS 相机输入
    ros2_perception_namespace: str = "/guide/perception"
    ros2_depth_topic: str = ""       # sensor_msgs/Image: 16UC1(mm) / 32FC1(m)
    ros2_rgb_topic: str = ""         # camera0 sensor_msgs/CompressedImage
    ros2_camera2_topic: str = ""     # camera2 CompressedImage，停止线/盲道共享
    ros2_camera_startup_timeout_s: float = 8.0
    depth_topic: str = "rt/camera/camera2/image_depth"   # 前置深度相机
    # camera0 = 前置纯 RGB（CompressedImage/JPEG）；camera2 = 前置深度相机的 RGB（与深度同轴）
    rgb_topic: str = "rt/camera/camera0/image_compressed"
    rgb_enabled: bool = False        # 是否启用 RGB 采集（main.py --rgb）
    rgb_log_s: float = 5.0           # RGB FPS / 无帧 / 转发日志间隔
    traffic_light_enabled: bool = False   # 红绿灯检测（camera0；--traffic-light）
    traffic_light_profile: str = "assets/traffic_light_profile.json"
    stop_line_enabled: bool = False       # 黄线停止线检测（camera2；--stop-line）
    stop_line_topic: str = "rt/camera/camera2/image_compressed"
    stop_line_profile: str = "assets/stop_line_profile.json"
    # --mangdao：独立盲道模式，仅保留稳定循环纠偏（原方案4）。
    # 新转向目标达到阈值后取消旧循环，交由同一执行器重新下发。
    lane_follow_enabled: bool = False
    lane_no_line_straight: bool = True    # 新鲜图像未检出盲道时直行；无帧/过期仍停车
    lane_profile: str = "assets/lane_profile.json"
    lane_topic: str = "rt/camera/camera2/image_compressed"
    lane_startup_timeout_s: float = 8.0
    lane_frame_timeout_s: float = 5.0
    lane_center_ref: float = 0.50          # 摄像头标定后可调整，不一定永远是图像正中
    lane_forward_vx: float = 0.40          # m/s；先用保守速度，实机确认步态稳定后再调整
    lane_control_duration_s: float = 1.20  # 单段速度目标时长；太短会频繁启停，太长则纠偏更新慢
    lane_trial_seconds: float = 0.0  # 盲道最长运行时间；0=不限时，持续运行
    lane_batch_steps: int = 2               # 循环序列内两个速度步骤
    lane_loop_update_threshold: float = 0.12 # yaw 变化超过此值才重新发指令
    lane_loop_min_update_s: float = 2.5     # 避免频繁打断导致再次顿挫
    lane_loop_heartbeat_timeout_s: float = 1.0  # 决策线程失活，执行器主动取消+刹车
    lane_min_confidence: float = 0.15
    lane_filter_alpha: float = 0.25        # 感知误差 EMA 系数
    lane_cross_deadband: float = 0.018     # 归一化中心误差死区
    lane_heading_weight: float = 0.08      # heading 归一化后加入横向误差
    lane_kp: float = 0.80                 # (rad/s) / 归一化误差
    lane_kd: float = 0.02                 # 对误差变化率的阻尼系数
    lane_max_yaw_rate: float = 0.18       # rad/s，转向速度限幅
    lane_yaw_accel_limit: float = 0.12    # rad/s^2，转向速度变化率限制
    lane_max_center_jump: float = 0.06
    lane_max_heading_jump_deg: float = 6.0
    lane_steer_sign: float = -1.0         # 保留原项目方向约定；首次实机低速验证正负方向
    lane_stale_s: float = 0.80        # 盲道帧间隔最多 8FPS；0.4s 容易误判过期
    lane_reacquire_enabled: bool = True  # 丢线后按最后线侧搜索一下
    lane_reacquire_yaw_rate: float = 0.12
    lane_reacquire_seconds: float = 3.0
    stop_line_stale_s: float = 0.5        # 停止线状态超过这个时间未更新就算过期
    # 启动握手：子进程订阅成功 / 收到第一帧的最长等待时间。超时说明 DDS
    # 初始化卡住或话题没有数据；--traffic-sequence 会直接中止，绝不带着
    # 失效的检测器前进。
    stop_line_startup_timeout_s: float = 8.0
    stop_line_frame_timeout_s: float = 5.0
    # 停止线序列：默认启用（红绿灯 + 停止线 + 到线停车/信号/通过）；
    # 只跑普通避障用 --no-traffic-sequence 关闭。
    traffic_sequence_enabled: bool = True
    # 0.15 / 0.30 m/s 实测都低于 miniQuad 步态有效速度：traffic_forward 一直
    # “完成”，但黄线 row 不走（0.3 m/s 跑 11.5s 只前进约 0.35m 后锁存）。
    # 最低 0.40，main.py 会拒绝更低的 --traffic-vx。
    traffic_vx: float = 0.40
    # 前进指令太短会被服务端加减速吃掉（0.5s 命令实际耗时 ~0.77s）。
    # 1.0s 让每段有真正的匀速时间；到线时仍然是 50Hz 检查 + 打断，不会多冲。
    traffic_forward_s: float = 1.0
    traffic_stall_s: float = 4.0          # 黄线/深度多久没进展就停车交回避障
    traffic_stall_row: float = 0.02       # 认为“有进展”的最小行号增量
    stop_line_trigger_row: float = 0.83   # 黄线底边到这个行号就停（0.844 是停车位）
    stop_settle_s: float = 0.25           # 停稳后再开红绿灯识别
    stop_line_align_enabled: bool = True  # 到停止线后按黄线倾角微调 yaw
    stop_line_align_threshold_deg: float = 2.0 # 倾角达到2°才回正
    stop_line_align_max_deg: float = 30.0  # 单次最大回正角度
    stop_line_align_gain: float = 1.7    # 实测倾角偏小，乘一点再转
    stop_line_align_max_attempts: int = 2
    stop_line_align_sign: float = 1.0     # 方向反了就改 -1
    traffic_pass_m: float = 1.5           # 绿灯后先按 line_walk 前进（米），0=直接找障碍
    traffic_pass_timeout_s: float = 10.0  # 2m 通过动作等待结果的上限
    traffic_pass_stop_m: float = 0.7      # 2m 途中前方小于它就提前停车掉头
    traffic_pass_guard_s: float = 2.0     # 起步后忽略这段时间（避开地面/停止线）
    traffic_state_stale_s: float = 0.5    # 红绿灯状态超过这个时间算过期
    traffic_signal_timeout_s: float = 10.0  # 红绿灯数据持续缺失多久就告警（保持停车）
    traffic_action_timeout_s: float = 3.0 # 序列动作等执行结果的最长时间（forward_s=1.0 + 加减速）
    speed_ratio: int = 60                                 # 机器人速度档位 10~100
    pose_timeout_s: float = 0.30                          # 查询姿态的单次 RPC 超时
    pose_cache_s: float = 0.05                            # 姿态缓存 50ms，避免 50Hz 打爆 RPC

    # ---------------- 感知 ----------------
    danger_m: float = 0.7        # 危险线：低于它就停车并避障
    clear_m: float = 1.0         # 允许前进 / 回正的最小中心距离
    stale_s: float = 0.5         # 多久没收到感知就认为失效
    min_valid_pixels: int = 50   # 中心 ROI 内有效像素少于这个数 → 判定无效
    min_zone_pixels: int = 30    # 左/右区域有效像素少于这个数 → 该区域不可用

     # ---------------- 语音提示 ----------------
    voice_enabled: bool = True
    voice_topic: str = "rt/voice/cmd"       # SDK 低层 E7 语音播放话题
    voice_task_id: str = "rover_avoid"      # VoiceCmd.task_id 前缀
    voice_pack_dir: str = "assets/voice"    # 预生成自然女声 WAV 目录
    voice_chunk_ms: int = 100               # 每块 PCM 时长
    voice_startup_s: float = 1.0            # DDS writer 发现时间
    voice_wait_timeout_s: float = 10.0
    voice_cache_size: int = 32              # 固定短句 PCM 缓存条数
    voice_tail_s: float = 0.30              # 最后一包后等尾音结束再回执

    # ROI：只看画面中心的一条带子（上, 下, 左, 右，按比例 0~1）
    # 注意：下边界不要超过 0.60，否则会看到自己脚下的地面（约 0.5m）导致永远报警
    roi_top: float = 0.05
    roi_bottom: float = 0.55
    roi_left: float = 0.30
    roi_right: float = 0.70

    # 左/右区域：与中心同一条水平带，分别看左右两侧
    roi_side_left_l: float = 0.05
    roi_side_left_r: float = 0.30
    roi_side_right_l: float = 0.70
    roi_side_right_r: float = 0.95

    percentile: float = 5.0      # 取区域里最近的 5% 像素

    # ---------------- 交通 → 掉头 → 盲道流程 ----------------
    mission_enabled: bool = True        # 默认串联：红绿灯 → 掉头 → 盲道；--no-mission 关闭
    uturn_forward_m: float = 1.00       # 两次左转90°之间的直行距离
    uturn_ignore_obstacles: bool = False  # 掉头时跳过所有深度净空检查（仅空旷场地）
    uturn_check_forward_space: bool = False  # 掉头直行前是否要求 1.9m 净空
    uturn_pose_stale_s: float = 0.5
    uturn_sensor_wait_s: float = 2.0
    supervisor_period_s: float = 0.05
    supervisor_timeout_s: float = 1.0
    controller_switch_timeout_s: float = 4.0  # 切换前必须收到停车成功回执
    # 掉头失败分级：航向偏差在容差内允许补转一次，超过才锁停
    uturn_retry_tol_deg: float = 20.0
    uturn_max_retries: int = 1

    # ---------------- 决策 ----------------
    decision_hz: float = 50.0    # 状态机运行频率

    # ---------------- 速度指令 ----------------
    forward_vx: float = 0.5     # 前进速度 m/s
    forward_s: float = 5.0       # 每条前进指令持续 5 秒，跑完自动补发
    stop_s: float = 0.3          # 零速指令持续时间
    stop_refresh_s: float = 1.0  # STOP/BLOCKED 状态下每隔多久补发一次零速
    max_forward_vx: float = 0.6  # --vx 允许的最大值，防止误操作

    # ---------------- 自动避障 ----------------
    max_heading_dev: float = 90.0   # 单次避障允许偏离 entry_yaw 的最大角度
    min_passage_m: float = 0.70     # 候选方向的最低空间
    wall_near_ratio: float = 0.60   # 中心近处占比高于它 → 当成整面墙，要求更宽侧向空间
    wall_extra_m: float = 0.20      # 墙面情况下额外要求的侧向空间
    rotate_clear_m: float = 0.20    # 原地转身需要的最小净空
    safety_buffer_m: float = 0.10   # 动态步长保留的安全余量
    max_avoid_cycles: int = 24       # 单次绕障的距离动作上限（含回切与中断）
    max_bypass_distance_m: float = 6.0  # 单次绕障累计实际路程上限，包含回切
    min_bypass_advance_m: float = 0.80  # 回正后并肩前进的初始目标距离，净空足够时一次执行
    bypass_step_max_m: float = 0.40      # 旧配置兼容保留；绕障不再使用固定分段上限
    bypass_step_min_m: float = 0.10
    bypass_timeout_s: float = 120.0     # 包括等待/播报/重规划的总时限
    bypass_max_replans: int = 8
    bypass_pose_stale_s: float = 0.50
    bypass_min_progress_ratio: float = 0.50  # 成功回执仍需实测位移验证
    path_return_enabled: bool = True     # 绕障后横向绕回原路线
    path_return_tolerance_m: float = 0.12  # 横向误差小于它认为已回到原路线
    path_return_angle_deg: float = 60.0  # 绕回时的切入角
    path_return_clearance_m: float = 0.40  # 回切侧清空后再沿原方向确认越过拐角
    path_return_min_step_m: float = 0.10
    path_return_max_step_m: float = 1.50
    path_return_max_attempts: int = 6
    max_realign_attempts: int = 12   # 单次绕障回正次数上限（含受阻回切）
    settle_s: float = 0.25          # 动作结束后等新帧的时间
    confirm_s: float = 0.50         # 障碍/安全状态稳定确认时间
    stable_delta_m: float = 0.05    # 中心距离波动上限
    stable_near_delta: float = 0.05 # near_ratio 波动上限
    approach_m: float = 1.20        # 距离持续减小且低于它 → 判定为接近中的动态障碍
    turn_timeout_s: float = 5.0     # 转身超时
    step_timeout_s: float = 3.0     # 短距离前进超时
    realign_timeout_s: float = 5.0  # 回正超时
    yaw_tol_deg: float = 10.0       # 航向校验容差
    emergency_m: float = 0.20       # 所有绕障/回切前进的紧急打断距离
    blocked_recheck_s: float = 2.0  # BLOCKED 后连续安全多久才恢复检查

    # ---------------- 日志 ----------------
    warn_log_interval_s: float = 1.0   # 危险/失效日志最多每秒一条


CFG = Config()

# 停止线序列允许的最低速度：低于它 miniQuad 只原地踏步，不产生有效位移。
# main.py 用它校验 --traffic-vx，selftest 用它做回归。
MIN_TRAFFIC_VX: float = 0.40
