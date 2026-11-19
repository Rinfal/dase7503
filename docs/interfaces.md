# 软件接口与联调约定

## 已实现的主机节点

| 节点 | 输入 | 输出/服务 |
| --- | --- | --- |
| `qr_scanner` | `camera/image_raw` Image | `qr/detections` String JSON、`qr/annotated` Image、PNG/JSONL 证据 |
| `safety_gate` | `cmd_vel_requested` Twist、`scan` LaserScan、`odom` Odometry、`safety/estop` Bool | `cmd_vel` Twist、`safety/status` String、`safety/arm` Trigger、`safety/disarm` Trigger、`safety/set_estop` SetBool |
| `odom_tf` | `odom_raw` Odometry | `odom`、`odom → base_link` TF |
| `mission` | QR、Nav2 action result、`mission/feedback`、安全状态 | `navigate_to_pose` action、`mission/requests`、`mission/status` |
| `keyboard_teleop` | 终端按键 | `cmd_vel_requested` Twist |
| `lidar_ms200p` | 明确指定的雷达串口 | `scan` LaserScan、`lidar/status`；不发送串口配置命令 |

已读到厂家 V2.1.0 固件，使用 serial transport、921600 baud、ROS_DOMAIN_ID=20。下一步启动代理后读取真实话题再适配；现阶段没有假定现有控制板使用 `/cmd_vel` 或 `/servo_s2`。

开发环境默认 ROS 自动发现范围为 LOCALHOST，所有控制节点运行在本树莓派；后续需要电脑跨主机遥控时再核对网络配置与域号。

正常地面运动只允许一个运动请求来源。Nav2 输出须重映射为 `cmd_vel_requested`，不能直接发布到控制板。手动模式与自主模式互斥，停止前一个来源后才启动另一个。

安全门控初始急停锁定，且 `hardware_ready=false`。解急停不会自动解锁运动；显式解锁还需新鲜且合法的激光和里程计。速度请求超过 0.25 秒、传感器超过 0.3 秒、点云不完整、非法数值或障碍进入停车包络均停车并锁定。当前保守地检查完整 360°，因此也保护横移和旋转；不是完成标定的碰撞防护系统。

雷达激光坐标原点需要靠近底盘原点，实际偏移必须计入 `robot_radius` 保守包络。该半径需覆盖带载轮廓，不能直接把默认值当作实测值。安全节点不替代 ESP32 硬件侧命令超时；主机崩溃时，只有板端看门狗能保证失联停车。

当前安全节点要求激光，因此雷达未接入时不会放行地面运动。`tools/bench_pulse.py` 提供独立的悬空短脉冲测试：必须显式确认悬空、指定实测控制板订阅及域号，线速度固定 0.03 m/s、角速度固定 0.15 rad/s、最长 0.3 秒，另有独立进程补发停车。该工具尚未进行真实电机验证，不代替板端看门狗，不能用于地面导航，也不伪造扫描数据。

## 任务流程

`wait_start → nav_pickup → identify → dock → lift → nav_drop → verify_end → lower → retreat → 下一货架/finished`

START 后最多 180 秒；任何阶段默认最多 25 秒。只接受本组目标货架的完整 QR。START 不能重置运行中的计时。END 必须在送达区导航完成后重新读到。放下且退出成功后才计为内部“已执行完一轮”，现场是否成功仍由实际货架位置及裁判决定。

每个动作有单调递增 token，每次 ROS 任务进程有随机 session。旧导航结果或其他动作反馈不能推进当前状态。失败、超时、退出和安全失锁都会取消导航并请求锁定运动。带载故障时停止底盘，不自动下降平台。

对接/升降执行器尚未实现，需机械标定和板端状态反馈。请求格式：

```json
{"session":"本次进程生成的值","token":4,"kind":"lift","value":"up"}
```

反馈格式：

```json
{"session":"与请求相同","token":4,"kind":"lift","success":true,"detail":"已验证的实际完成状态"}
```

不可仅因命令已经发布或经过固定延时就伪造实际抬升/离地成功；如果机构没有反馈传感器，需要明确采用何种标定与可验证条件。

## MS200p 解析边界

依厂家手册实现小端点云帧、毫米换算、顺时针到 ROS 逆时针角度转换和低置信度无效测量。CRC 使用多项式 0x4D，需真实报文验证。只在完整转过一周后发布；未知距离保持 NaN，不把断流/无效测距变成“前方无障碍”。

LaserScan 以 1° 格点保留最近距离，不对未知测量插值。原始采样方向与 ROS 角度排序相反，当前 `time_increment=0`，未做逐点运动畸变补偿；首次联调限低速，精度用实测评估。程序不读取或改变雷达模式/转速，默认上电模式须由真实设备确认。

## 外部依赖与参考

- ROS 2 Jazzy 官方安装：<https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>
- Raspberry Pi libcamera：<https://github.com/raspberrypi/libcamera>
- camera_ros：<https://github.com/christianrauch/camera_ros>
- 控制板厂家：<https://github.com/YahboomTechnology/MicroROS-Board>
- MS200p 手册：<https://oradar.com.cn/qinFiles/2024/09/d210f4202409091048581967.pdf>

先前 ZIP 的固件采用 Wi-Fi/UDP，并使用旧机械尺寸、旧舵机范围；不能直接烧入本车或照搬其参数。其 SDK 结构仅用于交叉核对协议，不使用其学生程序。
