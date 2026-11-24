# 遥控固件 dase-rc-0.10

本项目重新编写的四通道开环电机输出与编码器读取固件。GPIO 对照
[Yahboom 官方控制板资料](https://github.com/YahboomTechnology/MicroROS-Board)，
不复制往届学生控制程序。原厂完整 Flash 双份备份及哈希保存在
`runtime/firmware-backups/20261119T064817Z/`，含私有配置，不上传。

## 接口

串口 UART0 GPIO43/44，921600 baud，ROS domain 20。

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/wheel_pwm` | `std_msgs/msg/Int32MultiArray` | M1、M2、M3、M4 四路有符号占空比，单位千分之一，固件上限800（0.9及更早版本上限600） |
| `/drive_arm` | `std_msgs/msg/Bool` | 必须先收到200ms内的零命令，才允许显式解锁；false停车并撤销解锁 |
| `/drive/status` | `std_msgs/msg/String` | JSON，包含PWM、编码器累计值、解锁、超时、板端微秒时钟 |
| `/servo_s2_us` | `std_msgs/msg/Int32` | S2脉宽微秒；仅静止时接受，接收后2秒禁止驱动底盘 |

上电不解锁，不主动生成舵机PWM；电机独立任务每5ms检查命令时间，
250ms未收到新轮指令则输出归零并撤销解锁。恢复通信不能直接恢复运动。
上述时间为软件设计值，实测证据另存 `docs/evidence/`。

此阶段没有闭环轮速、IMU、标定过的电池电压或低电压报警（只报告原始ADC），不能沿用原厂固件的
遥测和低电保护验收记录。没有订阅旧 `/cmd_vel` 或角度 `/servo_s2`。
舵机150..2300us只是协议拒绝范围，并非机构已验证的安全行程。
正值为持续位置控制，负值为该绝对脉宽的空载探测（低于400us持续40ms，其余120ms），随后独立任务强制GPIO21拉低。
这限制的是信号持续时间，不限制舵机扭矩，也不保证撤去信号后机械立即停住。
`servo_us`是最后指令；`servo_active`是信号状态；`servo_edges`、`servo_high_us`是同一GPIO实际边沿和最近有效高电平宽度，不能证明舵机供电、输出轴运动或平台位置。
`servo_requests`累计已接受的请求，供探测工具检查应答。上电不会主动回中。

`wd_command_us`、`wd_stop_us`、`wd_settled_us`及`wd_encoders`由独立电机任务记录超时归零和编码器停止，通信恢复后仍可读取。代理运行时命令静默的四轮停车已通过；代理暂停实验需以对应新证据的verified字段为准。

## 构建和悬空调试

```bash
bash tools/build_controller.sh
bash tools/flash_controller.sh --wheels-raised --unloaded
bash tools/start_agent.sh
```

构建脚本串行运行colcon、Make和Ninja；运行时再用 `nice -n 12` 降低优先级。
上游组件的本地补丁见 `upstream-fixes.patch`，依赖提交见 `dependencies.lock.json`。
刷写前核验完整备份、原厂分区布局和配置保留范围，禁止覆盖NVS与yb_data。

观察轮胎上缘朝车头为前，朝车尾为后。本车已悬空逐轮验收：M1右前、M2左前、M3右后、M4左后，物理前进PWM与编码器符号均为 `[-1,+1,-1,+1]`。PWM600四轮能持续运行，450不起转；尚无地面速度标定。不可沿用原厂Twist的轮向修正符号。

```bash
source tools/environment.bash
python3 tools/bench_wheels.py --wheels-raised --unloaded \
  --pwm=-600,600,-600,600 --duration 15 \
  --report docs/evidence/wheel_raw_positive.json
```

工具检查新固件状态、其他发布者和显式解锁，主进程及独立进程均停车。
工具的PWM上限为600；每次脉冲最长15秒，不能用作地面遥控入口。

0.10新增800硬上限，250ms板端命令看门狗、上电零输出及舵机协议不变。
当前本机网页配置为 `forward_pwm: 750`、`lateral_pwm: 700`，旋转保留600。
`lateral_start_pwm: 800` 配合 `lateral_start_seconds: 0.18` 只在一次纯横移手势
起步时提供助力，持续按住不会重复触发；松手、反向、失联或禁行会重置。
超过600的配置仅允许在0.10固件启用；旧固件仍按600硬上限处理，
未配置轴向输出时沿用原有基准。更高输出来自用户手动按键，启动、解锁和通信恢复不自动运动。
落地横移起动能力必须由用户观察确认，不能把悬空轮向通过当作负载验收。

## 回滚

停止ROS代理并确认悬空、空载后，使用本地esptool将已校验的
`original_full_flash.bin`从偏移0完整写回。不要擦除或写入eFuse。
可执行 `bash tools/rollback_controller.sh --wheels-raised --unloaded`，
脚本会先核验双份备份与哈希，并拒绝占用中的串口。
回滚后恢复原厂接口和原厂尚未修复的命令静默持续运动问题；
不能把回滚等同于失联停车已通过。
