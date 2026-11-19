# DASE7503 智能物流小车

关机前最新进度、安全问题及继续开发步骤：见 `docs/RESUME.md`。真实测试记录优先于下方早期开发进度列表。

依据目录内四份课程 PDF 编写的新项目。上一届 ZIP 仅用于了解设备接口和方案；没有复制其学生编写的控制、视觉或导航程序。摄像头底层采用原作者的开源 libcamera / camera_ros，版本见 `tools/sources.lock.json`。

## 当前交付范围

用户已选择同一 Wi-Fi 下的手机网页遥控方案：实时相机画面、底盘移动与旋转、S2 升降、二维码截图留证与停车保护。自主导航与自动搬运不作为当前交付目标，雷达暂不接入。已有的自主任务及雷达代码保留为可选扩展，不会由遥控启动入口运行。

## 当前进度

- 原厂Flash已完整双读备份，原创ESP32-S3固件已刷入，保留原厂配置分区。
- 四轮位置、逐轮符号和整车前进/倒退/左右旋转已做15秒悬空观察并验收；250ms命令静默停车与重新解锁已实测。尚无地面速度或搬运验证。
- 当前固件为dase-rc-0.9；暂停代理时独立电机任务约0.252秒归零，通信恢复后未重新解锁的运动被拒绝。物理USB断开和代理重启仍待验收。
- S2为舵机驱动齿轮齿条机构，正在空载标定；150µs探测用户确认无额外位移，400/540µs网页上下循环已有信号记录，实际动作仍待用户确认，默认升降许可保持关闭。
- 手机网页、口令登录、实时画面、截图、控制会话和停车逻辑已实现；真实HTTP链路的前进、后退、左旋、右旋及客户端静默停车测试通过。已有独立热点启动脚本，手机端连接仍待验收，默认运动许可保持关闭。
- IMX708相机已实际输出1280×720；当前为减轻负载，默认不启动相机。
- 二维码扫描和截图留存已实现，课程六个二维码样本离线解码通过；本组实体二维码与实际搬运尚未验收。
- ROS2 Jazzy、micro-ROS代理、相机驱动均已构建；当前软件测试及最新实车证据见 `docs/RESUME.md` 和 `docs/evidence/`。

## 运行与检查

系统安装由助手调用系统认证窗口执行，不需要把密码发到聊天：

```bash
sudo bash tools/install_system.sh
```

系统依赖安装完成后，以普通用户构建：

```bash
bash tools/build_workspaces.sh
source tools/environment.bash
```

离线测试不需要 ROS 或真实硬件：

```bash
python3 -m unittest discover -s tests -v
python3 tools/dry_run.py
python3 tools/doctor.py
```

只启动观察节点，不启动导航：

```bash
source tools/environment.bash
ros2 launch logistics_robot observe.launch.py
```

此时安全节点持续发布零速度，急停锁定且 `hardware_ready=false`。该启动方式不能替代控制板自身的失联停车保护。

相机已构建并验证，可按需启动：

```bash
source tools/environment.bash
bash tools/start_camera.sh
```

`camera/image_raw` 是二维码扫描输入，`qr/annotated` 是带识别框的图像。首次稳定识别到每个合法二维码时，保存 PNG 和 JSONL 到 `~/dase7503_evidence`。

使用 `bash tools/start_agent.sh` 接入已确认的 ESP32 串口；它不会发布运动或舵机命令。使用 `python3 tools/inspect_ros_graph.py --output docs/evidence/controller_graph.json` 保存实际节点、话题类型与发布/订阅端。接口清单不等同于运动方向或失联停车已经验证。

## 遥控与比赛

课程项目描述 PDF 第 4 页将自动完成列为 A range、手动完成列为 B range，因此允许手动完成比赛操作。但课程同时要求开发定位、导航和自主搬运；遥控应先作为联调与保底功能。

当前网页遥控采用新固件的 `/wheel_pwm`、`/drive_arm` 与 `/drive/status`，启动入口为 `tools/start_remote.sh`。旧键盘、自主导航和原厂Twist接口不适用于当前PWM固件。网页许可由硬件配置中的实测验收记录控制。

## 文件组织

- `robot_ws/src/logistics_robot/`：本项目新编写的 ROS 包与纯 Python 控制核心。
- `camera_ws/src/`：按固定 commit 下载的上游摄像头驱动；不修改系统 libcamera。
- `agent_ws/src/`：上游 micro-ROS Agent 与消息包，按固定 commit 准备。
- `tools/`：系统安装、源码准备、构建、设备检查、被动串口检查及任务模拟。
- `tests/`：二维码、安全、任务流程及雷达报文测试。
- `docs/requirements.md`：课程要求与验收表。
- `docs/hardware.md`：当前接线、接入次序与待测量参数。
- `docs/interfaces.md`：话题、状态机和联调约定。
- `docs/evidence/`：实际诊断记录及参考场地图。

`config/field.yaml` 中只有从课程场地图读取的边界与区域，实际货架任务列表为空，`survey_confirmed=false`。必须获取本组完整二维码和实测停靠位置后才能启用任务节点。

## 仓库备份范围

仓库保存本项目源码、依赖版本、测试、文档和实测记录。课程PDF、往届代码ZIP、厂商PDF、构建产物、下载的上游源码、访问口令及含私有配置的完整Flash备份不上传；原文件保留在本机。通过 `tools/prepare_sources.py` 可恢复ROS与相机上游源码，固件依赖见 `firmware/drive_controller/dependencies.lock.json`。
