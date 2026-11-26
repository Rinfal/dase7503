# DASE7503 智能物流小车

实际扫码截图与码内容：[2026-11-26 实地运行及历史记录](docs/qr-scans/README.md)，已分文件夹归档。

手机热点遥控、流式视频和自动二维码留证已部署：连接 `Pi5-Control` 后打开 `http://10.42.50.1:8080/`。驾驶与升降操作、后台服务及验证范围见 [手机遥控说明](docs/phone-control.md)。

最新进度与验证范围见 [交接记录](docs/RESUME.md)，笔记本网线 SSH 开发见 [连接与配置恢复说明](docs/laptop-development.md)。当前无口令配置快照为 `robot_ws/src/logistics_robot/config/phone_profile_20261124.yaml`。

依据目录内四份课程 PDF 编写的新项目。上一届 ZIP 仅用于了解设备接口和方案；没有复制其学生编写的控制、视觉或导航程序。摄像头底层采用原作者的开源 libcamera / camera_ros，版本见 `tools/sources.lock.json`。

## 当前交付范围

用户已选择同一 Wi-Fi 下的手机网页遥控方案：实时相机画面、底盘移动与旋转、S2 升降、二维码截图留证与停车保护。自主导航与自动搬运不作为当前交付目标，雷达暂不接入。已有的自主任务及雷达代码保留为可选扩展，不会由遥控启动入口运行。

## 当前进度

- 原厂Flash已完整双读备份，原创ESP32-S3固件已刷入，保留原厂配置分区。
- 四轮位置、逐轮符号和整车前进/倒退/左右旋转已做15秒悬空观察并验收；250ms命令静默停车与重新解锁已实测。用户已确认下地前后和转向正常；地面速度未标定，实际搬运未验收。
- 当前固件为dase-rc-0.10，PWM硬上限800；暂停代理时独立电机任务约0.252秒归零，通信恢复后未重新解锁的运动被拒绝。物理USB断开和代理重启仍待验收。
- S2为舵机驱动齿轮齿条机构，用户已确认400/540µs手机升降正常；150µs重复短探测及300/325/340µs下降无动作，当前为350µs下降、1300µs升起（新高位待用户观察）；350µs会从手压的理想低位主动抬回偏高位置，机械低位需求尚未解决。主硬件配置保持默认禁用，本机部署配置记录用户授权。
- 手机热点访问、前进/后退/左右旋及原升降已由用户确认。已取消1秒试车限制；当前采用手动上锁/解锁，动作结束或超时只停车。新增左右平移并保留独立转向，0.10增力后用户已确认基本落地横移可用；场测后调为前后750、平移700/起步800持续180ms、旋转600，新比例待现场观察。整套 `dase7503.target` 开机自启已完成实际重启验收、数据停滞自恢复，启动时锁定；启动只读验收见 `runtime/boot-check.json`。
- IMX708相机原始输入960×540，手机JPEG预览640×360、上限8fps，实测约7fps；扫码与编码在独立视觉进程运行。
- 网页实时画面与方向、锁定/急停及升降按钮在同一驾驶区，二维码记录位于下方。
- 二维码扫描连续三帧确认，支持课程码与普通文本/网址，自动标框、保存截图和内容并跨重启去重；实际实体码与搬运待验收。
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
