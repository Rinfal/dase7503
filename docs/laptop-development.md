# 笔记本通过网线 SSH 开发

当前源码保存在 GitHub `Rinfal/dase7503`，树莓派工作目录为 `/home/rsi-bd-04/Documents/ChatGPT/dase7503`。后续让笔记本上的开发工具通过 SSH 在树莓派目录里工作，可继续使用现有 ROS、相机、控制板和自启服务。

## 连接前的实际状态

2026-11-24检查：树莓派SSH服务已启动，监听IPv4/IPv6的22端口；用户为 `rsi-bd-04`。`eth0` 无载波、没有IPv4地址，目前没有接通笔记本有线链路。此次没有改动网络配置或安装新SSH密钥。

网线接通后，若笔记本与树莓派连接到同一个提供DHCP的路由器/交换机网络，可在树莓派执行 `ip -4 -br addr show eth0` 查看有线地址，然后在笔记本运行：

```bash
ssh rsi-bd-04@<树莓派有线IPv4地址>
cd /home/rsi-bd-04/Documents/ChatGPT/dase7503
git status --short
source tools/environment.bash
```

如果是笔记本与树莓派直接用一根网线相连，不一定有DHCP服务器，需另行配置该有线接口的地址或笔记本网络共享，再验证SSH；现在没有预设或声称某个静态地址可用。后续只配置有线接口，保留USB联网、Pi5-Control热点、默认路由和代理。现有开发工具能否直接使用远程工作区取决于笔记本所用工具；已确认的入口是SSH终端。

## 当前配置与继续开发

- 当前固件0.10；前后PWM750、旋转600、横移700，起步800持续180ms。升降350/1300µs。新速度与新高位待用户现场观察。
- 视频与操纵合成一个驾驶区，二维码记录在下方；五项用户服务随开机启动。
- 本机权威配置：`~/.config/dase7503/phone-profile.yaml`。
- GitHub无口令快照：`robot_ws/src/logistics_robot/config/phone_profile_20261124.yaml`。
- 网页口令在 `~/.config/dase7503/remote-password`，不上传。热点设置由本机NetworkManager保存；完整Flash/NVS备份在本机runtime中，不上传。

在当前树莓派继续开发不需要复制快照或重新安装服务。修改静态网页后直接刷新；修改Python网页控制逻辑后先测试，再在停车状态重启 `dase7503-remote`，服务会从锁定状态启动。不要为了网页修改而重启整个target、串口代理或NetworkManager。

```bash
source tools/environment.bash
python3 -m unittest discover -s tests -v
systemctl --user status dase7503-remote
# 仅在部署Python网页控制改动、且小车已停车后使用：
systemctl --user restart dase7503-remote
```

需要在同一台硬件上恢复本次配置时，先备份本机配置，再安装已授权的快照；不要在其他机械结构或旧固件上直接套用：

```bash
umask 077
cp ~/.config/dase7503/phone-profile.yaml ~/.config/dase7503/phone-profile.before-restore.yaml
install -m 600 robot_ws/src/logistics_robot/config/phone_profile_20261124.yaml ~/.config/dase7503/phone-profile.yaml
```

这不会刷写固件；需要停车后重启网页服务才加载。快照不包含访问口令，不能独立恢复网络或认证。重新生成现有服务使用 `python3 tools/install_phone_remote.py --services-only --enable`，保留本机目标与速度。首次迁移还需要原有ROS环境、控制板0.10与本机认证配置，避免直接重刷或清空NVS。
