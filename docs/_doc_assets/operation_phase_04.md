# phase

## R12. 单独验收 DaBai DC1

### R12.1 识别设备

终端 1：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
ros2 run orbbec_camera list_devices_node
```

应看到 `DaBai DC1`、USB 端口和序列号。如果无输出：

```bash
lsusb | grep -i -E '2bc5|orbbec'
ros2 pkg prefix orbbec_camera
find "$(ros2 pkg prefix orbbec_camera)/share/orbbec_camera/launch" \
  -maxdepth 1 -iname 'dabai*.launch.py' -print
```

`ros2 pkg prefix orbbec_camera` 必须指向项目 `orbbec_ws/install`，而不是不含
legacy launch 的 `/opt/ros/humble` 包。

### R12.2 启动相机

终端 1：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
ros2 launch orbbec_camera dabai.launch.py \
  depth_registration:=true \
  enable_ir:=false \
  enable_point_cloud:=true
```

不要使用 DaBai DC1 不支持的同步配置；出现
`Not syncConfigurator found` 时，停止 launch，恢复项目验证过的
`dabai.launch.py` 参数。

### R12.3 检查图像、内参和点云

终端 2：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh

ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
```

按 `Ctrl+C` 停止频率检查，然后：

```bash
ros2 topic echo /camera/color/image_raw --once --field width
ros2 topic echo /camera/color/image_raw --once --field height
ros2 topic echo /camera/color/image_raw --once --field encoding

ros2 topic echo /camera/depth/image_raw --once --field width
ros2 topic echo /camera/depth/image_raw --once --field height
ros2 topic echo /camera/depth/image_raw --once --field encoding

ros2 topic echo /camera/depth/camera_info --once --field k
ros2 topic hz /camera/depth/points
```

验收条件：

- 彩色图连续发布，编码通常为 `rgb8`；
- 深度图连续发布，编码为 `16UC1`；
- 深度内参 `k` 不是 `[0,0,0,0,0,0,0,0,1]`；
- `/camera/depth/points` 有频率，不持续报 `No valid point in point cloud`；
- 深度 header 的 frame 为当前注册到彩色光学坐标系的 frame。

本项目历史验证过 `640×480` 深度及有效内参。彩色分辨率可以更高，但彩色和
深度 profile 必须是驱动实际支持、且能保持注册和内参有效的组合。

### R12.4 RViz 点云

```bash
rviz2
```

在 RViz：

1. `Global Options → Fixed Frame` 设为
   `camera_color_optical_frame`；
2. Add `PointCloud2`；
3. Topic 选 `/camera/depth/points`；
4. 若画面太近或太小，用滚轮缩放并按 F 聚焦。

保存配置：

```text
File → Save Config As
```

相机验收完后在终端 1 按 `Ctrl+C`，确认进程正常退出。


## R13. 配置并验收 Piper CAN

### R13.1 USB-CAN 检查

机械臂上电、急停释放后：

```bash
cd ~/robot_projects/foam_grasp_project
lsusb
ip -brief link
```

配置 `can0`：

```bash
./scripts/setup_can.sh
```

脚本等效于：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

检查：

```bash
ip -details -statistics link show can0
sudo ethtool -i can0
timeout 8 candump -n 20 can0
```

通过条件：

```text
UP,LOWER_UP
can state ERROR-ACTIVE
bitrate 1000000
RX持续增加
candump能看到2A1、2A2、2A3等Piper帧
```

如果 `can0` 为 UP 但 RX 为 0，优先排查机械臂电源、急停、CAN 线、插错 USB-CAN
和接头，而不是修改 ROS。

### R13.2 单独启动 Piper 驱动

终端 1：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=false \
  gripper_exist:=true \
  gripper_val_mutiple:=2 \
  log_level:=warn
```

终端 2：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh

ros2 node list | grep piper
ros2 topic echo /joint_states_single --once
ros2 topic echo /end_pose --once
ros2 topic echo /arm_status --once
```

`err_code` 应为 0，各关节通信错误标志应为 false。检查命令通道：

```bash
ros2 topic info /joint_states --verbose
ros2 topic info /joint_states_single --verbose
```

正常只读状态下：

```text
/joint_states Publisher count: 0
/joint_states_single Publisher count: 1（piper_ctrl_single_node）
```

测试完成后在终端 1 按 `Ctrl+C`。不要同时留下另一个 Piper 驱动。

