# 日常运行流程

## 1. 上电前

- 急停按钮可立即触及。
- 机械臂全工作空间无人、无线缆或障碍物。
- 夹爪内没有物体或手指。
- 相机支架没有松动。
- 待抓取物体放在已标定桌面和安全工作区内；同一类别不要同时放置多个。

## 2. 启动CAN

每次电脑重启后执行：

```bash
cd ~/robot_projects/foam_grasp_project
./scripts/setup_can.sh
```

应显示类似：

```text
can0 UP <NOARP,UP,LOWER_UP,ECHO>
```

## 3. 启动完整系统

终端 1：

```bash
cd ~/robot_projects/foam_grasp_project
./scripts/start_system.sh
```

该脚本自动 source 所有 overlay，并传入模型与标定路径。不要再手动运行散落在 `/home/rl` 的旧脚本。

注意：当前配置 `AUTO_ENABLE=true`。Piper驱动启动时可能先给空夹爪发送0 mm命令，因此启动前夹爪必须为空。

## 4. 只读检查

终端 2：

```bash
cd ~/robot_projects/foam_grasp_project
./scripts/check_system.sh
```

也可以单独查看：

```bash
source scripts/source_env.sh
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /foam_segmentation/mask
ros2 topic echo /foam_grasp/cube_point_base --once
```

## 5. 选择目标并自动抓取

确认目标静止后：

```bash
./启动.sh
```

输入方式：

```text
1 / cube / 方块 / 正方体
2 / cylinder / 圆柱 / 圆柱体
3 / sphere / 球 / 球体
```

也可跳过交互菜单：

```bash
./启动.sh cube
./启动.sh cylinder
./启动.sh sphere
```

脚本自动执行：

1. 检查所有关键节点。
2. 确认 `/joint_states` 没有其他发布者。
3. 确认 MoveIt 禁止自行执行轨迹。
4. 清除旧锁定目标。
5. 多帧采样并只锁定所选类别。
6. 生成 PREGRASP、GRASP、LIFT。
7. 检查工作空间、关节状态、IK、碰撞和三段路径。
8. 在观察高位先把夹爪预张开到70 mm并检查反馈。
9. 保持最大开口移动到PREGRASP，避免边下降边张开压到目标。
10. 垂直下降55 mm。
11. 按类别闭合：正方体40 mm、圆柱体55 mm、球体45 mm；圆柱在下降前张开到90 mm。
12. 根据实际夹爪开口验证物体确实阻挡了闭合；未夹住时拒绝抬升。
13. 垂直抬升55 mm并保持。

过程中不需要键盘输入。操作者仍须全程观察并准备使用硬件急停。

旧命令 `./scripts/run_cube_auto.sh` 仍可用于直接抓取正方体。

当前本机夹爪已实测电动行程超过100 mm。圆柱流程会在观察高位先张开到
90 mm，然后保持开口移动并沿圆柱中心下降。首次启用前必须用空夹爪执行
90 mm夹爪安全测试，确认反馈后再运行完整流程。18 mm偏心弦夹取仍可用
作70 mm受限夹爪的备用参数。

## 6. 停止

- 自动抓取脚本正常结束后，机械臂保持LIFT姿态和夹爪目标。
- 关闭完整系统时，在终端1按 `Ctrl+C`。
- 不要在机械臂运动中随意关闭Piper驱动。
- 硬件异常、碰撞风险或路径明显错误时直接使用硬件急停。

## 7. 常见问题

### 找不到 `dabai.launch.py`

说明没有加载项目 Orbbec overlay：

```bash
source scripts/source_env.sh
ros2 pkg prefix orbbec_camera
```

输出必须位于 `foam_grasp_project/workspaces/orbbec_ws/install`。

### CUDA不可用

先检查：

```bash
nvidia-smi
source scripts/source_env.sh
python -c 'import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))'
```

如果 `nvidia-smi` 正常但 PyTorch为False，应先重启电脑，再检查驱动和运行环境，不要让分割节点自动退回CPU。

### `joint5`略高于1.220 rad

脚本只允许当前起点有0.020 rad容差，且硬件不得报告限位。规划轨迹必须立即单调回到严格安全范围；目标和后续轨迹不会放宽。

### 目标坐标随机械臂移动明显漂移

停止抓取，检查手眼标定、相机支架和 `/end_pose`。不要通过扩大工作空间阈值掩盖标定问题。
