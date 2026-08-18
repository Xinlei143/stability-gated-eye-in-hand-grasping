# phase

## R14. 首次完整静态验收

确认模型、标定和构建结果：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh

ls -lh \
  runtime/models/best_model.pth \
  runtime/calibration/handeye_eye_in_hand.json

ros2 pkg prefix orbbec_camera
ros2 pkg prefix piper
ros2 pkg prefix foam_grasp

python -c \
  'import rclpy, cv_bridge, torch, cv2, can, piper_sdk; print("runtime imports OK")'
```

检查配置：

```bash
sed -n '1,200p' config/project.env
sed -n '1,220p' \
  workspaces/app_ws/src/foam_grasp/config/runtime.yaml
```

不要在复现初期修改速度、工作区、TCP 或抓取偏移。先用已验证默认值完成
plan-only 和近距离目标测试。


## R15. 启动完整系统

### R15.1 先确认没有残留进程

```bash
pgrep -af \
  'piper_single_ctrl|piper_ctrl_single|move_group|orbbec|foam_' \
  || echo "没有残留核心进程"
```

若上一次系统没有正常退出，先回到原终端按 `Ctrl+C`。只有确定是残留进程时，
才使用：

```bash
pkill -INT -f 'ros2 launch.*foam_grasp'
sleep 2
ros2 daemon stop
ros2 daemon start
```

### R15.2 终端 1：CAN 和系统

```bash
cd ~/robot_projects/foam_grasp_project
./scripts/setup_can.sh
timeout 5 candump -n 20 can0
./scripts/start_system.sh
```

启动脚本会加载完整 overlay，并启动：

- DaBai DC1；
- Piper 驱动；
- plan-only MoveIt；
- DeepLabV3 分割；
- 深度融合；
- camera→base 坐标转换；
- 多帧目标锁定；
- 抓取位姿预览；
- 可选 RViz。

### R15.3 终端 2：只读检查

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
./scripts/check_system.sh
```

进一步检查：

```bash
ros2 node list | sort -u
ros2 topic info /joint_states --verbose
ros2 topic info /joint_states_single --verbose
ros2 param get /move_group allow_trajectory_execution
```

放行条件：

```text
/piper_ctrl_single_node 存在
/move_group 存在
/joint_states Publisher count = 0
/joint_states_single 有 Piper 发布者
allow_trajectory_execution = False
```

MoveIt 在本工程里只计算轨迹，不能自己执行；实际命令由本项目受限执行器临时
占用 `/joint_states`。


## R16. 验收在线语义分割

检查输入：

```bash
ros2 topic hz /camera/color/image_raw
```

检查输出：

```bash
ros2 topic list | grep foam_segmentation
ros2 topic hz /foam_segmentation/mask
ros2 topic echo /foam_segmentation/mask --once --field header
```

使用图像查看：

```bash
ros2 run image_view image_view \
  --ros-args -r image:=/foam_segmentation/overlay
```

把正方体、圆柱体和球体依次放入画面。要求：

- 目标区域类别正确；
- mask 轮廓基本贴合；
- 背景没有大面积误识别；
- 移动物体后输出能更新；
- GPU 没有 CUDA 初始化错误。

可查看节点延迟：

```bash
ros2 topic echo /foam_segmentation/latency_ms
```

如果模型在训练集很好但现场误识别，不要继续抓取；补采当前背景和光照数据后
重新训练。


## R17. 验收 RGB-D 融合与 base 坐标

依次检查相机坐标点：

```bash
ros2 topic echo /foam_grasp/cube_point --once
ros2 topic echo /foam_grasp/cylinder_point --once
ros2 topic echo /foam_grasp/sphere_point --once
```

只放当前目标时，其他类别可能无输出，这是正常的。检查 base 坐标：

```bash
ros2 topic echo /foam_grasp/cube_point_base --once
```

目标点应在配置工作区内：

```text
X: 0.15–0.60 m
|Y|: ≤0.35 m
Z: -0.02–0.20 m
```

进行固定目标一致性检查：

1. 目标固定在桌面不动；
2. 记录观察姿态 1 的 base 点；
3. 缓慢改变机械臂观察姿态；
4. 再记录 base 点；
5. 比较差值。

```bash
ros2 topic echo /foam_grasp/cube_point_base --once
sleep 2
ros2 topic echo /foam_grasp/cube_point_base --once
```

同一姿态下多帧应稳定在毫米级；换观察姿态后明显漂移超过约 10 mm，应优先
检查手眼标定方向、单位、时间同步、相机支架或 `/end_pose` 定义，不要直接用
抓取偏移补偿。


## R18. 验收观察姿态和夹爪

### R18.1 观察姿态只规划

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
ros2 run foam_grasp move_to_observe
```

应输出轨迹点数、时长、最大步长和目标关节，不移动真机。

### R18.2 移动到观察姿态

确认急停可触及、工作区无人：

```bash
ros2 run foam_grasp move_to_observe \
  --execute \
  --confirm AUTO_MOVE_TO_OBSERVE \
  --countdown-seconds 5
```

到位后：

```bash
ros2 topic echo /joint_states_single --once --field position
ros2 topic echo /end_pose --once
ros2 topic echo /arm_status --once
```

默认观察关节为：

```text
0.142604700
0.269998232
-0.653208024
0.008059128
1.078754404
0.054111288
```

单位为弧度。只应在确认机械臂型号、URDF 和固件定义一致后使用。

### R18.3 夹爪只读预检

保持末端在安全高位：

```bash
ros2 run foam_grasp piper_gripper_safe_test \
  --actual-opening-mm 20
```

### R18.4 夹爪真实低风险测试

夹爪内无物体或手指：

```bash
ros2 run foam_grasp piper_gripper_safe_test \
  --actual-opening-mm 20 \
  --execute \
  --confirm GRIPPER_ONLY

ros2 run foam_grasp piper_gripper_safe_test \
  --actual-opening-mm 60 \
  --execute \
  --confirm GRIPPER_ONLY
```

若驱动尚未 enable，可仅在理解“该驱动 enable 时可能先发送 0 mm 夹爪命令”
的前提下增加：

```text
--enable-arm
```

夹爪反馈应接近命令值。Piper 驱动配置
`gripper_val_mutiple:=2`，脚本已完成实际总开口与驱动虚拟位置之间的换算，
不要再手动乘 2。

