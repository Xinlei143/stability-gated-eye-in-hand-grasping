# 故障排查

## 项目位于Downloads

脚本技术上支持可移动路径，但正式运行建议使用：

```text
~/robot_projects/foam_grasp_project
```

不要source旧、新两份工程的overlay。打开新终端，只执行当前工程的
`source scripts/source_env.sh`。

## `can0` 为DOWN或没有报文

```bash
./scripts/setup_can.sh
ip -details -statistics link show can0
timeout 8 candump -n 20 can0
```

若RX仍为0，检查机械臂供电、USB-CAN、CAN-H/CAN-L和线缆，不要只修改软件。

## 找不到 `dabai.launch.py`

```bash
source scripts/source_env.sh
ros2 pkg prefix orbbec_camera
```

输出必须位于当前项目的 `workspaces/orbbec_ws/install`，而不是 `/opt/ros/humble`。

## 没有Piper节点或 `/end_pose`

```bash
ros2 node list | grep piper_ctrl
ros2 topic info /end_pose --verbose
```

同时检查CAN报文和Piper SDK导入：

```bash
PYTHONNOUSERSITE=1 venvs/runtime/bin/python -c \
  'import can, piper_sdk; print(can.__version__, piper_sdk.__file__)'
```

## PyTorch检测不到CUDA

```bash
nvidia-smi
source scripts/source_env.sh
python - <<'PY'
import torch
print(torch.__version__)
print(torch.cuda.is_available())
print(torch.cuda.device_count())
PY
```

`nvidia-smi`正常但PyTorch仍为False时，先重启电脑，再检查NVIDIA驱动与cu121
wheel兼容性。在本项目中不自动退回CPU运行实时分割。

### `libnvJitLink.so.12` 缺失

这通常表示venv中的 `nvidia-nvjitlink-cu12` 元数据存在，但动态库丢失或venv在
复制时损坏。不要只修改 `LD_LIBRARY_PATH`，直接运行：

```bash
./scripts/install_python_runtime.sh
```

安装脚本会强制重装 `nvidia-nvjitlink-cu12==12.1.105`，再导出项目内的动态库
目录。

### Piper退出：`No module named can`

Piper ROS入口可能使用 `/usr/bin/python3` shebang，而不是venv的Python。最终版通过
`source_env.sh` 将项目venv site-packages显式加入 `PYTHONPATH`。修复后检查：

```bash
source scripts/source_env.sh
PYTHONNOUSERSITE=1 /usr/bin/python3 - <<'PY'
import can
import piper_sdk
print(can.__version__)
print(piper_sdk.__file__)
PY
```

如果仍失败，重新执行 `scripts/install_python_runtime.sh`。

## RGB有图像，深度内参为0或点云无效

回到已验证的DaBai DC1 legacy分辨率组合。检查：

```bash
ros2 topic echo /camera/depth/image_raw --once --field width
ros2 topic echo /camera/depth/image_raw --once --field height
ros2 topic echo /camera/depth/camera_info --once --field k
ros2 topic hz /camera/depth/points
```

当前已验证的深度输出为640×480、`16UC1`，有效内参约为 `fx=fy=489.48`。

## 有mask，没有 `*_point_base`

按数据链逐项检查：

```bash
timeout 5 ros2 topic echo /foam_segmentation/mask --once --field header
timeout 5 ros2 topic echo /foam_grasp/cube_point --once
timeout 5 ros2 topic echo /end_pose --once
timeout 5 ros2 topic echo /foam_grasp/cube_point_base --once
```

没有 `/end_pose` 会阻断相机点到 `base_link` 的转换。

## IK失败

距离可达不等于指定末端姿态可达。当前程序会搜索多高度、多偏航和径向倾斜
候选，每个候选必须通过IK、碰撞、CURRENT→PREGRASP和100%笛卡尔下降/抬升。
若全部失败，优先调整物体位置或重新检查TCP/手眼标定，不要直接删除安全检查。

## 抓取位置系统性偏移

如果目标坐标随机械臂移动而明显漂移，检查手眼标定和相机支架。如果漂移很小但
夹取中心存在稳定平面误差，先在RViz和plan-only中验证 `grasp_offset_x/y`，再写入
`runtime.yaml`。

## 回报Issue时应附带什么

请附上：

```bash
./scripts/project_status.sh
./scripts/check_system.sh
git rev-parse HEAD
nvidia-smi
```

还要附上失败阶段完整日志，但不要上传机器人私有标定、GitHub Token或包含人脸/
敏感环境的图像数据。
