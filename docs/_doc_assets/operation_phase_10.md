# phase

## R30. 项目完成后的可选归档与迁移

本章不属于学生开始实验前的准备。只有项目已经由学生实现、训练、标定并在真机
验收通过后，才可以制作部署包。部署包是学生自己工程的可执行快照，通常包含：

```text
学生自己的 ROS 2/Python 源码与脚本
固定提交的第三方源码或 .repos 清单
学生训练得到的 best_model.pth
当前机械臂安装对应的手眼标定 JSON
参数、launch、文档与环境清单
```

通常不包含 `build/`、`install/`、`log/`、虚拟环境和大数据集。它不是 deb/apt
依赖包，也不能代替“从零实现”的教学过程。

### R30.1 源电脑

```bash
cd ~/robot_projects/foam_grasp_project
./scripts/capture_environment.sh
./scripts/project_status.sh

mkdir -p ~/foam_grasp_transfer
./scripts/package_for_transfer.sh \
  ~/foam_grasp_transfer/foam_grasp_project_deploy.tar.gz

sha256sum -c \
  ~/foam_grasp_transfer/foam_grasp_project_deploy.tar.gz.sha256
```

数据集单独打包：

```bash
./scripts/package_data.sh \
  ~/foam_grasp_transfer/foam_grasp_data.tar.gz
```

### R30.2 目标电脑

```bash
mkdir -p ~/robot_projects
sha256sum -c \
  ~/Downloads/foam_grasp_project_deploy.tar.gz.sha256
tar -xzf \
  ~/Downloads/foam_grasp_project_deploy.tar.gz \
  -C ~/robot_projects

cd ~/robot_projects/foam_grasp_project
chmod +x scripts/*.sh 启动.sh
./scripts/bootstrap_new_machine.sh --install-system
```

然后：

```bash
./scripts/setup_can.sh
timeout 5 candump -n 20 can0
./scripts/start_system.sh
```

新终端：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
./scripts/check_system.sh
```

若只是电脑变化而相机仍刚性安装在同一机械臂同一位置，可以迁移原标定；如果
硬件安装变化，必须重新做 R19。


## R31. GitHub 发布和依赖边界

公开仓库只提交：

- 自研 ROS 包；
- scripts；
- training；
- docs；
- `requirements/runtime.txt`；
- 精确 `.repos` 清单；
- 配置模板。

不提交：

- 私有标定；
- 未获许可的模型；
- 数据集；
- `venvs/`；
- `build/`、`install/`、`log/`；
- 第三方源码副本；
- token、SSH key；
- 机器特定绝对路径。

检查：

```bash
cd ~/robot_projects/foam_grasp_project
./scripts/check_github_ready.sh
git status --ignored
```

导出源码发布包：

```bash
./scripts/export_source_release.sh
```

GitHub README 必须明确：公开源码不能在缺少模型、标定和精确第三方源码时直接
运行。


## R32. 典型故障的最短诊断命令

### R32.1 ROS 未加载

```bash
source /opt/ros/humble/setup.bash
ros2 --help
printenv | grep ROS_DISTRO
```

### R32.2 DaBai launch 找不到

```bash
source ~/robot_projects/foam_grasp_project/scripts/source_env.sh
ros2 pkg prefix orbbec_camera
find "$(ros2 pkg prefix orbbec_camera)/share/orbbec_camera/launch" \
  -maxdepth 1 -iname 'dabai*.launch.py' -print
```

### R32.3 相机无深度/内参为零

```bash
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/camera_info --once --field k
ros2 topic hz /camera/depth/points
```

恢复项目验证过的 `dabai.launch.py` 配置，不要继续组合不兼容的分辨率。

### R32.4 CAN UP 但无报文

```bash
ip -details -statistics link show can0
timeout 8 candump -n 20 can0
lsusb
sudo ethtool -i can0
```

RX 为 0 时检查物理线缆和电源。

### R32.5 Piper 节点没有启动

```bash
source scripts/source_env.sh
ros2 node list | grep piper_ctrl
pgrep -af 'piper_single_ctrl|piper_ctrl_single'
PYTHONNOUSERSITE=1 /usr/bin/python3 - <<'PY'
import can
import piper_sdk
print(can.__version__)
print(piper_sdk.__file__)
PY
```

若 system Python 无法导入，重新运行：

```bash
./scripts/install_python_runtime.sh
```

### R32.6 CUDA 异常

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

先重启，再检查 venv；不要在同一进程启动后修改 `CUDA_VISIBLE_DEVICES`。

### R32.7 所有 IK 失败

```bash
ros2 topic echo /foam_grasp/target_point_base_latched --once
ros2 topic echo /foam_grasp/cube_pregrasp_pose --once
ros2 run foam_grasp object_grasp_sequence --target-class cube
```

检查：

- 目标是否接近 X/Y 工作区边界；
- base 点是否因标定漂移；
- TCP/tool offset 是否与 URDF/算法一致；
- 目标补偿是否重复应用；
- 当前真机反馈是否被 MoveIt 正确订阅；
- 近似垂直多姿态搜索是否有可行解。

“距离够得到”不保证固定末端姿态下有无碰撞 IK。

### R32.8 PREGRASP 位置误差约 47 mm

```bash
ros2 topic echo /end_pose --once
ros2 topic echo /foam_grasp/cube_pregrasp_pose --once
```

这是典型的 `link6` 原点与 `gripper_tcp` 接触中心混用。所有目标生成、IK、轨迹
和到位验证必须统一使用同一参考点；不要简单把 47 mm 误差阈值调大。

### R32.9 跟踪误差过大

```bash
ros2 topic echo /joint_states_single --once
ros2 topic echo /arm_status --once
```

检查驱动速度上限、机械臂实际跟踪、旧命令发布者、线缆阻碍和关节反馈时间。
当前硬保护为 0.20 rad，到位误差为 0.05 rad。阈值是阻止继续向错误姿态运动的
最后防线，不应直接删除。

### R32.10 夹爪抓空或压物体

```bash
ros2 topic echo /joint_states_single --once
```

检查第 7 个 `gripper` 反馈、目标中心偏差、预张开、物体尺寸和标定。预张开必须
在安全高位完成；不要在下降到目标附近才边移动边张开。


## R33. 最终验收表

只有以下全部通过，才称为“从零复现完成”：

1. Ubuntu 22.04 x86_64；
2. ROS 2 Humble 正常；
3. 项目在固定工程目录；
4. 第三方源码提交可追踪；
5. `list_devices_node` 识别 DaBai DC1；
6. 彩色、深度、有效内参、点云持续发布；
7. `candump` 持续收到 Piper 帧；
8. `/joint_states_single`、`/end_pose`、`/arm_status` 正常；
9. PyTorch CUDA 可用；
10. 在线 overlay 对三类目标正确；
11. 相机点和 base 点稳定；
12. 手眼标定通过跨姿态验证；
13. `/joint_states` 在执行前无发布者；
14. `allow_trajectory_execution=False`；
15. 观察姿态 plan-only 和真机到位通过；
16. 夹爪专项低风险测试通过；
17. 目标锁定 spread 合格；
18. IK、完整规划、两段笛卡尔路径全部通过；
19. cube 真实抓取通过；
20. cylinder 和 sphere 真实抓取通过；
21. 项目能在另一台干净 Ubuntu 电脑重建。


## R34. 官方参考

- [ROS 2 Humble Ubuntu 安装](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [ROS 2 Humble 支持平台](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)
- [OrbbecSDK ROS 2](https://github.com/orbbec/OrbbecSDK_ROS2)
- [Piper SDK](https://github.com/agilexrobotics/piper_sdk)
- [Piper ROS](https://github.com/agilexrobotics/piper_ros)
- [MoveIt 2 Humble](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)
- [OpenCV 手眼标定](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [Torchvision DeepLabV3](https://docs.pytorch.org/vision/main/models/deeplabv3.html)

复现时以本项目固定版本为第一优先级，再用官方文档核对安装方式。不要在同一个
已验证环境中无计划升级 ROS、MoveIt、Orbbec、Piper SDK 或 PyTorch。
