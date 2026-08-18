# 快速开始

本文档面向已完成硬件组装的Ubuntu 22.04电脑。系统由DaBai DC1、Piper
机械臂、夹爪、NVIDIA GPU以及ROS 2 Humble组成。

> **安全警告**：这是会真实驱动机械臂的研究项目。任何自动运动都必须在急停
> 可立即触及、工作空间无人、无线缆干涉的条件下进行。首次部署必须先运行
> plan-only和夹爪空载测试。

## 1. 仓库与运行资源

普通Git历史不直接包含以下大文件：

- `runtime/models/best_model.pth`：约482 MB的语义分割模型；
- `runtime/calibration/handeye_eye_in_hand.json`：与实际相机安装关系绑定的标定；
- Orbbec、Piper和可选MoveIt第三方源码；
- 数据集、rosbag、venv以及 `build/install/log`。

正式发布版本通过 GitHub Release 提供模型和当前物理 rig 的标定，并由
`install.sh` 自动下载校验。不要把该标定用于另一套机械臂/相机安装。

## 2. 克隆到独立目录

```bash
mkdir -p ~/robot_projects
cd ~/robot_projects
git clone <YOUR_GITHUB_REPOSITORY_URL> foam_grasp_project
cd foam_grasp_project
chmod +x scripts/*.sh 启动.sh
```

正式 `v1.0.0` 可直接执行：

```bash
chmod +x install.sh scripts/*.sh 启动.sh
./install.sh
```

这会完成后续第三方源码、运行资源、系统/Python依赖、构建和udev步骤。

`Downloads`只用于接收压缩包，不作为正式运行目录。

## 3. 准备第三方源码

仓库应包含已工作电脑导出的：

```text
dependencies/orbbec.repos
dependencies/piper.repos
dependencies/moveit.repos    # 仅使用源码MoveIt overlay时需要
```

导入精确版本：

```bash
./scripts/import_vendor.sh
```

DaBai DC1必须使用本项目已验证的legacy OrbbecSDK_ROS2驱动，不要在部署
时自动切换到其他主分支。

## 4. 放入模型与标定

```bash
install -m 0644 /path/to/best_model.pth \
  runtime/models/best_model.pth

install -m 0644 /path/to/handeye_eye_in_hand.json \
  runtime/calibration/handeye_eye_in_hand.json
```

更换相机、支架、末端法兰关系或机械臂后，必须重新标定。

## 5. 安装和构建

新电脑已配置ROS 2 apt软件源且可以联网时：

```bash
./scripts/bootstrap_new_machine.sh --install-system
```

或者手动执行：

```bash
./scripts/install_system_dependencies.sh
./scripts/install_python_runtime.sh
./scripts/build_all.sh
./scripts/validate_project.sh
./scripts/project_status.sh
```

`project_status.sh`应全部显示 `[OK]`。

其中必须包含：

```text
[OK] Runtime imports          torch, python-can, piper_sdk
```

再做一次明确版本检查：

```bash
source scripts/source_env.sh
python - <<'PY'
import can
import piper_sdk
import torch

print("PyTorch:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
print("python-can:", can.__version__)
print("piper_sdk:", piper_sdk.__file__)
if torch.cuda.is_available():
    print("GPU:", torch.cuda.get_device_name(0))
PY
```

必须显示 `CUDA available: True` 和 `python-can: 4.6.1`。

## 6. 安装Orbbec udev规则

```bash
cd workspaces/orbbec_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

拔插相机，然后返回项目根目录。

## 7. 每次开机启动

确保Piper已上电、USB-CAN线正确、相机已连接、夹爪内无物体。

终端一：

```bash
cd ~/robot_projects/foam_grasp_project
./scripts/setup_can.sh
ip -brief link show can0
./scripts/start_system.sh
```

保持终端一运行。

终端二：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
./scripts/check_system.sh
```

使用交互菜单选择目标：

```bash
./启动.sh
```

或直接指定：

```bash
./启动.sh cube
./启动.sh cylinder
./启动.sh sphere
```

自动流程会移动到观察姿态、锁定目标、搜索IK/路径、张开夹爪、接近、闭合、
验证夹持并抬升。操作者全程看守，不需要输入 `DESCEND/CLOSE/LIFT`。

## 8. 停止

自动运动结束后，在终端一按 `Ctrl+C`关闭系统。不要在机械臂正在运动时
关闭Piper驱动；出现碰撞风险时使用硬件急停。

## 9. 更多文档

- [系统架构](ARCHITECTURE.md)
- [详细迁移](DEPLOYMENT.md)
- [日常运行](OPERATIONS.md)
- [数据与训练](DATA_AND_TRAINING.md)
- [故障排查](TROUBLESHOOTING.md)
