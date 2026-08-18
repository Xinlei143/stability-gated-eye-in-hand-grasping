# Foam Grasp 从空白电脑到首次自动夹取：逐条命令手册

本部分是一条可以照着执行的复现主线。默认平台、硬件和工程位置如下：

```text
Ubuntu 22.04 x86_64
ROS 2 Humble
Orbbec DaBai DC1（legacy/OpenNI）
AgileX Piper + 平行夹爪
NVIDIA GPU
~/robot_projects/foam_grasp_project
```

命令块中的 `$`、终端提示符和示例输出不要复制，只复制命令本身。所有真实机械臂
动作都必须由一名操作者全程看守；工作空间必须无人，急停必须可立即触及。

> 本项目不是“安装完 ROS 就能抓取”。完整闭环还需要：准确的第三方源码版本、
> DaBai udev 规则、Piper CAN、CUDA/PyTorch、分割模型、当前相机安装对应的手眼
> 标定、MoveIt 配置和本项目安全执行器。任何一项缺失，都应停在对应验收点。

## R0. 课程默认模式：学生从零实现，不提供项目代码

本手册的教学主线不假设教师提前提供 `foam_grasp` 项目代码、训练权重、标定结果
或私有部署包。学生只使用公开软件、硬件说明和接口要求，逐步完成自己的工程。

| 内容 | 教师/实验室可以提供 | 学生必须完成 |
|---|---|---|
| 系统基线 | Ubuntu 22.04、ROS 2 Humble、硬件型号 | 安装、验证并记录版本 |
| 第三方驱动 | 官方仓库 URL、指定分支或提交号 | 下载、编译并验收 |
| 自研 ROS 2 包 | 话题/服务接口和验收要求 | 创建包并实现全部节点 |
| 语义分割 | 三类物体和类别定义 | 采集、标注、训练、测试与部署 |
| 手眼标定 | 标定板尺寸和安装方式 | 对自己的机械臂和相机重新标定 |
| 抓取流程 | 安全约束和目标行为 | 规划、状态机、夹爪控制与联调 |

私有部署包是“项目已经完成以后”的整机迁移快照，不是安装依赖包，也不是学生
开始实验前必须拿到的材料。它通常会包含作者自己的项目代码、固定版本第三方
源码、模型权重和标定文件，因此不适合用作要求学生独立实现的课程输入。只有在
项目验收结束、需要备份或迁移到另一台电脑时，才使用 R30 生成它。

### R0.1 教学复现的实际操作顺序

Word 手册会按照真实依赖关系重新排列本文件中的章节，学生不需要在文档中来回
跳转。完整操作顺序如下：

```text
安全与硬件准备
→ Ubuntu、ROS 2、NVIDIA 基础环境
→ 创建工程目录、ROS 2 包和全部自研源文件
→ 安装系统/Python依赖、导入固定第三方源码并首次构建
→ 单独验收 DaBai DC1、CAN 和 Piper
→ 采集、标注、转换并划分数据集
→ 训练、独立测试并部署 DeepLabV3-ResNet50
→ 完成当前相机安装对应的眼在手上标定
→ 放置模型与标定，执行完整静态验收
→ 启动在线感知，验收分割、深度融合和 base 坐标
→ 验收观察姿态、夹爪、目标锁定、IK 与 plan-only 规划
→ 依次完成方块、圆柱体和球体真实夹取
→ 固化日常启动、停止、排障、迁移和 GitHub 发布流程
```

学生的工程应从第一次提交开始纳入 Git。不要把别人的成品代码、模型或标定结果
改名后当作“从零复现”；也不要把 R30 的部署包当成课程起始包。

## R1. 硬件、接线与安全准备

### R1.1 硬件清单

- x86_64 电脑，建议 16 GB 内存、100 GB 可用空间；
- NVIDIA GPU；本项目已在 RTX A4000 上验证；
- Orbbec DaBai DC1、数据线和稳定 USB 接口；
- AgileX Piper、平行夹爪、电源、急停；
- Linux 支持的 USB-CAN 适配器；
- 固定相机到末端的刚性支架；
- 标定板；
- 50 mm 正方体、直径 70 mm × 高 70 mm 圆柱体、直径 60 mm 球体。

### R1.2 接线顺序

1. 机械臂先断电，安装并锁紧夹爪与相机支架。
2. 相机数据线留出运动余量，不得跨过关节夹点。
3. USB-CAN 接到 Piper CAN 口，再接电脑。
4. 相机接电脑；优先使用主板直连 USB 端口。
5. 检查急停功能和电源电压。
6. 清空机械臂整个运动包络。
7. 最后给 Piper 上电。

### R1.3 首次运行的硬性规则

- 不在虚拟机中做 USB 深度相机和真机控制复现；
- 不把人的手放在夹爪内测试开合；
- 不在 Piper 驱动运行时启动第二个相同驱动；
- 不启动 MoveIt demo/fake hardware 与真机驱动并存；
- 不从未知脚本向 `/joint_states`、`/pos_cmd` 发布命令；
- 任何 `--execute` 之前先完成本手册对应的只读验收。

## R2. 安装并检查 Ubuntu 22.04

使用 Ubuntu 22.04 LTS x86_64。安装时建议选择正常安装和第三方显卡驱动。
首次进入系统后执行：

```bash
cat /etc/os-release | grep -E 'PRETTY_NAME|VERSION_ID'
uname -m
df -h /home
free -h
lspci | grep -i -E 'nvidia|vga'
```

通过条件：

```text
VERSION_ID="22.04"
x86_64
```

更新系统。ROS Humble 官方说明特别提醒 Jammy 应先更新系统包：

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

重启后安装基础工具：

```bash
sudo apt update
sudo apt install -y \
  curl wget git ca-certificates gnupg lsb-release \
  software-properties-common locales
```

配置 UTF-8：

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

## R3. 安装 ROS 2 Humble

### R3.1 启用 Universe

```bash
sudo add-apt-repository universe
sudo apt update
```

### R3.2 配置 ROS 软件源

优先按照 ROS 2 Humble 官方 Ubuntu 安装页当日给出的方式配置软件源。当前可用
的 `ros-apt-source` 安装流程为：

```bash
sudo apt update
sudo apt install -y curl

export ROS_APT_SOURCE_VERSION=$(
  curl -s \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' \
    | awk -F'"' '{print $4}'
)

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

如果官方页面已更改命令，以官方页面为准；不要混入其他 Ubuntu 版本或其他 ROS
发行版的软件源。

### R3.3 安装桌面版和开发工具

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools
```

加载并验证：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
printenv | grep -E '^ROS_(VERSION|DISTRO)='
ros2 doctor --report
```

必须看到：

```text
ROS_VERSION=2
ROS_DISTRO=humble
```

可把 ROS 基础环境写入 `~/.bashrc`，但不要在这里写入旧项目 overlay：

```bash
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

新开终端验证：

```bash
ros2 --help
```

## R4. 安装 NVIDIA 驱动并验证 GPU

先查看推荐驱动：

```bash
ubuntu-drivers devices
```

安装 Ubuntu 推荐驱动：

```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

重启后：

```bash
nvidia-smi
```

必须能显示 GPU 名称和驱动版本。`nvidia-smi` 顶部显示的 CUDA Version 是驱动
支持的最高 CUDA API，不等同于 PyTorch 自带的 CUDA runtime；本项目使用
PyTorch `cu121`，驱动只要满足兼容性即可。

不要先单独安装完整 CUDA Toolkit。R9 会在学生虚拟环境中安装 PyTorch CUDA
wheel 和它需要的运行库。

## R5. 创建学生工程、工作空间和 ROS 2 包

正式工程不要放在 `~/Downloads`。从空目录建立可迁移的工程骨架：

```bash
mkdir -p ~/robot_projects/foam_grasp_project
cd ~/robot_projects/foam_grasp_project

mkdir -p \
  config data dependencies docs requirements scripts training \
  runtime/models runtime/calibration venvs \
  workspaces/orbbec_ws/src \
  workspaces/piper_ws/src \
  workspaces/moveit_ws/src \
  workspaces/app_ws/src

git init
touch README.md .gitignore
```

建议立即编写 `.gitignore`，至少忽略：

```gitignore
**/build/
**/install/
**/log/
venvs/
data/
runtime/models/*.pth
runtime/calibration/*.json
__pycache__/
*.pyc
```

### R5.1 创建自研 ament_python 包

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_projects/foam_grasp_project/workspaces/app_ws/src

ros2 pkg create foam_grasp \
  --build-type ament_python \
  --license Apache-2.0 \
  --dependencies \
    rclpy sensor_msgs geometry_msgs visualization_msgs \
    std_msgs std_srvs cv_bridge tf2_ros moveit_msgs

cd foam_grasp
mkdir -p launch config
```

检查：

```bash
find ~/robot_projects/foam_grasp_project/workspaces/app_ws/src/foam_grasp \
  -maxdepth 2 -type f | sort
```

### R5.2 学生需要实现的程序

在 `foam_grasp/foam_grasp/` 中逐个实现下列模块，并在 `setup.py` 的
`console_scripts` 中注册入口：

| 模块 | 最小职责 | 关键输出 |
|---|---|---|
| `foam_segmentation_node.py` | RGB 图像推理、发布类别 mask 和 overlay | `/foam_segmentation/mask` |
| `foam_depth_fusion_node.py` | mask 与注册深度融合、鲁棒估计三维中心 | `/foam_grasp/*_point` |
| `foam_camera_to_base_node.py` | 手眼变换和末端位姿组合 | `/foam_grasp/*_point_base` |
| `foam_target_latch_node.py` | 多帧稳定性检查与目标锁存 | `/foam_grasp/target_point_base_latched` |
| `foam_grasp_pose_preview_node.py` | 依据类别尺寸生成 pregrasp/grasp/lift | `PoseStamped` 三个位姿 |
| `foam_move_to_observe.py` | 安全规划并到达观察姿态 | 只在显式执行时发命令 |
| `foam_cube_grasp_sequence.py` | 多姿态搜索、规划、夹爪和状态机 | 三类物体完整夹取 |
| `piper_gripper_safe_test.py` | 夹爪独立预检与限幅测试 | 反馈开口和力度 |

还应实现只规划、不执行的 IK、整段路径和笛卡尔路径检查工具，以及统一的
`launch/`、参数 YAML、环境加载和启动脚本。文件名可以不同，但接口、坐标系、
单位和安全行为必须一致。

Word 手册会紧接本节插入“阶段 2B：在首次构建前创建全部最终工程文件”，完整
收录当前已验证版本的全部自研源文件。教学时可以把这些代码页作为教师参考答案
单独保管，先让学生按本节接口独立实现；完成阶段验收后，再逐文件对照。代码页
不包含第三方 SDK、二进制模型或某台设备的标定结果。

### R5.3 首先冻结接口，不要一开始写完整自动抓取

建议先把话题契约写进 `docs/INTERFACES.md`：

```text
/camera/color/image_raw                  sensor_msgs/msg/Image
/camera/depth/image_raw                  sensor_msgs/msg/Image
/camera/depth/camera_info                sensor_msgs/msg/CameraInfo
/foam_segmentation/mask                  sensor_msgs/msg/Image
/foam_grasp/cube_point                   geometry_msgs/msg/PointStamped
/foam_grasp/cylinder_point               geometry_msgs/msg/PointStamped
/foam_grasp/sphere_point                 geometry_msgs/msg/PointStamped
/foam_grasp/*_point_base                 geometry_msgs/msg/PointStamped
/foam_grasp/target_point_base_latched    geometry_msgs/msg/PointStamped
/joint_states_single                     sensor_msgs/msg/JointState
/end_pose                                geometry_msgs/msg/Pose
```

所有长度统一用米，关节角统一用弧度，图像 mask 的像素值固定为：

```text
0=background, 1=cube, 2=cylinder, 3=sphere
```

### R5.4 推荐实现里程碑

1. 只启动相机，验收 RGB、深度、内参和点云。
2. 离线读取一张图片完成分割，再把模型封装成 ROS 2 节点。
3. 用注册深度把 mask 转为相机坐标三维点。
4. 完成手眼标定后，把目标转换到 `base_link`。
5. 实现目标锁存与三个抓取位姿，但先只在 RViz 显示。
6. 调用 MoveIt 做 IK、碰撞和笛卡尔 plan-only 检查。
7. 独立测试观察姿态和夹爪，最后才实现自动状态机。
8. 正方体成功后再为圆柱体、球体增加尺寸和夹爪参数。

每完成一个里程碑都提交 Git：

```bash
cd ~/robot_projects/foam_grasp_project
git status
git add README.md docs workspaces/app_ws/src/foam_grasp
git commit -m "完成第一个工程骨架和接口定义"
```

部署包恢复和已有 GitHub 成品仓库克隆不属于这条教学主线；它们只在 R30、R31
作为项目完成后的迁移与发布方式介绍。

## R6. 安装工程系统依赖

学生工程此时还没有现成安装脚本，先显式安装依赖：

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake rsync \
  python3-venv python3-pip python3-colcon-common-extensions \
  python3-rosdep python3-vcstool \
  libgflags-dev nlohmann-json3-dev libdw-dev \
  libopencv-dev python3-opencv \
  can-utils ethtool iproute2 usbutils \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-image-view \
  ros-humble-tf2-ros \
  ros-humble-tf-transformations \
  ros-humble-moveit
```

学生在依赖确认后，应把同一组命令整理成自己的
`scripts/install_system_dependencies.sh`，并提交到 Git，供后续迁移复用。

检查关键命令：

```bash
command -v colcon
command -v vcs
command -v candump
command -v ethtool
command -v lsusb
ros2 pkg prefix moveit_ros_move_group
```

首次使用 rosdep：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

如果某个无关第三方 apt 源超时，而所需包最终均安装成功，可以先继续；若 apt
返回非零或缺少上述命令，必须先修复软件源。

## R7. 准备精确的第三方源码

### R7.1 为什么必须固定源码

DaBai DC1 属于 legacy/OpenNI 设备。Orbbec 官方仓库默认分支现为 `v2-main`，
legacy 设备应使用 `main`。Piper 上游默认页面可能指向 ROS 1 `noetic`，因此
不能在 ROS 2 工程里直接无脑克隆默认分支。

教师应提供“公开上游仓库 + ROS 2 分支 + 提交号”的依赖清单，而不是提供自研
项目代码。学生下载后必须记录实际提交：

```bash
cd ~/robot_projects/foam_grasp_project
git -C workspaces/orbbec_ws/src/OrbbecSDK_ROS2 rev-parse HEAD
git -C workspaces/piper_ws/src/<Piper仓库目录> rev-parse HEAD
```

将仓库 URL、版本和提交号写入 `dependencies/README.md`。项目完成后再用 `vcs
export` 生成 `.repos` 文件：

```bash
vcs export --exact workspaces/orbbec_ws/src \
  > dependencies/orbbec.repos
vcs export --exact workspaces/piper_ws/src \
  > dependencies/piper.repos
```

### R7.2 只有 Orbbec 公共仓库时的最低恢复方式

如果 `.repos` 暂时缺失，可以只恢复 DaBai 驱动，再尽快在 Git 中记录提交：

```bash
mkdir -p \
  ~/robot_projects/foam_grasp_project/workspaces/orbbec_ws/src

git clone \
  --branch main \
  --single-branch \
  https://github.com/orbbec/OrbbecSDK_ROS2.git \
  ~/robot_projects/foam_grasp_project/workspaces/orbbec_ws/src/OrbbecSDK_ROS2

git -C \
  ~/robot_projects/foam_grasp_project/workspaces/orbbec_ws/src/OrbbecSDK_ROS2 \
  rev-parse HEAD
```

本项目曾验证的 legacy 包版本为 1.5.15，但新的复现应以项目
`dependencies/orbbec.repos` 记录的提交为准。

Piper ROS 2 源码必须使用实验室指定的公开 ROS 2 仓库、分支和提交；不要用
上游默认 `noetic` 分支替代。若课程资料没有给出这三个信息，应先向教师补齐，
因为这属于第三方依赖定义，不是学生需要自行猜测的算法实现。

## R8. 安装 Orbbec udev 规则

确认源码路径：

```bash
cd ~/robot_projects/foam_grasp_project
find workspaces/orbbec_ws/src \
  -path '*/orbbec_camera/scripts/install_udev_rules.sh' \
  -print
```

执行找到的脚本；标准项目路径为：

```bash
cd \
  ~/robot_projects/foam_grasp_project/workspaces/orbbec_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

拔插相机，再检查 USB：

```bash
lsusb | grep -i -E '2bc5|orbbec'
```

DaBai DC1 正常情况下可看到 Orbbec 设备，例如 PID `0557`/`0657`。设备 ID
可能因内部接口而显示两项。

## R9. 安装项目 Python/CUDA 环境

学生先显式建立运行环境：

```bash
cd ~/robot_projects/foam_grasp_project
python3 -m venv --system-site-packages venvs/runtime
source venvs/runtime/bin/activate
export PYTHONNOUSERSITE=1

python -m pip install --upgrade pip setuptools wheel
python -m pip install \
  "numpy==1.26.4" pillow matplotlib tqdm \
  "python-can==4.6.1" "piper-sdk==0.6.1"

python -m pip install \
  "torch==2.5.1+cu121" \
  "torchvision==0.20.1+cu121" \
  --index-url https://download.pytorch.org/whl/cu121
```

这会建立：

```text
venvs/runtime
```

并固定：

```text
NumPy 1.26.4
PyTorch 2.5.1+cu121
Torchvision 0.20.1+cu121
python-can 4.6.1
piper-sdk 0.6.1
```

环境使用 `--system-site-packages` 访问 apt 安装的 ROS Python 包，同时设置
`PYTHONNOUSERSITE=1` 阻止 `~/.local` 中不可追踪的包污染运行环境。

验证通过后，把这些命令写入学生自己的 `scripts/install_python_runtime.sh`，
并将精确版本写进 `requirements/runtime.txt`；后续电脑才能重建相同环境。

安装后单独验证：

```bash
cd ~/robot_projects/foam_grasp_project
source venvs/runtime/bin/activate
export PYTHONNOUSERSITE=1

python - <<'PY'
import torch
import torchvision
import cv2
import can
import piper_sdk
from cv_bridge import CvBridge

print("torch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("CUDA available:", torch.cuda.is_available())
print("torch CUDA:", torch.version.cuda)
print("OpenCV:", cv2.__version__)
print("python-can:", can.__version__)
print("piper_sdk:", piper_sdk.__file__)
print("cv_bridge:", CvBridge)
if torch.cuda.is_available():
    print("GPU:", torch.cuda.get_device_name(0))
    x = torch.randn(512, 512, device="cuda")
    print("GPU calculation:", (x @ x).shape)
PY
```

必须看到 `CUDA available: True`。若 `nvidia-smi` 正常但 PyTorch CUDA 为
False，先重启，再检查 `LD_LIBRARY_PATH`、`libnvJitLink.so.12` 和 venv；
不要改成 CPU 运行在线分割来掩盖问题。

## R10. 放置学生自己生成的模型和手眼标定

### R10.1 模型

教学主线中，`best_model.pth` 必须来自学生在 R23～R27 完成的数据采集、标注、
训练和独立测试。训练程序直接把最佳模型导出到：

```text
~/robot_projects/foam_grasp_project/runtime/models/best_model.pth
```

如果训练输出在其他目录，再复制并记录校验值：

```bash
cd ~/robot_projects/foam_grasp_project
mkdir -p runtime/models
cp <学生训练输出目录>/best_model.pth runtime/models/best_model.pth
ls -lh runtime/models/best_model.pth
sha256sum runtime/models/best_model.pth
```

验证 checkpoint：

```bash
source scripts/source_env.sh 2>/dev/null || true
venvs/runtime/bin/python - <<'PY'
import torch
path = "runtime/models/best_model.pth"
checkpoint = torch.load(path, map_location="cpu", weights_only=False)
print("keys:", sorted(checkpoint))
print("epoch:", checkpoint.get("epoch"))
print("foreground_miou:", checkpoint.get("foreground_miou"))
print("class_names:", checkpoint.get("class_names"))
PY
```

类别顺序必须为：

```text
background, cube, cylinder, sphere
```

### R10.2 标定

R19 完成后，把学生自己的标定程序输出保存为：

```text
~/robot_projects/foam_grasp_project/runtime/calibration/handeye_eye_in_hand.json
```

检查：

```bash
cd ~/robot_projects/foam_grasp_project
python3 -m json.tool \
  runtime/calibration/handeye_eye_in_hand.json
sha256sum runtime/calibration/handeye_eye_in_hand.json
```

JSON 至少应包含：

```json
{
  "position": [0.0, 0.0, 0.0],
  "orientation": [0.0, 0.0, 0.0, 1.0]
}
```

示例中的零值不能用于真机。位置单位为米，四元数顺序为 `x, y, z, w`，变换
方向必须是 `T_gripper_camera`。

不要用其他机械臂的 JSON 通过验收。只要相机、支架、法兰或夹爪安装发生变化，
就必须重新标定，见 R19。

## R11. 构建第三方工作空间和学生应用

首次构建时显式执行，便于定位每个工作空间的问题：

```bash
cd ~/robot_projects/foam_grasp_project
source /opt/ros/humble/setup.bash

colcon build \
  --base-paths workspaces/orbbec_ws/src \
  --build-base workspaces/orbbec_ws/build \
  --install-base workspaces/orbbec_ws/install
source workspaces/orbbec_ws/install/setup.bash

colcon build \
  --base-paths workspaces/piper_ws/src \
  --build-base workspaces/piper_ws/build \
  --install-base workspaces/piper_ws/install
source workspaces/piper_ws/install/setup.bash

if find workspaces/moveit_ws/src -mindepth 1 -print -quit | grep -q .
then
  colcon build \
    --base-paths workspaces/moveit_ws/src \
    --build-base workspaces/moveit_ws/build \
    --install-base workspaces/moveit_ws/install
  source workspaces/moveit_ws/install/setup.bash
fi

source venvs/runtime/bin/activate
colcon build \
  --base-paths workspaces/app_ws/src \
  --build-base workspaces/app_ws/build \
  --install-base workspaces/app_ws/install \
  --symlink-install
```

叠加顺序是：

```text
/opt/ros/humble
→ workspaces/orbbec_ws/install
→ workspaces/piper_ws/install
→ workspaces/moveit_ws/install（可选）
→ workspaces/app_ws/install
```

上述命令通过后，再把顺序封装为学生自己的 `scripts/build_all.sh`。开发期可只
重建 `app_ws`；第三方源码或依赖变化时才重建对应工作空间。

构建后：

```bash
source /opt/ros/humble/setup.bash
source workspaces/orbbec_ws/install/setup.bash
source workspaces/piper_ws/install/setup.bash
test ! -f workspaces/moveit_ws/install/setup.bash \
  || source workspaces/moveit_ws/install/setup.bash
source workspaces/app_ws/install/setup.bash
ros2 pkg prefix orbbec_camera
ros2 pkg prefix piper
ros2 pkg prefix foam_grasp
ros2 pkg executables foam_grasp | sort
```

期望 `foam_grasp` 指向项目内：

```text
~/robot_projects/foam_grasp_project/workspaces/app_ws/install/foam_grasp
```

执行静态检查：

```bash
./scripts/validate_project.sh
./scripts/project_status.sh
```

全新电脑也可一次执行：

```bash
./scripts/bootstrap_new_machine.sh --install-system
```

但前提是项目已经包含 `.repos`、模型和标定。这个脚本只安装和构建，不启动 ROS，
也不移动机械臂。

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

## R19. 眼在手上手眼标定

如果相机与法兰的刚性安装完全未变，可使用当前已验证 JSON；否则必须重新标定。
本项目在线节点消费标定结果，但当前仓库不包含一键标定采集器，因此本节给出
严格的数据与验收流程，不把示例数值伪装成可用标定。

### R19.1 坐标方向

工程需要：

```text
T_gripper_camera
p_base = T_base_gripper · T_gripper_camera · p_camera
```

机器人每个姿态记录：

```text
T_base_gripper
```

标定板检测记录：

```text
T_camera_target
```

OpenCV `calibrateHandEye` 的输入是 `gripper2base` 和 `target2cam`，输出
`cam2gripper`，与本项目所需方向一致。

### R19.2 启动相机与 Piper 反馈

可以启动完整系统，也可分别启动相机和 Piper。检查：

```bash
source scripts/source_env.sh
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /end_pose --once
```

### R19.3 采集 20–30 组姿态

标定板固定不动。每次机械臂到位并完全静止后，保存：

- 彩色图；
- 相机内参；
- 标定板在相机坐标系中的位姿；
- `/end_pose`；
- 时间戳。

姿态必须覆盖：

- 明显不同的 roll/pitch/yaw；
- 不同距离；
- 标定板位于画面不同区域；
- 至少 15 组，推荐 20–30 组；
- 另留 5–10 组只做验证。

检查末端位姿：

```bash
ros2 topic echo /end_pose --once
```

检查相机内参：

```bash
ros2 topic echo /camera/color/camera_info --once --field k
```

### R19.4 OpenCV 求解核心

在求解脚本中：

```python
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base,
    t_gripper2base,
    R_target2cam,
    t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI,
)
```

同时比较 Tsai、Park、Horaud 等方法，用保留验证姿态检查闭环一致性。将平移统一
转为米，旋转转为 `xyzw` 四元数，写入：

```text
runtime/calibration/handeye_eye_in_hand.json
```

### R19.5 标定验收

对固定标定板上的同一点，在不同机械臂姿态下计算 base 坐标。原型建议：

```text
验证集平移误差中位数 < 5 mm
最大误差 < 10 mm
```

再让真实桌面目标保持不动、机械臂改变观察姿态，确认
`/foam_grasp/{class}_point_base` 不明显漂移。只有通过后，才继续抓取。

## R20. 目标锁定、IK 和 plan-only 验收

把一个正方体放在机械臂容易到达、画面中央的桌面区域。

### R20.1 锁定目标

```bash
ros2 service call \
  /foam_grasp/clear_latched_target \
  std_srvs/srv/Trigger \
  "{}"

ros2 service call \
  /foam_grasp/latch_cube \
  std_srvs/srv/Trigger \
  "{}"
```

查看：

```bash
ros2 topic echo /foam_grasp/target_point_base_latched --once
ros2 topic echo /foam_grasp/latched_target_class --once
```

响应应包含样本数和 spread。默认锁定窗口 1.5 秒、最少 15 个样本、最大扩散
10 mm。

### R20.2 抓取位姿

```bash
ros2 topic echo /foam_grasp/cube_pregrasp_pose --once
ros2 topic echo /foam_grasp/cube_grasp_pose --once
ros2 topic echo /foam_grasp/cube_lift_pose --once
```

位姿 frame 应为 `base_link`。

### R20.3 IK 检查

```bash
ros2 run foam_grasp grasp_ik_check
```

必须看到 PREGRASP、GRASP、LIFT 都可达。该命令不会执行轨迹。

### R20.4 完整路径检查

```bash
ros2 run foam_grasp grasp_plan_check
```

必须看到：

```text
TABLE COLLISION OBJECT: APPLIED
CURRENT_TO_PREGRASP: PLAN SUCCESS
PREGRASP_TO_GRASP: PLAN SUCCESS
GRASP_TO_LIFT: PLAN SUCCESS
```

### R20.5 笛卡尔接近/抬升检查

```bash
ros2 run foam_grasp grasp_cartesian_check
```

必须看到接近和抬升均为 100%。这些检查只向 RViz 发布规划轨迹，不向 Piper
发送动作。

### R20.6 整体流程 plan-only

```bash
ros2 run foam_grasp object_grasp_sequence \
  --target-class cube
```

程序会搜索多姿态、多高度候选，做 IK、碰撞、OMPL 和笛卡尔检查，但没有
`--execute` 时不会移动真机。

## R21. 第一次真实正方体夹取

首次夹取条件：

- 机械臂已在观察姿态；
- 正方体位于画面中央和中近距离；
- 同类只放一个；
- 工作空间无人；
- 急停可触及；
- `/joint_states` 发布者为 0；
- `allow_trajectory_execution=False`；
- R20 全部通过。

使用项目统一入口：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
./启动.sh cube
```

流程自动完成：

```text
观察姿态
→ 等待视觉稳定
→ 清除旧锁定
→ 多帧锁定 cube
→ 搜索安全抓取候选
→ IK/碰撞/完整规划
→ 高位张开至 70 mm
→ CURRENT→PREGRASP
→ 垂直接近
→ 闭合命令 40 mm
→ 根据实际开口确认夹持
→ 垂直抬升
```

首次运行时盯住机械臂和终端。程序的安全拒绝不是“软件坏了”，而是说明至少一项
输入、状态或规划不满足当前放行条件。不要直接删除阈值。

## R22. 实现圆柱体和球体夹取

圆柱体：

```bash
./启动.sh cylinder
```

球体：

```bash
./启动.sh sphere
```

也可交互：

```bash
./启动.sh
```

输入：

```text
1 / cube / 正方体
2 / cylinder / 圆柱体
3 / sphere / 球体
```

默认夹爪配置：

| 类别 | 几何尺寸 | 预张开 | 闭合命令 | 最小夹持余量 |
|---|---|---:|---:|---:|
| cube | 边长 50 mm | 70 mm | 40 mm | 5 mm |
| cylinder | Ø70 × 70 mm | 90 mm | 55 mm | 4 mm |
| sphere | Ø60 mm | 70 mm | 45 mm | 5 mm |

圆柱体会在高位先张开到 90 mm，避免边下降边张开压到目标。闭合后的实际开口
必须比闭合命令大于最小余量，才认为夹爪中存在物体并允许抬升。

## R23. 数据采集：从零建立分割数据集

如果使用现有模型可以跳到 R15。要从零训练，先启动相机：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
ros2 launch orbbec_camera dabai.launch.py \
  depth_registration:=true \
  enable_ir:=false \
  enable_point_cloud:=false
```

另一个终端启动保存节点：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
mkdir -p data/raw/images/session_01

python3 training/capture_color_images_node.py \
  --output-dir data/raw/images/session_01 \
  --topic /camera/color/image_raw \
  --service /foam_dataset/save \
  --prefix frame_
```

第三个终端触发：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh

while true
do
  read -r -p "调整物体并静止1秒后按回车拍照；Ctrl+C退出："
  ros2 service call \
    /foam_dataset/save \
    std_srvs/srv/Trigger \
    "{}"
done
```

每次只在物体、位置、角度、背景、距离、光照或遮挡发生实际变化后拍一张。避免
连续保存大量相似帧。建议初版至少：

```text
每类 150–300 张有效实例
总图 450–900 张
```

历史原型使用 369 张完成闭环，但更换背景或光照后应扩充。

检查数量、尺寸和损坏文件：

```bash
find data/raw/images/session_01 \
  -maxdepth 1 -type f -name '*.png' | wc -l

file data/raw/images/session_01/frame_0000.png

python3 - <<'PY'
from pathlib import Path
from PIL import Image

root = Path("data/raw/images/session_01")
files = sorted(root.glob("*.png"))
sizes = {}
bad = []
for path in files:
    try:
        with Image.open(path) as image:
            image.verify()
        with Image.open(path) as image:
            sizes[image.size] = sizes.get(image.size, 0) + 1
    except Exception as error:
        bad.append((path.name, repr(error)))
print("count:", len(files))
print("sizes:", sizes)
print("bad:", bad)
PY
```

## R24. LabelMe 语义分割标注

使用独立环境：

```bash
cd ~/robot_projects/foam_grasp_project
python3 -m venv venvs/labelme
source venvs/labelme/bin/activate
python -m pip install --upgrade pip
python -m pip install "numpy<2" labelme
```

建立标注目录：

```bash
mkdir -p data/raw/annotations_labelme/session_01
labelme \
  data/raw/images/session_01 \
  --output data/raw/annotations_labelme/session_01
```

标签只能使用：

```text
cube
cylinder
sphere
```

标注规则：

- 使用 polygon 贴合可见轮廓；
- 被遮挡时只标可见区域；
- 只出现一半但仍能确认类别时仍需标注；
- 只剩无法辨认的小碎片时可以不作为训练实例；
- 阴影不属于物体；
- 不把夹爪、桌面或反光边缘标入目标；
- 同一标签始终使用完全相同的英文拼写。

开启自动保存后，关闭 LabelMe 前仍需验证：

```bash
echo "图片数量："
find data/raw/images/session_01 \
  -maxdepth 1 -type f -name '*.png' | wc -l

echo "JSON数量："
find data/raw/annotations_labelme/session_01 \
  -maxdepth 1 -type f -name '*.json' | wc -l

python3 - <<'PY'
from pathlib import Path
import json

root = Path("data/raw/annotations_labelme/session_01")
files = sorted(root.glob("*.json"))
for path in files:
    json.loads(path.read_text(encoding="utf-8"))
print("全部JSON可解析，数量：", len(files))
PY
```

图片与 JSON 必须一一对应。若决定删除一张无法标注的图片，要同时保证后续
输入中没有对应孤立 JSON。

## R25. 生成 mask、预览和数据划分

保持 labelme venv 或使用能导入 Pillow/NumPy 的环境：

```bash
cd ~/robot_projects/foam_grasp_project
source venvs/labelme/bin/activate
```

转换为索引 mask：

```bash
python training/labelme_to_masks.py \
  --images-dir data/raw/images/session_01 \
  --annotations-dir data/raw/annotations_labelme/session_01 \
  --output-dir data/raw/masks/session_01
```

像素值：

```text
0=background
1=cube
2=cylinder
3=sphere
```

检查并生成总览：

```bash
python training/make_mask_preview.py \
  --images-dir data/raw/images/session_01 \
  --masks-dir data/raw/masks/session_01 \
  --output-dir data/raw/previews/session_01 \
  --sample-count 24
```

人工查看：

```text
data/raw/previews/session_01/mask_contact_sheet.jpg
```

数量检查：

```bash
find data/raw/images/session_01 -maxdepth 1 -name '*.png' | wc -l
find data/raw/annotations_labelme/session_01 -maxdepth 1 -name '*.json' | wc -l
find data/raw/masks/session_01 -maxdepth 1 -name '*.png' | wc -l
```

三者必须一致。

划分：

```bash
python training/split_segmentation_dataset.py \
  --images-dir data/raw/images/session_01 \
  --masks-dir data/raw/masks/session_01 \
  --output-dir data/segmentation_dataset
```

若需要确认覆盖旧划分：

```bash
python training/split_segmentation_dataset.py \
  --images-dir data/raw/images/session_01 \
  --masks-dir data/raw/masks/session_01 \
  --output-dir data/segmentation_dataset \
  --force
```

默认比例 80/10/10，脚本搜索 2,000 个随机种子，使各集合类别出现率更接近全局
分布。历史 369 张结果是：

```text
train 295
val 37
test 37
```

不要从同一段几乎相同的视频抽帧后随机拆分到 train/test；应按采集场景分组，
避免数据泄漏。

## R26. 训练 DeepLabV3 + ResNet-50

退出 LabelMe 环境，加载项目 runtime：

```bash
deactivate
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
```

再次确认 GPU：

```bash
python - <<'PY'
import torch
print(torch.__version__)
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "NO GPU")
PY
```

先做烟雾测试：

```bash
python training/train_foam_segmentation.py \
  --dataset-root data/segmentation_dataset \
  --runs-root data/training_runs \
  --smoke-test \
  --batch-size 2 \
  --workers 2
```

烟雾测试的 mIoU 可能接近 0；它只验证数据、模型、GPU、前向、反向和保存链路。

完整训练：

```bash
python training/train_foam_segmentation.py \
  --dataset-root data/segmentation_dataset \
  --runs-root data/training_runs \
  --epochs 40 \
  --batch-size 4 \
  --workers 4 \
  --learning-rate 1e-4 \
  --seed 42
```

输出：

```text
data/training_runs/deeplabv3_resnet50/
├── best_model.pth
├── last_model.pth
└── train.log
```

显存不足时按顺序调整：

1. `--batch-size 2`；
2. `--workers 2`；
3. 关闭 RViz 和其他 GPU 程序；
4. 不要随意改输入尺寸，除非同步验证在线预处理。

## R27. 独立测试并发布模型

测试：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh

python training/evaluate_foam_segmentation.py \
  --dataset-root data/segmentation_dataset \
  --checkpoint \
    data/training_runs/deeplabv3_resnet50/best_model.pth \
  --output-dir \
    data/test_results/deeplabv3_resnet50 \
  --batch-size 4 \
  --workers 4
```

查看：

```text
data/test_results/deeplabv3_resnet50/test_metrics.json
data/test_results/deeplabv3_resnet50/prediction_masks
data/test_results/deeplabv3_resnet50/comparisons
data/test_results/deeplabv3_resnet50/test_contact_sheet.png
```

发布：

```bash
cp \
  data/training_runs/deeplabv3_resnet50/best_model.pth \
  runtime/models/best_model.pth

sha256sum runtime/models/best_model.pth
```

替换模型后重新启动系统，先查看 overlay，再做锁定、IK、plan-only，最后才做
真实抓取。

历史测试结果：

| 指标 | 数值 |
|---|---:|
| Pixel accuracy | 0.998865 |
| Foreground mIoU | 0.973698 |
| Cube IoU | 0.965921 |
| Cylinder IoU | 0.980466 |
| Sphere IoU | 0.974709 |

这些数字不能替代现场泛化检查。

## R28. 日常开机运行

终端 1：

```bash
cd ~/robot_projects/foam_grasp_project
./scripts/setup_can.sh
timeout 5 candump -n 20 can0
./scripts/start_system.sh
```

终端 2：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
./scripts/check_system.sh
```

选择目标：

```bash
./启动.sh
```

或：

```bash
./启动.sh cube
./启动.sh cylinder
./启动.sh sphere
```

## R29. 标准停止与重新启动

抓取完成后让机械臂保持在安全位置。停止系统：

1. 在运行 `start_system.sh` 的终端按一次 `Ctrl+C`；
2. 等所有子进程 cleanly finished；
3. 检查没有核心残留；
4. 再关闭机械臂电源或按实验室规定失能。

```bash
pgrep -af \
  'piper_single_ctrl|piper_ctrl_single|move_group|orbbec|foam_' \
  || echo "核心进程已退出"
```

不要用关闭终端窗口代替正常 `Ctrl+C`。若所有终端误关：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
ros2 node list
pgrep -af \
  'piper_single_ctrl|piper_ctrl_single|move_group|orbbec|foam_'
```

先确认实际进程状态，再决定是否重新启动。重复节点会导致 DDS 名称警告和错误
的命令/反馈连接。

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
