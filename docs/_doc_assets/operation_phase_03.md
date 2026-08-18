# phase

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

