# phase

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

