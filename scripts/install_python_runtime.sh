#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -d "${RUNTIME_VENV}" ]]; then
  python3 -m venv --system-site-packages "${RUNTIME_VENV}"
elif [[ -f "${RUNTIME_VENV}/pyvenv.cfg" ]]; then
  # Older project revisions may have created this venv without access to the
  # apt-installed ROS 2/colcon modules. Keep the installed CUDA wheels, but
  # restore the system-site-packages setting required by this runtime.
  sed -i \
    's/^include-system-site-packages = false$/include-system-site-packages = true/' \
    "${RUNTIME_VENV}/pyvenv.cfg"
fi

# shellcheck disable=SC1091
source "${RUNTIME_VENV}/bin/activate"
# A system-site-packages venv is required for ROS 2, but user-site packages
# under ~/.local must not leak into this reproducible CUDA runtime.
export PYTHONNOUSERSITE=1
python -m pip install \
  -i https://mirrors.aliyun.com/pypi/simple/ \
  "setuptools<80" wheel
# Reinstall this binary wheel even when stale metadata says it is present.
# A copied or interrupted venv can otherwise keep the dist-info directory but
# lose libnvJitLink.so.12, causing torch to fail before CUDA initialization.
python -m pip install \
  --ignore-installed \
  --no-deps \
  -i https://mirrors.aliyun.com/pypi/simple/ \
  nvidia-nvjitlink-cu12==12.1.105
python -m pip install \
  -i https://mirrors.aliyun.com/pypi/simple/ \
  -r "${PROJECT_ROOT}/requirements/runtime.txt"
python -m pip install \
  torch==2.5.1 torchvision==0.20.1 \
  --index-url https://download.pytorch.org/whl/cu121 \
  --extra-index-url https://mirrors.aliyun.com/pypi/simple/

runtime_site="$(python -c 'import site; print(site.getsitepackages()[0])')"
nvjitlink_lib="${runtime_site}/nvidia/nvjitlink/lib"
if [[ -d "${nvjitlink_lib}" ]]; then
  export LD_LIBRARY_PATH="${nvjitlink_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

python - <<'PY'
import torch
import torchvision
import can
import piper_sdk
import cv2
from cv_bridge import CvBridge
from colcon_core.command import main as colcon_main
print("PyTorch:", torch.__version__)
print("Torchvision:", torchvision.__version__)
print("python-can:", can.__version__)
print("piper_sdk:", piper_sdk.__file__)
print("OpenCV:", cv2.__version__)
print("cv_bridge:", CvBridge)
print("venv可导入系统colcon_core：是")
print("CUDA available:", torch.cuda.is_available())
if not torch.cuda.is_available():
    raise SystemExit(
        "CUDA GPU不可用；请先安装兼容NVIDIA驱动并重启"
    )
print("GPU:", torch.cuda.get_device_name(0))
x = torch.ones((16, 16), device="cuda")
print("CUDA smoke test:", float((x @ x).sum().item()))
PY

# Piper's ROS entry point is generated with /usr/bin/python3. Verify the exact
# interpreter/environment combination used by the launch file.
PYTHONPATH="${runtime_site}" PYTHONNOUSERSITE=1 /usr/bin/python3 - <<'PY'
import can
import piper_sdk
print("Piper system-interpreter import: OK")
PY
