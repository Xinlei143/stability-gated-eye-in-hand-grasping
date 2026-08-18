#!/usr/bin/env bash
set -eo pipefail

# Validate the semantic-segmentation checkpoint metadata and the hand-eye JSON.
# This is read-only and does not import rclpy or access hardware.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
release_config="${PROJECT_ROOT}/config/runtime-release.env"
require_file "${release_config}"
# shellcheck disable=SC1090
source "${release_config}"
require_file "${CHECKPOINT_PATH}"
require_file "${CALIBRATION_PATH}"

model_size="$(wc -c < "${CHECKPOINT_PATH}")"
calibration_size="$(wc -c < "${CALIBRATION_PATH}")"
model_sha="$(sha256sum "${CHECKPOINT_PATH}" | awk '{print $1}')"
calibration_sha="$(sha256sum "${CALIBRATION_PATH}" | awk '{print $1}')"
if [[ "${model_size}" != "${RUNTIME_MODEL_BYTES}" \
    || "${model_sha}" != "${RUNTIME_MODEL_SHA256}" ]]; then
  echo "模型不是v1.0.0验证权重，拒绝继续。" >&2
  echo "actual: bytes=${model_size}, sha256=${model_sha}" >&2
  exit 1
fi
if [[ "${calibration_size}" != "${RUNTIME_CALIBRATION_BYTES}" \
    || "${calibration_sha}" != "${RUNTIME_CALIBRATION_SHA256}" ]]; then
  echo "手眼标定不是当前rig的v1.0.0标定，拒绝继续。" >&2
  echo "actual: bytes=${calibration_size}, sha256=${calibration_sha}" >&2
  exit 1
fi

python3 - "${CALIBRATION_PATH}" <<'PY'
import json
import math
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
with path.open("r", encoding="utf-8") as stream:
    calibration = json.load(stream)

position = calibration.get("position")
orientation = calibration.get("orientation")
if not isinstance(position, list) or len(position) != 3:
    raise SystemExit("calibration.position必须包含3个数")
if not isinstance(orientation, list) or len(orientation) != 4:
    raise SystemExit("calibration.orientation必须包含4个数")
values = [float(value) for value in position + orientation]
if not all(math.isfinite(value) for value in values):
    raise SystemExit("标定包含NaN或无穷值")
norm = math.sqrt(sum(value * value for value in values[3:]))
if not 0.95 <= norm <= 1.05:
    raise SystemExit(f"标定四元数未归一化：norm={norm:.6f}")
if any(abs(value) > 1.0 for value in values[:3]):
    raise SystemExit("标定平移绝对值超过1m，请检查单位")
print("Calibration schema: OK")
PY

if [[ -x "${RUNTIME_VENV}/bin/python" ]]; then
  PYTHONNOUSERSITE=1 "${RUNTIME_VENV}/bin/python" \
    - "${CHECKPOINT_PATH}" <<'PY'
import pathlib
import sys
import torch
from torchvision.models.segmentation import deeplabv3_resnet50

path = pathlib.Path(sys.argv[1])
checkpoint = torch.load(path, map_location="cpu", weights_only=False)
if not isinstance(checkpoint, dict):
    raise SystemExit("checkpoint根节点不是dict")
required = {
    "model_state_dict",
    "class_names",
    "num_classes",
    "input_size",
}
missing = required.difference(checkpoint)
if missing:
    raise SystemExit(f"checkpoint缺少字段：{sorted(missing)}")
expected_classes = ["background", "cube", "cylinder", "sphere"]
if list(checkpoint["class_names"]) != expected_classes:
    raise SystemExit(
        f"类别顺序不匹配：{checkpoint['class_names']!r}"
    )
if int(checkpoint["num_classes"]) != 4:
    raise SystemExit("num_classes不是4")
if list(checkpoint["input_size"]) != [360, 640]:
    raise SystemExit(
        f"input_size不匹配：{checkpoint['input_size']!r}"
    )
state = checkpoint["model_state_dict"]
if not isinstance(state, dict) or not state:
    raise SystemExit("model_state_dict为空")
model = deeplabv3_resnet50(
    weights=None,
    weights_backbone=None,
    num_classes=4,
    aux_loss=True,
)
model.load_state_dict(state, strict=True)
print("Checkpoint metadata + strict model load: OK")
PY
else
  echo "运行venv尚未安装；已跳过checkpoint内部元数据检查。"
fi

echo "运行资源验证通过。"
