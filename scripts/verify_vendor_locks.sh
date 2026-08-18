#!/usr/bin/env bash
set -eo pipefail

# Verify every imported Git checkout against the exact commit in .repos.
# Requires python3-yaml, installed by install_system_dependencies.sh.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

python3 - \
  "${PROJECT_ROOT}/dependencies/orbbec.repos" \
  "${PROJECT_ROOT}/workspaces/orbbec_ws/src" \
  "${PROJECT_ROOT}/dependencies/piper.repos" \
  "${PROJECT_ROOT}/workspaces/piper_ws/_upstream" \
  "${PROJECT_ROOT}/dependencies/moveit.repos" \
  "${PROJECT_ROOT}/workspaces/moveit_ws/src" <<'PY'
import pathlib
import subprocess
import sys

import yaml


def git_output(repository, *arguments):
    return subprocess.check_output(
        ["git", "-C", str(repository), *arguments],
        text=True,
        stderr=subprocess.STDOUT,
    ).strip()


def verify_manifest(manifest_path, source_root, optional=False):
    manifest_path = pathlib.Path(manifest_path)
    source_root = pathlib.Path(source_root)
    if optional and not manifest_path.exists():
        return
    if not manifest_path.is_file():
        raise SystemExit(f"缺少版本清单：{manifest_path}")
    with manifest_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    repositories = data.get("repositories")
    if not isinstance(repositories, dict) or not repositories:
        raise SystemExit(f"无效版本清单：{manifest_path}")
    for name, description in repositories.items():
        repository = source_root / name
        if not (repository / ".git").exists():
            raise SystemExit(f"缺少Git仓库：{repository}")
        expected = str(description["version"])
        actual = git_output(repository, "rev-parse", "HEAD")
        if actual != expected:
            raise SystemExit(
                f"{name}提交不匹配：expected={expected}, actual={actual}"
            )
        if git_output(repository, "status", "--porcelain"):
            raise SystemExit(f"{name}源码存在未提交修改：{repository}")
        print(f"[OK] {name}: {actual}")


verify_manifest(sys.argv[1], sys.argv[2])
verify_manifest(sys.argv[3], sys.argv[4])
verify_manifest(sys.argv[5], sys.argv[6], optional=True)
PY

kinematics_file="${PROJECT_ROOT}/workspaces/piper_ws/src/piper_moveit/"\
"piper_with_gripper_moveit/config/kinematics.yaml"
if ! grep -Fqx \
    "  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin" \
    "${kinematics_file}"; then
  echo "Piper KDL补丁未生效：${kinematics_file}" >&2
  exit 1
fi
if grep -q "TRAC_IKKinematicsPlugin" "${kinematics_file}"; then
  echo "Piper配置仍残留TRAC-IK，和已验证配置不一致。" >&2
  exit 1
fi
echo "[OK] Piper patch: KDL kinematics"

echo "第三方源码精确版本验证通过。"
