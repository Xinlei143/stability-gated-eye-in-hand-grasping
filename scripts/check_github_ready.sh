#!/usr/bin/env bash
set -eo pipefail

# Read-only pre-publication audit. It never stages, commits, pushes, starts ROS,
# or communicates with hardware.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
release_config="${PROJECT_ROOT}/config/runtime-release.env"
if [[ -f "${release_config}" ]]; then
  # shellcheck disable=SC1090
  source "${release_config}"
fi

errors=0
warnings=0

error() {
  echo "[ERROR] $*" >&2
  errors=$((errors + 1))
}

warn() {
  echo "[WARN]  $*" >&2
  warnings=$((warnings + 1))
}

ok() {
  echo "[OK]    $*"
}

if ! command -v rg >/dev/null 2>&1; then
  error "缺少ripgrep（rg）；请先运行 scripts/install_system_dependencies.sh"
  echo "===== GitHub readiness: errors=${errors}, warnings=${warnings} ====="
  exit 1
fi

git_root="$(git -C "${PROJECT_ROOT}" rev-parse --show-toplevel 2>/dev/null || true)"
if [[ "${git_root}" != "${PROJECT_ROOT}" ]]; then
  error "foam_grasp_project必须是独立Git仓库；当前Git根目录为 ${git_root:-none}"
  echo "在项目根目录执行：git init -b main" >&2
else
  ok "独立Git根目录正确"
  if git -C "${PROJECT_ROOT}" rev-parse --verify HEAD >/dev/null 2>&1; then
    ok "Git仓库已有提交"
  else
    warn "Git仓库尚无提交；完成初始提交后请重新检查"
  fi
  origin_url="$(
    git -C "${PROJECT_ROOT}" remote get-url origin 2>/dev/null || true
  )"
  if [[ -n "${origin_url}" ]]; then
    ok "Git origin：${origin_url}"
  else
    warn "尚未设置Git origin"
  fi
fi

required_files=(
  VERSION
  README.md
  CONTRIBUTING.md
  SECURITY.md
  LICENSE
  THIRD_PARTY.md
  PROJECT_MANIFEST.md
  install.sh
  .gitignore
  config/runtime-release.env
  docs/ONE_CLICK_DEPLOY.md
  docs/RELEASE_CHECKLIST.md
  runtime/MODEL_CARD.md
  patches/README.md
  patches/piper_ros/0001-use-kdl-kinematics.patch
  scripts/fetch_runtime_assets.sh
  scripts/check_release_online.sh
  scripts/prepare_runtime_release.sh
  scripts/preflight_new_machine.sh
  scripts/post_install_smoke.sh
  scripts/verify_runtime_assets.sh
  scripts/verify_vendor_locks.sh
  workspaces/app_ws/src/foam_grasp/package.xml
  workspaces/app_ws/src/foam_grasp/setup.py
)
for path in "${required_files[@]}"; do
  if [[ -f "${PROJECT_ROOT}/${path}" ]]; then
    ok "${path}"
    if [[ "${git_root}" == "${PROJECT_ROOT}" ]] \
        && ! git -C "${PROJECT_ROOT}" \
          ls-files --error-unmatch "${path}" >/dev/null 2>&1; then
      warn "${path}尚未被Git跟踪"
    fi
  else
    error "缺少 ${path}"
  fi
done

runtime_requirements=(
  'nvidia-nvjitlink-cu12==12.1.105'
  'python-can==4.6.1'
  'piper-sdk==0.6.1'
)
for requirement in "${runtime_requirements[@]}"; do
  if grep -Fqx -- "${requirement}" \
      "${PROJECT_ROOT}/requirements/runtime.txt"; then
    ok "runtime requirement: ${requirement}"
  else
    error "最终运行时缺少：${requirement}"
  fi
done

if grep -q 'nvidia/nvjitlink/lib' "${PROJECT_ROOT}/scripts/_common.sh" \
    && grep -q 'Piper system-interpreter import: OK' \
      "${PROJECT_ROOT}/scripts/install_python_runtime.sh" \
    && grep -q 'Runtime imports' "${PROJECT_ROOT}/scripts/project_status.sh"; then
  ok "最终运行时导入修复已包含"
else
  error "缺少nvJitLink/Piper解释器/运行时导入修复"
fi

for manifest in \
  dependencies/orbbec.repos \
  dependencies/piper.repos \
  dependencies/moveit.repos; do
  if [[ -s "${PROJECT_ROOT}/${manifest}" ]]; then
    ok "${manifest}"
  else
    error "缺少 ${manifest}；请在已工作Ubuntu运行 scripts/capture_environment.sh"
  fi
done

version="$(tr -d '[:space:]' < "${PROJECT_ROOT}/VERSION")"
package_xml_version="$(
  sed -n 's:.*<version>\([^<]*\)</version>.*:\1:p' \
    "${PROJECT_ROOT}/workspaces/app_ws/src/foam_grasp/package.xml" \
    | head -n 1
)"
setup_version="$(
  sed -n 's/^[[:space:]]*version="\([^"]*\)",/\1/p' \
    "${PROJECT_ROOT}/workspaces/app_ws/src/foam_grasp/setup.py" \
    | head -n 1
)"
if [[ -n "${version}" \
    && "${package_xml_version}" == "${version}" \
    && "${setup_version}" == "${version}" \
    && "${RUNTIME_RELEASE_TAG:-}" == "v${version}" \
    && "${RUNTIME_RELEASE_ASSET:-}" \
      == "foam-grasp-runtime-v${version}.tar.gz" \
    && "${RUNTIME_RELEASE_CHECKSUM_ASSET:-}" \
      == "foam-grasp-runtime-v${version}.tar.gz.sha256" ]]; then
  ok "源码、ROS包和运行资源版本统一为v${version}"
else
  error "VERSION、ROS包版本或runtime-release.env不一致"
fi

if grep -Eq '^RUNTIME_MODEL_SHA256=[0-9a-f]{64}$' \
    "${PROJECT_ROOT}/config/runtime-release.env" \
    && grep -Eq '^RUNTIME_CALIBRATION_SHA256=[0-9a-f]{64}$' \
      "${PROJECT_ROOT}/config/runtime-release.env"; then
  ok "模型和手眼标定SHA-256已固定在源码版本中"
else
  error "runtime-release.env缺少固定资源SHA-256"
fi

if [[ -n "${origin_url:-}" \
    && -n "${RUNTIME_RELEASE_REPOSITORY:-}" ]]; then
  case "${origin_url}" in
    *"github.com/${RUNTIME_RELEASE_REPOSITORY}.git"|\
    *"github.com/${RUNTIME_RELEASE_REPOSITORY}"|\
    *"github.com:${RUNTIME_RELEASE_REPOSITORY}.git"|\
    *"github.com:${RUNTIME_RELEASE_REPOSITORY}")
      ok "Git origin与运行资源仓库一致"
      ;;
    *)
      error "Git origin与RUNTIME_RELEASE_REPOSITORY不一致"
      ;;
  esac
fi

if [[ "${git_root}" == "${PROJECT_ROOT}" ]] \
    && git -C "${PROJECT_ROOT}" \
      rev-parse --verify "refs/tags/v${version}" >/dev/null 2>&1; then
  ok "本地tag v${version}已存在"
else
  warn "本地tag v${version}尚未创建；发布Release前必须创建并推送"
fi

if rg -n 'maintainer@example\.com|YOUR_GITHUB_EMAIL|YOUR NAME' \
    "${PROJECT_ROOT}/workspaces/app_ws/src/foam_grasp/package.xml" \
    "${PROJECT_ROOT}/workspaces/app_ws/src/foam_grasp/setup.py"; then
  error "ROS包维护者信息仍是占位符"
else
  ok "ROS包维护者信息已设置"
fi

if [[ -n "${git_root}" && "${git_root}" == "${PROJECT_ROOT}" ]]; then
  forbidden_regex='(^|/)(venvs|build|install|log)(/|$)|\.pth$|\.pt$|\.onnx$|\.engine$|\.db3$|\.mcap$|\.bag$|runtime/calibration/.*\.json$'
  while IFS= read -r tracked; do
    if [[ "${tracked}" =~ ${forbidden_regex} ]]; then
      error "禁止跟踪的文件：${tracked}"
    fi
    if [[ -f "${PROJECT_ROOT}/${tracked}" ]]; then
      size="$(wc -c < "${PROJECT_ROOT}/${tracked}")"
      if (( size > 50 * 1024 * 1024 )); then
        error "跟踪文件超过50 MiB：${tracked} (${size} bytes)"
      fi
    fi
  done < <(git -C "${PROJECT_ROOT}" ls-files)
fi

if rg -n --hidden \
    --glob '!.git/**' \
    --glob '!scripts/check_github_ready.sh' \
    '(github_pat_[A-Za-z0-9_]+|gh[pousr]_[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16}|BEGIN (RSA |OPENSSH |EC )?PRIVATE KEY)' \
    "${PROJECT_ROOT}"; then
  error "发现疑似凭据或私钥"
else
  ok "未发现常见Token/私钥格式"
fi

if rg -n \
    --glob '*.py' --glob '*.sh' --glob '*.yaml' --glob '*.xml' \
    --glob '!validate_project.sh' --glob '!check_github_ready.sh' \
    '/home/rl|/Users/wl' \
    "${PROJECT_ROOT}/workspaces/app_ws/src/foam_grasp" \
    "${PROJECT_ROOT}/training" \
    "${PROJECT_ROOT}/scripts"; then
  error "发现不可迁移的个人绝对路径"
else
  ok "自研源码中未发现个人绝对路径"
fi

echo "===== GitHub readiness: errors=${errors}, warnings=${warnings} ====="
if (( errors > 0 )); then
  exit 1
fi
echo "GitHub发布检查通过。"
