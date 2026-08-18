#!/usr/bin/env bash

CLASSROOM_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${CLASSROOM_SCRIPT_DIR}/../.." && pwd)"

# shellcheck disable=SC1091
source "${PROJECT_ROOT}/scripts/_common.sh"

CLASSROOM_STATE_DIR="${XDG_RUNTIME_DIR:-/tmp}/foam_grasp_classroom_${UID}"
CLASSROOM_PID_FILE="${CLASSROOM_STATE_DIR}/launch.pid"
CLASSROOM_MODE_FILE="${CLASSROOM_STATE_DIR}/launch.mode"
CLASSROOM_LOG_FILE="${CLASSROOM_STATE_DIR}/launch.log"
CLASSROOM_OBSERVE_PLAN_FILE="${CLASSROOM_STATE_DIR}/observe_plan.ok"
CLASSROOM_GRASP_PLAN_PREFIX="${CLASSROOM_STATE_DIR}/grasp_plan"
CLASSROOM_PLAN_MAX_AGE_SECONDS="${CLASSROOM_PLAN_MAX_AGE_SECONDS:-1800}"
CLASSROOM_CONFLICTING_NODES=(
  /camera/camera
  /foam_segmentation
  /foam_depth_fusion
  /foam_camera_to_base
  /foam_target_latch
  /foam_grasp_pose_preview
  /piper_ctrl_single_node
  /move_group
)

mkdir -p "${CLASSROOM_STATE_DIR}"

die() {
  echo "错误：$*" >&2
  exit 1
}

require_interactive_terminal() {
  if [[ ! -t 0 || ! -t 1 ]]; then
    die "该教师操作必须在交互式终端中运行。"
  fi
}

normalize_target_class() {
  local selection="${1:-}"
  case "${selection}" in
    cube|Cube|CUBE|1|方块|正方体)
      printf 'cube\n'
      ;;
    cylinder|Cylinder|CYLINDER|2|圆柱|圆柱体)
      printf 'cylinder\n'
      ;;
    sphere|Sphere|SPHERE|3|球|球体)
      printf 'sphere\n'
      ;;
    *)
      return 1
      ;;
  esac
}

target_name_zh() {
  case "$1" in
    cube) printf '正方体\n' ;;
    cylinder) printf '圆柱体\n' ;;
    sphere) printf '球体\n' ;;
    *) return 1 ;;
  esac
}

managed_launch_pid() {
  local pid
  [[ -f "${CLASSROOM_PID_FILE}" ]] || return 1
  read -r pid < "${CLASSROOM_PID_FILE}"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  if ! kill -0 "${pid}" 2>/dev/null \
      && ! kill -0 -- "-${pid}" 2>/dev/null; then
    return 1
  fi
  printf '%s\n' "${pid}"
}

managed_launch_mode() {
  [[ -f "${CLASSROOM_MODE_FILE}" ]] || return 1
  tr -d '[:space:]' < "${CLASSROOM_MODE_FILE}"
}

clear_plan_markers() {
  rm -f \
    "${CLASSROOM_OBSERVE_PLAN_FILE}" \
    "${CLASSROOM_GRASP_PLAN_PREFIX}_"*.ok
}

ensure_no_managed_launch() {
  local pid
  if pid="$(managed_launch_pid)"; then
    die "已有课堂系统在运行（PID=${pid}，模式=$(managed_launch_mode || printf 'unknown')）。请先运行99_stop_system.sh。"
  fi
  rm -f "${CLASSROOM_PID_FILE}" "${CLASSROOM_MODE_FILE}"
  clear_plan_markers
}

conflicting_ros_nodes() {
  local node_list
  local node
  local found=()

  if ! node_list="$(timeout 8 ros2 node list 2>/dev/null)"; then
    echo "无法读取ROS节点图，不能确认启动环境为空。" >&2
    return 2
  fi
  node_list="$(sort -u <<<"${node_list}")"
  for node in "${CLASSROOM_CONFLICTING_NODES[@]}"; do
    if grep -qx -- "${node}" <<<"${node_list}"; then
      found+=("${node}")
    fi
  done
  if (( ${#found[@]} > 0 )); then
    printf '%s\n' "${found[@]}"
    return 0
  fi
  return 1
}

require_clean_ros_graph_before_start() {
  local existing
  local status

  if existing="$(conflicting_ros_nodes)"; then
    status=0
  else
    status=$?
  fi
  if (( status == 2 )); then
    die "无法检查现有ROS节点。请教师检查ROS环境后再启动。"
  fi
  if (( status == 0 )); then
    echo "检测到已存在的项目ROS节点：" >&2
    sed 's/^/  - /' <<<"${existing}" >&2
    die "拒绝重复启动。请在原启动终端按Ctrl+C有序停止；不要使用大范围pkill。"
  fi
}

start_managed_launch() {
  local mode="$1"
  shift

  ensure_no_managed_launch
  require_clean_ros_graph_before_start
  command -v setsid >/dev/null 2>&1 \
    || die "缺少setsid，无法安全管理ROS进程组。"
  : > "${CLASSROOM_LOG_FILE}"
  # A dedicated session/process group lets the stop script signal ros2 launch
  # and every child node without touching the SSH shell or unrelated ROS work.
  nohup setsid "$@" > "${CLASSROOM_LOG_FILE}" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" > "${CLASSROOM_PID_FILE}"
  printf '%s\n' "${mode}" > "${CLASSROOM_MODE_FILE}"
  sleep 1

  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "启动失败，日志如下：" >&2
    tail -n 80 "${CLASSROOM_LOG_FILE}" >&2 || true
    rm -f "${CLASSROOM_PID_FILE}" "${CLASSROOM_MODE_FILE}"
    return 1
  fi

  echo "课堂系统已在后台启动：mode=${mode}, PID=${pid}"
  echo "运行日志：${CLASSROOM_LOG_FILE}"
}

stop_managed_launch() {
  local pid
  if ! pid="$(managed_launch_pid)"; then
    echo "没有由课堂脚本启动的系统进程。"
    rm -f "${CLASSROOM_PID_FILE}" "${CLASSROOM_MODE_FILE}"
    clear_plan_markers
    return 0
  fi

  echo "正在向课堂系统进程组 PGID=${pid} 发送SIGINT，请等待ROS节点有序退出。"
  kill -INT -- "-${pid}"
  local attempt
  for attempt in {1..15}; do
    if ! kill -0 -- "-${pid}" 2>/dev/null; then
      rm -f "${CLASSROOM_PID_FILE}" "${CLASSROOM_MODE_FILE}"
      clear_plan_markers
      echo "课堂系统已停止。"
      return 0
    fi
    sleep 1
  done

  echo "SIGINT后进程仍未退出，发送SIGTERM。" >&2
  kill -TERM -- "-${pid}"
  for attempt in {1..5}; do
    if ! kill -0 -- "-${pid}" 2>/dev/null; then
      rm -f "${CLASSROOM_PID_FILE}" "${CLASSROOM_MODE_FILE}"
      clear_plan_markers
      echo "课堂系统已停止。"
      return 0
    fi
    sleep 1
  done

  die "课堂系统仍在运行。不要继续操作机械臂，请教师检查 ${CLASSROOM_LOG_FILE}。"
}

wait_for_nodes() {
  local timeout_seconds="$1"
  shift
  local required_nodes=("$@")
  local deadline=$((SECONDS + timeout_seconds))
  local node_list
  local missing_nodes
  local node

  while true; do
    node_list="$(ros2 node list 2>/dev/null | sort -u || true)"
    missing_nodes=()
    for node in "${required_nodes[@]}"; do
      if ! grep -qx -- "${node}" <<<"${node_list}"; then
        missing_nodes+=("${node}")
      fi
    done
    if (( ${#missing_nodes[@]} == 0 )); then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "等待超时，仍缺少节点：${missing_nodes[*]}" >&2
      return 1
    fi
    sleep 1
  done
}

require_nodes() {
  local node_list
  local node
  node_list="$(ros2 node list 2>/dev/null | sort -u || true)"
  for node in "$@"; do
    if ! grep -qx -- "${node}" <<<"${node_list}"; then
      die "缺少ROS节点 ${node}。请先启动对应的课堂系统。"
    fi
  done
}

require_command_path_idle() {
  local publisher_count
  publisher_count="$(
    { ros2 topic info /joint_states 2>/dev/null || true; } \
      | awk '/Publisher count:/ {print $3; exit}'
  )"
  if [[ "${publisher_count:-unknown}" != "0" ]]; then
    echo "/joint_states Publisher count=${publisher_count:-unknown}，存在命令发布者，禁止继续。" >&2
    return 1
  fi
}

require_moveit_plan_only() {
  local execution_setting
  execution_setting="$(
    ros2 param get /move_group allow_trajectory_execution 2>/dev/null || true
  )"
  if [[ "${execution_setting}" != *"False"* ]]; then
    echo "MoveIt allow_trajectory_execution不是False，禁止继续。" >&2
    return 1
  fi
}

require_planning_mode() {
  local pid
  local mode
  pid="$(managed_launch_pid)" || die "课堂规划系统没有运行。"
  mode="$(managed_launch_mode || true)"
  [[ "${mode}" == "planning" ]] || die "当前是${mode:-unknown}模式，不是planning模式。"
  printf '%s\n' "${pid}"
}

record_plan_marker() {
  local marker_file="$1"
  local pid
  pid="$(require_planning_mode)"
  printf '%s %s\n' "${pid}" "$(date +%s)" > "${marker_file}"
}

require_recent_plan_marker() {
  local marker_file="$1"
  local label="$2"
  local current_pid
  local marker_pid
  local marker_time
  local now

  current_pid="$(require_planning_mode)"
  [[ -f "${marker_file}" ]] || die "缺少${label}成功记录，请先完成对应的plan-only脚本。"
  read -r marker_pid marker_time < "${marker_file}"
  [[ "${marker_pid}" == "${current_pid}" ]] || die "${label}记录不属于当前课堂系统，请重新规划。"
  [[ "${marker_time}" =~ ^[0-9]+$ ]] || die "${label}记录格式无效，请重新规划。"
  now="$(date +%s)"
  if (( now - marker_time > CLASSROOM_PLAN_MAX_AGE_SECONDS )); then
    die "${label}记录已超过${CLASSROOM_PLAN_MAX_AGE_SECONDS}秒，请重新规划。"
  fi
}

show_recent_launch_log() {
  if [[ -f "${CLASSROOM_LOG_FILE}" ]]; then
    tail -n "${1:-80}" "${CLASSROOM_LOG_FILE}"
  else
    echo "尚无课堂系统日志。"
  fi
}
