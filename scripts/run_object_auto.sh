#!/usr/bin/env bash
set -eo pipefail

# Non-interactive automatic grasp runner. The interactive entry point is
# scripts/grasp.sh (or the project-root 启动.sh wrapper).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
source_complete_stack

TARGET_CLASS="${1:-}"
case "${TARGET_CLASS}" in
  cube)
    TARGET_NAME="正方体"
    PREOPEN_MM=70
    CLOSE_MM=40
    MINIMUM_GRIP_MARGIN_MM=5
    CYLINDER_CHORD_OFFSET_MM=18
    ;;
  cylinder)
    TARGET_NAME="圆柱体"
    PREOPEN_MM=90
    CLOSE_MM=55
    MINIMUM_GRIP_MARGIN_MM=4
    CYLINDER_CHORD_OFFSET_MM=0
    ;;
  sphere)
    TARGET_NAME="球体"
    PREOPEN_MM=70
    CLOSE_MM=45
    MINIMUM_GRIP_MARGIN_MM=5
    CYLINDER_CHORD_OFFSET_MM=18
    ;;
  *)
    echo "用法：$0 cube|cylinder|sphere" >&2
    exit 2
    ;;
esac

required_nodes=(
  /camera/camera
  /foam_segmentation
  /foam_depth_fusion
  /foam_camera_to_base
  /foam_target_latch
  /foam_grasp_pose_preview
  /piper_ctrl_single_node
  /move_group
)

deadline=$((SECONDS + 40))
while true; do
  node_list="$(ros2 node list 2>/dev/null | sort -u || true)"
  missing_nodes=()
  for node in "${required_nodes[@]}"; do
    if ! grep -qx -- "${node}" <<<"${node_list}"; then
      missing_nodes+=("${node}")
    fi
  done
  if (( ${#missing_nodes[@]} == 0 )); then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "自动流程拒绝：40秒后仍缺少节点 ${missing_nodes[*]}" >&2
    exit 1
  fi
  echo "等待系统节点：${missing_nodes[*]}"
  sleep 1
done

publisher_count="$({ ros2 topic info /joint_states 2>/dev/null || true; } \
  | awk '/Publisher count:/ {print $3; exit}')"
if [[ "${publisher_count:-unknown}" != "0" ]]; then
  echo "自动流程拒绝：/joint_states Publisher count=${publisher_count:-unknown}" >&2
  exit 1
fi

execution_setting="$(ros2 param get /move_group allow_trajectory_execution 2>/dev/null || true)"
if [[ "${execution_setting}" != *"False"* ]]; then
  echo "自动流程拒绝：MoveIt allow_trajectory_execution不是False" >&2
  exit 1
fi

echo "系统接口检查通过，即将自动锁定并抓取${TARGET_NAME}（${TARGET_CLASS}）。"
echo "请确保夹爪为空、目标静止、工作空间无人、硬件急停可触及。"
echo "夹爪配置：预张开${PREOPEN_MM}mm，闭合命令${CLOSE_MM}mm，夹持余量至少${MINIMUM_GRIP_MARGIN_MM}mm。"
if [[ "${TARGET_CLASS}" == "cylinder" ]]; then
  echo "圆柱策略：夹爪在观察高位先张开到${PREOPEN_MM}mm，再沿圆柱中心对称夹取。"
fi
echo "速度配置：观察${OBSERVE_SLOWDOWN}x，PREGRASP${PREGRASP_SLOWDOWN}x，驱动上限${ARM_SPEED_PERCENT}%。"

if [[ "${OBSERVE_BEFORE_GRASP}" == "true" ]]; then
  read -r -a observe_joints <<<"${OBSERVE_JOINTS}"
  if (( ${#observe_joints[@]} != 6 )); then
    echo "自动流程拒绝：OBSERVE_JOINTS必须恰好包含6个关节角" >&2
    exit 1
  fi
  echo "先规划并移动到观察姿态，再锁定${TARGET_NAME}。"
  ros2 run foam_grasp move_to_observe \
    --execute \
    --confirm AUTO_MOVE_TO_OBSERVE \
    --countdown-seconds 0 \
    --slowdown "${OBSERVE_SLOWDOWN}" \
    --speed-percent "${ARM_SPEED_PERCENT}" \
    --observe-joints "${observe_joints[@]}"
  echo "观察姿态已到达，等待视觉和末端反馈稳定。"
  sleep "${OBSERVE_SETTLE_SECONDS}"
fi
if [[ "${CENTER_TARGET_BEFORE_GRASP}" == "true" ]]; then
  echo "根据分割mask小幅调整相机，使${TARGET_NAME}横向居中。"

  ros2 run foam_grasp target_center \
    --target-class "${TARGET_CLASS}" \
    --execute \
    --confirm AUTO_CENTER_TARGET \
    --pan-joint "${CENTER_PAN_JOINT}" \
    --pixels-per-radian "${CENTER_PIXELS_PER_RADIAN}" \
    --gain "${CENTER_GAIN}" \
    --tolerance-pixels "${CENTER_TOLERANCE_PIXELS}" \
    --border-margin-pixels "${CENTER_BORDER_MARGIN_PIXELS}" \
    --minimum-step-rad "${CENTER_MIN_STEP_RAD}" \
    --maximum-step-rad "${CENTER_MAX_STEP_RAD}" \
    --maximum-total-offset-rad "${CENTER_MAX_TOTAL_OFFSET_RAD}" \
    --maximum-iterations "${CENTER_MAX_ITERATIONS}" \
    --sample-count "${CENTER_SAMPLE_COUNT}" \
    --mask-timeout "${CENTER_MASK_TIMEOUT}" \
    --settle-seconds "${CENTER_SETTLE_SECONDS}" \
    --slowdown "${OBSERVE_SLOWDOWN}" \
    --speed-percent "${ARM_SPEED_PERCENT}" \
    --tracking-limit 0.20

  echo "相机居中完成；清空移动前和移动中的旧目标样本。"

  ros2 service call \
    /foam_grasp/clear_latched_target \
    std_srvs/srv/Trigger "{}"

  sleep "${CENTER_POST_SETTLE_SECONDS}"
fi
exec ros2 run foam_grasp object_grasp_sequence \
  --target-class "${TARGET_CLASS}" \
  --execute \
  --auto \
  --auto-latch \
  --auto-pause "${AUTO_STAGE_PAUSE}" \
  --countdown-seconds 0 \
  --slowdown "${PREGRASP_SLOWDOWN}" \
  --speed-percent "${ARM_SPEED_PERCENT}" \
  --cartesian-joint-rate "${CARTESIAN_JOINT_RATE}" \
  --preopen-opening-mm "${PREOPEN_MM}" \
  --close-opening-mm "${CLOSE_MM}" \
  --minimum-grip-margin-mm "${MINIMUM_GRIP_MARGIN_MM}" \
  --cylinder-chord-offset-mm "${CYLINDER_CHORD_OFFSET_MM}" \
  --confirm AUTO_FULL_OBJECT_GRASP
