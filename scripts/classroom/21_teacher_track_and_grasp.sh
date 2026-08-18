#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"

target_class="$(normalize_target_class "${1:-}")" || {
  echo "用法：$0 cube|cylinder|sphere [最长跟踪秒数，默认120]" >&2
  exit 2
}
target_name="$(target_name_zh "${target_class}")"
track_timeout="${2:-120}"

if [[ ! "${track_timeout}" =~ ^[0-9]+$ ]] \
    || (( track_timeout < 60 || track_timeout > 600 )); then
  die "最长跟踪秒数必须为60到600之间的整数。"
fi

case "${target_class}" in
  cube)
    preopen_mm=70
    close_mm=40
    minimum_grip_margin_mm=5
    cylinder_chord_offset_mm=18
    ;;
  cylinder)
    preopen_mm=90
    close_mm=55
    minimum_grip_margin_mm=4
    cylinder_chord_offset_mm=0
    ;;
  sphere)
    preopen_mm=70
    close_mm=45
    minimum_grip_margin_mm=5
    cylinder_chord_offset_mm=18
    ;;
esac

require_interactive_terminal
source_complete_stack
require_nodes \
  /camera/camera \
  /foam_segmentation \
  /foam_depth_fusion \
  /foam_camera_to_base \
  /foam_target_latch \
  /foam_grasp_pose_preview \
  /piper_ctrl_single_node \
  /move_group
require_command_path_idle
require_moveit_plan_only

echo "===== ${target_name}动态跟踪 + 稳定5秒自动抓取 ====="
echo "最长等待：${track_timeout}秒；未满足稳定条件会退出且不抓取。"
echo "稳定条件：目标三维位置连续5秒变化不超过8mm，"
echo "          目标靠近画面中心、位于抓取安全区，机械臂已停止追赶。"
echo
echo "[ ] 工作空间无人，硬件急停可触及"
echo "[ ] 夹爪为空，桌面和运动范围内没有障碍物"
echo "[ ] 画面中只有一个待抓取的${target_name}"
echo "[ ] 分割叠加图类别确认为 ${target_class}"
echo "[ ] 目标当前在画面中心附近，深度与手眼标定节点正常"
echo "[ ] 线缆足够松，允许机械臂左右约46度并向外伸展15cm"
echo "[ ] 目标停止后，任何人都不会再触碰目标或机械臂"
echo
read -r -p "逐项确认后输入 SAFE_TRACK_GRASP：" safety_confirmation
[[ "${safety_confirmation}" == "SAFE_TRACK_GRASP" ]] \
  || die "安全确认不完整，已取消。"

required_confirmation="EXECUTE_TRACK_GRASP_${target_class^^}"
read -r -p "将真实跟踪并抓取${target_name}。输入 ${required_confirmation}：" \
  execute_confirmation
[[ "${execute_confirmation}" == "${required_confirmation}" ]] \
  || die "执行确认不匹配，已取消。"

echo "先检查${target_name}是否被正确识别；本步骤不移动机械臂。"
ros2 run foam_grasp target_center \
  --target-class "${target_class}" \
  --inspect-only

echo
echo "开始大范围动态跟踪。目标连续稳定5秒后，节点会保持当前位置并退出。"
ros2 run foam_grasp target_center \
  --target-class "${target_class}" \
  --workspace-follow \
  --follow-seconds "${track_timeout}" \
  --stop-when-stable-seconds 5 \
  --stable-position-spread-m 0.008 \
  --stable-center-error-pixels 30 \
  --stable-vertical-center-error-pixels 80 \
  --stable-joint-error-rad 0.030 \
  --execute \
  --confirm AUTO_CENTER_TARGET \
  --tolerance-pixels 20 \
  --sample-count 3 \
  --mask-timeout 2.0 \
  --servo-rate-hz 20 \
  --servo-max-speed-rad-s 0.20 \
  --servo-max-accel-rad-s2 0.80 \
  --servo-gain-per-second 2.0 \
  --servo-command-lead-rad 0.040 \
  --servo-target-timeout 0.30 \
  --servo-status-hz 2 \
  --workspace-max-angle-rad 0.80 \
  --workspace-inward-m 0.01 \
  --workspace-outward-m 0.15 \
  --workspace-ik-rate-hz 8 \
  --workspace-target-timeout 0.50 \
  --workspace-max-target-jump-m 0.12 \
  --workspace-filter-alpha 0.30 \
  --workspace-angular-deadband-rad 0.025 \
  --workspace-radial-deadband-m 0.008 \
  --workspace-ik-timeout 0.30 \
  --speed-percent 15 \
  --effort 0.5 \
  --tracking-limit 0.15

sleep 1
require_command_path_idle

echo
echo "稳定条件已满足。重新锁定最新目标并执行完整抓取plan-only；"
echo "本步骤不创建命令发布者，不移动机械臂。"
ros2 run foam_grasp object_grasp_sequence \
  --target-class "${target_class}" \
  --auto-latch \
  --slowdown 2.0 \
  --speed-percent 10 \
  --cartesian-joint-rate 0.08 \
  --preopen-opening-mm "${preopen_mm}" \
  --close-opening-mm "${close_mm}" \
  --minimum-grip-margin-mm "${minimum_grip_margin_mm}" \
  --cylinder-chord-offset-mm "${cylinder_chord_offset_mm}"

sleep 1
require_command_path_idle

echo
echo "最新目标的完整抓取规划验证通过。将再次锁定目标、重新规划并真实抓取。"
echo "倒计时期间可按Ctrl+C取消；执行期间全程看守急停。"
exec ros2 run foam_grasp object_grasp_sequence \
  --target-class "${target_class}" \
  --execute \
  --auto \
  --auto-latch \
  --auto-pause 1.0 \
  --countdown-seconds 5 \
  --slowdown 2.0 \
  --speed-percent 10 \
  --cartesian-joint-rate 0.08 \
  --preopen-opening-mm "${preopen_mm}" \
  --close-opening-mm "${close_mm}" \
  --minimum-grip-margin-mm "${minimum_grip_margin_mm}" \
  --cylinder-chord-offset-mm "${cylinder_chord_offset_mm}" \
  --confirm AUTO_FULL_OBJECT_GRASP
