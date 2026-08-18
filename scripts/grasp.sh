#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if (( $# > 0 )); then
  selection="$1"
else
  echo "请选择要夹取的目标物体："
  echo "  1) 正方体  cube"
  echo "  2) 圆柱体  cylinder"
  echo "  3) 球体    sphere"
  read -r -p "请输入序号或名称（输入 q 退出）：" selection
fi

# Bash's case matching is exact here, so accepting both Chinese and English
# cannot accidentally select a neighbouring class.
case "${selection}" in
  1|cube|Cube|CUBE|方块|正方体)
    target_class="cube"
    ;;
  2|cylinder|Cylinder|CYLINDER|圆柱|圆柱体)
    target_class="cylinder"
    ;;
  3|sphere|Sphere|SPHERE|球|球体)
    target_class="sphere"
    ;;
  q|Q|quit|QUIT|退出)
    echo "已退出，没有发送机械臂命令。"
    exit 0
    ;;
  *)
    echo "无法识别目标：${selection}" >&2
    echo "请输入 1/2/3、cube/cylinder/sphere 或中文名称。" >&2
    exit 2
    ;;
esac

exec "${SCRIPT_DIR}/run_object_auto.sh" "${target_class}"
