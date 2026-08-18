# 系统架构

## 工作空间为什么分开

第三方驱动更新频率、构建方式和依赖与本项目不同，因此不应全部塞进一个 `src`。分成四层后：

- Orbbec 驱动可以保持已验证的 legacy `main` 分支，不被系统 apt 包覆盖。
- Piper 驱动和自定义消息单独构建。
- 当前电脑若必须使用源码版 MoveIt，可以保留 `moveit_ws`；其他电脑可使用 apt MoveIt 并让此工作空间为空。
- 自研代码只在 `app_ws`，修改后只需重新构建这一层。

`app_ws` 使用 `venvs/runtime/bin/python` 调用系统已有的 `colcon_core` 构建，使 ROS 2 可执行脚本的 shebang 指向包含 PyTorch、NumPy 1.26 和系统 ROS 包的运行环境。这样无需从PyPI重复下载colcon，也能避免再次出现 `cv_bridge` 与 NumPy 2 不兼容的问题。

运行环境设置 `PYTHONNOUSERSITE=1`：ROS 2 系统包仍通过 `--system-site-packages` 可见，但 `~/.local` 中版本不受控的CUDA/Python包不会混入项目venv。

DaBai DC1 属于 Orbbec 的 legacy/OpenNI 设备，本项目沿用已经验证的 `OrbbecSDK_ROS2 main` 分支；不要自动切换到 `v2-main`。

## ROS 2 功能包

唯一自研包：

```text
workspaces/app_ws/src/foam_grasp
```

主要可执行程序：

| 可执行程序 | ROS节点 | 作用 |
|---|---|---|
| `segmentation_node` | `/foam_segmentation` | 彩色图语义分割 |
| `depth_fusion_node` | `/foam_depth_fusion` | mask与对齐深度融合 |
| `camera_to_base_node` | `/foam_camera_to_base` | camera点转换到base_link |
| `target_latch_node` | `/foam_target_latch` | 多帧稳定采样和锁定 |
| `grasp_pose_preview_node` | `/foam_grasp_pose_preview` | 生成三段抓取位姿 |
| `object_grasp_sequence` | 临时执行节点 | 按所选类别自动规划并抓取目标 |
| `cube_grasp_sequence` | 临时执行节点 | 兼容旧方块命令，使用相同实现 |

检查工具也注册为 ROS 2 可执行程序：

```text
grasp_ik_check
grasp_plan_check
grasp_cartesian_check
move_to_pregrasp
piper_gripper_safe_test
```

## 数据流

```text
DaBai彩色图
  → segmentation_node
  → /foam_segmentation/mask
  → depth_fusion_node + 对齐深度
  → /foam_grasp/{cube,cylinder,sphere}_point（相机光学坐标）
  → camera_to_base_node + /end_pose + 手眼标定
  → /foam_grasp/{cube,cylinder,sphere}_point_base
  → target_latch_node
  → /foam_grasp/target_point_base_latched + latched_target_class
  → grasp_pose_preview_node
  → PREGRASP / GRASP / LIFT
  → IK + 碰撞 + 笛卡尔路径检查
  → Piper /joint_states 命令输入
```

## Launch 文件

### `system.launch.py`

完整冷启动：

- `orbbec_camera/dabai.launch.py`
- `piper/start_single_piper.launch.py`
- `safe_plan_only.launch.py`
- 五个感知与抓取位姿节点

模型和标定路径由 `scripts/start_system.sh` 传入，不写死用户目录。

可用参数：

```text
checkpoint
calibration_file
can_port:=can0
auto_enable:=true
start_camera:=true
start_piper:=true
start_moveit:=true
use_rviz:=true
```

### `safe_plan_only.launch.py`

该 launch：

- 把 MoveIt 和 robot_state_publisher 的 `/joint_states` 输入重映射到真机反馈 `/joint_states_single`。
- 设置 `allow_trajectory_execution=false`。
- 不启动 fake hardware、ros2_control 或 joint_state_publisher。
- Piper 真机仍将 `/joint_states` 作为命令入口。

抓取程序只有在规划、安全检查和互斥发布者检查全部通过后，才临时创建 `/joint_states` 发布者。

## 配置归属

| 配置 | 位置 | 是否跨电脑复用 |
|---|---|---|
| CAN口、ROS版本、RViz | `config/project.env` | 通常可以 |
| 感知/锁定/抓取参数 | `app_ws/.../config/runtime.yaml` | 同一机械结构可复用 |
| 神经网络模型 | `runtime/models/best_model.pth` | 可以 |
| 手眼标定 | `runtime/calibration/*.json` | 仅相同安装关系可复用 |
| 数据集 | `data/` | 可选，不参与在线运行 |
