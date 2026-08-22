# 工程文件清单

本清单定义 Foam Grasp 1.0.0 的工程边界。项目分为“Git 源码”“GitHub
Release运行资产”“可重建第三方依赖”和“生成产物”四类。

## 1. 应进入 Git 的工程源码

```text
foam_grasp_project/
├── .github/                         # GitHub PR模板
├── config/
│   └── project.env                  # CAN、观察姿态、速度等机器级配置
│   └── runtime-release.env          # 固定Release版本、资产名和rig_id
├── dependencies/
│   ├── README.md
│   ├── orbbec.repos                 # 在工作Ubuntu上导出后提交
│   ├── piper.repos                  # 在工作Ubuntu上导出后提交
│   └── moveit.repos                 # 仅使用源码MoveIt overlay时存在
├── docs/                            # 架构、部署、运行、训练、排障
├── requirements/
│   └── runtime.txt                  # 项目Python运行时依赖
├── patches/
│   └── piper_ros/                   # 固定上游提交上的最小真机补丁
├── runtime/
│   ├── models/README.md
│   └── calibration/README.md
├── scripts/                         # 安装、构建、启动、抓取、打包
├── install.sh                       # 新电脑一键软件部署
├── training/                        # 采集、标注转换、划分、训练、测试
├── analysis/                        # 只读 benchmark 汇总、配对差异和绘图
├── workspaces/
│   ├── app_ws/src/foam_grasp/       # 真机感知与抓取 ROS 2包
│   ├── app_ws/src/foam_grasp_sim/   # Gazebo、benchmark runner 与 RGB-D 仿真
│   ├── orbbec_ws/src/.gitkeep
│   ├── piper_ws/src/.gitkeep
│   └── moveit_ws/src/.gitkeep
├── .gitignore
├── CONTRIBUTING.md
├── LICENSE
├── README.md
├── SECURITY.md
└── 启动.sh                          # 选择目标并启动自动抓取
```

`workspaces/app_ws/src/foam_grasp` 内的主要文件：

| 文件 | 职责 |
|---|---|
| `launch/system.launch.py` | 启动相机、Piper、安全 MoveIt 和感知节点 |
| `launch/safe_plan_only.launch.py` | 禁止 MoveIt 自行执行，接入真机反馈 |
| `config/runtime.yaml` | 在线感知、工作区和抓取几何参数 |
| `foam_segmentation_node.py` | DeepLabV3-ResNet50 实时分割 |
| `foam_depth_fusion_node.py` | mask 与注册深度融合为相机系 3D 点 |
| `foam_camera_to_base_node.py` | 手眼标定链转换到 `base_link` |
| `foam_target_latch_node.py` | 稳定采样、目标锁定、工作区过滤 |
| `foam_grasp_pose_preview_node.py` | 类别几何、TCP 偏移和三段位姿 |
| `foam_move_to_observe.py` | 规划并移动到观察姿态 |
| `foam_move_to_pregrasp.py` | 规划、轨迹验证和安全执行基础类 |
| `foam_cube_grasp_sequence.py` | 三类别自适应姿态搜索和完整抓取状态机 |
| `foam_grasp_*_check.py` | IK、完整路径、笛卡尔路径只读检查 |
| `piper_gripper_safe_test.py` | 保持六轴不变的夹爪专项测试 |

`workspaces/app_ws/src/foam_grasp_sim` 的 Stage 6/7 接口包括：

| 文件 | 职责 |
|---|---|
| `launch/sim_bringup.launch.py` | 唯一单次 Gazebo trial 入口 |
| `launch/full_pipeline.launch.py` | 外层 RGB-D 感知与仿真组合 |
| `urdf/piper_eye_in_hand_gazebo.xacro` | link6 工具轴上的 RGB-D 相机 fixture |
| `foam_grasp_sim/benchmark_suite.py` | YAML 校验、确定性展开与配对 ID |
| `foam_grasp_sim/experiment_runner.py` | 顺序 campaign、超时、恢复与归档 |

## 2. 不进入普通 Git、由 GitHub Release 提供

| 文件 | 说明 |
|---|---|
| `runtime/models/best_model.pth` | 约 482 MB 的训练权重 |
| `runtime/calibration/handeye_eye_in_hand.json` | 当前眼在手上安装关系 |

两个文件由 `scripts/prepare_runtime_release.sh` 打成同一个版本化资产并
生成 SHA-256 校验。`scripts/fetch_runtime_assets.sh` 下载、校验并原子安装。
标定只适用于 Release 清单中记录的 `rig_id`。

## 3. 可从精确版本清单重建

```text
workspaces/orbbec_ws/src/     # OrbbecSDK_ROS2 legacy
workspaces/piper_ws/src/      # Piper驱动、消息和MoveIt配置
workspaces/moveit_ws/src/     # 可选的源码MoveIt overlay
```

在已工作的 Ubuntu 上运行 `scripts/capture_environment.sh` 生成
`dependencies/*.repos`。新电脑运行 `scripts/import_vendor.sh` 恢复相同提交。
Piper 源码恢复后还会应用 `patches/piper_ros` 中已验证的 KDL 配置补丁；
`scripts/verify_vendor_locks.sh` 同时验证上游提交和补丁结果。

## 4. 永不跨电脑复制的生成产物

```text
workspaces/*_ws/build/
workspaces/*_ws/install/
workspaces/*_ws/log/
venvs/
__pycache__/
```

这些文件含旧电脑绝对路径、ABI 或解释器位置，必须在目标电脑重新生成。

## 5. 单独管理的大数据

```text
data/raw/
data/segmentation_dataset/
data/training_runs/
data/test_results/
```

数据不参与在线抓取，也不进入普通 Git。需要迁移时使用
`scripts/package_data.sh` 单独打包。
