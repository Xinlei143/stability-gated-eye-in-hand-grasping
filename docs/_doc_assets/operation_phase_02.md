# phase

## R5. 创建学生工程、工作空间和 ROS 2 包

正式工程不要放在 `~/Downloads`。从空目录建立可迁移的工程骨架：

```bash
mkdir -p ~/robot_projects/foam_grasp_project
cd ~/robot_projects/foam_grasp_project

mkdir -p \
  config data dependencies docs requirements scripts training \
  runtime/models runtime/calibration venvs \
  workspaces/orbbec_ws/src \
  workspaces/piper_ws/src \
  workspaces/moveit_ws/src \
  workspaces/app_ws/src

git init
touch README.md .gitignore
```

建议立即编写 `.gitignore`，至少忽略：

```gitignore
**/build/
**/install/
**/log/
venvs/
data/
runtime/models/*.pth
runtime/calibration/*.json
__pycache__/
*.pyc
```

### R5.1 创建自研 ament_python 包

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_projects/foam_grasp_project/workspaces/app_ws/src

ros2 pkg create foam_grasp \
  --build-type ament_python \
  --license Apache-2.0 \
  --dependencies \
    rclpy sensor_msgs geometry_msgs visualization_msgs \
    std_msgs std_srvs cv_bridge tf2_ros moveit_msgs

cd foam_grasp
mkdir -p launch config
```

检查：

```bash
find ~/robot_projects/foam_grasp_project/workspaces/app_ws/src/foam_grasp \
  -maxdepth 2 -type f | sort
```

### R5.2 学生需要实现的程序

在 `foam_grasp/foam_grasp/` 中逐个实现下列模块，并在 `setup.py` 的
`console_scripts` 中注册入口：

| 模块 | 最小职责 | 关键输出 |
|---|---|---|
| `foam_segmentation_node.py` | RGB 图像推理、发布类别 mask 和 overlay | `/foam_segmentation/mask` |
| `foam_depth_fusion_node.py` | mask 与注册深度融合、鲁棒估计三维中心 | `/foam_grasp/*_point` |
| `foam_camera_to_base_node.py` | 手眼变换和末端位姿组合 | `/foam_grasp/*_point_base` |
| `foam_target_latch_node.py` | 多帧稳定性检查与目标锁存 | `/foam_grasp/target_point_base_latched` |
| `foam_grasp_pose_preview_node.py` | 依据类别尺寸生成 pregrasp/grasp/lift | `PoseStamped` 三个位姿 |
| `foam_move_to_observe.py` | 安全规划并到达观察姿态 | 只在显式执行时发命令 |
| `foam_cube_grasp_sequence.py` | 多姿态搜索、规划、夹爪和状态机 | 三类物体完整夹取 |
| `piper_gripper_safe_test.py` | 夹爪独立预检与限幅测试 | 反馈开口和力度 |

还应实现只规划、不执行的 IK、整段路径和笛卡尔路径检查工具，以及统一的
`launch/`、参数 YAML、环境加载和启动脚本。文件名可以不同，但接口、坐标系、
单位和安全行为必须一致。

Word 手册会紧接本节插入“阶段 2B：在首次构建前创建全部最终工程文件”，完整
收录当前已验证版本的全部自研源文件。教学时可以把这些代码页作为教师参考答案
单独保管，先让学生按本节接口独立实现；完成阶段验收后，再逐文件对照。代码页
不包含第三方 SDK、二进制模型或某台设备的标定结果。

### R5.3 首先冻结接口，不要一开始写完整自动抓取

建议先把话题契约写进 `docs/INTERFACES.md`：

```text
/camera/color/image_raw                  sensor_msgs/msg/Image
/camera/depth/image_raw                  sensor_msgs/msg/Image
/camera/depth/camera_info                sensor_msgs/msg/CameraInfo
/foam_segmentation/mask                  sensor_msgs/msg/Image
/foam_grasp/cube_point                   geometry_msgs/msg/PointStamped
/foam_grasp/cylinder_point               geometry_msgs/msg/PointStamped
/foam_grasp/sphere_point                 geometry_msgs/msg/PointStamped
/foam_grasp/*_point_base                 geometry_msgs/msg/PointStamped
/foam_grasp/target_point_base_latched    geometry_msgs/msg/PointStamped
/joint_states_single                     sensor_msgs/msg/JointState
/end_pose                                geometry_msgs/msg/Pose
```

所有长度统一用米，关节角统一用弧度，图像 mask 的像素值固定为：

```text
0=background, 1=cube, 2=cylinder, 3=sphere
```

### R5.4 推荐实现里程碑

1. 只启动相机，验收 RGB、深度、内参和点云。
2. 离线读取一张图片完成分割，再把模型封装成 ROS 2 节点。
3. 用注册深度把 mask 转为相机坐标三维点。
4. 完成手眼标定后，把目标转换到 `base_link`。
5. 实现目标锁存与三个抓取位姿，但先只在 RViz 显示。
6. 调用 MoveIt 做 IK、碰撞和笛卡尔 plan-only 检查。
7. 独立测试观察姿态和夹爪，最后才实现自动状态机。
8. 正方体成功后再为圆柱体、球体增加尺寸和夹爪参数。

每完成一个里程碑都提交 Git：

```bash
cd ~/robot_projects/foam_grasp_project
git status
git add README.md docs workspaces/app_ws/src/foam_grasp
git commit -m "完成第一个工程骨架和接口定义"
```

部署包恢复和已有 GitHub 成品仓库克隆不属于这条教学主线；它们只在 R30、R31
作为项目完成后的迁移与发布方式介绍。

