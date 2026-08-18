# phase

## R19. 眼在手上手眼标定

如果相机与法兰的刚性安装完全未变，可使用当前已验证 JSON；否则必须重新标定。
本项目在线节点消费标定结果，但当前仓库不包含一键标定采集器，因此本节给出
严格的数据与验收流程，不把示例数值伪装成可用标定。

### R19.1 坐标方向

工程需要：

```text
T_gripper_camera
p_base = T_base_gripper · T_gripper_camera · p_camera
```

机器人每个姿态记录：

```text
T_base_gripper
```

标定板检测记录：

```text
T_camera_target
```

OpenCV `calibrateHandEye` 的输入是 `gripper2base` 和 `target2cam`，输出
`cam2gripper`，与本项目所需方向一致。

### R19.2 启动相机与 Piper 反馈

可以启动完整系统，也可分别启动相机和 Piper。检查：

```bash
source scripts/source_env.sh
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /end_pose --once
```

### R19.3 采集 20–30 组姿态

标定板固定不动。每次机械臂到位并完全静止后，保存：

- 彩色图；
- 相机内参；
- 标定板在相机坐标系中的位姿；
- `/end_pose`；
- 时间戳。

姿态必须覆盖：

- 明显不同的 roll/pitch/yaw；
- 不同距离；
- 标定板位于画面不同区域；
- 至少 15 组，推荐 20–30 组；
- 另留 5–10 组只做验证。

检查末端位姿：

```bash
ros2 topic echo /end_pose --once
```

检查相机内参：

```bash
ros2 topic echo /camera/color/camera_info --once --field k
```

### R19.4 OpenCV 求解核心

在求解脚本中：

```python
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base,
    t_gripper2base,
    R_target2cam,
    t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI,
)
```

同时比较 Tsai、Park、Horaud 等方法，用保留验证姿态检查闭环一致性。将平移统一
转为米，旋转转为 `xyzw` 四元数，写入：

```text
runtime/calibration/handeye_eye_in_hand.json
```

### R19.5 标定验收

对固定标定板上的同一点，在不同机械臂姿态下计算 base 坐标。原型建议：

```text
验证集平移误差中位数 < 5 mm
最大误差 < 10 mm
```

再让真实桌面目标保持不动、机械臂改变观察姿态，确认
`/foam_grasp/{class}_point_base` 不明显漂移。只有通过后，才继续抓取。


## R10. 放置学生自己生成的模型和手眼标定

### R10.1 模型

教学主线中，`best_model.pth` 必须来自学生在 R23～R27 完成的数据采集、标注、
训练和独立测试。训练程序直接把最佳模型导出到：

```text
~/robot_projects/foam_grasp_project/runtime/models/best_model.pth
```

如果训练输出在其他目录，再复制并记录校验值：

```bash
cd ~/robot_projects/foam_grasp_project
mkdir -p runtime/models
cp <学生训练输出目录>/best_model.pth runtime/models/best_model.pth
ls -lh runtime/models/best_model.pth
sha256sum runtime/models/best_model.pth
```

验证 checkpoint：

```bash
source scripts/source_env.sh 2>/dev/null || true
venvs/runtime/bin/python - <<'PY'
import torch
path = "runtime/models/best_model.pth"
checkpoint = torch.load(path, map_location="cpu", weights_only=False)
print("keys:", sorted(checkpoint))
print("epoch:", checkpoint.get("epoch"))
print("foreground_miou:", checkpoint.get("foreground_miou"))
print("class_names:", checkpoint.get("class_names"))
PY
```

类别顺序必须为：

```text
background, cube, cylinder, sphere
```

### R10.2 标定

R19 完成后，把学生自己的标定程序输出保存为：

```text
~/robot_projects/foam_grasp_project/runtime/calibration/handeye_eye_in_hand.json
```

检查：

```bash
cd ~/robot_projects/foam_grasp_project
python3 -m json.tool \
  runtime/calibration/handeye_eye_in_hand.json
sha256sum runtime/calibration/handeye_eye_in_hand.json
```

JSON 至少应包含：

```json
{
  "position": [0.0, 0.0, 0.0],
  "orientation": [0.0, 0.0, 0.0, 1.0]
}
```

示例中的零值不能用于真机。位置单位为米，四元数顺序为 `x, y, z, w`，变换
方向必须是 `T_gripper_camera`。

不要用其他机械臂的 JSON 通过验收。只要相机、支架、法兰或夹爪安装发生变化，
就必须重新标定，见 R19。

