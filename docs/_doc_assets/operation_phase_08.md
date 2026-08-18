# phase

## R20. 目标锁定、IK 和 plan-only 验收

把一个正方体放在机械臂容易到达、画面中央的桌面区域。

### R20.1 锁定目标

```bash
ros2 service call \
  /foam_grasp/clear_latched_target \
  std_srvs/srv/Trigger \
  "{}"

ros2 service call \
  /foam_grasp/latch_cube \
  std_srvs/srv/Trigger \
  "{}"
```

查看：

```bash
ros2 topic echo /foam_grasp/target_point_base_latched --once
ros2 topic echo /foam_grasp/latched_target_class --once
```

响应应包含样本数和 spread。默认锁定窗口 1.5 秒、最少 15 个样本、最大扩散
10 mm。

### R20.2 抓取位姿

```bash
ros2 topic echo /foam_grasp/cube_pregrasp_pose --once
ros2 topic echo /foam_grasp/cube_grasp_pose --once
ros2 topic echo /foam_grasp/cube_lift_pose --once
```

位姿 frame 应为 `base_link`。

### R20.3 IK 检查

```bash
ros2 run foam_grasp grasp_ik_check
```

必须看到 PREGRASP、GRASP、LIFT 都可达。该命令不会执行轨迹。

### R20.4 完整路径检查

```bash
ros2 run foam_grasp grasp_plan_check
```

必须看到：

```text
TABLE COLLISION OBJECT: APPLIED
CURRENT_TO_PREGRASP: PLAN SUCCESS
PREGRASP_TO_GRASP: PLAN SUCCESS
GRASP_TO_LIFT: PLAN SUCCESS
```

### R20.5 笛卡尔接近/抬升检查

```bash
ros2 run foam_grasp grasp_cartesian_check
```

必须看到接近和抬升均为 100%。这些检查只向 RViz 发布规划轨迹，不向 Piper
发送动作。

### R20.6 整体流程 plan-only

```bash
ros2 run foam_grasp object_grasp_sequence \
  --target-class cube
```

程序会搜索多姿态、多高度候选，做 IK、碰撞、OMPL 和笛卡尔检查，但没有
`--execute` 时不会移动真机。


## R21. 第一次真实正方体夹取

首次夹取条件：

- 机械臂已在观察姿态；
- 正方体位于画面中央和中近距离；
- 同类只放一个；
- 工作空间无人；
- 急停可触及；
- `/joint_states` 发布者为 0；
- `allow_trajectory_execution=False`；
- R20 全部通过。

使用项目统一入口：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
./启动.sh cube
```

流程自动完成：

```text
观察姿态
→ 等待视觉稳定
→ 清除旧锁定
→ 多帧锁定 cube
→ 搜索安全抓取候选
→ IK/碰撞/完整规划
→ 高位张开至 70 mm
→ CURRENT→PREGRASP
→ 垂直接近
→ 闭合命令 40 mm
→ 根据实际开口确认夹持
→ 垂直抬升
```

首次运行时盯住机械臂和终端。程序的安全拒绝不是“软件坏了”，而是说明至少一项
输入、状态或规划不满足当前放行条件。不要直接删除阈值。


## R22. 实现圆柱体和球体夹取

圆柱体：

```bash
./启动.sh cylinder
```

球体：

```bash
./启动.sh sphere
```

也可交互：

```bash
./启动.sh
```

输入：

```text
1 / cube / 正方体
2 / cylinder / 圆柱体
3 / sphere / 球体
```

默认夹爪配置：

| 类别 | 几何尺寸 | 预张开 | 闭合命令 | 最小夹持余量 |
|---|---|---:|---:|---:|
| cube | 边长 50 mm | 70 mm | 40 mm | 5 mm |
| cylinder | Ø70 × 70 mm | 90 mm | 55 mm | 4 mm |
| sphere | Ø60 mm | 70 mm | 45 mm | 5 mm |

圆柱体会在高位先张开到 90 mm，避免边下降边张开压到目标。闭合后的实际开口
必须比闭合命令大于最小余量，才认为夹爪中存在物体并允许抬升。

