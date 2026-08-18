# 第三方源码补丁

第三方仓库始终由 `dependencies/*.repos` 固定到精确提交。本目录只保存当前
Piper + DaBai DC1 真机配置相对上游提交的最小补丁，避免把完整第三方源码
复制进主仓库。

`scripts/import_vendor.sh` 会先恢复上游源码，再按顺序应用这里的补丁。补丁
应用后生成的 `workspaces/piper_ws/src` 仍属于可重建产物，不进入 Git。

当前补丁：

- `piper_ros/0001-use-kdl-kinematics.patch`：将 Piper 手臂 IK 插件固定为已在
  本机验证的 KDL 配置，移除未使用的 gripper IK 组。
