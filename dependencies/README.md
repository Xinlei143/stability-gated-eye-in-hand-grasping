# Dependency lock files

在已经正常工作的Ubuntu电脑运行：

```bash
./scripts/capture_environment.sh
```

会生成：

```text
orbbec.repos
piper.repos
moveit.repos（当前使用源码MoveIt时）
python-system.freeze.txt
ubuntu-packages.txt
nvidia-smi.txt
```

其中 `.repos` 应提交到GitHub；其他三个是当前机器的私有诊断快照，只随
项目迁移包保存，已被 `.gitignore` 排除。

`.repos` 使用 `vcs export --exact`，保存具体Git提交而不是随时间变化的分支名。新电脑可运行 `scripts/import_vendor.sh` 重建相同源码。

`capture_environment.sh`会优先从本项目的 `workspaces/*_ws/src` 导出；只在首次
旧目录迁移时才回退到 `~/orbbec_legacy_ws` 和 `~/ROS_button`。每次升级
第三方驱动后都应重新运行并将 `.repos` 与自研源码一起备份。

Orbbec DC1继续使用官方 legacy `main` 分支。Piper和MoveIt不要在部署时盲目切到最新版，应优先使用当前电脑导出的精确提交。

Gazebo grasp stabilization is a separate pinned import. Build it with:

```bash
bash scripts/setup_gazebo_grasp_plugin.sh
source scripts/source_env.sh
```

The exact upstream URL and commit are in `gazebo_grasp_plugin.repos`. The
standalone CMake adaptation is intentionally kept as a reviewable patch under
`patches/gazebo_grasp_plugin/`; generated source/build/install trees stay under
`.external/` and are ignored by Git.
