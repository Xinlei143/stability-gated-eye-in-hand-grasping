# v1.0.0 发布清单

只有以下项目全部完成，才能把当前版本称为“可一键部署”。

## 1. 已验证电脑

- [ ] 三类目标 `cube / cylinder / sphere` 均通过真机测试；
- [ ] `runtime/models/best_model.pth` 是最终模型；
- [ ] `runtime/calibration/handeye_eye_in_hand.json` 属于
      `piper-dabai-dc1-rig-01`；
- [ ] Orbbec、Piper、MoveIt 工作空间均无本地未记录修改；
- [ ] `./scripts/validate_project.sh` 通过；
- [ ] `./scripts/project_status.sh` 通过；
- [ ] `./scripts/post_install_smoke.sh` 通过。

## 2. Git 源码发布

- [ ] `./scripts/check_github_ready.sh` 没有 ERROR；
- [ ] Git 中没有模型、标定、数据集、venv、build、install 或 log；
- [ ] GitHub 仓库中没有 Token、私钥、密码或个人绝对路径；
- [ ] `main` 已推送；
- [ ] `v1.0.0` tag 指向最终提交并已推送。

## 3. 运行资源发布

- [ ] 在最终 Ubuntu 电脑执行
      `./scripts/prepare_runtime_release.sh --upload`；
- [ ] Release 同时包含 `.tar.gz` 与 `.sha256`；
- [ ] Release tag 与 `config/runtime-release.env` 完全一致；
- [ ] `./scripts/check_release_online.sh` 通过；
- [ ] 已发布版本没有被覆盖；内容变化时提升版本号。

## 4. 全新电脑冷部署

- [ ] 从 GitHub `v1.0.0` tag 克隆，而不是复制旧 build/install；
- [ ] 执行 `./install.sh` 完成；
- [ ] 按提示重启并拔插 DaBai DC1；
- [ ] `can0` 持续收到 Piper 帧；
- [ ] `./scripts/check_system.sh` 通过；
- [ ] plan-only、低速真机与急停检查完成。

安装器不会安装或修复 NVIDIA 内核驱动，也不能自动完成重启、相机拔插和
USB-CAN/机械臂接线。这些属于部署前后必须人工完成的硬件步骤。
