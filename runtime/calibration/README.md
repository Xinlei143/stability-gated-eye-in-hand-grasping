# Runtime calibration

正式部署时运行：

```bash
./scripts/fetch_runtime_assets.sh
```

眼在手上标定文件最终位于：

```text
handeye_eye_in_hand.json
```

JSON必须包含：

```json
{
  "position": [0.0, 0.0, 0.0],
  "orientation": [0.0, 0.0, 0.0, 1.0]
}
```

示例数字不是有效标定。标定文件不进入普通 Git 历史，而是作为
版本化 GitHub Release 资产发布。更换相机、支架、末端安装关系后
必须更换 `rig_id` 并重新标定。

当前 `piper-dabai-dc1-rig-01` 的 v1.0.0 标定指纹：

```text
bytes: 397
sha256: 570acb56f9bf06fbca8ca4e8de04c103fb4198ad64b65900aab256f179b8c65f
```

模型与标定会被打包进同一个不可变 Release 资产：

```bash
./scripts/prepare_runtime_release.sh --upload
```
