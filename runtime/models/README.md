# Runtime model

将已通过测试的模型放在：

```text
best_model.pth
```

模型不提交普通 Git 历史。正式部署时从与源码版本一致的
GitHub Release 资产恢复：

```bash
./scripts/fetch_runtime_assets.sh
```

发布前由已验证 Ubuntu 电脑执行：

```bash
./scripts/prepare_runtime_release.sh --upload
```
