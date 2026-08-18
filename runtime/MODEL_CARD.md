# Foam object segmentation model card

## 模型

- 架构：DeepLabV3 + ResNet-50；
- 类别：`background / cube / cylinder / sphere`；
- 输入尺寸：`640 × 360`；
- 训练框架：PyTorch 2.5.1 + Torchvision 0.20.1；
- 部署文件：`runtime/models/best_model.pth`；
- 发布方式：与源码 tag 同版本的 GitHub Release 资产。
- v1.0.0 文件大小：`504461281` bytes；
- v1.0.0 SHA-256：
  `1ecbbe59c87f075bcc1f6bb44465038ea7728bc6998beaa36f5c0a9396104a47`。

## 当前测试结果

当前最终实验记录包含 369 张已标注图像，其中训练集 295 张、验证集 37 张、
测试集 37 张。第 40 轮最佳 checkpoint 在测试集上的结果为：

| 指标 | 数值 |
|---|---:|
| Pixel accuracy | 0.998865 |
| Foreground mIoU | 0.973698 |
| All-class mIoU | 0.979978 |
| Cube IoU | 0.965921 |
| Cylinder IoU | 0.980466 |
| Sphere IoU | 0.974709 |

这些结果只代表当前采集环境和测试划分。不同光照、背景、遮挡、相机安装和
泡沫材质可能降低精度；真机抓取前仍需检查实时 overlay、深度和三维坐标。

## 完整性

模型不进入普通 Git 历史。`prepare_runtime_release.sh` 会把模型与当前 rig
手眼标定打包，并写入各自 SHA-256；`fetch_runtime_assets.sh` 下载后会同时
验证外层归档、内部文件、类别顺序、输入尺寸，并把 state dict 严格加载到
DeepLabV3-ResNet50。源码中的 `config/runtime-release.env` 还固定了模型和
标定的预期大小与 SHA-256，避免仅依赖和归档一起下载的校验文件。
