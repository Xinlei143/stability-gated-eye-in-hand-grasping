# 数据、训练与模型发布

在线抓取只依赖模型和标定，不依赖原始训练数据。数据统一放在项目 `data/`，不要放在 ROS 2 包内。

## 推荐数据结构

```text
data/
├── raw/
│   ├── images/
│   ├── annotations_labelme/
│   └── bags/
├── segmentation_dataset/
│   ├── images/{train,val,test}/
│   └── masks/{train,val,test}/
├── training_runs/
│   └── deeplabv3_resnet50/
└── test_results/
```

mask像素约定：

```text
0 = background
1 = cube
2 = cylinder
3 = sphere
```

## 数据采集与预处理工具

仓库已经包含完整的数据闭环工具：

```text
training/capture_color_images_node.py
training/labelme_to_masks.py
training/make_mask_preview.py
training/split_segmentation_dataset.py
```

完整命令和标注规则见
[完整工程与工作流程](COMPLETE_WORKFLOW.md#7-数据采集闭环)。这些工具不会
发布 Piper 命令；其中只有采集节点需要 ROS 2 相机话题。

## 训练

先加载项目运行环境：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
```

烟雾测试：

```bash
python training/train_foam_segmentation.py \
  --smoke-test --batch-size 2 --workers 2
```

完整训练：

```bash
python training/train_foam_segmentation.py \
  --epochs 40 --batch-size 4 --workers 4
```

可用参数覆盖默认目录：

```bash
python training/train_foam_segmentation.py \
  --dataset-root /path/to/segmentation_dataset \
  --runs-root /path/to/training_runs
```

## 测试

```bash
python training/evaluate_foam_segmentation.py \
  --checkpoint data/training_runs/deeplabv3_resnet50/best_model.pth
```

## 发布模型到运行环境

只有在独立测试集和预览图检查通过后才替换在线模型：

```bash
cp data/training_runs/deeplabv3_resnet50/best_model.pth \
  runtime/models/best_model.pth
sha256sum runtime/models/best_model.pth
```

建议把哈希、数据集版本、训练提交、测试指标和发布日期记录到单独的模型说明文件。替换模型后必须先只观察实时overlay，再进行真实抓取。
