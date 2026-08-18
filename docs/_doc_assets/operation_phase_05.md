# phase

## R23. 数据采集：从零建立分割数据集

如果使用现有模型可以跳到 R15。要从零训练，先启动相机：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
ros2 launch orbbec_camera dabai.launch.py \
  depth_registration:=true \
  enable_ir:=false \
  enable_point_cloud:=false
```

另一个终端启动保存节点：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
mkdir -p data/raw/images/session_01

python3 training/capture_color_images_node.py \
  --output-dir data/raw/images/session_01 \
  --topic /camera/color/image_raw \
  --service /foam_dataset/save \
  --prefix frame_
```

第三个终端触发：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh

while true
do
  read -r -p "调整物体并静止1秒后按回车拍照；Ctrl+C退出："
  ros2 service call \
    /foam_dataset/save \
    std_srvs/srv/Trigger \
    "{}"
done
```

每次只在物体、位置、角度、背景、距离、光照或遮挡发生实际变化后拍一张。避免
连续保存大量相似帧。建议初版至少：

```text
每类 150–300 张有效实例
总图 450–900 张
```

历史原型使用 369 张完成闭环，但更换背景或光照后应扩充。

检查数量、尺寸和损坏文件：

```bash
find data/raw/images/session_01 \
  -maxdepth 1 -type f -name '*.png' | wc -l

file data/raw/images/session_01/frame_0000.png

python3 - <<'PY'
from pathlib import Path
from PIL import Image

root = Path("data/raw/images/session_01")
files = sorted(root.glob("*.png"))
sizes = {}
bad = []
for path in files:
    try:
        with Image.open(path) as image:
            image.verify()
        with Image.open(path) as image:
            sizes[image.size] = sizes.get(image.size, 0) + 1
    except Exception as error:
        bad.append((path.name, repr(error)))
print("count:", len(files))
print("sizes:", sizes)
print("bad:", bad)
PY
```


## R24. LabelMe 语义分割标注

使用独立环境：

```bash
cd ~/robot_projects/foam_grasp_project
python3 -m venv venvs/labelme
source venvs/labelme/bin/activate
python -m pip install --upgrade pip
python -m pip install "numpy<2" labelme
```

建立标注目录：

```bash
mkdir -p data/raw/annotations_labelme/session_01
labelme \
  data/raw/images/session_01 \
  --output data/raw/annotations_labelme/session_01
```

标签只能使用：

```text
cube
cylinder
sphere
```

标注规则：

- 使用 polygon 贴合可见轮廓；
- 被遮挡时只标可见区域；
- 只出现一半但仍能确认类别时仍需标注；
- 只剩无法辨认的小碎片时可以不作为训练实例；
- 阴影不属于物体；
- 不把夹爪、桌面或反光边缘标入目标；
- 同一标签始终使用完全相同的英文拼写。

开启自动保存后，关闭 LabelMe 前仍需验证：

```bash
echo "图片数量："
find data/raw/images/session_01 \
  -maxdepth 1 -type f -name '*.png' | wc -l

echo "JSON数量："
find data/raw/annotations_labelme/session_01 \
  -maxdepth 1 -type f -name '*.json' | wc -l

python3 - <<'PY'
from pathlib import Path
import json

root = Path("data/raw/annotations_labelme/session_01")
files = sorted(root.glob("*.json"))
for path in files:
    json.loads(path.read_text(encoding="utf-8"))
print("全部JSON可解析，数量：", len(files))
PY
```

图片与 JSON 必须一一对应。若决定删除一张无法标注的图片，要同时保证后续
输入中没有对应孤立 JSON。


## R25. 生成 mask、预览和数据划分

保持 labelme venv 或使用能导入 Pillow/NumPy 的环境：

```bash
cd ~/robot_projects/foam_grasp_project
source venvs/labelme/bin/activate
```

转换为索引 mask：

```bash
python training/labelme_to_masks.py \
  --images-dir data/raw/images/session_01 \
  --annotations-dir data/raw/annotations_labelme/session_01 \
  --output-dir data/raw/masks/session_01
```

像素值：

```text
0=background
1=cube
2=cylinder
3=sphere
```

检查并生成总览：

```bash
python training/make_mask_preview.py \
  --images-dir data/raw/images/session_01 \
  --masks-dir data/raw/masks/session_01 \
  --output-dir data/raw/previews/session_01 \
  --sample-count 24
```

人工查看：

```text
data/raw/previews/session_01/mask_contact_sheet.jpg
```

数量检查：

```bash
find data/raw/images/session_01 -maxdepth 1 -name '*.png' | wc -l
find data/raw/annotations_labelme/session_01 -maxdepth 1 -name '*.json' | wc -l
find data/raw/masks/session_01 -maxdepth 1 -name '*.png' | wc -l
```

三者必须一致。

划分：

```bash
python training/split_segmentation_dataset.py \
  --images-dir data/raw/images/session_01 \
  --masks-dir data/raw/masks/session_01 \
  --output-dir data/segmentation_dataset
```

若需要确认覆盖旧划分：

```bash
python training/split_segmentation_dataset.py \
  --images-dir data/raw/images/session_01 \
  --masks-dir data/raw/masks/session_01 \
  --output-dir data/segmentation_dataset \
  --force
```

默认比例 80/10/10，脚本搜索 2,000 个随机种子，使各集合类别出现率更接近全局
分布。历史 369 张结果是：

```text
train 295
val 37
test 37
```

不要从同一段几乎相同的视频抽帧后随机拆分到 train/test；应按采集场景分组，
避免数据泄漏。


## R26. 训练 DeepLabV3 + ResNet-50

退出 LabelMe 环境，加载项目 runtime：

```bash
deactivate
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
```

再次确认 GPU：

```bash
python - <<'PY'
import torch
print(torch.__version__)
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "NO GPU")
PY
```

先做烟雾测试：

```bash
python training/train_foam_segmentation.py \
  --dataset-root data/segmentation_dataset \
  --runs-root data/training_runs \
  --smoke-test \
  --batch-size 2 \
  --workers 2
```

烟雾测试的 mIoU 可能接近 0；它只验证数据、模型、GPU、前向、反向和保存链路。

完整训练：

```bash
python training/train_foam_segmentation.py \
  --dataset-root data/segmentation_dataset \
  --runs-root data/training_runs \
  --epochs 40 \
  --batch-size 4 \
  --workers 4 \
  --learning-rate 1e-4 \
  --seed 42
```

输出：

```text
data/training_runs/deeplabv3_resnet50/
├── best_model.pth
├── last_model.pth
└── train.log
```

显存不足时按顺序调整：

1. `--batch-size 2`；
2. `--workers 2`；
3. 关闭 RViz 和其他 GPU 程序；
4. 不要随意改输入尺寸，除非同步验证在线预处理。


## R27. 独立测试并发布模型

测试：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh

python training/evaluate_foam_segmentation.py \
  --dataset-root data/segmentation_dataset \
  --checkpoint \
    data/training_runs/deeplabv3_resnet50/best_model.pth \
  --output-dir \
    data/test_results/deeplabv3_resnet50 \
  --batch-size 4 \
  --workers 4
```

查看：

```text
data/test_results/deeplabv3_resnet50/test_metrics.json
data/test_results/deeplabv3_resnet50/prediction_masks
data/test_results/deeplabv3_resnet50/comparisons
data/test_results/deeplabv3_resnet50/test_contact_sheet.png
```

发布：

```bash
cp \
  data/training_runs/deeplabv3_resnet50/best_model.pth \
  runtime/models/best_model.pth

sha256sum runtime/models/best_model.pth
```

替换模型后重新启动系统，先查看 overlay，再做锁定、IK、plan-only，最后才做
真实抓取。

历史测试结果：

| 指标 | 数值 |
|---|---:|
| Pixel accuracy | 0.998865 |
| Foreground mIoU | 0.973698 |
| Cube IoU | 0.965921 |
| Cylinder IoU | 0.980466 |
| Sphere IoU | 0.974709 |

这些数字不能替代现场泛化检查。

