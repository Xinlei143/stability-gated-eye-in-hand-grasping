#!/usr/bin/env python3
"""Create balanced train/validation/test image-mask splits."""

import argparse
import json
import random
import shutil
from pathlib import Path

import numpy as np
from PIL import Image


CLASS_NAMES = {
    1: "cube",
    2: "cylinder",
    3: "sphere",
}


def parse_arguments():
    parser = argparse.ArgumentParser(description="划分语义分割数据集")
    parser.add_argument("--images-dir", type=Path, required=True)
    parser.add_argument("--masks-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--ratios",
        type=float,
        nargs=3,
        default=(0.8, 0.1, 0.1),
        metavar=("TRAIN", "VAL", "TEST"),
    )
    parser.add_argument("--seed-search-count", type=int, default=2000)
    parser.add_argument(
        "--force",
        action="store_true",
        help="覆盖已有输出目录",
    )
    return parser.parse_args()


def class_presence(mask_path):
    with Image.open(mask_path) as mask_file:
        values = np.asarray(mask_file.convert("L"), dtype=np.uint8)
    illegal = np.setdiff1d(np.unique(values), [0, 1, 2, 3])
    if illegal.size:
        raise RuntimeError(
            f"{mask_path.name}包含非法像素：{illegal.tolist()}"
        )
    return {
        class_id for class_id in CLASS_NAMES
        if np.any(values == class_id)
    }


def rate(items, class_id):
    if not items:
        return 0.0
    return sum(class_id in item["classes"] for item in items) / len(items)


def split_score(splits, global_rates):
    score = 0.0
    for split_items in splits.values():
        for class_id in CLASS_NAMES:
            split_rate = rate(split_items, class_id)
            score += abs(split_rate - global_rates[class_id])
            if not any(class_id in item["classes"] for item in split_items):
                score += 10.0
    return score


def main():
    arguments = parse_arguments()
    images_dir = arguments.images_dir.expanduser().resolve()
    masks_dir = arguments.masks_dir.expanduser().resolve()
    output_dir = arguments.output_dir.expanduser().resolve()
    ratios = tuple(arguments.ratios)
    if any(value <= 0.0 for value in ratios):
        raise RuntimeError("三个划分比例必须为正数")
    if abs(sum(ratios) - 1.0) > 1e-6:
        raise RuntimeError("三个划分比例之和必须为1")
    if arguments.seed_search_count < 1:
        raise RuntimeError("--seed-search-count必须至少为1")

    image_paths = sorted(images_dir.glob("*.png"))
    if not image_paths:
        raise RuntimeError(f"没有找到PNG图片：{images_dir}")
    items = []
    for image_path in image_paths:
        mask_path = masks_dir / image_path.name
        if not mask_path.is_file():
            raise RuntimeError(f"缺少mask：{mask_path}")
        items.append(
            {
                "image": image_path,
                "mask": mask_path,
                "classes": class_presence(mask_path),
            }
        )

    total = len(items)
    train_count = round(total * ratios[0])
    val_count = round(total * ratios[1])
    test_count = total - train_count - val_count
    if min(train_count, val_count, test_count) < 1:
        raise RuntimeError("数据量过少，无法创建三个非空集合")

    global_rates = {
        class_id: rate(items, class_id) for class_id in CLASS_NAMES
    }
    best_seed = None
    best_score = float("inf")
    best_splits = None
    for seed in range(arguments.seed_search_count):
        indices = list(range(total))
        random.Random(seed).shuffle(indices)
        train_end = train_count
        val_end = train_end + val_count
        splits = {
            "train": [items[index] for index in indices[:train_end]],
            "val": [items[index] for index in indices[train_end:val_end]],
            "test": [items[index] for index in indices[val_end:]],
        }
        score = split_score(splits, global_rates)
        if score < best_score:
            best_seed = seed
            best_score = score
            best_splits = splits

    if output_dir.exists():
        if not arguments.force:
            raise RuntimeError(
                f"输出目录已存在：{output_dir}；确认后使用--force覆盖"
            )
        shutil.rmtree(output_dir)

    manifest = {
        "seed": best_seed,
        "ratios": {
            "train": ratios[0],
            "val": ratios[1],
            "test": ratios[2],
        },
        "splits": {},
    }
    for split_name, split_items in best_splits.items():
        image_output = output_dir / "images" / split_name
        mask_output = output_dir / "masks" / split_name
        image_output.mkdir(parents=True, exist_ok=True)
        mask_output.mkdir(parents=True, exist_ok=True)
        filenames = []
        for item in split_items:
            shutil.copy2(item["image"], image_output / item["image"].name)
            shutil.copy2(item["mask"], mask_output / item["mask"].name)
            filenames.append(item["image"].name)
        manifest["splits"][split_name] = filenames

    with (output_dir / "split_manifest.json").open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(manifest, file, ensure_ascii=False, indent=2)

    print("===== 数据集划分完成 =====")
    print(f"随机种子：{best_seed}")
    print(f"总图片：{total}")
    for split_name in ("train", "val", "test"):
        split_items = best_splits[split_name]
        print(f"{split_name}：{len(split_items)}张")
        for class_id, name in CLASS_NAMES.items():
            count = sum(
                class_id in item["classes"] for item in split_items
            )
            print(
                f"  {class_id}={name}：{count}张，"
                f"{100.0 * count / len(split_items):.1f}%"
            )
    print(f"输出目录：{output_dir}")


if __name__ == "__main__":
    main()
