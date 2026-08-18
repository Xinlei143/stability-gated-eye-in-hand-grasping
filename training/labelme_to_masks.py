#!/usr/bin/env python3
"""Convert LabelMe JSON polygons into four-class indexed PNG masks."""

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


LABEL_IDS = {
    "cube": 1,
    "cylinder": 2,
    "sphere": 3,
}
SUPPORTED_IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg")


def parse_arguments():
    parser = argparse.ArgumentParser(description="LabelMe JSON转语义分割mask")
    parser.add_argument("--images-dir", type=Path, required=True)
    parser.add_argument("--annotations-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def draw_shape(draw, shape, class_id):
    points = [
        (float(point[0]), float(point[1]))
        for point in shape.get("points", [])
    ]
    shape_type = shape.get("shape_type") or "polygon"

    if shape_type == "polygon":
        if len(points) < 3:
            raise ValueError("polygon至少需要3个点")
        draw.polygon(points, fill=class_id)
    elif shape_type == "rectangle":
        if len(points) != 2:
            raise ValueError("rectangle必须有2个点")
        draw.rectangle([points[0], points[1]], fill=class_id)
    elif shape_type == "circle":
        if len(points) != 2:
            raise ValueError("circle必须有圆心和圆周点")
        center, edge = points
        radius = (
            (edge[0] - center[0]) ** 2
            + (edge[1] - center[1]) ** 2
        ) ** 0.5
        draw.ellipse(
            [
                center[0] - radius,
                center[1] - radius,
                center[0] + radius,
                center[1] + radius,
            ],
            fill=class_id,
        )
    else:
        raise ValueError(f"不支持的LabelMe shape_type：{shape_type}")


def image_paths(directory):
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_IMAGE_SUFFIXES
    )


def main():
    arguments = parse_arguments()
    images_dir = arguments.images_dir.expanduser().resolve()
    annotations_dir = arguments.annotations_dir.expanduser().resolve()
    output_dir = arguments.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    images = image_paths(images_dir)
    if not images:
        raise RuntimeError(f"没有找到输入图片：{images_dir}")

    missing_annotations = []
    counts = Counter()
    output_count = 0
    for image_path in images:
        annotation_path = annotations_dir / f"{image_path.stem}.json"
        if not annotation_path.is_file():
            missing_annotations.append(annotation_path.name)
            continue

        with Image.open(image_path) as image_file:
            width, height = image_file.size
        with annotation_path.open("r", encoding="utf-8") as file:
            annotation = json.load(file)

        json_width = int(annotation.get("imageWidth") or width)
        json_height = int(annotation.get("imageHeight") or height)
        if (json_width, json_height) != (width, height):
            raise RuntimeError(
                f"{annotation_path.name}尺寸{json_width}x{json_height}"
                f"与图片{width}x{height}不一致"
            )

        mask = Image.new("L", (width, height), 0)
        draw = ImageDraw.Draw(mask)
        for index, shape in enumerate(annotation.get("shapes", [])):
            label = str(shape.get("label", "")).strip().lower()
            if label not in LABEL_IDS:
                raise RuntimeError(
                    f"{annotation_path.name}第{index + 1}个标注类别"
                    f"{label!r}不在{sorted(LABEL_IDS)}中"
                )
            try:
                draw_shape(draw, shape, LABEL_IDS[label])
            except ValueError as error:
                raise RuntimeError(
                    f"{annotation_path.name}第{index + 1}个shape无效：{error}"
                ) from error
            counts[label] += 1

        values = np.asarray(mask, dtype=np.uint8)
        illegal = np.setdiff1d(np.unique(values), [0, 1, 2, 3])
        if illegal.size:
            raise RuntimeError(
                f"{annotation_path.name}生成了非法像素值：{illegal.tolist()}"
            )
        mask.save(output_dir / f"{image_path.stem}.png")
        output_count += 1

    if missing_annotations:
        preview = ", ".join(missing_annotations[:8])
        raise RuntimeError(
            f"缺少{len(missing_annotations)}个JSON；示例：{preview}"
        )

    print("===== LabelMe转换完成 =====")
    print(f"输入图片：{len(images)}")
    print(f"输出mask：{output_count}")
    for label, class_id in LABEL_IDS.items():
        print(f"{class_id}={label}：{counts[label]}个标注形状")
    print(f"输出目录：{output_dir}")


if __name__ == "__main__":
    main()
