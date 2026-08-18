#!/usr/bin/env python3
"""Validate indexed masks and create overlay/contact-sheet previews."""

import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


CLASS_NAMES = ("background", "cube", "cylinder", "sphere")
COLORS = np.array(
    [
        [0, 0, 0],
        [255, 40, 40],
        [40, 220, 40],
        [40, 100, 255],
    ],
    dtype=np.uint8,
)


def parse_arguments():
    parser = argparse.ArgumentParser(description="检查mask并生成叠加预览")
    parser.add_argument("--images-dir", type=Path, required=True)
    parser.add_argument("--masks-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sample-count", type=int, default=24)
    return parser.parse_args()


def evenly_spaced(paths, count):
    if len(paths) <= count:
        return paths
    indices = np.linspace(0, len(paths) - 1, count).round().astype(int)
    return [paths[int(index)] for index in indices]


def overlay(image, mask):
    image_array = np.asarray(image.convert("RGB"), dtype=np.uint8).copy()
    mask_array = np.asarray(mask.convert("L"), dtype=np.uint8)
    foreground = mask_array > 0
    if np.any(foreground):
        blended = (
            image_array[foreground].astype(np.float32) * 0.55
            + COLORS[mask_array[foreground]].astype(np.float32) * 0.45
        )
        image_array[foreground] = np.clip(blended, 0, 255).astype(np.uint8)
    return Image.fromarray(image_array, mode="RGB")


def main():
    arguments = parse_arguments()
    images_dir = arguments.images_dir.expanduser().resolve()
    masks_dir = arguments.masks_dir.expanduser().resolve()
    output_dir = arguments.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if arguments.sample_count < 1:
        raise RuntimeError("--sample-count必须至少为1")

    mask_paths = sorted(masks_dir.glob("*.png"))
    if not mask_paths:
        raise RuntimeError(f"没有找到mask：{masks_dir}")

    pixel_counts = np.zeros(4, dtype=np.int64)
    image_by_stem = {
        path.stem: path
        for path in images_dir.iterdir()
        if path.is_file() and path.suffix.lower() in (".png", ".jpg", ".jpeg")
    }
    missing_images = []
    for mask_path in mask_paths:
        if mask_path.stem not in image_by_stem:
            missing_images.append(mask_path.name)
            continue
        with Image.open(mask_path) as mask_file:
            mask_array = np.asarray(mask_file.convert("L"), dtype=np.uint8)
        illegal = np.setdiff1d(np.unique(mask_array), [0, 1, 2, 3])
        if illegal.size:
            raise RuntimeError(
                f"{mask_path.name}包含非法像素：{illegal.tolist()}"
            )
        pixel_counts += np.bincount(
            mask_array.reshape(-1),
            minlength=4,
        )[:4]
    if missing_images:
        raise RuntimeError(f"{len(missing_images)}个mask没有对应图片")

    preview_paths = []
    for mask_path in evenly_spaced(mask_paths, arguments.sample_count):
        image_path = image_by_stem[mask_path.stem]
        with Image.open(image_path) as image_file:
            image = image_file.convert("RGB")
        with Image.open(mask_path) as mask_file:
            mask = mask_file.convert("L")
        if image.size != mask.size:
            raise RuntimeError(
                f"{image_path.name}与{mask_path.name}尺寸不一致"
            )
        result = overlay(image, mask)
        result.thumbnail((480, 320), Image.Resampling.LANCZOS)
        panel = Image.new("RGB", (500, 360), "white")
        panel.paste(result, ((500 - result.width) // 2, 30))
        ImageDraw.Draw(panel).text((10, 6), image_path.name, fill="black")
        preview_path = output_dir / f"{mask_path.stem}_overlay.jpg"
        panel.save(preview_path, quality=92)
        preview_paths.append(preview_path)

    columns = 4
    rows = math.ceil(len(preview_paths) / columns)
    sheet = Image.new("RGB", (columns * 500, rows * 360), "white")
    for index, path in enumerate(preview_paths):
        with Image.open(path) as panel_file:
            panel = panel_file.convert("RGB")
        sheet.paste(
            panel,
            ((index % columns) * 500, (index // columns) * 360),
        )
    contact_sheet = output_dir / "mask_contact_sheet.jpg"
    sheet.save(contact_sheet, quality=90)

    total_pixels = max(int(pixel_counts.sum()), 1)
    print("===== Mask检查完成 =====")
    print(f"检查mask：{len(mask_paths)}")
    for class_id, name in enumerate(CLASS_NAMES):
        percentage = 100.0 * int(pixel_counts[class_id]) / total_pixels
        print(
            f"{class_id}={name}：{int(pixel_counts[class_id])}像素，"
            f"{percentage:.3f}%"
        )
    print(f"抽查预览：{len(preview_paths)}")
    print(f"总览图：{contact_sheet}")


if __name__ == "__main__":
    main()
