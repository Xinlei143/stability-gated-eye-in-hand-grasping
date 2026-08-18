#!/usr/bin/env python3
"""Evaluate the trained foam segmentation model on the held-out test set."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
from torchvision.models.segmentation import deeplabv3_resnet50
from torchvision.transforms import functional as TF
from torchvision.transforms.functional import InterpolationMode
from tqdm import tqdm


PROJECT_ROOT = Path(__file__).resolve().parents[1]

NUM_CLASSES = 4
CLASS_NAMES = ["background", "cube", "cylinder", "sphere"]
COLORS = {
    0: (0, 0, 0),
    1: (255, 40, 40),
    2: (40, 220, 40),
    3: (40, 100, 255),
}
INPUT_HEIGHT = 360
INPUT_WIDTH = 640
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


class TestDataset(Dataset):
    def __init__(self, dataset_root):
        self.image_dir = dataset_root / "images" / "test"
        self.mask_dir = dataset_root / "masks" / "test"
        self.image_paths = sorted(self.image_dir.glob("*.png"))

        if not self.image_paths:
            raise RuntimeError(f"No test images found in {self.image_dir}")

        for image_path in self.image_paths:
            mask_path = self.mask_dir / image_path.name
            if not mask_path.exists():
                raise RuntimeError(f"Missing test mask: {mask_path}")

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, index):
        image_path = self.image_paths[index]
        mask_path = self.mask_dir / image_path.name

        with Image.open(image_path) as image_file:
            image = image_file.convert("RGB")

        with Image.open(mask_path) as mask_file:
            mask = mask_file.convert("L")

        image = TF.resize(
            image,
            [INPUT_HEIGHT, INPUT_WIDTH],
            interpolation=InterpolationMode.BILINEAR,
            antialias=True,
        )
        mask = TF.resize(
            mask,
            [INPUT_HEIGHT, INPUT_WIDTH],
            interpolation=InterpolationMode.NEAREST,
        )

        image = TF.to_tensor(image)
        image = TF.normalize(image, IMAGENET_MEAN, IMAGENET_STD)
        target = torch.from_numpy(
            np.asarray(mask, dtype=np.int64).copy()
        ).long()
        return image, target, image_path.name


def build_model():
    model = deeplabv3_resnet50(
        weights=None,
        weights_backbone=None,
        num_classes=NUM_CLASSES,
        aux_loss=True,
    )
    return model


def update_confusion_matrix(confusion, prediction, target):
    valid = (target >= 0) & (target < NUM_CLASSES)
    encoded = NUM_CLASSES * target[valid] + prediction[valid]
    confusion += torch.bincount(
        encoded,
        minlength=NUM_CLASSES * NUM_CLASSES,
    ).reshape(NUM_CLASSES, NUM_CLASSES)


def safe_divide(numerator, denominator):
    return np.divide(
        numerator,
        denominator,
        out=np.full_like(numerator, np.nan, dtype=np.float64),
        where=denominator > 0,
    )


def calculate_metrics(confusion):
    matrix = confusion.cpu().numpy().astype(np.float64)
    true_positive = np.diag(matrix)
    target_pixels = matrix.sum(axis=1)
    predicted_pixels = matrix.sum(axis=0)
    union = target_pixels + predicted_pixels - true_positive

    iou = safe_divide(true_positive, union)
    precision = safe_divide(true_positive, predicted_pixels)
    recall = safe_divide(true_positive, target_pixels)
    dice = safe_divide(2.0 * true_positive, target_pixels + predicted_pixels)

    return {
        "confusion_matrix": matrix.astype(np.int64).tolist(),
        "pixel_accuracy": float(true_positive.sum() / matrix.sum()),
        "foreground_miou": float(np.nanmean(iou[1:])),
        "all_class_miou": float(np.nanmean(iou)),
        "iou": iou,
        "precision": precision,
        "recall": recall,
        "dice": dice,
    }


def colorize_mask(mask):
    palette = [0] * 768
    for class_id, color in COLORS.items():
        start = class_id * 3
        palette[start:start + 3] = list(color)

    colored = mask.convert("P")
    colored.putpalette(palette)
    return colored.convert("RGB")


def overlay_mask(image, mask):
    colored = colorize_mask(mask)
    alpha = mask.point(lambda value: 150 if value else 0)
    result = image.copy()
    result.paste(colored, (0, 0), alpha)
    return result


def make_comparison(image, target_mask, prediction_mask, filename):
    panels = [
        (image, "Original"),
        (overlay_mask(image, target_mask), "Ground truth"),
        (overlay_mask(image, prediction_mask), "Prediction"),
    ]

    panel_width = 480
    panel_height = 270
    title_height = 42
    canvas = Image.new(
        "RGB",
        (panel_width * 3, panel_height + title_height),
        "white",
    )
    draw = ImageDraw.Draw(canvas)

    for index, (panel, title) in enumerate(panels):
        resized = panel.resize(
            (panel_width, panel_height),
            Image.Resampling.LANCZOS,
        )
        canvas.paste(resized, (index * panel_width, title_height))
        draw.text(
            (index * panel_width + 10, 8),
            f"{title}: {filename}" if index == 0 else title,
            fill="black",
        )

    return canvas


def create_contact_sheet(comparison_paths, output_dir):
    if not comparison_paths:
        return None

    sample_count = min(8, len(comparison_paths))
    if sample_count == 1:
        selected = [comparison_paths[0]]
    else:
        indices = [
            round(i * (len(comparison_paths) - 1) / (sample_count - 1))
            for i in range(sample_count)
        ]
        selected = [comparison_paths[index] for index in indices]

    row_width = 1440
    row_height = 312
    sheet = Image.new(
        "RGB",
        (row_width, row_height * sample_count),
        "white",
    )

    for row, path in enumerate(selected):
        with Image.open(path) as comparison_file:
            comparison = comparison_file.convert("RGB")
        sheet.paste(comparison, (0, row * row_height))

    output_path = output_dir / "test_contact_sheet.png"
    sheet.save(output_path)
    return output_path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=PROJECT_ROOT / "data" / "segmentation_dataset",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=PROJECT_ROOT / "runtime" / "models" / "best_model.pth",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "data" / "test_results" / "deeplabv3_resnet50",
    )
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--workers", type=int, default=4)
    return parser.parse_args()


def main():
    args = parse_args()
    dataset_root = args.dataset_root.expanduser().resolve()
    checkpoint_path = args.checkpoint.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA GPU was not detected")
    if not checkpoint_path.exists():
        raise RuntimeError(f"Checkpoint not found: {checkpoint_path}")

    prediction_dir = output_dir / "prediction_masks"
    comparison_dir = output_dir / "comparisons"
    prediction_dir.mkdir(parents=True, exist_ok=True)
    comparison_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda")
    dataset = TestDataset(dataset_root)
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=True,
    )

    checkpoint = torch.load(
        checkpoint_path,
        map_location="cpu",
        weights_only=False,
    )
    model = build_model()
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    model.to(device)
    model.eval()

    confusion = torch.zeros(
        (NUM_CLASSES, NUM_CLASSES),
        dtype=torch.int64,
        device=device,
    )
    comparison_paths = []

    with torch.inference_mode():
        for images, targets, filenames in tqdm(loader, desc="Testing"):
            images = images.to(device, non_blocking=True)
            targets = targets.to(device, non_blocking=True)

            with torch.amp.autocast("cuda", enabled=True):
                logits = model(images)["out"]

            predictions = logits.argmax(dim=1)
            update_confusion_matrix(confusion, predictions, targets)

            predictions_cpu = predictions.cpu().numpy().astype(np.uint8)

            for index, filename in enumerate(filenames):
                image_path = dataset_root / "images" / "test" / filename
                target_path = dataset_root / "masks" / "test" / filename

                with Image.open(image_path) as image_file:
                    original_image = image_file.convert("RGB")
                with Image.open(target_path) as target_file:
                    original_target = target_file.convert("L")

                prediction_mask = Image.fromarray(
                    predictions_cpu[index], mode="L"
                ).resize(original_image.size, Image.Resampling.NEAREST)
                prediction_mask.save(prediction_dir / filename)

                comparison = make_comparison(
                    original_image,
                    original_target,
                    prediction_mask,
                    filename,
                )
                comparison_path = comparison_dir / f"{Path(filename).stem}.jpg"
                comparison.save(comparison_path, quality=92)
                comparison_paths.append(comparison_path)

    metrics = calculate_metrics(confusion)
    serializable_metrics = {
        "checkpoint": str(checkpoint_path),
        "checkpoint_epoch": checkpoint["epoch"],
        "test_images": len(dataset),
        "pixel_accuracy": metrics["pixel_accuracy"],
        "foreground_miou": metrics["foreground_miou"],
        "all_class_miou": metrics["all_class_miou"],
        "classes": {},
        "confusion_matrix": metrics["confusion_matrix"],
    }

    print("===== Test results =====")
    print("Checkpoint epoch:", checkpoint["epoch"])
    print("Test images:", len(dataset))
    print(f"Pixel accuracy: {metrics['pixel_accuracy']:.6f}")
    print(f"Foreground mIoU: {metrics['foreground_miou']:.6f}")
    print(f"All-class mIoU: {metrics['all_class_miou']:.6f}")

    for class_id, name in enumerate(CLASS_NAMES):
        class_metrics = {
            "iou": float(metrics["iou"][class_id]),
            "precision": float(metrics["precision"][class_id]),
            "recall": float(metrics["recall"][class_id]),
            "dice": float(metrics["dice"][class_id]),
        }
        serializable_metrics["classes"][name] = class_metrics
        print(
            f"{class_id}={name}: "
            f"IoU={class_metrics['iou']:.6f}, "
            f"precision={class_metrics['precision']:.6f}, "
            f"recall={class_metrics['recall']:.6f}, "
            f"Dice={class_metrics['dice']:.6f}"
        )

    metrics_path = output_dir / "test_metrics.json"
    with metrics_path.open("w", encoding="utf-8") as file:
        json.dump(serializable_metrics, file, indent=2, ensure_ascii=False)

    contact_sheet_path = create_contact_sheet(
        sorted(comparison_paths), output_dir
    )
    print("Metrics:", metrics_path)
    print("Prediction masks:", prediction_dir)
    print("Comparisons:", comparison_dir)
    print("Contact sheet:", contact_sheet_path)


if __name__ == "__main__":
    main()
