#!/usr/bin/env python3
"""Train a four-class DeepLabV3 model for the foam grasp dataset."""

import argparse
import json
import random
import time
from pathlib import Path

import numpy as np
from PIL import Image
import torch
from torch import nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset
from torchvision.models.segmentation import (
    DeepLabV3_ResNet50_Weights,
    deeplabv3_resnet50,
)
from torchvision.transforms import ColorJitter
from torchvision.transforms import functional as TF
from torchvision.transforms.functional import InterpolationMode
from tqdm import tqdm


PROJECT_ROOT = Path(__file__).resolve().parents[1]

NUM_CLASSES = 4
CLASS_NAMES = ["background", "cube", "cylinder", "sphere"]
INPUT_HEIGHT = 360
INPUT_WIDTH = 640
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=PROJECT_ROOT / "data" / "segmentation_dataset",
    )
    parser.add_argument(
        "--runs-root",
        type=Path,
        default=PROJECT_ROOT / "data" / "training_runs",
    )
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="Run only two train and two validation batches.",
    )
    return parser.parse_args()


def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


class FoamSegmentationDataset(Dataset):
    def __init__(self, root, split, training):
        self.image_dir = root / "images" / split
        self.mask_dir = root / "masks" / split
        self.training = training
        self.image_paths = sorted(self.image_dir.glob("*.png"))
        self.color_jitter = ColorJitter(
            brightness=0.20,
            contrast=0.20,
            saturation=0.15,
            hue=0.03,
        )

        if not self.image_paths:
            raise RuntimeError(f"No images found in {self.image_dir}")

        for image_path in self.image_paths:
            mask_path = self.mask_dir / image_path.name
            if not mask_path.exists():
                raise RuntimeError(f"Missing mask: {mask_path}")

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, index):
        image_path = self.image_paths[index]
        mask_path = self.mask_dir / image_path.name

        with Image.open(image_path) as image_file:
            image = image_file.convert("RGB")

        with Image.open(mask_path) as mask_file:
            mask = mask_file.convert("L")

        if image.size != mask.size:
            raise RuntimeError(
                f"Image/mask size mismatch for {image_path.name}: "
                f"{image.size} vs {mask.size}"
            )

        if self.training:
            if random.random() < 0.5:
                image = TF.hflip(image)
                mask = TF.hflip(mask)

            if random.random() < 0.8:
                image = self.color_jitter(image)

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

        mask_array = np.asarray(mask, dtype=np.int64).copy()
        illegal = np.setdiff1d(np.unique(mask_array), np.arange(NUM_CLASSES))
        if illegal.size:
            raise RuntimeError(
                f"Illegal mask values in {mask_path.name}: {illegal.tolist()}"
            )

        target = torch.from_numpy(mask_array).long()
        return image, target, image_path.name


def compute_class_weights(mask_dir):
    pixel_counts = np.zeros(NUM_CLASSES, dtype=np.float64)
    mask_paths = sorted(mask_dir.glob("*.png"))

    if not mask_paths:
        raise RuntimeError(f"No training masks found in {mask_dir}")

    for mask_path in tqdm(mask_paths, desc="Scanning masks", leave=False):
        with Image.open(mask_path) as mask_file:
            values = np.asarray(mask_file.convert("L"), dtype=np.int64)
        pixel_counts += np.bincount(
            values.reshape(-1), minlength=NUM_CLASSES
        )[:NUM_CLASSES]

    if np.any(pixel_counts == 0):
        raise RuntimeError(f"A class has zero pixels: {pixel_counts.tolist()}")

    frequencies = pixel_counts / pixel_counts.sum()
    weights = 1.0 / np.sqrt(frequencies)
    weights /= weights.mean()
    return torch.tensor(weights, dtype=torch.float32), pixel_counts, frequencies


class CombinedSegmentationLoss(nn.Module):
    def __init__(self, class_weights):
        super().__init__()
        self.cross_entropy = nn.CrossEntropyLoss(weight=class_weights)

    def forward(self, logits, target):
        ce_loss = self.cross_entropy(logits, target)
        probabilities = torch.softmax(logits, dim=1)
        one_hot = F.one_hot(target, NUM_CLASSES).permute(0, 3, 1, 2).float()

        dimensions = (0, 2, 3)
        intersection = (probabilities * one_hot).sum(dim=dimensions)
        denominator = probabilities.sum(dim=dimensions) + one_hot.sum(dim=dimensions)
        dice = (2.0 * intersection + 1.0) / (denominator + 1.0)
        foreground_dice_loss = 1.0 - dice[1:].mean()

        return ce_loss + foreground_dice_loss


def build_model():
    model = deeplabv3_resnet50(
        weights=DeepLabV3_ResNet50_Weights.DEFAULT,
        progress=True,
    )
    model.classifier[4] = nn.Conv2d(256, NUM_CLASSES, kernel_size=1)

    if model.aux_classifier is not None:
        model.aux_classifier[4] = nn.Conv2d(256, NUM_CLASSES, kernel_size=1)

    return model


def update_confusion_matrix(confusion, prediction, target):
    valid = (target >= 0) & (target < NUM_CLASSES)
    encoded = NUM_CLASSES * target[valid] + prediction[valid]
    confusion += torch.bincount(
        encoded,
        minlength=NUM_CLASSES * NUM_CLASSES,
    ).reshape(NUM_CLASSES, NUM_CLASSES)


def calculate_metrics(confusion):
    matrix = confusion.cpu().numpy().astype(np.float64)
    intersection = np.diag(matrix)
    union = matrix.sum(axis=1) + matrix.sum(axis=0) - intersection
    iou = np.divide(
        intersection,
        union,
        out=np.full(NUM_CLASSES, np.nan),
        where=union > 0,
    )
    foreground_miou = float(np.nanmean(iou[1:]))
    pixel_accuracy = float(intersection.sum() / max(matrix.sum(), 1.0))
    return iou, foreground_miou, pixel_accuracy


def run_epoch(
    model,
    loader,
    criterion,
    device,
    training,
    optimizer=None,
    scaler=None,
    max_batches=None,
):
    model.train(training)
    total_loss = 0.0
    total_samples = 0
    confusion = torch.zeros(
        (NUM_CLASSES, NUM_CLASSES), dtype=torch.int64, device=device
    )

    description = "Train" if training else "Validate"
    progress = tqdm(loader, desc=description, leave=False)

    for batch_index, (images, targets, _) in enumerate(progress):
        if max_batches is not None and batch_index >= max_batches:
            break

        images = images.to(device, non_blocking=True)
        targets = targets.to(device, non_blocking=True)

        if training:
            optimizer.zero_grad(set_to_none=True)

        with torch.set_grad_enabled(training):
            with torch.amp.autocast("cuda", enabled=device.type == "cuda"):
                outputs = model(images)
                loss = criterion(outputs["out"], targets)

                if training and "aux" in outputs:
                    loss = loss + 0.4 * criterion(outputs["aux"], targets)

            if training:
                scaler.scale(loss).backward()
                scaler.step(optimizer)
                scaler.update()

        predictions = outputs["out"].argmax(dim=1)
        update_confusion_matrix(confusion, predictions, targets)

        batch_size = images.shape[0]
        total_loss += loss.item() * batch_size
        total_samples += batch_size
        progress.set_postfix(loss=f"{loss.item():.4f}")

    average_loss = total_loss / max(total_samples, 1)
    iou, foreground_miou, pixel_accuracy = calculate_metrics(confusion)
    return average_loss, iou, foreground_miou, pixel_accuracy


def save_checkpoint(path, model, optimizer, epoch, metrics, class_weights):
    torch.save(
        {
            "epoch": epoch,
            "model_state_dict": model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict(),
            "foreground_miou": metrics["foreground_miou"],
            "class_iou": metrics["class_iou"],
            "class_weights": class_weights.tolist(),
            "class_names": CLASS_NAMES,
            "num_classes": NUM_CLASSES,
            "input_size": [INPUT_HEIGHT, INPUT_WIDTH],
        },
        path,
    )


def main():
    args = parse_args()
    dataset_root = args.dataset_root.expanduser().resolve()
    runs_root = args.runs_root.expanduser().resolve()
    set_seed(args.seed)

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA GPU is required but was not detected")

    device = torch.device("cuda")
    torch.backends.cudnn.benchmark = True

    run_name = "deeplabv3_resnet50_smoke" if args.smoke_test else "deeplabv3_resnet50"
    run_dir = runs_root / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    train_dataset = FoamSegmentationDataset(dataset_root, "train", training=True)
    val_dataset = FoamSegmentationDataset(dataset_root, "val", training=False)

    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=True,
        persistent_workers=args.workers > 0,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=True,
        persistent_workers=args.workers > 0,
    )

    class_weights, pixel_counts, frequencies = compute_class_weights(
        dataset_root / "masks" / "train"
    )

    print("Device:", torch.cuda.get_device_name(0))
    print("Train images:", len(train_dataset))
    print("Validation images:", len(val_dataset))
    for class_id, name in enumerate(CLASS_NAMES):
        print(
            f"  {class_id}={name}: pixels={int(pixel_counts[class_id])}, "
            f"frequency={frequencies[class_id]:.6f}, "
            f"weight={class_weights[class_id]:.4f}"
        )

    print("Loading pretrained DeepLabV3-ResNet50...")
    model = build_model().to(device)
    criterion = CombinedSegmentationLoss(class_weights.to(device))
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=1e-4,
    )
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer,
        mode="max",
        factor=0.5,
        patience=3,
    )
    scaler = torch.amp.GradScaler("cuda", enabled=True)

    epochs = 1 if args.smoke_test else args.epochs
    max_batches = 2 if args.smoke_test else None
    history = []
    best_miou = -1.0
    epochs_without_improvement = 0
    early_stopping_patience = 10
    start_time = time.time()

    for epoch in range(1, epochs + 1):
        train_loss, _, _, _ = run_epoch(
            model,
            train_loader,
            criterion,
            device,
            training=True,
            optimizer=optimizer,
            scaler=scaler,
            max_batches=max_batches,
        )
        val_loss, class_iou, foreground_miou, pixel_accuracy = run_epoch(
            model,
            val_loader,
            criterion,
            device,
            training=False,
            max_batches=max_batches,
        )

        scheduler.step(foreground_miou)
        learning_rate = optimizer.param_groups[0]["lr"]
        readable_iou = [None if np.isnan(v) else float(v) for v in class_iou]
        record = {
            "epoch": epoch,
            "train_loss": train_loss,
            "val_loss": val_loss,
            "foreground_miou": foreground_miou,
            "pixel_accuracy": pixel_accuracy,
            "class_iou": readable_iou,
            "learning_rate": learning_rate,
        }
        history.append(record)

        iou_text = ", ".join(
            f"{CLASS_NAMES[i]}={class_iou[i]:.3f}" for i in range(NUM_CLASSES)
        )
        print(
            f"Epoch {epoch:02d}/{epochs}: train_loss={train_loss:.4f}, "
            f"val_loss={val_loss:.4f}, foreground_mIoU={foreground_miou:.4f}, "
            f"pixel_acc={pixel_accuracy:.4f}, lr={learning_rate:.2e}"
        )
        print("  IoU:", iou_text)

        metrics = {
            "foreground_miou": foreground_miou,
            "class_iou": readable_iou,
        }
        save_checkpoint(
            run_dir / "last_model.pth",
            model,
            optimizer,
            epoch,
            metrics,
            class_weights,
        )

        if foreground_miou > best_miou:
            best_miou = foreground_miou
            epochs_without_improvement = 0
            save_checkpoint(
                run_dir / "best_model.pth",
                model,
                optimizer,
                epoch,
                metrics,
                class_weights,
            )
            print(f"  Saved new best model (mIoU={best_miou:.4f})")
        else:
            epochs_without_improvement += 1

        with (run_dir / "history.json").open("w", encoding="utf-8") as file:
            json.dump(history, file, ensure_ascii=False, indent=2)

        if not args.smoke_test and epochs_without_improvement >= early_stopping_patience:
            print("Early stopping: validation mIoU did not improve for 10 epochs.")
            break

    elapsed = time.time() - start_time
    print(f"Finished in {elapsed / 60.0:.1f} minutes")
    print("Best foreground mIoU:", f"{best_miou:.4f}")
    print("Best model:", run_dir / "best_model.pth")


if __name__ == "__main__":
    main()
