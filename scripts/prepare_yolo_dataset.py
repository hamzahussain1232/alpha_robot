#!/usr/bin/env python3
import argparse
import random
import shutil
from pathlib import Path
from typing import List, Tuple


def list_pairs(images_dir: Path, labels_dir: Path) -> List[Tuple[Path, Path]]:
    image_exts = {".jpg", ".jpeg", ".png", ".bmp"}
    pairs = []
    for img in sorted(images_dir.iterdir()):
        if not img.is_file() or img.suffix.lower() not in image_exts:
            continue
        label = labels_dir / f"{img.stem}.txt"
        if not label.exists():
            continue
        pairs.append((img, label))
    return pairs


def ensure_clean_dirs(root: Path) -> None:
    for part in ("images", "labels"):
        for split in ("train", "val", "test"):
            d = root / part / split
            if d.exists():
                shutil.rmtree(d)
            d.mkdir(parents=True, exist_ok=True)


def copy_split(items: List[Tuple[Path, Path]], root: Path, split: str) -> None:
    for img, lbl in items:
        shutil.copy2(img, root / "images" / split / img.name)
        shutil.copy2(lbl, root / "labels" / split / lbl.name)


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare YOLO train/val/test splits")
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("~/ros2_ws/src/articubot_one/assets/ml/dataset").expanduser(),
        help="Dataset root containing images_all/ and labels_all/",
    )
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--val-ratio", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--classes",
        nargs="+",
        default=["medicine_bottle", "medicine_box", "cup", "remote", "book"],
        help="Class names in YOLO index order",
    )
    args = parser.parse_args()

    dataset_root = args.dataset_root
    images_all = dataset_root / "images_all"
    labels_all = dataset_root / "labels_all"

    if not images_all.exists():
        raise SystemExit(f"Missing directory: {images_all}")
    if not labels_all.exists():
        raise SystemExit(f"Missing directory: {labels_all}")

    pairs = list_pairs(images_all, labels_all)
    if not pairs:
        raise SystemExit("No image/label pairs found in images_all + labels_all")

    random.seed(args.seed)
    random.shuffle(pairs)

    n = len(pairs)
    n_train = int(n * args.train_ratio)
    n_val = int(n * args.val_ratio)
    n_test = max(0, n - n_train - n_val)

    train_items = pairs[:n_train]
    val_items = pairs[n_train:n_train + n_val]
    test_items = pairs[n_train + n_val:]

    ensure_clean_dirs(dataset_root)
    copy_split(train_items, dataset_root, "train")
    copy_split(val_items, dataset_root, "val")
    copy_split(test_items, dataset_root, "test")

    data_yaml = dataset_root / "dataset.yaml"
    names_yaml = ", ".join([f"'{n}'" for n in args.classes])
    data_yaml.write_text(
        "\n".join(
            [
                f"path: {dataset_root}",
                "train: images/train",
                "val: images/val",
                "test: images/test",
                "",
                f"names: [{names_yaml}]",
                "",
            ]
        ),
        encoding="utf-8",
    )

    print(f"Prepared dataset at: {dataset_root}")
    print(f"train={len(train_items)} val={len(val_items)} test={len(test_items)} total={n}")
    print(f"Generated: {data_yaml}")


if __name__ == "__main__":
    main()
