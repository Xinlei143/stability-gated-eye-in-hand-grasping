"""Summarize depth-fusion mask/depth timestamp synchronization diagnostics."""

from __future__ import annotations

import argparse
import ast
import json
import math
from collections import Counter
from pathlib import Path
from typing import Iterable, Mapping


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _percentile(values: list[float], fraction: float):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def parse_topic_echo(path: str | Path) -> list[dict[str, object]]:
    """Parse standard ``ros2 topic echo`` output for String diagnostics."""

    records = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.startswith("data:"):
            continue
        encoded = line.partition(":")[2].strip()
        try:
            decoded = ast.literal_eval(encoded)
            payload = json.loads(decoded if isinstance(decoded, str) else encoded)
        except (SyntaxError, ValueError, TypeError, json.JSONDecodeError):
            continue
        if isinstance(payload, dict):
            records.append(payload)
    return records


def _record_age(record: Mapping[str, object]):
    age = _finite(record.get("mask_age_s"))
    if age is not None:
        return age
    depth_stamp = _finite(record.get("depth_stamp"))
    mask_stamp = _finite(record.get("mask_stamp"))
    if depth_stamp is None or mask_stamp is None:
        return None
    return depth_stamp - mask_stamp


def summarize_records(
    records: Iterable[Mapping[str, object]],
    *,
    target_class: str = "cube",
    threshold_s: float = 0.15,
) -> dict[str, object]:
    """Return signed/absolute age statistics without changing input records."""

    if threshold_s <= 0.0 or not math.isfinite(threshold_s):
        raise ValueError("threshold_s must be a positive finite value")

    materialized = list(records)
    ages = []
    status_counts = Counter()
    for record in materialized:
        target = record.get("classes", {}).get(target_class, {})
        status = target.get("status") if isinstance(target, Mapping) else None
        if status:
            status_counts[str(status)] += 1
        age = _record_age(record)
        if age is not None:
            ages.append(age)

    absolute_ages = [abs(age) for age in ages]
    over_threshold = [age for age in absolute_ages if age > threshold_s]
    p95_absolute = _percentile(absolute_ages, 0.95)
    sync_pass = bool(absolute_ages) and (
        p95_absolute is not None
        and p95_absolute < threshold_s
        and not over_threshold
    )
    return {
        "target_class": target_class,
        "threshold_s": threshold_s,
        "record_count": len(materialized),
        "paired_stamp_count": len(ages),
        "median_mask_age_s": _percentile(ages, 0.50),
        "p95_mask_age_s": _percentile(ages, 0.95),
        "median_abs_mask_age_s": _percentile(absolute_ages, 0.50),
        "p95_abs_mask_age_s": p95_absolute,
        "over_threshold_count": len(over_threshold),
        "over_threshold_fraction": (
            len(over_threshold) / len(ages) if ages else 0.0
        ),
        "valid_count": status_counts.get("valid", 0),
        "stale_mask_count": status_counts.get("stale_mask", 0),
        "no_mask_count": status_counts.get("no_mask", 0),
        "status_counts": dict(sorted(status_counts.items())),
        "sync_pass": sync_pass,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic_echo", type=Path)
    parser.add_argument("--target-class", default="cube")
    parser.add_argument("--threshold-s", type=float, default=0.15)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    summary = summarize_records(
        parse_topic_echo(args.topic_echo),
        target_class=args.target_class,
        threshold_s=args.threshold_s,
    )
    encoded = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
