"""Measure raw RGB-D localization quality from a ``states.csv`` file.

The observed point is the value recorded from ``<target>_point_base``.  It is
compared directly with ``target_ground_truth``; method-policy selection and
latched targets are deliberately not used here.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable, Mapping


AXES = ("x", "y", "z")
REQUIRED_COLUMNS = tuple(
    f"target_{source}_{axis}"
    for source in ("ground_truth", "observed")
    for axis in AXES
)


def _number(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _point(row: Mapping[str, object], source: str) -> tuple[float, float, float] | None:
    values = tuple(_number(row.get(f"target_{source}_{axis}")) for axis in AXES)
    return values if all(value is not None for value in values) else None


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _is_fresh_observation(
    row: Mapping[str, object],
    max_observation_age_s: float,
) -> bool:
    """Return whether an observed point is new enough for this sample.

    Older states files did not have ``observation_fresh``.  For those files,
    retain the historical presence-based behavior; current benchmark files
    use the explicit freshness flag emitted by the metrics logger.
    """

    if "observation_fresh" in row:
        flag = _number(row.get("observation_fresh"))
        return flag is not None and flag > 0.5
    age = _number(row.get("observation_age_s"))
    if age is not None:
        return age <= max_observation_age_s
    return True


def summarize_rows(
    rows: Iterable[Mapping[str, object]],
    *,
    max_observation_age_s: float = 0.20,
) -> dict[str, object]:
    """Return raw observation-vs-ground-truth error statistics.

    The valid-observation fraction uses rows with a finite ground-truth point
    as its denominator. Rows with missing or non-finite ground truth are not
    eligible for localization accuracy. Current states files additionally
    require an explicit fresh observation; this prevents a latched point from
    making an intermittent depth-fusion dropout look valid.
    """

    if max_observation_age_s < 0.0:
        raise ValueError("max_observation_age_s must be non-negative")

    materialized = list(rows)
    if materialized:
        missing = [column for column in REQUIRED_COLUMNS if column not in materialized[0]]
        if missing:
            raise ValueError("states.csv missing required columns: " + ", ".join(missing))

    ground_truth_valid = 0
    valid_observations = 0
    absolute_errors = {axis: [] for axis in AXES}
    planar_errors: list[float] = []
    three_d_errors: list[float] = []

    for row in materialized:
        ground_truth = _point(row, "ground_truth")
        if ground_truth is None:
            continue
        ground_truth_valid += 1
        observed = _point(row, "observed")
        if observed is None or not _is_fresh_observation(
            row, max_observation_age_s
        ):
            continue
        valid_observations += 1
        errors = tuple(observed[index] - ground_truth[index] for index in range(3))
        for axis, error in zip(AXES, errors):
            absolute_errors[axis].append(abs(error))
        planar_errors.append(math.hypot(errors[0], errors[1]))
        three_d_errors.append(math.sqrt(sum(error * error for error in errors)))

    summary: dict[str, object] = {
        "row_count": len(materialized),
        "ground_truth_valid_count": ground_truth_valid,
        "valid_observation_count": valid_observations,
        "valid_observation_fraction": (
            valid_observations / ground_truth_valid if ground_truth_valid else 0.0
        ),
    }
    for axis in AXES:
        values = absolute_errors[axis]
        summary[f"median_abs_e{axis}_m"] = _percentile(values, 0.50)
        summary[f"p95_abs_e{axis}_m"] = _percentile(values, 0.95)
    for name, values in (
        ("planar_error_m", planar_errors),
        ("3d_error_m", three_d_errors),
    ):
        summary[f"median_{name}"] = _percentile(values, 0.50)
        summary[f"p95_{name}"] = _percentile(values, 0.95)
        summary[f"max_{name}"] = max(values) if values else None
    return summary


def summarize_file(
    path: str | Path,
    *,
    max_observation_age_s: float = 0.20,
) -> dict[str, object]:
    """Read one states.csv without modifying it and return its summary."""

    input_path = Path(path)
    with input_path.open(encoding="utf-8", newline="") as stream:
        return summarize_rows(
            csv.DictReader(stream),
            max_observation_age_s=max_observation_age_s,
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("states_csv", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        help="optional JSON output path; raw states.csv is never modified",
    )
    parser.add_argument(
        "--max-observation-age-s",
        type=float,
        default=0.20,
        help="freshness threshold used when observation_fresh is absent",
    )
    args = parser.parse_args(argv)
    summary = summarize_file(
        args.states_csv,
        max_observation_age_s=args.max_observation_age_s,
    )
    encoded = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
