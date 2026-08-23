"""Summarize per-finger contact wrench samples for grasp qualification."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median


def _percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return float(ordered[index])


def summarize_contact_rows(
    rows,
    *,
    hold_s=1.0,
    minimum_force_N=0.5,
    maximum_p95_force_N=3.0,
):
    """Aggregate contact points by side and timestamp and return pass/fail data."""

    grouped = defaultdict(lambda: defaultdict(float))
    for row in rows:
        side = str(row.get("side", "")).strip()
        if side not in {"left", "right"}:
            continue
        try:
            stamp = int(float(row.get("sim_time_ns", "")))
            force = float(row.get("normal_force_N", ""))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(force):
            continue
        grouped[side][stamp] += max(force, 0.0)

    per_side = {}
    failure_reasons = []
    for side in ("left", "right"):
        samples = grouped.get(side, {})
        stamps = sorted(samples)
        forces = [samples[stamp] for stamp in stamps]
        duration_s = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) >= 2 else 0.0
        median_force = float(median(forces)) if forces else math.nan
        p95_force = _percentile(forces, 0.95)
        stable_fraction = (
            sum(force >= minimum_force_N for force in forces) / len(forces)
            if forces else 0.0
        )
        per_side[side] = {
            "sample_count": len(forces),
            "duration_s": duration_s,
            "median_normal_force_N": median_force,
            "p95_normal_force_N": p95_force,
            "stable_fraction": stable_fraction,
            "passed": bool(
                forces
                and duration_s >= hold_s * 0.8
                and median_force >= minimum_force_N
                and p95_force <= maximum_p95_force_N
                and stable_fraction >= 0.8
            ),
        }
        if not forces:
            failure_reasons.append(f"missing_{side}_contact")
        elif not per_side[side]["passed"]:
            failure_reasons.append(f"unstable_{side}_force")

    sides = sorted(
        side for side, values in per_side.items() if values["sample_count"] > 0
    )
    if len(sides) < 2:
        failure_reasons.append("missing_side")
    return {
        "passed": not failure_reasons,
        "sides": sides,
        "per_side": per_side,
        "hold_s": float(hold_s),
        "minimum_force_N": float(minimum_force_N),
        "maximum_p95_force_N": float(maximum_p95_force_N),
        "failure_reasons": failure_reasons,
    }


def summarize_contact_file(input_path, output_path=None):
    input_path = Path(input_path)
    with input_path.open(encoding="utf-8", newline="") as stream:
        summary = summarize_contact_rows(csv.DictReader(stream))
    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    summary = summarize_contact_file(args.input_csv, args.output)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
