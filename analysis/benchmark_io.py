"""Read-only, deterministic IO helpers for Stage 6 campaign artifacts."""

from __future__ import annotations

import csv
import json
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping


REQUIRED_METRICS = ("trial_success", "task_success")
RUN_ARTIFACTS = ("metadata.json", "states.csv", "events.csv", "metrics.json")


@dataclass(frozen=True)
class RunRecord:
    """One complete, successful campaign run plus its resolved condition."""

    run_id: str
    pair_id: str
    method: str
    trajectory: str
    seed: int
    condition: Mapping[str, Any]
    metrics: Mapping[str, Any]
    metadata: Mapping[str, Any] = field(repr=False)

    def value(self, metric: str) -> float | None:
        value = self.metrics.get(metric)
        if value in (None, ""):
            return None
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if result == result and abs(result) != float("inf") else None


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _artifact_complete(run_dir: Path) -> bool:
    return run_dir.is_dir() and all((run_dir / name).is_file() for name in RUN_ARTIFACTS)


def _condition(metadata: Mapping[str, Any]) -> Mapping[str, Any]:
    condition = metadata.get("condition", metadata.get("resolved", {}))
    if not condition and metadata.get("condition_json"):
        try:
            condition = json.loads(str(metadata["condition_json"]))
        except (TypeError, json.JSONDecodeError) as error:
            raise ValueError(f"invalid condition_json: {error}") from error
    return dict(condition) if isinstance(condition, Mapping) else {}


def load_campaign_runs(campaign_dir: str | Path) -> tuple[list[RunRecord], list[dict[str, str]]]:
    """Load complete finished runs without mutating the raw campaign.

    Malformed or incomplete runs are returned in ``exclusions`` rather than
    silently contributing to summaries.
    """

    root = Path(campaign_dir)
    trials_path = root / "trials.csv"
    if not trials_path.is_file():
        raise FileNotFoundError(f"missing campaign trials.csv: {trials_path}")
    records: list[RunRecord] = []
    exclusions: list[dict[str, str]] = []
    with trials_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        run_id = str(row.get("run_id", ""))
        run_dir = root / "runs" / run_id
        reason = ""
        if row.get("status") != "finished":
            reason = f"status={row.get('status', '') or 'missing'}"
        elif row.get("artifacts_complete", "").lower() != "true":
            reason = "artifacts incomplete"
        elif not _artifact_complete(run_dir):
            reason = "run artifact file missing"
        if reason:
            exclusions.append({"run_id": run_id, "reason": reason})
            continue
        try:
            metadata = _json(run_dir / "metadata.json")
            metrics = _json(run_dir / "metrics.json")
            missing = [name for name in REQUIRED_METRICS if name not in metrics]
            if missing:
                raise ValueError("metrics missing " + ", ".join(missing))
            record = RunRecord(
                run_id=run_id,
                pair_id=str(row.get("pair_id") or metadata.get("pair_id", "")),
                method=str(row.get("method") or metadata.get("method", "")),
                trajectory=str(row.get("trajectory") or metadata.get("scenario", "")),
                seed=int(row.get("seed") or metadata.get("seed", 0)),
                condition=_condition(metadata),
                metrics=metrics,
                metadata=metadata,
            )
        except (TypeError, ValueError, KeyError) as error:
            exclusions.append({"run_id": run_id, "reason": str(error)})
            continue
        records.append(record)
    records.sort(key=lambda item: (item.pair_id, item.method, item.run_id))
    exclusions.sort(key=lambda item: item["run_id"])
    return records, exclusions


def bootstrap_mean_ci(values: Iterable[float], *, seed: int = 2026, samples: int = 10_000, alpha: float = 0.05) -> tuple[float | None, float | None, float | None]:
    """Return mean and percentile bootstrap confidence interval."""

    numbers = [float(value) for value in values]
    if not numbers:
        return None, None, None
    if samples <= 0 or not 0.0 < alpha < 1.0:
        raise ValueError("samples must be positive and alpha must be in (0, 1)")
    rng = random.Random(seed)
    means = []
    for _ in range(samples):
        means.append(sum(rng.choice(numbers) for _ in numbers) / len(numbers))
    means.sort()

    def percentile(fraction: float) -> float:
        position = fraction * (len(means) - 1)
        lower = int(position)
        upper = min(lower + 1, len(means) - 1)
        weight = position - lower
        return means[lower] * (1.0 - weight) + means[upper] * weight

    return sum(numbers) / len(numbers), percentile(alpha / 2.0), percentile(1.0 - alpha / 2.0)


def as_mapping(record: RunRecord | Mapping[str, Any]) -> Mapping[str, Any]:
    if isinstance(record, RunRecord):
        return {**record.metrics, "run_id": record.run_id, "pair_id": record.pair_id, "method": record.method, "trajectory": record.trajectory, "seed": record.seed, "condition": record.condition}
    return record


def paired_differences(records: Iterable[RunRecord | Mapping[str, Any]], *, metric: str) -> list[dict[str, Any]]:
    """Return deterministic gated-minus-baseline differences by ``pair_id``."""

    grouped: dict[str, dict[str, Mapping[str, Any]]] = {}
    for item in records:
        record = as_mapping(item)
        pair_id = str(record.get("pair_id", ""))
        method = str(record.get("method", ""))
        if pair_id and method in {"snapshot", "tracking", "gated"}:
            grouped.setdefault(pair_id, {})[method] = record
    output = []
    for pair_id in sorted(grouped):
        methods = grouped[pair_id]
        gated = methods.get("gated")
        if gated is None:
            continue
        gated_value = gated.get(metric)
        if gated_value in (None, ""):
            continue
        for baseline in ("snapshot", "tracking"):
            other = methods.get(baseline)
            if other is None or other.get(metric) in (None, ""):
                continue
            output.append({"pair_id": pair_id, f"gated_minus_{baseline}": float(gated_value) - float(other[metric])})
    return output


def write_csv(path: str | Path, rows: Iterable[Mapping[str, Any]], fieldnames: Iterable[str]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fieldnames), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
