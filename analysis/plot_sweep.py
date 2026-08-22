"""Shared sweep plotting implementation; public scripts select a parameter."""

from __future__ import annotations

import csv
from pathlib import Path

from .benchmark_io import bootstrap_mean_ci, load_campaign_runs, write_csv
from .summarize import default_output_dir


def plot_sweep(campaign_dir: str | Path, parameter: str, output_dir: str | Path | None = None) -> Path:
    root = Path(campaign_dir).resolve()
    output = Path(output_dir).resolve() if output_dir else default_output_dir(root)
    output.mkdir(parents=True, exist_ok=True)
    records, exclusions = load_campaign_runs(root)
    rows = []
    for record in records:
        value = record.condition.get(parameter)
        if value is None:
            continue
        metric_values = [item.value("tracking_rms_error_m") for item in records if item.condition.get(parameter) == value and item.method == record.method]
        metric_values = [item for item in metric_values if item is not None]
        mean, lower, upper = bootstrap_mean_ci(metric_values)
        rows.append({"parameter": parameter, "value": value, "method": record.method, "metric": "tracking_rms_error_m", "mean": mean, "ci_low": lower, "ci_high": upper, "count": len(metric_values)})
    unique = {(row["method"], str(row["value"])): row for row in rows}
    rows = [unique[key] for key in sorted(unique)]
    write_csv(output / "plot_data.csv", rows, ["parameter", "value", "method", "metric", "mean", "ci_low", "ci_high", "count"])
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError("matplotlib is required for plot generation") from error
    figure, axis = plt.subplots(figsize=(6.4, 4.2))
    for method in sorted({row["method"] for row in rows}):
        selected = [row for row in rows if row["method"] == method]
        selected.sort(key=lambda row: float(row["value"]))
        x = [float(row["value"]) for row in selected]
        y = [float(row["mean"]) for row in selected]
        axis.plot(x, y, marker="o", label=method)
        axis.fill_between(x, [float(row["ci_low"]) for row in selected], [float(row["ci_high"]) for row in selected], alpha=0.15)
    axis.set_xlabel(parameter)
    axis.set_ylabel("tracking RMS error (m)")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    stem = f"{parameter}_sweep"
    figure.savefig(output / f"{stem}.png", dpi=160)
    figure.savefig(output / f"{stem}.pdf")
    plt.close(figure)
    return output
