"""Create a compact Markdown/CSV result table from summary artifacts."""

from __future__ import annotations

import argparse
from pathlib import Path

from .benchmark_io import write_csv
from .summarize import summarize_campaign


def make_result_table(campaign_dir: str | Path, output_dir: str | Path | None = None) -> Path:
    output = summarize_campaign(campaign_dir, output_dir)
    rows = []
    import csv
    with (output / "group_summary.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    fields = ["method", "trajectory", "count", "trial_success_rate", "task_success_rate", "tracking_rms_error_m.mean", "time_to_ready_s.mean"]
    table_rows = [{field: row.get(field, "") for field in fields} for row in rows]
    write_csv(output / "result_table.csv", table_rows, fields)
    lines = ["| " + " | ".join(fields) + " |", "| " + " | ".join("---" for _ in fields) + " |"]
    lines.extend("| " + " | ".join(str(row.get(field, "")) for field in fields) + " |" for row in table_rows)
    (output / "result_table.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign")
    parser.add_argument("--output-dir")
    args = parser.parse_args(argv)
    print(make_result_table(args.campaign, args.output_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
