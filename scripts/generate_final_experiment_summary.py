#!/usr/bin/env python3
"""Generate the final experiment audit bundle from frozen artifacts."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from analysis.final_experiment_summary import run_audit


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory; defaults to results/final_experiment_summary",
    )
    args = parser.parse_args()
    output_dir = args.output_dir or args.repo_root / "results" / "final_experiment_summary"
    result = run_audit(args.repo_root, output_dir)
    print(f"wrote final audit bundle to {output_dir}")
    print(f"controlled={len(result['gt'])} rgbd={len(result['rgbd'])} all={len(result['all'])}")
    print(f"input_hashes_unchanged={result['input_hashes_unchanged']}")
    print(f"artifact_inconsistencies={len(result['inconsistencies'])}")
    return 0 if result["input_hashes_unchanged"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
