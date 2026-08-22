"""Sequential supervisor for reproducible Gazebo benchmark campaigns."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import selectors
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping

from foam_grasp.benchmark_events import TERMINAL_EVENTS
from foam_grasp_sim.benchmark_event_logger import parse_event
from foam_grasp_sim.benchmark_suite import TrialSpec, expand_suite, load_suite


TERMINAL_PREFIX = "BENCHMARK_TERMINAL_EVENT="
RUN_ARTIFACTS = ("metadata.json", "states.csv", "events.csv", "metrics.json")
TRIAL_FIELDS = (
    "run_id", "pair_id", "method", "trajectory", "scenario", "seed", "config_hash",
    "status", "attempt", "started_at", "finished_at", "exit_code", "terminal_event",
    "trial_success", "task_success", "artifacts_complete", "result_path", "log_path",
    "error",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def parse_terminal_line(line: str) -> dict[str, Any] | None:
    if not line.startswith(TERMINAL_PREFIX):
        return None
    try:
        event = parse_event(line[len(TERMINAL_PREFIX):].strip())
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return event if event.get("event") in TERMINAL_EVENTS else None


def status_for_terminal(event: Mapping[str, Any]) -> dict[str, Any]:
    name = str(event.get("event", "")).upper()
    details = dict(event.get("details") or {})
    if name == "TRIAL_FINISHED":
        task_success = bool(details.get("task_success", details.get("execution_mode") == "execute"))
        return {"status": "finished", "trial_success": True, "task_success": task_success}
    return {"status": "failed", "trial_success": False, "task_success": False}


def terminal_matches(spec: TrialSpec, event: Mapping[str, Any]) -> bool:
    return (
        str(event.get("event", "")).upper() in TERMINAL_EVENTS
        and str(event.get("method", "")) == spec.method
        and str(event.get("scenario", "")) == spec.scenario
        and int(event.get("seed", -1)) == spec.seed
    )


def artifacts_complete(run_dir: Path) -> bool:
    return run_dir.is_dir() and all((run_dir / name).is_file() for name in RUN_ARTIFACTS)


def signal_process_group(pid: int, sig: signal.Signals) -> None:
    """Signal only the process group created by this runner."""

    try:
        os.killpg(os.getpgid(pid), sig)
    except ProcessLookupError:
        return


def _wait_process(process: subprocess.Popen[str], seconds: float) -> bool:
    deadline = time.monotonic() + seconds
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.05)
    return process.poll() is not None


def stop_process_group(process: subprocess.Popen[str], *, grace_s: float = 2.0) -> str:
    if process.poll() is not None:
        return "already_exited"
    signal_process_group(process.pid, signal.SIGINT)
    if _wait_process(process, grace_s):
        return "sigint"
    signal_process_group(process.pid, signal.SIGTERM)
    if _wait_process(process, grace_s):
        return "sigterm"
    signal_process_group(process.pid, signal.SIGKILL)
    _wait_process(process, grace_s)
    return "sigkill"


def _replace_arg(args: list[str], name: str, value: Any) -> None:
    prefix = f"{name}:="
    for index, arg in enumerate(args):
        if arg.startswith(prefix):
            args[index] = prefix + str(value)
            return
    args.append(prefix + str(value))


def _condition_json(spec: TrialSpec) -> str:
    condition = {key: value for key, value in spec.resolved.items() if key != "method"}
    return json.dumps(condition, sort_keys=True, separators=(",", ":"))


class CampaignRunner:
    def __init__(
        self,
        specs: list[TrialSpec],
        campaign_dir: Path,
        *,
        timeout_s: float | None = None,
        popen_factory: Callable[..., subprocess.Popen[str]] = subprocess.Popen,
    ):
        self.specs = specs
        self.campaign_dir = campaign_dir
        self.timeout_s = timeout_s
        self.popen_factory = popen_factory
        self.campaign_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir = self.campaign_dir / "logs"
        self.runs_dir = self.campaign_dir / "runs"
        self.logs_dir.mkdir(exist_ok=True)
        self.runs_dir.mkdir(exist_ok=True)
        self.rows = self._load_rows()

    @property
    def trials_path(self) -> Path:
        return self.campaign_dir / "trials.csv"

    @property
    def manifest_path(self) -> Path:
        return self.campaign_dir / "campaign.json"

    def _load_rows(self) -> dict[str, dict[str, str]]:
        if not self.trials_path.is_file():
            return {}
        with self.trials_path.open(encoding="utf-8", newline="") as stream:
            return {row["run_id"]: row for row in csv.DictReader(stream)}

    def _write_rows(self) -> None:
        output = []
        from io import StringIO
        stream = StringIO()
        writer = csv.DictWriter(stream, fieldnames=TRIAL_FIELDS)
        writer.writeheader()
        for spec in self.specs:
            row = self.rows.get(spec.run_id, self._base_row(spec))
            writer.writerow({field: row.get(field, "") for field in TRIAL_FIELDS})
        _atomic_write(self.trials_path, stream.getvalue())

    @staticmethod
    def _base_row(spec: TrialSpec) -> dict[str, str]:
        return {
            "run_id": spec.run_id, "pair_id": spec.pair_id, "method": spec.method,
            "trajectory": spec.trajectory, "scenario": spec.scenario, "seed": str(spec.seed),
            "config_hash": spec.config_hash, "status": "pending", "attempt": "0",
            "trial_success": "false", "task_success": "false", "artifacts_complete": "false",
        }

    def _write_manifest(self, *, suite_name: str, suite_hash: str, started_at: str, finished_at: str | None = None) -> None:
        counts: dict[str, int] = {}
        for row in self.rows.values():
            counts[row.get("status", "pending")] = counts.get(row.get("status", "pending"), 0) + 1
        payload = {
            "schema_version": 1,
            "runner": "foam_grasp_sim.experiment_runner",
            "suite": suite_name,
            "suite_hash": suite_hash,
            "campaign_id": self.campaign_dir.name,
            "started_at": started_at,
            "finished_at": finished_at,
            "trial_count": len(self.specs),
            "counts": counts,
            "trials": [dict(self.rows.get(spec.run_id, self._base_row(spec))) for spec in self.specs],
        }
        _atomic_write(self.manifest_path, json.dumps(payload, indent=2, sort_keys=True) + "\n")

    def _command(self, spec: TrialSpec, attempt: int) -> tuple[list[str], Path, Path]:
        run_name = spec.run_id if attempt == 1 else f"{spec.run_id}__attempt-{attempt:03d}"
        args = list(spec.launch_args)
        _replace_arg(args, "results_root", str(self.runs_dir))
        _replace_arg(args, "run_id", run_name)
        _replace_arg(args, "config_hash", spec.config_hash)
        _replace_arg(args, "pair_id", spec.pair_id)
        _replace_arg(args, "condition_json", _condition_json(spec))
        log_name = spec.run_id if attempt == 1 else f"{spec.run_id}__attempt-{attempt:03d}"
        return args, self.runs_dir / run_name, self.logs_dir / f"{log_name}.log"

    def _should_run(self, spec: TrialSpec, *, resume: bool, rerun_failed: bool) -> bool:
        row = self.rows.get(spec.run_id)
        if row is None:
            return True
        status = row.get("status", "pending")
        complete = row.get("artifacts_complete") == "true"
        if status == "finished" and complete:
            return False
        if rerun_failed:
            return status in {"failed", "timed_out", "interrupted"} or not complete
        return resume or status == "pending"

    def _next_attempt(self, spec: TrialSpec) -> int:
        row = self.rows.get(spec.run_id)
        return int(row.get("attempt", "0")) + 1 if row else 1

    def run_trial(self, spec: TrialSpec, *, attempt: int) -> dict[str, str]:
        command, run_dir, log_path = self._command(spec, attempt)
        row = self.rows.get(spec.run_id, self._base_row(spec))
        row.update({"status": "running", "attempt": str(attempt), "started_at": utc_now(), "log_path": str(log_path), "result_path": str(run_dir)})
        self.rows[spec.run_id] = row
        self._write_rows()
        terminal = None
        timed_out = False
        process = None
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("w", encoding="utf-8") as log:
            try:
                process = self.popen_factory(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, start_new_session=True,
                )
                selector = selectors.DefaultSelector()
                if process.stdout is not None:
                    selector.register(process.stdout, selectors.EVENT_READ)
                deadline = time.monotonic() + (self.timeout_s or spec.timeout_s)
                while process.poll() is None:
                    if time.monotonic() >= deadline:
                        timed_out = True
                        break
                    for key, _ in selector.select(timeout=0.1):
                        line = key.fileobj.readline()
                        if not line:
                            selector.unregister(key.fileobj)
                            continue
                        log.write(line)
                        log.flush()
                        event = parse_terminal_line(line.strip())
                        if event is not None and terminal_matches(spec, event):
                            terminal = event
                            break
                    if terminal is not None:
                        break
                if terminal is not None or timed_out:
                    cleanup = stop_process_group(process)
                else:
                    cleanup = "already_exited"
                # Drain output that was already buffered before finalizing.
                if process.stdout is not None:
                    for line in process.stdout:
                        log.write(line)
                        if terminal is None:
                            event = parse_terminal_line(line.strip())
                            if event is not None and terminal_matches(spec, event):
                                terminal = event
            except KeyboardInterrupt:
                if process is not None:
                    cleanup = stop_process_group(process)
                row.update({"status": "interrupted", "error": "runner interrupted"})
                raise
            finally:
                if process is not None:
                    process.wait(timeout=5)
                    if process.stdout is not None:
                        process.stdout.close()
        row["finished_at"] = utc_now()
        row["exit_code"] = str(process.returncode if process is not None else "")
        row["terminal_event"] = terminal.get("event", "") if terminal else ""
        row["artifacts_complete"] = str(artifacts_complete(run_dir)).lower()
        if timed_out:
            row.update({"status": "timed_out", "error": "trial timeout"})
        elif terminal is not None:
            values = status_for_terminal(terminal)
            row.update({key: str(value).lower() for key, value in values.items()})
        else:
            row.update({"status": "failed", "trial_success": "false", "task_success": "false", "error": "process exited without terminal event"})
        self.rows[spec.run_id] = row
        self._write_rows()
        return row

    def run(self, *, suite_name: str, suite_hash: str, dry_run: bool = False, resume: bool = False, rerun_failed: bool = False, max_trials: int | None = None) -> list[dict[str, str]]:
        started = utc_now()
        self._write_rows()
        self._write_manifest(suite_name=suite_name, suite_hash=suite_hash, started_at=started)
        selected = [spec for spec in self.specs if self._should_run(spec, resume=resume, rerun_failed=rerun_failed)]
        if max_trials is not None:
            selected = selected[:max_trials]
        if dry_run:
            for spec in selected:
                command, _, _ = self._command(spec, 1)
                print(" ".join(command))
            self._write_manifest(suite_name=suite_name, suite_hash=suite_hash, started_at=started, finished_at=utc_now())
            return [self.rows.get(spec.run_id, self._base_row(spec)) for spec in selected]
        results = []
        for spec in selected:
            results.append(self.run_trial(spec, attempt=self._next_attempt(spec)))
            self._write_manifest(suite_name=suite_name, suite_hash=suite_hash, started_at=started)
        self._write_manifest(suite_name=suite_name, suite_hash=suite_hash, started_at=started, finished_at=utc_now())
        return results


def _suite_hash(suite: Mapping[str, Any]) -> str:
    canonical = json.dumps(suite, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(canonical.encode()).hexdigest()


def suite_path(value: str) -> Path:
    candidate = Path(value)
    if candidate.is_file():
        return candidate
    roots = [Path(__file__).resolve().parents[1] / "config" / "benchmark_suites"]
    try:
        from ament_index_python.packages import get_package_share_directory
        roots.insert(0, Path(get_package_share_directory("foam_grasp_sim")) / "config" / "benchmark_suites")
    except Exception:
        pass
    for root in roots:
        candidate = root / f"{value}.yaml"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"benchmark suite not found: {value}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", help="suite name or YAML path")
    parser.add_argument("--results-root", default="results")
    parser.add_argument("--campaign-id")
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--rerun-failed", action="store_true")
    parser.add_argument("--max-trials", type=int)
    args = parser.parse_args(argv)
    if args.resume or args.rerun_failed:
        if not args.campaign_id:
            parser.error("--resume/--rerun-failed require --campaign-id")
    path = suite_path(args.suite)
    suite = load_suite(path)
    specs = expand_suite(suite)
    suite_hash = _suite_hash(suite)
    campaign_id = args.campaign_id or f"{suite['name']}-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{suite_hash[:8]}"
    campaign_dir = Path(args.results_root) / campaign_id
    if campaign_dir.exists() and not (args.resume or args.rerun_failed or args.dry_run):
        raise SystemExit(f"campaign exists; choose --campaign-id with --resume or --rerun-failed: {campaign_dir}")
    runner = CampaignRunner(specs, campaign_dir, timeout_s=args.timeout)
    runner.run(
        suite_name=suite["name"], suite_hash=suite_hash,
        dry_run=args.dry_run, resume=args.resume,
        rerun_failed=args.rerun_failed, max_trials=args.max_trials,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
