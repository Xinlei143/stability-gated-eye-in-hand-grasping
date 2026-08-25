"""Sequential supervisor for reproducible Gazebo benchmark campaigns."""

from __future__ import annotations

import argparse
import csv
import fcntl
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
from foam_grasp_sim.contact_qualification import summarize_contact_file


TERMINAL_PREFIX = "BENCHMARK_TERMINAL_EVENT="
RUN_ARTIFACTS = ("metadata.json", "states.csv", "events.csv", "metrics.json")
TRIAL_FIELDS = (
    "run_id", "pair_id", "method", "trajectory", "scenario", "seed", "config_hash",
    "status", "attempt", "started_at", "finished_at", "exit_code", "terminal_event",
    "trial_success", "task_success", "outcome", "failure_class", "failure_stage",
    "artifacts_complete", "result_path", "log_path", "cleanup", "error",
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
    # ros2 launch prefixes child stdout (for example ``[node-12] ``), while
    # the event contract remains the fixed single-line marker.
    marker = line.find(TERMINAL_PREFIX)
    if marker < 0:
        return None
    try:
        event = parse_event(line[marker + len(TERMINAL_PREFIX):].strip())
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return event if event.get("event") in TERMINAL_EVENTS else None


def status_for_terminal(event: Mapping[str, Any]) -> dict[str, Any]:
    name = str(event.get("event", "")).upper()
    details = dict(event.get("details") or {})
    if name == "TRIAL_FINISHED":
        execution_mode = str(details.get("execution_mode", "execute"))
        task_success = bool(details.get("task_success", False))
        outcome = str(
            details.get("outcome")
            or ("success" if task_success else "task_failure")
        )
        return {
            "status": "finished", "trial_success": True,
            "task_success": task_success, "execution_mode": execution_mode,
            "outcome": outcome,
            "failure_stage": str(details.get("failure_stage", "")),
            "error": str(details.get("reason", "")),
        }
    return {
        "status": "failed", "trial_success": False, "task_success": False,
        "outcome": "infrastructure_failure",
        "failure_class": str(details.get("failure_class", "infrastructure")),
        "failure_stage": str(details.get("failure_stage", "runtime")),
        "error": str(details.get("reason", "")),
    }


def terminal_matches(spec: TrialSpec, event: Mapping[str, Any]) -> bool:
    return (
        str(event.get("event", "")).upper() in TERMINAL_EVENTS
        and str(event.get("method", "")) == spec.method
        and str(event.get("scenario", "")) == spec.scenario
        and int(event.get("seed", -1)) == spec.seed
    )


def artifacts_complete(run_dir: Path, extra=()) -> bool:
    required = tuple(RUN_ARTIFACTS) + tuple(extra)
    return run_dir.is_dir() and all((run_dir / name).is_file() for name in required)


def status_for_artifacts(run_dir: Path, status: Mapping[str, Any]) -> dict[str, Any]:
    """Apply post-run physical checks without trusting a command-only success."""
    result = dict(status)
    if result.get("execution_mode") == "plan_only":
        return result
    metrics_path = run_dir / "metrics.json"
    try:
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        if result.get("execution_mode") == "execute":
            result.update({
                "status": "failed", "trial_success": False,
                "task_success": False, "outcome": "infrastructure_failure",
                "failure_class": "infrastructure", "failure_stage": "metrics",
                "error": "execute metrics missing",
            })
            return result
        return result
    if result.get("execution_mode") == "execute":
        if metrics.get("trial_status") not in (None, "finished"):
            result.update({
                "status": "failed", "trial_success": False,
                "task_success": False, "outcome": "infrastructure_failure",
                "failure_class": "infrastructure", "failure_stage": "metrics",
                "error": f"execute metrics trial_status={metrics.get('trial_status')}",
            })
            return result
        if not isinstance(metrics.get("task_success"), bool):
            result.update({
                "status": "failed", "trial_success": False,
                "task_success": False, "outcome": "infrastructure_failure",
                "failure_class": "infrastructure", "failure_stage": "metrics",
                "error": "execute metrics task_success missing",
            })
            return result
        if metrics["task_success"] and metrics.get("physical_grasp_success") is not True:
            result.update({
                "status": "failed", "trial_success": False,
                "task_success": False, "outcome": "infrastructure_failure",
                "failure_class": "infrastructure", "failure_stage": "metrics",
                "error": "execute metrics task_success without physical grasp success",
            })
            return result
        events_path = run_dir / "events.csv"
        if events_path.is_file():
            try:
                with events_path.open(encoding="utf-8", newline="") as stream:
                    events = list(csv.DictReader(stream))
                terminal_events = [
                    event for event in events
                    if event.get("event") in {"TRIAL_FINISHED", "TRIAL_FAILED"}
                ]
                terminal = terminal_events[-1] if len(terminal_events) == 1 else None
                details = json.loads(terminal.get("details", "{}")) if terminal else {}
            except (OSError, ValueError, json.JSONDecodeError, TypeError):
                terminal = None
                details = {}
            if (
                terminal is None
                or terminal.get("event") != "TRIAL_FINISHED"
                or not isinstance(details, dict)
                or details.get("task_success") != metrics.get("task_success")
                or (
                    metrics.get("outcome") is not None
                    and details.get("outcome") != metrics.get("outcome")
                )
            ):
                result.update({
                    "status": "failed", "trial_success": False,
                    "task_success": False, "outcome": "infrastructure_failure",
                    "failure_class": "infrastructure", "failure_stage": "events",
                    "error": "events.csv terminal does not match finalized metrics",
                })
                return result
        result.update({
            "status": "finished", "trial_success": True,
            "task_success": metrics["task_success"],
        })
        if metrics["task_success"]:
            result.update({"outcome": "success", "failure_stage": "", "error": ""})
        else:
            result.update({"outcome": "task_failure"})
        return result
    if result.get("task_success") and metrics.get("physical_grasp_success") is not True:
        return {
            "status": "failed", "trial_success": False,
            "task_success": False, "error": "physical grasp verification failed",
        }
    return result


def _signal_process_group_id(pgid: int, sig: signal.Signals) -> None:
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        return


def signal_process_group(pid: int, sig: signal.Signals) -> None:
    """Signal only the process group created by this runner."""

    try:
        pgid = os.getpgid(pid)
    except ProcessLookupError:
        return
    _signal_process_group_id(pgid, sig)


def _process_group_exists(pgid: int) -> bool:
    live_member_found = False
    proc_root = Path("/proc")
    if proc_root.is_dir():
        for entry in proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                stat_text = (entry / "stat").read_text(encoding="utf-8")
                _, fields = stat_text.rsplit(")", 1)
                fields = fields.split()
                state = fields[0]
                process_group = int(fields[2])
            except (OSError, ValueError, IndexError):
                continue
            if process_group == pgid and state != "Z":
                live_member_found = True
                break
        return live_member_found
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_process(process: subprocess.Popen[str], seconds: float) -> bool:
    deadline = time.monotonic() + seconds
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.05)
    return process.poll() is not None


def _wait_process_group(pgid: int, seconds: float) -> bool:
    deadline = time.monotonic() + seconds
    while _process_group_exists(pgid) and time.monotonic() < deadline:
        time.sleep(0.05)
    return not _process_group_exists(pgid)


def stop_process_group(
    process: subprocess.Popen[str], *, pgid: int | None = None, grace_s: float = 2.0
) -> str:
    if pgid is None:
        try:
            pgid = os.getpgid(process.pid)
        except ProcessLookupError:
            return "already_exited"
    if not _process_group_exists(pgid):
        return "already_exited"
    # ros2 launch forwards a signal from its leader to the child processes.
    # Signal the live leader first so the metrics logger receives one orderly
    # shutdown request instead of a duplicate SIGINT from the whole group.
    if process.poll() is None:
        try:
            os.kill(process.pid, signal.SIGINT)
        except ProcessLookupError:
            _signal_process_group_id(pgid, signal.SIGINT)
    else:
        _signal_process_group_id(pgid, signal.SIGINT)
    if _wait_process_group(pgid, grace_s):
        return "sigint"
    _signal_process_group_id(pgid, signal.SIGTERM)
    if _wait_process_group(pgid, grace_s):
        return "sigterm"
    _signal_process_group_id(pgid, signal.SIGKILL)
    return "sigkill" if _wait_process_group(pgid, grace_s) else "cleanup_failed"


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
        logger_flush_grace_s: float = 1.0,
        ros_domain_id: int | None = None,
        gazebo_master_uri: str | None = None,
        popen_factory: Callable[..., subprocess.Popen[str]] = subprocess.Popen,
    ):
        self.specs = specs
        self.campaign_dir = campaign_dir
        self.timeout_s = timeout_s
        self.logger_flush_grace_s = float(logger_flush_grace_s)
        if self.logger_flush_grace_s < 0.0:
            raise ValueError("logger_flush_grace_s must be non-negative")
        campaign_material = campaign_dir.name.encode("utf-8")
        digest = int(hashlib.sha256(campaign_material).hexdigest()[:8], 16)
        if ros_domain_id is None:
            ros_domain_id = 20 + digest % 60
        if not 0 <= int(ros_domain_id) <= 232:
            raise ValueError("ros_domain_id must be within 0..232")
        if gazebo_master_uri is None:
            gazebo_master_uri = f"http://127.0.0.1:{12000 + digest % 2000}"
        if not str(gazebo_master_uri).startswith("http://"):
            raise ValueError("gazebo_master_uri must be an http:// URI")
        self.runtime = {
            "ros_domain_id": int(ros_domain_id),
            "gazebo_master_uri": str(gazebo_master_uri),
        }
        runtime_key = hashlib.sha256(
            f"{self.runtime['ros_domain_id']}|{self.runtime['gazebo_master_uri']}".encode()
        ).hexdigest()[:16]
        self._runtime_lock_path = Path("/tmp") / f"foam_grasp_sim_runtime_{runtime_key}.lock"
        self._runtime_lock = None
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
            "outcome": "", "failure_class": "", "failure_stage": "",
        }

    def _acquire_runtime_lock(self) -> None:
        self._runtime_lock = self._runtime_lock_path.open("a+")
        try:
            fcntl.flock(self._runtime_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self._runtime_lock.close()
            self._runtime_lock = None
            raise RuntimeError(
                "another Gazebo campaign is using the same ROS/Gazebo runtime: "
                f"{self.runtime}"
            ) from error

    def _release_runtime_lock(self) -> None:
        if self._runtime_lock is None:
            return
        fcntl.flock(self._runtime_lock.fileno(), fcntl.LOCK_UN)
        self._runtime_lock.close()
        self._runtime_lock = None

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
            "runtime": dict(self.runtime),
            "trials": [dict(self.rows.get(spec.run_id, self._base_row(spec))) for spec in self.specs],
        }
        _atomic_write(self.manifest_path, json.dumps(payload, indent=2, sort_keys=True) + "\n")

    def _command(self, spec: TrialSpec, attempt: int) -> tuple[list[str], Path, Path]:
        run_name = spec.run_id if attempt == 1 else f"{spec.run_id}__attempt-{attempt:03d}"
        run_dir = self.runs_dir / run_name
        args = list(spec.launch_args)
        _replace_arg(args, "results_root", str(self.runs_dir))
        _replace_arg(args, "run_id", run_name)
        _replace_arg(args, "config_hash", spec.config_hash)
        _replace_arg(args, "pair_id", spec.pair_id)
        _replace_arg(args, "condition_json", _condition_json(spec))
        if bool(spec.resolved.get("record_contact_diagnostics", False)):
            _replace_arg(args, "record_contact_diagnostics", "true")
            _replace_arg(args, "contact_diagnostics_output", str(run_dir / "contact_diagnostics.csv"))
        log_name = spec.run_id if attempt == 1 else f"{spec.run_id}__attempt-{attempt:03d}"
        return args, run_dir, self.logs_dir / f"{log_name}.log"

    @staticmethod
    def _extra_artifacts(spec: TrialSpec):
        if bool(spec.resolved.get("record_contact_diagnostics", False)):
            return ("contact_diagnostics.csv", "contact_metrics.json")
        return ()

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
        cleanup = "not_started"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("w", encoding="utf-8") as log:
            try:
                environment = os.environ.copy()
                environment.update({
                    "ROS_DOMAIN_ID": str(self.runtime["ros_domain_id"]),
                    "GAZEBO_MASTER_URI": self.runtime["gazebo_master_uri"],
                    "ROS_LOCALHOST_ONLY": "1",
                })
                process = self.popen_factory(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, start_new_session=True, env=environment,
                )
                process_group_id = process.pid
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
                if terminal is not None and not timed_out:
                    # The terminal publisher and metrics logger are separate
                    # ROS nodes.  Give the logger one bounded flush window
                    # before interrupting the launch process group.
                    time.sleep(self.logger_flush_grace_s)
                cleanup = stop_process_group(process, pgid=process_group_id)
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
                    cleanup = stop_process_group(process, pgid=process_group_id)
                row.update({"status": "interrupted", "error": "runner interrupted"})
                raise
            finally:
                if process is not None:
                    if _process_group_exists(process_group_id):
                        cleanup = stop_process_group(process, pgid=process_group_id)
                    process.wait(timeout=5)
                    if process.stdout is not None:
                        process.stdout.close()
        row["finished_at"] = utc_now()
        row["exit_code"] = str(process.returncode if process is not None else "")
        row["terminal_event"] = terminal.get("event", "") if terminal else ""
        row["cleanup"] = cleanup
        if bool(spec.resolved.get("record_contact_diagnostics", False)):
            contact_path = run_dir / "contact_diagnostics.csv"
            events_path = run_dir / "events.csv"
            if contact_path.is_file():
                summarize_contact_file(
                    contact_path,
                    run_dir / "contact_metrics.json",
                    events_path=events_path,
                )
        row["artifacts_complete"] = str(
            artifacts_complete(run_dir, self._extra_artifacts(spec))
        ).lower()
        if timed_out:
            row.update({
                "status": "timed_out", "outcome": "infrastructure_failure",
                "failure_class": "infrastructure", "failure_stage": "timeout",
                "error": "trial timeout",
            })
        elif terminal is not None:
            values = status_for_terminal(terminal)
            values = status_for_artifacts(run_dir, values)
            # A rerun starts from the prior CSV row.  Ensure a completed
            # terminal event cannot inherit its previous infrastructure class.
            values.setdefault("failure_class", "")
            row.update({key: str(value).lower() for key, value in values.items()})
        else:
            row.update({
                "status": "failed", "trial_success": "false", "task_success": "false",
                "outcome": "infrastructure_failure", "failure_class": "infrastructure",
                "failure_stage": "terminal_event",
                "error": "process exited without terminal event",
            })
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
        self._acquire_runtime_lock()
        try:
            results = []
            for spec in selected:
                results.append(self.run_trial(spec, attempt=self._next_attempt(spec)))
                self._write_manifest(suite_name=suite_name, suite_hash=suite_hash, started_at=started)
            self._write_manifest(suite_name=suite_name, suite_hash=suite_hash, started_at=started, finished_at=utc_now())
            return results
        finally:
            self._release_runtime_lock()


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
    parser.add_argument("--ros-domain-id", type=int)
    parser.add_argument("--gazebo-master-uri")
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
    runner = CampaignRunner(
        specs,
        campaign_dir,
        timeout_s=args.timeout,
        ros_domain_id=args.ros_domain_id,
        gazebo_master_uri=args.gazebo_master_uri,
    )
    runner.run(
        suite_name=suite["name"], suite_hash=suite_hash,
        dry_run=args.dry_run, resume=args.resume,
        rerun_failed=args.rerun_failed, max_trials=args.max_trials,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
