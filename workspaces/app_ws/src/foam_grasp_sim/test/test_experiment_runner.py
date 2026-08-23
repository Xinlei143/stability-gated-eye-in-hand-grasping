import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

from foam_grasp_sim.experiment_runner import (
    CampaignRunner,
    TERMINAL_PREFIX,
    artifacts_complete,
    parse_terminal_line,
    signal_process_group,
    stop_process_group,
    status_for_artifacts,
    status_for_terminal,
)
from foam_grasp_sim.benchmark_suite import expand_suite


def _specs():
    return expand_suite({
        "schema_version": 1,
        "name": "runner-unit",
        "defaults": {"target_model": "cube", "execute_motion": False},
        "methods": ["gated"], "trajectories": ["static"], "seeds": [42], "sweeps": [],
    })


class ExperimentRunnerTest(unittest.TestCase):
    def test_parses_only_valid_terminal_event(self):
        payload = {
            "schema_version": 1,
            "event": "TRIAL_FINISHED",
            "sim_time_ns": 3,
            "method": "gated",
            "scenario": "static",
            "seed": 42,
            "details": {"execution_mode": "plan_only"},
        }
        self.assertEqual(parse_terminal_line(TERMINAL_PREFIX + json.dumps(payload))["event"], "TRIAL_FINISHED")
        self.assertEqual(
            parse_terminal_line(
                "[object_grasp_sequence-12] "
                + TERMINAL_PREFIX
                + json.dumps({**payload, "event": "TRIAL_FAILED"})
            )["event"],
            "TRIAL_FAILED",
        )
        self.assertIsNone(parse_terminal_line("PLAN_SUCCEEDED"))
        self.assertIsNone(parse_terminal_line(TERMINAL_PREFIX + "not-json"))

    def test_status_distinguishes_trial_and_task_success(self):
        finished = status_for_terminal({"event": "TRIAL_FINISHED", "details": {"execution_mode": "plan_only"}})
        self.assertEqual(finished["status"], "finished")
        self.assertTrue(finished["trial_success"])
        self.assertFalse(finished["task_success"])
        failed = status_for_terminal({"event": "TRIAL_FAILED", "details": {"reason": "x"}})
        self.assertEqual(failed["status"], "failed")
        self.assertFalse(failed["trial_success"])

    def test_terminal_event_must_match_trial_identity(self):
        from foam_grasp_sim.experiment_runner import terminal_matches
        spec = _specs()[0]
        event = {"event": "TRIAL_FINISHED", "method": "snapshot", "scenario": "static", "seed": 42}
        self.assertFalse(terminal_matches(spec, event))
        event["method"] = "gated"
        self.assertTrue(terminal_matches(spec, event))

    def test_artifacts_require_all_four_run_files(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            self.assertFalse(artifacts_complete(run_dir))
            for name in ("metadata.json", "states.csv", "events.csv", "metrics.json"):
                (run_dir / name).write_text("{}")
            self.assertTrue(artifacts_complete(run_dir))

    def test_signal_process_group_does_not_use_global_kill(self):
        child = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            start_new_session=True,
        )
        try:
            self.assertEqual(os.getpgid(child.pid), child.pid)
            signal_process_group(child.pid, signal.SIGTERM)
            child.wait(timeout=3)
            self.assertIsNotNone(child.returncode)
        finally:
            if child.poll() is None:
                child.kill()

    def test_dry_run_never_starts_subprocess(self):
        class NeverStart:
            def __call__(self, *args, **kwargs):
                raise AssertionError("dry-run started a process")
        with tempfile.TemporaryDirectory() as directory:
            runner = CampaignRunner(_specs(), Path(directory) / "campaign", popen_factory=NeverStart())
            rows = runner.run(suite_name="runner-unit", suite_hash="x", dry_run=True)
            self.assertEqual(len(rows), 1)
            self.assertTrue((Path(directory) / "campaign" / "campaign.json").is_file())

    def test_finished_terminal_event_is_recorded(self):
        def factory(command, **kwargs):
            script = (
                "import json; "
                "print('BENCHMARK_TERMINAL_EVENT=' + json.dumps({"
                "'schema_version':1,'event':'TRIAL_FINISHED','sim_time_ns':1,"
                "'method':'gated','scenario':'static','seed':42,'details':{"
                "'execution_mode':'plan_only','task_success':False}}), flush=True)"
            )
            return subprocess.Popen([sys.executable, "-c", script], **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            runner = CampaignRunner(_specs(), Path(directory) / "campaign", popen_factory=factory)
            rows = runner.run(suite_name="runner-unit", suite_hash="x")
            self.assertEqual(rows[0]["status"], "finished")
            self.assertEqual(rows[0]["task_success"], "false")
            self.assertIn("TRIAL_FINISHED", rows[0]["terminal_event"])

    def test_physical_artifact_failure_overrides_finished_terminal_status(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            (run_dir / "metrics.json").write_text(json.dumps({
                "physical_grasp_success": False,
                "task_success": False,
            }))
            status = status_for_artifacts(
                run_dir,
                {"status": "finished", "trial_success": True, "task_success": True},
            )
            self.assertEqual(status["status"], "failed")
            self.assertFalse(status["trial_success"])
            self.assertFalse(status["task_success"])

    def test_execute_terminal_success_requires_physical_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            (run_dir / "metrics.json").write_text(json.dumps({
                "physical_grasp_success": False,
                "task_success": False,
                "trial_success": True,
            }))
            status = status_for_artifacts(
                run_dir,
                {"status": "finished", "trial_success": True, "task_success": True,
                 "execution_mode": "execute"},
            )
            self.assertEqual(status["status"], "failed")
            self.assertFalse(status["trial_success"])
            self.assertFalse(status["task_success"])

    def test_runner_waits_for_logger_flush_after_terminal_event(self):
        with tempfile.TemporaryDirectory() as directory:
            runner = CampaignRunner(_specs(), Path(directory) / "campaign")
            self.assertGreaterEqual(runner.logger_flush_grace_s, 0.5)

    def test_runner_passes_runtime_isolation_to_child(self):
        observed = {}

        def factory(command, **kwargs):
            observed.update(kwargs["env"])
            script = (
                "import json; "
                "print('BENCHMARK_TERMINAL_EVENT=' + json.dumps({"
                "'schema_version':1,'event':'TRIAL_FINISHED','sim_time_ns':1,"
                "'method':'gated','scenario':'static','seed':42,'details':{"
                "'execution_mode':'plan_only','task_success':False}}), flush=True)"
            )
            return subprocess.Popen([sys.executable, "-c", script], **kwargs)

        with tempfile.TemporaryDirectory() as directory:
            runner = CampaignRunner(
                _specs(),
                Path(directory) / "campaign",
                ros_domain_id=77,
                gazebo_master_uri="http://127.0.0.1:11447",
                popen_factory=factory,
            )
            runner.run(suite_name="runner-unit", suite_hash="x")
        self.assertEqual(observed["ROS_DOMAIN_ID"], "77")
        self.assertEqual(observed["GAZEBO_MASTER_URI"], "http://127.0.0.1:11447")
        self.assertEqual(observed["ROS_LOCALHOST_ONLY"], "1")

    def test_runner_rewrites_contact_diagnostics_to_the_run_directory(self):
        specs = expand_suite({
            "schema_version": 1,
            "name": "contact-path",
            "defaults": {
                "target_model": "cube",
                "execute_motion": True,
                "record_contact_diagnostics": True,
            },
            "methods": ["snapshot"],
            "trajectories": ["static"],
            "seeds": [42],
            "sweeps": [],
        })
        with tempfile.TemporaryDirectory() as directory:
            runner = CampaignRunner(specs, Path(directory) / "campaign")
            command, run_dir, _ = runner._command(specs[0], 1)
        self.assertIn("record_contact_diagnostics:=true", command)
        self.assertIn(
            f"contact_diagnostics_output:={run_dir / 'contact_diagnostics.csv'}",
            command,
        )

    def test_cleanup_signals_group_after_launch_leader_exits(self):
        script = (
            "import subprocess, sys, time; "
            "subprocess.Popen([sys.executable, '-c', 'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); time.sleep(30)']); "
            "time.sleep(0.2)"
        )
        leader = subprocess.Popen(
            [sys.executable, "-c", script],
            start_new_session=True,
        )
        leader.wait(timeout=3)
        cleanup = stop_process_group(leader, pgid=leader.pid, grace_s=0.2)
        self.assertIn(cleanup, {"sigint", "sigterm", "sigkill", "already_exited"})

    def test_timeout_marks_trial_and_preserves_log(self):
        def factory(command, **kwargs):
            return subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            runner = CampaignRunner(_specs(), Path(directory) / "campaign", timeout_s=0.1, popen_factory=factory)
            rows = runner.run(suite_name="runner-unit", suite_hash="x")
            self.assertEqual(rows[0]["status"], "timed_out")
            self.assertTrue(Path(rows[0]["log_path"]).is_file())

    def test_resume_skips_finished_complete_trial(self):
        class NeverStart:
            def __call__(self, *args, **kwargs):
                raise AssertionError("resume relaunched a complete trial")
        with tempfile.TemporaryDirectory() as directory:
            campaign = Path(directory) / "campaign"
            runner = CampaignRunner(_specs(), campaign, popen_factory=NeverStart())
            spec = _specs()[0]
            run_dir = campaign / "runs" / spec.run_id
            run_dir.mkdir(parents=True)
            for name in ("metadata.json", "states.csv", "events.csv", "metrics.json"):
                (run_dir / name).write_text("{}")
            runner.rows[spec.run_id] = dict(runner._base_row(spec), status="finished", artifacts_complete="true")
            runner._write_rows()
            rows = runner.run(suite_name="runner-unit", suite_hash="x", resume=True)
            self.assertEqual(rows, [])

if __name__ == "__main__":
    unittest.main()
