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

    def test_timeout_marks_trial_and_preserves_log(self):
        def factory(command, **kwargs):
            return subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            runner = CampaignRunner(_specs(), Path(directory) / "campaign", timeout_s=0.1, popen_factory=factory)
            rows = runner.run(suite_name="runner-unit", suite_hash="x")
            self.assertEqual(rows[0]["status"], "timed_out")
            self.assertTrue(Path(rows[0]["log_path"]).is_file())

if __name__ == "__main__":
    unittest.main()
