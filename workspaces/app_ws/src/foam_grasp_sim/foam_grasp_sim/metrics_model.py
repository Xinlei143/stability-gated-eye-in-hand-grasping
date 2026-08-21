"""Pure metric aggregation for Stage-5 benchmark logs."""

import math


def _point(row, prefix):
    values = [row.get(f"{prefix}_{axis}") for axis in "xyz"]
    if any(value in (None, "") for value in values):
        return None
    try:
        values = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    return values if all(math.isfinite(value) for value in values) else None


def _distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


class MetricsAccumulator:
    """Accumulate state/event rows and compute deterministic benchmark metrics."""

    def __init__(self):
        self.states = []
        self.events = []

    def record_state(self, row):
        self.states.append(dict(row))

    def record_event(self, event):
        self.events.append(dict(event))

    def _event_time(self, name):
        times = [
            int(event["sim_time_ns"])
            for event in self.events
            if event.get("event") == name and event.get("sim_time_ns") is not None
        ]
        return min(times) if times else None

    def _states_until(self, end_ns):
        return [
            row
            for row in self.states
            if end_ns is None or int(row.get("sim_time_ns", 0)) <= end_ns
        ]

    def _estimation_error(self, row):
        ground_truth = _point(row, "target_ground_truth")
        selected = _point(row, "target_selected")
        if ground_truth is None or selected is None:
            return None
        return _distance(ground_truth, selected)

    def _tracking_error(self, row):
        ground_truth = _point(row, "target_ground_truth")
        tcp = _point(row, "tcp")
        if ground_truth is not None and tcp is not None:
            return _distance(ground_truth, tcp)
        # Plan-only trials do not have a TCP transform; retain a useful
        # deterministic proxy rather than dropping all samples.
        return self._estimation_error(row)

    def _false_ready(self, ready_ns):
        if ready_ns is None:
            return False
        window_start = ready_ns - 500_000_000
        window = [
            row
            for row in self.states
            if window_start <= int(row.get("sim_time_ns", 0)) <= ready_ns
        ]
        if len(window) < 2:
            return False
        positions = [_point(row, "target_ground_truth") for row in window]
        if any(position is None for position in positions):
            return False
        displacement = _distance(positions[0], positions[-1])
        duration = (int(window[-1]["sim_time_ns"]) - int(window[0]["sim_time_ns"])) / 1e9
        speed = displacement / duration if duration > 0 else 0.0
        return displacement > 0.002 or speed > 0.003

    def finalize(self):
        first_observation_ns = self._event_time("TARGET_OBSERVED")
        ready_ns = self._event_time("READY")
        grasp_ns = self._event_time("GRASP_STARTED")
        metric_end_ns = grasp_ns
        tracking_errors = []
        for row in self._states_until(metric_end_ns):
            if (
                first_observation_ns is not None
                and int(row.get("sim_time_ns", 0)) < first_observation_ns
            ):
                continue
            error = self._tracking_error(row)
            if error is not None:
                tracking_errors.append(error)
        ready_row = None
        grasp_row = None
        if ready_ns is not None:
            ready_candidates = [
                row for row in self.states if int(row.get("sim_time_ns", 0)) >= ready_ns
            ]
            if ready_candidates:
                ready_row = min(
                    ready_candidates,
                    key=lambda row: abs(int(row.get("sim_time_ns", 0)) - ready_ns),
                )
        if grasp_ns is not None:
            grasp_candidates = [
                row for row in self.states if int(row.get("sim_time_ns", 0)) >= grasp_ns
            ]
            if grasp_candidates:
                grasp_row = min(
                    grasp_candidates,
                    key=lambda row: abs(int(row.get("sim_time_ns", 0)) - grasp_ns),
                )

        reset_count = sum(
            event.get("event") == "GATE_RESET" for event in self.events
        )
        planning_success = any(
            event.get("event") == "PLAN_SUCCEEDED" for event in self.events
        )
        task_success = any(
            event.get("event") == "TASK_FINISHED" for event in self.events
        )
        result = {
            "tracking_rms_error_m": (
                math.sqrt(sum(error * error for error in tracking_errors) / len(tracking_errors))
                if tracking_errors
                else None
            ),
            "target_error_at_ready_m": self._estimation_error(ready_row) if ready_row else None,
            "grasp_initiation_error_m": self._estimation_error(grasp_row) if grasp_row else None,
            "time_to_ready_s": (
                (ready_ns - first_observation_ns) / 1e9
                if ready_ns is not None and first_observation_ns is not None
                else None
            ),
            "gate_resets": int(reset_count),
            "planning_success": bool(planning_success),
            "task_success": bool(task_success),
            "false_ready": bool(self._false_ready(ready_ns)),
            "state_samples": len(self.states),
            "event_count": len(self.events),
        }
        return result
