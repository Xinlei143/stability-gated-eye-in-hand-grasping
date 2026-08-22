"""Small, dependency-light benchmark event contract shared by ROS nodes."""

import json


SCHEMA_VERSION = 1
EVENT_TOPIC = "/foam_grasp/benchmark_event"
TERMINAL_EVENTS = frozenset(("TRIAL_FINISHED", "TRIAL_FAILED"))


def make_event(
    event,
    *,
    sim_time_ns=0,
    method="",
    scenario="",
    seed=0,
    details=None,
):
    """Return one canonical JSON event for the Stage-5 recorder."""

    if not str(event).strip():
        raise ValueError("event must not be empty")
    payload = {
        "schema_version": SCHEMA_VERSION,
        "event": str(event).strip().upper(),
        "sim_time_ns": int(sim_time_ns),
        "method": str(method),
        "scenario": str(scenario),
        "seed": int(seed),
        "details": dict(details or {}),
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


class BenchmarkEventPublisher:
    """Publish canonical events without forcing ROS imports on pure helpers."""

    def __init__(self, node, topic=EVENT_TOPIC):
        from std_msgs.msg import String

        self.node = node
        self.message_type = String
        self.publisher = node.create_publisher(String, topic, 50)

    def publish(
        self,
        event,
        *,
        method="",
        scenario="",
        seed=0,
        details=None,
        sim_time_ns=None,
    ):
        if sim_time_ns is None:
            sim_time_ns = int(self.node.get_clock().now().nanoseconds)
        message = self.message_type()
        message.data = make_event(
            event,
            sim_time_ns=sim_time_ns,
            method=method,
            scenario=scenario,
            seed=seed,
            details=details,
        )
        self.publisher.publish(message)
        return message.data


class EdgeTracker:
    """Track boolean state and report only false->true/true->false edges."""

    def __init__(self):
        self._values = {}

    def transition(self, key, value):
        value = bool(value)
        previous = self._values.get(key, False)
        self._values[key] = value
        return value != previous, value
