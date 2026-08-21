"""Utilities for validating and serializing benchmark event rows."""

import json


EVENT_FIELDS = (
    "schema_version",
    "sim_time_ns",
    "event",
    "method",
    "scenario",
    "seed",
    "details",
)


def parse_event(payload):
    """Parse one event message and return a normalized dictionary."""

    event = json.loads(payload) if isinstance(payload, str) else dict(payload)
    missing = [field for field in EVENT_FIELDS if field not in event]
    if missing:
        raise ValueError("benchmark event missing fields: " + ", ".join(missing))
    if int(event["schema_version"]) != 1:
        raise ValueError("unsupported benchmark event schema")
    event["sim_time_ns"] = int(event["sim_time_ns"])
    event["seed"] = int(event["seed"])
    event["event"] = str(event["event"]).upper()
    event["details"] = dict(event["details"] or {})
    return event


def event_row(event):
    event = parse_event(event)
    return {
        "schema_version": event["schema_version"],
        "sim_time_ns": event["sim_time_ns"],
        "event": event["event"],
        "method": event["method"],
        "scenario": event["scenario"],
        "seed": event["seed"],
        "details": json.dumps(event["details"], sort_keys=True, separators=(",", ":")),
    }


def main():
    # The active recorder is metrics_logger_node; this entry point remains a
    # small standalone validator for ros2 run and offline log checks.
    raise SystemExit("benchmark_event_logger is a library helper; use metrics_logger_node")
