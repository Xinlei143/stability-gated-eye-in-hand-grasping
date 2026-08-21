"""Deterministic method selection and readiness policy for stage-4 trials."""

from collections import deque
from dataclasses import dataclass
import math
import statistics


METHODS = ("snapshot", "tracking", "gated")


def _finite_point(point):
    values = tuple(float(value) for value in point)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("point must contain three finite values")
    return values


@dataclass(frozen=True)
class MethodConfig:
    """Runtime contract shared by snapshot, tracking and gated methods."""

    method: str = "gated"
    stability_duration_s: float = 5.0
    position_spread_threshold_m: float = 0.006
    center_error_threshold_px: float = 30.0
    joint_error_threshold_rad: float = 0.030
    minimum_stable_samples: int = 25
    observation_timeout_s: float = 1.0

    def __post_init__(self):
        if self.method not in METHODS:
            raise ValueError("method must be one of " + ", ".join(METHODS))
        for name in (
            "stability_duration_s",
            "position_spread_threshold_m",
            "center_error_threshold_px",
            "joint_error_threshold_rad",
            "observation_timeout_s",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.stability_duration_s < 0.1:
            raise ValueError("stability_duration_s must be at least 0.1")
        if isinstance(self.minimum_stable_samples, bool) or not isinstance(
            self.minimum_stable_samples, int
        ):
            raise ValueError("minimum_stable_samples must be an integer")
        if self.minimum_stable_samples < 3:
            raise ValueError("minimum_stable_samples must be at least 3")


@dataclass(frozen=True)
class MethodDecision:
    point: tuple[float, float, float] | None
    ready: bool
    reset: bool = False
    reason: str = ""


class MethodPolicy:
    """Select the target used until readiness and expose an immutable latch."""

    def __init__(self, config: MethodConfig):
        self.config = config
        self.latched_point = None
        self.current_point = None
        self.last_observation_at = None
        self._stable_samples = deque()
        self._ready = False

    @property
    def stable_sample_count(self):
        return len(self._stable_samples)

    def reset(self):
        self.latched_point = None
        self.current_point = None
        self.last_observation_at = None
        self._stable_samples.clear()
        self._ready = False

    def _decision(self, *, reset=False, reason=""):
        return MethodDecision(self.current_point, self._ready, reset, reason)

    def _within_optional_error_limits(self, center_error_px, joint_error_rad):
        if center_error_px is not None:
            value = float(center_error_px)
            if not math.isfinite(value) or abs(value) > self.config.center_error_threshold_px:
                return False
        if joint_error_rad is not None:
            value = float(joint_error_rad)
            if not math.isfinite(value) or abs(value) > self.config.joint_error_threshold_rad:
                return False
        return True

    def update(
        self,
        point,
        timestamp,
        *,
        valid=True,
        center_error_px=None,
        joint_error_rad=None,
    ):
        """Consume one observation and return the selected point/readiness."""

        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if not valid:
            self._stable_samples.clear()
            self._ready = False if self.config.method == "gated" else self._ready
            return self._decision(reset=True, reason="invalid_observation")
        point = _finite_point(point)
        if self.last_observation_at is not None and timestamp < self.last_observation_at:
            raise ValueError("timestamps must be monotonically non-decreasing")

        self.last_observation_at = timestamp
        if self.config.method == "snapshot":
            if self.latched_point is None:
                self.latched_point = point
                self.current_point = point
                self._ready = True
            return self._decision(reason="snapshot_latched")

        self.current_point = point
        if self.config.method == "tracking":
            self._ready = True
            return self._decision(reason="tracking_observation")

        if not self._within_optional_error_limits(center_error_px, joint_error_rad):
            self._stable_samples.clear()
            self._ready = False
            return self._decision(reset=True, reason="error_threshold")

        self._stable_samples.append((timestamp, point))
        center = self._median_point(item[1] for item in self._stable_samples)
        spread = max(self._distance(item[1], center) for item in self._stable_samples)
        if spread > self.config.position_spread_threshold_m:
            self._stable_samples.clear()
            self._stable_samples.append((timestamp, point))
            self._ready = False
            return self._decision(reset=True, reason="position_spread")

        elapsed = self._stable_samples[-1][0] - self._stable_samples[0][0]
        self._ready = (
            elapsed >= self.config.stability_duration_s
            and len(self._stable_samples) >= self.config.minimum_stable_samples
        )
        if self._ready:
            self.latched_point = center
            self.current_point = center
        return self._decision(reason="stable_window" if self._ready else "stabilizing")

    def expire(self, timestamp):
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if self.last_observation_at is None:
            return self._decision(reset=True, reason="no_observation")
        if timestamp - self.last_observation_at <= self.config.observation_timeout_s:
            return self._decision(reason="observation_fresh")
        self.current_point = None
        self._stable_samples.clear()
        self._ready = False
        if self.config.method != "snapshot":
            self.latched_point = None
        return self._decision(reset=True, reason="observation_timeout")

    @staticmethod
    def _median_point(points):
        points = list(points)
        return tuple(statistics.median(point[axis] for point in points) for axis in range(3))

    @staticmethod
    def _distance(first, second):
        return math.sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))
