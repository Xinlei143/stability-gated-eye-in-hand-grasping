"""Pure delayed-observation and seeded-disturbance helpers."""

from collections import deque
from dataclasses import dataclass
import math
import random


PERCEPTION_SOURCES = ("ground_truth", "disturbed", "rgbd")


@dataclass(frozen=True)
class TimedPoint:
    timestamp: float
    point: tuple[float, float, float]


def effective_latency_seconds(source, configured_latency_seconds):
    """Ground truth is the zero-latency ideal-perception baseline."""

    return 0.0 if source == "ground_truth" else float(configured_latency_seconds)


def _finite_point(name, values):
    point = tuple(float(value) for value in values)
    if len(point) != 3 or not all(math.isfinite(value) for value in point):
        raise ValueError(f"{name} must contain three finite values")
    return point


def validate_perception_parameters(
    source,
    sampling_rate,
    latency_ms,
    noise_std_mm,
    dropout_probability,
    outlier_probability,
    outlier_range_mm,
    history_duration,
    seed,
):
    if source not in PERCEPTION_SOURCES:
        raise ValueError(
            "source must be one of " + ", ".join(PERCEPTION_SOURCES)
        )
    sampling_rate = float(sampling_rate)
    history_duration = float(history_duration)
    latency_ms = float(latency_ms)
    noise_std_mm = float(noise_std_mm)
    outlier_range_mm = float(outlier_range_mm)
    dropout_probability = float(dropout_probability)
    outlier_probability = float(outlier_probability)
    if not math.isfinite(sampling_rate) or not 1.0 <= sampling_rate <= 200.0:
        raise ValueError("sampling_rate must be finite and within 1--200 Hz")
    if not math.isfinite(latency_ms) or not 0.0 <= latency_ms <= 5000.0:
        raise ValueError("latency_ms must be finite and within 0--5000")
    if not math.isfinite(noise_std_mm) or not 0.0 <= noise_std_mm <= 100.0:
        raise ValueError("noise_std_mm must be finite and within 0--100")
    if not math.isfinite(outlier_range_mm) or not 0.0 <= outlier_range_mm <= 500.0:
        raise ValueError("outlier_range_mm must be finite and within 0--500")
    for name, probability in (
        ("dropout_probability", dropout_probability),
        ("outlier_probability", outlier_probability),
    ):
        if not math.isfinite(probability) or not 0.0 <= probability <= 1.0:
            raise ValueError(f"{name} must be finite and within [0, 1]")
    if not math.isfinite(history_duration) or history_duration <= latency_ms / 1000.0:
        raise ValueError("history_duration must exceed configured latency")
    if isinstance(seed, bool):
        raise ValueError("seed must be an integer")
    try:
        seed = int(seed)
    except (TypeError, ValueError) as error:
        raise ValueError("seed must be an integer") from error
    if not 0 <= seed <= 2**31 - 1:
        raise ValueError("seed must be within [0, 2147483647]")
    return {
        "source": source,
        "sampling_rate": sampling_rate,
        "latency_seconds": latency_ms / 1000.0,
        "noise_std_m": noise_std_mm / 1000.0,
        "dropout_probability": dropout_probability,
        "outlier_probability": outlier_probability,
        "outlier_range_m": outlier_range_mm / 1000.0,
        "history_duration": history_duration,
        "seed": seed,
    }


class DelayedPointBuffer:
    """Store timestamped source points and select a delayed sample."""

    def __init__(self, history_duration):
        self.history_duration = float(history_duration)
        self._points = deque()

    def append(self, timestamp, point):
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        point = _finite_point("point", point)
        if self._points and timestamp < self._points[-1].timestamp:
            raise ValueError("timestamps must be monotonically non-decreasing")
        self._points.append(TimedPoint(timestamp, point))

    def prune(self, current_time):
        cutoff = float(current_time) - self.history_duration
        # Retain one point before the cutoff so latency selection remains
        # defined when the source rate is lower than the sampling rate.
        while len(self._points) > 1 and self._points[1].timestamp < cutoff:
            self._points.popleft()

    def latest_at_or_before(self, timestamp):
        cutoff = float(timestamp)
        for point in reversed(self._points):
            if point.timestamp <= cutoff:
                return point
        return None


class DisturbanceModel:
    """Apply seeded dropout, Gaussian noise and uniform outliers in order."""

    def __init__(
        self,
        seed,
        noise_std_m,
        dropout_probability,
        outlier_probability,
        outlier_range_m,
    ):
        self.random = random.Random(int(seed))
        self.noise_std_m = float(noise_std_m)
        self.dropout_probability = float(dropout_probability)
        self.outlier_probability = float(outlier_probability)
        self.outlier_range_m = float(outlier_range_m)

    def apply(self, point):
        point = _finite_point("point", point)
        if (
            self.dropout_probability > 0.0
            and self.random.random() < self.dropout_probability
        ):
            return None
        observed = point
        if self.noise_std_m > 0.0:
            observed = tuple(
                value + self.random.gauss(0.0, self.noise_std_m)
                for value in observed
            )
        if (
            self.outlier_probability > 0.0
            and self.random.random() < self.outlier_probability
        ):
            observed = tuple(
                value
                + self.random.uniform(-self.outlier_range_m, self.outlier_range_m)
                for value in observed
            )
        return observed
