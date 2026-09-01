"""
Unified WT901BLECL Fall-Evidence and SOS Node for ROS 2
======================================================

One BLE connection and one common signal-processing pipeline are shared by
two independent detectors:

1. Fall evidence detector
   - Uses a higher dynamic threshold.
   - Publishes one finalized strong impact as JSON on /fall/evidence.
   - The message is evidence for fall fusion, not a final fall decision.

2. SOS detector
   - Uses a lower dynamic threshold.
   - Collects several finalized impacts.
   - Detects strong, irregular repeated impacts.
   - Rejects footstep-like regular repetition.
   - Publishes a confirmed SOS JSON on /sos/detected.

ROS message type:
    std_msgs/msg/String

The JSON inside String.data follows the agreed fall-evidence schema:

{
  "schema_version": 1,
  "event_type": "fall_evidence",
  "modality": "imu",
  "sensor_id": "floor_imu_01",
  "ros_stamp": {"sec": ..., "nanosec": ...},
  "sensor_stamp": {"unix_time": ..., "ts_iso": "..."},
  "confidence": 0.82,
  "severity": "strong",
  "modality_data": {...}
}

Run after sourcing ROS 2:
    python3 imu_fall_sos_node.py

Package execution:
    ros2 run imu_fall_sos_ros2 imu_fall_sos_node
"""

from __future__ import annotations

import asyncio
import csv
import json
import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from bleak import BleakClient
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# 1. DEFAULT DEVICE AND TOPIC SETTINGS
# ============================================================

DEFAULT_DEVICE_ADDRESS = "DD:D6:0F:01:23:A5"
DEFAULT_NOTIFY_CHARACTERISTIC_UUID = ""
DEFAULT_SENSOR_ID = "floor_imu_01"

DEFAULT_FALL_TOPIC = "/fall/evidence"
DEFAULT_SOS_TOPIC = "/sos/detected"

PROCESS_TIMER_SEC = 0.02
SOS_TIMEOUT_TIMER_SEC = 0.10

DEFAULT_SAVE_CSV = True
DEFAULT_LOG_DIRECTORY = "."



# ============================================================
# 2. COMMON SIGNAL-PROCESSING SETTINGS
# ============================================================

# Recent duration used for baseline, noise and threshold estimation.
WINDOW_SEC = 5.0

# Envelope smoothing:
# current vibration 30% + previous envelope 70%
ENVELOPE_ALPHA = 0.30

# Expands the difference between ordinary vibration and impacts.
SCORE_BOOST_GAIN = 16.0

# Prevents division by an almost-zero noise estimate.
NOISE_EPS = 0.0015


# ============================================================
# 3. FALL DETECTOR SETTINGS
# ============================================================

# Fall evidence requires a stronger impact than SOS tapping.
DEFAULT_FALL_THRESHOLD_K = 6.0
DEFAULT_FALL_MIN_SCORE_THRESHOLD = 25.0
DEFAULT_FALL_MAX_SCORE_THRESHOLD = 80.0
DEFAULT_FALL_MIN_SIGNAL_G = 0.004
DEFAULT_FALL_MIN_PEAK_DISTANCE_SEC = 0.40

# peak_ratio = boosted_score / dynamic_threshold
DEFAULT_FALL_MEDIUM_RATIO = 1.50
DEFAULT_FALL_STRONG_RATIO = 2.50

# Confidence becomes 1.0 at or above this ratio.
DEFAULT_FALL_CONFIDENCE_FULL_RATIO = 3.00


# ============================================================
# 4. SOS PEAK DETECTOR SETTINGS
# ============================================================

# SOS uses a lower threshold so repeated intentional hits are not missed.
DEFAULT_SOS_THRESHOLD_K = 5.0
DEFAULT_SOS_MIN_SCORE_THRESHOLD = 15.0
DEFAULT_SOS_MAX_SCORE_THRESHOLD = 60.0
DEFAULT_SOS_MIN_SIGNAL_G = 0.002
DEFAULT_SOS_MIN_PEAK_DISTANCE_SEC = 0.25


# ============================================================
# 5. FINAL SOS PATTERN SETTINGS
# ============================================================

DEFAULT_MIN_SOS_HITS = 4
DEFAULT_SOS_WINDOW_SEC = 3.0

DEFAULT_MIN_IOI_SEC = 0.25
DEFAULT_MAX_IOI_SEC = 1.00

DEFAULT_MIN_SEQUENCE_DURATION_SEC = 0.70
DEFAULT_MAX_SEQUENCE_DURATION_SEC = 3.00

DEFAULT_MIN_HIT_RATE_HZ = 1.00
DEFAULT_MIN_MEDIAN_PEAK_RATIO = 1.05

# std(IOI) / mean(IOI)
DEFAULT_MIN_SOS_IOI_CV = 0.20

# Regular repetition tends to have a higher autocorrelation value.
DEFAULT_MAX_SOS_AUTOCORR_SCORE = 0.60

DEFAULT_FOOTSTEP_IOI_CV_MAX = 0.15
DEFAULT_FOOTSTEP_AUTOCORR_MIN = 0.60

DEFAULT_SEQUENCE_QUIET_RESET_SEC = 1.20
DEFAULT_SOS_COOLDOWN_SEC = 5.0

AUTOCORR_BIN_SEC = 0.01


# ============================================================
# 6. DATA CLASSES
# ============================================================

@dataclass
class PeakCandidate:
    event_time: float
    ts_iso: str

    boosted_score: float
    raw_score: float
    threshold: float
    peak_ratio: float

    envelope_g: float
    noise_level_g: float
    acc_mag_g: float

    sample: dict[str, Any]


@dataclass
class PeakDetectorState:
    name: str
    min_peak_distance_sec: float

    in_peak: bool = False
    candidate: PeakCandidate | None = None
    last_peak_time: float = -999.0


@dataclass
class SosPeak:
    event_time: float
    ts_iso: str

    boosted_score: float
    threshold: float
    peak_ratio: float

    envelope_g: float
    noise_level_g: float


# ============================================================
# 7. GENERAL HELPER FUNCTIONS
# ============================================================

def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default

    if not math.isfinite(number):
        return default

    return number


def clipped(value: float, low: float, high: float) -> float:
    return float(np.clip(value, low, high))


def coefficient_of_variation(values: np.ndarray) -> float:
    if len(values) == 0:
        return 0.0

    mean_value = float(np.mean(values))

    if mean_value <= 1e-12:
        return 0.0

    return float(np.std(values) / mean_value)


# ============================================================
# 8. WT901BLECL PACKET PARSING
# ============================================================

def to_int16(low: int, high: int) -> int:
    value = (high << 8) | low

    if value >= 32768:
        value -= 65536

    return value


def parse_packet(
    packet: bytes | bytearray,
) -> dict[str, Any] | None:
    if len(packet) != 20:
        return None

    if packet[0] != 0x55 or packet[1] != 0x61:
        return None

    ax_raw = to_int16(packet[2], packet[3])
    ay_raw = to_int16(packet[4], packet[5])
    az_raw = to_int16(packet[6], packet[7])

    wx_raw = to_int16(packet[8], packet[9])
    wy_raw = to_int16(packet[10], packet[11])
    wz_raw = to_int16(packet[12], packet[13])

    roll_raw = to_int16(packet[14], packet[15])
    pitch_raw = to_int16(packet[16], packet[17])
    yaw_raw = to_int16(packet[18], packet[19])

    ax = ax_raw / 32768.0 * 16.0
    ay = ay_raw / 32768.0 * 16.0
    az = az_raw / 32768.0 * 16.0

    wx = wx_raw / 32768.0 * 2000.0
    wy = wy_raw / 32768.0 * 2000.0
    wz = wz_raw / 32768.0 * 2000.0

    roll = roll_raw / 32768.0 * 180.0
    pitch = pitch_raw / 32768.0 * 180.0
    yaw = yaw_raw / 32768.0 * 180.0

    acc_mag = math.sqrt(
        ax * ax
        + ay * ay
        + az * az
    )

    gyro_mag = math.sqrt(
        wx * wx
        + wy * wy
        + wz * wz
    )

    unix_time = time.time()

    return {
        "unix_time": unix_time,
        "ts_iso": datetime.fromtimestamp(
            unix_time,
            tz=timezone.utc,
        ).isoformat(timespec="milliseconds"),

        "ax": ax,
        "ay": ay,
        "az": az,

        "wx": wx,
        "wy": wy,
        "wz": wz,

        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,

        "acc_mag": acc_mag,
        "gyro_mag": gyro_mag,
    }


# ============================================================
# 9. COMMON NOISE AND THRESHOLD FUNCTIONS
# ============================================================

def robust_noise(values: list[float]) -> float:
    """
    Estimate ordinary floor vibration using median and MAD.

    A large impact has less influence than it would with a simple mean.
    """
    if len(values) < 10:
        return NOISE_EPS

    arr = np.asarray(values, dtype=float)

    median_value = np.median(arr)
    mad = np.median(
        np.abs(arr - median_value)
    )

    robust_std = 1.4826 * mad
    noise_level = median_value + robust_std

    return max(
        float(noise_level),
        NOISE_EPS,
    )


def dynamic_score_threshold(
    values: list[float],
    threshold_k: float,
    min_threshold: float,
    max_threshold: float,
) -> float:
    """
    Calculate one adaptive threshold from recent boosted-score values.

    Fall and SOS call this function with different K, minimum and maximum
    settings, so their thresholds remain separate.
    """
    if len(values) < 10:
        return min_threshold

    arr = np.asarray(values, dtype=float)

    # Ignore the largest 10 percent so a strong impact does not immediately
    # raise the threshold too much.
    upper = np.percentile(arr, 90)
    trimmed = arr[arr <= upper]

    if len(trimmed) < 10:
        return min_threshold

    median_value = np.median(trimmed)
    mad = np.median(
        np.abs(trimmed - median_value)
    )
    robust_std = 1.4826 * mad

    threshold = (
        median_value
        + threshold_k * robust_std
    )

    return clipped(
        float(threshold),
        min_threshold,
        max_threshold,
    )


# ============================================================
# 10. PEAK-DETECTOR FUNCTION
# ============================================================

def build_peak_candidate(
    sample: dict[str, Any],
    boosted_score: float,
    raw_score: float,
    threshold: float,
    envelope_g: float,
    noise_level_g: float,
) -> PeakCandidate:
    peak_ratio = (
        boosted_score
        / max(threshold, 1e-12)
    )

    return PeakCandidate(
        event_time=float(
            sample["unix_time"]
        ),
        ts_iso=str(
            sample["ts_iso"]
        ),

        boosted_score=float(
            boosted_score
        ),
        raw_score=float(
            raw_score
        ),
        threshold=float(
            threshold
        ),
        peak_ratio=float(
            peak_ratio
        ),

        envelope_g=float(
            envelope_g
        ),
        noise_level_g=float(
            noise_level_g
        ),
        acc_mag_g=float(
            sample["acc_mag"]
        ),

        sample=sample.copy(),
    )


def update_peak_detector(
    detector: PeakDetectorState,
    is_above: bool,
    sample: dict[str, Any],
    boosted_score: float,
    raw_score: float,
    threshold: float,
    envelope_g: float,
    noise_level_g: float,
) -> PeakCandidate | None:
    """
    Return one finalized PeakCandidate when a threshold-crossing region ends.

    It does not return every sample over the threshold. It keeps the largest
    point in the region and returns only that one point.
    """
    event_time = float(
        sample["unix_time"]
    )

    if is_above:
        if not detector.in_peak:
            if (
                event_time
                - detector.last_peak_time
                >= detector.min_peak_distance_sec
            ):
                detector.in_peak = True
                detector.candidate = build_peak_candidate(
                    sample=sample,
                    boosted_score=boosted_score,
                    raw_score=raw_score,
                    threshold=threshold,
                    envelope_g=envelope_g,
                    noise_level_g=noise_level_g,
                )

        elif (
            detector.candidate is None
            or boosted_score
            > detector.candidate.boosted_score
        ):
            detector.candidate = build_peak_candidate(
                sample=sample,
                boosted_score=boosted_score,
                raw_score=raw_score,
                threshold=threshold,
                envelope_g=envelope_g,
                noise_level_g=noise_level_g,
            )

        return None

    if detector.in_peak:
        finalized = detector.candidate

        detector.in_peak = False
        detector.candidate = None

        if finalized is not None:
            detector.last_peak_time = (
                finalized.event_time
            )

        return finalized

    return None


# ============================================================
# 11. SOS AUTOCORRELATION
# ============================================================

def impulse_autocorrelation_score(
    event_times: list[float],
    min_lag_sec: float,
    max_lag_sec: float,
) -> float:
    """
    Convert Peak times to an impulse train and calculate autocorrelation.

    Regular footsteps:
        repeated spacing produces a larger autocorrelation value.

    Irregular SOS impacts:
        no stable repeating lag, so the value tends to be smaller.
    """
    if len(event_times) < 3:
        return 0.0

    relative_times = (
        np.asarray(
            event_times,
            dtype=float,
        )
        - float(event_times[0])
    )

    signal_length = int(
        math.ceil(
            (
                float(relative_times[-1])
                + max_lag_sec
            )
            / AUTOCORR_BIN_SEC
        )
    ) + 2

    impulse = np.zeros(
        max(signal_length, 3),
        dtype=float,
    )

    indices = np.rint(
        relative_times
        / AUTOCORR_BIN_SEC
    ).astype(int)

    indices = np.clip(
        indices,
        0,
        len(impulse) - 1,
    )

    impulse[indices] = 1.0

    autocorrelation = np.correlate(
        impulse,
        impulse,
        mode="full",
    )

    autocorrelation = autocorrelation[
        len(impulse) - 1:
    ]

    zero_lag = float(
        autocorrelation[0]
    )

    if zero_lag <= 0:
        return 0.0

    min_index = max(
        1,
        int(
            round(
                min_lag_sec
                / AUTOCORR_BIN_SEC
            )
        ),
    )

    max_index = min(
        len(autocorrelation) - 1,
        int(
            round(
                max_lag_sec
                / AUTOCORR_BIN_SEC
            )
        ),
    )

    if min_index > max_index:
        return 0.0

    score = float(
        np.max(
            autocorrelation[
                min_index:max_index + 1
            ]
        )
        / zero_lag
    )

    return clipped(
        score,
        0.0,
        1.0,
    )


# ============================================================
# 12. UNIFIED ROS 2 NODE
# ============================================================

class ImuFallSosNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "imu_fall_sos_node"
        )

        self._declare_parameters()
        self._read_parameters()

        # Two outputs from one node.
        self.fall_publisher = self.create_publisher(
            String,
            self.fall_topic,
            10,
        )

        self.sos_publisher = self.create_publisher(
            String,
            self.sos_topic,
            10,
        )

        # BLE callback thread writes to this queue.
        # ROS timer processes the queue.
        self.data_queue: queue.Queue[
            dict[str, Any]
        ] = queue.Queue()

        self.stop_event = threading.Event()
        self.packet_buffer = bytearray()

        # Common adaptive-processing history.
        self.times: deque[float] = deque()
        self.acc_mags: deque[float] = deque()
        self.envelopes: deque[float] = deque()
        self.boosted_scores: deque[float] = deque()

        self.prev_envelope = 0.0

        # Separate Peak states.
        self.fall_peak_detector = PeakDetectorState(
            name="fall",
            min_peak_distance_sec=(
                self.fall_min_peak_distance_sec
            ),
        )

        self.sos_peak_detector = PeakDetectorState(
            name="sos",
            min_peak_distance_sec=(
                self.sos_min_peak_distance_sec
            ),
        )

        # SOS sequence state.
        self.sos_peaks: deque[
            SosPeak
        ] = deque()

        self.last_sos_peak_wall_time = -999.0
        self.last_sos_publish_wall_time = -999.0

        self.fall_evidence_count = 0
        self.sos_count = 0


        # CSV.
        self.sample_csv_file = None
        self.sample_csv_writer = None

        self.event_csv_file = None
        self.event_csv_writer = None

        if self.save_csv:
            self._open_csv_files()

        # Timers.
        self.process_timer = self.create_timer(
            PROCESS_TIMER_SEC,
            self._process_queue,
        )

        self.sos_timeout_timer = self.create_timer(
            SOS_TIMEOUT_TIMER_SEC,
            self._check_sos_timeout,
        )

        # One BLE connection for both detectors.
        self.ble_thread = threading.Thread(
            target=self._run_ble_thread,
            daemon=True,
        )
        self.ble_thread.start()

        self._log_startup_settings()

    # --------------------------------------------------------
    # ROS parameters
    # --------------------------------------------------------

    def _declare_parameters(self) -> None:
        # Device and output.
        self.declare_parameter(
            "device_address",
            DEFAULT_DEVICE_ADDRESS,
        )
        self.declare_parameter(
            "notify_characteristic_uuid",
            DEFAULT_NOTIFY_CHARACTERISTIC_UUID,
        )
        self.declare_parameter(
            "sensor_id",
            DEFAULT_SENSOR_ID,
        )
        self.declare_parameter(
            "fall_topic",
            DEFAULT_FALL_TOPIC,
        )
        self.declare_parameter(
            "sos_topic",
            DEFAULT_SOS_TOPIC,
        )

        # Logging.
        self.declare_parameter(
            "save_csv",
            DEFAULT_SAVE_CSV,
        )
        self.declare_parameter(
            "log_directory",
            DEFAULT_LOG_DIRECTORY,
        )

        # Fall threshold.
        self.declare_parameter(
            "fall_threshold_k",
            DEFAULT_FALL_THRESHOLD_K,
        )
        self.declare_parameter(
            "fall_min_score_threshold",
            DEFAULT_FALL_MIN_SCORE_THRESHOLD,
        )
        self.declare_parameter(
            "fall_max_score_threshold",
            DEFAULT_FALL_MAX_SCORE_THRESHOLD,
        )
        self.declare_parameter(
            "fall_min_signal_g",
            DEFAULT_FALL_MIN_SIGNAL_G,
        )
        self.declare_parameter(
            "fall_min_peak_distance_sec",
            DEFAULT_FALL_MIN_PEAK_DISTANCE_SEC,
        )

        # Fall confidence and severity.
        self.declare_parameter(
            "fall_medium_ratio",
            DEFAULT_FALL_MEDIUM_RATIO,
        )
        self.declare_parameter(
            "fall_strong_ratio",
            DEFAULT_FALL_STRONG_RATIO,
        )
        self.declare_parameter(
            "fall_confidence_full_ratio",
            DEFAULT_FALL_CONFIDENCE_FULL_RATIO,
        )

        # SOS Peak threshold.
        self.declare_parameter(
            "sos_threshold_k",
            DEFAULT_SOS_THRESHOLD_K,
        )
        self.declare_parameter(
            "sos_min_score_threshold",
            DEFAULT_SOS_MIN_SCORE_THRESHOLD,
        )
        self.declare_parameter(
            "sos_max_score_threshold",
            DEFAULT_SOS_MAX_SCORE_THRESHOLD,
        )
        self.declare_parameter(
            "sos_min_signal_g",
            DEFAULT_SOS_MIN_SIGNAL_G,
        )
        self.declare_parameter(
            "sos_min_peak_distance_sec",
            DEFAULT_SOS_MIN_PEAK_DISTANCE_SEC,
        )

        # SOS sequence.
        self.declare_parameter(
            "min_sos_hits",
            DEFAULT_MIN_SOS_HITS,
        )
        self.declare_parameter(
            "sos_window_sec",
            DEFAULT_SOS_WINDOW_SEC,
        )
        self.declare_parameter(
            "min_ioi_sec",
            DEFAULT_MIN_IOI_SEC,
        )
        self.declare_parameter(
            "max_ioi_sec",
            DEFAULT_MAX_IOI_SEC,
        )
        self.declare_parameter(
            "min_sequence_duration_sec",
            DEFAULT_MIN_SEQUENCE_DURATION_SEC,
        )
        self.declare_parameter(
            "max_sequence_duration_sec",
            DEFAULT_MAX_SEQUENCE_DURATION_SEC,
        )
        self.declare_parameter(
            "min_hit_rate_hz",
            DEFAULT_MIN_HIT_RATE_HZ,
        )
        self.declare_parameter(
            "min_median_peak_ratio",
            DEFAULT_MIN_MEDIAN_PEAK_RATIO,
        )
        self.declare_parameter(
            "min_sos_ioi_cv",
            DEFAULT_MIN_SOS_IOI_CV,
        )
        self.declare_parameter(
            "max_sos_autocorr_score",
            DEFAULT_MAX_SOS_AUTOCORR_SCORE,
        )
        self.declare_parameter(
            "footstep_ioi_cv_max",
            DEFAULT_FOOTSTEP_IOI_CV_MAX,
        )
        self.declare_parameter(
            "footstep_autocorr_min",
            DEFAULT_FOOTSTEP_AUTOCORR_MIN,
        )
        self.declare_parameter(
            "sequence_quiet_reset_sec",
            DEFAULT_SEQUENCE_QUIET_RESET_SEC,
        )
        self.declare_parameter(
            "sos_cooldown_sec",
            DEFAULT_SOS_COOLDOWN_SEC,
        )

    def _read_parameters(self) -> None:
        self.device_address = str(
            self.get_parameter(
                "device_address"
            ).value
        )

        self.notify_characteristic_uuid = str(
            self.get_parameter(
                "notify_characteristic_uuid"
            ).value
        ).strip()

        self.sensor_id = str(
            self.get_parameter(
                "sensor_id"
            ).value
        )

        self.fall_topic = str(
            self.get_parameter(
                "fall_topic"
            ).value
        )

        self.sos_topic = str(
            self.get_parameter(
                "sos_topic"
            ).value
        )

        self.save_csv = bool(
            self.get_parameter(
                "save_csv"
            ).value
        )

        self.log_directory = Path(
            str(
                self.get_parameter(
                    "log_directory"
                ).value
            )
        ).expanduser().resolve()

        # Fall.
        self.fall_threshold_k = safe_float(
            self.get_parameter(
                "fall_threshold_k"
            ).value,
            DEFAULT_FALL_THRESHOLD_K,
        )

        self.fall_min_score_threshold = safe_float(
            self.get_parameter(
                "fall_min_score_threshold"
            ).value,
            DEFAULT_FALL_MIN_SCORE_THRESHOLD,
        )

        self.fall_max_score_threshold = safe_float(
            self.get_parameter(
                "fall_max_score_threshold"
            ).value,
            DEFAULT_FALL_MAX_SCORE_THRESHOLD,
        )

        self.fall_min_signal_g = safe_float(
            self.get_parameter(
                "fall_min_signal_g"
            ).value,
            DEFAULT_FALL_MIN_SIGNAL_G,
        )

        self.fall_min_peak_distance_sec = safe_float(
            self.get_parameter(
                "fall_min_peak_distance_sec"
            ).value,
            DEFAULT_FALL_MIN_PEAK_DISTANCE_SEC,
        )

        self.fall_medium_ratio = safe_float(
            self.get_parameter(
                "fall_medium_ratio"
            ).value,
            DEFAULT_FALL_MEDIUM_RATIO,
        )

        self.fall_strong_ratio = safe_float(
            self.get_parameter(
                "fall_strong_ratio"
            ).value,
            DEFAULT_FALL_STRONG_RATIO,
        )

        self.fall_confidence_full_ratio = safe_float(
            self.get_parameter(
                "fall_confidence_full_ratio"
            ).value,
            DEFAULT_FALL_CONFIDENCE_FULL_RATIO,
        )

        # SOS Peak.
        self.sos_threshold_k = safe_float(
            self.get_parameter(
                "sos_threshold_k"
            ).value,
            DEFAULT_SOS_THRESHOLD_K,
        )

        self.sos_min_score_threshold = safe_float(
            self.get_parameter(
                "sos_min_score_threshold"
            ).value,
            DEFAULT_SOS_MIN_SCORE_THRESHOLD,
        )

        self.sos_max_score_threshold = safe_float(
            self.get_parameter(
                "sos_max_score_threshold"
            ).value,
            DEFAULT_SOS_MAX_SCORE_THRESHOLD,
        )

        self.sos_min_signal_g = safe_float(
            self.get_parameter(
                "sos_min_signal_g"
            ).value,
            DEFAULT_SOS_MIN_SIGNAL_G,
        )

        self.sos_min_peak_distance_sec = safe_float(
            self.get_parameter(
                "sos_min_peak_distance_sec"
            ).value,
            DEFAULT_SOS_MIN_PEAK_DISTANCE_SEC,
        )

        # SOS sequence.
        self.min_sos_hits = int(
            self.get_parameter(
                "min_sos_hits"
            ).value
        )

        self.sos_window_sec = safe_float(
            self.get_parameter(
                "sos_window_sec"
            ).value,
            DEFAULT_SOS_WINDOW_SEC,
        )

        self.min_ioi_sec = safe_float(
            self.get_parameter(
                "min_ioi_sec"
            ).value,
            DEFAULT_MIN_IOI_SEC,
        )

        self.max_ioi_sec = safe_float(
            self.get_parameter(
                "max_ioi_sec"
            ).value,
            DEFAULT_MAX_IOI_SEC,
        )

        self.min_sequence_duration_sec = safe_float(
            self.get_parameter(
                "min_sequence_duration_sec"
            ).value,
            DEFAULT_MIN_SEQUENCE_DURATION_SEC,
        )

        self.max_sequence_duration_sec = safe_float(
            self.get_parameter(
                "max_sequence_duration_sec"
            ).value,
            DEFAULT_MAX_SEQUENCE_DURATION_SEC,
        )

        self.min_hit_rate_hz = safe_float(
            self.get_parameter(
                "min_hit_rate_hz"
            ).value,
            DEFAULT_MIN_HIT_RATE_HZ,
        )

        self.min_median_peak_ratio = safe_float(
            self.get_parameter(
                "min_median_peak_ratio"
            ).value,
            DEFAULT_MIN_MEDIAN_PEAK_RATIO,
        )

        self.min_sos_ioi_cv = safe_float(
            self.get_parameter(
                "min_sos_ioi_cv"
            ).value,
            DEFAULT_MIN_SOS_IOI_CV,
        )

        self.max_sos_autocorr_score = safe_float(
            self.get_parameter(
                "max_sos_autocorr_score"
            ).value,
            DEFAULT_MAX_SOS_AUTOCORR_SCORE,
        )

        self.footstep_ioi_cv_max = safe_float(
            self.get_parameter(
                "footstep_ioi_cv_max"
            ).value,
            DEFAULT_FOOTSTEP_IOI_CV_MAX,
        )

        self.footstep_autocorr_min = safe_float(
            self.get_parameter(
                "footstep_autocorr_min"
            ).value,
            DEFAULT_FOOTSTEP_AUTOCORR_MIN,
        )

        self.sequence_quiet_reset_sec = safe_float(
            self.get_parameter(
                "sequence_quiet_reset_sec"
            ).value,
            DEFAULT_SEQUENCE_QUIET_RESET_SEC,
        )

        self.sos_cooldown_sec = safe_float(
            self.get_parameter(
                "sos_cooldown_sec"
            ).value,
            DEFAULT_SOS_COOLDOWN_SEC,
        )

    def _log_startup_settings(self) -> None:
        self.get_logger().info(
            f"[IMU] Fall/SOS detector started | sensor={self.sensor_id}"
        )

    # --------------------------------------------------------
    # CSV
    # --------------------------------------------------------

    def _open_csv_files(self) -> None:
        self.log_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        sample_path = (
            self.log_directory
            / "imu_fall_sos_stream.csv"
        )

        event_path = (
            self.log_directory
            / "imu_fall_sos_events.csv"
        )

        sample_fields = [
            "unix_time",
            "ts_iso",

            "ax",
            "ay",
            "az",

            "wx",
            "wy",
            "wz",

            "roll",
            "pitch",
            "yaw",

            "acc_mag",
            "gyro_mag",

            "baseline_acc",
            "vib",
            "envelope",
            "noise_level",

            "raw_score",
            "boosted_score",

            "fall_threshold",
            "sos_threshold",

            "fall_is_above",
            "sos_is_above",

            "fall_final_peak",
            "sos_final_peak",

            "sos_sequence_hits",
        ]

        self.sample_csv_file = open(
            sample_path,
            "w",
            newline="",
            encoding="utf-8",
        )

        self.sample_csv_writer = csv.DictWriter(
            self.sample_csv_file,
            fieldnames=sample_fields,
        )

        self.sample_csv_writer.writeheader()

        event_fields = [
            "logged_at",
            "event_type",
            "confidence",
            "severity",
            "payload_json",
        ]

        self.event_csv_file = open(
            event_path,
            "w",
            newline="",
            encoding="utf-8",
        )

        self.event_csv_writer = csv.DictWriter(
            self.event_csv_file,
            fieldnames=event_fields,
        )

        self.event_csv_writer.writeheader()


    def _write_event_csv(
        self,
        payload: dict[str, Any],
    ) -> None:
        if self.event_csv_writer is None:
            return

        self.event_csv_writer.writerow({
            "logged_at": time.time(),
            "event_type": payload.get(
                "event_type",
                "",
            ),
            "confidence": payload.get(
                "confidence",
                "",
            ),
            "severity": payload.get(
                "severity",
                "",
            ),
            "payload_json": json.dumps(
                payload,
                ensure_ascii=False,
                separators=(",", ":"),
            ),
        })

        if self.event_csv_file is not None:
            self.event_csv_file.flush()

    # --------------------------------------------------------
    # BLE
    # --------------------------------------------------------

    def _handle_ble_data(
        self,
        data: bytearray,
    ) -> None:
        self.packet_buffer.extend(data)

        while len(self.packet_buffer) >= 20:
            start_index = -1

            for index in range(
                len(self.packet_buffer) - 1
            ):
                if (
                    self.packet_buffer[index] == 0x55
                    and self.packet_buffer[index + 1] == 0x61
                ):
                    start_index = index
                    break

            if start_index == -1:
                self.packet_buffer.clear()
                return

            if start_index > 0:
                del self.packet_buffer[
                    :start_index
                ]

            if len(self.packet_buffer) < 20:
                return

            packet = self.packet_buffer[:20]
            del self.packet_buffer[:20]

            parsed = parse_packet(packet)

            if parsed is not None:
                self.data_queue.put(parsed)

    async def _find_notify_characteristic(
        self,
        client: BleakClient,
    ) -> Any:
        if self.notify_characteristic_uuid:
            characteristic = (
                client.services.get_characteristic(
                    self.notify_characteristic_uuid
                )
            )

            if characteristic is None:
                raise RuntimeError(
                    "Configured notify characteristic "
                    f"was not found: "
                    f"{self.notify_characteristic_uuid}"
                )

            return characteristic

        for service in client.services:
            for characteristic in service.characteristics:
                if (
                    "notify"
                    in characteristic.properties
                ):
                    return characteristic

        raise RuntimeError(
            "No BLE notify characteristic was found"
        )

    async def _ble_connection_loop(self) -> None:
        first_attempt = True

        while not self.stop_event.is_set():
            try:
                if first_attempt:
                    self.get_logger().info(
                        f"[BLE] connecting... {self.device_address}"
                    )
                    first_attempt = False

                async with BleakClient(
                    self.device_address
                ) as client:
                    self.get_logger().info(
                        "[BLE] Connected | monitoring started"
                    )

                    notify_characteristic = (
                        await self._find_notify_characteristic(
                            client
                        )
                    )

                    def callback(
                        sender: Any,
                        data: bytearray,
                    ) -> None:
                        del sender
                        self._handle_ble_data(data)

                    await client.start_notify(
                        notify_characteristic,
                        callback,
                    )

                    while (
                        not self.stop_event.is_set()
                        and client.is_connected
                    ):
                        await asyncio.sleep(0.05)

                    if client.is_connected:
                        await client.stop_notify(
                            notify_characteristic
                        )

                    if not self.stop_event.is_set():
                        self.get_logger().warning(
                            "[BLE] disconnected | reconnecting..."
                        )

            except Exception as exc:
                if self.stop_event.is_set():
                    break

                self.get_logger().warning(
                    "[BLE] Connection failed | retrying in 3s"
                )
                await asyncio.sleep(3.0)

    def _run_ble_thread(self) -> None:
        try:
            asyncio.run(
                self._ble_connection_loop()
            )
        except Exception as exc:
            self.get_logger().error(
                f"BLE thread stopped: {exc}"
            )

    # --------------------------------------------------------
    # Common real-time processing
    # --------------------------------------------------------

    def _process_queue(self) -> None:
        processed_count = 0

        while True:
            try:
                sample = (
                    self.data_queue.get_nowait()
                )
            except queue.Empty:
                break

            self._process_sample(sample)
            processed_count += 1

        if (
            processed_count > 0
            and self.sample_csv_file is not None
        ):
            self.sample_csv_file.flush()

    def _process_sample(
        self,
        sample: dict[str, Any],
    ) -> None:
        event_time = float(
            sample["unix_time"]
        )
        acc_mag = float(
            sample["acc_mag"]
        )

        # Keep only the common adaptive window.
        while (
            self.times
            and event_time - self.times[0]
            > WINDOW_SEC
        ):
            self.times.popleft()
            self.acc_mags.popleft()
            self.envelopes.popleft()
            self.boosted_scores.popleft()

        recent_acc = list(
            self.acc_mags
        )
        recent_envelope = list(
            self.envelopes
        )
        recent_boosted = list(
            self.boosted_scores
        )

        if len(recent_acc) < 10:
            baseline_acc = acc_mag
        else:
            baseline_acc = float(
                np.median(recent_acc)
            )

        vib = abs(
            acc_mag
            - baseline_acc
        )

        envelope = (
            ENVELOPE_ALPHA * vib
            + (
                1.0
                - ENVELOPE_ALPHA
            ) * self.prev_envelope
        )

        self.prev_envelope = envelope

        noise_level = robust_noise(
            recent_envelope
        )

        raw_score = (
            envelope
            / noise_level
        )

        if raw_score > 1.0:
            boosted_score = (
                1.0
                + (
                    raw_score
                    - 1.0
                ) * SCORE_BOOST_GAIN
            )
        else:
            boosted_score = raw_score

        fall_threshold = dynamic_score_threshold(
            values=recent_boosted,
            threshold_k=self.fall_threshold_k,
            min_threshold=(
                self.fall_min_score_threshold
            ),
            max_threshold=(
                self.fall_max_score_threshold
            ),
        )

        sos_threshold = dynamic_score_threshold(
            values=recent_boosted,
            threshold_k=self.sos_threshold_k,
            min_threshold=(
                self.sos_min_score_threshold
            ),
            max_threshold=(
                self.sos_max_score_threshold
            ),
        )

        self.times.append(event_time)
        self.acc_mags.append(acc_mag)
        self.envelopes.append(envelope)
        self.boosted_scores.append(
            boosted_score
        )

        fall_is_above = (
            boosted_score > fall_threshold
            and envelope
            > self.fall_min_signal_g
        )

        sos_is_above = (
            boosted_score > sos_threshold
            and envelope
            > self.sos_min_signal_g
        )

        fall_final_peak = (
            update_peak_detector(
                detector=(
                    self.fall_peak_detector
                ),
                is_above=fall_is_above,
                sample=sample,
                boosted_score=boosted_score,
                raw_score=raw_score,
                threshold=fall_threshold,
                envelope_g=envelope,
                noise_level_g=noise_level,
            )
        )

        sos_final_peak = (
            update_peak_detector(
                detector=(
                    self.sos_peak_detector
                ),
                is_above=sos_is_above,
                sample=sample,
                boosted_score=boosted_score,
                raw_score=raw_score,
                threshold=sos_threshold,
                envelope_g=envelope,
                noise_level_g=noise_level,
            )
        )

        fall_final_peak_flag = 0
        sos_final_peak_flag = 0

        if fall_final_peak is not None:
            fall_final_peak_flag = 1
            self._publish_fall_evidence(
                fall_final_peak
            )

        if sos_final_peak is not None:
            sos_final_peak_flag = 1
            self._register_sos_peak(
                sos_final_peak
            )

        if self.sample_csv_writer is not None:
            self.sample_csv_writer.writerow({
                "unix_time": sample["unix_time"],
                "ts_iso": sample["ts_iso"],

                "ax": sample["ax"],
                "ay": sample["ay"],
                "az": sample["az"],

                "wx": sample["wx"],
                "wy": sample["wy"],
                "wz": sample["wz"],

                "roll": sample["roll"],
                "pitch": sample["pitch"],
                "yaw": sample["yaw"],

                "acc_mag": sample["acc_mag"],
                "gyro_mag": sample["gyro_mag"],

                "baseline_acc": baseline_acc,
                "vib": vib,
                "envelope": envelope,
                "noise_level": noise_level,

                "raw_score": raw_score,
                "boosted_score": boosted_score,

                "fall_threshold": fall_threshold,
                "sos_threshold": sos_threshold,

                "fall_is_above": int(
                    fall_is_above
                ),
                "sos_is_above": int(
                    sos_is_above
                ),

                "fall_final_peak": (
                    fall_final_peak_flag
                ),
                "sos_final_peak": (
                    sos_final_peak_flag
                ),

                "sos_sequence_hits": len(
                    self.sos_peaks
                ),
            })

    # --------------------------------------------------------
    # Fall evidence output
    # --------------------------------------------------------

    def _fall_confidence(
        self,
        peak_ratio: float,
    ) -> float:
        denominator = max(
            self.fall_confidence_full_ratio
            - 1.0,
            1e-12,
        )

        return clipped(
            (
                peak_ratio
                - 1.0
            )
            / denominator,
            0.0,
            1.0,
        )

    def _fall_severity(
        self,
        peak_ratio: float,
    ) -> str:
        if (
            peak_ratio
            >= self.fall_strong_ratio
        ):
            return "strong"

        if (
            peak_ratio
            >= self.fall_medium_ratio
        ):
            return "medium"

        return "weak"

    def _publish_fall_evidence(
        self,
        peak: PeakCandidate,
    ) -> None:
        ros_stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        confidence = self._fall_confidence(
            peak.peak_ratio
        )

        severity = self._fall_severity(
            peak.peak_ratio
        )

        payload = {
            "schema_version": 1,
            "event_type": "fall_evidence",
            "modality": "imu",
            "sensor_id": self.sensor_id,

            "ros_stamp": {
                "sec": int(
                    ros_stamp.sec
                ),
                "nanosec": int(
                    ros_stamp.nanosec
                ),
            },

            "sensor_stamp": {
                "unix_time": (
                    peak.event_time
                ),
                "ts_iso": peak.ts_iso,
            },

            "confidence": confidence,
            "severity": severity,

            "modality_data": {
                "detection_type": (
                    "dynamic_threshold_peak"
                ),

                "boosted_score": (
                    peak.boosted_score
                ),
                "raw_score": (
                    peak.raw_score
                ),
                "dynamic_threshold": (
                    peak.threshold
                ),
                "peak_ratio": (
                    peak.peak_ratio
                ),

                "envelope_g": (
                    peak.envelope_g
                ),
                "noise_level_g": (
                    peak.noise_level_g
                ),
                "acc_mag_g": (
                    peak.acc_mag_g
                ),

                "acceleration_g": {
                    "x": peak.sample["ax"],
                    "y": peak.sample["ay"],
                    "z": peak.sample["az"],
                },

                "angular_velocity_dps": {
                    "x": peak.sample["wx"],
                    "y": peak.sample["wy"],
                    "z": peak.sample["wz"],
                },

                "orientation_deg": {
                    "roll": (
                        peak.sample["roll"]
                    ),
                    "pitch": (
                        peak.sample["pitch"]
                    ),
                    "yaw": (
                        peak.sample["yaw"]
                    ),
                },

                "detector_settings": {
                    "threshold_k": (
                        self.fall_threshold_k
                    ),
                    "min_score_threshold": (
                        self.fall_min_score_threshold
                    ),
                    "max_score_threshold": (
                        self.fall_max_score_threshold
                    ),
                    "min_signal_g": (
                        self.fall_min_signal_g
                    ),
                    "min_peak_distance_sec": (
                        self.fall_min_peak_distance_sec
                    ),
                },

                "confidence_method": (
                    "linear_peak_ratio"
                ),
            },
        }

        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )

        self.fall_publisher.publish(
            message
        )

        self.fall_evidence_count += 1

        self.get_logger().warning(
            "[FALL DETECTED] "
            f"confidence={confidence:.2f} | "
            f"severity={severity} | "
            f"score={peak.boosted_score:.1f} | "
            f"threshold={peak.threshold:.1f}"
        )

        self._write_event_csv(
            payload
        )

    # --------------------------------------------------------
    # SOS Peak sequence
    # --------------------------------------------------------

    def _register_sos_peak(
        self,
        peak: PeakCandidate,
    ) -> None:
        current_wall_time = time.time()

        if (
            current_wall_time
            - self.last_sos_publish_wall_time
            < self.sos_cooldown_sec
        ):
            return

        if self.sos_peaks:
            gap = (
                peak.event_time
                - self.sos_peaks[-1].event_time
            )

            if (
                gap <= 0
                or gap
                > self.sequence_quiet_reset_sec
            ):
                self._close_sos_sequence(
                    reason="new_peak_after_gap"
                )

        self.sos_peaks.append(
            SosPeak(
                event_time=peak.event_time,
                ts_iso=peak.ts_iso,

                boosted_score=(
                    peak.boosted_score
                ),
                threshold=peak.threshold,
                peak_ratio=peak.peak_ratio,

                envelope_g=peak.envelope_g,
                noise_level_g=(
                    peak.noise_level_g
                ),
            )
        )

        self.last_sos_peak_wall_time = (
            current_wall_time
        )

        # Keep only the most recent SOS window.
        while (
            self.sos_peaks
            and peak.event_time
            - self.sos_peaks[0].event_time
            > self.sos_window_sec
        ):
            self.sos_peaks.popleft()

        if (
            len(self.sos_peaks)
            >= self.min_sos_hits
        ):
            self._evaluate_sos_sequence()

    def _calculate_sos_metrics(
        self,
    ) -> dict[str, Any] | None:
        if len(self.sos_peaks) < 2:
            return None

        peaks = list(
            self.sos_peaks
        )

        event_times = np.asarray(
            [
                peak.event_time
                for peak in peaks
            ],
            dtype=float,
        )

        peak_ratios = np.asarray(
            [
                peak.peak_ratio
                for peak in peaks
            ],
            dtype=float,
        )

        iois = np.diff(
            event_times
        )

        duration = float(
            event_times[-1]
            - event_times[0]
        )

        if duration <= 0:
            return None

        autocorrelation_score = (
            impulse_autocorrelation_score(
                event_times=(
                    event_times.tolist()
                ),
                min_lag_sec=(
                    self.min_ioi_sec
                ),
                max_lag_sec=(
                    self.max_ioi_sec
                ),
            )
        )

        return {
            "hit_count": len(peaks),

            "start_time": float(
                event_times[0]
            ),
            "end_time": float(
                event_times[-1]
            ),
            "start_ts_iso": peaks[0].ts_iso,
            "end_ts_iso": peaks[-1].ts_iso,

            "duration_sec": duration,

            "peak_times": (
                event_times.tolist()
            ),
            "iois_sec": iois.tolist(),

            "mean_ioi_sec": float(
                np.mean(iois)
            ),
            "ioi_cv": (
                coefficient_of_variation(
                    iois
                )
            ),

            "autocorr_score": (
                autocorrelation_score
            ),

            "hit_rate_hz": float(
                (
                    len(peaks)
                    - 1
                )
                / duration
            ),

            "median_peak_ratio": float(
                np.median(
                    peak_ratios
                )
            ),

            "peak_ratio_cv": (
                coefficient_of_variation(
                    peak_ratios
                )
            ),

            "peak_ratios": (
                peak_ratios.tolist()
            ),

            "boosted_scores": [
                peak.boosted_score
                for peak in peaks
            ],

            "thresholds": [
                peak.threshold
                for peak in peaks
            ],

            "envelopes_g": [
                peak.envelope_g
                for peak in peaks
            ],

            "noise_levels_g": [
                peak.noise_level_g
                for peak in peaks
            ],
        }

    def _sos_condition_report(
        self,
        metrics: dict[str, Any],
    ) -> tuple[
        bool,
        bool,
        list[str],
    ]:
        iois = np.asarray(
            metrics["iois_sec"],
            dtype=float,
        )

        hit_count_ok = (
            metrics["hit_count"]
            >= self.min_sos_hits
        )

        duration_ok = (
            self.min_sequence_duration_sec
            <= metrics["duration_sec"]
            <= self.max_sequence_duration_sec
        )

        ioi_range_ok = bool(
            np.all(
                iois
                >= self.min_ioi_sec
            )
            and np.all(
                iois
                <= self.max_ioi_sec
            )
        )

        hit_rate_ok = (
            metrics["hit_rate_hz"]
            >= self.min_hit_rate_hz
        )

        strength_ok = (
            metrics["median_peak_ratio"]
            >= self.min_median_peak_ratio
        )

        irregular_timing_ok = (
            metrics["ioi_cv"]
            >= self.min_sos_ioi_cv
        )

        low_autocorrelation_ok = (
            metrics["autocorr_score"]
            <= self.max_sos_autocorr_score
        )

        footstep_like = (
            metrics["ioi_cv"]
            <= self.footstep_ioi_cv_max
            and metrics["autocorr_score"]
            >= self.footstep_autocorr_min
        )

        failed_conditions: list[str] = []

        if not hit_count_ok:
            failed_conditions.append(
                "hit_count"
            )

        if not duration_ok:
            failed_conditions.append(
                "duration"
            )

        if not ioi_range_ok:
            failed_conditions.append(
                "ioi_range"
            )

        if not hit_rate_ok:
            failed_conditions.append(
                "hit_rate"
            )

        if not strength_ok:
            failed_conditions.append(
                "strength"
            )

        if not irregular_timing_ok:
            failed_conditions.append(
                "timing_too_regular"
            )

        if not low_autocorrelation_ok:
            failed_conditions.append(
                "autocorrelation_too_high"
            )

        if footstep_like:
            failed_conditions.append(
                "footstep_like"
            )

        is_sos = (
            hit_count_ok
            and duration_ok
            and ioi_range_ok
            and hit_rate_ok
            and strength_ok
            and irregular_timing_ok
            and low_autocorrelation_ok
            and not footstep_like
        )

        return (
            is_sos,
            footstep_like,
            failed_conditions,
        )

    def _sos_confidence(
        self,
        metrics: dict[str, Any],
    ) -> float:
        strength_margin = clipped(
            (
                metrics["median_peak_ratio"]
                - self.min_median_peak_ratio
            )
            / max(
                2.50
                - self.min_median_peak_ratio,
                1e-12,
            ),
            0.0,
            1.0,
        )

        irregularity_margin = clipped(
            (
                metrics["ioi_cv"]
                - self.min_sos_ioi_cv
            )
            / max(
                0.60
                - self.min_sos_ioi_cv,
                1e-12,
            ),
            0.0,
            1.0,
        )

        aperiodic_margin = clipped(
            (
                self.max_sos_autocorr_score
                - metrics["autocorr_score"]
            )
            / max(
                self.max_sos_autocorr_score,
                1e-12,
            ),
            0.0,
            1.0,
        )

        extra_hit_margin = clipped(
            (
                metrics["hit_count"]
                - self.min_sos_hits
            )
            / 3.0,
            0.0,
            1.0,
        )

        confidence = (
            0.50
            + 0.20 * strength_margin
            + 0.15 * irregularity_margin
            + 0.10 * aperiodic_margin
            + 0.05 * extra_hit_margin
        )

        return clipped(
            confidence,
            0.0,
            1.0,
        )

    def _evaluate_sos_sequence(
        self,
    ) -> None:
        metrics = (
            self._calculate_sos_metrics()
        )

        if metrics is None:
            return

        (
            is_sos,
            footstep_like,
            failed_conditions,
        ) = self._sos_condition_report(
            metrics
        )

        if is_sos:
            self._publish_sos(
                metrics
            )
            self.sos_peaks.clear()
            self.last_sos_publish_wall_time = (
                time.time()
            )
            return

        # Rejected/footstep-like sequences are intentionally not logged
        # to keep the terminal focused on useful events.
        del footstep_like, failed_conditions

    def _publish_sos(
        self,
        metrics: dict[str, Any],
    ) -> None:
        ros_stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        confidence = self._sos_confidence(
            metrics
        )

        payload = {
            "schema_version": 1,
            "event_type": "sos_detected",
            "modality": "imu",
            "sensor_id": self.sensor_id,

            "ros_stamp": {
                "sec": int(
                    ros_stamp.sec
                ),
                "nanosec": int(
                    ros_stamp.nanosec
                ),
            },

            # The final Peak time is used as the sensor event stamp.
            "sensor_stamp": {
                "unix_time": (
                    metrics["end_time"]
                ),
                "ts_iso": (
                    metrics["end_ts_iso"]
                ),
            },

            "confidence": confidence,
            "severity": "critical",

            "modality_data": {
                "sos_type": (
                    "strong_irregular_repeated_impacts"
                ),

                "sequence": {
                    "start_time": (
                        metrics["start_time"]
                    ),
                    "end_time": (
                        metrics["end_time"]
                    ),

                    "start_ts_iso": (
                        metrics["start_ts_iso"]
                    ),
                    "end_ts_iso": (
                        metrics["end_ts_iso"]
                    ),

                    "duration_sec": (
                        metrics["duration_sec"]
                    ),
                    "hit_count": (
                        metrics["hit_count"]
                    ),

                    "peak_times": (
                        metrics["peak_times"]
                    ),
                    "iois_sec": (
                        metrics["iois_sec"]
                    ),
                },

                "metrics": {
                    "hit_rate_hz": (
                        metrics["hit_rate_hz"]
                    ),
                    "mean_ioi_sec": (
                        metrics["mean_ioi_sec"]
                    ),
                    "ioi_cv": (
                        metrics["ioi_cv"]
                    ),
                    "autocorr_score": (
                        metrics["autocorr_score"]
                    ),

                    "median_peak_ratio": (
                        metrics[
                            "median_peak_ratio"
                        ]
                    ),
                    "peak_ratio_cv": (
                        metrics[
                            "peak_ratio_cv"
                        ]
                    ),

                    "peak_ratios": (
                        metrics["peak_ratios"]
                    ),
                    "boosted_scores": (
                        metrics["boosted_scores"]
                    ),
                    "thresholds": (
                        metrics["thresholds"]
                    ),

                    "envelopes_g": (
                        metrics["envelopes_g"]
                    ),
                    "noise_levels_g": (
                        metrics[
                            "noise_levels_g"
                        ]
                    ),
                },

                "footstep_like": False,

                "detector_settings": {
                    "threshold_k": (
                        self.sos_threshold_k
                    ),
                    "min_score_threshold": (
                        self.sos_min_score_threshold
                    ),
                    "max_score_threshold": (
                        self.sos_max_score_threshold
                    ),
                    "min_signal_g": (
                        self.sos_min_signal_g
                    ),
                    "min_peak_distance_sec": (
                        self.sos_min_peak_distance_sec
                    ),

                    "min_sos_hits": (
                        self.min_sos_hits
                    ),
                    "sos_window_sec": (
                        self.sos_window_sec
                    ),
                    "min_ioi_sec": (
                        self.min_ioi_sec
                    ),
                    "max_ioi_sec": (
                        self.max_ioi_sec
                    ),
                    "min_sos_ioi_cv": (
                        self.min_sos_ioi_cv
                    ),
                    "max_sos_autocorr_score": (
                        self.max_sos_autocorr_score
                    ),
                },
            },
        }

        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )

        self.sos_publisher.publish(
            message
        )

        self.sos_count += 1

        self.get_logger().error(
            "[SOS DETECTED] "
            f"hits={metrics['hit_count']} | "
            f"confidence={confidence:.2f}"
        )

        self._write_event_csv(
            payload
        )

    def _check_sos_timeout(self) -> None:
        if not self.sos_peaks:
            return

        if (
            time.time()
            - self.last_sos_peak_wall_time
            >= self.sequence_quiet_reset_sec
        ):
            self._close_sos_sequence(
                reason="quiet_timeout"
            )

    def _close_sos_sequence(
        self,
        reason: str,
    ) -> None:
        if not self.sos_peaks:
            return

        metrics = (
            self._calculate_sos_metrics()
        )

        if metrics is None:
            result = "single_sos_peak"
            detail = ""
        else:
            (
                is_sos,
                footstep_like,
                failed_conditions,
            ) = self._sos_condition_report(
                metrics
            )

            if is_sos:
                self._publish_sos(
                    metrics
                )
                self.last_sos_publish_wall_time = (
                    time.time()
                )
                self.sos_peaks.clear()
                return

            result = (
                "footstep_like"
                if footstep_like
                else "rejected_sos_sequence"
            )

            detail = ",".join(
                failed_conditions
            )

        # Sequence rejection details stay internal. Only confirmed SOS
        # events are shown in the console.
        del result, reason, detail

        self.sos_peaks.clear()

    # --------------------------------------------------------
    # Shutdown
    # --------------------------------------------------------

    def destroy_node(self) -> bool:
        self.stop_event.set()

        if self.sample_csv_file is not None:
            self.sample_csv_file.flush()
            self.sample_csv_file.close()
            self.sample_csv_file = None

        if self.event_csv_file is not None:
            self.event_csv_file.flush()
            self.event_csv_file.close()
            self.event_csv_file = None

        return super().destroy_node()


# ============================================================
# 13. MAIN
# ============================================================

def main(
    args: list[str] | None = None,
) -> None:
    rclpy.init(args=args)

    node = ImuFallSosNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "[IMU] stopped"
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

