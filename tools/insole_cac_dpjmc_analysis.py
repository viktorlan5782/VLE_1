#!/usr/bin/env python3
"""Offline CAC/DPJMC analysis for left-foot 18-zone pressure-insole CSV logs.

This script intentionally uses only the Python standard library so it can run on
lab machines without pandas/scipy/matplotlib. It reconstructs the left-foot
pressure geometry, removes cross-file overlap, estimates CoP, gait state, and a
pressure-only ankle moment proxy, then writes CSV/JSON/Markdown/PNG artifacts.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import re
import statistics
import struct
import zlib
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


ZONE_COUNT = 18
EXPECTED_SAMPLE_PERIOD_S = 0.05
EXPECTED_PACKET_PERIOD_US = 50000
FRAME_TIMEOUT_S = 0.150
COP_MIN_TOTAL_G = 200.0
TOTAL_STANCE_THRESHOLD_G = 2000.0
FOREFOOT_THRESHOLD_G = 600.0
ARCH_THRESHOLD_G = 400.0
HEEL_THRESHOLD_G = 800.0
STANCE_ON_FACTOR = 1.05
SWING_OFF_FACTOR = 0.95
MIN_STATE_FRAMES = 3
SATURATION_THRESHOLD_G = 65000.0
FOOT_LENGTH_M = 0.21089
ANKLE_X_AP_NORM = 0.18
TAU_PROXY_SCALE = 1.0
TAU_MAX_NM = 1.5
G_TO_NEWTON = 0.00980665

FOOT_Y_MIN_MM = 36.68
FOOT_Y_MAX_MM = 252.20
FOOT_CENTER_X_MM = 39.805
FOOT_HALF_WIDTH_MM = 39.805

# Manual point order: L1 maps to point 1, ..., L18 maps to point 18.
# Values are the polygon centroids already used by ExoCode/src/PressureInsoleHighLevel.cpp.
ZONE_CENTER_X_MM = [
    12.08,
    6.90,
    40.68,
    40.68,
    32.80,
    22.49,
    30.72,
    23.28,
    64.27,
    64.27,
    50.93,
    48.67,
    49.97,
    38.71,
    68.00,
    71.34,
    68.70,
    59.46,
]
ZONE_CENTER_Y_MM = [
    188.14,
    227.50,
    49.82,
    84.70,
    119.14,
    153.56,
    188.14,
    229.97,
    49.82,
    84.70,
    119.14,
    153.56,
    188.14,
    229.97,
    119.14,
    153.56,
    188.14,
    225.20,
]

GAIT_INVALID = "Invalid"
GAIT_SWING = "Swing"
GAIT_INITIAL_CONTACT = "InitialContact"
GAIT_LOADING_RESPONSE = "LoadingResponse"
GAIT_MID_STANCE = "MidStance"
GAIT_TERMINAL_STANCE = "TerminalStance"
GAIT_PRE_SWING = "PreSwing"

METRICS = [
    "stance_duration_s",
    "swing_duration_s",
    "peak_total_g",
    "loading_rate_g_s",
    "cop_ap_excursion_norm",
    "cop_ml_excursion_norm",
    "peak_tau_proxy_pf_nm",
    "tau_proxy_impulse_nm_s",
]


@dataclass
class RawSample:
    source_file: str
    source_row: int
    timestamp: str
    parsed_timestamp_s: Optional[float]
    zones_g: Tuple[float, ...]

    @property
    def overlap_key(self) -> Tuple[str, Tuple[float, ...]]:
        return self.timestamp, self.zones_g


@dataclass
class FeatureSample:
    sample_index: int
    timestamp: str
    time_s: float
    source_file: str
    activity_hint: str
    zones_g: Tuple[float, ...]
    total_g: float
    p_fore_g: float
    p_arch_g: float
    p_heel_g: float
    p_medial_g: float
    p_lateral_g: float
    cop_x_mm: float
    cop_y_mm: float
    cop_ap_norm: float
    cop_ml_norm: float
    cop_ap_dot_norm_s: float
    cop_ml_dot_norm_s: float
    gait_state: str
    stance_phase: float
    tau_proxy_pf_nm: float
    tau_proxy_pf_nm_clipped: float
    activity_pred: str
    quality_flags: Tuple[str, ...]


@dataclass
class FileRecord:
    path: Path
    samples: List[RawSample]
    first_ts_s: float
    last_ts_s: float
    mtime_s: float
    activity_hint: str


@dataclass
class CleanSegment:
    file_record: FileRecord
    samples: List[RawSample]
    features: List[FeatureSample] = field(default_factory=list)
    cycles: List[Dict[str, object]] = field(default_factory=list)


class GaitStateMachine:
    def __init__(self) -> None:
        self.confirmed_contact = False
        self.candidate_contact = False
        self.candidate_frames = 0
        self.stance_start_s: Optional[float] = None
        self.expected_stance_s = 0.65
        self.last_cop_ap_norm = 0.0
        self.last_cop_ml_norm = 0.0
        self.last_cop_time_s: Optional[float] = None
        self.last_cop_valid = False

    def reset_contact(self) -> None:
        self.confirmed_contact = False
        self.candidate_contact = False
        self.candidate_frames = 0
        self.stance_start_s = None
        self.last_cop_valid = False

    def update_contact(self, total_g: float, forefoot_g: float, arch_g: float, heel_g: float, time_s: float) -> None:
        loaded_regions = 0
        if forefoot_g > FOREFOOT_THRESHOLD_G:
            loaded_regions += 1
        if arch_g > ARCH_THRESHOLD_G:
            loaded_regions += 1
        if heel_g > HEEL_THRESHOLD_G:
            loaded_regions += 1

        proposed_contact = self.confirmed_contact
        if total_g > TOTAL_STANCE_THRESHOLD_G * STANCE_ON_FACTOR and loaded_regions > 0:
            proposed_contact = True
        elif total_g < TOTAL_STANCE_THRESHOLD_G * SWING_OFF_FACTOR or loaded_regions == 0:
            proposed_contact = False

        if proposed_contact != self.candidate_contact:
            self.candidate_contact = proposed_contact
            self.candidate_frames = 1
        else:
            self.candidate_frames = min(255, self.candidate_frames + 1)

        if self.candidate_contact != self.confirmed_contact and self.candidate_frames >= MIN_STATE_FRAMES:
            if self.candidate_contact:
                self.confirmed_contact = True
                self.stance_start_s = time_s
            else:
                if self.stance_start_s is not None:
                    measured_stance_s = time_s - self.stance_start_s
                    if 0.2 < measured_stance_s < 2.0:
                        self.expected_stance_s = 0.8 * self.expected_stance_s + 0.2 * measured_stance_s
                self.confirmed_contact = False
                self.stance_start_s = None
                self.last_cop_valid = False

    def stance_phase(self, time_s: float) -> float:
        if self.stance_start_s is None or self.expected_stance_s <= 0.0:
            return 0.0
        return clamp((time_s - self.stance_start_s) / self.expected_stance_s, 0.0, 1.0)

    def classify_gait(self, cop_ap_norm: float, stance_phase: float) -> str:
        if not self.confirmed_contact:
            return GAIT_SWING
        if cop_ap_norm < 0.25:
            if stance_phase < 0.12:
                return GAIT_INITIAL_CONTACT
            return GAIT_LOADING_RESPONSE
        if cop_ap_norm < 0.40:
            return GAIT_LOADING_RESPONSE
        if cop_ap_norm < 0.65:
            return GAIT_MID_STANCE
        if stance_phase < 0.85:
            return GAIT_TERMINAL_STANCE
        return GAIT_PRE_SWING


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def safe_float(text: object) -> float:
    if text is None:
        return 0.0
    value = str(text).strip()
    if value == "":
        return 0.0
    try:
        return float(value)
    except ValueError:
        return 0.0


def parse_timestamp_s(text: str) -> Optional[float]:
    text = text.strip()
    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S", "%Y/%m/%d %H:%M:%S"):
        try:
            return datetime.strptime(text, fmt).timestamp()
        except ValueError:
            pass
    return None


def zone_ap_norm(zone_index: int) -> float:
    return clamp((ZONE_CENTER_Y_MM[zone_index] - FOOT_Y_MIN_MM) / (FOOT_Y_MAX_MM - FOOT_Y_MIN_MM), 0.0, 1.0)


def zone_ml_norm_left(zone_index: int) -> float:
    ml_norm = (ZONE_CENTER_X_MM[zone_index] - FOOT_CENTER_X_MM) / FOOT_HALF_WIDTH_MM
    return clamp(-ml_norm, -1.0, 1.0)


def infer_activity_hint(filename: str) -> str:
    name = filename.lower()
    if "noexo" in name:
        return "noexo_control"
    if "levelground" in name:
        return "level_walking"
    if "sit" in name and "stand" in name:
        return "sittostand"
    if "ramp" in name:
        return "ramp"
    if "stair1" in name:
        return "stair1"
    if "stair2" in name:
        return "stair2"
    if "stair" in name:
        return "stair"
    return "unknown"


def classify_activity(activity_hint: str, features_so_far: Optional[Sequence[FeatureSample]] = None) -> str:
    if activity_hint == "level_walking":
        return "level walking"
    if activity_hint == "sittostand":
        return "sittostand"
    if activity_hint == "ramp":
        return "ambiguous_ramp"
    if activity_hint in {"stair", "stair1", "stair2"}:
        return "ambiguous_stair"
    if activity_hint == "noexo_control":
        return "unknown_transition"
    return "unknown_transition"


def read_csv_samples(path: Path) -> List[RawSample]:
    encodings = ("utf-8-sig", "utf-8", "gb18030")
    last_error: Optional[Exception] = None
    for encoding in encodings:
        try:
            with path.open("r", newline="", encoding=encoding, errors="replace") as f:
                reader = csv.reader(f)
                header = next(reader)
                l_indices = []
                for zone_idx in range(1, ZONE_COUNT + 1):
                    expected = f"L{zone_idx}(g)"
                    try:
                        l_indices.append(header.index(expected))
                    except ValueError:
                        l_indices.append(zone_idx)
                samples: List[RawSample] = []
                for row_number, row in enumerate(reader, start=2):
                    if not row:
                        continue
                    timestamp = row[0].strip()
                    if not timestamp:
                        continue
                    zones = []
                    for col_idx in l_indices:
                        zones.append(safe_float(row[col_idx] if col_idx < len(row) else ""))
                    samples.append(
                        RawSample(
                            source_file=path.name,
                            source_row=row_number,
                            timestamp=timestamp,
                            parsed_timestamp_s=parse_timestamp_s(timestamp),
                            zones_g=tuple(zones),
                        )
                    )
                return samples
        except Exception as exc:  # pragma: no cover - defensive fallback for lab encodings.
            last_error = exc
    raise RuntimeError(f"Failed to read {path}: {last_error}")


def load_file_records(input_dir: Path) -> List[FileRecord]:
    records: List[FileRecord] = []
    for path in sorted(input_dir.glob("*.csv")):
        samples = read_csv_samples(path)
        parsed = [s.parsed_timestamp_s for s in samples if s.parsed_timestamp_s is not None]
        first_ts = min(parsed) if parsed else 0.0
        last_ts = max(parsed) if parsed else 0.0
        records.append(
            FileRecord(
                path=path,
                samples=samples,
                first_ts_s=first_ts,
                last_ts_s=last_ts,
                mtime_s=path.stat().st_mtime,
                activity_hint=infer_activity_hint(path.name),
            )
        )

    return sorted(records, key=lambda r: (r.first_ts_s, r.last_ts_s, r.mtime_s, r.path.name.lower()))


def split_nonoverlapping(records: Sequence[FileRecord]) -> List[CleanSegment]:
    prior_counts: Counter[Tuple[str, Tuple[float, ...]]] = Counter()
    segments: List[CleanSegment] = []
    for record in records:
        remaining_prior = prior_counts.copy()
        clean_samples: List[RawSample] = []
        for sample in record.samples:
            key = sample.overlap_key
            if remaining_prior[key] > 0:
                remaining_prior[key] -= 1
                continue
            clean_samples.append(sample)

        for sample in clean_samples:
            prior_counts[sample.overlap_key] += 1

        segments.append(CleanSegment(file_record=record, samples=clean_samples))
    return segments


def compute_features(segment: CleanSegment) -> List[FeatureSample]:
    fsm = GaitStateMachine()
    features: List[FeatureSample] = []
    activity_pred = classify_activity(segment.file_record.activity_hint)
    last_time_s: Optional[float] = None
    last_cop_ap_norm = 0.0
    last_cop_ml_norm = 0.0
    last_cop_valid = False

    for i, sample in enumerate(segment.samples):
        time_s = i * EXPECTED_SAMPLE_PERIOD_S
        flags: List[str] = []
        if last_time_s is not None and (time_s - last_time_s) > FRAME_TIMEOUT_S:
            flags.append("timeout")
            fsm.reset_contact()
            last_cop_valid = False

        zones = sample.zones_g
        if any(v >= SATURATION_THRESHOLD_G for v in zones):
            flags.append("saturation")

        total_g = sum(zones)
        p_fore_g = sum(zones[0:6])
        p_arch_g = sum(zones[6:12])
        p_heel_g = sum(zones[12:18])
        p_medial_g = 0.0
        p_lateral_g = 0.0
        for zone_idx, zone_g in enumerate(zones):
            if zone_ml_norm_left(zone_idx) >= 0.0:
                p_lateral_g += zone_g
            else:
                p_medial_g += zone_g

        fsm.update_contact(total_g, p_fore_g, p_arch_g, p_heel_g, time_s)

        has_cop = fsm.confirmed_contact and total_g >= COP_MIN_TOTAL_G
        if not has_cop:
            if total_g < COP_MIN_TOTAL_G:
                flags.append("low_pressure")
            flags.append("cop_invalid")
            cop_x_mm = 0.0
            cop_y_mm = 0.0
            cop_ap_norm = 0.0
            cop_ml_norm = 0.0
            cop_ap_dot = 0.0
            cop_ml_dot = 0.0
            stance_phase = 0.0
            gait_state = GAIT_SWING if not flags or "timeout" not in flags else GAIT_INVALID
            tau_proxy_pf_nm = 0.0
            tau_proxy_pf_nm_clipped = 0.0
            last_cop_valid = False
        else:
            cop_x_mm = sum(z * ZONE_CENTER_X_MM[idx] for idx, z in enumerate(zones)) / total_g
            cop_y_mm = sum(z * ZONE_CENTER_Y_MM[idx] for idx, z in enumerate(zones)) / total_g
            cop_ap_norm = clamp(sum(z * zone_ap_norm(idx) for idx, z in enumerate(zones)) / total_g, 0.0, 1.0)
            cop_ml_norm = clamp(sum(z * zone_ml_norm_left(idx) for idx, z in enumerate(zones)) / total_g, -1.0, 1.0)

            if last_cop_valid and last_time_s is not None:
                dt_s = max(EXPECTED_SAMPLE_PERIOD_S, time_s - last_time_s)
                cop_ap_dot = (cop_ap_norm - last_cop_ap_norm) / dt_s
                cop_ml_dot = (cop_ml_norm - last_cop_ml_norm) / dt_s
            else:
                cop_ap_dot = 0.0
                cop_ml_dot = 0.0

            stance_phase = fsm.stance_phase(time_s)
            gait_state = fsm.classify_gait(cop_ap_norm, stance_phase)
            fz_n = total_g * G_TO_NEWTON
            tau_raw = (cop_ap_norm - ANKLE_X_AP_NORM) * FOOT_LENGTH_M * fz_n * TAU_PROXY_SCALE
            tau_proxy_pf_nm = max(0.0, tau_raw)
            tau_proxy_pf_nm_clipped = clamp(tau_proxy_pf_nm, 0.0, TAU_MAX_NM)

            last_cop_ap_norm = cop_ap_norm
            last_cop_ml_norm = cop_ml_norm
            last_cop_valid = True

        features.append(
            FeatureSample(
                sample_index=i,
                timestamp=sample.timestamp,
                time_s=time_s,
                source_file=sample.source_file,
                activity_hint=segment.file_record.activity_hint,
                zones_g=zones,
                total_g=total_g,
                p_fore_g=p_fore_g,
                p_arch_g=p_arch_g,
                p_heel_g=p_heel_g,
                p_medial_g=p_medial_g,
                p_lateral_g=p_lateral_g,
                cop_x_mm=cop_x_mm,
                cop_y_mm=cop_y_mm,
                cop_ap_norm=cop_ap_norm,
                cop_ml_norm=cop_ml_norm,
                cop_ap_dot_norm_s=cop_ap_dot,
                cop_ml_dot_norm_s=cop_ml_dot,
                gait_state=gait_state,
                stance_phase=stance_phase,
                tau_proxy_pf_nm=tau_proxy_pf_nm,
                tau_proxy_pf_nm_clipped=tau_proxy_pf_nm_clipped,
                activity_pred=activity_pred,
                quality_flags=tuple(sorted(set(flags))),
            )
        )
        last_time_s = time_s

    segment.features = features
    return features


def extract_cycles(segment: CleanSegment) -> List[Dict[str, object]]:
    features = segment.features
    cycles: List[Dict[str, object]] = []
    in_stance = False
    start_idx = 0
    previous_stance_end_s: Optional[float] = None

    def is_stance(feature: FeatureSample) -> bool:
        return feature.gait_state not in {GAIT_INVALID, GAIT_SWING}

    for idx, feature in enumerate(features):
        stance = is_stance(feature)
        if stance and not in_stance:
            in_stance = True
            start_idx = idx
        elif not stance and in_stance:
            cycle = summarize_cycle(segment, len(cycles), start_idx, idx - 1, previous_stance_end_s)
            if cycle is not None:
                cycles.append(cycle)
                previous_stance_end_s = float(cycle["end_time_s"])
            in_stance = False

    if in_stance and start_idx < len(features):
        cycle = summarize_cycle(segment, len(cycles), start_idx, len(features) - 1, previous_stance_end_s)
        if cycle is not None:
            cycles.append(cycle)

    segment.cycles = cycles
    return cycles


def summarize_cycle(
    segment: CleanSegment,
    cycle_index: int,
    start_idx: int,
    end_idx: int,
    previous_stance_end_s: Optional[float],
) -> Optional[Dict[str, object]]:
    samples = segment.features[start_idx : end_idx + 1]
    if len(samples) < MIN_STATE_FRAMES:
        return None

    start_time_s = samples[0].time_s
    end_time_s = samples[-1].time_s
    stance_duration_s = max(EXPECTED_SAMPLE_PERIOD_S, end_time_s - start_time_s + EXPECTED_SAMPLE_PERIOD_S)
    swing_duration_s = 0.0 if previous_stance_end_s is None else max(0.0, start_time_s - previous_stance_end_s)
    peak_total_g = max(s.total_g for s in samples)
    peak_tau = max(s.tau_proxy_pf_nm for s in samples)
    tau_impulse = sum(s.tau_proxy_pf_nm * EXPECTED_SAMPLE_PERIOD_S for s in samples)
    cop_ap_values = [s.cop_ap_norm for s in samples if "cop_invalid" not in s.quality_flags]
    cop_ml_values = [s.cop_ml_norm for s in samples if "cop_invalid" not in s.quality_flags]
    cop_ap_excursion = (max(cop_ap_values) - min(cop_ap_values)) if cop_ap_values else 0.0
    cop_ml_excursion = (max(cop_ml_values) - min(cop_ml_values)) if cop_ml_values else 0.0
    loading_rate = 0.0
    for prev, cur in zip(samples, samples[1:]):
        loading_rate = max(loading_rate, (cur.total_g - prev.total_g) / EXPECTED_SAMPLE_PERIOD_S)

    quality_flags = sorted(set(flag for sample in samples for flag in sample.quality_flags))
    return {
        "source_file": segment.file_record.path.name,
        "cycle_index": cycle_index,
        "start_sample_index": start_idx,
        "end_sample_index": end_idx,
        "start_time_s": round(start_time_s, 6),
        "end_time_s": round(end_time_s, 6),
        "stance_duration_s": round(stance_duration_s, 6),
        "swing_duration_s": round(swing_duration_s, 6),
        "peak_total_g": round(peak_total_g, 6),
        "loading_rate_g_s": round(loading_rate, 6),
        "cop_ap_excursion_norm": round(cop_ap_excursion, 6),
        "cop_ml_excursion_norm": round(cop_ml_excursion, 6),
        "peak_tau_proxy_pf_nm": round(peak_tau, 6),
        "tau_proxy_impulse_nm_s": round(tau_impulse, 6),
        "activity_hint": segment.file_record.activity_hint,
        "activity_pred": samples[0].activity_pred,
        "quality_flags": "|".join(quality_flags),
    }


def clean_filename(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    return cleaned.strip("_") or "segment"


def write_clean_csv(segment: CleanSegment, out_path: Path) -> None:
    header = ["sample_index", "timestamp", "source_file", "activity_hint"] + [f"L{i}_g" for i in range(1, 19)]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i, sample in enumerate(segment.samples):
            writer.writerow([i, sample.timestamp, sample.source_file, segment.file_record.activity_hint, *sample.zones_g])


def write_feature_csv(segment: CleanSegment, out_path: Path) -> None:
    header = [
        "sample_index",
        "timestamp",
        "time_s",
        "source_file",
        "activity_hint",
        *[f"L{i}_g" for i in range(1, 19)],
        "total_g",
        "p_fore_g",
        "p_arch_g",
        "p_heel_g",
        "p_medial_g",
        "p_lateral_g",
        "cop_x_mm",
        "cop_y_mm",
        "cop_ap_norm",
        "cop_ml_norm",
        "cop_ap_dot_norm_s",
        "cop_ml_dot_norm_s",
        "gait_state",
        "stance_phase",
        "tau_proxy_pf_nm",
        "tau_proxy_pf_nm_clipped",
        "activity_pred",
        "quality_flags",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for s in segment.features:
            writer.writerow(
                [
                    s.sample_index,
                    s.timestamp,
                    f"{s.time_s:.6f}",
                    s.source_file,
                    s.activity_hint,
                    *[f"{v:.6f}" for v in s.zones_g],
                    f"{s.total_g:.6f}",
                    f"{s.p_fore_g:.6f}",
                    f"{s.p_arch_g:.6f}",
                    f"{s.p_heel_g:.6f}",
                    f"{s.p_medial_g:.6f}",
                    f"{s.p_lateral_g:.6f}",
                    f"{s.cop_x_mm:.6f}",
                    f"{s.cop_y_mm:.6f}",
                    f"{s.cop_ap_norm:.6f}",
                    f"{s.cop_ml_norm:.6f}",
                    f"{s.cop_ap_dot_norm_s:.6f}",
                    f"{s.cop_ml_dot_norm_s:.6f}",
                    s.gait_state,
                    f"{s.stance_phase:.6f}",
                    f"{s.tau_proxy_pf_nm:.6f}",
                    f"{s.tau_proxy_pf_nm_clipped:.6f}",
                    s.activity_pred,
                    "|".join(s.quality_flags),
                ]
            )


def write_cycles_csv(cycles: Sequence[Dict[str, object]], out_path: Path) -> None:
    header = [
        "source_file",
        "cycle_index",
        "start_sample_index",
        "end_sample_index",
        "start_time_s",
        "end_time_s",
        *METRICS,
        "activity_hint",
        "activity_pred",
        "quality_flags",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        for cycle in cycles:
            writer.writerow(cycle)


def summarize_segments(segments: Sequence[CleanSegment]) -> Dict[str, object]:
    files = []
    for segment in segments:
        flags = Counter(flag for sample in segment.features for flag in sample.quality_flags)
        timestamps = [s.timestamp for s in segment.samples]
        files.append(
            {
                "source_file": segment.file_record.path.name,
                "activity_hint": segment.file_record.activity_hint,
                "raw_rows": len(segment.file_record.samples),
                "clean_rows": len(segment.samples),
                "removed_overlap_rows": len(segment.file_record.samples) - len(segment.samples),
                "first_timestamp": timestamps[0] if timestamps else None,
                "last_timestamp": timestamps[-1] if timestamps else None,
                "last_write_time": datetime.fromtimestamp(segment.file_record.mtime_s).isoformat(timespec="seconds"),
                "cycle_count": len(segment.cycles),
                "quality_flags": dict(flags),
            }
        )

    return {
        "parameters": {
            "sampling_rate_hz": 20.0,
            "expected_sample_period_s": EXPECTED_SAMPLE_PERIOD_S,
            "expected_packet_period_us": EXPECTED_PACKET_PERIOD_US,
            "timeout_s": FRAME_TIMEOUT_S,
            "total_stance_threshold_g": TOTAL_STANCE_THRESHOLD_G,
            "cop_min_total_g": COP_MIN_TOTAL_G,
            "min_state_frames": MIN_STATE_FRAMES,
            "saturation_threshold_g": SATURATION_THRESHOLD_G,
            "foot_length_m": FOOT_LENGTH_M,
            "ankle_x_ap_norm": ANKLE_X_AP_NORM,
            "tau_proxy_scale": TAU_PROXY_SCALE,
            "tau_max_nm": TAU_MAX_NM,
            "pressure_unit": "g, positive compressive normal load",
            "force_conversion": "Fz_N = total_g * 0.00980665",
            "coordinate_convention": "AP 0=heel, 1=toe; ML 0=centerline, +ML=lateral for left foot",
            "torque_convention": "tau_proxy_pf_nm is plantarflexion-positive; OpenExo ankle command is dorsiflexion-positive",
            "timeline_note": "Input CSV timestamps are second-resolution; time_s is reconstructed from row order at 20 Hz.",
        },
        "files": files,
    }


def median(values: Sequence[float]) -> float:
    if not values:
        return float("nan")
    return statistics.median(values)


def percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * pct
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def bootstrap_median_diff_ci(a: Sequence[float], b: Sequence[float], reps: int = 2000) -> Tuple[float, float]:
    if not a or not b:
        return float("nan"), float("nan")
    rng = random.Random(7)
    diffs = []
    for _ in range(reps):
        sample_a = [a[rng.randrange(len(a))] for _ in range(len(a))]
        sample_b = [b[rng.randrange(len(b))] for _ in range(len(b))]
        diffs.append(median(sample_a) - median(sample_b))
    return percentile(diffs, 0.025), percentile(diffs, 0.975)


def cliffs_delta(a: Sequence[float], b: Sequence[float]) -> float:
    if not a or not b:
        return float("nan")
    greater = 0
    less = 0
    for av in a:
        for bv in b:
            if av > bv:
                greater += 1
            elif av < bv:
                less += 1
    return (greater - less) / (len(a) * len(b))


def mann_whitney_u_pvalue(a: Sequence[float], b: Sequence[float]) -> float:
    if len(a) < 2 or len(b) < 2:
        return float("nan")
    combined = [(v, 0) for v in a] + [(v, 1) for v in b]
    combined.sort(key=lambda item: item[0])

    ranks = [0.0] * len(combined)
    idx = 0
    while idx < len(combined):
        j = idx + 1
        while j < len(combined) and combined[j][0] == combined[idx][0]:
            j += 1
        avg_rank = (idx + 1 + j) / 2.0
        for k in range(idx, j):
            ranks[k] = avg_rank
        idx = j

    rank_sum_a = sum(rank for rank, (_, group) in zip(ranks, combined) if group == 0)
    n1 = len(a)
    n2 = len(b)
    u1 = rank_sum_a - n1 * (n1 + 1) / 2.0
    mean_u = n1 * n2 / 2.0
    sd_u = math.sqrt(n1 * n2 * (n1 + n2 + 1) / 12.0)
    if sd_u == 0.0:
        return float("nan")
    z = abs((u1 - mean_u) / sd_u)
    return math.erfc(z / math.sqrt(2.0))


def compare_cycles(cycles: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    noexo = [c for c in cycles if c.get("activity_hint") == "noexo_control"]
    exo = [c for c in cycles if c.get("activity_hint") != "noexo_control"]
    rows: List[Dict[str, object]] = []
    for metric in METRICS:
        noexo_values = [float(c[metric]) for c in noexo if usable_cycle(c)]
        exo_values = [float(c[metric]) for c in exo if usable_cycle(c)]
        med_exo = median(exo_values)
        med_noexo = median(noexo_values)
        diff = med_exo - med_noexo if exo_values and noexo_values else float("nan")
        ci_low, ci_high = bootstrap_median_diff_ci(exo_values, noexo_values)
        p_value = mann_whitney_u_pvalue(exo_values, noexo_values)
        rows.append(
            {
                "metric": metric,
                "n_exo": len(exo_values),
                "n_noexo": len(noexo_values),
                "median_exo": med_exo,
                "median_noexo": med_noexo,
                "median_diff_exo_minus_noexo": diff,
                "ci95_low": ci_low,
                "ci95_high": ci_high,
                "cliffs_delta": cliffs_delta(exo_values, noexo_values),
                "p_value_mann_whitney": p_value,
                "interpretation": interpret_effect(p_value, ci_low, ci_high, len(exo_values), len(noexo_values)),
            }
        )
    return rows


def usable_cycle(cycle: Dict[str, object]) -> bool:
    flags = str(cycle.get("quality_flags", ""))
    return "saturation" not in flags and "timeout" not in flags


def interpret_effect(p_value: float, ci_low: float, ci_high: float, n_exo: int, n_noexo: int) -> str:
    if n_exo < 2 or n_noexo < 2 or math.isnan(p_value):
        return "not_testable"
    if p_value < 0.05 and (ci_low > 0.0 or ci_high < 0.0):
        return "exploratory_significant"
    return "not_significant"


def write_comparison_csv(rows: Sequence[Dict[str, object]], out_path: Path) -> None:
    header = [
        "metric",
        "n_exo",
        "n_noexo",
        "median_exo",
        "median_noexo",
        "median_diff_exo_minus_noexo",
        "ci95_low",
        "ci95_high",
        "cliffs_delta",
        "p_value_mann_whitney",
        "interpretation",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: format_value(row[k]) for k in header})


def format_value(value: object) -> object:
    if isinstance(value, float):
        if math.isnan(value):
            return ""
        return f"{value:.6g}"
    return value


def write_metadata(metadata: Dict[str, object], out_path: Path) -> None:
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)


def write_report(
    segments: Sequence[CleanSegment],
    all_cycles: Sequence[Dict[str, object]],
    comparison_rows: Sequence[Dict[str, object]],
    out_path: Path,
) -> None:
    total_raw = sum(len(s.file_record.samples) for s in segments)
    total_clean = sum(len(s.samples) for s in segments)
    total_overlap = total_raw - total_clean
    lines: List[str] = []
    lines.append("# CAC/DPJMC 左脚鞋垫离线分析报告")
    lines.append("")
    lines.append("## 结论边界")
    lines.append("")
    lines.append(
        "本报告使用单只左脚 18-zone pressure-only data，输出 CoP（center of pressure）、步态状态机（gait finite-state machine）和跖屈为正（plantarflexion-positive）的踝矩代理量（ankle moment proxy）。"
    )
    lines.append(
        "该 `tau_proxy_pf_nm` 不是完整逆动力学（inverse dynamics）踝关节力矩，因为没有 GRF vector、COM acceleration、ankle kinematics 和 body mass。"
    )
    lines.append("")
    lines.append("## 数据分离")
    lines.append("")
    lines.append(f"- 原始行数（raw rows）：{total_raw}")
    lines.append(f"- clean 行数（clean rows）：{total_clean}")
    lines.append(f"- 跨文件重叠删除（removed overlap rows）：{total_overlap}")
    lines.append("- 去重策略：跨文件按 `(timestamp, L1-L18 pressure vector)` 做 multiset subtraction；文件内不按秒级 timestamp 压缩。")
    lines.append("- 时间轴：CSV timestamp 为秒级，`time_s` 按 20 Hz sampling rate 从 row order 重建。")
    lines.append("")
    lines.append("| source_file | activity_hint | raw_rows | clean_rows | cycles | removed_overlap_rows |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for s in segments:
        lines.append(
            f"| {s.file_record.path.name} | {s.file_record.activity_hint} | {len(s.file_record.samples)} | {len(s.samples)} | {len(s.cycles)} | {len(s.file_record.samples) - len(s.samples)} |"
        )
    lines.append("")
    lines.append("## 方法")
    lines.append("")
    lines.append("- 坐标系（coordinate system）：AP 0=heel、1=toe；ML 0=centerline、左脚 +ML=lateral。")
    lines.append("- 单位（unit）：pressure 为 g，正值表示 compressive normal load；`Fz_N = total_g * 0.00980665`。")
    lines.append(
        f"- FSM 阈值：`TOTAL_STANCE_THRESHOLD_G={TOTAL_STANCE_THRESHOLD_G}`、`COP_MIN_TOTAL_G={COP_MIN_TOTAL_G}`、`MIN_STATE_FRAMES={MIN_STATE_FRAMES}`、timeout={FRAME_TIMEOUT_S * 1000:.0f} ms。"
    )
    lines.append(
        f"- 踝矩代理量：`tau_proxy_pf_nm=max(0,(cop_ap_norm-{ANKLE_X_AP_NORM})*{FOOT_LENGTH_M}*Fz_N*{TAU_PROXY_SCALE})`，另输出 clipped column，`tau_max_nm={TAU_MAX_NM}`。"
    )
    lines.append("- Failure behavior：low pressure、timeout、invalid CoP 时 `tau_proxy_pf_nm=0`，并写入 `quality_flags`。")
    lines.append("")
    lines.append("## CAC 分类限制")
    lines.append("")
    lines.append(
        "当前 `activity_pred` 是 rule-based baseline，不训练机器学习模型（machine learning model）。单只左脚 pressure-only 特征通常不足以可靠区分上坡/下坡与上台阶/下台阶，因此 ramp/stair trial 输出 `ambiguous_ramp` 或 `ambiguous_stair`。"
    )
    lines.append("")
    lines.append("## Exo vs No-Exo 探索性统计")
    lines.append("")
    lines.append(
        "统计以 stance cycle 为单位，过滤 saturation/timeout cycle。由于当前数据未证明严格 paired trial，只能报告 exploratory effect，不能直接宣称 causal significance。"
    )
    lines.append("")
    lines.append("| metric | n_exo | n_noexo | median_exo | median_noexo | median_diff | 95% CI | Cliff's delta | p_value | interpretation |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    for row in comparison_rows:
        ci = ""
        if not math.isnan(float(row["ci95_low"])) and not math.isnan(float(row["ci95_high"])):
            ci = f"[{float(row['ci95_low']):.4g}, {float(row['ci95_high']):.4g}]"
        lines.append(
            "| {metric} | {n_exo} | {n_noexo} | {median_exo} | {median_noexo} | {diff} | {ci} | {delta} | {p} | {interp} |".format(
                metric=row["metric"],
                n_exo=row["n_exo"],
                n_noexo=row["n_noexo"],
                median_exo=format_value(row["median_exo"]),
                median_noexo=format_value(row["median_noexo"]),
                diff=format_value(row["median_diff_exo_minus_noexo"]),
                ci=ci,
                delta=format_value(row["cliffs_delta"]),
                p=format_value(row["p_value_mann_whitney"]),
                interp=row["interpretation"],
            )
        )
    lines.append("")
    lines.append("## Verification")
    lines.append("")
    lines.append("- CoP AP 已 clamp 到 `[0,1]`，ML 已 clamp 到 `[-1,1]`。")
    lines.append("- `quality_flags` 记录 low_pressure、cop_invalid、saturation、timeout。")
    lines.append("- 与 firmware 常量保持一致：20 Hz sampling、150 ms pressure timeout、g pressure unit、plantarflexion-positive tau proxy、OpenExo ankle command dorsiflexion-positive。")
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)


def write_rgb_png(path: Path, width: int, height: int, pixels: List[Tuple[int, int, int]]) -> None:
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        row_start = y * width
        for r, g, b in pixels[row_start : row_start + width]:
            raw.extend((r, g, b))
    data = b"\x89PNG\r\n\x1a\n"
    data += png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    data += png_chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    data += png_chunk(b"IEND", b"")
    path.write_bytes(data)


class Canvas:
    def __init__(self, width: int, height: int, bg: Tuple[int, int, int] = (255, 255, 255)) -> None:
        self.width = width
        self.height = height
        self.pixels = [bg for _ in range(width * height)]

    def set_px(self, x: int, y: int, color: Tuple[int, int, int]) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            self.pixels[y * self.width + x] = color

    def line(self, x0: int, y0: int, x1: int, y1: int, color: Tuple[int, int, int]) -> None:
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            self.set_px(x0, y0, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def rect(self, x0: int, y0: int, x1: int, y1: int, color: Tuple[int, int, int], fill: bool = True) -> None:
        if fill:
            for y in range(max(0, y0), min(self.height, y1 + 1)):
                for x in range(max(0, x0), min(self.width, x1 + 1)):
                    self.set_px(x, y, color)
        else:
            self.line(x0, y0, x1, y0, color)
            self.line(x1, y0, x1, y1, color)
            self.line(x1, y1, x0, y1, color)
            self.line(x0, y1, x0, y0, color)


def color_map(value: float, max_value: float) -> Tuple[int, int, int]:
    ratio = 0.0 if max_value <= 0.0 else clamp(value / max_value, 0.0, 1.0)
    r = int(255 * ratio)
    g = int(160 * (1.0 - abs(ratio - 0.5) * 2.0))
    b = int(255 * (1.0 - ratio))
    return r, g, b


def make_pressure_heatmap(segment: CleanSegment, out_path: Path) -> None:
    width, height = 500, 800
    c = Canvas(width, height)
    means = []
    for zone_idx in range(ZONE_COUNT):
        values = [s.zones_g[zone_idx] for s in segment.features]
        means.append(sum(values) / len(values) if values else 0.0)
    max_mean = max(means) if means else 1.0
    for idx, value in enumerate(means):
        x = int(60 + (ZONE_CENTER_X_MM[idx] / 80.0) * 360)
        y = int(height - 80 - ((ZONE_CENTER_Y_MM[idx] - FOOT_Y_MIN_MM) / (FOOT_Y_MAX_MM - FOOT_Y_MIN_MM)) * 640)
        c.rect(x - 22, y - 22, x + 22, y + 22, color_map(value, max_mean), True)
        c.rect(x - 22, y - 22, x + 22, y + 22, (0, 0, 0), False)
    write_rgb_png(out_path, width, height, c.pixels)


def make_cop_trajectory(segment: CleanSegment, out_path: Path) -> None:
    width, height = 700, 700
    c = Canvas(width, height)
    margin = 70
    c.rect(margin, margin, width - margin, height - margin, (0, 0, 0), False)
    points = [
        (
            int(margin + ((s.cop_ml_norm + 1.0) / 2.0) * (width - 2 * margin)),
            int(height - margin - s.cop_ap_norm * (height - 2 * margin)),
        )
        for s in segment.features
        if "cop_invalid" not in s.quality_flags
    ]
    for p0, p1 in zip(points, points[1:]):
        c.line(p0[0], p0[1], p1[0], p1[1], (20, 90, 180))
    for x, y in points[:: max(1, len(points) // 200)]:
        c.rect(x - 2, y - 2, x + 2, y + 2, (190, 40, 40), True)
    write_rgb_png(out_path, width, height, c.pixels)


def make_time_series(segment: CleanSegment, out_path: Path) -> None:
    width, height = 1000, 500
    c = Canvas(width, height)
    margin = 60
    c.rect(margin, margin, width - margin, height - margin, (0, 0, 0), False)
    if not segment.features:
        write_rgb_png(out_path, width, height, c.pixels)
        return
    max_tau = max(max(s.tau_proxy_pf_nm for s in segment.features), 1.0)
    max_total = max(max(s.total_g for s in segment.features), 1.0)
    t_max = max(segment.features[-1].time_s, EXPECTED_SAMPLE_PERIOD_S)

    def to_x(t: float) -> int:
        return int(margin + (t / t_max) * (width - 2 * margin))

    def to_y(value: float, max_value: float) -> int:
        return int(height - margin - clamp(value / max_value, 0.0, 1.0) * (height - 2 * margin))

    for prev, cur in zip(segment.features, segment.features[1:]):
        c.line(to_x(prev.time_s), to_y(prev.total_g, max_total), to_x(cur.time_s), to_y(cur.total_g, max_total), (100, 100, 100))
        c.line(to_x(prev.time_s), to_y(prev.tau_proxy_pf_nm, max_tau), to_x(cur.time_s), to_y(cur.tau_proxy_pf_nm, max_tau), (200, 50, 50))
    for s in segment.features:
        if s.gait_state not in {GAIT_SWING, GAIT_INVALID}:
            x = to_x(s.time_s)
            c.line(x, height - margin, x, height - margin - 8, (40, 150, 70))
    write_rgb_png(out_path, width, height, c.pixels)


def make_comparison_plot(rows: Sequence[Dict[str, object]], out_path: Path) -> None:
    width, height = 1000, 500
    c = Canvas(width, height)
    margin = 60
    usable = [r for r in rows if not math.isnan(float(r["median_exo"])) and not math.isnan(float(r["median_noexo"]))]
    if not usable:
        write_rgb_png(out_path, width, height, c.pixels)
        return
    max_value = max(max(float(r["median_exo"]), float(r["median_noexo"])) for r in usable)
    max_value = max(max_value, 1.0)
    group_w = (width - 2 * margin) / max(1, len(usable))
    for idx, row in enumerate(usable):
        x0 = int(margin + idx * group_w + group_w * 0.2)
        x1 = int(margin + idx * group_w + group_w * 0.45)
        x2 = int(margin + idx * group_w + group_w * 0.55)
        x3 = int(margin + idx * group_w + group_w * 0.8)
        exo_h = int((float(row["median_exo"]) / max_value) * (height - 2 * margin))
        noexo_h = int((float(row["median_noexo"]) / max_value) * (height - 2 * margin))
        c.rect(x0, height - margin - exo_h, x1, height - margin, (200, 70, 70), True)
        c.rect(x2, height - margin - noexo_h, x3, height - margin, (70, 120, 200), True)
    c.rect(margin, margin, width - margin, height - margin, (0, 0, 0), False)
    write_rgb_png(out_path, width, height, c.pixels)


def write_plots(segments: Sequence[CleanSegment], comparison_rows: Sequence[Dict[str, object]], plots_dir: Path) -> None:
    for segment in segments:
        stem = clean_filename(segment.file_record.path.stem)
        make_pressure_heatmap(segment, plots_dir / f"{stem}_pressure_heatmap.png")
        make_cop_trajectory(segment, plots_dir / f"{stem}_cop_trajectory.png")
        make_time_series(segment, plots_dir / f"{stem}_timeseries.png")
    make_comparison_plot(comparison_rows, plots_dir / "exo_vs_noexo_comparison.png")


def ensure_dirs(output_dir: Path) -> Dict[str, Path]:
    dirs = {
        "clean": output_dir / "clean_segments",
        "features": output_dir / "features",
        "cycles": output_dir / "cycles",
        "plots": output_dir / "plots",
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    for path in dirs.values():
        path.mkdir(parents=True, exist_ok=True)
    return dirs


def run(args: argparse.Namespace) -> int:
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    dirs = ensure_dirs(output_dir)

    records = load_file_records(input_dir)
    segments = split_nonoverlapping(records)

    all_cycles: List[Dict[str, object]] = []
    for segment in segments:
        compute_features(segment)
        cycles = extract_cycles(segment)
        all_cycles.extend(cycles)

        stem = clean_filename(segment.file_record.path.stem)
        write_clean_csv(segment, dirs["clean"] / f"{stem}_clean.csv")
        write_feature_csv(segment, dirs["features"] / f"{stem}_features.csv")
        write_cycles_csv(cycles, dirs["cycles"] / f"{stem}_cycles.csv")

    write_cycles_csv(all_cycles, output_dir / "all_cycles.csv")
    comparison_rows = compare_cycles(all_cycles)
    write_comparison_csv(comparison_rows, output_dir / "exo_vs_noexo_stats.csv")
    metadata = summarize_segments(segments)
    metadata["comparison"] = comparison_rows
    metadata["geometry_source"] = str(Path(args.geometry))
    write_metadata(metadata, output_dir / "metadata.json")
    write_report(segments, all_cycles, comparison_rows, output_dir / "report.md")
    write_plots(segments, comparison_rows, dirs["plots"])

    print(f"Processed {len(records)} CSV files")
    print(f"Raw rows: {sum(len(r.samples) for r in records)}")
    print(f"Clean rows: {sum(len(s.samples) for s in segments)}")
    print(f"Cycles: {len(all_cycles)}")
    print(f"Output: {output_dir}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline CAC/DPJMC analysis for 18-zone left-foot insole CSV logs.")
    parser.add_argument("--input-dir", default="0_insole_experiment_c", help="Directory containing raw insole CSV files.")
    parser.add_argument(
        "--geometry",
        default="Documentation/Insole_files/insole_geometry_parameter.m",
        help="Geometry reference file. The script uses fixed centroids derived from this file.",
    )
    parser.add_argument("--output-dir", default="0_insole_experiment_c/processed", help="Output artifact directory.")
    return parser


if __name__ == "__main__":
    raise SystemExit(run(build_parser().parse_args()))
