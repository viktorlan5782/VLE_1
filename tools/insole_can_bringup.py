#!/usr/bin/env python3
"""Passive pressure-insole CAN bring-up monitor.

The 18-zone pressure insole emits a 39-byte TTL packet:
  AA foot_id zone_01_hi zone_01_lo ... zone_18_hi zone_18_lo checksum

In WitMotion TTL-CAN transparent mode the TTL byte stream is fragmented across
classic CAN data frames. This tool reconstructs that byte stream per CAN ID,
validates checksum and packet timing, and writes bring-up evidence artifacts.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


PACKET_LEN = 39
PACKET_HEADER = 0xAA
ZONE_COUNT = 18
DEFAULT_CAN_IDS = (0x601, 0x602)
SELF_TEST_CAN_ID = DEFAULT_CAN_IDS[0]
DEFAULT_EXPECTED_PERIOD_MS = 50.0
FOOT_LABELS = {1: "left", 2: "right"}

MANUAL_SAMPLE_HEX = (
    "AA 01 00 4E 00 11 01 06 00 00 01 6A 00 30 01 3F 00 0E 00 00 "
    "02 BD 00 00 00 B2 00 37 00 AF 01 E2 00 08 00 15 10 23 84"
)


@dataclass
class InsolePacket:
    packet_index: int
    timestamp_s: float
    can_id: int
    source_frame_index: int
    foot_id: int
    zones_g: Tuple[int, ...]
    raw_hex: str

    @property
    def foot_label(self) -> str:
        return FOOT_LABELS.get(self.foot_id, f"unknown_{self.foot_id}")


@dataclass
class FrameRecord:
    frame_index: int
    timestamp_s: float
    can_id: int
    dlc: int
    data_hex: str
    delta_ms: Optional[float]


@dataclass
class PacketIntervalRecord:
    packet_index: int
    timestamp_s: float
    can_id: int
    foot_id: int
    delta_ms: float
    sample_rate_hz: float


@dataclass
class DropEvent:
    timestamp_s: float
    can_id: int
    foot_id: int
    delta_ms: float
    expected_period_ms: float
    estimated_missing_packets: int
    reason: str


@dataclass
class TimeoutEvent:
    timestamp_s: float
    elapsed_ms: float
    timeout_ms: float
    reason: str


@dataclass
class PressureAnomaly:
    packet_index: int
    timestamp_s: float
    can_id: int
    foot_id: int
    zone_index: int
    value_g: int
    limit_g: int
    reason: str


class StreamStats:
    def __init__(self, can_id: int) -> None:
        self.can_id = can_id
        self.input_frames = 0
        self.input_bytes = 0
        self.decoded_packets = 0
        self.bad_checksums = 0
        self.bad_foot_ids = 0
        self.orphan_bytes = 0
        self.fragment_timeouts = 0
        self.unfinished_bytes = 0
        self.bad_checksum_previews: List[str] = []

    def as_dict(self) -> Dict[str, object]:
        return {
            "can_id": f"0x{self.can_id:X}",
            "input_frames": self.input_frames,
            "input_bytes": self.input_bytes,
            "decoded_packets": self.decoded_packets,
            "bad_checksums": self.bad_checksums,
            "bad_foot_ids": self.bad_foot_ids,
            "orphan_bytes": self.orphan_bytes,
            "fragment_timeouts": self.fragment_timeouts,
            "unfinished_bytes": self.unfinished_bytes,
            "bad_checksum_previews": self.bad_checksum_previews,
        }


class InsoleStreamParser:
    def __init__(self, can_id: int, fragment_timeout_ms: float) -> None:
        self.can_id = can_id
        self.fragment_timeout_ms = fragment_timeout_ms
        self.buffer = bytearray()
        self.last_frame_timestamp_s: Optional[float] = None
        self.stats = StreamStats(can_id)

    def feed(self, data: bytes, timestamp_s: float, frame_index: int) -> List[InsolePacket]:
        self.stats.input_frames += 1
        self.stats.input_bytes += len(data)

        if self.buffer and self.last_frame_timestamp_s is not None:
            gap_ms = (timestamp_s - self.last_frame_timestamp_s) * 1000.0
            if gap_ms > self.fragment_timeout_ms:
                self.stats.fragment_timeouts += 1
                self.stats.orphan_bytes += len(self.buffer)
                self.buffer.clear()

        self.last_frame_timestamp_s = timestamp_s
        self.buffer.extend(data)
        return self._drain_packets(timestamp_s, frame_index)

    def finalize(self) -> None:
        self.stats.unfinished_bytes = len(self.buffer)

    def _drain_packets(self, timestamp_s: float, frame_index: int) -> List[InsolePacket]:
        packets: List[InsolePacket] = []

        while self.buffer:
            header_pos = self.buffer.find(bytes([PACKET_HEADER]))
            if header_pos < 0:
                self.stats.orphan_bytes += len(self.buffer)
                self.buffer.clear()
                break

            if header_pos > 0:
                self.stats.orphan_bytes += header_pos
                del self.buffer[:header_pos]

            if len(self.buffer) < PACKET_LEN:
                break

            candidate = bytes(self.buffer[:PACKET_LEN])
            foot_id = candidate[1]
            if foot_id not in FOOT_LABELS:
                self.stats.bad_foot_ids += 1
                del self.buffer[0]
                continue

            if not checksum_ok(candidate):
                self.stats.bad_checksums += 1
                if len(self.stats.bad_checksum_previews) < 5:
                    self.stats.bad_checksum_previews.append(hex_bytes(candidate[:PACKET_LEN]))
                del self.buffer[0]
                continue

            zones = tuple(
                (candidate[2 + zone * 2] << 8) | candidate[3 + zone * 2]
                for zone in range(ZONE_COUNT)
            )
            self.stats.decoded_packets += 1
            packets.append(
                InsolePacket(
                    packet_index=0,
                    timestamp_s=timestamp_s,
                    can_id=self.can_id,
                    source_frame_index=frame_index,
                    foot_id=foot_id,
                    zones_g=zones,
                    raw_hex=hex_bytes(candidate),
                )
            )
            del self.buffer[:PACKET_LEN]

        return packets


class InsoleCanMonitor:
    def __init__(
        self,
        expected_period_ms: float,
        drop_threshold: float,
        fragment_timeout_ms: float,
        packet_timeout_ms: float,
        max_zone_g: int,
        print_packets: bool,
        quiet: bool,
    ) -> None:
        self.expected_period_ms = expected_period_ms
        self.drop_threshold = drop_threshold
        self.fragment_timeout_ms = fragment_timeout_ms
        self.packet_timeout_ms = packet_timeout_ms
        self.max_zone_g = max_zone_g
        self.print_packets = print_packets
        self.quiet = quiet

        self.parsers: Dict[int, InsoleStreamParser] = {}
        self.frames: List[FrameRecord] = []
        self.packets: List[InsolePacket] = []
        self.packet_intervals: List[PacketIntervalRecord] = []
        self.drop_events: List[DropEvent] = []
        self.timeout_events: List[TimeoutEvent] = []
        self.pressure_anomalies: List[PressureAnomaly] = []

        self.ignored_frames_by_id = 0
        self.ignored_extended_frames = 0
        self.ignored_remote_frames = 0
        self.recv_timeouts = 0

        self._last_frame_timestamp_by_can_id: Dict[int, float] = {}
        self._last_packet_timestamp_by_stream: Dict[Tuple[int, int], float] = {}
        self._last_complete_packet_timestamp_s: Optional[float] = None
        self._packet_timeout_active = False
        self._start_time_s = time.time()

    def feed_frame(
        self,
        can_id: int,
        data: bytes,
        timestamp_s: float,
        dlc: Optional[int] = None,
    ) -> List[InsolePacket]:
        frame_index = len(self.frames) + 1
        previous_frame_ts = self._last_frame_timestamp_by_can_id.get(can_id)
        delta_ms = None
        if previous_frame_ts is not None:
            delta_ms = (timestamp_s - previous_frame_ts) * 1000.0
        self._last_frame_timestamp_by_can_id[can_id] = timestamp_s

        self.frames.append(
            FrameRecord(
                frame_index=frame_index,
                timestamp_s=timestamp_s,
                can_id=can_id,
                dlc=len(data) if dlc is None else dlc,
                data_hex=hex_bytes(data),
                delta_ms=delta_ms,
            )
        )

        parser = self.parsers.setdefault(
            can_id, InsoleStreamParser(can_id, self.fragment_timeout_ms)
        )
        decoded = parser.feed(data, timestamp_s, frame_index)
        for packet in decoded:
            self._register_packet(packet)
        return decoded

    def check_packet_timeout(self, now_s: float) -> Optional[TimeoutEvent]:
        if self.packet_timeout_ms <= 0:
            return None

        reference_s = self._last_complete_packet_timestamp_s or self._start_time_s
        elapsed_ms = (now_s - reference_s) * 1000.0
        if elapsed_ms <= self.packet_timeout_ms:
            return None

        if self._packet_timeout_active:
            return None

        self._packet_timeout_active = True
        event = TimeoutEvent(
            timestamp_s=now_s,
            elapsed_ms=elapsed_ms,
            timeout_ms=self.packet_timeout_ms,
            reason="no_complete_pressure_packet",
        )
        self.timeout_events.append(event)
        return event

    def finalize(self) -> None:
        for parser in self.parsers.values():
            parser.finalize()

    def _register_packet(self, packet: InsolePacket) -> None:
        packet.packet_index = len(self.packets) + 1
        self.packets.append(packet)
        self._last_complete_packet_timestamp_s = packet.timestamp_s
        self._packet_timeout_active = False

        stream_key = (packet.can_id, packet.foot_id)
        previous_packet_ts = self._last_packet_timestamp_by_stream.get(stream_key)
        if previous_packet_ts is not None:
            delta_ms = (packet.timestamp_s - previous_packet_ts) * 1000.0
            sample_rate_hz = 1000.0 / delta_ms if delta_ms > 0 else 0.0
            self.packet_intervals.append(
                PacketIntervalRecord(
                    packet_index=packet.packet_index,
                    timestamp_s=packet.timestamp_s,
                    can_id=packet.can_id,
                    foot_id=packet.foot_id,
                    delta_ms=delta_ms,
                    sample_rate_hz=sample_rate_hz,
                )
            )
            self._check_drop(packet, delta_ms)
        self._last_packet_timestamp_by_stream[stream_key] = packet.timestamp_s

        for zone_index, value_g in enumerate(packet.zones_g, start=1):
            if value_g > self.max_zone_g:
                self.pressure_anomalies.append(
                    PressureAnomaly(
                        packet_index=packet.packet_index,
                        timestamp_s=packet.timestamp_s,
                        can_id=packet.can_id,
                        foot_id=packet.foot_id,
                        zone_index=zone_index,
                        value_g=value_g,
                        limit_g=self.max_zone_g,
                        reason="zone_value_above_limit",
                    )
                )

        if self.print_packets and not self.quiet:
            zones = " ".join(str(value) for value in packet.zones_g)
            print(
                f"packet={packet.packet_index} t={packet.timestamp_s:.6f}s "
                f"can=0x{packet.can_id:X} foot={packet.foot_label} zones_g=[{zones}]"
            )

    def _check_drop(self, packet: InsolePacket, delta_ms: float) -> None:
        if self.expected_period_ms <= 0:
            return

        if delta_ms <= self.expected_period_ms * self.drop_threshold:
            return

        estimated_missing = max(1, int(round(delta_ms / self.expected_period_ms)) - 1)
        self.drop_events.append(
            DropEvent(
                timestamp_s=packet.timestamp_s,
                can_id=packet.can_id,
                foot_id=packet.foot_id,
                delta_ms=delta_ms,
                expected_period_ms=self.expected_period_ms,
                estimated_missing_packets=estimated_missing,
                reason="packet_interval_exceeded_threshold",
            )
        )

    def write_outputs(self, output_dir: Path, hist_bin_ms: float) -> None:
        output_dir.mkdir(parents=True, exist_ok=True)
        self._write_packets_csv(output_dir / "raw_18_zone_trace.csv")
        self._write_frame_intervals_csv(output_dir / "can_frame_intervals.csv")
        self._write_packet_intervals_csv(output_dir / "packet_intervals.csv")
        self._write_histogram_csv(
            output_dir / "can_frame_interval_histogram.csv",
            [frame.delta_ms for frame in self.frames if frame.delta_ms is not None],
            hist_bin_ms,
        )
        self._write_histogram_csv(
            output_dir / "packet_interval_histogram.csv",
            [record.delta_ms for record in self.packet_intervals],
            hist_bin_ms,
        )
        self._write_drop_events_csv(output_dir / "drop_events.csv")
        self._write_timeout_events_csv(output_dir / "timeout_events.csv")
        self._write_pressure_anomalies_csv(output_dir / "pressure_anomalies.csv")
        write_json(output_dir / "summary.json", self.summary(hist_bin_ms))

    def summary(self, hist_bin_ms: float) -> Dict[str, object]:
        frame_intervals = [frame.delta_ms for frame in self.frames if frame.delta_ms is not None]
        packet_intervals = [record.delta_ms for record in self.packet_intervals]
        packets_by_foot: Dict[str, int] = {}
        for packet in self.packets:
            key = f"can_0x{packet.can_id:X}_{packet.foot_label}"
            packets_by_foot[key] = packets_by_foot.get(key, 0) + 1

        return {
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "pressure_unit": "g",
            "pressure_sign_convention": "positive means compressive normal load on the zone",
            "expected_period_ms": self.expected_period_ms,
            "expected_sample_rate_hz": (
                1000.0 / self.expected_period_ms if self.expected_period_ms > 0 else None
            ),
            "drop_threshold_multiplier": self.drop_threshold,
            "fragment_timeout_ms": self.fragment_timeout_ms,
            "packet_timeout_ms": self.packet_timeout_ms,
            "max_zone_g": self.max_zone_g,
            "histogram_bin_ms": hist_bin_ms,
            "total_can_frames": len(self.frames),
            "total_pressure_packets": len(self.packets),
            "packets_by_stream": packets_by_foot,
            "recv_timeouts": self.recv_timeouts,
            "ignored_frames_by_id": self.ignored_frames_by_id,
            "ignored_extended_frames": self.ignored_extended_frames,
            "ignored_remote_frames": self.ignored_remote_frames,
            "estimated_missing_packets": sum(
                event.estimated_missing_packets for event in self.drop_events
            ),
            "drop_events": len(self.drop_events),
            "timeout_events": len(self.timeout_events),
            "pressure_anomalies": len(self.pressure_anomalies),
            "can_frame_interval_ms": summarize_values(frame_intervals),
            "pressure_packet_interval_ms": summarize_values(packet_intervals),
            "pressure_packet_sample_rate_hz": summarize_values(
                [record.sample_rate_hz for record in self.packet_intervals]
            ),
            "streams": {
                f"0x{can_id:X}": parser.stats.as_dict()
                for can_id, parser in sorted(self.parsers.items())
            },
        }

    def print_summary(self, output_dir: Path) -> None:
        packet_interval_summary = summarize_values(
            [record.delta_ms for record in self.packet_intervals]
        )
        frame_interval_summary = summarize_values(
            [frame.delta_ms for frame in self.frames if frame.delta_ms is not None]
        )
        sample_rate_summary = summarize_values(
            [record.sample_rate_hz for record in self.packet_intervals]
        )

        print("\nPressure insole CAN bring-up summary")
        print(f"  output_dir: {output_dir}")
        print(f"  CAN frames: {len(self.frames)}")
        print(f"  pressure packets: {len(self.packets)}")
        print(f"  packet interval ms: {format_summary(packet_interval_summary)}")
        print(f"  sample rate Hz: {format_summary(sample_rate_summary)}")
        print(f"  CAN frame interval ms: {format_summary(frame_interval_summary)}")
        print(
            "  drop check: "
            f"events={len(self.drop_events)} "
            f"estimated_missing={sum(event.estimated_missing_packets for event in self.drop_events)}"
        )
        print(
            "  parser health: "
            f"bad_checksums={sum(p.stats.bad_checksums for p in self.parsers.values())} "
            f"fragment_timeouts={sum(p.stats.fragment_timeouts for p in self.parsers.values())} "
            f"orphan_bytes={sum(p.stats.orphan_bytes for p in self.parsers.values())}"
        )
        print(
            "  timeout/anomaly: "
            f"recv_timeouts={self.recv_timeouts} "
            f"packet_timeouts={len(self.timeout_events)} "
            f"pressure_anomalies={len(self.pressure_anomalies)}"
        )

    def _write_packets_csv(self, path: Path) -> None:
        fieldnames = [
            "packet_index",
            "timestamp_s",
            "timestamp_iso",
            "can_id_hex",
            "source_frame_index",
            "foot_id",
            "foot",
        ]
        fieldnames.extend(f"zone_{idx:02d}_g" for idx in range(1, ZONE_COUNT + 1))
        fieldnames.append("raw_hex")

        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for packet in self.packets:
                row = {
                    "packet_index": packet.packet_index,
                    "timestamp_s": f"{packet.timestamp_s:.9f}",
                    "timestamp_iso": timestamp_iso(packet.timestamp_s),
                    "can_id_hex": f"0x{packet.can_id:X}",
                    "source_frame_index": packet.source_frame_index,
                    "foot_id": packet.foot_id,
                    "foot": packet.foot_label,
                    "raw_hex": packet.raw_hex,
                }
                row.update(
                    {
                        f"zone_{idx:02d}_g": value
                        for idx, value in enumerate(packet.zones_g, start=1)
                    }
                )
                writer.writerow(row)

    def _write_frame_intervals_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "frame_index",
                    "timestamp_s",
                    "timestamp_iso",
                    "can_id_hex",
                    "dlc",
                    "delta_ms",
                    "data_hex",
                ],
            )
            writer.writeheader()
            for frame in self.frames:
                writer.writerow(
                    {
                        "frame_index": frame.frame_index,
                        "timestamp_s": f"{frame.timestamp_s:.9f}",
                        "timestamp_iso": timestamp_iso(frame.timestamp_s),
                        "can_id_hex": f"0x{frame.can_id:X}",
                        "dlc": frame.dlc,
                        "delta_ms": "" if frame.delta_ms is None else f"{frame.delta_ms:.6f}",
                        "data_hex": frame.data_hex,
                    }
                )

    def _write_packet_intervals_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "packet_index",
                    "timestamp_s",
                    "timestamp_iso",
                    "can_id_hex",
                    "foot_id",
                    "foot",
                    "delta_ms",
                    "sample_rate_hz",
                ],
            )
            writer.writeheader()
            for record in self.packet_intervals:
                writer.writerow(
                    {
                        "packet_index": record.packet_index,
                        "timestamp_s": f"{record.timestamp_s:.9f}",
                        "timestamp_iso": timestamp_iso(record.timestamp_s),
                        "can_id_hex": f"0x{record.can_id:X}",
                        "foot_id": record.foot_id,
                        "foot": FOOT_LABELS.get(record.foot_id, f"unknown_{record.foot_id}"),
                        "delta_ms": f"{record.delta_ms:.6f}",
                        "sample_rate_hz": f"{record.sample_rate_hz:.6f}",
                    }
                )

    def _write_histogram_csv(self, path: Path, values: Sequence[float], bin_width_ms: float) -> None:
        rows = histogram(values, bin_width_ms)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["bin_lower_ms", "bin_upper_ms", "count"],
            )
            writer.writeheader()
            for lower_ms, upper_ms, count in rows:
                writer.writerow(
                    {
                        "bin_lower_ms": f"{lower_ms:.6f}",
                        "bin_upper_ms": f"{upper_ms:.6f}",
                        "count": count,
                    }
                )

    def _write_drop_events_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "timestamp_s",
                    "timestamp_iso",
                    "can_id_hex",
                    "foot_id",
                    "foot",
                    "delta_ms",
                    "expected_period_ms",
                    "estimated_missing_packets",
                    "reason",
                ],
            )
            writer.writeheader()
            for event in self.drop_events:
                writer.writerow(
                    {
                        "timestamp_s": f"{event.timestamp_s:.9f}",
                        "timestamp_iso": timestamp_iso(event.timestamp_s),
                        "can_id_hex": f"0x{event.can_id:X}",
                        "foot_id": event.foot_id,
                        "foot": FOOT_LABELS.get(event.foot_id, f"unknown_{event.foot_id}"),
                        "delta_ms": f"{event.delta_ms:.6f}",
                        "expected_period_ms": f"{event.expected_period_ms:.6f}",
                        "estimated_missing_packets": event.estimated_missing_packets,
                        "reason": event.reason,
                    }
                )

    def _write_timeout_events_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["timestamp_s", "timestamp_iso", "elapsed_ms", "timeout_ms", "reason"],
            )
            writer.writeheader()
            for event in self.timeout_events:
                writer.writerow(
                    {
                        "timestamp_s": f"{event.timestamp_s:.9f}",
                        "timestamp_iso": timestamp_iso(event.timestamp_s),
                        "elapsed_ms": f"{event.elapsed_ms:.6f}",
                        "timeout_ms": f"{event.timeout_ms:.6f}",
                        "reason": event.reason,
                    }
                )

    def _write_pressure_anomalies_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "packet_index",
                    "timestamp_s",
                    "timestamp_iso",
                    "can_id_hex",
                    "foot_id",
                    "foot",
                    "zone_index",
                    "value_g",
                    "limit_g",
                    "reason",
                ],
            )
            writer.writeheader()
            for anomaly in self.pressure_anomalies:
                writer.writerow(
                    {
                        "packet_index": anomaly.packet_index,
                        "timestamp_s": f"{anomaly.timestamp_s:.9f}",
                        "timestamp_iso": timestamp_iso(anomaly.timestamp_s),
                        "can_id_hex": f"0x{anomaly.can_id:X}",
                        "foot_id": anomaly.foot_id,
                        "foot": FOOT_LABELS.get(anomaly.foot_id, f"unknown_{anomaly.foot_id}"),
                        "zone_index": anomaly.zone_index,
                        "value_g": anomaly.value_g,
                        "limit_g": anomaly.limit_g,
                        "reason": anomaly.reason,
                    }
                )


def checksum_ok(packet: bytes) -> bool:
    return len(packet) == PACKET_LEN and (sum(packet[: PACKET_LEN - 1]) & 0xFF) == packet[-1]


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def parse_hex_bytes(text: str) -> bytes:
    compact = re.sub(r"[^0-9A-Fa-f]", "", text)
    if len(compact) % 2 != 0:
        raise ValueError(f"odd number of hex digits in: {text!r}")
    return bytes(int(compact[index : index + 2], 16) for index in range(0, len(compact), 2))


def parse_can_id(value: str) -> int:
    return int(value, 0)


def parse_can_ids(values: Optional[Sequence[str]]) -> List[int]:
    if not values:
        return list(DEFAULT_CAN_IDS)

    ids: List[int] = []
    for value in values:
        for chunk in value.split(","):
            chunk = chunk.strip()
            if chunk:
                ids.append(parse_can_id(chunk))
    return ids


def timestamp_iso(timestamp_s: float) -> str:
    if 946684800 <= timestamp_s <= 4102444800:
        return datetime.fromtimestamp(timestamp_s).isoformat(timespec="milliseconds")
    return ""


def percentile(sorted_values: Sequence[float], percent: float) -> Optional[float]:
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]

    position = (len(sorted_values) - 1) * (percent / 100.0)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return sorted_values[int(position)]
    weight = position - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def summarize_values(values: Sequence[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {
            "count": 0,
            "min": None,
            "mean": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "max": None,
        }

    sorted_values = sorted(values)
    return {
        "count": len(sorted_values),
        "min": sorted_values[0],
        "mean": sum(sorted_values) / len(sorted_values),
        "p50": percentile(sorted_values, 50.0),
        "p95": percentile(sorted_values, 95.0),
        "p99": percentile(sorted_values, 99.0),
        "max": sorted_values[-1],
    }


def format_summary(summary: Dict[str, Optional[float]]) -> str:
    if summary["count"] == 0:
        return "count=0"
    return (
        f"count={summary['count']} "
        f"mean={summary['mean']:.3f} "
        f"p50={summary['p50']:.3f} "
        f"p95={summary['p95']:.3f} "
        f"max={summary['max']:.3f}"
    )


def histogram(values: Sequence[float], bin_width_ms: float) -> List[Tuple[float, float, int]]:
    if not values:
        return []
    if bin_width_ms <= 0:
        raise ValueError("--hist-bin-ms must be positive")

    counts: Dict[int, int] = {}
    for value in values:
        bin_index = int(math.floor(value / bin_width_ms))
        counts[bin_index] = counts.get(bin_index, 0) + 1

    return [
        (bin_index * bin_width_ms, (bin_index + 1) * bin_width_ms, count)
        for bin_index, count in sorted(counts.items())
    ]


def write_json(path: Path, data: Dict[str, object]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def iter_fragmented_payloads(payload: bytes, chunk_size: int = 8) -> Iterable[bytes]:
    for offset in range(0, len(payload), chunk_size):
        yield payload[offset : offset + chunk_size]


def run_self_test(monitor: InsoleCanMonitor) -> None:
    sample = parse_hex_bytes(MANUAL_SAMPLE_HEX)
    if not checksum_ok(sample):
        raise SystemExit("Internal self-test packet checksum is invalid.")

    timestamps = [0.000, 0.050, 0.150]
    for packet_start_s in timestamps:
        for fragment_index, fragment in enumerate(iter_fragmented_payloads(sample)):
            monitor.feed_frame(
                SELF_TEST_CAN_ID,
                fragment,
                packet_start_s + fragment_index * 0.001,
                dlc=len(fragment),
            )

    if len(monitor.packets) != 3:
        raise SystemExit(f"Self-test decoded {len(monitor.packets)} packets, expected 3.")

    expected_zones = (78, 17, 262, 0, 362, 48, 319, 14, 0, 701, 0, 178, 55, 175, 482, 8, 21, 4131)
    if monitor.packets[0].zones_g != expected_zones:
        raise SystemExit("Self-test decoded pressure zones do not match the manual sample.")


def run_input_hex(path: Path, monitor: InsoleCanMonitor, offline_frame_period_ms: float) -> None:
    payload = parse_hex_bytes(read_text_guess_encoding(path))
    timestamp_s = 0.0
    for fragment in iter_fragmented_payloads(payload):
        monitor.feed_frame(SELF_TEST_CAN_ID, fragment, timestamp_s, dlc=len(fragment))
        timestamp_s += offline_frame_period_ms / 1000.0


def run_input_candump(path: Path, monitor: InsoleCanMonitor) -> None:
    for timestamp_s, can_id, data in iter_candump_frames(path):
        monitor.feed_frame(can_id, data, timestamp_s, dlc=len(data))


def run_input_wit_log(path: Path, monitor: InsoleCanMonitor) -> None:
    for timestamp_s, can_id, data in iter_wit_log_frames(path):
        monitor.feed_frame(can_id, data, timestamp_s, dlc=len(data))


def iter_candump_frames(path: Path) -> Iterable[Tuple[float, int, bytes]]:
    hash_pattern = re.compile(
        r"^(?:\((?P<timestamp>[0-9]+(?:\.[0-9]+)?)\)\s+)?"
        r"(?P<channel>\S+)\s+"
        r"(?P<can_id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)"
    )
    bracket_pattern = re.compile(
        r"^(?:(?P<timestamp>[0-9]+(?:\.[0-9]+)?)\s+)?"
        r"(?P<channel>\S+)\s+"
        r"(?P<can_id>[0-9A-Fa-f]+)\s+"
        r"\[(?P<dlc>[0-9]+)\]\s+"
        r"(?P<data>(?:[0-9A-Fa-f]{2}\s*)+)"
    )

    synthetic_timestamp_s = 0.0
    for line in read_text_guess_encoding(path).splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue

        match = hash_pattern.match(stripped) or bracket_pattern.match(stripped)
        if not match:
            continue

        timestamp_text = match.groupdict().get("timestamp")
        timestamp_s = float(timestamp_text) if timestamp_text else synthetic_timestamp_s
        synthetic_timestamp_s = timestamp_s + 0.001

        can_id = int(match.group("can_id"), 16)
        data = parse_hex_bytes(match.group("data"))
        yield timestamp_s, can_id, data


def iter_wit_log_frames(path: Path) -> Iterable[Tuple[float, int, bytes]]:
    text = read_text_guess_encoding(path)
    reader = csv.reader(text.splitlines(), delimiter="\t")
    rows = [row for row in reader if any(cell.strip() for cell in row)]
    if not rows:
        return

    header = [cell.strip() for cell in rows[0]]
    time_index = find_header_index(header, ["时间标识", "Time stamp", "Time"])
    id_index = find_header_index(header, ["帧ID", "Frame ID"])
    data_index = find_header_index(header, ["数据(HEX)", "Data (HEX)"])
    direction_index = find_header_index(header, ["传输方向", "Transmission direction"], required=False)

    first_time_of_day_s: Optional[float] = None
    day_offset_s = 0.0
    previous_time_of_day_s: Optional[float] = None

    for row in rows[1:]:
        if max(time_index, id_index, data_index) >= len(row):
            continue
        if direction_index is not None and direction_index < len(row):
            direction = row[direction_index].strip().lower()
            if direction and direction not in {"接收", "receive", "rx"}:
                continue

        time_of_day_s = parse_time_of_day(row[time_index].strip())
        if previous_time_of_day_s is not None and time_of_day_s < previous_time_of_day_s:
            day_offset_s += 24.0 * 3600.0
        previous_time_of_day_s = time_of_day_s
        if first_time_of_day_s is None:
            first_time_of_day_s = time_of_day_s

        timestamp_s = day_offset_s + time_of_day_s - first_time_of_day_s
        can_id = int(row[id_index].strip(), 16)
        data = parse_hex_bytes(row[data_index])
        yield timestamp_s, can_id, data


def find_header_index(header: Sequence[str], names: Sequence[str], required: bool = True) -> Optional[int]:
    for name in names:
        for index, cell in enumerate(header):
            if name.lower() in cell.lower():
                return index
    if required:
        raise ValueError(f"Could not find any of header names {names!r} in {header!r}")
    return None


def parse_time_of_day(value: str) -> float:
    match = re.match(r"^(?P<h>\d{1,2}):(?P<m>\d{2}):(?P<s>\d{2})(?:\.(?P<frac>\d+))?$", value)
    if not match:
        return float(value)
    fraction = match.group("frac") or "0"
    return (
        int(match.group("h")) * 3600.0
        + int(match.group("m")) * 60.0
        + int(match.group("s"))
        + float(f"0.{fraction}")
    )


def read_text_guess_encoding(path: Path) -> str:
    for encoding in ("utf-8-sig", "utf-8", "gbk", "latin-1"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    return path.read_text(errors="replace")


def run_live_capture(args: argparse.Namespace, monitor: InsoleCanMonitor, target_can_ids: List[int]) -> None:
    try:
        import can
    except ImportError as exc:
        raise SystemExit(
            "python-can is required for live capture. Install it with: "
            "python -m pip install python-can"
        ) from exc

    bus_kwargs = {
        "interface": args.interface,
        "channel": args.channel,
        "bitrate": args.bitrate,
        "receive_own_messages": False,
    }
    if args.listen_only:
        bus_kwargs["listen_only"] = True

    if not args.quiet:
        ids = "all" if args.all_can_ids else ", ".join(f"0x{can_id:X}" for can_id in target_can_ids)
        print(
            f"Opening CAN: interface={args.interface} channel={args.channel} "
            f"bitrate={args.bitrate} ids={ids} listen_only={args.listen_only}"
        )

    deadline_s = None if args.duration_s <= 0 else time.time() + args.duration_s
    with can.Bus(**bus_kwargs) as bus:
        while True:
            if deadline_s is not None and time.time() >= deadline_s:
                break

            msg = bus.recv(args.rx_timeout_s)
            now_s = time.time()
            if msg is None:
                monitor.recv_timeouts += 1
                event = monitor.check_packet_timeout(now_s)
                if event is not None and not args.quiet:
                    print(
                        f"TIMEOUT no pressure packet for {event.elapsed_ms:.1f} ms "
                        f"(limit {event.timeout_ms:.1f} ms)"
                    )
                continue

            if msg.is_remote_frame:
                monitor.ignored_remote_frames += 1
                continue
            if msg.is_extended_id and not args.accept_extended:
                monitor.ignored_extended_frames += 1
                continue

            can_id = int(msg.arbitration_id)
            if not args.all_can_ids and can_id not in target_can_ids:
                monitor.ignored_frames_by_id += 1
                continue

            data = bytes(msg.data)
            timestamp_s = float(msg.timestamp) if msg.timestamp else now_s
            decoded = monitor.feed_frame(can_id, data, timestamp_s, dlc=getattr(msg, "dlc", len(data)))
            monitor.check_packet_timeout(timestamp_s)

            if decoded and not args.quiet and not args.print_packets:
                latest = decoded[-1]
                load_sum = sum(latest.zones_g)
                print(
                    f"packet={latest.packet_index} t={latest.timestamp_s:.6f}s "
                    f"can=0x{latest.can_id:X} foot={latest.foot_label} "
                    f"sum_g={load_sum} max_g={max(latest.zones_g)}"
                )


def list_serial_ports() -> None:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise SystemExit("pyserial is required to list serial ports: python -m pip install pyserial") from exc

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    for port in ports:
        print(f"{port.device}: {port.description} [{port.hwid}]")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Passive 18-zone pressure-insole CAN bring-up verifier."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--self-test", action="store_true", help="Run parser self-test with the manual sample packet.")
    mode.add_argument("--input-hex", type=Path, help="Replay raw hex bytes from a text file.")
    mode.add_argument("--input-candump", type=Path, help="Replay candump-style CAN log.")
    mode.add_argument("--input-wit-log", type=Path, help="Replay WitMotion UART-CAN GUI TSV log.")
    mode.add_argument("--list-serial", action="store_true", help="List Windows serial ports and exit.")

    parser.add_argument("--interface", help="python-can backend, e.g. pcan, kvaser, vector, slcan.")
    parser.add_argument("--channel", help="CAN channel, e.g. PCAN_USBBUS1 or COM11.")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate, default 1000000.")
    parser.add_argument("--can-id", action="append", help="CAN ID to reconstruct, e.g. 0x601. Can be repeated or comma-separated.")
    parser.add_argument("--all-can-ids", action="store_true", help="Create one byte-stream parser per observed CAN ID.")
    parser.add_argument("--accept-extended", action="store_true", help="Accept extended CAN frames.")
    parser.add_argument("--listen-only", action="store_true", help="Request listen-only mode if the backend supports it.")
    parser.add_argument("--duration-s", type=float, default=0.0, help="Live capture duration in seconds; 0 means run until Ctrl+C.")
    parser.add_argument("--rx-timeout-s", type=float, default=0.1, help="CAN receive timeout in seconds.")
    parser.add_argument("--packet-timeout-ms", type=float, default=200.0, help="No-complete-packet timeout, default 200 ms.")
    parser.add_argument("--fragment-timeout-ms", type=float, default=30.0, help="Max gap inside one fragmented TTL packet, default 30 ms.")
    parser.add_argument("--expected-period-ms", type=float, default=DEFAULT_EXPECTED_PERIOD_MS, help="Expected pressure packet period, default 50 ms.")
    parser.add_argument("--drop-threshold", type=float, default=1.5, help="Drop event threshold as a multiplier of expected period.")
    parser.add_argument("--max-zone-g", type=int, default=70000, help="Pressure anomaly limit per zone in g, default 70000.")
    parser.add_argument("--hist-bin-ms", type=float, default=1.0, help="Histogram bin width in ms.")
    parser.add_argument("--offline-frame-period-ms", type=float, default=1.0, help="Synthetic frame spacing for --input-hex.")
    parser.add_argument("--output-dir", type=Path, help="Directory for CSV/JSON outputs.")
    parser.add_argument("--print-packets", action="store_true", help="Print all 18 zone values for every decoded packet.")
    parser.add_argument("--quiet", action="store_true", help="Suppress live packet prints; final summary still prints.")
    return parser


def default_output_dir() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"insole_can_{stamp}"


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.list_serial:
        list_serial_ports()
        return 0

    output_dir = args.output_dir or default_output_dir()
    target_can_ids = parse_can_ids(args.can_id)
    monitor = InsoleCanMonitor(
        expected_period_ms=args.expected_period_ms,
        drop_threshold=args.drop_threshold,
        fragment_timeout_ms=args.fragment_timeout_ms,
        packet_timeout_ms=args.packet_timeout_ms,
        max_zone_g=args.max_zone_g,
        print_packets=args.print_packets,
        quiet=args.quiet,
    )

    try:
        if args.self_test:
            run_self_test(monitor)
        elif args.input_hex:
            run_input_hex(args.input_hex, monitor, args.offline_frame_period_ms)
        elif args.input_candump:
            run_input_candump(args.input_candump, monitor)
        elif args.input_wit_log:
            run_input_wit_log(args.input_wit_log, monitor)
        else:
            if not args.interface or not args.channel:
                parser.error("--interface and --channel are required for live capture.")
            run_live_capture(args, monitor, target_can_ids)
    except KeyboardInterrupt:
        if not args.quiet:
            print("\nInterrupted by user; writing partial capture.")

    monitor.finalize()
    monitor.write_outputs(output_dir, args.hist_bin_ms)
    monitor.print_summary(output_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
