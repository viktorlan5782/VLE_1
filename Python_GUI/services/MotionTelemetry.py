from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable, List

try:
    from PySide6 import QtCore
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e


MAGIC = b"MOTN"
VERSION = 1
ZONE_COUNT = 18
HEADER_STRUCT = struct.Struct("<4sBBH")
FRAME_STRUCT = struct.Struct("<4sBBHIIII18HBBBBII16f")
FRAME_LEN = FRAME_STRUCT.size

PRESSURE_VALID = 1 << 0
COP_VALID = 1 << 1
TAU_PROXY_VALID = 1 << 2
INTENT_UNLOAD_REQUEST = 1 << 3
HEEL_STRIKE_PULSE = 1 << 4
TOE_OFF_PULSE = 1 << 5


@dataclass(frozen=True)
class MotionFrame:
    side: int
    timestamp_us: int
    sample_age_us: int
    raw_fault_flags: int
    hl_fault_flags: int
    zones_g: tuple[int, ...]
    validity: int
    gait_state: int
    motion_state: int
    motion_intent: int
    insole_heel_strike_timestamp_us: int
    insole_toe_off_timestamp_us: int
    p_total: float
    p_fore: float
    p_arch: float
    p_heel: float
    p_medial: float
    p_lateral: float
    fz_n: float
    cop_x_mm: float
    cop_y_mm: float
    cop_ap_norm: float
    cop_ml_norm: float
    cop_ap_dot_norm_s: float
    cop_ml_dot_norm_s: float
    stance_phase: float
    tau_proxy_nm: float
    intent_confidence: float

    @property
    def side_name(self) -> str:
        return "Left" if self.side == 1 else "Right"

    @property
    def pressure_valid(self) -> bool:
        return bool(self.validity & PRESSURE_VALID)

    @property
    def cop_valid(self) -> bool:
        return bool(self.validity & COP_VALID)

    @property
    def tau_proxy_valid(self) -> bool:
        return bool(self.validity & TAU_PROXY_VALID)

    @property
    def intent_unload_request(self) -> bool:
        return bool(self.validity & INTENT_UNLOAD_REQUEST)

    @property
    def insole_heel_strike(self) -> bool:
        return bool(self.validity & HEEL_STRIKE_PULSE)

    @property
    def insole_toe_off(self) -> bool:
        return bool(self.validity & TOE_OFF_PULSE)

    @property
    def has_sample(self) -> bool:
        return self.timestamp_us != 0

    @property
    def stale(self) -> bool:
        return self.sample_age_us == 0xFFFFFFFF or self.sample_age_us > 150000 or not self.pressure_valid

    @staticmethod
    def csv_header() -> list[str]:
        return [
            "host_epoch",
            "side",
            "timestamp_us",
            "sample_age_us",
            *[f"zone_{i:02d}_g" for i in range(1, ZONE_COUNT + 1)],
            "pressure_valid",
            "cop_valid",
            "tau_proxy_valid",
            "intent_unload_request",
            "raw_fault_flags",
            "hl_fault_flags",
            "p_total",
            "p_fore",
            "p_arch",
            "p_heel",
            "p_medial",
            "p_lateral",
            "fz_n",
            "cop_x_mm",
            "cop_y_mm",
            "cop_ap_norm",
            "cop_ml_norm",
            "cop_ap_dot_norm_s",
            "cop_ml_dot_norm_s",
            "stance_phase",
            "tau_proxy_nm",
            "gait_state",
            "motion_state",
            "motion_intent",
            "intent_confidence",
            "insole_heel_strike",
            "insole_toe_off",
            "insole_heel_strike_timestamp_us",
            "insole_toe_off_timestamp_us",
        ]

    def to_csv_row(self, host_epoch: float) -> list[object]:
        return [
            f"{host_epoch:.6f}",
            self.side_name.lower(),
            self.timestamp_us,
            self.sample_age_us,
            *self.zones_g,
            int(self.pressure_valid),
            int(self.cop_valid),
            int(self.tau_proxy_valid),
            int(self.intent_unload_request),
            self.raw_fault_flags,
            self.hl_fault_flags,
            self.p_total,
            self.p_fore,
            self.p_arch,
            self.p_heel,
            self.p_medial,
            self.p_lateral,
            self.fz_n,
            self.cop_x_mm,
            self.cop_y_mm,
            self.cop_ap_norm,
            self.cop_ml_norm,
            self.cop_ap_dot_norm_s,
            self.cop_ml_dot_norm_s,
            self.stance_phase,
            self.tau_proxy_nm,
            self.gait_state,
            self.motion_state,
            self.motion_intent,
            self.intent_confidence,
            int(self.insole_heel_strike),
            int(self.insole_toe_off),
            self.insole_heel_strike_timestamp_us,
            self.insole_toe_off_timestamp_us,
        ]


def parse_motion_frame(frame_bytes: bytes) -> MotionFrame:
    if len(frame_bytes) != FRAME_LEN:
        raise ValueError(f"unexpected motion frame length: {len(frame_bytes)}")

    unpacked = FRAME_STRUCT.unpack(frame_bytes)
    magic, version, side, frame_len = unpacked[:4]
    if magic != MAGIC or version != VERSION or frame_len != FRAME_LEN:
        raise ValueError("invalid motion frame header")

    timestamp_us, sample_age_us, raw_fault_flags, hl_fault_flags = unpacked[4:8]
    zones_g = tuple(unpacked[8:26])
    validity, gait_state, motion_state, motion_intent = unpacked[26:30]
    hs_ts, to_ts = unpacked[30:32]
    floats = unpacked[32:]

    return MotionFrame(
        side=side,
        timestamp_us=timestamp_us,
        sample_age_us=sample_age_us,
        raw_fault_flags=raw_fault_flags,
        hl_fault_flags=hl_fault_flags,
        zones_g=zones_g,
        validity=validity,
        gait_state=gait_state,
        motion_state=motion_state,
        motion_intent=motion_intent,
        insole_heel_strike_timestamp_us=hs_ts,
        insole_toe_off_timestamp_us=to_ts,
        p_total=floats[0],
        p_fore=floats[1],
        p_arch=floats[2],
        p_heel=floats[3],
        p_medial=floats[4],
        p_lateral=floats[5],
        fz_n=floats[6],
        cop_x_mm=floats[7],
        cop_y_mm=floats[8],
        cop_ap_norm=floats[9],
        cop_ml_norm=floats[10],
        cop_ap_dot_norm_s=floats[11],
        cop_ml_dot_norm_s=floats[12],
        stance_phase=floats[13],
        tau_proxy_nm=floats[14],
        intent_confidence=floats[15],
    )


class MotionTelemetryBridge(QtCore.QObject):
    motionFrameUpdated = QtCore.Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._buffer = bytearray()

    def reset(self):
        self._buffer.clear()

    def route_bytes(self, data: bytes) -> list[bytes]:
        if data:
            self._buffer.extend(data)

        passthrough: list[bytes] = []
        while self._buffer:
            magic_index = self._buffer.find(MAGIC)
            if magic_index < 0:
                keep_len = self._trailing_magic_prefix_len(self._buffer)
                emit_len = len(self._buffer) - keep_len
                if emit_len:
                    passthrough.append(bytes(self._buffer[:emit_len]))
                    del self._buffer[:emit_len]
                break

            if magic_index > 0:
                passthrough.append(bytes(self._buffer[:magic_index]))
                del self._buffer[:magic_index]

            if len(self._buffer) < HEADER_STRUCT.size:
                break

            magic, version, _side, frame_len = HEADER_STRUCT.unpack_from(self._buffer)
            if magic != MAGIC or version != VERSION or frame_len != FRAME_LEN:
                passthrough.append(bytes(self._buffer[:1]))
                del self._buffer[:1]
                continue

            if len(self._buffer) < frame_len:
                break

            frame_bytes = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]
            try:
                self.motionFrameUpdated.emit(parse_motion_frame(frame_bytes))
            except ValueError:
                continue

        return passthrough

    @staticmethod
    def _trailing_magic_prefix_len(buffer: Iterable[int]) -> int:
        raw = bytes(buffer)
        max_len = min(len(raw), len(MAGIC) - 1)
        for size in range(max_len, 0, -1):
            if raw[-size:] == MAGIC[:size]:
                return size
        return 0
