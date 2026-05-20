from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e

try:
    import pyqtgraph as pg
except ImportError as e:
    raise SystemExit("pyqtgraph is required. Install with: pip install pyqtgraph") from e

from services.MotionTelemetry import MotionFrame
from utils import UIConfig, create_separator, style_button


FOOT_WIDTH_MM = 79.61
ACTIVE_ZONE_THRESHOLD_G = 50


def _rounded_rect_vertices(xr: tuple[float, float], yr: tuple[float, float], corner: str, radius: float) -> tuple[tuple[float, float], ...]:
    xmin, xmax = xr
    ymin, ymax = yr
    points: list[tuple[float, float]] = []
    if corner == "bl":
        center = (xmin + radius, ymin + radius)
        points.append((xmax, ymax))
        points.append((xmax, ymin))
        for i in range(25):
            theta = math.radians(270 + (180 - 270) * i / 24)
            points.append((center[0] + radius * math.cos(theta), center[1] + radius * math.sin(theta)))
        points.append((xmin, ymax))
    else:
        center = (xmax - radius, ymin + radius)
        points.append((xmax, ymax))
        for i in range(25):
            theta = math.radians(0 + (-90) * i / 24)
            points.append((center[0] + radius * math.cos(theta), center[1] + radius * math.sin(theta)))
        points.append((xmin, ymin))
        points.append((xmin, ymax))
    return tuple(points)


# Polygons are ordered by firmware raw zone index: zones_g[0] is manual point 1,
# zones_g[17] is manual point 18. Geometry is reconstructed from
# Documentation/Insole_files/insole_geometry_parameter.m.
ZONE_POLYGONS_RIGHT_MM: tuple[tuple[tuple[float, float], ...], ...] = (
    ((2.09, 203.47), (19.21, 203.47), (19.21, 172.80), (7.81, 172.80)),     # 点1
    ((0.00, 207.73), (13.81, 207.73), (13.81, 247.57)),                    # 点2
    _rounded_rect_vertices((32.24, 49.11), (36.68, 62.96), "bl", 16.87),   # 点3
    ((32.24, 68.72), (49.11, 68.72), (49.11, 100.68), (32.24, 100.68)),    # 点4
    ((23.86, 133.33), (40.31, 133.33), (40.31, 104.94), (28.31, 104.94)),  # 点5
    ((11.76, 168.54), (33.71, 168.54), (33.71, 138.58), (22.81, 138.58)),  # 点6
    ((22.97, 172.80), (38.46, 172.80), (38.46, 203.47), (22.97, 203.47)),  # 点7
    ((17.57, 207.73), (28.99, 207.73), (28.99, 252.20), (17.57, 249.23)),  # 点8
    _rounded_rect_vertices((55.83, 72.70), (36.68, 62.96), "br", 16.87),   # 点9
    ((55.83, 68.72), (72.70, 68.72), (72.70, 100.68), (55.83, 100.68)),    # 点10
    ((44.07, 104.94), (57.79, 104.94), (57.79, 133.33), (44.07, 133.33)),  # 点11
    ((40.56, 138.58), (56.78, 138.58), (56.78, 168.54), (40.56, 168.54)),  # 点12
    ((42.22, 172.80), (57.71, 172.80), (57.71, 203.47), (42.22, 203.47)),  # 点13
    ((32.75, 207.73), (44.67, 207.73), (44.67, 246.73), (32.75, 251.73)),  # 点14
    ((61.55, 133.33), (77.24, 133.33), (75.55, 104.94), (61.55, 104.94)),  # 点15
    ((63.83, 138.58), (78.85, 138.58), (78.85, 168.54), (63.83, 168.54)),  # 点16
    ((61.47, 203.47), (72.27, 203.47), (79.61, 172.80), (61.47, 172.80)),  # 点17
    ((48.43, 207.73), (70.48, 207.73), (48.43, 242.66)),                   # 点18
)


def _mirror_polygon(vertices: tuple[tuple[float, float], ...]) -> tuple[tuple[float, float], ...]:
    return tuple((FOOT_WIDTH_MM - x, y) for x, y in vertices)


def _polygon_center(vertices: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    return (
        sum(x for x, _ in vertices) / len(vertices),
        sum(y for _, y in vertices) / len(vertices),
    )

GAIT_STATE_NAMES = {
    0: "Invalid",
    1: "Swing",
    2: "Initial Contact",
    3: "Loading Response",
    4: "Mid Stance",
    5: "Terminal Stance",
    6: "Pre Swing",
}
MOTION_STATE_NAMES = {
    0: "Unload",
    1: "Static",
    2: "Walking",
    255: "Unknown",
}
MOTION_INTENT_NAMES = {
    0: "Normal",
    1: "Medial Deviation",
    2: "Lateral Deviation",
    3: "Unload",
    255: "Unknown",
}


@dataclass
class _SideRuntime:
    last_frame: MotionFrame | None = None
    last_hs_ts: int = 0
    last_to_ts: int = 0
    current_hs_ts: int = 0
    current_tau_samples: list[tuple[int, float]] = field(default_factory=list)
    completed_tau_cycles: deque = field(default_factory=lambda: deque(maxlen=5))
    cop_path: deque = field(default_factory=lambda: deque(maxlen=240))


class _FootMotionPanel(QtWidgets.QWidget):
    def __init__(self, side_name: str, is_left: bool, parent=None):
        super().__init__(parent)
        self.side_name = side_name
        self.is_left = is_left
        self._zone_polygons = tuple(
            _mirror_polygon(vertices) if is_left else vertices
            for vertices in ZONE_POLYGONS_RIGHT_MM
        )
        self._zone_centers = tuple(_polygon_center(vertices) for vertices in self._zone_polygons)
        self.runtime = _SideRuntime()
        self._hs_flash_timer = QtCore.QTimer(self)
        self._hs_flash_timer.setSingleShot(True)
        self._hs_flash_timer.timeout.connect(lambda: self._set_event_style(self.lbl_hs, False))
        self._to_flash_timer = QtCore.QTimer(self)
        self._to_flash_timer.setSingleShot(True)
        self._to_flash_timer.timeout.connect(lambda: self._set_event_style(self.lbl_to, False))
        self._build_ui()
        self.set_unavailable()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel(self.side_name)
        title_font = title.font()
        title_font.setPointSize(UIConfig.FONT_SUBTITLE)
        title_font.setBold(True)
        title.setFont(title_font)
        header.addWidget(title)
        header.addStretch(1)
        self.lbl_status = QtWidgets.QLabel("Unavailable")
        header.addWidget(self.lbl_status)
        layout.addLayout(header)

        top_row = QtWidgets.QHBoxLayout()
        self.pressure_plot = pg.PlotWidget()
        self.pressure_plot.setAspectLocked(True)
        self.pressure_plot.setXRange(-5, FOOT_WIDTH_MM + 5, padding=0)
        self.pressure_plot.setYRange(30, 255, padding=0)
        self.pressure_plot.setLabel("left", "AP y (mm)")
        self.pressure_plot.setLabel("bottom", "ML x (mm)")
        self.pressure_plot.showGrid(x=True, y=True, alpha=0.18)
        self.pressure_plot.invertY(False)
        self._zone_items: list[QtWidgets.QGraphicsPolygonItem] = []
        self._zone_labels = []
        for idx, vertices in enumerate(self._zone_polygons, start=1):
            polygon = QtGui.QPolygonF([QtCore.QPointF(x, y) for x, y in vertices])
            item = QtWidgets.QGraphicsPolygonItem(polygon)
            item.setPen(QtGui.QPen(QtGui.QColor("#263238"), 1.2))
            item.setBrush(QtGui.QBrush(QtGui.QColor("#455A64")))
            item.setZValue(1)
            self.pressure_plot.addItem(item)
            self._zone_items.append(item)

            x, y = self._zone_centers[idx - 1]
            label = pg.TextItem("", anchor=(0.5, 0.5))
            label.setZValue(2)
            label.setPos(x, y)
            self.pressure_plot.addItem(label)
            self._zone_labels.append(label)
        self._cop_curve = self.pressure_plot.plot(pen=pg.mkPen("#1565C0", width=2))
        self._cop_point = pg.ScatterPlotItem(size=12, brush=pg.mkBrush("#D32F2F"), pen=pg.mkPen("#FFFFFF", width=1))
        self.pressure_plot.addItem(self._cop_point)
        top_row.addWidget(self.pressure_plot, 2)

        metrics = QtWidgets.QFormLayout()
        metrics.setLabelAlignment(QtCore.Qt.AlignLeft)
        self.lbl_gait = QtWidgets.QLabel("--")
        self.lbl_motion = QtWidgets.QLabel("--")
        self.lbl_intent = QtWidgets.QLabel("--")
        self.lbl_hs = QtWidgets.QLabel("HS --")
        self.lbl_to = QtWidgets.QLabel("TO --")
        self.lbl_load = QtWidgets.QLabel("--")
        self.lbl_tau = QtWidgets.QLabel("--")
        self.lbl_cop = QtWidgets.QLabel("--")
        self.lbl_fault = QtWidgets.QLabel("--")
        for label in (self.lbl_intent, self.lbl_load, self.lbl_cop, self.lbl_fault):
            label.setWordWrap(True)
        metrics.addRow("步态相位 (gait state)", self.lbl_gait)
        metrics.addRow("运动状态 (motion state)", self.lbl_motion)
        metrics.addRow("运动意图 (motion intent)", self.lbl_intent)
        metrics.addRow("步态事件 (gait event)", self._event_row())
        metrics.addRow("载荷 (load)", self.lbl_load)
        metrics.addRow("踝矩代理 (tau proxy)", self.lbl_tau)
        metrics.addRow("COP 动力学", self.lbl_cop)
        metrics.addRow("故障 (fault)", self.lbl_fault)
        metrics_widget = QtWidgets.QWidget()
        metrics_widget.setLayout(metrics)
        top_row.addWidget(metrics_widget, 2)
        layout.addLayout(top_row, 2)

        self.tau_plot = pg.PlotWidget()
        self.tau_plot.showGrid(x=True, y=True, alpha=0.25)
        self.tau_plot.setLabel("left", "tau proxy (Nm)")
        self.tau_plot.setLabel("bottom", "gait cycle (%)")
        self.tau_plot.setXRange(0, 100, padding=0)
        self.tau_curves = [
            self.tau_plot.plot(pen=pg.mkPen(color, width=2))
            for color in ("#90A4AE", "#78909C", "#607D8B", "#455A64", "#D32F2F")
        ]
        layout.addWidget(self.tau_plot, 1)

    def _event_row(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(widget)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        row.addWidget(self.lbl_hs)
        row.addWidget(self.lbl_to)
        row.addStretch(1)
        self._set_event_style(self.lbl_hs, False)
        self._set_event_style(self.lbl_to, False)
        return widget

    @staticmethod
    def _set_event_style(label: QtWidgets.QLabel, active: bool):
        if active:
            label.setStyleSheet("padding: 3px 6px; background-color: #2E7D32; color: white; border-radius: 3px;")
        else:
            label.setStyleSheet("padding: 3px 6px; background-color: #616161; color: white; border-radius: 3px;")

    def reset(self):
        self.runtime = _SideRuntime()
        self.set_unavailable()
        self._update_tau_plot()

    def set_unavailable(self):
        self.lbl_status.setText("Unavailable")
        self.lbl_status.setStyleSheet("color: #757575;")
        self.lbl_gait.setText("--")
        self.lbl_motion.setText("--")
        self.lbl_intent.setText("--")
        self.lbl_hs.setText("HS --")
        self.lbl_to.setText("TO --")
        self.lbl_load.setText("--")
        self.lbl_tau.setText("--")
        self.lbl_cop.setText("--")
        self.lbl_fault.setText("--")
        self._update_pressure_visual(tuple(0 for _ in range(18)), stale=True)
        self._cop_curve.setData([], [])
        self._cop_point.setData([], [])

    def apply_frame(self, frame: MotionFrame):
        runtime = self.runtime
        runtime.last_frame = frame

        new_hs = frame.insole_heel_strike_timestamp_us and frame.insole_heel_strike_timestamp_us != runtime.last_hs_ts
        new_to = frame.insole_toe_off_timestamp_us and frame.insole_toe_off_timestamp_us != runtime.last_to_ts

        if new_hs:
            self._close_tau_cycle(frame.insole_heel_strike_timestamp_us)
            runtime.current_hs_ts = frame.insole_heel_strike_timestamp_us
            runtime.current_tau_samples = []
            runtime.cop_path.clear()
            runtime.last_hs_ts = frame.insole_heel_strike_timestamp_us
            self.lbl_hs.setText(f"HS {self._fmt_ts(runtime.last_hs_ts)}")
            self._set_event_style(self.lbl_hs, True)
            self._hs_flash_timer.start(250)

        if new_to:
            runtime.last_to_ts = frame.insole_toe_off_timestamp_us
            self.lbl_to.setText(f"TO {self._fmt_ts(runtime.last_to_ts)}")
            self._set_event_style(self.lbl_to, True)
            self._to_flash_timer.start(250)

        if frame.tau_proxy_valid and runtime.current_hs_ts:
            runtime.current_tau_samples.append((frame.timestamp_us, frame.tau_proxy_nm))

        if frame.cop_valid:
            runtime.cop_path.append((self._display_x_mm(frame.cop_x_mm), frame.cop_y_mm))

        stale = frame.stale
        self._update_pressure_visual(frame.zones_g, stale=stale)
        self._update_cop_visual(frame)
        self._update_metrics(frame)
        self._update_tau_plot()

    def _close_tau_cycle(self, next_hs_ts: int):
        runtime = self.runtime
        if not runtime.current_hs_ts or not runtime.current_tau_samples:
            return
        duration_us = self._delta_us(runtime.current_hs_ts, next_hs_ts)
        if duration_us < 200000 or duration_us > 3000000 or len(runtime.current_tau_samples) < 4:
            return

        x_vals = []
        y_vals = []
        for timestamp_us, tau_nm in runtime.current_tau_samples:
            phase_pct = 100.0 * self._delta_us(runtime.current_hs_ts, timestamp_us) / duration_us
            if 0.0 <= phase_pct <= 100.0:
                x_vals.append(phase_pct)
                y_vals.append(tau_nm)
        if len(x_vals) >= 4:
            runtime.completed_tau_cycles.append((x_vals, y_vals))

    def _update_pressure_visual(self, zones_g: tuple[int, ...], stale: bool):
        max_zone = max(max(zones_g, default=0), 1)
        for idx, value in enumerate(zones_g, start=1):
            if stale:
                fill = QtGui.QColor("#607D8B")
                edge = QtGui.QColor("#37474F")
            else:
                ratio = min(1.0, float(value) / max_zone)
                fill = QtGui.QColor(
                    int(38 + 217 * ratio),
                    int(80 + 120 * ratio),
                    int(100 - 80 * ratio),
                )
                edge = QtGui.QColor("#263238")

            self._zone_items[idx - 1].setBrush(QtGui.QBrush(fill))
            self._zone_items[idx - 1].setPen(QtGui.QPen(edge, 1.2))
            active = (not stale) and value >= ACTIVE_ZONE_THRESHOLD_G
            color = "#D50000" if active else ("#CFD8DC" if stale else "#F5F5F5")
            weight = 800 if active else 650
            font_size = 14 if active else 12
            self._zone_labels[idx - 1].setHtml(
                "<div style='text-align:center; "
                f"color:{color}; font-size:{font_size}pt; font-weight:{weight}; "
                "line-height:1.0;'>"
                f"{idx}<br>{value}"
                "</div>"
            )

    def _update_cop_visual(self, frame: MotionFrame):
        if not frame.cop_valid:
            self._cop_curve.setData([], [])
            self._cop_point.setData([], [])
            return

        path = list(self.runtime.cop_path)
        if path:
            xs, ys = zip(*path)
            self._cop_curve.setData(xs, ys)
        self._cop_point.setData([self._display_x_mm(frame.cop_x_mm)], [frame.cop_y_mm])

    def _update_metrics(self, frame: MotionFrame):
        if frame.stale:
            self.lbl_status.setText(f"Stale / invalid ({frame.sample_age_us} us)")
            self.lbl_status.setStyleSheet("color: #C62828; font-weight: bold;")
        else:
            self.lbl_status.setText(f"Live ({frame.sample_age_us} us)")
            self.lbl_status.setStyleSheet("color: #2E7D32; font-weight: bold;")

        self.lbl_gait.setText(GAIT_STATE_NAMES.get(frame.gait_state, f"Unknown {frame.gait_state}"))
        self.lbl_motion.setText(MOTION_STATE_NAMES.get(frame.motion_state, f"Unknown {frame.motion_state}"))
        self.lbl_intent.setText(
            f"{MOTION_INTENT_NAMES.get(frame.motion_intent, f'Unknown {frame.motion_intent}')} "
            f"({frame.intent_confidence:.2f})"
        )
        self.lbl_load.setText(
            f"total={frame.p_total:.0f} g / {frame.fz_n:.1f} N; "
            f"heel={frame.p_heel:.0f}, arch={frame.p_arch:.0f}, fore={frame.p_fore:.0f} g; "
            f"medial={frame.p_medial:.0f}, lateral={frame.p_lateral:.0f} g"
        )
        tau_text = f"{frame.tau_proxy_nm:.2f} Nm" if frame.tau_proxy_valid else "invalid / 0.00 Nm"
        self.lbl_tau.setText(tau_text)
        if frame.cop_valid:
            self.lbl_cop.setText(
                f"x={frame.cop_x_mm:.1f} mm, y={frame.cop_y_mm:.1f} mm; "
                f"vAP={frame.cop_ap_dot_norm_s:.2f}, vML={frame.cop_ml_dot_norm_s:.2f}"
            )
        else:
            self.lbl_cop.setText("invalid / zero")
        self.lbl_fault.setText(f"raw=0x{frame.raw_fault_flags:08X}, hl=0x{frame.hl_fault_flags:08X}")

    def _update_tau_plot(self):
        curves = list(self.runtime.completed_tau_cycles)
        visible_curves = curves[-5:]
        pad = 5 - len(visible_curves)
        for curve in self.tau_curves[:pad]:
            curve.setData([], [])
        for curve, (x_vals, y_vals) in zip(self.tau_curves[pad:], visible_curves):
            curve.setData(x_vals, y_vals)

    @staticmethod
    def _fmt_ts(timestamp_us: int) -> str:
        return f"{timestamp_us / 1_000_000.0:.3f}s"

    @staticmethod
    def _delta_us(start_us: int, end_us: int) -> int:
        return (end_us - start_us) & 0xFFFFFFFF

    def _display_x_mm(self, x_mm: float) -> float:
        return FOOT_WIDTH_MM - x_mm if self.is_left else x_mm


class MotionSensingPage(QtWidgets.QWidget):
    backRequested = QtCore.Signal()
    deviceStartRequested = QtCore.Signal()
    deviceStopRequested = QtCore.Signal()
    markTrialRequested = QtCore.Signal()
    endTrialRequested = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("MotionSensingPage")
        self._is_paused = False
        self._build_ui()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE)
        layout.setSpacing(UIConfig.SPACING_LARGE)

        header = QtWidgets.QHBoxLayout()
        self.btn_back = QtWidgets.QPushButton("Back")
        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_mark = QtWidgets.QPushButton("Mark Trial (0)")
        self.btn_end_trial = QtWidgets.QPushButton("END TRIAL")
        for btn in (self.btn_back, self.btn_pause, self.btn_mark):
            style_button(btn, height=UIConfig.BTN_HEIGHT_MEDIUM, width=UIConfig.BTN_WIDTH_SMALL,
                         font_size=UIConfig.FONT_MEDIUM, padding="6px 12px")
        style_button(self.btn_end_trial, height=UIConfig.BTN_HEIGHT_MEDIUM, width=UIConfig.BTN_WIDTH_SMALL,
                     font_size=UIConfig.FONT_MEDIUM, padding="6px 12px")
        self.btn_end_trial.setStyleSheet(
            f"background-color: {UIConfig.COLOR_CRITICAL}; color: white; padding: 6px 12px; "
            "font-weight: bold; border-radius: 4px;"
        )

        title = QtWidgets.QLabel("Motion Sensing")
        title_font = title.font()
        title_font.setPointSize(UIConfig.FONT_TITLE)
        title.setFont(title_font)
        title.setAlignment(QtCore.Qt.AlignCenter)

        header.addWidget(self.btn_back)
        header.addWidget(self.btn_pause)
        header.addWidget(self.btn_mark)
        header.addStretch(1)
        header.addWidget(title, 1)
        header.addStretch(1)
        header.addWidget(self.btn_end_trial)
        layout.addLayout(header)
        layout.addWidget(create_separator())

        panels = QtWidgets.QHBoxLayout()
        panels.setSpacing(10)
        self.left_panel = _FootMotionPanel("Left Foot", is_left=True)
        self.right_panel = _FootMotionPanel("Right Foot", is_left=False)
        panels.addWidget(self.left_panel, 1)
        panels.addWidget(self.right_panel, 1)
        layout.addLayout(panels, 1)

        self.btn_back.clicked.connect(self.backRequested.emit)
        self.btn_pause.clicked.connect(self._toggle_pause)
        self.btn_mark.clicked.connect(self.markTrialRequested.emit)
        self.btn_end_trial.clicked.connect(self.endTrialRequested.emit)

    def start_monitoring(self):
        self._is_paused = False
        self.btn_pause.setText("Pause")

    def stop_monitoring(self):
        pass

    def clear(self):
        self.left_panel.reset()
        self.right_panel.reset()

    def update_mark_count(self, count: int):
        self.btn_mark.setText(f"Mark Trial ({count})")

    def apply_frame(self, frame: MotionFrame):
        if frame.side == 1:
            self.left_panel.apply_frame(frame)
        elif frame.side == 2:
            self.right_panel.apply_frame(frame)

    def _toggle_pause(self):
        if not self._is_paused:
            self._is_paused = True
            self.btn_pause.setText("Play")
            self.deviceStopRequested.emit()
        else:
            self._is_paused = False
            self.btn_pause.setText("Pause")
            self.deviceStartRequested.emit()
