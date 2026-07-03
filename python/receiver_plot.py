#!/usr/bin/env python3
"""
Receive RTL_SDR_Scanner JSON data and plot live spectrum data.

Default endpoint:
    http://127.0.0.1:23568/api/service
"""

import argparse
import json
import logging
import threading
from dataclasses import dataclass, field
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("receiver_plot")


@dataclass
class SpectrumFrame:
    start_hz: float = 0.0
    end_hz: float = 0.0
    spectrum: list[float] = field(default_factory=list)
    detections: list[dict] = field(default_factory=list)
    received_at: datetime = field(default_factory=datetime.now)


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame: SpectrumFrame | None = None
        self._dirty = False

    def update(self, frame: SpectrumFrame) -> None:
        with self._lock:
            self._frame = frame
            self._dirty = True

    def take_if_dirty(self) -> SpectrumFrame | None:
        with self._lock:
            if not self._dirty:
                return None
            self._dirty = False
            return self._frame


class SpectrumPlotter:
    def __init__(self, state: SharedState, refresh_ms: int, history_size: int):
        self.state = state
        self.refresh_ms = refresh_ms
        self.history_size = history_size
        self.annotation_artists = []
        self.last_frame: SpectrumFrame | None = None
        self.waterfall_buffer: np.ndarray | None = None
        self.waterfall_shape: tuple[int, float, float] | None = None

        self.fig = plt.figure(figsize=(15, 9))
        self.fig.canvas.manager.set_window_title("RTL-SDR Spectrum Scanner")

        gs = self.fig.add_gridspec(2, 2, width_ratios=[24, 1], height_ratios=[3, 2])
        self.ax = self.fig.add_subplot(gs[0, 0])
        self.ax_waterfall = self.fig.add_subplot(gs[1, 0], sharex=self.ax)
        self.cax = self.fig.add_subplot(gs[1, 1])
        self._ax_padding = self.fig.add_subplot(gs[0, 1])
        self._ax_padding.set_visible(False)

        (self.spectrum_line,) = self.ax.plot([], [], color="#1f77b4", linewidth=0.8, label="Spectrum")
        (self.noise_line,) = self.ax.plot([], [], color="#6c757d", linestyle="--", linewidth=1.0, label="Noise floor")
        (self.peak_marker,) = self.ax.plot([], [], marker="o", color="#111111", linestyle="None", markersize=5, label="Peak")

        self.info_box = self.ax.text(
            0.01,
            0.98,
            "Waiting for scan_data...",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="#bbbbbb", alpha=0.85),
        )
        self.coord_box = self.ax.text(
            0.99,
            0.02,
            "",
            transform=self.ax.transAxes,
            va="bottom",
            ha="right",
            fontsize=9,
            bbox=dict(boxstyle="round,pad=0.3", facecolor="#fff8dc", edgecolor="#d0b85a", alpha=0.85),
        )

        self.ax.set_title("RTL-SDR Spectrum")
        self.ax.set_xlabel("Frequency (MHz)")
        self.ax.set_ylabel("Power (dBFS)")
        self.ax.grid(True, which="major", linestyle="--", linewidth=0.6, alpha=0.35)
        self.ax.grid(True, which="minor", linestyle=":", linewidth=0.4, alpha=0.25)
        self.ax.minorticks_on()
        self.ax.legend(loc="lower left")

        self.waterfall = self.ax_waterfall.imshow(
            np.full((self.history_size, 2), np.nan),
            aspect="auto",
            interpolation="nearest",
            origin="upper",
            cmap="viridis",
        )
        self.colorbar = self.fig.colorbar(self.waterfall, cax=self.cax)
        self.colorbar.set_label("Power (dBFS)")
        self.ax_waterfall.set_title("Waterfall / Time-Frequency")
        self.ax_waterfall.set_xlabel("Frequency (MHz)")
        self.ax_waterfall.set_ylabel(f"Recent sweeps ({self.history_size})")
        self.ax_waterfall.set_ylim(self.history_size, 0)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)

    def run(self) -> None:
        timer = self.fig.canvas.new_timer(interval=self.refresh_ms)
        timer.add_callback(self._poll)
        timer.start()
        plt.tight_layout()
        plt.show()

    def _poll(self) -> None:
        frame = self.state.take_if_dirty()
        if frame is not None:
            self.draw(frame)

    def draw(self, frame: SpectrumFrame) -> None:
        if not frame.spectrum:
            return

        self.last_frame = frame
        data = np.asarray(frame.spectrum, dtype=np.float64)
        valid = np.isfinite(data) & (data > -180.0)
        if not np.any(valid):
            return

        start_mhz = frame.start_hz / 1e6
        end_mhz = frame.end_hz / 1e6
        freqs = np.linspace(start_mhz, end_mhz, data.size)
        valid_data = data[valid]

        self.spectrum_line.set_data(freqs, data)

        noise = float(np.median(valid_data))
        self.noise_line.set_data([start_mhz, end_mhz], [noise, noise])

        peak_idx = int(np.nanargmax(data))
        peak_freq = float(freqs[peak_idx])
        peak_db = float(data[peak_idx])
        self.peak_marker.set_data([peak_freq], [peak_db])

        y_bottom, y_top = self._power_axis_limits(valid_data)

        self.ax.set_xlim(start_mhz, end_mhz)
        self.ax.set_ylim(y_bottom, y_top)
        self.ax.set_title(f"RTL-SDR Spectrum  {start_mhz:.3f}-{end_mhz:.3f} MHz")

        rbw_hz = (frame.end_hz - frame.start_hz) / max(data.size - 1, 1)
        fm_count = sum(1 for det in frame.detections if isinstance(det, dict) and det.get("type") == "fm_spec")
        am_count = sum(1 for det in frame.detections if isinstance(det, dict) and det.get("type") == "am_spec")
        self.info_box.set_text(
            f"Range: {start_mhz:.3f}-{end_mhz:.3f} MHz\n"
            f"Bins: {data.size}  RBW: {format_hz(rbw_hz)}\n"
            f"Peak: {peak_db:.1f} dBFS @ {peak_freq:.4f} MHz\n"
            f"Noise floor: {noise:.1f} dBFS  FM labels: {fm_count}  AM labels: {am_count}\n"
            f"Updated: {frame.received_at:%H:%M:%S}"
        )

        self._clear_annotations()
        self._draw_detections(frame.detections, y_bottom, y_top)
        self._update_waterfall(data, valid, start_mhz, end_mhz, y_bottom, y_top)
        self._deduplicate_legend()
        self.fig.canvas.draw_idle()

    def _power_axis_limits(self, valid_data: np.ndarray) -> tuple[float, float]:
        data_min = float(np.min(valid_data))
        data_max = float(np.max(valid_data))
        span = max(data_max - data_min, 1.0)
        margin = max(span * 0.18, 6.0)
        y_bottom = np.floor((data_min - margin) / 5.0) * 5.0
        y_top = np.ceil((data_max + margin) / 5.0) * 5.0
        if y_top - y_bottom < 20.0:
            center = (y_top + y_bottom) * 0.5
            y_bottom = center - 10.0
            y_top = center + 10.0
        return float(y_bottom), float(y_top)

    def _update_waterfall(
        self,
        data: np.ndarray,
        valid: np.ndarray,
        start_mhz: float,
        end_mhz: float,
        y_bottom: float,
        y_top: float,
    ) -> None:
        new_shape_key = (data.size, start_mhz, end_mhz)

        row = data.copy()
        if not np.all(valid):
            fill = float(np.median(data[valid])) if np.any(valid) else y_bottom
            row[~valid] = fill

        if self.waterfall_shape != new_shape_key or self.waterfall_buffer is None:
            self.waterfall_shape = new_shape_key
            self.waterfall_buffer = np.full((self.history_size, data.size), y_bottom)
            self.waterfall_buffer[0, :] = row
        else:
            self.waterfall_buffer[1:, :] = self.waterfall_buffer[:-1, :]
            self.waterfall_buffer[0, :] = row

        wf = self.waterfall_buffer
        vmin = max(float(np.percentile(wf, 5)), y_bottom)
        vmax = min(float(np.percentile(wf, 99)), y_top)
        if vmax <= vmin:
            vmin = y_bottom
            vmax = y_top

        bin_width = (end_mhz - start_mhz) / max(data.size - 1, 1)
        half_bin = bin_width * 0.5
        self.waterfall.set_data(wf)
        self.waterfall.set_extent([start_mhz - half_bin, end_mhz + half_bin, self.history_size, 0])
        self.waterfall.set_clim(vmin, vmax)
        self.ax_waterfall.set_xlim(start_mhz, end_mhz)
        self.ax_waterfall.set_ylim(self.history_size, 0)
        self.ax_waterfall.set_ylabel(f"Recent sweeps ({self.history_size})")

    def _draw_detections(self, detections: list[dict], y_bottom: float, y_top: float) -> None:
        height = y_top - y_bottom
        if height <= 0:
            return

        for det in detections:
            if not isinstance(det, dict) or det.get("type") not in {"fm_spec", "am_spec"}:
                continue

            start_hz = det.get("start_freq")
            end_hz = det.get("end_freq")
            if start_hz is None or end_hz is None:
                continue

            start_mhz = float(start_hz) / 1e6
            end_mhz = float(end_hz) / 1e6
            if end_mhz <= start_mhz:
                continue

            is_am = det.get("type") == "am_spec"
            edge_color = "#ff7f0e" if is_am else "#d62728"
            signal_name = "AM" if is_am else "FM"
            legend_label = "AM detection" if is_am else "FM detection"

            rect = Rectangle(
                (start_mhz, y_bottom),
                end_mhz - start_mhz,
                height,
                fill=False,
                edgecolor=edge_color,
                linewidth=1.6,
                alpha=0.95,
                label=legend_label,
            )
            self.ax.add_patch(rect)
            self.annotation_artists.append(rect)

            center_mhz = float(det.get("cf", (float(start_hz) + float(end_hz)) * 0.5)) / 1e6
            bw_khz = float(det.get("bw", float(end_hz) - float(start_hz))) / 1e3
            snr_db = float(det.get("snr_db", 0.0))
            confidence = float(det.get("confidence", 0.0))
            flags = []
            if det.get("verified"):
                flags.append("verified")
            if not is_am and det.get("stereo"):
                flags.append("stereo")
            if not is_am and det.get("rds"):
                flags.append("RDS")
            suffix = f" ({', '.join(flags)})" if flags else ""
            if is_am:
                audio_snr = float(det.get("audio_snr_db", 0.0))
                depth = float(det.get("modulation_depth", 0.0))
                label = (
                    f"{signal_name} {center_mhz:.3f} MHz{suffix}\n"
                    f"BW {bw_khz:.0f} kHz  SNR {snr_db:.1f} dB  AM {depth:.2f}\n"
                    f"Audio {audio_snr:.1f} dB  C {confidence:.2f}"
                )
            else:
                label = (
                    f"{signal_name} {center_mhz:.3f} MHz{suffix}\n"
                    f"BW {bw_khz:.0f} kHz  SNR {snr_db:.1f} dB  C {confidence:.2f}"
                )
            text = self.ax.annotate(
                label,
                xy=(center_mhz, y_top - height * 0.08),
                xytext=(0, 0),
                textcoords="offset points",
                ha="center",
                va="top",
                fontsize=8,
                color=edge_color,
                bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor=edge_color, alpha=0.8),
            )
            self.annotation_artists.append(text)

    def _clear_annotations(self) -> None:
        for artist in self.annotation_artists:
            artist.remove()
        self.annotation_artists.clear()

    def _deduplicate_legend(self) -> None:
        handles, labels = self.ax.get_legend_handles_labels()
        unique = {}
        for handle, label in zip(handles, labels, strict=False):
            if label not in unique:
                unique[label] = handle
        self.ax.legend(unique.values(), unique.keys(), loc="lower left")

    def _on_mouse_move(self, event) -> None:
        if event.inaxes != self.ax or self.last_frame is None or event.xdata is None:
            self.coord_box.set_text("")
            self.fig.canvas.draw_idle()
            return

        data = np.asarray(self.last_frame.spectrum, dtype=np.float64)
        if data.size < 2:
            return

        start_mhz = self.last_frame.start_hz / 1e6
        end_mhz = self.last_frame.end_hz / 1e6
        idx = int(round((event.xdata - start_mhz) / max(end_mhz - start_mhz, 1e-12) * (data.size - 1)))
        if idx < 0 or idx >= data.size:
            self.coord_box.set_text("")
            self.fig.canvas.draw_idle()
            return

        freq_mhz = start_mhz + (end_mhz - start_mhz) * idx / (data.size - 1)
        self.coord_box.set_text(f"{freq_mhz:.5f} MHz\n{data[idx]:.2f} dBFS")
        self.fig.canvas.draw_idle()


class DataHandler(BaseHTTPRequestHandler):
    state: SharedState | None = None
    path: str = "/api/service"

    def do_POST(self) -> None:
        if self.path != self.__class__.path:
            self.send_error(404, "Not Found")
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0:
            self.send_error(400, "Empty Body")
            return

        try:
            payload = json.loads(self.rfile.read(content_length))
        except json.JSONDecodeError as exc:
            self.send_error(400, f"Invalid JSON: {exc}")
            return

        event = payload.get("event", "")
        data = payload.get("data", {})

        if event == "scan_data":
            self._handle_scan(data)
        elif event == "ADSB_DATA_LIST":
            self._handle_adsb(data)
        elif event == "scan_heartbeat":
            self._handle_heartbeat(data)
        elif event == "scan_events":
            self._handle_scan_events(data)
        else:
            logger.debug("Ignoring event: %s", event)

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"status":"ok"}')

    def _handle_scan(self, data: dict) -> None:
        if not isinstance(data, dict):
            logger.warning("scan_data.data is not an object")
            return

        spectrum = data.get("data", [])
        if not isinstance(spectrum, list) or not spectrum:
            logger.warning("scan_data contains no spectrum")
            return

        frame = SpectrumFrame(
            start_hz=float(data.get("start_freq", 0.0)),
            end_hz=float(data.get("end_freq", 0.0)),
            spectrum=spectrum,
            detections=data.get("result", []) if isinstance(data.get("result", []), list) else [],
            received_at=datetime.now(),
        )
        if self.__class__.state is not None:
            self.__class__.state.update(frame)

        logger.info(
            "scan_data %.3f-%.3f MHz bins=%d result=%d",
            frame.start_hz / 1e6,
            frame.end_hz / 1e6,
            len(frame.spectrum),
            len(frame.detections),
        )

    def _handle_adsb(self, data) -> None:
        if isinstance(data, list):
            logger.info("ADSB_DATA_LIST aircraft=%d", len(data))

    def _handle_heartbeat(self, data) -> None:
        dev_state = data.get("dev_state", 0) if isinstance(data, dict) else 0
        logger.debug(
            "Heartbeat: dev_state=%d running=%s scanning=%s",
            dev_state,
            bool(dev_state & 4),
            bool(dev_state & 2),
        )

    def _handle_scan_events(self, data) -> None:
        msg = data.get("msg", "") if isinstance(data, dict) else ""
        logger.warning("Device event: %s", msg)

    def log_message(self, fmt: str, *args) -> None:
        logger.debug("%s - %s", self.client_address[0], fmt % args)


def format_hz(value: float) -> str:
    if abs(value) >= 1e6:
        return f"{value / 1e6:.3f} MHz"
    if abs(value) >= 1e3:
        return f"{value / 1e3:.2f} kHz"
    return f"{value:.1f} Hz"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive and plot RTL_SDR_Scanner spectrum data.")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP listen host, default 127.0.0.1")
    parser.add_argument("--port", type=int, default=23568, help="HTTP listen port, default 23568")
    parser.add_argument("--path", default="/api/service", help="HTTP POST path, default /api/service")
    parser.add_argument("--refresh-ms", type=int, default=200, help="Plot refresh interval, default 200")
    parser.add_argument("--history", type=int, default=60, help="Waterfall history length in sweeps, default 60")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    state = SharedState()

    DataHandler.state = state
    DataHandler.path = args.path
    server = HTTPServer((args.host, args.port), DataHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    logger.info("Listening on http://%s:%d%s", args.host, args.port, args.path)
    try:
        SpectrumPlotter(state, args.refresh_ms, max(args.history, 1)).run()
    finally:
        server.shutdown()
        server.server_close()
        logger.info("Stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
