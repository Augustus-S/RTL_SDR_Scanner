#!/usr/bin/env python3
"""
RTL-SDR 频谱图接收脚本
监听 127.0.0.1:235678，绘制实时频谱图
"""

import json
import logging
import sys
import threading
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("receiver_plot")


class SpectrumPlotter:

    def __init__(self):
        self.fig, (self.ax_spectrum, self.ax_info) = plt.subplots(
            2, 1,
            figsize=(16, 8),
            gridspec_kw={"height_ratios": [5, 1]},
        )
        self.fig.canvas.manager.set_window_title("RTL-SDR Spectrum Scanner")

        self.spectrum_line = None
        self.annotation = None
        self.latest_data = None
        self.latest_start = None
        self.latest_end = None
        self.need_redraw = False

        self._setup_spectrum_axes()
        self._setup_info_panel()
        self._setup_cursor_tracking()

        plt.tight_layout()
        self.fig.canvas.mpl_connect("close_event", self._on_close)

    def _setup_spectrum_axes(self):
        ax = self.ax_spectrum
        ax.set_xlabel("Frequency (MHz)")
        ax.set_ylabel("Power (dBFS)")
        ax.set_title("RTL-SDR Spectrum")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.set_ylim(-120, 0)
        ax.set_xlim(0, 1)

        self.spectrum_line, = ax.plot([], [], "b-", linewidth=0.3, alpha=0.85)
        self.annotation = ax.annotate(
            "", xy=(0, 0), xytext=(15, 15), textcoords="offset points",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="lightyellow", alpha=0.9),
            fontsize=9,
            arrowprops=dict(arrowstyle="->", color="gray"),
        )
        self.annotation.set_visible(False)

    def _setup_info_panel(self):
        ax = self.ax_info
        ax.axis("off")
        self.info_text = ax.text(
            0.5, 0.5, "Waiting for scan data...",
            transform=ax.transAxes,
            fontsize=11,
            fontfamily="monospace",
            ha="center",
            va="center",
            bbox=dict(boxstyle="round,pad=0.6", facecolor="#f0f0f0", alpha=0.8),
        )

    def _setup_cursor_tracking(self):
        self.cursor = Cursor(
            self.ax_spectrum,
            useblit=True,
            color="red",
            linewidth=0.8,
            linestyle="--",
        )

        self.fig.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)

    def _on_mouse_move(self, event):
        if event.inaxes != self.ax_spectrum:
            self.annotation.set_visible(False)
            self.fig.canvas.draw_idle()
            return

        if self.latest_data is None:
            return

        x = event.xdata
        if x is None:
            return

        start = self.latest_start
        end = self.latest_end
        spectrum = self.latest_data
        if start is None or end is None or spectrum is None or len(spectrum) == 0:
            return

        n = len(spectrum)
        if n < 2:
            return

        bin_width = (end - start) / n
        idx = int((x - start) / bin_width)
        if idx < 0 or idx >= n:
            self.annotation.set_visible(False)
            self.fig.canvas.draw_idle()
            return

        freq = start + idx * bin_width
        power = spectrum[idx]

        self.annotation.xy = (x, power)
        self.annotation.set_text(f"  {freq:.4f} MHz\n  {power:.2f} dBFS")
        self.annotation.set_visible(True)
        self.fig.canvas.draw_idle()

    def update(self, start_freq, end_freq, spectrum):
        if not spectrum or len(spectrum) == 0:
            return

        self.latest_data = np.array(spectrum, dtype=np.float64)
        self.latest_start = float(start_freq)
        self.latest_end = float(end_freq)
        self.need_redraw = True

    def _draw(self):
        if not self.need_redraw or self.latest_data is None:
            return

        data = self.latest_data
        n = len(data)
        freqs = np.linspace(self.latest_start, self.latest_end, n)

        self.spectrum_line.set_data(freqs, data)
        self.ax_spectrum.set_xlim(self.latest_start, self.latest_end)

        data_min = np.min(data)
        data_max = np.max(data)
        margin = max((data_max - data_min) * 0.1, 2.0)
        y_bottom = data_min - margin
        y_top = data_max + margin
        self.ax_spectrum.set_ylim(y_bottom, y_top)

        self.ax_spectrum.set_title(
            f"RTL-SDR Spectrum  ({self.latest_start:.3f} – {self.latest_end:.3f} MHz)"
        )

        self.ax_spectrum.relim()

        peak_idx = np.argmax(data)
        peak_freq = freqs[peak_idx]
        peak_val = data[peak_idx]
        noise_floor = np.median(data)
        rbw_hz = (self.latest_end - self.latest_start) / n * 1e6

        if rbw_hz >= 1000:
            rbw_str = f"{rbw_hz / 1000:.2f} kHz"
        else:
            rbw_str = f"{rbw_hz:.1f} Hz"

        info = (
            f"Range: {self.latest_start:.3f} – {self.latest_end:.3f} MHz  |  "
            f"Bins: {n}  |  "
            f"RBW: {rbw_str}  |  "
            f"Peak: {peak_val:.1f} dBFS @ {peak_freq:.4f} MHz  |  "
            f"Noise: {noise_floor:.1f} dBFS"
        )
        self.info_text.set_text(info)
        self.annotation.set_visible(False)
        self.fig.canvas.draw_idle()
        self.need_redraw = False

    def _on_close(self, event):
        self._running = False

    def run(self):
        self._running = True
        timer = self.fig.canvas.new_timer(interval=200)
        timer.add_callback(self._poll)
        timer.start()
        plt.show()

    def _poll(self):
        if self.need_redraw:
            self._draw()


class DataHandler(BaseHTTPRequestHandler):

    def do_POST(self):
        if self.path != "/api/service":
            self.send_error(404, "Not Found")
            return

        content_length = int(self.headers.get("Content-Length", 0))
        if content_length == 0:
            self.send_error(400, "Empty Body")
            return

        body = self.rfile.read(content_length)
        try:
            payload = json.loads(body)
        except json.JSONDecodeError as e:
            self.send_error(400, f"Invalid JSON: {e}")
            return

        event = payload.get("event", "")
        data = payload.get("data", {})

        if event == "ADSB_DATA_LIST":
            self._handle_adsb(data)
        elif event == "scan_data":
            self._handle_scan(data)
        elif event == "scan_heartbeat":
            self._handle_heartbeat(data)
        elif event == "scan_events":
            self._handle_scan_events(data)
        else:
            logger.warning("Unknown event: %s", event)

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"status":"ok"}')

    def _handle_adsb(self, data):
        if not isinstance(data, list):
            logger.warning("ADSB_DATA_LIST data is not a list")
            return

        logger.info("=== ADS-B Data (%d aircraft) ===", len(data))
        for ac in data:
            logger.info(
                "  %s | %-8s | alt=%7.1fm | spd=%6.1fm/s | lat=%8.4f lon=%9.4f | yaw=%5.1f | msg=%d | %s",
                ac.get("uuid", "?"),
                ac.get("flight", "-"),
                ac.get("alt", 0),
                ac.get("horizontal_speed", 0),
                ac.get("lat", 0),
                ac.get("lng", 0),
                ac.get("yaw", 0),
                ac.get("seq", 0),
                "ground" if ac.get("on_ground") else "air",
            )

    def _handle_scan(self, data):
        if not isinstance(data, dict):
            logger.warning("scan_data data is not a dict")
            return

        start_freq_hz = data.get("start_freq", 0)
        end_freq_hz = data.get("end_freq", 0)
        max_val = data.get("max_value", 0)
        min_val = data.get("min_value", 0)
        spectrum = data.get("data", [])

        if not spectrum:
            logger.warning("scan_data contains empty spectrum")
            return

        start_freq = start_freq_hz / 1e6
        end_freq = end_freq_hz / 1e6

        logger.info(
            "=== Scan Data ===  %.3f - %.3f MHz | bins=%d | max=%.1f dBFS | min=%.1f dBFS",
            start_freq,
            end_freq,
            len(spectrum),
            max_val,
            min_val,
        )

        if G_PLOTTER is not None:
            G_PLOTTER.update(start_freq, end_freq, spectrum)

    def _handle_heartbeat(self, data):
        dev_state = data.get("dev_state", 0)
        is_running = bool(dev_state & 4)
        is_scanning = bool(dev_state & 2)
        logger.debug("Heartbeat: dev_state=%d (running=%s, scanning=%s)", dev_state, is_running, is_scanning)

    def _handle_scan_events(self, data):
        msg = data.get("msg", "")
        logger.warning("Device event: %s", msg)

    def log_message(self, format, *args):
        logger.debug("%s - %s", self.client_address[0], format % args)


G_PLOTTER = None
host = "127.0.0.1"
port = 23568


def main():
    global G_PLOTTER

    plotter = SpectrumPlotter()
    G_PLOTTER = plotter

    server = HTTPServer((host, port), DataHandler)
    server.timeout = 0.5

    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    logger.info("Spectrum plot window opened. Use toolbar to zoom/pan, hover for precise reading.")
    logger.info("Press Ctrl+C to stop")

    try:
        plotter.run()
    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        server.shutdown()
        server.server_close()
        logger.info("Service stopped")


if __name__ == "__main__":
    main()
