"""
main.py — Solder Sentry desktop GUI.

Beautiful (Johnny Ive vibe) live monitor + controller for the ECE453 board:
  * IR thermal heatmap (8x8) embedded in a matplotlib canvas
  * TOF distance bar with red→orange→yellow→green safety zones
  * Connection indicator (filled dot, green when paired & alive)
  * "PAUSED" pill (lit when fan killed by TOF safety trip)
  * Fan speed bar (live from BLE telemetry)
  * Servo joystick — drag the dot to set pan/tilt; release snaps to centre

Runs the BLE client in a background asyncio thread and pumps state into the
GUI via thread-safe queues.

Install + run:
    cd desktop
    pip install -r requirements.txt
    python main.py
"""

import asyncio
import math
import queue
import threading
import tkinter as tk
from dataclasses import dataclass
from typing import Optional

import customtkinter as ctk
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.colors import Normalize
import numpy as np

from ble_client import SolderSentryClient, run_ble


# ── Visual language ─────────────────────────────────────────────────────────
ACCENT_BLUE   = "#0A84FF"
ACCENT_GREEN  = "#30D158"
ACCENT_RED    = "#FF453A"
ACCENT_AMBER  = "#FF9F0A"
ACCENT_YELLOW = "#FFD60A"
GREY_900      = "#0E0E10"
GREY_800      = "#1C1C1E"
GREY_700      = "#2C2C2E"
GREY_500      = "#636366"
GREY_300      = "#AEAEB2"
WHITE         = "#FFFFFF"

FONT_DISPLAY = ("SF Pro Display", 36, "bold")
FONT_TITLE   = ("SF Pro Display", 18, "bold")
FONT_LABEL   = ("SF Pro Text", 12)
FONT_MONO    = ("SF Mono", 11)

TEMP_MIN_C = 20.0
TEMP_MAX_C = 80.0
GRID_ROWS, GRID_COLS = 8, 8

TOF_RED_MM    = 100
TOF_AMBER_MM  = 200
TOF_YELLOW_MM = 400
TOF_MAX_BAR   = 1500

SERVO_PAN_CENTER  = 90
SERVO_TILT_CENTER = 90
SERVO_RANGE_DEG   = 90


# ── Shared state + queues ───────────────────────────────────────────────────
@dataclass
class AppState:
    connected: bool = False
    tof_mm: Optional[int] = None
    fan_pct: Optional[int] = None
    paused: bool = False


_state = AppState()
_frame_queue: "queue.Queue" = queue.Queue(maxsize=4)
_event_queue: "queue.Queue" = queue.Queue(maxsize=64)


def _push_event(kind: str, payload):
    try:
        _event_queue.put_nowait((kind, payload))
    except queue.Full:
        pass


# ── Widgets ─────────────────────────────────────────────────────────────────
class StatusDot(ctk.CTkFrame):
    """Pill-style status indicator with label."""

    def __init__(self, master, label: str, on_color: str, off_color: str):
        super().__init__(master, fg_color=GREY_800, corner_radius=14)
        self._on_color = on_color
        self._off_color = off_color

        self._canvas = tk.Canvas(self, width=14, height=14,
                                 bg=GREY_800, highlightthickness=0)
        self._canvas.grid(row=0, column=0, padx=(14, 8), pady=12)
        self._dot = self._canvas.create_oval(2, 2, 13, 13,
                                             fill=off_color, outline="")

        self._label = ctk.CTkLabel(self, text=label, font=FONT_LABEL,
                                   text_color=WHITE)
        self._label.grid(row=0, column=1, padx=(0, 16), pady=12)

    def set_state(self, on: bool, label: Optional[str] = None):
        self._canvas.itemconfig(self._dot,
                                fill=self._on_color if on else self._off_color)
        if label is not None:
            self._label.configure(text=label)


class ZoneBar(ctk.CTkFrame):
    """Horizontal bar with R/O/Y/G zones and a moving needle."""

    BAR_HEIGHT = 16
    NEEDLE_W   = 4

    def __init__(self, master, title: str, max_val: int, unit: str = "mm"):
        super().__init__(master, fg_color=GREY_800, corner_radius=18)
        self._max = max_val
        self._unit = unit

        ctk.CTkLabel(self, text=title, font=FONT_LABEL,
                     text_color=GREY_300).grid(
            row=0, column=0, sticky="w", padx=20, pady=(16, 0))

        self._value_lbl = ctk.CTkLabel(self, text="—", font=FONT_DISPLAY,
                                       text_color=WHITE)
        self._value_lbl.grid(row=1, column=0, sticky="w", padx=20, pady=(0, 4))

        self._canvas = tk.Canvas(self, height=self.BAR_HEIGHT + 6,
                                 bg=GREY_800, highlightthickness=0)
        self._canvas.grid(row=2, column=0, sticky="ew", padx=20, pady=(0, 18))
        self.grid_columnconfigure(0, weight=1)

        self._canvas.bind("<Configure>", self._redraw)
        self._current_val: Optional[int] = None

    def set_value(self, mm: Optional[int]):
        self._current_val = mm
        if mm is None:
            self._value_lbl.configure(text="—", text_color=GREY_500)
        else:
            self._value_lbl.configure(text=f"{mm} {self._unit}",
                                      text_color=self._zone_color(mm))
        self._redraw()

    @staticmethod
    def _zone_color(mm: int) -> str:
        if mm <= TOF_RED_MM:    return ACCENT_RED
        if mm <= TOF_AMBER_MM:  return ACCENT_AMBER
        if mm <= TOF_YELLOW_MM: return ACCENT_YELLOW
        return ACCENT_GREEN

    def _redraw(self, _event=None):
        c = self._canvas
        c.delete("all")
        w = max(c.winfo_width(), 100)
        h = self.BAR_HEIGHT
        y = 3
        zones = [
            (0,            TOF_RED_MM,    ACCENT_RED),
            (TOF_RED_MM,   TOF_AMBER_MM,  ACCENT_AMBER),
            (TOF_AMBER_MM, TOF_YELLOW_MM, ACCENT_YELLOW),
            (TOF_YELLOW_MM, self._max,    ACCENT_GREEN),
        ]
        for lo, hi, color in zones:
            x0 = int(w * lo / self._max)
            x1 = int(w * hi / self._max)
            c.create_rectangle(x0, y, x1, y + h, fill=color, outline="")
        # Round the bar ends
        c.create_oval(0, y, h, y + h, fill=ACCENT_RED, outline="")
        c.create_oval(w - h, y, w, y + h, fill=ACCENT_GREEN, outline="")
        # Needle
        if self._current_val is not None:
            mm = max(0, min(self._max, self._current_val))
            x = int(w * mm / self._max)
            c.create_rectangle(max(0, x - self.NEEDLE_W // 2),
                               y - 2,
                               min(w, x + self.NEEDLE_W // 2),
                               y + h + 2,
                               fill=WHITE, outline="")


class FanBar(ctk.CTkFrame):
    """Single-fill progress bar for fan duty %."""

    BAR_HEIGHT = 12

    def __init__(self, master, title: str = "Fan"):
        super().__init__(master, fg_color=GREY_800, corner_radius=18)

        ctk.CTkLabel(self, text=title, font=FONT_LABEL,
                     text_color=GREY_300).grid(
            row=0, column=0, sticky="w", padx=20, pady=(16, 0))

        self._value_lbl = ctk.CTkLabel(self, text="—", font=FONT_DISPLAY,
                                       text_color=WHITE)
        self._value_lbl.grid(row=1, column=0, sticky="w", padx=20, pady=(0, 4))

        self._canvas = tk.Canvas(self, height=self.BAR_HEIGHT + 6,
                                 bg=GREY_800, highlightthickness=0)
        self._canvas.grid(row=2, column=0, sticky="ew", padx=20, pady=(0, 18))
        self.grid_columnconfigure(0, weight=1)
        self._canvas.bind("<Configure>", self._redraw)
        self._pct: Optional[int] = None

    def set_pct(self, pct: Optional[int]):
        self._pct = pct
        if pct is None:
            self._value_lbl.configure(text="—", text_color=GREY_500)
        else:
            self._value_lbl.configure(text=f"{pct}%", text_color=ACCENT_BLUE)
        self._redraw()

    def _redraw(self, _event=None):
        c = self._canvas
        c.delete("all")
        w = max(c.winfo_width(), 100)
        h = self.BAR_HEIGHT
        y = 3
        c.create_rectangle(0, y, w, y + h, fill=GREY_700, outline="")
        c.create_oval(0, y, h, y + h, fill=GREY_700, outline="")
        c.create_oval(w - h, y, w, y + h, fill=GREY_700, outline="")
        if self._pct is not None and self._pct > 0:
            fw = int(w * (self._pct / 100.0))
            c.create_rectangle(0, y, fw, y + h, fill=ACCENT_BLUE, outline="")
            c.create_oval(0, y, h, y + h, fill=ACCENT_BLUE, outline="")
            if fw >= h:
                c.create_oval(fw - h, y, fw, y + h, fill=ACCENT_BLUE,
                              outline="")


class Joystick(ctk.CTkFrame):
    """Touch-style 2D joystick. Drag the dot, on release sends servo cmd."""

    SIZE = 200
    DOT_R = 18

    def __init__(self, master, on_change, on_grab=None, on_release=None):
        super().__init__(master, fg_color=GREY_800, corner_radius=24)
        self._on_change = on_change
        self._grab_cb = on_grab or (lambda: None)
        self._release_cb = on_release or (lambda: None)

        ctk.CTkLabel(self, text="Servo", font=FONT_LABEL,
                     text_color=GREY_300).grid(
            row=0, column=0, sticky="w", padx=20, pady=(16, 4))

        self._readout = ctk.CTkLabel(self, text="pan 90°  tilt 90°",
                                     font=FONT_MONO, text_color=WHITE)
        self._readout.grid(row=1, column=0, sticky="w", padx=20, pady=(0, 8))

        self._canvas = tk.Canvas(self, width=self.SIZE, height=self.SIZE,
                                 bg=GREY_800, highlightthickness=0)
        self._canvas.grid(row=2, column=0, padx=20, pady=(0, 20))
        self._canvas.bind("<Button-1>", self._on_press)
        self._canvas.bind("<B1-Motion>", self._on_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_release)

        self._cx = self.SIZE / 2
        self._cy = self.SIZE / 2
        self._draw_pad()

    def _draw_pad(self):
        c = self._canvas
        c.delete("all")
        s = self.SIZE
        c.create_oval(8, 8, s - 8, s - 8, outline=GREY_700, width=2)
        c.create_line(s / 2, 24, s / 2, s - 24, fill=GREY_700)
        c.create_line(24, s / 2, s - 24, s / 2, fill=GREY_700)
        c.create_oval(s / 2 - 4, s / 2 - 4, s / 2 + 4, s / 2 + 4,
                      fill=GREY_500, outline="")
        r = self.DOT_R
        self._thumb = c.create_oval(self._cx - r, self._cy - r,
                                    self._cx + r, self._cy + r,
                                    fill=ACCENT_BLUE, outline="")

    def _on_press(self, e):
        self._grab_cb()
        self._move_to(e.x, e.y)

    def _on_drag(self, e):
        self._move_to(e.x, e.y)

    def _on_release(self, _e):
        self._cx = self.SIZE / 2
        self._cy = self.SIZE / 2
        self._draw_pad()
        # Don't send a center-position servo cmd — that races track:1 on
        # the board (the queued servo cmd hits the auto-disable path
        # AFTER track:1 enabled tracking, flipping it back off). Just
        # update the readout and hand control back to the tracker via
        # the release callback.
        self._readout.configure(
            text=f"pan {SERVO_PAN_CENTER}°  tilt {SERVO_TILT_CENTER}°")
        self._release_cb()

    def _move_to(self, x, y):
        s = self.SIZE
        cx, cy = s / 2, s / 2
        dx, dy = x - cx, y - cy
        max_r = s / 2 - 12
        d = math.hypot(dx, dy)
        if d > max_r:
            scale = max_r / d
            dx *= scale
            dy *= scale
        self._cx = cx + dx
        self._cy = cy + dy
        self._draw_pad()

        nx = dx / max_r
        ny = -dy / max_r
        pan = max(0, min(180,
                         int(round(SERVO_PAN_CENTER + nx * SERVO_RANGE_DEG))))
        tilt = max(0, min(180,
                          int(round(SERVO_TILT_CENTER + ny * SERVO_RANGE_DEG))))
        self._readout.configure(text=f"pan {pan}°  tilt {tilt}°")
        self._on_change(pan, tilt)


# ── Main app ────────────────────────────────────────────────────────────────
class SolderSentryApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Solder Sentry")
        self.geometry("1180x720")
        self.configure(fg_color=GREY_900)

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self._stop_event_async: Optional[asyncio.Event] = None
        self._client = SolderSentryClient(
            on_frame=self._on_frame_thread,
            on_telemetry=self._on_telemetry_thread,
            on_log=self._on_log_thread,
            on_state=self._on_state_thread,
        )
        self._ble_thread = threading.Thread(target=self._ble_main, daemon=True)
        self._ble_thread.start()

        self._build_layout()
        self.after(50, self._pump_events)
        self.after(50, self._pump_frames)

    def _build_layout(self):
        self.grid_columnconfigure(0, weight=2)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # Header
        header = ctk.CTkFrame(self, fg_color=GREY_900)
        header.grid(row=0, column=0, columnspan=2, sticky="ew",
                    padx=24, pady=(20, 12))

        ctk.CTkLabel(header, text="Solder Sentry",
                     font=("SF Pro Display", 28, "bold"),
                     text_color=WHITE).pack(side="left")

        self._connection_dot = StatusDot(header, "Disconnected",
                                         on_color=ACCENT_GREEN,
                                         off_color=GREY_500)
        self._connection_dot.pack(side="right", padx=(8, 0))

        self._paused_dot = StatusDot(header, "Active",
                                     on_color=ACCENT_RED,
                                     off_color=GREY_500)
        self._paused_dot.pack(side="right", padx=(8, 0))

        # Tracking light: lit when board auto-tracks hottest IR pixel,
        # dim while user is driving the joystick manually.
        self._tracking_dot = StatusDot(header, "Manual",
                                       on_color=ACCENT_GREEN,
                                       off_color=GREY_500)
        self._tracking_dot.pack(side="right", padx=(8, 0))

        # Left: thermal heatmap
        left = ctk.CTkFrame(self, fg_color=GREY_800, corner_radius=20)
        left.grid(row=1, column=0, sticky="nsew", padx=(24, 12), pady=(0, 24))
        left.grid_rowconfigure(1, weight=1)
        left.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(left, text="Thermal", font=FONT_TITLE,
                     text_color=WHITE).grid(
            row=0, column=0, sticky="w", padx=20, pady=(18, 4))

        self._fig, self._ax = plt.subplots(figsize=(6, 5.5))
        self._fig.patch.set_facecolor(GREY_800)
        self._ax.set_facecolor(GREY_800)
        dummy = np.full((GRID_ROWS, GRID_COLS), TEMP_MIN_C, dtype=np.float32)
        self._im = self._ax.imshow(dummy, cmap="inferno",
                                   norm=Normalize(vmin=TEMP_MIN_C,
                                                  vmax=TEMP_MAX_C),
                                   interpolation="nearest", origin="upper")
        cbar = self._fig.colorbar(self._im, ax=self._ax, fraction=0.046,
                                  pad=0.04)
        cbar.set_label("°C", color=WHITE)
        cbar.ax.yaxis.set_tick_params(color=WHITE)
        plt.setp(cbar.ax.yaxis.get_ticklabels(), color=WHITE)
        self._ax.tick_params(colors=WHITE)
        for spine in self._ax.spines.values():
            spine.set_color(GREY_700)

        self._therm_text = self._ax.text(
            0.5, 1.05, "—", color=WHITE, ha="center",
            transform=self._ax.transAxes, fontsize=12)

        canvas = FigureCanvasTkAgg(self._fig, master=left)
        canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew",
                                    padx=14, pady=(0, 18))
        self._canvas = canvas

        # Right column
        right = ctk.CTkFrame(self, fg_color=GREY_900)
        right.grid(row=1, column=1, sticky="nsew", padx=(12, 24), pady=(0, 24))
        right.grid_columnconfigure(0, weight=1)

        self._tof_bar = ZoneBar(right, "Distance", max_val=TOF_MAX_BAR)
        self._tof_bar.grid(row=0, column=0, sticky="ew", pady=(0, 12))

        self._fan_bar = FanBar(right, "Fan")
        self._fan_bar.grid(row=1, column=0, sticky="ew", pady=(0, 12))

        self._joystick = Joystick(right,
                                  on_change=self._on_servo_change,
                                  on_grab=self._on_joystick_grab,
                                  on_release=self._on_joystick_release)
        self._joystick.grid(row=2, column=0, sticky="ew", pady=(0, 12))

    # ── BLE thread callbacks ──
    def _on_frame_thread(self, thermistor_c: float, grid: np.ndarray):
        try:
            _frame_queue.put_nowait((thermistor_c, grid))
        except queue.Full:
            pass

    def _on_telemetry_thread(self, key: str, value: int):
        _push_event("telemetry", (key, value))

    def _on_log_thread(self, line: str):
        _push_event("log", line)

    def _on_state_thread(self, connected: bool):
        _push_event("state", connected)

    # ── GUI thread ──
    def _on_servo_change(self, pan_deg: int, tilt_deg: int):
        self._client.send_servo_threadsafe(pan_deg, tilt_deg)

    def _on_joystick_grab(self):
        # User taking manual control — kill auto-tracking until release.
        self._client.send_track_threadsafe(False)

    def _on_joystick_release(self):
        # Joystick snapped back to center — hand control back to tracker.
        self._client.send_track_threadsafe(True)

    def _pump_frames(self):
        try:
            while True:
                therm_c, grid = _frame_queue.get_nowait()
                self._im.set_data(grid)
                self._therm_text.set_text(f"thermistor {therm_c:.1f} °C")
                self._canvas.draw_idle()
        except queue.Empty:
            pass
        self.after(40, self._pump_frames)

    def _pump_events(self):
        try:
            while True:
                kind, payload = _event_queue.get_nowait()
                if kind == "telemetry":
                    key, value = payload
                    if key == "tof":
                        _state.tof_mm = value
                        self._tof_bar.set_value(value)
                    elif key == "fan":
                        _state.fan_pct = value
                        self._fan_bar.set_pct(value)
                    elif key == "paused":
                        _state.paused = bool(value)
                        self._paused_dot.set_state(
                            _state.paused,
                            "PAUSED — too close" if _state.paused else "Active")
                    elif key == "track":
                        tracking = bool(value)
                        self._tracking_dot.set_state(
                            tracking,
                            "Auto-track" if tracking else "Manual")
                elif kind == "state":
                    _state.connected = bool(payload)
                    self._connection_dot.set_state(
                        _state.connected,
                        "Connected" if _state.connected else "Disconnected")
        except queue.Empty:
            pass
        self.after(50, self._pump_events)

    # ── BLE thread runner ──
    def _ble_main(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._stop_event_async = asyncio.Event()
        try:
            loop.run_until_complete(run_ble(self._client,
                                            self._stop_event_async))
        finally:
            loop.close()

    def destroy(self):
        if self._stop_event_async is not None:
            try:
                self._stop_event_async._loop.call_soon_threadsafe(
                    self._stop_event_async.set)
            except Exception:
                pass
        super().destroy()


def main():
    app = SolderSentryApp()
    app.mainloop()


if __name__ == "__main__":
    main()
