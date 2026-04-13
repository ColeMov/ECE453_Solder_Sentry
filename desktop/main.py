"""
main.py — Solder Sentry desktop GUI.

Connects to the ECE453 board over BLE and displays a live 8x8 thermal
heatmap from the AMG8834 Grid-EYE IR sensor.

Usage:
    pip install -r requirements.txt
    python main.py
"""

import asyncio
import threading
import queue
import sys
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import Normalize

from ble_client import run_ble

# ── Layout ──────────────────────────────────────────────────────────────────
GRID_ROWS, GRID_COLS = 8, 8
TEMP_MIN_C = 20.0   # colormap low end (room temp)
TEMP_MAX_C = 80.0   # colormap high end (soldering range)

# Thread-safe queue: BLE thread → GUI thread
_frame_queue = queue.Queue(maxsize=4)


def _on_frame(thermistor_c: float, grid: np.ndarray):
    """Called from the BLE asyncio thread when a complete frame arrives."""
    try:
        _frame_queue.put_nowait((thermistor_c, grid))
    except queue.Full:
        pass  # drop oldest-equivalent: GUI is just slow


# ── GUI ─────────────────────────────────────────────────────────────────────

def build_gui(stop_event: asyncio.Event):
    fig, ax = plt.subplots(figsize=(6, 5.5))
    fig.patch.set_facecolor("#1e1e1e")
    ax.set_facecolor("#1e1e1e")

    dummy = np.full((GRID_ROWS, GRID_COLS), TEMP_MIN_C, dtype=np.float32)
    norm  = Normalize(vmin=TEMP_MIN_C, vmax=TEMP_MAX_C)
    im    = ax.imshow(dummy, cmap="inferno", norm=norm,
                      interpolation="nearest", origin="upper")

    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Temperature (°C)", color="white")
    cbar.ax.yaxis.set_tick_params(color="white")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="white")

    ax.set_title("Solder Sentry — IR Heatmap", color="white", fontsize=13)
    ax.set_xlabel("Column", color="white")
    ax.set_ylabel("Row", color="white")
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_edgecolor("white")

    # Pixel value overlay
    text_objs = [
        [ax.text(c, r, "", ha="center", va="center",
                 fontsize=7, color="white", fontweight="bold")
         for c in range(GRID_COLS)]
        for r in range(GRID_ROWS)
    ]

    status_text = ax.text(
        0.01, 0.99, "Waiting for BLE connection...",
        transform=ax.transAxes, color="#aaaaaa",
        fontsize=8, va="top", ha="left"
    )

    frame_count = [0]

    def update(_):
        updated = False
        thermistor_c = None
        grid = None

        # Drain the queue — keep only the newest frame
        while not _frame_queue.empty():
            try:
                thermistor_c, grid = _frame_queue.get_nowait()
                updated = True
            except queue.Empty:
                break

        if updated and grid is not None:
            frame_count[0] += 1
            im.set_data(grid)
            hottest = float(grid.max())
            hottest_pos = np.unravel_index(grid.argmax(), grid.shape)

            for r in range(GRID_ROWS):
                for c in range(GRID_COLS):
                    t = grid[r, c]
                    text_objs[r][c].set_text(f"{t:.1f}")
                    # White text on dark pixels, dark text on bright pixels
                    brightness = (t - TEMP_MIN_C) / (TEMP_MAX_C - TEMP_MIN_C)
                    text_objs[r][c].set_color("black" if brightness > 0.6 else "white")

            status_text.set_text(
                f"Frame #{frame_count[0]}  |  "
                f"Thermistor: {thermistor_c:.1f}°C  |  "
                f"Hottest: {hottest:.1f}°C @ ({hottest_pos[0]},{hottest_pos[1]})"
            )

        return [im, status_text] + [text_objs[r][c]
                                     for r in range(GRID_ROWS)
                                     for c in range(GRID_COLS)]

    ani = animation.FuncAnimation(
        fig, update, interval=100, blit=True, cache_frame_data=False
    )

    def on_close(_):
        stop_event.set()

    fig.canvas.mpl_connect("close_event", on_close)
    plt.tight_layout()
    plt.show()

    # Window closed — signal BLE thread
    stop_event.set()
    return ani  # keep reference alive


# ── BLE thread ───────────────────────────────────────────────────────────────

def ble_thread(stop_event: asyncio.Event, loop: asyncio.AbstractEventLoop):
    async def _run():
        try:
            await run_ble(_on_frame, stop_event)
        except Exception as exc:
            import traceback
            print(f"BLE error: {exc!r}", file=sys.stderr)
            traceback.print_exc()
            stop_event.set()

    asyncio.set_event_loop(loop)
    loop.run_until_complete(_run())


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    ble_loop = asyncio.new_event_loop()
    stop_event = asyncio.Event()

    t = threading.Thread(target=ble_thread, args=(stop_event, ble_loop), daemon=True)
    t.start()

    ani = build_gui(stop_event)  # blocks until window is closed  # noqa: F841

    ble_loop.call_soon_threadsafe(stop_event.set)
    t.join(timeout=3.0)


if __name__ == "__main__":
    main()
