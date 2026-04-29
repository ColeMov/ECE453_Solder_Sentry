"""
ble_client.py — BLE client for ECE453 Solder Sentry.

Connects to the board (advertises as "ECE453"), enables notifications on the
NUS TX characteristic, and dispatches:

  * IR frames (binary protocol, 0xAA header + 8 × 0xBB chunks per frame)
  * Telemetry text lines emitted by the firmware via task_print_info:
        tof:<mm>     – TOF distance in millimetres
        fan:<pct>    – Fan duty cycle 0–100 %
        paused:0|1   – TOF safety state (1 = fan killed, too close)
        any other    – generic log line

Outbound:
  * `send_servo(pan_deg, tilt_deg)` writes "servo:<pan>,<tilt>\\n" to NUS RX.
"""

import asyncio
import struct
import re
import numpy as np
from bleak import BleakClient, BleakScanner

DEVICE_NAME       = "ECE453"
NUS_SERVICE_UUID  = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID       = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID       = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

FRAME_MARKER      = 0xAA
DATA_MARKER       = 0xBB
THERM_LSB         = 0.0625
PIXEL_LSB         = 0.25
CHUNKS_PER_FRAME  = 8
PIXELS_PER_CHUNK  = 8
TOTAL_PIXELS      = CHUNKS_PER_FRAME * PIXELS_PER_CHUNK


class IRFrameReceiver:
    """Reassembles multi-packet IR frames + parses telemetry lines."""

    _TELEM_RE = re.compile(rb"(tof|fan|paused|track):(-?\d+)")

    def __init__(self, on_frame, on_telemetry=None, on_log=None):
        self._on_frame = on_frame
        self._on_telemetry = on_telemetry or (lambda k, v: None)
        self._on_log = on_log or (lambda s: None)
        self._pending_seq = None
        self._thermistor_c = 0.0
        self._pixels = np.zeros(TOTAL_PIXELS, dtype=np.float32)
        self._chunks_received = set()
        self._line_buf = bytearray()

    def handle_notification(self, _handle, data: bytearray):
        if not data:
            return

        # Diagnostic: dump every notification packet so we can tell
        # whether anything is arriving from the board at all.
        try:
            print(f"[notif {len(data)}B] {bytes(data)!r}", flush=True)
        except Exception:
            pass

        marker = data[0]

        if marker == FRAME_MARKER:
            seq = data[1]
            raw_therm = struct.unpack_from(">h", data, 2)[0]
            self._pending_seq = seq
            self._thermistor_c = raw_therm * THERM_LSB
            self._pixels[:] = 0.0
            self._chunks_received = set()

        elif marker == DATA_MARKER and self._pending_seq is not None:
            chunk_idx = data[1]
            if chunk_idx >= CHUNKS_PER_FRAME or len(data) < 2 + PIXELS_PER_CHUNK * 2:
                return
            base = chunk_idx * PIXELS_PER_CHUNK
            for p in range(PIXELS_PER_CHUNK):
                raw = struct.unpack_from(">h", data, 2 + p * 2)[0]
                self._pixels[base + p] = raw * PIXEL_LSB
            self._chunks_received.add(chunk_idx)
            if len(self._chunks_received) == CHUNKS_PER_FRAME:
                grid = self._pixels.reshape(8, 8).copy()
                self._on_frame(self._thermistor_c, grid)
                self._pending_seq = None

        else:
            # Treat as ASCII log/telemetry; firmware splits messages with \n.
            self._line_buf.extend(data)
            while b"\n" in self._line_buf or b"\r" in self._line_buf:
                # Find earliest delimiter
                nl = self._line_buf.find(b"\n")
                cr = self._line_buf.find(b"\r")
                idx = min(x for x in (nl, cr) if x >= 0)
                line = bytes(self._line_buf[:idx])
                # Drop the delimiter (and a paired \r\n if present)
                drop = 1
                if idx + 1 < len(self._line_buf) and self._line_buf[idx:idx+2] in (b"\r\n", b"\n\r"):
                    drop = 2
                del self._line_buf[:idx + drop]
                self._dispatch_line(line)

    def _dispatch_line(self, line: bytes):
        if not line:
            return
        # Diagnostic: dump every assembled line so we can tell whether
        # BLE data is arriving at all, vs arriving but failing the
        # telemetry regex.
        try:
            print(f"[line] {line!r}", flush=True)
        except Exception:
            pass
        # Telemetry tokens may appear anywhere in the line (the print
        # framework prefixes things like "[Info] : ToF :"). Find them.
        m = self._TELEM_RE.search(line)
        if m:
            try:
                key = m.group(1).decode("ascii")
                value = int(m.group(2))
                print(f"[telemetry] {key}={value}", flush=True)
                self._on_telemetry(key, value)
                return
            except ValueError:
                pass
        try:
            self._on_log(line.decode("utf-8", errors="replace"))
        except Exception:
            pass


class SolderSentryClient:
    """Connection wrapper exposing send_servo() + state callbacks."""

    def __init__(self, on_frame, on_telemetry=None, on_log=None,
                 on_state=None):
        self._receiver = IRFrameReceiver(on_frame, on_telemetry, on_log)
        self._on_state = on_state or (lambda connected: None)
        self._client = None
        self._loop = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    async def send_text(self, text: str):
        if self._client is None or not self._client.is_connected:
            return
        try:
            await self._client.write_gatt_char(
                NUS_RX_UUID, text.encode("ascii"), response=False)
        except Exception:
            pass

    async def send_servo(self, pan_deg: int, tilt_deg: int):
        await self.send_text(f"servo:{int(pan_deg)},{int(tilt_deg)}\n")

    def send_servo_threadsafe(self, pan_deg: int, tilt_deg: int):
        if self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(
            self.send_servo(pan_deg, tilt_deg), self._loop)

    async def send_track(self, enable: bool):
        if self._client is None or not self._client.is_connected:
            return
        msg = f"track:{1 if enable else 0}\n".encode("ascii")
        try:
            await self._client.write_gatt_char(NUS_RX_UUID, msg, response=False)
        except Exception:
            pass

    def send_track_threadsafe(self, enable: bool):
        if self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(
            self.send_track(enable), self._loop)


async def _find_device():
    print(f"Scanning for '{DEVICE_NAME}'...")
    def _match(d, adv):
        if d.name and DEVICE_NAME.lower() in d.name.lower():
            return True
        if adv.local_name and DEVICE_NAME.lower() in adv.local_name.lower():
            return True
        if NUS_SERVICE_UUID in (adv.service_uuids or []):
            return True
        return False
    device = await BleakScanner.find_device_by_filter(_match, timeout=15.0)
    if device is None:
        raise RuntimeError(
            f"Device '{DEVICE_NAME}' not found. Toggle macOS Bluetooth and retry."
        )
    print(f"Found: {device.name!r} [{device.address}]")
    return device


async def run_ble(client_obj: "SolderSentryClient", stop_event: asyncio.Event):
    """Connect + stream until stop_event is set. Auto-reconnects on drop.

    Reports tri-state to client_obj._on_state:
        "scanning" — actively looking for the device
        "connected" — paired and streaming
        "disconnected" — idle, will retry shortly
    """
    client_obj._loop = asyncio.get_running_loop()
    while not stop_event.is_set():
        try:
            client_obj._on_state("scanning")
            device = await _find_device()
            async with BleakClient(device) as client:
                client_obj._client = client
                client_obj._connected = True
                client_obj._on_state("connected")
                print("Connected. Enabling notifications...")
                await client.start_notify(
                    NUS_TX_UUID, client_obj._receiver.handle_notification)
                while client.is_connected and not stop_event.is_set():
                    await asyncio.sleep(0.5)
                await client.stop_notify(NUS_TX_UUID)
        except Exception as e:
            print(f"BLE error: {e}")
        finally:
            client_obj._client = None
            client_obj._connected = False
            client_obj._on_state("disconnected")
        if not stop_event.is_set():
            print("Reconnecting in 2 s...")
            try:
                await asyncio.wait_for(stop_event.wait(), timeout=2.0)
            except asyncio.TimeoutError:
                pass
    print("BLE loop exited.")


# ── Backwards-compat shim for the old `run_ble(on_frame, stop_event)` API ──
async def run_ble_compat(on_frame, stop_event: asyncio.Event):
    client = SolderSentryClient(on_frame=on_frame)
    await run_ble(client, stop_event)
