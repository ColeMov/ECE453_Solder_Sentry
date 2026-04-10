"""
ble_client.py — BLE client for ECE453 Solder Sentry.

Connects to the board (advertises as "ECE453"), enables notifications on the
NUS TX characteristic, and reassembles the IR frame packets sent by the firmware.

Protocol (matches task_ble.c):
  Header packet (20 bytes):
    [0]    = 0xAA  (frame-start marker)
    [1]    = frame sequence number
    [2:3]  = thermistor as int16, big-endian, units = 0.0625 °C/LSB
    [4:19] = padding

  Data packets (8 total, 20 bytes each):
    [0]    = 0xBB  (data marker)
    [1]    = chunk index (0-7)
    [2:17] = 8 pixels as int16, big-endian, units = 0.25 °C/LSB
    [18:19]= padding
"""

import asyncio
import struct
import numpy as np
from bleak import BleakClient, BleakScanner

DEVICE_NAME       = "ECE453"
NUS_TX_UUID       = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID       = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

FRAME_MARKER      = 0xAA
DATA_MARKER       = 0xBB
THERM_LSB         = 0.0625   # °C per LSB for thermistor
PIXEL_LSB         = 0.25     # °C per LSB for pixels
CHUNKS_PER_FRAME  = 8
PIXELS_PER_CHUNK  = 8
TOTAL_PIXELS      = CHUNKS_PER_FRAME * PIXELS_PER_CHUNK  # 64


class IRFrameReceiver:
    """Reassembles multi-packet IR frames from BLE notifications."""

    def __init__(self, on_frame):
        """
        Args:
            on_frame: callback(thermistor_c: float, grid: np.ndarray[8,8])
        """
        self._on_frame = on_frame
        self._pending_seq = None
        self._thermistor_c = 0.0
        self._pixels = np.zeros(TOTAL_PIXELS, dtype=np.float32)
        self._chunks_received = set()

    def handle_notification(self, _handle, data: bytearray):
        print(f"  PKT len={len(data)} marker=0x{data[0]:02X}" if data else "  PKT empty")
        if len(data) < 2:
            return

        marker = data[0]

        if marker == FRAME_MARKER:
            # New frame — reset state
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


NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"

async def find_device():
    print(f"Scanning for '{DEVICE_NAME}' by service UUID...")
    # Primary: find by NUS service UUID in advertising packet
    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: NUS_SERVICE_UUID in (adv.service_uuids or []),
        timeout=15.0
    )
    if device is not None:
        print(f"Found by service UUID: {device.name!r} [{device.address}]")
        return device

    # Fallback: broad scan, match by name
    print("Service UUID not found, falling back to name scan...")
    devices = await BleakScanner.discover(timeout=10.0)
    for d in devices:
        print(f"  Saw: {d.name!r} [{d.address}]")
    match = next(
        (d for d in devices if d.name and DEVICE_NAME.lower() in d.name.lower()),
        None
    )
    if match is None:
        raise RuntimeError(
            f"Device '{DEVICE_NAME}' not found. Is it powered on and advertising?"
        )
    print(f"Connecting to: {match.name!r} [{match.address}]")
    return match


async def run_ble(on_frame, stop_event: asyncio.Event):
    """Connect to the board and stream IR frames until stop_event is set."""
    device = await find_device()
    receiver = IRFrameReceiver(on_frame)

    async with BleakClient(device) as client:
        print("Connected. Enabling notifications...")
        # List services/characteristics for debugging
        for svc in client.services:
            print(f"  SVC {svc.uuid}")
            for ch in svc.characteristics:
                print(f"    CHAR {ch.uuid} props={ch.properties}")
        await client.start_notify(NUS_TX_UUID, receiver.handle_notification)
        print("Streaming IR data. Close the window to stop.")
        await stop_event.wait()
        await client.stop_notify(NUS_TX_UUID)

    print("Disconnected.")
