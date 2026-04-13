# Solder Sentry — Desktop GUI

A Python desktop application that connects to the ECE453 Solder Sentry board over BLE and displays a live 8x8 thermal heatmap from the AMG8834 Grid-EYE IR sensor.

## Prerequisites

- **Python 3.10+** (tested on macOS)
- **Bluetooth** enabled on your machine
- The ECE453 board powered on and advertising as `"ECE453"` over BLE

## Setup

1. Open a terminal and navigate to this directory:

   ```bash
   cd desktop
   ```

2. Create a virtual environment (one-time):

   ```bash
   python3 -m venv .venv
   ```

3. Activate the virtual environment:

   ```bash
   source .venv/bin/activate
   ```

4. Install dependencies:

   ```bash
   pip install -r requirements.txt
   ```

   This installs:
   - **bleak** — cross-platform BLE client
   - **matplotlib** — heatmap visualization
   - **numpy** — array handling

## Running the GUI

With the virtual environment activated:

```bash
python main.py
```

The application will:

1. Scan for a BLE device named `"ECE453"` (timeout: 15 seconds)
2. Connect and enable BLE notifications on the NUS TX characteristic
3. Open a matplotlib window showing a live 8x8 thermal heatmap

### What you'll see

- **Heatmap** — color-coded temperature grid (inferno colormap, 20–80 °C range)
- **Pixel labels** — each cell shows its temperature in °C
- **Status bar** — frame count, thermistor reading, and hottest pixel location

Close the window to disconnect from the board and exit.

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `Device 'ECE453' not found` | Make sure the board is powered on and advertising. Toggle macOS Bluetooth off and back on, then retry. |
| GUI opens but stays at "Waiting for BLE connection..." | The board may be connected but not sending data yet. Check that the firmware is running the IR sensor + BLE tasks. |
| `ModuleNotFoundError` | Make sure the virtual environment is activated (`source .venv/bin/activate`) and dependencies are installed. |

## BLE Scan Utility

To verify your board is visible over BLE without launching the full GUI:

```bash
python scan.py
```

This prints all BLE devices found within 10 seconds, including their names, addresses, RSSI, and advertised UUIDs.
