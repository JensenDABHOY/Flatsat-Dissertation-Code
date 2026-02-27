# FlatSat Dissertation Code
Operational FlatSat control software, downlink logging tools, and analysis notebooks used for the dissertation project.
Document version: 2026-02-27 (UTC)

## What This Repository Contains
- `flatsat_ops.py` (Raspberry Pi): main ops loop, hardware polling, uplink command handler, and compact downlink record generation.
- `ground_station_16.py` (laptop): TCP client + curses terminal + stateful CSV logger.
- `spectral_proc.c` (Raspberry Pi): C executable for spectral line detection/fitting; called from `flatsat_ops.py`.
- `flatsat_log_analysis.ipynb`: telemetry/event analysis notebook for the downlinked CSV sessions.
- `spectrometer_characterization_from_tiffs.ipynb`: full spectrometer characterization workflow from TIFF stacks (dark subtraction, spot metrics, UV calibration, and resolution summary).
- `spectral_res calc.ipynb`: small scratch notebook for quick spectral-resolution calculations.

## High-Level System Flow
1. Ground station sends one-line uplink commands over TCP to an XBee bridge.
2. Raspberry Pi receives uplinks over XBee UART, samples subsystems, and emits compact ASCII downlink records.
3. Ground station parses those records into a single stateful CSV (`flatsat_session_YYYYMMDD_HHMMSS.csv`).
4. Notebooks analyze logged CSV sessions or characterization TIFF data.

## Raspberry Pi Runtime (`flatsat_ops.py`)
### Subsystems and behavior
- IMU (BNO055): periodic telemetry (`TEL`) with Euler angles and accelerometer values.
- Sun sensor (via Arduino): periodic/commanded `SUN` records.
- Camera: captures JPEG via `rpicam-still` or `libcamera-still`, then downlinks top-3 brightest pixels (`CAM`).
- Motor control (via Arduino): manual, timed, SAFE stop, and AUTO pitch-threshold logic.
- Magnetorquer: PWM duty command plus optional INA219 current/voltage sensing (`MTQ`).
- Spectrum processing: `SPEC` command generates a synthetic frame, runs `spectral_proc`, and downlinks fit summary + up to 8 lines.

### Timing defaults (hard-coded)
- Telemetry: every `0.5 s`
- Sun sensor: every `5 s`
- Camera: every `30 s`

### Local output
- Output directory: `~/flatsat_tests`
- Stores captured camera images and synthetic spectrum frame binaries.

## Ground Station (`ground_station_16.py`)
### Features
- Curses terminal that stays interactive while downlink data streams.
- Automatic reconnect if TCP link drops.
- Stateful CSV logging with duplicate suppression (important for `DLCSV` history replay).
- Progress/ETA status during history downloads.

### CSV behavior
- Every structured record writes a full "latest-state" row.
- `event_type` identifies which subsystem triggered the row (`TEL`, `SUN`, `CAM`, `MTQ`, `MOT`, `SPEC`).
- Includes spectrum summary and `spec_line<i>_*` columns for up to 8 fitted lines.

## Spectrum Processor (`spectral_proc.c`)
### Build
```bash
gcc -O3 -std=c11 spectral_proc.c -lm -o spectral_proc
```
If needed for `clock_gettime`:
```bash
gcc -O3 -std=c11 -D_POSIX_C_SOURCE=200809L spectral_proc.c -lm -o spectral_proc
```

### Processing approach
- Sums 2D frame to 1D spectrum.
- Fits continuum with iterative sigma-clipped cubic polynomial.
- Detects peaks above SNR threshold.
- Fits Moffat profiles per line (centroid, FWHM, peak, flux, SNR).
- Reports packet-size estimate for line downlink budgeting.
- Optional JSON output (`--json`) used by `flatsat_ops.py`.

## Installation Prerequisites
### Raspberry Pi
Python packages:
- `pyserial`
- `numpy`
- `Pillow`
- `adafruit-circuitpython-bno055`
- `board`, `busio`

Optional magnetorquer sensing/control:
- `gpiozero`
- `adafruit-ina219`

System tools:
- `gcc`
- `rpicam-still` or `libcamera-still`

### Laptop / analysis machine
- Python 3.x
- `windows-curses` (for Windows terminal UI)
- `pandas`, `matplotlib`, `numpy` (CSV analysis notebook)
- `scipy`, `Pillow` (characterization notebook)

Optional for automatic UV source-prior extraction in characterization notebook:
- `requests`
- `PyMuPDF` (`fitz`)

## Running
### Raspberry Pi
```bash
export ARDUINO_PORT=/dev/ttyACM0
export XBEE_PORT=/dev/serial0
export SPECTRAL_BIN=./spectral_proc
python3 flatsat_ops.py
```
Stop locally with `q` + Enter.

### Laptop
```bash
python3 ground_station_16.py --host 192.168.10.1 --port 9750 --logdir C:\Windows\System32\logs
```

## Environment Variables (`flatsat_ops.py`)
- `ARDUINO_PORT` (default `/dev/ttyACM0`)
- `XBEE_PORT` (default `/dev/serial0`)
- `SUN_THRESHOLD` (default `15`)
- `PITCH_ON_DEG` (default `5.0`)
- `PITCH_FAST_DEG` (default `10.0`)
- `SPECTRAL_BIN` (default `./spectral_proc`)
- `SPEC_ROWS`, `SPEC_COLS` (default `64`, `2048`)
- `SPEC_WL_MIN`, `SPEC_WL_MAX` (default `300.0`, `330.0`)
- `MTQ_GPIO` (optional PWM pin)
- `MTQ_SENSOR=ina219` (optional sensor enable)
- `MTQ_INA219_ADDR` (default `0x40`)
- `MTQ_TURNS`, `MTQ_AREA_M2` (optional dipole estimate parameters)

## Uplink Commands
Commands are case-insensitive.
- `HELP` or `?`
- `STATUS`
- `SUN`
- `CAM`
- `SPEC`
- `SAFE`
- `AUTO`
- `MOTOR 0|1|2`
- `MOTOR_TIMED <0|1|2> <seconds>`
- `MTQ <duty 0..1>`
- `DLCSV [ALL|seconds]`
- `XOFF` / `XON`

## Downlink Record Types
Structured records written to CSV:
- `TEL,t_ms,...`
- `SUN,t_ms,...`
- `CAM,t_ms,...`
- `MTQ,t_ms,...`
- `MOT,t_ms,...`
- `SPEC,t_ms,...`

Informational lines (shown live, not CSV rows):
- `ACK ...`, `ERR ...`, `INFO ...`, `STATUS ...`
- `HIST_BEGIN,...`, `HIST_PROGRESS,...`, `HIST_END`

## Notebook Notes
### `flatsat_log_analysis.ipynb`
- Auto-loads the newest CSV in `C:\Windows\System32\logs` (configurable in notebook).
- Plots event rates, IMU/accel data, motor behavior, sun sensor trends, camera bright-pixel behavior, and spectral summary stats.

### `spectrometer_characterization_from_tiffs.ipynb`
- Expects TIFF datasets under `C:\Work\Dissertation\Data` in subfolders:
  - `no laser`
  - `laser`
  - `IR source better` (fallback: `IR source`)
  - `UV`
- Produces dark-subtracted stacks, histogram/SNR diagnostics, row-aligned 1D profiles, IR spot-size metrics with bootstrap uncertainty, UV wavelength calibration, and pixel-limited resolving-power summary.
- If PDF digitization dependencies are unavailable, UV source prior automatically falls back to embedded anchor points.

### `spectral_res calc.ipynb`
- Lightweight manual calculation notebook used for quick checks.
- Not a production processing path.

## Troubleshooting
- No TCP connection: verify XBee bridge host/port.
- No IMU data: verify BNO055 wiring and Adafruit libraries.
- Camera failures: verify `rpicam-still`/`libcamera-still` installation and permissions.
- `SPEC` failures: confirm `spectral_proc` is built and `SPECTRAL_BIN` path is valid.
- Motor control issues: verify Arduino serial port and command mapping for `0/1/2`.
- Missing sun values: confirm Arduino returns `SUN:<value>` on command `S`.

## Safety Notes
- `SAFE` immediately commands motor speed `0`.
- AUTO mode re-baselines initial pitch when enabled.
- Validate driver electronics before high magnetorquer duty settings.
