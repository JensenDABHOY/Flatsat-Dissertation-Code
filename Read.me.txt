# FlatSat Ops System
README and operator guide for `flatsat_ops.py`, `ground_station_16.py`, and `spectral_proc.c`.
Document version: 2026-01-28 (UTC)

## Overview
This system runs a Raspberry Pi FlatSat and a laptop ground station. The Pi collects IMU, sun sensor,
camera, magnetorquer, motor, and spectrum processing data, then downlinks compact ASCII records over
an XBee transparent UART link. The laptop logs these records into a single wide CSV and provides a
terminal UI for uplink commands.

## Files and responsibilities
- `flatsat_ops.py` (Raspberry Pi): main ops loop, hardware sampling, uplink handler, downlink record
  generator.
- `spectral_proc.c` (Raspberry Pi): C spectrum processor called by `flatsat_ops.py`. Produces a JSON
  line with detected spectral lines.
- `ground_station_16.py` (Laptop): curses-based terminal, TCP client to the XBee bridge, CSV logger
  with stateful snapshot rows and DLCSV replay support.
- `flatsat_log_analysis.ipynb`: notebook that loads the latest CSV from `C:\Windows\System32\logs`
  and plots telemetry, events, motor activity, camera brightest pixels, magnetorquer data, and
  spectral results.

## High-level data flow
- Uplink: one ASCII command per line from laptop to Pi.
- Downlink: one ASCII record per line from Pi to laptop.
- The laptop is the source of truth for the CSV; the Pi never sends CSV.
- DLCSV triggers a replay of buffered records from the Pi.

## Installation prerequisites

### Raspberry Pi
Python packages:
- pyserial
- numpy
- Pillow (PIL)
- adafruit-circuitpython-bno055, board, busio
Optional (magnetorquer sensing/control):
- gpiozero
- adafruit-ina219

System tools:
- gcc
- rpicam-still or libcamera-still

### Laptop
- Python 3.x
- curses UI dependency (Windows): `pip install windows-curses`

## Build the spectrum processor
```
gcc -O3 -std=c11 spectral_proc.c -lm -o spectral_proc
```

If you see `clock_gettime` errors:
```
gcc -O3 -std=c11 -D_POSIX_C_SOURCE=200809L spectral_proc.c -lm -o spectral_proc
```

## Running the system

### Raspberry Pi (flatsat_ops.py)
```
export ARDUINO_PORT=/dev/ttyACM0
export XBEE_PORT=/dev/serial0
export SPECTRAL_BIN=./spectral_proc
python3 flatsat_ops.py
```
- Stop locally by typing `q` and pressing Enter.

### Laptop (ground_station_16.py)
```
python3 ground_station_16.py --host 192.168.10.1 --port 9750 --logdir C:\Windows\System32\logs
```
- Output CSV name: `flatsat_session_YYYYMMDD_HHMMSS.csv`.

## Configuration (flatsat_ops.py env vars)
- `ARDUINO_PORT`: Arduino serial port (default `/dev/ttyACM0`).
- `XBEE_PORT`: XBee UART device (default `/dev/serial0`).
- `SUN_THRESHOLD`: sun detection threshold (default `15`).
- `PITCH_ON_DEG`: AUTO motor on threshold (default `5.0`).
- `PITCH_FAST_DEG`: AUTO motor fast threshold (default `10.0`).
- `SPECTRAL_BIN`: path to `spectral_proc` (default `./spectral_proc`).
- `SPEC_ROWS`, `SPEC_COLS`: synthetic frame size (default `64`, `2048`).
- `SPEC_WL_MIN`, `SPEC_WL_MAX`: synthetic wavelength range (default `300.0`, `330.0`).
- `MTQ_GPIO`: GPIO pin for PWM duty control (optional).
- `MTQ_SENSOR`: set to `ina219` to enable current/voltage sensing (optional).
- `MTQ_INA219_ADDR`: INA219 I2C address (default `0x40`).
- `MTQ_TURNS`, `MTQ_AREA_M2`: coil parameters for dipole estimate (optional).

Sampling intervals are hard-coded in `flatsat_ops.py` (TELE=0.5 s, SUN=5 s, CAM=30 s).

## Protocol reference

### Structured downlink records (stored to CSV)
- `TEL,t_ms,yaw,roll,pitch,ax,ay,az,accel_pitch,motor_state,motor_mode,initial_pitch`
- `SUN,t_ms,sun_raw,threshold,detected`
- `CAM,t_ms,x1,y1,v1,x2,y2,v2,x3,y3,v3`
- `MTQ,t_ms,duty,current_mA,bus_V,dipole_Am2` (CSV stores only the first three values after `t_ms`)
- `MOT,t_ms,src,mode,speed,seconds`
- `SPEC,t_ms,ok,line_count,noise_std_adu,time_ms,wall_s,packet_bytes,(line0_centroid_nm,line0_fwhm_nm,line0_snr ... up to 8)`

### Informational downlink lines (shown live, not stored)
- `ACK <message>`
- `ERR <message>`
- `INFO <message>`
- `STATUS <message>`
- `HIST_BEGIN,<count>`, `HIST_PROGRESS,<i>,<count>`, `HIST_END`

## Uplink commands
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
- `XOFF` (disable downlink stream)
- `XON` (enable downlink stream)

## CSV schema (ground_station_16.py)
The CSV is stateful: each row contains the latest values for all fields. Use `event_type` to filter
for fresh data from a subsystem.

Core columns:
- `rx_utc_iso`: laptop receive timestamp (UTC, ISO 8601).
- `event_type`: triggering tag (TEL, SUN, CAM, MTQ, MOT, SPEC).
- `event_sat_time_ms`: Pi time in milliseconds.

Telemetry (TEL rows):
- `imu_yaw_deg`, `imu_roll_deg`, `imu_pitch_deg`
- `accel_x_mps2`, `accel_y_mps2`, `accel_z_mps2`, `accel_pitch_deg`
- `motor_state_cmd`, `motor_mode`, `motor_initial_pitch_deg`

Sun sensor (SUN rows):
- `sun_value_raw`, `sun_threshold_raw`, `sun_detected`

Camera (CAM rows):
- `cam_p1_x_px`, `cam_p1_y_px`, `cam_p1_value`
- `cam_p2_x_px`, `cam_p2_y_px`, `cam_p2_value`
- `cam_p3_x_px`, `cam_p3_y_px`, `cam_p3_value`

Magnetorquer (MTQ rows):
- `mtq_x` (duty), `mtq_y` (current_mA), `mtq_z` (bus_V)

Motor events (MOT rows):
- `motor_last_src`, `motor_last_mode`, `motor_last_speed`, `motor_last_seconds`

Spectrum summary (SPEC rows):
- `spec_ok`, `spec_num_lines`, `spec_noise_std_adu`
- `spec_runtime_ms`, `spec_wall_time_s`, `spec_packet_bytes`

Spectrum lines (SPEC rows):
- `spec_line<i>_centroid_nm`, `spec_line<i>_fwhm_nm`, `spec_line<i>_snr` for i=0..7.

## Analysis notebook
Open `flatsat_log_analysis.ipynb` in Jupyter. It automatically selects the latest CSV from
`C:\Windows\System32\logs` and generates plots for all logged subsystems, events, and spectral line
details.

## Troubleshooting
- No connection: verify host/port for the TCP bridge.
- No IMU data: confirm BNO055 wiring and Adafruit libraries.
- Camera capture fails: ensure `rpicam-still` or `libcamera-still` is installed.
- Spectrum fails: confirm `spectral_proc` is compiled and `SPECTRAL_BIN` is correct.
- Motor does not respond: check Arduino port and firmware command mapping (0/1/2).
- Sun sensor is NA: ensure Arduino `S` command returns `SUN:<value>`.

## Safety and operational notes
- `SAFE` stops the motor immediately.
- AUTO mode baselines pitch on the first valid reading or after `AUTO`.
- For bench tests, increase `CAM_DT` in code or disable camera triggers if you want less disk churn.
- If you use `MTQ_GPIO`, validate your driver circuit before commanding high duty cycles.
