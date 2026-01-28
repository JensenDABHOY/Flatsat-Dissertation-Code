#!/usr/bin/env python3
import os
import sys
import time
import math
import json
import select
import subprocess
import threading
from datetime import datetime
from collections import deque

import serial
import numpy as np
from PIL import Image


# ---------------- utils ----------------

def now_tag() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def stop_requested() -> bool:
    # local stop only: q + Enter on Pi keyboard
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if r:
            s = sys.stdin.readline().strip().lower()
            return s == "q"
    except Exception:
        pass
    return False


def clamp(x, a, b):
    return max(a, min(b, x))


def f3(x):
    if x is None:
        return ""
    try:
        return f"{float(x):.3f}"
    except Exception:
        return ""


def accel_pitch_deg(ax, ay, az):
    if ax is None or ay is None or az is None:
        return None
    try:
        ax = float(ax)
        ay = float(ay)
        az = float(az)
    except Exception:
        return None

    denom = math.sqrt(ay * ay + az * az)
    if denom <= 1e-9:
        return None
    return math.degrees(math.atan2(-ax, denom))


def which(name: str):
    for p in os.environ.get("PATH", "").split(os.pathsep):
        cand = os.path.join(p, name)
        if os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    return None


# ---------------- Camera ----------------

def capture_image(out_dir: str) -> str:
    """
    Captures a JPEG WITHOUT preview window.
    """
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"camera_{now_tag()}.jpg")

    if which("rpicam-still"):
        # -n disables preview
        cmd = ["rpicam-still", "-n", "-o", out_path]
    elif which("libcamera-still"):
        # -n disables preview
        cmd = ["libcamera-still", "-n", "-o", out_path]
    else:
        raise RuntimeError("No rpicam-still or libcamera-still found")

    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return out_path


def top3_brightest_pixels(image_path: str):
    img = Image.open(image_path).convert("L")
    arr = np.array(img, dtype=np.uint8)
    flat = arr.ravel()
    if flat.size < 3:
        raise ValueError("Image too small for top-3")

    idx = np.argpartition(flat, -3)[-3:]
    idx = idx[np.argsort(flat[idx])[::-1]]

    h, w = arr.shape
    out = []
    for k in idx:
        y = int(k // w)
        x = int(k % w)
        out.append((x, y, int(arr[y, x])))
    return out, (w, h)


# ---------------- XBee UART link ----------------

class XBeeLink:
    def __init__(self, port: str, baud: int = 9600):
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.ser = None

    def open(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                return
            self.ser = serial.Serial(self.port, self.baud, timeout=0.0)

    def close(self):
        with self.lock:
            try:
                if self.ser:
                    self.ser.close()
            finally:
                self.ser = None

    def send_line(self, line: str):
        b = (line.rstrip("\n") + "\n").encode("utf-8")
        with self.lock:
            if not self.ser or not self.ser.is_open:
                self.open()
            self.ser.write(b)
            self.ser.flush()

    def read_lines(self, max_lines: int = 64):
        out = []
        with self.lock:
            if not self.ser or not self.ser.is_open:
                self.open()

            for _ in range(max_lines):
                raw = self.ser.readline()
                if not raw:
                    break
                s = raw.decode("utf-8", errors="replace").strip()
                if s:
                    out.append(s)
        return out


# ---------------- Arduino manager ----------------

class ArduinoManager:
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.ser = None

    def connect(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                return
            self.ser = serial.Serial(self.port, self.baud, timeout=0.3)
            time.sleep(1.8)
            self.ser.reset_input_buffer()

    def close(self):
        with self.lock:
            try:
                if self.ser:
                    self.ser.close()
            finally:
                self.ser = None

    def send_cmd(self, c: str, read: bool = True, max_wait_s: float = 0.6) -> list[str]:
        if len(c) != 1:
            raise ValueError("Arduino command must be a single character")

        with self.lock:
            if not self.ser or not self.ser.is_open:
                self.connect()

            self.ser.write((c + "\n").encode("ascii"))
            self.ser.flush()

            if not read:
                return []

            lines = []
            t0 = time.time()
            while time.time() - t0 < max_wait_s:
                line = self.ser.readline()
                if not line:
                    break
                s = line.decode("utf-8", errors="replace").strip()
                if s:
                    lines.append(s)
            return lines


def parse_sun_value(lines: list[str]):
    for ln in lines:
        if ln.startswith("SUN:"):
            try:
                return int(ln.split(":", 1)[1].strip())
            except Exception:
                return None
    return None


def motor_set(arduino: ArduinoManager, speed: int):
    cmd = {0: "0", 1: "1", 2: "2"}.get(speed)
    if cmd is None:
        raise ValueError("speed must be 0, 1, or 2")
    arduino.send_cmd(cmd, read=False)


# ---------------- Magnetorquer (coil) ----------------

class Magnetorquer:
    """
    Controls a coil with PWM duty and tries to measure coil current/voltage "directly"
    if an INA219 is available inline.

    Env:
      MTQ_GPIO=18              (PWM pin, optional)
      MTQ_SENSOR=ina219        (optional)
      MTQ_INA219_ADDR=0x40     (optional)
      MTQ_TURNS=200            (optional)
      MTQ_AREA_M2=0.0025       (optional)
    """

    def __init__(self):
        self.duty = 0.0
        self._pwm = None
        self._ina = None

        # coil parameters (for dipole moment m = N*A*I)
        self.turns = float(os.environ.get("MTQ_TURNS", "0"))
        self.area_m2 = float(os.environ.get("MTQ_AREA_M2", "0"))

        # PWM control
        gpio = os.environ.get("MTQ_GPIO", "").strip()
        if gpio:
            try:
                from gpiozero import PWMOutputDevice
                self._pwm = PWMOutputDevice(int(gpio), frequency=2000, initial_value=0.0)
            except Exception:
                self._pwm = None

        # INA219 sensing
        if os.environ.get("MTQ_SENSOR", "").strip().lower() == "ina219":
            try:
                import board
                import busio
                from adafruit_ina219 import INA219

                addr_str = os.environ.get("MTQ_INA219_ADDR", "0x40")
                addr = int(addr_str, 16)

                i2c = busio.I2C(board.SCL, board.SDA)
                self._ina = INA219(i2c, addr=addr)
            except Exception:
                self._ina = None

    def set_duty(self, duty: float):
        try:
            duty = float(duty)
        except Exception:
            return False

        duty = clamp(duty, 0.0, 1.0)
        self.duty = duty

        if self._pwm is not None:
            try:
                self._pwm.value = duty
            except Exception:
                pass
        return True

    def read_current_ma(self):
        if self._ina is None:
            return None
        try:
            # adafruit_ina219 current is in mA
            return float(self._ina.current)
        except Exception:
            return None

    def read_bus_v(self):
        if self._ina is None:
            return None
        try:
            return float(self._ina.bus_voltage)
        except Exception:
            return None

    def dipole_Am2(self, current_ma):
        if current_ma is None:
            return None
        if self.turns <= 0 or self.area_m2 <= 0:
            return None
        I = float(current_ma) / 1000.0
        return self.turns * self.area_m2 * I


# ---------------- Spectral processing ----------------

def generate_synthetic_frame(rows=64, cols=2048, seed=123, wl_min=300.0, wl_max=330.0):
    rng = np.random.default_rng(seed)
    wl = np.linspace(wl_min, wl_max, cols)
    x = (wl - wl.mean()) / (wl_max - wl_min)

    continuum = 1200.0 * (1.0 + 0.2 * x + 0.05 * x * x)
    y = continuum.copy()

    centers = np.array([312.34, 313.56, 327.85])
    amps = np.array([600.0, 750.0, 500.0])
    sig = 0.10
    for c, a in zip(centers, amps):
        y += a * np.exp(-0.5 * ((wl - c) / sig) ** 2)

    y += rng.normal(0.0, 80.0, size=cols)

    frame = np.repeat(y[None, :], rows, axis=0).astype(np.float64)
    frame += rng.normal(0.0, 5.0, size=frame.shape)
    return frame


def run_spectral_proc(frame: np.ndarray, out_dir: str,
                      spectral_bin: str = "./spectral_proc",
                      rows: int = 64, cols: int = 2048,
                      wl_min: float = 300.0, wl_max: float = 330.0):
    os.makedirs(out_dir, exist_ok=True)
    frame_path = os.path.join(out_dir, f"frame_{now_tag()}.bin")
    frame.astype(np.float64).tofile(frame_path)

    cmd = [
        spectral_bin,
        "--frame", frame_path,
        "--rows", str(rows),
        "--cols", str(cols),
        "--wl-min", str(wl_min),
        "--wl-max", str(wl_max),
        "--json",
    ]

    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True)
    t1 = time.time()

    out_lines = (p.stdout or "").splitlines()
    js = None
    for ln in out_lines:
        if ln.startswith("JSON:"):
            try:
                js = json.loads(ln[5:].strip())
            except Exception:
                js = None
            break

    return {
        "ok": (p.returncode == 0),
        "returncode": p.returncode,
        "wall_time_s": (t1 - t0),
        "json": js,
    }


# ---------------- Operational mode ----------------

class OperationalMode:
    """
    Compact downlink records (laptop builds CSV):

    TEL,t_ms,yaw_deg,roll_deg,pitch_deg,ax_ms2,ay_ms2,az_ms2,accel_pitch_deg,motor_state,motor_mode,init_pitch_deg
    SUN,t_ms,sun_raw,threshold,detected
    CAM,t_ms,x1,y1,val1,x2,y2,val2,x3,y3,val3
    MTQ,t_ms,duty,current_mA,bus_V,dipole_Am2
    MOT,t_ms,src,mode,speed,seconds
    SPEC,t_ms,ok,line_count,noise_std_adu,time_ms,wall_s,packet_bytes,(line0_centroid_nm,line0_fwhm_nm,line0_snr ... up to 8)

    History:
      HIST_BEGIN,count
      HIST_PROGRESS,n,count   (periodic)
      ...records...
      HIST_END
    """

    def __init__(self, arduino: ArduinoManager, xbee: XBeeLink, out_dir: str):
        self.arduino = arduino
        self.xbee = xbee
        self.out_dir = out_dir

        self.mtq = Magnetorquer()

        # sample rates
        self.TELE_DT = 0.5
        self.SUN_DT = 5.0
        self.CAM_DT = 30.0
        self.HB_DT = 6.0

        # thresholds
        self.sun_threshold = int(os.environ.get("SUN_THRESHOLD", "15"))
        self.pitch_on = float(os.environ.get("PITCH_ON_DEG", "5.0"))
        self.pitch_fast = float(os.environ.get("PITCH_FAST_DEG", "10.0"))

        # spectrum params
        self.spectral_bin = os.environ.get("SPECTRAL_BIN", "./spectral_proc")
        self.spec_rows = int(os.environ.get("SPEC_ROWS", "64"))
        self.spec_cols = int(os.environ.get("SPEC_COLS", "2048"))
        self.spec_wl_min = float(os.environ.get("SPEC_WL_MIN", "300.0"))
        self.spec_wl_max = float(os.environ.get("SPEC_WL_MAX", "330.0"))

        # state
        self.motor_state = 0
        self.motor_mode = "AUTO"  # AUTO or MANUAL
        self.initial_pitch = None

        # timed motor
        self._motor_timed_end = None

        # record buffer for DLCSV resend
        self.record_buf = deque(maxlen=25000)
        self.export_cursor_ms = 0

        # stream enabled
        self.stream_enabled = True

        # IMU
        self.sensor = None
        self._init_imu()

    def _init_imu(self):
        try:
            import board
            import busio
            import adafruit_bno055
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        except Exception:
            self.sensor = None

    def t_ms(self):
        return int(time.time() * 1000)

    def send(self, line: str):
        # save for DLCSV
        self.record_buf.append((self.t_ms(), line))
        if self.stream_enabled:
            self.xbee.send_line(line)

    def ack(self, msg: str):
        self.xbee.send_line("ACK " + msg)

    def err(self, msg: str):
        self.xbee.send_line("ERR " + msg)

    # -------- record builders --------

    def rec_tel(self, t, yaw, roll, pitch, ax, ay, az, apitch):
        ip = f3(self.initial_pitch)
        return "TEL," + ",".join([
            str(t),
            f3(yaw), f3(roll), f3(pitch),
            f3(ax), f3(ay), f3(az),
            f3(apitch),
            str(int(self.motor_state)),
            self.motor_mode,
            ip
        ])

    def rec_sun(self, t, sun, det):
        s = "" if sun is None else str(int(sun))
        d = "1" if det else "0"
        return f"SUN,{t},{s},{self.sun_threshold},{d}"

    def rec_cam(self, t, top3):
        if not top3 or len(top3) != 3:
            return f"CAM,{t},-1,-1,-1,-1,-1,-1,-1,-1,-1"
        vals = []
        for (x, y, v) in top3:
            vals += [str(int(x)), str(int(y)), str(int(v))]
        return "CAM," + str(t) + "," + ",".join(vals)

    def rec_mtq(self, t):
        cur = self.mtq.read_current_ma()
        bus = self.mtq.read_bus_v()
        dip = self.mtq.dipole_Am2(cur)
        return "MTQ," + ",".join([
            str(t),
            f"{self.mtq.duty:.3f}",
            "" if cur is None else f"{cur:.3f}",
            "" if bus is None else f"{bus:.3f}",
            "" if dip is None else f"{dip:.6f}",
        ])

    def rec_mot(self, t, src, mode, speed, seconds):
        sec = "" if seconds is None else f"{float(seconds):.3f}"
        return f"MOT,{t},{src},{mode},{int(speed)},{sec}"

    def rec_spec(self, t, ok, js, wall_s):
        if not js:
            return f"SPEC,{t},0,0,,,,,"
        line_count = js.get("line_count", 0)
        noise_std = js.get("noise_std", "")
        time_ms = js.get("time_ms", "")
        packet_bytes = js.get("packet_bytes", "")

        parts = [
            "SPEC",
            str(t),
            "1" if ok else "0",
            str(int(line_count)) if line_count is not None else "0",
            "" if noise_std in ("", None) else f"{float(noise_std):.6f}",
            "" if time_ms in ("", None) else f"{float(time_ms):.3f}",
            "" if wall_s is None else f"{float(wall_s):.4f}",
            "" if packet_bytes in ("", None) else str(int(packet_bytes)),
        ]

        lines = js.get("lines", []) or []
        for i in range(8):
            if i < len(lines):
                ln = lines[i]
                c = ln.get("centroid_nm", "")
                f = ln.get("fwhm_nm", "")
                s = ln.get("snr", "")
                parts += [
                    "" if c == "" else f"{float(c):.6f}",
                    "" if f == "" else f"{float(f):.6f}",
                    "" if s == "" else f"{float(s):.3f}",
                ]
            else:
                parts += ["", "", ""]
        return ",".join(parts)

    # -------- actions --------

    def do_sun(self):
        t = self.t_ms()
        try:
            lines = self.arduino.send_cmd("S", read=True)
            sun = parse_sun_value(lines)
        except Exception:
            sun = None
        det = (sun is not None and sun > self.sun_threshold)
        self.send(self.rec_sun(t, sun, det))
        self.ack(f"SUN value={sun} thr={self.sun_threshold} det={int(det)}")

    def do_cam(self):
        t = self.t_ms()
        try:
            path = capture_image(self.out_dir)
            top3, _ = top3_brightest_pixels(path)
            self.send(self.rec_cam(t, top3))
            self.ack(f"CAM top3={top3}")
        except Exception as e:
            self.send(self.rec_cam(t, None))
            self.err(f"CAM failed {e}")

    def do_spec(self):
        t = self.t_ms()
        try:
            frame = generate_synthetic_frame(
                rows=self.spec_rows,
                cols=self.spec_cols,
                seed=int(time.time()) & 0xFFFFFFFF,
                wl_min=self.spec_wl_min,
                wl_max=self.spec_wl_max,
            )
            res = run_spectral_proc(
                frame,
                out_dir=self.out_dir,
                spectral_bin=self.spectral_bin,
                rows=self.spec_rows,
                cols=self.spec_cols,
                wl_min=self.spec_wl_min,
                wl_max=self.spec_wl_max,
            )
            ok = bool(res["ok"])
            js = res.get("json", None)
            wall_s = res.get("wall_time_s", None)

            self.send(self.rec_spec(t, ok, js, wall_s))

            if js:
                self.ack(f"SPEC ok={int(ok)} line_count={js.get('line_count')} packet_bytes={js.get('packet_bytes')} time_ms={js.get('time_ms')}")
            else:
                self.ack(f"SPEC ok={int(ok)} no_json")
        except Exception as e:
            self.send(self.rec_spec(t, False, None, None))
            self.err(f"SPEC failed {e}")

    def set_motor(self, speed: int, src="MANUAL", mode="SET", seconds=None):
        speed = int(speed)
        speed = 0 if speed < 0 else (2 if speed > 2 else speed)
        try:
            motor_set(self.arduino, speed)
        except Exception:
            pass
        self.motor_state = speed
        self.send(self.rec_mot(self.t_ms(), src, mode, speed, seconds))

    def motor_manual(self, speed: int):
        self.motor_mode = "MANUAL"
        self._motor_timed_end = None
        self.set_motor(speed, src="MANUAL", mode="SET", seconds=None)
        self.ack(f"MOTOR speed={speed} mode=MANUAL")

    def motor_timed(self, speed: int, seconds: float):
        seconds = float(seconds)
        if seconds <= 0:
            self.err("MOTOR_TIMED usage: MOTOR_TIMED <0|1|2> <sec>")
            return
        self.motor_mode = "MANUAL"
        self.set_motor(speed, src="MANUAL", mode="TIMED", seconds=seconds)
        self._motor_timed_end = time.time() + seconds
        self.ack(f"MOTOR_TIMED speed={speed} seconds={seconds:.3f}")

    def motor_auto(self):
        self.motor_mode = "AUTO"
        self.initial_pitch = None
        self.ack("AUTO enabled")

    def safe(self):
        self.motor_mode = "AUTO"
        self.initial_pitch = None
        self._motor_timed_end = None
        self.set_motor(0, src="SAFE", mode="STOP", seconds=None)
        self.ack("SAFE stop motor")

    def set_mtq(self, duty):
        if not self.mtq.set_duty(duty):
            self.err("MTQ usage: MTQ <duty 0..1>")
            return
        t = self.t_ms()
        self.send(self.rec_mtq(t))
        cur = self.mtq.read_current_ma()
        bus = self.mtq.read_bus_v()
        self.ack(f"MTQ duty={self.mtq.duty:.3f} current_mA={cur} bus_V={bus}")

    def dlcsv(self, mode: str):
        """
        DLCSV
          DLCSV        -> send since last DLCSV
          DLCSV ALL    -> send all buffered
          DLCSV 60     -> send last 60 seconds (still respects cursor)
        """
        parts = mode.split()
        arg = parts[1] if len(parts) > 1 else ""

        now_ms = self.t_ms()
        start_ms = self.export_cursor_ms

        cutoff = None
        force_all = False

        if arg.upper() == "ALL":
            force_all = True
        elif arg.strip() != "":
            try:
                sec = int(float(arg))
                cutoff = now_ms - sec * 1000
            except Exception:
                cutoff = None

        rows = []
        for (t, line) in list(self.record_buf):
            if not force_all and t < start_ms:
                continue
            if cutoff is not None and t < cutoff:
                continue
            rows.append(line)

        self.xbee.send_line(f"HIST_BEGIN,{len(rows)}")
        for i, ln in enumerate(rows, 1):
            self.xbee.send_line(ln)
            if (i % 200) == 0:
                self.xbee.send_line(f"HIST_PROGRESS,{i},{len(rows)}")
        self.xbee.send_line("HIST_END")

        self.export_cursor_ms = now_ms
        self.ack(f"DLCSV sent={len(rows)}")

    def _auto_motor_control(self, pitch_val):
        if self.motor_mode != "AUTO":
            return
        if pitch_val is None:
            return

        if self.initial_pitch is None:
            self.initial_pitch = float(pitch_val)

        diff = float(pitch_val) - float(self.initial_pitch)
        ad = abs(diff)

        target = 0
        if ad > self.pitch_fast:
            target = 2
        elif ad >= self.pitch_on:
            target = 1
        else:
            target = 0

        if target != self.motor_state:
            self.set_motor(target, src="AUTO", mode="CTRL", seconds=None)
            self.ack(f"AUTO motor_state={target} pitch={pitch_val:.3f} init_pitch={self.initial_pitch:.3f}")

    # -------- command handler --------

    def handle_cmd(self, s: str):
        parts = s.strip().split()
        if not parts:
            return
        cmd = parts[0].upper()

        if cmd in ("HELP", "?"):
            self.xbee.send_line(
                "HELP: SUN CAM SPEC STATUS SAFE AUTO MOTOR <0|1|2> MOTOR_TIMED <0|1|2> <sec> "
                "MTQ <duty0..1> DLCSV [ALL|seconds] XON XOFF"
            )
            return

        if cmd == "STATUS":
            self.ack(f"STATUS motor_state={self.motor_state} motor_mode={self.motor_mode} init_pitch={self.initial_pitch}")
            return

        if cmd == "SUN":
            self.do_sun()
            return

        if cmd == "CAM":
            self.do_cam()
            return

        if cmd == "SPEC":
            self.do_spec()
            return

        if cmd == "SAFE":
            self.safe()
            return

        if cmd == "AUTO":
            self.motor_auto()
            return

        if cmd == "MOTOR":
            if len(parts) != 2:
                self.err("MOTOR usage: MOTOR 0|1|2")
                return
            try:
                self.motor_manual(int(parts[1]))
            except Exception:
                self.err("MOTOR usage: MOTOR 0|1|2")
            return

        if cmd == "MOTOR_TIMED":
            if len(parts) != 3:
                self.err("MOTOR_TIMED usage: MOTOR_TIMED <0|1|2> <sec>")
                return
            try:
                self.motor_timed(int(parts[1]), float(parts[2]))
            except Exception:
                self.err("MOTOR_TIMED usage: MOTOR_TIMED <0|1|2> <sec>")
            return

        if cmd == "MTQ":
            if len(parts) != 2:
                self.err("MTQ usage: MTQ <duty 0..1>")
                return
            self.set_mtq(parts[1])
            return

        if cmd == "DLCSV":
            self.dlcsv(s)
            return

        if cmd == "XOFF":
            self.stream_enabled = False
            self.ack("STREAM_OFF")
            return

        if cmd == "XON":
            self.stream_enabled = True
            self.ack("STREAM_ON")
            return

        self.err(f"Unknown command: {s}")

    # -------- main loop --------

    def loop(self):
        self.ack("OPS_READY")

        next_tel = time.time()
        next_sun = time.time() + 0.5
        next_cam = time.time() + 2.0

        while True:
            if stop_requested():
                break

            # uplink handling
            for cmd in self.xbee.read_lines():
                self.handle_cmd(cmd)

            # timed motor stop
            if self._motor_timed_end is not None and time.time() >= self._motor_timed_end:
                self._motor_timed_end = None
                self.set_motor(0, src="MANUAL", mode="TIMED_END", seconds=None)
                self.motor_mode = "AUTO"
                self.initial_pitch = None
                self.ack("MOTOR_TIMED finished, back to AUTO")

            now = time.time()

            # telemetry sampling
            if now >= next_tel:
                next_tel += self.TELE_DT

                t = self.t_ms()
                yaw = roll = pitch = None
                ax = ay = az = None

                if self.sensor is not None:
                    try:
                        euler = self.sensor.euler
                        accel = self.sensor.acceleration
                        if euler and len(euler) == 3:
                            yaw, roll, pitch = euler[0], euler[1], euler[2]
                        if accel and len(accel) == 3:
                            ax, ay, az = accel[0], accel[1], accel[2]
                    except Exception:
                        pass

                apitch = accel_pitch_deg(ax, ay, az)
                pitch_val = pitch if pitch is not None else apitch
                self._auto_motor_control(pitch_val)

                self.send(self.rec_tel(t, yaw, roll, pitch, ax, ay, az, apitch))

            # periodic sun
            if now >= next_sun:
                next_sun += self.SUN_DT
                try:
                    self.do_sun()
                except Exception:
                    pass

            # periodic camera
            if now >= next_cam:
                next_cam += self.CAM_DT
                try:
                    self.do_cam()
                except Exception:
                    pass

            time.sleep(0.01)

        # stop motor on exit
        try:
            motor_set(self.arduino, 0)
        except Exception:
            pass
        self.ack("OPS_EXIT")


def main():
    out_dir = os.path.expanduser("~/flatsat_tests")
    arduino_port = os.environ.get("ARDUINO_PORT", "/dev/ttyACM0")
    xbee_port = os.environ.get("XBEE_PORT", "/dev/serial0")

    print(f"Arduino port: {arduino_port}")
    print(f"XBee UART:    {xbee_port}")
    print(f"Output dir:   {out_dir}")
    print("Operational mode running. Type q + Enter locally to stop.")

    arduino = ArduinoManager(arduino_port)
    xbee = XBeeLink(xbee_port, 9600)

    try:
        arduino.connect()
    except Exception:
        pass

    try:
        xbee.open()
    except Exception:
        pass

    ops = OperationalMode(arduino, xbee, out_dir)
    try:
        ops.loop()
    finally:
        try:
            arduino.close()
        except Exception:
            pass
        try:
            xbee.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()