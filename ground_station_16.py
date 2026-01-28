#!/usr/bin/env python3
import os
import csv
import time
import socket
import curses
import select
from datetime import datetime


# ======================
# Helpers
# ======================

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def utc_iso():
    return datetime.utcnow().isoformat(timespec="milliseconds")

def now_tag():
    return datetime.utcnow().strftime("%Y%m%d_%H%M%S")

def safe_str(x):
    return "" if x is None else str(x).strip()

def human_eta(seconds: float) -> str:
    if seconds < 0 or not (seconds < 1e9):
        return "?"
    m = int(seconds // 60)
    s = int(seconds % 60)
    if m <= 0:
        return f"{s}s"
    return f"{m}m{s:02d}s"


# ======================
# TCP connect/reconnect
# ======================

def tcp_connect_wait(host: str, port: int, retry_s: float = 1.0) -> socket.socket:
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((host, port))
            s.settimeout(None)
            return s
        except Exception:
            time.sleep(retry_s)


# ======================
# CSV Logger (stateful snapshot rows)
# ======================

class StatefulCSVLogger:
    """
    Logs "snapshot rows" where each row contains the latest known values for all fields.
    Every incoming record updates a subset of state, then we write a full row.

    Key features:
    - Descriptive column names
    - Dedup protection (important when DLCSV replays history)
    - One CSV file for the whole session
    """

    def __init__(self, out_dir: str, filename: str | None = None):
        ensure_dir(out_dir)
        if filename is None:
            filename = f"flatsat_session_{now_tag()}.csv"
        self.path = os.path.join(out_dir, filename)

        # Core columns
        cols = [
            # when laptop received the message
            "rx_utc_iso",

            # what message triggered the row
            "event_type",
            "event_sat_time_ms",

            # -------- IMU (Euler) --------
            "imu_yaw_deg",
            "imu_roll_deg",
            "imu_pitch_deg",

            # -------- Accelerometer --------
            "accel_x_mps2",
            "accel_y_mps2",
            "accel_z_mps2",
            "accel_pitch_deg",

            # -------- Motor status --------
            "motor_state_cmd",     # 0/1/2 actual motor state
            "motor_mode",          # AUTO/MANUAL
            "motor_initial_pitch_deg",

            # last motor command metadata (most recent MOT record)
            "motor_last_src",      # MANUAL/AUTO/SAFE
            "motor_last_mode",     # SET/TIMED/CTRL/TIMED_END/STOP
            "motor_last_speed",
            "motor_last_seconds",

            # -------- Sun sensor --------
            "sun_value_raw",
            "sun_threshold_raw",
            "sun_detected",

            # -------- Camera brightest pixels --------
            "cam_p1_x_px", "cam_p1_y_px", "cam_p1_value",
            "cam_p2_x_px", "cam_p2_y_px", "cam_p2_value",
            "cam_p3_x_px", "cam_p3_y_px", "cam_p3_value",

            # -------- Magnetorquer (coil) setpoints/telemetry --------
            # (these are whatever the Pi downlinks; can be measured/current later)
            "mtq_x",
            "mtq_y",
            "mtq_z",

            # -------- Spectrum summary --------
            "spec_ok",
            "spec_num_lines",
            "spec_noise_std_adu",
            "spec_runtime_ms",
            "spec_wall_time_s",
            "spec_packet_bytes",
        ]

        # Up to 8 fitted lines: centroid, fwhm, snr
        for i in range(8):
            cols += [
                f"spec_line{i}_centroid_nm",
                f"spec_line{i}_fwhm_nm",
                f"spec_line{i}_snr",
            ]

        self.cols = cols

        file_exists = os.path.isfile(self.path)
        self.f = open(self.path, "a", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.f, fieldnames=self.cols)
        if not file_exists:
            self.w.writeheader()
            self.f.flush()

        # latest known state
        self.state = {k: "" for k in self.cols}

        # dedup cache
        self._seen = {}
        self._seen_max = 8000

        # stats
        self.rows_written = 0

    def close(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass

    def _dedup_ok(self, event_type: str, sat_ms: str, raw_line: str) -> bool:
        # stable key for duplicate suppression
        key = f"{event_type}|{sat_ms}|{raw_line}"
        if key in self._seen:
            return False
        self._seen[key] = time.time()

        if len(self._seen) > self._seen_max:
            # prune oldest quickly
            for k in list(self._seen.keys())[:1500]:
                self._seen.pop(k, None)
        return True

    def update_and_write(self, event_type: str, sat_ms: str, updates: dict, raw_line: str):
        if not self._dedup_ok(event_type, sat_ms, raw_line):
            return

        self.state["rx_utc_iso"] = utc_iso()
        self.state["event_type"] = event_type
        self.state["event_sat_time_ms"] = sat_ms

        for k, v in updates.items():
            if k in self.state:
                self.state[k] = v

        self.w.writerow(self.state)
        self.f.flush()
        self.rows_written += 1


# ======================
# Downlink parser
# ======================

def parse_downlink_record(line: str):
    """
    Converts a compact downlink line into:
      (event_type, sat_ms, updates_dict)

    If not a structured record, returns (None, None, None).
    """
    parts = line.split(",")
    tag = parts[0].strip().upper()

    def g(i):
        return safe_str(parts[i]) if i < len(parts) else ""

    # TEL,t_ms,yaw,roll,pitch,ax,ay,az,apitch,motor_state,motor_mode,init_pitch
    if tag == "TEL":
        sat_ms = g(1)
        updates = {
            "imu_yaw_deg": g(2),
            "imu_roll_deg": g(3),
            "imu_pitch_deg": g(4),
            "accel_x_mps2": g(5),
            "accel_y_mps2": g(6),
            "accel_z_mps2": g(7),
            "accel_pitch_deg": g(8),
            "motor_state_cmd": g(9),
            "motor_mode": g(10),
            "motor_initial_pitch_deg": g(11),
        }
        return "TEL", sat_ms, updates

    # SUN,t_ms,sun,thr,det
    if tag == "SUN":
        sat_ms = g(1)
        updates = {
            "sun_value_raw": g(2),
            "sun_threshold_raw": g(3),
            "sun_detected": g(4),
        }
        return "SUN", sat_ms, updates

    # CAM,t_ms,x1,y1,v1,x2,y2,v2,x3,y3,v3
    if tag == "CAM":
        sat_ms = g(1)
        updates = {
            "cam_p1_x_px": g(2), "cam_p1_y_px": g(3), "cam_p1_value": g(4),
            "cam_p2_x_px": g(5), "cam_p2_y_px": g(6), "cam_p2_value": g(7),
            "cam_p3_x_px": g(8), "cam_p3_y_px": g(9), "cam_p3_value": g(10),
        }
        return "CAM", sat_ms, updates

    # MTQ,t_ms,mx,my,mz
    if tag == "MTQ":
        sat_ms = g(1)
        updates = {
            "mtq_x": g(2),
            "mtq_y": g(3),
            "mtq_z": g(4),
        }
        return "MTQ", sat_ms, updates

    # MOT,t_ms,src,mode,speed,seconds
    if tag == "MOT":
        sat_ms = g(1)
        updates = {
            "motor_last_src": g(2),
            "motor_last_mode": g(3),
            "motor_last_speed": g(4),
            "motor_last_seconds": g(5),
        }
        return "MOT", sat_ms, updates

    # SPEC,t_ms,ok,line_count,noise_std,time_ms,wall_s,packet_bytes,(c0,f0,s0 ... up to 8 lines)
    if tag == "SPEC":
        sat_ms = g(1)
        updates = {
            "spec_ok": g(2),
            "spec_num_lines": g(3),
            "spec_noise_std_adu": g(4),
            "spec_runtime_ms": g(5),
            "spec_wall_time_s": g(6),
            "spec_packet_bytes": g(7),
        }
        base = 8
        for i in range(8):
            c_i = base + i * 3
            f_i = base + i * 3 + 1
            s_i = base + i * 3 + 2
            updates[f"spec_line{i}_centroid_nm"] = g(c_i)
            updates[f"spec_line{i}_fwhm_nm"] = g(f_i)
            updates[f"spec_line{i}_snr"] = g(s_i)
        return "SPEC", sat_ms, updates

    # HIST markers handled outside
    return None, None, None


# ======================
# Curses UI
# ======================

class UI:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        curses.curs_set(1)
        stdscr.nodelay(True)
        stdscr.keypad(True)

        self.resize()

    def resize(self):
        h, w = self.stdscr.getmaxyx()
        self.h = h
        self.w = w

        # layout:
        # [output window]
        # [status line]
        # [input line]
        out_h = max(6, h - 2)
        self.out_win = curses.newwin(out_h, w, 0, 0)
        self.status_win = curses.newwin(1, w, out_h, 0)
        self.input_win = curses.newwin(1, w, out_h + 1, 0)

        self.out_win.scrollok(True)
        self.out_win.idlok(True)

    def out(self, s: str):
        # prints to output area only
        self.out_win.addstr(s[: max(0, self.w - 1)] + "\n")
        self.out_win.refresh()

    def status(self, s: str):
        self.status_win.erase()
        self.status_win.addstr(0, 0, s[: max(0, self.w - 1)])
        self.status_win.refresh()

    def draw_input(self, buf: str, cursor_pos: int):
        """
        Critical: this never gets overwritten by output updates.
        So you can keep typing while data arrives.
        """
        self.input_win.erase()
        prompt = "> "
        shown = prompt + buf
        self.input_win.addstr(0, 0, shown[: max(0, self.w - 1)])

        cur_x = min(self.w - 1, len(prompt) + cursor_pos)
        self.input_win.move(0, cur_x)
        self.input_win.refresh()

    def get_key(self):
        try:
            return self.stdscr.getch()
        except Exception:
            return -1


# ======================
# Main app loop
# ======================

def run(stdscr, host: str, port: int, logdir: str):
    ui = UI(stdscr)

    ensure_dir(logdir)
    logger = StatefulCSVLogger(logdir)

    ui.out(f"Connected CSV file: {logger.path}")
    ui.out("Commands: HELP, STATUS, SUN, CAM, SPEC, MOTOR 0|1|2, MOTOR_TIMED 2 3.0, MTQ x y z, DLCSV")
    ui.out("Local quit: Ctrl+C")

    sock = None
    buf = b""

    # input buffer
    in_buf = ""
    cur = 0

    # DLCSV history progress
    hist_mode = False
    hist_total = 0
    hist_got = 0
    hist_rows_written_at_start = 0
    hist_t0 = 0.0

    # useful stats
    tel_count = 0
    tel_rate = 0.0
    tel_t0 = time.time()

    def connect():
        nonlocal sock, buf
        ui.status("Connecting...")
        sock = tcp_connect_wait(host, port)
        sock.setblocking(False)
        buf = b""
        ui.out(f"TCP connected to {host}:{port}")
        ui.status(f"CSV={logger.path}")

    connect()

    def send_cmd(cmd: str):
        nonlocal sock
        if not sock:
            return
        try:
            sock.sendall((cmd.strip() + "\n").encode("utf-8"))
        except Exception:
            pass

    ui.draw_input(in_buf, cur)

    while True:
        try:
            # Handle terminal resize
            if curses.is_term_resized(ui.h, ui.w):
                ui.resize()
                ui.out("UI resized")
                ui.draw_input(in_buf, cur)

            # -------------------------
            # Receive socket data
            # -------------------------
            if sock:
                r, _, _ = select.select([sock], [], [], 0.0)
                if r:
                    data = sock.recv(4096)
                    if not data:
                        raise ConnectionError("Socket closed by remote")
                    buf += data

                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        s = line.decode("utf-8", errors="replace").strip()
                        if not s:
                            continue

                        # History begin/end
                        if s.startswith("HIST_BEGIN"):
                            hist_mode = True
                            hist_got = 0
                            hist_total = 0
                            hist_t0 = time.time()
                            hist_rows_written_at_start = logger.rows_written
                            try:
                                hist_total = int(s.split(",", 1)[1].strip())
                            except Exception:
                                hist_total = 0
                            ui.out(f"HIST_BEGIN total={hist_total}")
                            ui.draw_input(in_buf, cur)
                            continue

                        if s == "HIST_END":
                            dt = max(1e-6, time.time() - hist_t0)
                            wrote = logger.rows_written - hist_rows_written_at_start
                            rate = hist_got / dt
                            ui.out(f"HIST_END got={hist_got} wrote_rows={wrote} rate={rate:.1f} rec/s")
                            ui.out(f"CSV path: {logger.path}")
                            hist_mode = False
                            hist_total = 0
                            hist_got = 0
                            ui.draw_input(in_buf, cur)
                            continue

                        # ACK/ERR/INFO/STATUS lines should show
                        if s.startswith(("ACK", "ERR", "INFO", "STATUS")):
                            ui.out(s)
                            ui.draw_input(in_buf, cur)
                            continue

                        event, sat_ms, updates = parse_downlink_record(s)

                        if event is None:
                            # unknown line, but still show if not too spammy
                            if not s.startswith("TEL"):
                                ui.out(s)
                                ui.draw_input(in_buf, cur)
                            continue

                        # CSV append
                        logger.update_and_write(event, sat_ms, updates, raw_line=s)

                        # Progress stats
                        if event == "TEL":
                            tel_count += 1
                            now = time.time()
                            if now - tel_t0 >= 2.0:
                                tel_rate = tel_count / (now - tel_t0)
                                tel_count = 0
                                tel_t0 = now

                        if hist_mode:
                            hist_got += 1
                            # update status with progress, do NOT disturb input
                            if hist_total > 0:
                                dt = max(1e-6, time.time() - hist_t0)
                                rps = hist_got / dt
                                remain = hist_total - hist_got
                                eta = human_eta(remain / rps) if rps > 0 else "?"
                                pct = 100.0 * hist_got / hist_total
                                ui.status(f"DLCSV {hist_got}/{hist_total} ({pct:.1f}%)  {rps:.1f} rec/s  ETA {eta}   CSV={logger.path}")
                            else:
                                ui.status(f"DLCSV got={hist_got}   CSV={logger.path}")
                        else:
                            # show important event lines
                            if event in ("SUN", "CAM", "SPEC", "MOT", "MTQ"):
                                ui.out(s)
                                ui.draw_input(in_buf, cur)

            # -------------------------
            # Draw status (normal)
            # -------------------------
            if not hist_mode:
                ui.status(f"TEL ~{tel_rate:.1f}/s   CSV={logger.path}")

            # -------------------------
            # Keyboard input
            # -------------------------
            key = ui.get_key()
            if key != -1:
                # Enter
                if key in (10, 13):
                    cmd = in_buf.strip()
                    if cmd:
                        send_cmd(cmd)
                        ui.out(f"UPLINK: {cmd}")
                    in_buf = ""
                    cur = 0
                    ui.draw_input(in_buf, cur)

                # Backspace
                elif key in (127, 8, curses.KEY_BACKSPACE):
                    if cur > 0:
                        in_buf = in_buf[:cur - 1] + in_buf[cur:]
                        cur -= 1
                        ui.draw_input(in_buf, cur)

                # Left / right arrows
                elif key == curses.KEY_LEFT:
                    cur = max(0, cur - 1)
                    ui.draw_input(in_buf, cur)

                elif key == curses.KEY_RIGHT:
                    cur = min(len(in_buf), cur + 1)
                    ui.draw_input(in_buf, cur)

                # Home/end
                elif key == curses.KEY_HOME:
                    cur = 0
                    ui.draw_input(in_buf, cur)

                elif key == curses.KEY_END:
                    cur = len(in_buf)
                    ui.draw_input(in_buf, cur)

                # Printable characters
                elif 32 <= key <= 126:
                    ch = chr(key)
                    in_buf = in_buf[:cur] + ch + in_buf[cur:]
                    cur += 1
                    ui.draw_input(in_buf, cur)

            time.sleep(0.02)

        except KeyboardInterrupt:
            break
        except Exception as e:
            ui.out(f"DISCONNECTED: {e}")
            ui.out("Reconnecting...")
            try:
                if sock:
                    sock.close()
            except Exception:
                pass
            sock = None
            time.sleep(1.0)
            connect()
            ui.draw_input(in_buf, cur)

    # shutdown
    try:
        if sock:
            sock.close()
    except Exception:
        pass
    logger.close()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--logdir", default="logs")
    args = ap.parse_args()

    ensure_dir(args.logdir)
    curses.wrapper(run, args.host, args.port, args.logdir)


if __name__ == "__main__":
    main()
