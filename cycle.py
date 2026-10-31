"""Surfacing cycle for the wheel mechtrode.

Geometry (v2): the substrate is a Y-Z plane; the wheel spins in the
vertical spindle; X is the force axis. One line is

    engage   +X at engage_feed until force reaches the target
    traverse  Y at feed, a P controller on X holds the force
    retract  -X by backoff
    step      Z by z_step (signed), then the next line

Lines run as a serpentine (alternate Y direction) or all the same way
(rapid back while retracted). Work-frame coordinates throughout; the
start position is wherever the head is when Cycle Start is pressed.

The traverse is streamed as ~slice_s long G1 slices paced about two
slices ahead of execution, so each X correction lands with roughly one
slice of latency instead of waiting behind a whole pass (lesson from v1).
"""
import csv
import math
import os
import threading
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "logs")

PARAMS = {                     # user-facing cycle parameters (defaults)
    "wheel_diameter_mm": 50.0,
    "surface_speed_m_min": 100.0,   # 0 = leave the spindle alone
    "force_g": 1000.0,
    "feed_mm_min": 60.0,            # Y traverse feed
    "pass_len_mm": 20.0,            # Y length of each line
    "n_lines": 1,
    "z_step_mm": 2.0,               # signed; applied between lines
    "backoff_mm": 1.0,              # -X retract between lines
    "engage_feed_mm_min": 30.0,
    "max_engage_mm": 5.0,           # +X travel allowed to find contact
    "travel_feed_mm_min": 600.0,
    "direction": "serpentine",      # or "same"
    "spindle_off_at_end": True,
    "hold_x": False,                # True: no engage / force loop / retract; X stays put
}
CONTROL = {                    # force-loop tuning (config.json "control")
    "kp_mm_per_g": 2e-5,       # X correction per gram of error, per slice
    "x_step_max_mm": 0.02,     # per-slice correction clamp
    "deadband_g": 50.0,
    "max_force_g": 4000.0,     # fault + retract above this
    "slice_s": 0.15,
    "engage_step_mm": 0.05,
    "spindle_settle_s": 90.0,
}
LOG_COLS = ["t", "state", "line", "raw", "force_g", "target_g",
            "x", "y", "z", "x_cmd", "spindle_hz", "spindle_a"]


class Abort(Exception):
    pass


class Cycle:
    def __init__(self, machine, loadcell, spindle, control):
        self.m, self.lc, self.sp = machine, loadcell, spindle
        self.ctl = control
        self.state = "idle"       # idle|spinup|engage|traverse|retract|step|hold|done|fault
        self.message = ""
        self.line = 0
        self.params = dict(PARAMS)
        self.engage_x = []        # work-frame X where contact was found, per line
        self.log_path = None
        self.events = []
        self._thread = None
        self._abort = threading.Event()
        self._hold = threading.Event()
        self._log = None
        self._x_cmd = 0.0

    # ------------------------------------------------------------------ #
    def status(self):
        return {"state": self.state, "message": self.message,
                "line": self.line, "n_lines": self.params["n_lines"],
                "engage_x": self.engage_x, "log": self.log_path,
                "params": self.params, "events": self.events[-12:],
                "running": self.running}

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def _event(self, msg):
        self.message = msg
        self.events.append(f"{time.strftime('%H:%M:%S')} {msg}")
        del self.events[:-200]

    def _preflight(self, need_cell=True):
        if self.running:
            return "already running"
        if not self.m.ready:
            return f"Klipper not ready ({self.m.state['klipper']})"
        if not self.m.homed_xyz:
            return "home all axes first"
        if need_cell and not self.lc.ok:
            return f"load cell: {self.lc.error}"
        return None

    def _launch(self, target, *args):
        self._abort.clear()
        self._hold.clear()
        self._thread = threading.Thread(target=self._guard, args=(target,) + args,
                                        daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ #
    # Public controls (Mach3-ish): Cycle Start / Feed Hold / Stop

    def start(self, params):
        err = self._preflight()
        if err:
            return False, err
        p = dict(PARAMS)
        for k, v in (params or {}).items():
            if k in p:
                p[k] = type(PARAMS[k])(v) if not isinstance(PARAMS[k], bool) else bool(v)
        if p["n_lines"] < 1 or p["pass_len_mm"] <= 0 or p["feed_mm_min"] <= 0:
            return False, "lines, pass length and feed must be positive"
        self.params = p
        self._launch(self._run_cycle)
        return True, "cycle started"

    def engage_only(self, params):
        """Setup aid: find contact at the current position and hold there."""
        err = self._preflight()
        if err:
            return False, err
        for k in ("force_g", "engage_feed_mm_min", "max_engage_mm"):
            if k in (params or {}):
                self.params[k] = float(params[k])
        self._launch(self._run_engage_only)
        return True, "engaging"

    def retract_only(self):
        err = self._preflight(need_cell=False)
        if err:
            return False, err
        self._launch(self._run_retract_only)
        return True, "retracting"

    def hold(self):
        if not self.running:
            return False, "nothing running"
        self._hold.set()
        return True, "feed hold after this slice"

    def resume(self):
        if not self.running or not self._hold.is_set():
            return False, "not holding"
        self._hold.clear()
        return True, "resuming"

    def stop(self):
        """Stop the cycle and back the wheel off the substrate."""
        if not self.running:
            return False, "nothing running"
        self._abort.set()
        self._hold.clear()
        return True, "stopping"

    def reset(self):
        if self.running:
            return False, "stop the cycle first"
        self.state, self.message, self.line = "idle", "", 0
        return True, "reset"

    # ------------------------------------------------------------------ #
    # Worker plumbing

    def _guard(self, target, *args):
        try:
            target(*args)
        except Abort:
            self._safe_retract()
            self.state = "idle"
            self._event("stopped")
        except Exception as e:           # noqa: BLE001 - any failure = fault
            self._safe_retract()
            self.state = "fault"
            self._event(f"FAULT: {e}")
        finally:
            self._close_log()

    def _g(self, script, timeout=120.0):
        ok, msg = self.m.gcode(script, timeout=timeout)
        if not ok:
            raise RuntimeError(f"{msg} [{script.splitlines()[0]}]")

    def _check(self):
        if self._abort.is_set():
            raise Abort()

    def _safe_retract(self):
        """Best effort: finish queued motion, then back off in -X."""
        try:
            self.m.wait_idle(timeout=60.0)
            x = self.m.work[0] - self.params["backoff_mm"]
            self.m.gcode(f"G90\nG1 X{x:.4f} F{self.params['travel_feed_mm_min']:g}\nM400",
                         timeout=60.0)
        except Exception:
            pass

    def _row(self, line):
        if self._log:
            sp = self.sp.status()
            w = self.m.work
            self._log.writerow([f"{time.time():.3f}", self.state, line,
                                self.lc.raw, f"{self.lc.grams:.1f}",
                                f"{self.params['force_g']:g}",
                                f"{w[0]:.4f}", f"{w[1]:.3f}", f"{w[2]:.3f}",
                                f"{self._x_cmd:.4f}",
                                sp.get("freq_hz", 0), sp.get("current_a", 0)])

    def _open_log(self, kind):
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = os.path.join(
            LOG_DIR, time.strftime(f"{kind}_%Y%m%d_%H%M%S.csv"))
        self._logf = open(self.log_path, "w", newline="")
        for k, v in self.params.items():
            self._logf.write(f"# {k}: {v}\n")
        for k, v in self.ctl.items():
            self._logf.write(f"# control.{k}: {v}\n")
        self._logf.write(f"# zero_counts: {self.lc.cfg['zero_counts']}  "
                         f"counts_per_g: {self.lc.cfg['counts_per_g']}\n")
        self._log = csv.writer(self._logf)
        self._log.writerow(LOG_COLS)

    def _close_log(self):
        if self._log:
            self._log = None
            self._logf.close()

    # ------------------------------------------------------------------ #
    # Primitives

    def _spinup(self):
        p = self.params
        if p["surface_speed_m_min"] <= 0:
            return
        rpm = p["surface_speed_m_min"] * 1000.0 / (math.pi * p["wheel_diameter_mm"])
        self.state = "spinup"
        ok, msg = self.sp.set_rpm(rpm)
        if not ok:
            raise RuntimeError(f"spindle: {msg}")
        self._event(f"spindle setpoint {rpm:.0f} rpm ({msg})")
        deadline = time.time() + self.ctl["spindle_settle_s"]
        while True:
            self._check()
            s = self.sp.status()
            if s["running"] and abs(s["freq_hz"] - s["setpoint_hz"]) <= 0.05 * max(s["setpoint_hz"], 1):
                return
            self.message = ("waiting for spindle: press RUN on the VFD"
                            if not s["running"] else
                            f"spindle ramping {s['freq_hz']:.1f}/{s['setpoint_hz']:.1f} Hz")
            if time.time() > deadline:
                raise RuntimeError("spindle did not reach speed")
            time.sleep(0.25)

    def _engage(self, line):
        """+X in small steps until the force target is met. Returns work X."""
        p, c = self.params, self.ctl
        self.state = "engage"
        x0 = self.m.work[0]
        x = x0
        if p["hold_x"]:                 # position mode: run from where X is
            self._event(f"line {line}: hold X at {x:.3f} (no engage)")
            return x
        self._g("G90")
        while True:
            self._check()
            f = self.lc.grams
            if f > c["max_force_g"]:
                raise RuntimeError(f"force {f:.0f} g over max during engage")
            if f >= p["force_g"] - c["deadband_g"]:
                break
            if x - x0 >= p["max_engage_mm"]:
                raise RuntimeError(f"no contact within {p['max_engage_mm']:g} mm")
            x += c["engage_step_mm"]
            self._x_cmd = x
            self._g(f"G1 X{x:.4f} F{p['engage_feed_mm_min']:g}\nM400")
            self._row(line)
        self._event(f"line {line}: contact at X {x:.3f} ({f:.0f} g)")
        return x

    def _traverse(self, line, x, y_from, y_to):
        """Stream Y slices; the P loop rides X corrections on each slice."""
        p, c = self.params, self.ctl
        self.state = "traverse"
        feed = p["feed_mm_min"]
        dist = abs(y_to - y_from)
        n = max(1, int(math.ceil(dist / max(0.01, feed / 60.0 * c["slice_s"]))))
        slice_dur = dist / n / (feed / 60.0)
        x_lim = x + p["max_engage_mm"]
        ahead = time.time()
        k = 0
        while k < n:
            self._check()
            if self._hold.is_set():
                self._do_hold(line)
                x = self._engage(line)          # re-find contact where we stopped
                self.state = "traverse"
                x_lim = x + p["max_engage_mm"]
                ahead = time.time()
            f = self.lc.grams
            if f > c["max_force_g"]:
                raise RuntimeError(f"force {f:.0f} g over max")
            err = p["force_g"] - f
            if not p["hold_x"] and abs(err) > c["deadband_g"]:
                dx = max(-c["x_step_max_mm"], min(c["x_step_max_mm"], c["kp_mm_per_g"] * err))
                x = min(x_lim, x + dx)
            k += 1
            y = y_from + (y_to - y_from) * k / n
            self._x_cmd = x
            self._g(f"G1 X{x:.4f} Y{y:.4f}" + (f" F{feed:g}" if k == 1 else ""))
            ahead = max(time.time(), ahead) + slice_dur
            while ahead - time.time() > 2.0 * slice_dur:
                time.sleep(0.01)
            self._row(line)
        self._g("M400")

    def _do_hold(self, line):
        self._g("M400")
        self._retract()
        self.state = "hold"
        self._event(f"feed hold at line {line} (retracted)")
        while self._hold.is_set():
            self._check()
            self._row(line)
            time.sleep(0.1)
        self._event("resuming")

    def _retract(self):
        p = self.params
        self.state = "retract"
        if p["hold_x"]:
            return
        x = self.m.work[0] - p["backoff_mm"]
        self._x_cmd = x
        self._g(f"G90\nG1 X{x:.4f} F{p['travel_feed_mm_min']:g}\nM400")

    def _step(self, z, y=None):
        p = self.params
        self.state = "step"
        words = f"Z{z:.4f}" + (f" Y{y:.4f}" if y is not None else "")
        self._g(f"G90\nG1 {words} F{p['travel_feed_mm_min']:g}\nM400")

    # ------------------------------------------------------------------ #
    # Programs

    def _run_cycle(self):
        p = self.params
        self.engage_x = []
        self._open_log("run")
        self._event("cycle start")
        self._spinup()
        x0, y0, z0 = self.m.work
        for i in range(p["n_lines"]):
            self.line = i + 1
            z = z0 + i * p["z_step_mm"]
            fwd = (i % 2 == 0) or p["direction"] != "serpentine"
            y_from, y_to = (y0, y0 + p["pass_len_mm"]) if fwd else (y0 + p["pass_len_mm"], y0)
            if i > 0:
                self._check()
                self._step(z, y_from if p["direction"] != "serpentine" else None)
            x = self._engage(self.line)
            self.engage_x.append(round(x, 4))
            self._traverse(self.line, x, y_from, y_to)
            self._retract()
        if p["spindle_off_at_end"] and p["surface_speed_m_min"] > 0:
            self.sp.set_hz(0.0)
        self.state = "done"
        self._event(f"done: {p['n_lines']} line(s), log {os.path.basename(self.log_path)}")

    def _run_engage_only(self):
        self._open_log("engage")
        x = self._engage(0)
        self.engage_x = [round(x, 4)]
        self.state = "done"
        self._event(f"engaged at X {x:.3f}; holding position (no force loop)")

    def _run_retract_only(self):
        self._retract()
        self.state = "idle"
        self._event("retracted")
