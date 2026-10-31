"""Klipper via Moonraker: send g-code, track position and homing state.

Positions are reported two ways, Mach3-style:
  machine - Klipper's live toolhead position (motion_report.live_position)
  work    - the same point in the g-code frame, i.e. after G92 / offsets.
G92 Xn (the DRO "Zero" buttons) shifts the work frame; G1 moves in it.
"""
import re
import threading
import time

import requests

MOONRAKER = "http://127.0.0.1:7125"
POLL_S = 0.1
QUERY = ("/printer/objects/query?gcode_move=gcode_position,position"
         "&toolhead=homed_axes&motion_report=live_position"
         "&webhooks=state,state_message"
         "&gcode_macro%20THETA_STATE=pos&stepper_enable=steppers")  # theta, optional


class Machine:
    def __init__(self, url=MOONRAKER):
        self.url = url
        self._s = requests.Session()
        self._running = True
        self.state = {
            "online": False, "klipper": "unknown", "message": "",
            "homed": "", "machine": [0.0, 0.0, 0.0], "work": [0.0, 0.0, 0.0],
            # theta: rotary tombstone on a Klipper [manual_stepper theta]; its
            # position lives in the THETA_STATE macro variable (klipper/theta.cfg).
            # present=False until that config is included in printer.cfg.
            "theta": {"present": False, "pos": 0.0, "enabled": False},
        }

    def start(self):
        threading.Thread(target=self._poll, daemon=True).start()

    # ------------------------------------------------------------------ #
    def _poll(self):
        while self._running:
            t0 = time.time()
            try:
                r = self._s.get(self.url + QUERY, timeout=2.0)
                r.raise_for_status()
                st = r.json()["result"]["status"]
                gm = st.get("gcode_move", {})
                live = st.get("motion_report", {}).get("live_position")
                pos, gpos = gm.get("position"), gm.get("gcode_position")
                if live and pos and gpos:
                    off = [p - g for p, g in zip(pos, gpos)]
                    self.state["machine"] = [round(v, 3) for v in live[:3]]
                    self.state["work"] = [round(l - o, 3)
                                          for l, o in zip(live[:3], off[:3])]
                self.state["homed"] = st.get("toolhead", {}).get("homed_axes", "")
                th = st.get("gcode_macro THETA_STATE") or {}
                en = st.get("stepper_enable", {}).get("steppers", {})
                self.state["theta"] = (
                    {"present": True, "pos": round(float(th["pos"]), 3),
                     "enabled": bool(en.get("manual_stepper theta", False))}
                    if "pos" in th else
                    {"present": False, "pos": 0.0, "enabled": False})
                wh = st.get("webhooks", {})
                self.state["klipper"] = wh.get("state", "?")
                self.state["message"] = (wh.get("state_message") or "").strip()
                self.state["online"] = True
            except Exception as e:           # keep polling through outages
                self.state["online"] = False
                self.state["klipper"] = "offline"
                self.state["message"] = f"Moonraker: {type(e).__name__}"
            time.sleep(max(0.02, POLL_S - (time.time() - t0)))

    # ------------------------------------------------------------------ #
    @property
    def ready(self):
        return self.state["online"] and self.state["klipper"] == "ready"

    @property
    def homed_xyz(self):
        return all(a in self.state["homed"] for a in "xyz")

    @property
    def work(self):
        return list(self.state["work"])

    @property
    def has_theta(self):
        return self.state["theta"]["present"]

    def gcode(self, script, timeout=30.0):
        """Run a g-code script -> (ok, message).

        Moonraker answers once Klipper has *processed* the script: a G1 is
        answered when queued, so append M400 to wait for motion to finish.
        """
        if self.state["online"] and self.state["klipper"] != "ready":
            first = self.state["message"].split("\n")[0]
            return False, f"Klipper {self.state['klipper']}: {first}"
        try:
            r = self._s.post(self.url + "/printer/gcode/script",
                             json={"script": script}, timeout=timeout)
        except requests.Timeout:
            return False, f"no reply from Klipper within {timeout:g} s"
        except requests.RequestException as e:
            return False, f"Moonraker unreachable ({type(e).__name__})"
        if r.status_code == 200:
            return True, "ok"
        try:
            msg = r.json()["error"]["message"]
        except (ValueError, KeyError, TypeError):
            msg = f"Moonraker HTTP {r.status_code}"
        m = re.search(r"'message':\s*'([^']+)'", msg)
        return False, (m.group(1) if m else msg)

    def wait_idle(self, timeout=600.0):
        return self.gcode("M400", timeout=timeout)

    def emergency_stop(self):
        return self._post("/printer/emergency_stop")

    def firmware_restart(self):
        return self._post("/printer/firmware_restart")

    def _post(self, path):
        try:
            r = self._s.post(self.url + path, timeout=5.0)
            return r.status_code == 200, f"HTTP {r.status_code}"
        except requests.RequestException as e:
            return False, f"Moonraker unreachable ({type(e).__name__})"
