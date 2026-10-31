"""fs_wheel - barebones CNC-style control for the wheel friction surfacer.

    python app.py            -> http://<pi>:5000

One page: DRO + zero, home, jog, MDI, spindle rpm, force, and the
surfacing cycle (Cycle Start / Feed Hold / Stop / E-STOP / Reset).
"""
import json
import os
import sys

from flask import Flask, jsonify, request, send_from_directory

import cycle as cyc
from loadcell import LoadCell
from machine import Machine
from spindle import SpindleController

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config.json")
DEFAULT_CONFIG = {
    "loadcell": {"zero_counts": 0.0, "counts_per_g": 1.0, "smooth_alpha": 0.15},
    "control": dict(cyc.CONTROL),
    "cycle": dict(cyc.PARAMS),
    "jog_feed_mm_min": 600.0,
    "jog_theta_deg_s": 10.0,
}


def load_config():
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    try:
        with open(CONFIG_PATH) as f:
            saved = json.load(f)
        for k, v in saved.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    except (OSError, ValueError):
        pass
    return cfg


def save_config():
    tmp = CONFIG_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(config, f, indent=1)
    os.replace(tmp, CONFIG_PATH)


config = load_config()
machine = Machine()
loadcell = LoadCell(config["loadcell"], save_config)
spindle = SpindleController()
cycle = cyc.Cycle(machine, loadcell, spindle, config["control"])
cycle.params.update(config["cycle"])

app = Flask(__name__, static_folder=os.path.join(BASE_DIR, "static"))


def reply(ok, msg, code_fail=400):
    return jsonify({"ok": ok, "message": msg}), (200 if ok else code_fail)


def body():
    return request.get_json(force=True, silent=True) or {}


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.get("/api/status")
def api_status():
    return jsonify({
        "machine": machine.state,
        "loadcell": loadcell.status(),
        "spindle": spindle.status(),
        "cycle": cycle.status(),
        "control": config["control"],
        "jog_feed_mm_min": config["jog_feed_mm_min"],
        "jog_theta_deg_s": config["jog_theta_deg_s"],
    })


# --- motion -------------------------------------------------------------- #
@app.post("/api/gcode")
def api_gcode():
    """MDI line. Runs in the work frame as typed; add M400 yourself if
    you want the call to wait for motion."""
    if cycle.running:
        return reply(False, "cycle running")
    line = str(body().get("line", "")).strip()
    if not line:
        return reply(False, "empty line")
    return reply(*machine.gcode(line, timeout=120.0))


@app.post("/api/jog")
def api_jog():
    if cycle.running:
        return reply(False, "cycle running")
    d = body()
    axis = str(d.get("axis", "")).upper()
    try:
        dist = float(d.get("dist", 0))
        feed = float(d.get("feed", config["jog_feed_mm_min"]))
    except (TypeError, ValueError):
        return reply(False, "bad jog values")
    if axis not in ("X", "Y", "Z", "T") or dist == 0:
        return reply(False, "bad jog")
    if axis == "T":                     # theta: degrees at deg/s via THETA_JOG macro
        if not machine.has_theta:
            return reply(False, "theta axis not configured in Klipper")
        config["jog_theta_deg_s"] = feed
        return reply(*machine.gcode(f"THETA_JOG D={dist:.4f} F={feed:g}"))
    config["jog_feed_mm_min"] = feed
    return reply(*machine.gcode(f"G91\nG1 {axis}{dist:.4f} F{feed:g}\nG90"))


@app.post("/api/home")
def api_home():
    if cycle.running:
        return reply(False, "cycle running")
    axes = "".join(a for a in str(body().get("axes", "")).upper() if a in "XYZ")
    return reply(*machine.gcode(f"G28 {' '.join(axes)}".strip(), timeout=300.0))


@app.post("/api/zero")
def api_zero():
    """Mach3 'Zero X': make the current position 0 on that axis (G92)."""
    if cycle.running:
        return reply(False, "cycle running")
    axes = "".join(a for a in str(body().get("axes", "")).upper() if a in "XYZT")
    if not axes:
        return reply(False, "no axis")
    if "T" in axes:                     # theta has its own position register
        if not machine.has_theta:
            return reply(False, "theta axis not configured in Klipper")
        ok, msg = machine.gcode("THETA_ZERO")
        if not ok or axes == "T":
            return reply(ok, msg)
        axes = axes.replace("T", "")
    return reply(*machine.gcode("G92 " + " ".join(f"{a}0" for a in axes)))


@app.post("/api/estop")
def api_estop():
    cycle.stop()
    return reply(*machine.emergency_stop())


@app.post("/api/firmware_restart")
def api_firmware_restart():
    if cycle.running:
        return reply(False, "cycle running")
    cycle.reset()
    return reply(*machine.firmware_restart())


# --- spindle -------------------------------------------------------------- #
@app.post("/api/spindle")
def api_spindle():
    d = body()
    if "rpm" in d:
        return reply(*spindle.set_rpm(d["rpm"]))
    if "hz" in d:
        return reply(*spindle.set_hz(d["hz"]))
    return reply(*spindle.stop())


# --- load cell ------------------------------------------------------------ #
@app.post("/api/loadcell/tare")
def api_tare():
    return reply(*loadcell.tare())


@app.post("/api/loadcell/reconnect")
def api_loadcell_reconnect():
    return reply(*loadcell.reconnect())


@app.post("/api/loadcell/calibrate")
def api_calibrate():
    return reply(*loadcell.calibrate(body().get("grams")))


@app.post("/api/loadcell/config")
def api_loadcell_config():
    d = body()
    try:
        for k in ("zero_counts", "counts_per_g", "smooth_alpha"):
            if k in d:
                config["loadcell"][k] = float(d[k])
    except (TypeError, ValueError):
        return reply(False, "bad value")
    save_config()
    return reply(True, "saved")


# --- cycle ---------------------------------------------------------------- #
@app.post("/api/cycle/start")
def api_cycle_start():
    params = body().get("params") or {}
    ok, msg = cycle.start(params)
    if ok:
        config["cycle"] = dict(cycle.params)
        save_config()
    return reply(ok, msg)


@app.post("/api/cycle/hold")
def api_cycle_hold():
    return reply(*cycle.hold())


@app.post("/api/cycle/resume")
def api_cycle_resume():
    return reply(*cycle.resume())


@app.post("/api/cycle/stop")
def api_cycle_stop():
    return reply(*cycle.stop())


@app.post("/api/cycle/reset")
def api_cycle_reset():
    return reply(*cycle.reset())


@app.post("/api/cycle/engage")
def api_cycle_engage():
    return reply(*cycle.engage_only(body().get("params") or {}))


@app.post("/api/cycle/retract")
def api_cycle_retract():
    return reply(*cycle.retract_only())


@app.post("/api/control")
def api_control():
    d = body()
    try:
        for k in cyc.CONTROL:
            if k in d:
                config["control"][k] = float(d[k])
    except (TypeError, ValueError):
        return reply(False, "bad value")
    save_config()
    return reply(True, "saved")


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5000
    machine.start()
    loadcell.start()
    spindle.start()
    print(f"[*] fs_wheel: http://0.0.0.0:{port}")
    app.run(host="0.0.0.0", port=port, threaded=True)
