#!/usr/bin/env python3
"""Stream the live NAU7802 load-cell reading to the terminal.

By default it polls the running fs_wheel app's /api/status (so nothing
else touches the I2C bus) and prints one line per poll: elapsed time,
raw ADC counts, grams, sample rate and a bar showing where the raw
count sits in the ADC's +/-2^23 range.

    python3 tools/live_adc.py                               # pi.local:5000
    python3 tools/live_adc.py --url http://10.42.0.165:5000 # by IP (cable)
    python3 tools/live_adc.py --rate 10 --csv adc.csv       # 10 Hz + log
    python3 tools/live_adc.py --direct                      # on the Pi,
                                      # with `systemctl --user stop fswheel`

Ctrl-C stops it and prints min / max / mean of what was seen.
"""

import argparse
import csv
import json
import sys
import time
import urllib.request

FULL_SCALE = 2 ** 23


def api_reader(url):
    def read():
        with urllib.request.urlopen(f"{url}/api/status", timeout=3) as r:
            lc = json.load(r)["loadcell"]
        if not lc["ok"]:
            raise RuntimeError(lc.get("error") or "load cell not ok")
        return lc["raw"], lc["grams"], lc["hz"]
    return read


def direct_reader():
    import qwiic_nau7802 as q
    s = q.QwiicNAU7802()
    if not s.is_connected():
        raise RuntimeError("NAU7802 not detected on I2C")
    s.begin()
    s.set_sample_rate(q.QwiicNAU7802.NAU7802_SPS_80)
    state = {"n": 0, "t": time.time(), "hz": 0.0, "raw": 0}

    def read():
        deadline = time.time() + 1.0
        while time.time() < deadline:
            if s.available():
                state["raw"] = s.get_reading()
                state["n"] += 1
                now = time.time()
                if now - state["t"] >= 1.0:
                    state["hz"] = state["n"] / (now - state["t"])
                    state["n"], state["t"] = 0, now
                return state["raw"], float("nan"), state["hz"]
            time.sleep(0.001)
        raise RuntimeError("no sample within 1 s")
    return read


def bar(raw, width=31):
    pos = int(round((raw + FULL_SCALE) / (2 * FULL_SCALE) * (width - 1)))
    pos = max(0, min(width - 1, pos))
    cells = ["-"] * width
    cells[width // 2] = "|"
    cells[pos] = "#"
    return "".join(cells)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--url", default="http://pi.local:5000",
                    help="fs_wheel app base URL (default %(default)s)")
    ap.add_argument("--rate", type=float, default=5.0,
                    help="polls per second (default %(default)s)")
    ap.add_argument("--direct", action="store_true",
                    help="read the NAU7802 over I2C directly (on the Pi, "
                         "with fswheel stopped)")
    ap.add_argument("--csv", help="append t,raw,grams,hz rows to this file")
    args = ap.parse_args()

    read = direct_reader() if args.direct else api_reader(args.url)
    out = open(args.csv, "a", newline="") if args.csv else None
    w = csv.writer(out) if out else None
    if w and out.tell() == 0:
        w.writerow(["t_s", "raw", "grams", "hz"])

    seen = []
    t0 = time.time()
    period = 1.0 / args.rate
    print(f"{'t(s)':>7} {'raw':>10} {'grams':>10} {'hz':>5}  "
          f"{'-FS':<15}0{'':<14}+FS")
    try:
        while True:
            t = time.time() - t0
            try:
                raw, grams, hz = read()
            except Exception as e:  # keep streaming through hiccups
                print(f"{t:7.1f}  ! {e}")
                time.sleep(period)
                continue
            seen.append(raw)
            railed = "  RAILED" if abs(raw) >= FULL_SCALE * 0.999 else ""
            print(f"{t:7.1f} {raw:10d} {grams:10.1f} {hz:5.1f}  "
                  f"{bar(raw)}{railed}", flush=True)
            if w:
                w.writerow([f"{t:.3f}", raw, f"{grams:.2f}", f"{hz:.1f}"])
                out.flush()
            time.sleep(max(0.0, period - (time.time() - t0 - t)))
    except KeyboardInterrupt:
        pass
    finally:
        if out:
            out.close()
    if seen:
        mean = sum(seen) / len(seen)
        print(f"\n{len(seen)} readings  min {min(seen)}  max {max(seen)}  "
              f"mean {mean:.0f}  span {max(seen) - min(seen)}")


if __name__ == "__main__":
    sys.exit(main())
