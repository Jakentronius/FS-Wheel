"""NAU7802 load cell (SparkFun Qwiic, I2C) sampled at ~80 Hz.

v2: the 5 kg cell sits on the spindle, turned 90 deg, so it reads the
X-direction contact force. grams = (raw - zero) / counts_per_g, then an
exponential moving average (smooth_alpha) is what the controller uses.
Tare and calibration are manual only - by design there is no automatic
re-tare before an engage; we go with what the cell reads.
"""
import threading
import time

try:
    import qwiic_nau7802
    AVAILABLE = True
except ImportError:                      # dev machine without the lib
    AVAILABLE = False

RETRY_S = 5.0


class LoadCell:
    def __init__(self, cfg, save):
        self.cfg = cfg          # {"zero_counts", "counts_per_g", "smooth_alpha"}
        self._save = save
        self.raw = 0
        self.grams_raw = 0.0    # unsmoothed
        self.grams = 0.0        # smoothed; what control uses
        self.ok = False
        self.error = "not started"
        self.hz = 0.0
        self.seq = 0
        self._running = True

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def status(self):
        return {"ok": self.ok, "error": self.error, "raw": self.raw,
                "grams": round(self.grams, 1),
                "grams_raw": round(self.grams_raw, 1), "hz": round(self.hz, 1),
                "zero_counts": self.cfg["zero_counts"],
                "counts_per_g": self.cfg["counts_per_g"],
                "smooth_alpha": self.cfg["smooth_alpha"]}

    # ------------------------------------------------------------------ #
    def to_grams(self, raw):
        cpg = self.cfg["counts_per_g"] or 1.0
        return (raw - self.cfg["zero_counts"]) / cpg

    def _connect(self):
        if not AVAILABLE:
            self.error = "qwiic_nau7802 library not installed"
            return None
        s = qwiic_nau7802.QwiicNAU7802()
        if not s.is_connected():
            self.error = "NAU7802 not detected on I2C"
            return None
        s.begin()
        s.set_sample_rate(qwiic_nau7802.QwiicNAU7802.NAU7802_SPS_80)
        return s

    def _loop(self):
        while self._running:
            scale = self._connect()
            if scale is None:
                self.ok = False
                time.sleep(RETRY_S)
                continue
            self.ok, self.error = True, None
            n, t_win, t_last = 0, time.time(), time.time()
            while self._running:
                if scale.available():
                    raw = scale.get_reading()
                    g = self.to_grams(raw)
                    a = self.cfg["smooth_alpha"]
                    self.raw, self.grams_raw = raw, g
                    self.grams = g if self.seq == 0 else self.grams + a * (g - self.grams)
                    self.seq += 1
                    n += 1
                    t_last = time.time()
                    if t_last - t_win >= 1.0:
                        self.hz = n / (t_last - t_win)
                        n, t_win = 0, t_last
                else:
                    time.sleep(0.001)
                    if time.time() - t_last > 3.0:      # stalled: reconnect
                        self.ok, self.error = False, "no samples for 3 s"
                        break

    # ------------------------------------------------------------------ #
    def _avg_raw(self, n=40, timeout=2.0):
        vals, last, deadline = [], self.seq, time.time() + timeout
        while len(vals) < n and time.time() < deadline:
            if self.seq != last:
                last = self.seq
                vals.append(self.raw)
            else:
                time.sleep(0.002)
        if not vals:
            raise RuntimeError("no samples from the load cell")
        return sum(vals) / len(vals)

    def tare(self):
        try:
            self.cfg["zero_counts"] = self._avg_raw()
        except RuntimeError as e:
            return False, str(e)
        self._save()
        return True, f"zero = {self.cfg['zero_counts']:.0f} counts"

    def calibrate(self, known_g):
        """One-point span: the cell currently carries known_g."""
        try:
            known_g = float(known_g)
            if known_g == 0:
                raise ValueError
        except (TypeError, ValueError):
            return False, "need a non-zero known mass in grams"
        try:
            avg = self._avg_raw()
        except RuntimeError as e:
            return False, str(e)
        self.cfg["counts_per_g"] = (avg - self.cfg["zero_counts"]) / known_g
        self._save()
        return True, f"counts_per_g = {self.cfg['counts_per_g']:.3f}"
