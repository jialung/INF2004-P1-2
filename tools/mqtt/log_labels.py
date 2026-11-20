"""
log_labels.py -- turn the car's console lines into labelled, filterable
entries for web_panel.py.

label_line(raw) -> list of (category, text).  One console line can give
several entries: "[line] idle: left 333 (white), right GP17 0 (white),
barcode 240, thr 2000" becomes three, one per sensor:

    Left line sensor:   333 (white)
    Right line sensor:  0 (white)  [GP17, digital]
    Barcode sensor:     240 (white; black above 2200, white below 1800)

The patterns follow the firmware's tm_printf formats (app_program/...).
The Supervisor's "[sup] bypass ..." lines go under "Ultrasonic / obstacle"
with the obstacle task's own "[obst]" lines, so one filter shows the
whole bypass.
A line no pattern knows is kept as it is, in the "other" category, so
nothing is ever hidden by accident.
"""

import json
import re

# (key, label in the filter bar).  Order = order of the checkboxes.
CATEGORIES = [
    ("left",       "Left line sensor"),
    ("right",      "Right line sensor"),
    ("barcode",    "Barcode sensor"),
    ("line",       "Line following / cross"),
    ("tilt",       "IMU tilt / hump / impact"),
    ("compass",    "Compass"),
    ("ultrasonic", "Ultrasonic / obstacle"),
    ("encoders",   "Encoders / motors"),
    ("state",      "Supervisor / state"),
    ("events",     "Map events"),
    ("net",        "Wi-Fi / MQTT"),
    ("diag",       "Diagnostics"),
    ("other",      "Other"),
]
LABEL = dict(CATEGORIES)
WIDTH = max(len(v) for v in LABEL.values()) + 2      # "Label:" column width

STATE_NAME = ["Init", "Idle", "Calibrate", "LineFollow", "Junction", "Hump",
              "Obstacle", "LineSearch", "Finished", "Fault"]

# app_program/fault_flags.h
FAULT_BITS = ["MOTOR_STALL", "ENC_FAULT", "IMU_SENSOR", "IMPACT", "US_FAULT",
              "OBST_STOP", "LINE_SEARCH_FAIL", "WIFI_DOWN", "INIT_FAIL",
              "NO_BYPASS", "PICKUP"]

BC_HYST = 200          # task_barcode.c: black above thr + 200, white below thr - 200

WIFI_LINK = {0: "down", 1: "joining", 2: "no IP yet", 3: "up",
             -1: "failed", -2: "no network", -3: "bad password"}


def fault_names(mask):
    if mask == 0:
        return "none"
    names = [n for i, n in enumerate(FAULT_BITS) if mask & (1 << i)]
    return ", ".join(names) or ("0x%x" % mask)


def state_name(n):
    return STATE_NAME[n] if 0 <= n < len(STATE_NAME) else "state %d" % n


def deg(q):
    """0.25 deg units -> '-2.50 deg'"""
    return "%+.2f deg" % (q / 4.0)


def entry(cat, text):
    return (cat, "%-*s %s" % (WIDTH, LABEL[cat] + ":", text))


# ---- patterns --------------------------------------------------------------
# "thr" = LEFT sensor threshold.  Firmware from 30 Oct 15:5x also prints
# "bc thr" (the barcode's own); older firmware does not, and then the
# barcode's threshold is unknown here.
R_LINE_IDLE = re.compile(r"\[line\] idle: left (\d+) \((\w+)\), right GP17 (\d+) \((\w+)\), "
                         r"barcode (\d+), thr (\d+)(?:, bc thr (\d+))?")
R_IMU_IDLE = re.compile(r"\[imu\] idle: heading (\d+) deg, roll (-?\d+), pitch (-?\d+) "
                        r"\(0\.25 deg\), compass (.*)")
R_TELEM = re.compile(r"\[telem\] st (\d+) flt 0x([0-9a-fA-F]+) odo (\d+) v (-?\d+) "
                     r"hdg (\d+) wifi (-?\d+) lost (\S+)")
R_SUP_MOVE = re.compile(r"\[sup\] (\d+) -> (\d+)$")
# task_barcode.c: "[bc] read 'X' (forward), 29 elements"
R_BC_READ = re.compile(r"\[bc\] read '([A-Z])' \((\w+)\), (\d+) elements")
# supervisor.c: "[sup] barcode 'X' stored -> left at the next junction"
R_BC_STORED = re.compile(r"\[sup\] barcode '([A-Z])' stored -> (\S+) at the next junction")
READ_MARK = "BARCODE READ:"          # the web panel highlights lines with this


def barcode_read(raw):
    """(letter, direction) if this console line is a successful barcode
    read, else None."""
    m = R_BC_READ.search(raw)
    return (m.group(1), m.group(2)) if m else None


# supervisor.c: the 50 deg side-step obstacle bypass.  "depth" is missing in
# firmware from before 12 Nov, the reason for the side before 20 Nov.
R_BYP_START = re.compile(r"\[sup\] bypass (LEFT|RIGHT): stopped at (\d+) cm, step 1 = (-?\d+) mm"
                         r"(?:, depth (\d+) mm)?(?: \(([^)]*)\))?")
# supervisor.c width_result (20 Nov): the side the next bypass takes
R_BYP_SIDE = re.compile(r"\[sup\] bypass side (LEFT|RIGHT)( \(default\))?: (.*)")
R_DEPTH_SET = re.compile(r"\[sup\] bypass depth set to (\d+) mm")
R_BYP_SCAN = re.compile(r"\[sup\] bypass: scan (\d+) at along (-?\d+) lat (-?\d+): "
                        r"((?:\d+ ){6}\d+) cm")
R_BYP_STEP2 = re.compile(r"\[sup\] bypass: edge (-?\d+) mm, step 2 = (-?\d+) mm")
R_BYP_PASS = re.compile(r"\[sup\] bypass: scan (\d+) clear, pass (-?\d+) mm")
R_BYP_EXTRA = re.compile(r"\[sup\] bypass: (.+), extra step (\d+) of (\d+)")
R_BYP_LINE = re.compile(r"\[sup\] bypass: line found")
R_BYP_NOLINE = re.compile(r"\[sup\] bypass: no line where expected")
R_BYP_DONE = re.compile(r"\[sup\] bypass: done")
R_BYP_FAIL = re.compile(r"\[sup\] bypass: (stopped too close to the obstacle to turn|"
                        r"still blocked after the extra steps|"
                        r"obstacle too close while passing|"
                        r"bypass took longer than 60 s)")
BYP_FAIL_WHY = {
    "stopped too close to the obstacle to turn":
        "it stopped too close to the obstacle to turn",
    "still blocked after the extra steps":
        "the side scan still saw the obstacle in the way after 2 extra steps",
    "obstacle too close while passing":
        "the obstacle was too close beside the car while passing, after 2 extra steps",
    "bypass took longer than 60 s":
        "it took longer than 60 s",
}


def bypass_depth(raw):
    """The box depth (mm) the car has, if this line says it, else None."""
    m = R_DEPTH_SET.search(raw)
    if m:
        return int(m.group(1))
    m = R_BYP_START.search(raw)
    return int(m.group(4)) if m and m.group(4) else None


def bypass_event(raw):
    """What a Supervisor bypass line says, or None.  (kind, side, text):
    kind 'start' (side 'left' / 'right'), 'phase', 'done', 'lost' (no line
    after the bypass), 'fail' (text = the reason), 'side' (the width sweep
    chose the next bypass side), 'depth' (PC depth applied), 'scan' (ranges
    only, not a new phase)."""
    m = R_BYP_START.search(raw)
    if m:
        return ("start", m.group(1).lower(),
                "step 1 out to the %s (%s mm), then side scan 1%s%s"
                % (m.group(1).lower(), m.group(3),
                   (" (box depth assumed %s mm)" % m.group(4)) if m.group(4) else "",
                   ("; " + m.group(5)) if m.group(5) else ""))
    m = R_BYP_SIDE.search(raw)
    if m:
        side, default, why = m.groups()
        return ("side", side.lower(), "next bypass on the %s%s: %s"
                % (side, " (default side)" if default else "", why))
    m = R_DEPTH_SET.search(raw)
    if m:
        return ("depth", None, "box depth for the bypass set to %s mm (from the next "
                "obstacle)" % m.group(1))
    m = R_BYP_SCAN.search(raw)
    if m:
        return ("scan", None, "side scan %s at %s mm along, %s mm out: %s cm (0, 5 ... 30 deg)"
                % (m.group(1), m.group(2), m.group(3), m.group(4)))
    m = R_BYP_STEP2.search(raw)
    if m:
        return ("phase", None, "obstacle edge %s mm from the line; step 2 out (%s mm), "
                "then side scan 2" % m.groups())
    m = R_BYP_PASS.search(raw)
    if m:
        return ("phase", None, "scan %s clear: passing the obstacle (%s mm), then "
                "turning back to the line" % m.groups())
    m = R_BYP_EXTRA.search(raw)
    if m:
        return ("phase", None, "%s: extra step %s of %s (80 mm out), then another side scan"
                % m.groups())
    if R_BYP_LINE.search(raw):
        return ("phase", None, "line found: rolling onto it, then turning back to the line heading")
    if R_BYP_NOLINE.search(raw):
        return ("lost", None, "no line where expected: turning back, then line search")
    if R_BYP_DONE.search(raw):
        return ("done", None, "bypass done: line following again")
    m = R_BYP_FAIL.search(raw)
    if m:
        return ("fail", None, BYP_FAIL_WHY[m.group(1)])
    return None
R_TAG = re.compile(r"\[([A-Za-z]+)\]\s*(.*)")

# tag -> category for lines without a special rule
TAG_CAT = {
    "bc": "barcode", "line": "line", "obst": "ultrasonic", "obstacle": "ultrasonic",
    "motion": "encoders", "sup": "state", "map": "events", "telem": "state",
    "diag": "diag", "watch": "diag", "pm": "diag", "tasks": "diag", "wcet": "diag",
    "TEST": "diag",
}


def label_line(raw):
    """One console line -> list of (category, labelled text)."""
    line = raw.strip()
    if not line:
        return []

    m = R_LINE_IDLE.search(line)
    if m:
        l, lcol, r, rcol, b, thr_l, thr_b = m.groups()
        b = int(b)
        if thr_b is None:
            btext = "%d  (old firmware: barcode threshold not reported)" % b
        else:
            t = int(thr_b)
            bcol = "black" if b > t + BC_HYST else ("white" if b < t - BC_HYST else "in between")
            btext = "%d (%s; black above %d, white below %d)" % (b, bcol, t + BC_HYST, t - BC_HYST)
        return [
            entry("left", "%s (%s; threshold %s)" % (l, lcol, thr_l)),
            entry("right", "%s (%s)  [GP17, digital: 0 white / 1 black]" % (r, rcol)),
            entry("barcode", btext),
        ]

    m = R_IMU_IDLE.search(line)
    if m:
        hdg, roll, pitch, comp = m.groups()
        cal = "NOT calibrated" if "not" in comp.lower() else "calibrated"
        wheel = re.search(r"right wheel (-?\d+\.\d) mm(?: \(2 s mean, range (-?\d+\.\d)\.\.(-?\d+\.\d)\))?", comp)
        acc = re.search(r"accel raw x (-?\d+) y (-?\d+) z (-?\d+) \(2 mg/count\), "
                        r"car fwd (-?\d+) left (-?\d+) up (-?\d+) mg, \|a\| (\d+) mg", comp)
        out = [
            entry("compass", "heading %s deg (%s)" % (hdg, cal)),
            entry("tilt", "roll %s, pitch %s  (car parked: both should be near 0)"
                  % (deg(int(roll)), deg(int(pitch)))),
        ]
        if acc:
            x, y, z, f, lf, u, mag = (int(g) for g in acc.groups())
            out.append(entry("tilt", "accelerometer raw  x %+5d  y %+5d  z %+5d  (2 mg/count)"
                             % (x, y, z)))
            out.append(entry("tilt", "accelerometer car  forward %+5d  left %+5d  up %+5d mg, "
                                     "total %d mg  (parked on the flat: total ~1000, up ~1000, "
                                     "forward and left ~0)" % (f, lf, u, mag)))
        if wheel:
            rng = (" (2 s mean, range %s..%s)" % (wheel.group(2), wheel.group(3))) if wheel.group(2) else ""
            out.append(entry("tilt", "right wheel height %s mm%s  (flat: near 0; "
                                     "check with a spacer under the right wheel)"
                             % (wheel.group(1), rng)))
        return out

    m = R_TELEM.search(line)
    if m:
        st, flt, odo, v, hdg, wifi, lost = m.groups()
        flt = int(flt, 16)
        return [
            entry("state", "%s, faults: %s" % (state_name(int(st)), fault_names(flt))),
            entry("encoders", "odometer %s mm, speed %s mm/s" % (odo, v)),
            entry("net", "Wi-Fi link %s (%s), telemetry lost %s"
                  % (wifi, WIFI_LINK.get(int(wifi), "?"), lost)),
        ]

    m = R_BC_READ.search(line)
    if m:
        letter, direction, n = m.groups()
        return [entry("barcode", "%s  %s   (read %s, %s elements)"
                      % (READ_MARK, letter, direction, n))]

    m = R_BC_STORED.search(line)
    if m:
        letter, move = m.groups()
        return [entry("state", "%s  %s stored: at the next junction -> %s"
                      % (READ_MARK, letter, move.upper()))]

    byp = bypass_event(line)
    if byp:
        kind, side, text = byp
        if kind == "start":
            m = R_BYP_START.search(line)
            text = "BYPASS %s: stopped at %s cm; %s" % (side.upper(), m.group(2), text)
        elif kind == "fail":
            text = "BYPASS FAILED: " + text
        return [entry("ultrasonic", text)]

    m = R_SUP_MOVE.search(line)
    if m:
        a, b = int(m.group(1)), int(m.group(2))
        return [entry("state", "%s -> %s" % (state_name(a), state_name(b)))]

    if line.startswith("[map] {"):
        try:
            ev = json.loads(line[6:])
        except ValueError:
            return [entry("events", line[6:])]
        kind = ev.pop("ev", "?")
        x, y = ev.pop("x", None), ev.pop("y", None)
        rest = ", ".join("%s %s" % (k, v) for k, v in ev.items())
        where = "" if x is None else "  at x %s, y %s mm" % (x, y)
        return [entry("events", "%s%s%s" % (kind, (": " + rest) if rest else "", where))]

    m = re.search(r"\[imu\] hump watch: right wheel now (-?\d+), min (-?\d+) max (-?\d+) "
                  r"\(0\.1 mm, last 1 s; trigger (\d+)\), vs calibration (-?\d+), (.+)$", line)
    if m:
        v = [int(g) / 10 for g in m.groups()[:5]]
        return [entry("tilt", "hump watch: right wheel %.1f mm (last 1 s: %.1f..%.1f, "
                              "hump above %.1f), vs calibration %.1f mm, %s"
                      % (v[0], v[1], v[2], v[3], v[4], m.group(6)))]

    if line.startswith("[imu]"):
        body = line[5:].strip()
        cat = "compass" if ("compass" in body and "hump" not in body) else "tilt"
        return [entry(cat, body)]

    if line.startswith("[line]"):
        return [entry("line", line[6:].strip())]

    if line.startswith(("mqtt:", "wifi:", "telnet:", "cyw43")):
        return [entry("net", line)]

    m = R_TAG.match(line)
    if m and m.group(1) in TAG_CAT:
        return [entry(TAG_CAT[m.group(1)], m.group(2))]

    return [entry("other", line)]
