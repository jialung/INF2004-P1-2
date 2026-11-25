"""
car_common.py -- what the web panel (web_panel.py) shares about the car:
MQTT topics, the telem frame decoder, "why did it stop" (explain_stop),
console line patterns and the command limits.  No user interface here.
(Split out of the old Tkinter window, mapping_ui.py, removed 19 Nov.)

Topics (prefix = MQTT_CLIENT_ID in config/wifi_secrets.h):
  cmd          PC -> car   "start" / "stop" / "mode" (MODE short press) /
                           "calibrate" (MODE held 2 s) /
                           "depth <mm>" / "wcal <left> <right>" (each edge
                           from the centre line, Idle only) / "esweep"
                           (measure only, Idle only) / "wcal clear"
                           (never retained).  The bypass side is not a
                           command: the car picks it from the width sweep.
  cmd/ack      car -> PC   "ok ..." / "refused ..." / "error ..."
  map, map/ack             mapping and the car's answer
  snapshot, snapshot/reply
  log          car -> PC   console text, whole lines
  event        car -> PC   JSON, used here for the state display
  heartbeat    car -> PC   "hb N" every 5 s; any car -> PC topic counts as
                           "heard from the car" in the panel
"""

import os
import re
import shutil
import subprocess

import log_labels as ll               # same folder: sensor labels and filters

# hump indicator: console lines from task_imu.c
R_HUMP_START = re.compile(r"\[imu\] hump start")
R_HUMP_END = re.compile(r"\[imu\] hump end: (\w+) wheel, flat top (\d+\.\d) mm "
                        r"\((\d+) samples, highest (\d+\.\d)\) -> (\d+) mm")
R_ACC_RAW = re.compile(r"accel raw x (-?\d+) y (-?\d+) z (-?\d+)")
# supervisor.c prints this when a STOP command arrives over MQTT
R_REMOTE_STOP = re.compile(r"\[sup\] remote STOP\s*$")
# supervisor.c, obstacle width: the car stops to measure, then prints the
# result ("obstacle width 152 mm ...", "... not measured: ...", "... no
# result in ...").  Only the finished width is shown in the panel.
R_WIDTH_BEGIN = re.compile(r"\[sup\] obstacle ahead at \d+ cm: stopping to measure")
R_WIDTH_END = re.compile(r"\[sup\] obstacle width\b")
R_WIDTH = re.compile(r"\[sup\] obstacle width (\d+) mm")
HUMP_SAME_S = 3.0                     # log line and JSON event of one hump arrive close together
# live right wheel height: Idle line (every 2 s) and "hump watch" (1 s, while following)
R_WHEEL_IDLE = re.compile(r"\[imu\] idle: .*right wheel (-?\d+\.\d) mm"
                          r"(?: \(2 s mean, range (-?\d+\.\d)\.\.(-?\d+\.\d)\))?")
R_HUMP_WATCH = re.compile(r"\[imu\] hump watch: right wheel now (-?\d+), min (-?\d+) "
                          r"max (-?\d+) \(0\.1 mm, last 1 s; trigger (\d+)\), "
                          r"vs calibration (-?\d+), ([A-Za-z ]+)")

# colour per log category (dark background)
CAT_COLOUR = {
    "left": "#7fd4ff", "right": "#4fa8ff", "barcode": "#ffd166",
    "line": "#9be89b", "tilt": "#f4a261", "compass": "#e9c46a",
    "ultrasonic": "#ff9f9f", "encoders": "#c3a6ff", "state": "#8fd18f",
    "events": "#80e0d0", "net": "#9a9a9a", "diag": "#7a7a7a", "other": "#dddddd",
}

CLIENT_ID = "mtk3smp-pico"            # MQTT_CLIENT_ID in config/wifi_secrets.h
TOPIC_MAP = CLIENT_ID + "/map"
TOPIC_ACK = TOPIC_MAP + "/ack"
TOPIC_SNAP = CLIENT_ID + "/snapshot"
TOPIC_SNAP_REPLY = TOPIC_SNAP + "/reply"
TOPIC_CMD = CLIENT_ID + "/cmd"
TOPIC_CMD_ACK = TOPIC_CMD + "/ack"
TOPIC_LOG = CLIENT_ID + "/log"
TOPIC_EVENT = CLIENT_ID + "/event"
TOPIC_HB = CLIENT_ID + "/heartbeat"
TOPIC_TELEM = CLIENT_ID + "/telem"    # 14-byte binary frame every 500 ms

# telem frame (app_program/subsystem5/telem_frame.h), little-endian
TF_LEN = 14
TF_TYPE_STATUS = 1
SPEED_LEVELS = ["stop", "slow", "hump", "cruise"]
RUNNING_STATES = {"line_follow", "junction", "hump", "obstacle", "line_search"}
TELEM_STALE_S = 2.0                   # no frame this long: speed shown grey


def crc8(data):
    """CRC-8, polynomial 0x07, init 0 (tf_crc8)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def decode_telem(raw):
    """tf_unpack(): the fields the UI uses, or None if the frame is bad."""
    if len(raw) != TF_LEN or crc8(raw[:TF_LEN - 1]) != raw[TF_LEN - 1]:
        return None
    if raw[0] & 0x7 != TF_TYPE_STATUS:
        return None
    w = raw[7] | (raw[8] << 8)
    return {
        "state": raw[0] >> 4,
        "faults": raw[1] | (raw[2] << 8),
        "odo_mm": raw[3] | (raw[4] << 8),
        "v_mm_s": int.from_bytes(raw[5:7], "little", signed=True),
        "speed_level": (w >> 12) & 0x3,
    }


# ---- "why did it stop": state + fault mask + speed -> a reason ----------
# State numbers in the telem frame = the SYS_STATE enum (msg_types.h), the
# same names task_telemetry.c puts in the JSON "state" events.
STATE_KEYS = ["init", "idle", "calibrate", "line_follow", "junction", "hump",
              "obstacle", "line_search", "finished", "fault"]
STATE_LABEL = {"line_follow": "Line following", "junction": "Junction turn",
               "hump": "Crossing a hump", "obstacle": "Obstacle bypass",
               "line_search": "Line search"}
V_MOVING = 5                          # mm/s: below this the car counts as still
START_GRACE_S = 2.0                   # after START, speed 0 is "starting", not "stopped"

# fault_flags.h: bits that put the car in Fault (WIFI_DOWN only reports,
# OBST_STOP only holds the car, so neither is a trip)
TRIP_MASK = sum(1 << i for i, n in enumerate(ll.FAULT_BITS)
                if n not in ("OBST_STOP", "WIFI_DOWN"))
OBST_BIT = 1 << ll.FAULT_BITS.index("OBST_STOP")

# fault name -> (short, what it means).  Conditions are from the firmware.
FAULT_WHY = {
    "MOTOR_STALL": "Both wheels driven hard but not turning: blocked, or no "
                   "motor power (battery?).",
    "ENC_FAULT": "One wheel driven hard but its encoder reads nothing: loose "
                 "encoder wire, or that wheel is jammed.",
    "IMU_SENSOR": "The IMU failed 3 reads in a row: check its I2C wiring.",
    "IMPACT": "The IMU measured a sudden knock above the limit twice in a row.",
    "US_FAULT": "No usable ultrasonic echo several times in a row: check "
                "the wiring.",
    "LINE_SEARCH_FAIL": "The line was lost and the search did not find it again.",
    "INIT_FAIL": "A start-up check failed (a driver did not initialise).",
    "NO_BYPASS": "The obstacle bypass gave up: it stopped too close to turn, "
                 "the side scan still saw the obstacle after 2 extra steps, the "
                 "obstacle was too close while passing, or it took longer than 60 s.",
    "PICKUP": "The car was picked up while running: the IMU saw it held tilted "
              "past 25 degrees for 0.3 s, or jerk upwards. Put it back on the track.",
}


def trip_names(faults):
    """Names of the fault bits that stop the car (not OBST_STOP / WIFI_DOWN)."""
    return [n for i, n in enumerate(ll.FAULT_BITS)
            if faults & (1 << i) and TRIP_MASK & (1 << i)]


def explain_stop(state, faults, v, idle_cause=None, fault_from=None,
                 state_age=None, bypass=None, bypass_fail=None, side=None,
                 measuring=False):
    """(kind, headline, detail).  kind: 'moving' | 'info' (deliberate, or not
    known yet) | 'stopped' | 'fault'.  idle_cause is a (headline, detail) the
    caller worked out for Idle, fault_from the running state a Fault
    interrupted.  v is None until a telem frame has arrived for the current
    state (the speed from before the change is stale), state_age is how many
    seconds the car has been in this state.  bypass = the current bypass
    phase from the car's log, bypass_fail = why the last bypass gave up,
    side = the side the car chose for this obstacle ('left' / 'right' / None),
    measuring = the car stopped to measure an obstacle's width."""
    if state is None:
        return ("info", "Waiting for the car", "No state received yet.")
    names = trip_names(faults)
    if names or state == "fault":
        head = ("FAULT: " + ", ".join(names)) if names else "FAULT"
        # explain the first (lowest) bit, the one the car's own
        # "[sup] FAULT:" line names; list the rest
        detail = FAULT_WHY.get(names[0], names[0]) if names else ""
        if names and names[0] == "NO_BYPASS" and bypass_fail:
            detail = "The obstacle bypass gave up: %s." % bypass_fail
        if len(names) > 1:
            detail += " Also: %s." % ", ".join(names[1:])
        if fault_from in STATE_LABEL:
            detail += " During: %s." % STATE_LABEL[fault_from].lower()
        detail += " Press MODE to clear."
        return ("fault", head, detail.strip())
    running = state in RUNNING_STATES
    if running and faults & OBST_BIT:
        if state == "obstacle":
            # only the emergency distance stops the car during a bypass
            return ("stopped", "Too close during the bypass",
                    "Something is closer than 6 cm ahead, so the car is held. It "
                    "moves on when the reading is above 26 cm again; the bypass "
                    "gives up after 60 s.")
        # the stop at 21 cm: the bypass starts at once
        return ("info", "Obstacle ahead",
                "The ultrasonic guard saw something 15 cm in front of the car. It "
                "goes round it %s." % (("on the %s" % side.upper()) if side else
                                       "on the side with more room (from the width "
                                       "sweep; LEFT if the box was centred or not "
                                       "measured)"))
    if state == "idle":
        head, detail = idle_cause or ("Idle", "Ready: press START.")
        return ("stopped", head, detail)
    if state == "finished":
        return ("stopped", "Finished", "The course is complete.")
    if state == "calibrate":
        return ("info", "Calibrating", "Holding still for the level reading, "
                "then spinning for the compass and line sensors.")
    if state == "init":
        return ("stopped", "Starting up", "The Supervisor has not left Init yet.")
    if running:
        if v is None:                  # state just changed: speed not known yet
            return ("info", STATE_LABEL[state],
                    "Waiting for the first speed reading.")
        if abs(v) >= V_MOVING:
            if state == "obstacle" and bypass:
                return ("moving", "Moving", "Obstacle bypass at %d mm/s: %s." % (v, bypass))
            return ("moving", "Moving", "%s at %d mm/s." % (STATE_LABEL[state], v))
        if measuring and state == "line_follow":
            return ("info", "Measuring obstacle width",
                    "Stopped about 40 cm from an obstacle while the servo sweeps "
                    "to find its two edges (about 6 s). The car then carries on "
                    "and goes round it on the side with more room.")
        if state == "junction":
            return ("info", "Junction: turning on the spot",
                    "Stopped over the crossing, pivoting to the new direction.")
        if state == "line_search":
            return ("info", "Searching for the line",
                    "The line was lost; the car sweeps left and right to find it.")
        if state == "obstacle":
            return ("info", "Obstacle bypass: pivoting or scanning",
                    (bypass[0].upper() + bypass[1:] + ".") if bypass else
                    "Pivoting 50 deg on the spot, or stopped for a side scan "
                    "(0 to 30 deg towards the obstacle).")
        if state_age is not None and state_age < START_GRACE_S:
            return ("info", "Starting", "Speeding up from rest.")
        return ("stopped", "Not moving",
                "Running, but the speed is 0: a wheel is not turning, or the "
                "car is held by something not reported here.")
    return ("info", "State: %s" % state, "")

MOVES = [("Forward", "forward"), ("Left", "left"),
         ("Right", "right"), ("U-turn", "uturn")]
DEFAULT = {"forward": "A", "left": "B", "right": "C", "uturn": "D"}
LETTERS = [chr(c) for c in range(ord("A"), ord("Z") + 1)]
# What the car does at a junction it reaches with NO barcode pending.  A
# barcode counts for ONE junction, so every junction after it uses this.
MOVE_WORDS = [w for _, w in MOVES]
DEFAULT_MOVE = "forward"              # what the car boots with

# Obstacle depth the bypass assumes, mm (subsystem5/bypass_geom.h:
# BG_DEPTH_DEFAULT_MM, BG_DEPTH_MIN_MM, BG_DEPTH_MAX_MM)
BYPASS_DEPTH_DEFAULT = 140
BYPASS_DEPTH_MIN = 10
BYPASS_DEPTH_MAX = 300
BYPASS_SIDE_DEFAULT = "left"          # subsystem5.h BYPASS_SIDE_DEFAULT

LOG_MAX_LINES = 5000                  # older lines are removed from the view
OFFLINE_AFTER_S = 12                  # heartbeat is every 5 s


def find_tool(name):
    """mosquitto_pub / mosquitto_sub: on PATH or in the default folders."""
    found = shutil.which(name)
    if found:
        return found
    for base in (r"C:\Program Files\mosquitto", r"C:\Program Files (x86)\mosquitto"):
        path = os.path.join(base, name + ".exe")
        if os.path.exists(path):
            return path
    return None


NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


def car_broker_ip():
    """The broker the car was built for (MQTT_BROKER_IP in
    config/wifi_secrets.h).  Using the same address as the car matters on
    a PC where the Mosquitto Windows service also runs: it listens on
    localhost:1883 only, so "localhost" can reach that broker instead of
    the one the car is connected to."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "config", "wifi_secrets.h")
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 3 and parts[0] == "#define" and parts[1] == "MQTT_BROKER_IP":
                    return parts[2].strip('"')
    except OSError:
        pass
    return "localhost"
