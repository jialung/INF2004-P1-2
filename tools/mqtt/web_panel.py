#!/usr/bin/env python3
"""
web_panel.py -- browser control panel for the robot car over MQTT.

The car's control panel, as a web page:

    python web_panel.py          (or double-click web_panel.bat)

It opens http://127.0.0.1:8765 in the browser.  Leave the console window
open while you use the panel: closing it stops the panel.

Needs only Python 3 and Mosquitto's command-line tools.
This program runs mosquitto_sub / mosquitto_pub; the browser only talks to
this program (Server-Sent Events for updates, small JSON POSTs for
commands).  Console lines are read with log_labels.py, width sweeps with
edge_cal.EdgeData, and the "why stopped" text comes from
car_common.explain_stop.  All state
is kept here, so reloading the page or opening a second tab loses nothing.

Who may open the page: this PC, and devices on the car's network (the
/24 around MQTT_BROKER_IP, e.g. 192.168.137.x on the Windows hotspot).
Anyone allowed can drive the car.  Other networks the PC is on (e.g.
SIT-WIFI) are refused.  --allow 192.168.1.0/24 (repeatable) replaces the
car's network, --allow any lets everyone in, --local-only is this PC only.
"""

import argparse
import collections
import datetime
import ipaddress
import json
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import edge_cal                     # noqa: E402  width sweep / calibration parser
import log_labels as ll             # noqa: E402  console line labels
import car_common as mu             # noqa: E402  topics, telem decoder, explain_stop

WEB_DIR = os.path.join(HERE, "web")
STATIC = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/app.css": ("app.css", "text/css; charset=utf-8"),
}
DEFAULT_PORT = 8765
TICK_S = 0.25                       # log lines and status go out at most this often
STATUS_EVERY_S = 1.0                # status also goes out this often (ages, offline)
RETRY_S = 5.0                       # restart a stopped listener after this long
CONNECTED_AFTER_S = 1.5             # mosquitto_sub exits at once if it cannot connect
REPLIES_KEPT = 60
MAX_BODY = 4096

TOPICS_IN = (mu.TOPIC_ACK, mu.TOPIC_SNAP_REPLY, mu.TOPIC_CMD_ACK, mu.TOPIC_LOG,
             mu.TOPIC_EVENT, mu.TOPIC_HB, mu.TOPIC_TELEM)

# commands the panel may send on the cmd topic (remote_cmd.h)
FIXED_CMDS = {"start", "stop", "mode", "calibrate", "esweep", "wcal clear"}
R_BROKER = re.compile(r"[A-Za-z0-9][A-Za-z0-9.\-]*(:\d{1,5})?")


def hms():
    return time.strftime("%H:%M:%S")


def check_cmd(word):
    """None if 'word' is a command the car knows, with values in range;
    otherwise the reason it is not sent."""
    if word in FIXED_CMDS:
        return None
    m = re.fullmatch(r"depth (\d{1,4})", word)
    if m:
        lo, hi = mu.BYPASS_DEPTH_MIN, mu.BYPASS_DEPTH_MAX
        return None if lo <= int(m.group(1)) <= hi else \
            "Box depth must be %d to %d mm." % (lo, hi)
    m = re.fullmatch(r"wcal (-?\d{1,4}) (-?\d{1,4})", word)
    if m:
        left, right = int(m.group(1)), int(m.group(2))
        lo, hi = edge_cal.EDGE_MIN, edge_cal.EDGE_MAX
        if not (lo <= left <= hi and lo <= right <= hi):
            return "Each edge must be %d to %d mm." % (lo, hi)
        if not edge_cal.WIDTH_MIN <= left + right <= edge_cal.WIDTH_MAX:
            return ("Left + right is the box width: it must be %d to %d mm."
                    % (edge_cal.WIDTH_MIN, edge_cal.WIDTH_MAX))
        return None
    return "Unknown command."


def reply_tone(text):
    low = text.lower()
    if any(w in low for w in ("refused", "error", "failed", "could not",
                              "not found", "stopped:")):
        return "bad"
    if low.startswith(("sent", "car: ok", "car (map): ok", "snapshot")):
        return "ok"
    return "info"


def config():
    """Fixed values the page needs (sent once per connection)."""
    return {
        "moves": mu.MOVES,
        "letters": mu.LETTERS,
        "move_words": mu.MOVE_WORDS,
        "default_map": mu.DEFAULT,
        "default_move": mu.DEFAULT_MOVE,
        "categories": ll.CATEGORIES,
        "colours": mu.CAT_COLOUR,
        "log_max": mu.LOG_MAX_LINES,
        "limits": {
            "depth": [mu.BYPASS_DEPTH_MIN, mu.BYPASS_DEPTH_MAX, mu.BYPASS_DEPTH_DEFAULT],
            "edge": [edge_cal.EDGE_MIN, edge_cal.EDGE_MAX],
            "box": [edge_cal.WIDTH_MIN, edge_cal.WIDTH_MAX],
            "spread": edge_cal.SPREAD_MM,
        },
        "edge_geom": {"pivot": edge_cal.PIVOT_MM, "side_max": edge_cal.SIDE_MAX_DEG},
    }


def entry_body(cat, text):
    """log_labels pads every entry with its category label ("Left line
    sensor:    333 ..."); the page shows the category as a tag instead."""
    head = ll.LABEL.get(cat, "") + ":"
    return text[len(head):].strip() if text.startswith(head) else text


def sse(kind, data):
    return "event: %s\ndata: %s\n\n" % (kind, json.dumps(data, separators=(",", ":")))


class Hub:
    """The open browser connections, one queue each."""

    def __init__(self):
        self.lock = threading.Lock()
        self.clients = set()

    def add(self):
        q = queue.Queue(maxsize=2000)
        with self.lock:
            self.clients.add(q)
        return q

    def remove(self, q):
        with self.lock:
            self.clients.discard(q)

    def send(self, kind, data):
        msg = sse(kind, data)
        with self.lock:
            for q in list(self.clients):
                try:
                    q.put_nowait(msg)
                except queue.Full:           # a stuck tab: drop it, it reconnects
                    self.clients.discard(q)
                    try:
                        q.put_nowait(None)
                    except queue.Full:
                        pass


class Car:
    """Everything known about the car, worked out from MQTT.  status() is what the page shows."""

    def __init__(self, hub, broker):
        self.hub = hub
        self.lock = threading.RLock()
        self.pub = mu.find_tool("mosquitto_pub")
        self.sub = mu.find_tool("mosquitto_sub")
        self.broker_text = broker
        self.sub_proc = None
        self.sub_started = None
        self.retry_at = None
        self.conn_tone = None
        self.dirty = True
        self.replies = collections.deque(maxlen=REPLIES_KEPT)
        self.reply_n = 0
        self.records = collections.deque(maxlen=mu.LOG_MAX_LINES)
        self.rec_n = 0
        self.pending_log = []
        self.log_partial = ""
        self.last_heard = None
        # obstacle bypass and width
        self.byp_phase = None
        self.byp_fail = None
        self.depth_sent = None
        self.measuring = False
        self.edge = edge_cal.EdgeData()
        # the side the car chose (width sweep) or used (bypass start)
        self.side = {"side": None, "tone": "wait",
                     "note": "no obstacle measured yet: the car picks the side at "
                             "the next one"}
        self.depth = {"mm": None, "tone": "wait",
                      "note": "not confirmed: the car starts with %d mm; press Set "
                              "or Snapshot" % mu.BYPASS_DEPTH_DEFAULT}
        self.mapping = dict(mu.DEFAULT, default=mu.DEFAULT_MOVE)
        self.map_note = {"text": "not sent from this panel yet", "tone": "wait"}
        self.wheel_now = "waiting for the IMU"
        self.acc_raw = "waiting for the IMU"
        self.clear_readings()
        # speed and why-stopped
        self.telem_last = None
        self.telem_prev = None
        self.car_running = False
        self.cur_state = None
        self.state_t = None
        self.cur_faults = 0
        self.cur_v = None
        self.idle_prev = None
        self.idle_t = None
        self.fault_from = None
        self.ui_stop_t = None
        self.remote_stop_t = None
        self.why_key = None
        self.why_since = ""
        self.last_stop = None
        self.spd = None                  # (v, level, time) of the last frame
        self.reset_speed_stats()
        if not (self.pub and self.sub):
            self.reply("Mosquitto tools not found. Install Mosquitto (it includes "
                       "mosquitto_pub.exe and mosquitto_sub.exe), then restart the panel.")

    # ---- small helpers ---------------------------------------------------
    def reply(self, text):
        with self.lock:
            self.reply_n += 1
            text = text.rstrip()
            self.replies.append({"id": self.reply_n, "t": hms(), "text": text,
                                 "tone": reply_tone(text)})
            self.dirty = True

    def clear_readings(self):
        """Barcode, hump and obstacle width boxes only; nothing is sent."""
        self.last_bc_letter = None
        self.bc = {"letter": None, "detail": "none read yet", "at": ""}
        self.hump_count = 0
        self.hump_max = None
        self.hump_last = None
        self.hump = {"text": None, "info": "none yet", "at": "", "climbing": False}
        self.obst_w = None
        self.dirty = True

    def reset_speed_stats(self):
        """Max / average / distance are per run (from START)."""
        self.run_dist_mm = 0
        self.run_time_s = 0.0
        self.run_max = 0

    # ---- MQTT ------------------------------------------------------------
    def broker_args(self):
        text = self.broker_text.strip() or "localhost"
        host, sep, port = text.rpartition(":")
        if sep and port.isdigit():
            return ["-h", host, "-p", port]
        return ["-h", text]

    def listen(self, quiet=False):
        """(Re)start mosquitto_sub.  Payloads come as hex so a log chunk
        with several lines stays one message."""
        with self.lock:
            if not self.sub:
                return
            if self.sub_proc:
                old, self.sub_proc = self.sub_proc, None
                old.kill()
            cmd = [self.sub] + self.broker_args() + ["-F", "%t %x"]
            for t in TOPICS_IN:
                cmd += ["-t", t]
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True,
                                        creationflags=mu.NO_WINDOW)
            except OSError as err:
                self.reply("Could not start mosquitto_sub: %s" % err)
                self.retry_at = time.time() + RETRY_S
                return
            self.sub_proc = proc
            self.sub_started = time.time()
            self.retry_at = None
            self.dirty = True
            if not quiet:
                self.reply("Listening on %s ..." % self.broker_text)

        def reader():
            for line in proc.stdout:
                self.handle(line.rstrip("\r\n"), quiet)
            with self.lock:
                if proc is self.sub_proc:
                    self.sub_proc = None
                    self.retry_at = time.time() + RETRY_S
                    self.dirty = True
                    if not quiet:
                        self.reply("Listener stopped: is Mosquitto running with "
                                   "mosquitto_robot.conf? Retrying every %d s."
                                   % RETRY_S)

        threading.Thread(target=reader, daemon=True).start()

    def want_retry(self, now):
        return (self.sub is not None and self.sub_proc is None
                and self.retry_at is not None and now >= self.retry_at)

    def publish(self, topic, payload, retain=False, done=None):
        """mosquitto_pub in a worker thread.  payload None = empty message."""
        if not self.pub:
            self.reply("Send failed: mosquitto_pub not found.")
            return
        cmd = [self.pub] + self.broker_args() + ["-t", topic, "-q", "1"]
        cmd += ["-n"] if payload is None else ["-m", payload]
        if retain:
            cmd.append("-r")

        def work():
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=10,
                                   creationflags=mu.NO_WINDOW)
                ok = r.returncode == 0
                err = (r.stderr or r.stdout).strip()
            except (OSError, subprocess.TimeoutExpired) as e:
                ok, err = False, str(e)
            if not ok:
                self.reply("Send failed: " + (err or "no broker?"))
            elif done:
                self.reply(done)

        threading.Thread(target=work, daemon=True).start()

    def handle(self, line, quiet=False):
        """One line from mosquitto_sub: '<topic> <hex payload>'.  quiet:
        an automatic retry, whose connection errors were reported already."""
        topic, _, hexdata = line.partition(" ")
        try:
            raw = bytes.fromhex(hexdata)
        except ValueError:
            if not quiet:
                self.reply(line)              # e.g. "Error: Connection refused"
            return
        payload = raw.decode("utf-8", "replace")
        with self.lock:
            self.dirty = True
            if topic in TOPICS_IN:            # every one of them comes from the car
                self.last_heard = time.time()
            if topic == mu.TOPIC_TELEM:
                self.on_telem(raw)
            elif topic == mu.TOPIC_LOG:
                self.on_log(payload)
            elif topic == mu.TOPIC_EVENT:
                self.on_event(payload)
            elif topic == mu.TOPIC_CMD_ACK:
                self.reply("car: " + payload)
                self.on_cmd_ack(payload)
            elif topic == mu.TOPIC_ACK:
                self.reply("car (map): " + payload)
                self.map_note = {"text": "car: " + payload + "  " + hms(),
                                 "tone": "bad" if reply_tone(payload) == "bad" else "ok"}
            elif topic == mu.TOPIC_SNAP_REPLY:
                self.reply("snapshot: " + payload)
                self.on_snapshot(payload)

    def on_event(self, payload):
        try:
            ev = json.loads(payload)
        except ValueError:
            return
        if not isinstance(ev, dict):
            return
        kind = ev.get("ev")
        if kind == "state":
            self.on_state(str(ev.get("state", "?")))
        elif kind == "barcode" and ev.get("letter"):
            if ev["letter"] != self.last_bc_letter:   # the log may have shown it
                self.show_barcode(str(ev["letter"]), "read (event)")
        elif kind == "hump" and isinstance(ev.get("height_mm"), int):
            self.show_hump(ev["height_mm"])
        elif kind == "obstacle" and isinstance(ev.get("width_mm"), int) \
                and ev["width_mm"] > 0:
            self.obst_w = ev["width_mm"]

    def on_snapshot(self, payload):
        try:
            snap = json.loads(payload)
        except ValueError:
            return
        if not isinstance(snap, dict):
            return
        if snap.get("state"):
            self.on_state(str(snap["state"]))
        if snap.get("bypass_side") in ("left", "right") and self.side["side"] is None:
            # the snapshot cannot say why: only fill in an unknown side
            self.side_seen(snap["bypass_side"], "next bypass, from the snapshot", "wait")
        if isinstance(snap.get("depth_mm"), int):
            self.depth_confirmed(snap["depth_mm"], "snapshot")
        w = snap.get("obstacle_width_mm")           # 0 = none measured yet
        if isinstance(w, int) and w > 0:
            self.obst_w = w

    # ---- console log -----------------------------------------------------
    def on_log(self, chunk):
        """Console text; only whole lines are kept."""
        text = self.log_partial + chunk
        lines = text.split("\n")
        self.log_partial = lines.pop()
        now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        for line in lines:
            line = line.rstrip("\r")
            if not line.strip():
                continue
            self.rec_n += 1
            rec = {"id": self.rec_n, "t": now, "raw": line,
                   "e": [[c, entry_body(c, t)] for c, t in ll.label_line(line)],
                   "hb": line.startswith("mqtt: ") and ("[" + mu.TOPIC_HB + "]" in line)}
            self.records.append(rec)
            self.pending_log.append(rec)
            self.read_line(line)

    def read_line(self, line):
        """What one console line changes in the indicators."""
        if mu.R_REMOTE_STOP.search(line):
            self.remote_stop_t = time.time()
        if mu.R_WIDTH_BEGIN.search(line):
            self.measuring = True
            self.obst_w = None                # the last width is not this obstacle's
        elif mu.R_WIDTH_END.search(line):
            self.measuring = False
            m = mu.R_WIDTH.search(line)
            if m:
                self.obst_w = int(m.group(1))
        self.on_bypass_line(line)
        self.edge.feed(line)
        got = ll.barcode_read(line)
        if got:
            self.show_barcode(got[0], "read " + got[1])
        m = mu.R_WHEEL_IDLE.search(line)
        if m:
            rng = (", range %s..%s" % (m.group(2), m.group(3))) if m.group(2) else ""
            self.wheel_now = ("%s mm%s (parked, 2 s mean vs calibration)  %s"
                              % (m.group(1), rng, hms()))
        m = mu.R_ACC_RAW.search(line)
        if m:
            self.acc_raw = ("x %+5d  y %+5d  z %+5d  (2 mg/count)"
                            % tuple(int(g) for g in m.groups()))
        m = mu.R_HUMP_WATCH.search(line)
        if m:
            now_, lo, hi, trig, _cal, st = m.groups()
            self.wheel_now = ("%.1f mm, last 1 s %.1f..%.1f (trigger %.1f) %s  %s"
                              % (int(now_) / 10, int(lo) / 10, int(hi) / 10,
                                 int(trig) / 10, st.upper(), hms()))
        if mu.R_HUMP_START.search(line):
            self.hump = {"text": "...", "info": "on a hump now",
                         "at": "since " + hms(), "climbing": True}
        m = mu.R_HUMP_END.search(line)
        if m:
            self.show_hump(int(m.group(5)), exact=float(m.group(2)), side=m.group(1))
        m = ll.R_BC_STORED.search(line)
        if m and m.group(1) == self.last_bc_letter:
            self.show_barcode(m.group(1), "-> " + m.group(2).upper()
                              + " at the next junction")

    def show_barcode(self, letter, detail):
        self.last_bc_letter = letter
        self.bc = {"letter": letter, "detail": detail, "at": "at " + hms()}

    def show_hump(self, height_mm, exact=None, side=None):
        """One finished hump.  The console line and the JSON event both
        report it; the second within HUMP_SAME_S is ignored."""
        now = time.time()
        if (self.hump_last and self.hump_last[0] == height_mm
                and now - self.hump_last[1] < mu.HUMP_SAME_S):
            return
        self.hump_last = (height_mm, now)
        self.hump_count += 1
        value = exact if exact is not None else float(height_mm)
        if self.hump_max is None or value > self.hump_max:
            self.hump_max = value
        if height_mm >= 31:                   # 5-bit field full: may be higher
            text = "31+ mm"
        elif exact is not None:
            text = "%.1f mm" % exact
        else:
            text = "%d mm" % height_mm
        info = "#%d, highest %.1f mm" % (self.hump_count, self.hump_max)
        if side:
            info += ", %s wheel" % side
        self.hump = {"text": text, "info": info, "at": "at " + hms(), "climbing": False}

    # ---- obstacle bypass side and depth ----------------------------------
    def side_seen(self, side, why, tone="ok"):
        self.side = {"side": side, "note": "%s  %s" % (why, hms()), "tone": tone}

    def depth_confirmed(self, mm, how):
        if self.depth_sent == mm:
            self.depth_sent = None
        self.depth = {"mm": mm, "note": "%s %s" % (how, hms()), "tone": "ok"}

    def on_cmd_ack(self, payload):
        """'ok depth 140 mm ...' = queued; the car's own log line (with the
        beep) confirms it applied."""
        m = re.match(r"ok depth (\d+) mm", payload)
        if m:
            if self.depth_sent == int(m.group(1)):
                self.depth.update(note="car accepted %s mm, waiting for the beep"
                                  % m.group(1), tone="wait")
            return
        if payload.startswith("error: depth") and self.depth_sent is not None:
            self.depth_sent = None
            self.depth.update(note="refused: " + payload[7:], tone="bad")
            return
        if payload.startswith("error: unknown command"):
            if self.depth_sent is not None and "depth" not in payload:
                self.depth_sent = None
                self.depth = {"mm": None, "tone": "bad",
                              "note": "this car's firmware has no depth command: "
                                      "flash the new image"}

    def on_bypass_line(self, line):
        got = ll.bypass_event(line)
        if not got:
            return
        kind, side, text = got
        mm = ll.bypass_depth(line)
        if mm is not None:
            self.depth_confirmed(mm, "used at the obstacle" if kind == "start"
                                 else "set on the car")
        if kind == "start":
            self.byp_fail = None
            self.byp_phase = text
            m = ll.R_BYP_START.search(line)
            why = m.group(5) if m and m.group(5) else "used at the obstacle"
            self.side_seen(side, "bypassing now: " + why)
        elif kind in ("phase", "lost", "done"):
            self.byp_phase = text
        elif kind == "fail":
            self.byp_fail = text
            self.byp_phase = None
        elif kind == "side":
            self.side_seen(side, text[0].upper() + text[1:])

    # ---- state, speed, why stopped ---------------------------------------
    def on_state(self, name):
        prev = self.cur_state
        if name != prev:
            if name == "idle":
                self.idle_prev, self.idle_t = prev, time.time()
            if name == "fault":
                self.fault_from = prev if prev in mu.RUNNING_STATES else None
            self.cur_state = name
            self.state_t = time.time()
            self.cur_v = None                 # the old state's speed is stale
            if name != "obstacle":
                self.byp_phase = None
            if name != "line_follow":
                self.measuring = False
        running = name in mu.RUNNING_STATES
        if running and not self.car_running:
            self.reset_speed_stats()          # a new run started
        self.car_running = running

    def on_telem(self, raw):
        f = mu.decode_telem(raw)
        if f is None:
            return
        keys = mu.STATE_KEYS
        name = keys[f["state"]] if f["state"] < len(keys) else None
        if name and name != self.cur_state:  # a state event was missed
            self.on_state(name)
        now = time.time()
        if self.telem_prev is not None and self.car_running:
            d = (f["odo_mm"] - self.telem_prev[0]) & 0xFFFF
            if d >= 0x8000:
                d -= 0x10000
            dt = now - self.telem_prev[1]
            if dt < 2.0:                      # skip gaps (Wi-Fi drop)
                self.run_dist_mm += abs(d)
                self.run_time_s += dt
        self.telem_prev = (f["odo_mm"], now)
        self.telem_last = now
        v = f["v_mm_s"]
        if self.car_running:
            self.run_max = max(self.run_max, abs(v))
        self.spd = (v, mu.SPEED_LEVELS[f["speed_level"]], hms())
        self.cur_faults = f["faults"]
        self.cur_v = v

    def idle_cause(self):
        """(headline, detail) for why the car is in Idle."""
        prev, t = self.idle_prev, self.idle_t
        if t is None:
            return None

        def near(x):
            return x is not None and abs(x - t) < 4.0

        if prev in mu.RUNNING_STATES or prev == "finished":
            if near(self.ui_stop_t):
                return ("STOP pressed in the panel",
                        "STOP (or Esc) was pressed in this panel and the car stopped.")
            if near(self.remote_stop_t):
                return ("STOP sent over MQTT",
                        "A STOP command from another client reached the car.")
            return ("STOP at the car (button)",
                    "No STOP command was seen from the network, so the START "
                    "button on the car was pressed while it was running.")
        if prev == "calibrate":
            return ("Calibration ended",
                    "Calibration finished (or MODE aborted it); the car is back in Idle.")
        if prev == "fault":
            return ("Fault cleared", "MODE cleared the fault; the car is in Idle "
                    "and can be started.")
        return None

    def why_now(self):
        cause = self.idle_cause() if self.cur_state == "idle" else None
        age = None if self.state_t is None else time.time() - self.state_t
        kind, head, detail = mu.explain_stop(
            self.cur_state, self.cur_faults, self.cur_v, cause, self.fault_from,
            age, self.byp_phase, self.byp_fail, self.side["side"], self.measuring)
        key = (kind, self.cur_state, self.cur_faults & (mu.TRIP_MASK | mu.OBST_BIT))
        if key != self.why_key:
            self.why_key = key
            self.why_since = hms()
        if kind in ("stopped", "fault"):
            self.last_stop = "%s (since %s)" % (head, self.why_since)
        last = "" if kind in ("stopped", "fault") or not self.last_stop \
            else "Last stop: " + self.last_stop
        return {"kind": kind, "head": head, "detail": detail, "last": last,
                "since": self.why_since, "stale": None}

    # ---- what the page shows ---------------------------------------------
    def status(self):
        with self.lock:
            now = time.time()
            alive = self.sub_proc is not None and self.sub_proc.poll() is None
            if not (self.pub and self.sub):
                conn = {"tone": "bad", "text": "Mosquitto tools not found"}
            elif alive and now - self.sub_started >= CONNECTED_AFTER_S:
                conn = {"tone": "ok", "text": "Broker " + self.broker_text}
            elif alive:
                conn = {"tone": "wait", "text": "Connecting to " + self.broker_text}
            else:
                conn = {"tone": "bad", "text": "No broker at %s, retrying"
                        % self.broker_text}
            if conn["tone"] != self.conn_tone:
                if conn["tone"] == "ok" and self.conn_tone == "bad":
                    self.reply("Connected to the broker at %s." % self.broker_text)
                self.conn_tone = conn["tone"]
            if self.last_heard is None:
                online = {"tone": "wait", "text": "Car: no contact yet"}
            elif now - self.last_heard > mu.OFFLINE_AFTER_S:
                # Only the car -> PC direction is known to be quiet: commands
                # may still reach the car (seen on a lossy Wi-Fi uplink).
                online = {"tone": "bad", "text": "No data from car (%d s)"
                          % (now - self.last_heard)}
            else:
                online = {"tone": "ok", "text": "Car online"}
            stale = None
            if self.telem_last and now - self.telem_last > mu.TELEM_STALE_S:
                stale = int(now - self.telem_last)
            why = self.why_now()
            if stale is not None:
                why["stale"] = ("No telemetry for %d s: the car is off, or its Wi-Fi "
                                "uplink is stalled (commands may still reach it). "
                                "This is the last known reason." % stale)
            avg = (self.run_dist_mm / self.run_time_s) if self.run_time_s > 0 else 0
            speed = {
                "v": self.spd[0] if self.spd else None,
                "level": self.spd[1] if self.spd else None,
                "at": self.spd[2] if self.spd else "",
                "max": self.run_max, "avg": int(avg),
                "dist_m": round(self.run_dist_mm / 1000.0, 2),
                "time_s": int(self.run_time_s), "stale": stale,
            }
            e = self.edge
            return {
                "conn": conn, "broker": self.broker_text, "online": online,
                "state": self.cur_state,
                "faults": [n for i, n in enumerate(ll.FAULT_BITS)
                           if self.cur_faults & (1 << i)],
                "why": why, "speed": speed, "barcode": self.bc, "hump": self.hump,
                "wheel_now": self.wheel_now, "acc_raw": self.acc_raw,
                "width": {"mm": self.obst_w, "measuring": self.measuring},
                "side": self.side, "depth": self.depth,
                "mapping": self.mapping, "map_note": self.map_note,
                "replies": list(self.replies),
                "edge": {"looks": e.looks, "face": e.face, "bound": e.bound,
                         "oov": e.oov, "edges": e.edges, "known": e.known,
                         "runs": [dict(e.runs[n], n=n) for n in sorted(e.runs)],
                         "fit": e.fit, "status": e.status},
            }

    # ---- actions from the page -------------------------------------------
    def command(self, word):
        err = check_cmd(word)
        if err:
            return err
        with self.lock:
            if word == "stop":
                self.ui_stop_t = time.time()  # lets "why stopped" name the panel
            elif word.startswith("depth "):
                mm = int(word[6:])
                self.depth_sent = mm
                self.depth.update(note="sent %d mm, waiting for the car" % mm,
                                  tone="wait")
            self.dirty = True
        # never retained: a retained "start" would restart the car on reconnect
        self.publish(mu.TOPIC_CMD, word, retain=False, done="sent: " + word)
        return None

    def send_map(self, body):
        picked = {}
        for _, word in mu.MOVES:
            letter = body.get(word)
            if letter not in mu.LETTERS:
                return "Pick a letter A to Z for every movement."
            picked[word] = letter
        if len(set(picked.values())) != len(picked):
            return "Each movement needs a different letter."
        default = body.get("default")
        if default not in mu.MOVE_WORDS:
            return "Pick what the car does with no barcode."
        payload = ",".join("%s=%s" % (picked[w], w) for _, w in mu.MOVES)
        payload += ",default=" + default
        with self.lock:
            self.mapping = dict(picked, default=default)
            self.map_note = {"text": "sent %s, waiting for the car" % hms(), "tone": "wait"}
            self.dirty = True
        # retained: a car that (re)connects later still gets it
        self.publish(mu.TOPIC_MAP, payload, retain=True, done="sent: " + payload)
        return None

    def api(self, action, body):
        """One POST from the page.  Returns None or an error message."""
        if action == "cmd":
            word = body.get("word")
            return self.command(word) if isinstance(word, str) else "Unknown command."
        if action == "map":
            return self.send_map(body)
        if action == "snapshot":
            self.publish(mu.TOPIC_SNAP, "now", done="snapshot requested")
            return None
        if action == "connect":
            text = body.get("broker")
            if not isinstance(text, str) or not R_BROKER.fullmatch(text.strip()):
                return "Enter the broker as an IP or host name, optionally :port."
            with self.lock:
                self.broker_text = text.strip()
            self.listen()
            return None
        if action == "clear_readings":
            with self.lock:
                self.clear_readings()
            return None
        if action == "clear_log":
            with self.lock:
                self.records.clear()
                self.pending_log = []
            self.hub.send("clear_log", {})
            return None
        return "Unknown action."

    def close(self):
        with self.lock:
            if self.sub_proc:
                self.sub_proc.kill()
                self.sub_proc = None


def ticker(car, hub):
    """Sends new log lines and the status to the page, restarts a stopped
    listener."""
    last = 0.0
    while True:
        time.sleep(TICK_S)
        now = time.time()
        with car.lock:
            logs, car.pending_log = car.pending_log, []
            st = None
            if car.dirty or now - last >= STATUS_EVERY_S:
                st = car.status()
                car.dirty = False
            retry = car.want_retry(now)
        if logs:
            hub.send("log", logs)
        if st:
            hub.send("status", st)
            last = now
        if retry:
            car.listen(quiet=True)


def allowed_networks(specs, broker):
    """The client networks allowed besides this PC.  None = everyone.
    No --allow: the /24 around the broker address (the car's network), if
    that is a private IPv4 address."""
    if specs:
        if "any" in specs:
            return None
        return [ipaddress.ip_network(s, strict=False) for s in specs]
    host = broker.rpartition(":")[0] if re.search(r":\d+$", broker) else broker
    try:
        ip = ipaddress.ip_address(host)
    except ValueError:
        return []
    if ip.version == 4 and ip.is_private and not ip.is_loopback:
        return [ipaddress.ip_network("%s/24" % ip, strict=False)]
    return []


def own_addresses():
    """This PC's IPv4 addresses (for the URLs printed at start-up)."""
    try:
        infos = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
    except OSError:
        return []
    return sorted({i[4][0] for i in infos})


class Handler(BaseHTTPRequestHandler):
    car = None
    hub = None
    networks = []                        # client networks allowed; None = any
    server_version = "RobotCarPanel/1"

    def log_message(self, fmt, *args):     # keep the console for errors only
        pass

    def client_ok(self):
        try:
            ip = ipaddress.ip_address(self.client_address[0])
        except ValueError:
            return False
        if getattr(ip, "ipv4_mapped", None):
            ip = ip.ipv4_mapped
        if ip.is_loopback or self.networks is None:
            return True
        return any(ip in n for n in self.networks)

    def host_ok(self):
        """Only clients from the allowed networks, and only with an IP
        address, localhost or this PC's name in the Host header: that
        blocks other web sites (and DNS rebinding) from using the panel."""
        if not self.client_ok():
            return False
        host = (self.headers.get("Host") or "").lower()
        name = host.split("]")[0] + "]" if host.startswith("[") else host.rsplit(":", 1)[0]
        try:
            ipaddress.ip_address(name.strip("[]"))
        except ValueError:
            if name not in ("localhost", socket.gethostname().lower()):
                return False
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc.lower() != host:
            return False
        return True

    def send_body(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, code, obj):
        self.send_body(code, json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        if not self.host_ok():
            return self.send_json(403, {"ok": False, "error": "forbidden"})
        path = urlparse(self.path).path
        if path == "/events":
            return self.events()
        if path in STATIC:
            name, ctype = STATIC[path]
            try:
                with open(os.path.join(WEB_DIR, name), "rb") as f:
                    body = f.read()
            except OSError:
                return self.send_json(500, {"ok": False, "error": "missing " + name})
            return self.send_body(200, body, ctype)
        return self.send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        if not self.host_ok():
            return self.send_json(403, {"ok": False, "error": "forbidden"})
        if not (self.headers.get("Content-Type") or "").startswith("application/json"):
            return self.send_json(415, {"ok": False, "error": "JSON only"})
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = -1
        if not 0 <= n <= MAX_BODY:
            return self.send_json(413, {"ok": False, "error": "bad length"})
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            body = None
        if not isinstance(body, dict):
            return self.send_json(400, {"ok": False, "error": "bad JSON"})
        path = urlparse(self.path).path
        if not path.startswith("/api/"):
            return self.send_json(404, {"ok": False, "error": "not found"})
        err = self.car.api(path[5:], body)
        if err:
            return self.send_json(400, {"ok": False, "error": err})
        return self.send_json(200, {"ok": True})

    def events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        q = self.hub.add()                   # before the snapshot: nothing is missed
        try:
            with self.car.lock:
                hello = {"config": config(), "status": self.car.status(),
                         "records": list(self.car.records)}
            self.wfile.write(sse("hello", hello).encode())
            self.wfile.flush()
            while True:
                try:
                    msg = q.get(timeout=15)
                except queue.Empty:
                    msg = ": ping\n\n"
                if msg is None:
                    break
                self.wfile.write(msg.encode())
                self.wfile.flush()
        except OSError:                      # the tab was closed
            pass
        finally:
            self.hub.remove(q)


def main():
    ap = argparse.ArgumentParser(description="Robot car web control panel")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--allow", action="append", metavar="NETWORK",
                    help="client network allowed besides this PC, e.g. "
                         "192.168.1.0/24 (repeatable), or 'any'.  Default: the "
                         "/24 around the broker address (the car's network)")
    ap.add_argument("--local-only", action="store_true",
                    help="this PC only: nothing on the network can open the page")
    ap.add_argument("--broker", default=None,
                    help="MQTT broker, IP or IP:port (default: MQTT_BROKER_IP "
                         "from config/wifi_secrets.h)")
    ap.add_argument("--no-browser", action="store_true", help="don't open the browser")
    args = ap.parse_args()

    broker = args.broker or mu.car_broker_ip()
    try:
        nets = [] if args.local_only else allowed_networks(args.allow, broker)
    except ValueError as err:
        print("Bad --allow network: %s" % err)
        return 1
    bind = "127.0.0.1" if args.local_only or nets == [] else "0.0.0.0"
    url = "http://127.0.0.1:%d/" % args.port
    hub = Hub()
    car = Car(hub, broker)
    Handler.car, Handler.hub, Handler.networks = car, hub, nets
    try:
        server = ThreadingHTTPServer((bind, args.port), Handler)
    except OSError as err:
        print("Cannot serve on port %d (%s)." % (args.port, err))
        print("Is the panel already running?  Opening it: " + url)
        if not args.no_browser:
            webbrowser.open(url)
        return 1
    server.daemon_threads = True

    car.listen()
    threading.Thread(target=ticker, args=(car, hub), daemon=True).start()
    print("Robot car panel: " + url)
    if nets is None:
        print("Open to EVERY network this PC is on (--allow any): anyone who can "
              "reach port %d can drive the car." % args.port)
        lan = [ip for ip in own_addresses() if not ip.startswith("127.")]
    elif nets:
        print("Also open to: %s  (anyone there can drive the car)"
              % ", ".join(str(n) for n in nets))
        lan = [ip for ip in own_addresses()
               if any(ipaddress.ip_address(ip) in n for n in nets)]
    else:
        print("This PC only.")
        lan = []
    for ip in lan:
        print("  from other devices: http://%s:%d/" % (ip, args.port))
    if nets != [] and not lan:
        print("  (this PC has no address on that network yet: is the hotspot on?)")
    print("Broker: %s   (change it in the page)" % car.broker_text)
    print("Keep this window open; press Ctrl+C or close it to stop the panel.")
    if not args.no_browser:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        car.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
