"""
edge_cal.py -- "Edge calibration" data for the obstacle width sweep
(the web panel draws it; the Tkinter window was removed 19 Nov).

The car's width sweep (app_program/subsystem4/task_obstacle.c) swings the
ultrasonic sensor up to 35 deg each side and finds, per side, the angle
where the readings stop being the box face.  Each side is calibrated on
its own (subsystem5/obst_width.h):  tan(boundary) = S + K x edge / F.

The web panel's Edge calibration section
  * draws the last sweep from above: every look as a ray (green = box
    face, grey = off it), the face line, each side's boundary, the edges
    the car worked out (orange) and the edges you measured (blue);
  * "Measure only"   -> "esweep": sweep, print both edges, nothing stored;
  * "Add calibration run" -> "wcal <left> <right>": sweep against the
    edges you measured with a ruler from the car's centre line;
  * lists the runs and, once each side has edges far enough apart, the
    four constants to copy into obst_width.h.

Everything comes from the car's console log ([obst] / [sup] lines), so
the panel also follows sweeps started anywhere else.  Car in Idle only.
"""

import re

PIVOT_MM = 12                   # obst_width.h OW_PIVOT_MM
SIDE_MAX_DEG = 35               # subsystem4.h SCAN_SIDE_MAX_DEG
EDGE_MIN, EDGE_MAX = -150, 250  # subsystem5.h WM_EDGE_MIN_MM / WM_EDGE_MAX_MM
WIDTH_MIN, WIDTH_MAX = 20, 300  # subsystem5.h WM_CAL_MIN_MM / WM_CAL_MAX_MM
SPREAD_MM = 40                  # obst_width.h OW_CAL_MIN_SPREAD_MM

# task_obstacle.c width_sweep() / face_look()
R_START = re.compile(r"\[obst\] sweep start: ahead (\d+) (\d+) (\d+) mm")
R_LOOK = re.compile(r"\[obst\] sweep (-?\d+) deg: (\d+) mm (face|off)")
R_END = re.compile(r"\[obst\] width sweep: face (\d+) mm, left (\d+)\.(\d) deg, "
                   r"right (\d+)\.(\d) deg(.*)")
# supervisor.c width_result()
R_ESWEEP = re.compile(r"\[sup\] edge sweep: face (\d+) mm, left (\d+\.\d) deg -> (-?\d+) mm"
                      r"( \(out of view\))?, right (\d+\.\d) deg -> (-?\d+) mm"
                      r"( \(out of view\))?, width (-?\d+) mm")
R_ESWEEP_NONE = re.compile(r"\[sup\] edge sweep: nothing straight ahead")
R_RUN = re.compile(r"\[sup\] width cal run (\d+): face (\d+) mm, left (\d+\.\d) deg "
                   r"\(edge (-?\d+) mm, reads (-?\d+)\), right (\d+\.\d) deg "
                   r"\(edge (-?\d+) mm, reads (-?\d+)\)")
R_FIT = re.compile(r"\[sup\] width cal fit over (\d+) runs: set OW_SL_E4 (-?\d+) "
                   r"OW_KL_E4 (-?\d+) OW_SR_E4 (-?\d+) OW_KR_E4 (-?\d+)")
R_CHECK = re.compile(r"\[sup\] width cal   run (\d+): left (-?\d+) mm \(edge (-?\d+)\), "
                     r"right (-?\d+) mm \(edge (-?\d+)\), width (-?\d+) mm \(box (-?\d+)\)")
R_NEED = re.compile(r"\[sup\] width cal: (\d+) run\(s\) so far; (.*)")
R_FAIL = re.compile(r"\[sup\] width cal failed: (.*)")
R_CLEARED = re.compile(r"\[sup\] width cal runs cleared")
R_IGNORED = re.compile(r"\[sup\] width sweep ignored in state (\d+)(.*)")


class EdgeData:
    """What the log has said about sweeps and calibration runs."""

    def __init__(self):
        self.looks = []          # (deg, reading mm, is_face) of the last sweep
        self.face = None         # straight-ahead reading of the last sweep, mm
        self.bound = None        # (left deg, right deg) of the last sweep
        self.oov = (False, False)
        self.edges = None        # (left mm, right mm, width mm) the car worked out
        self.known = None        # (left mm, right mm) of the last calibration run
        self.runs = {}           # run number -> dict
        self.fit = None          # (n, SL, KL, SR, KR)
        self.status = "No sweep yet.  Car in Idle, box in front, then Measure only."

    def feed(self, line):
        """One console line.  True if something shown here changed."""
        m = R_START.search(line)
        if m:
            vals = sorted(int(v) for v in m.groups())
            self.looks = [(0, vals[1], True)]
            self.face = vals[1]
            self.bound = None
            self.edges = None
            self.oov = (False, False)
            self.status = "Sweeping..."
            return True
        m = R_LOOK.search(line)
        if m:
            self.looks.append((int(m.group(1)), int(m.group(2)), m.group(3) == "face"))
            return True
        m = R_END.search(line)
        if m:
            self.face = int(m.group(1))
            self.bound = (int(m.group(2)) + int(m.group(3)) / 10.0,
                          int(m.group(4)) + int(m.group(5)) / 10.0)
            tail = m.group(6)
            self.oov = ("left edge out of view" in tail, "right edge out of view" in tail)
            if "nothing ahead" in tail:
                self.status = "Nothing straight ahead: is the box within about 60 cm?"
            return True
        m = R_ESWEEP.search(line)
        if m:
            self.face = int(m.group(1))
            self.edges = (int(m.group(3)), int(m.group(6)), int(m.group(8)))
            self.oov = (bool(m.group(4)), bool(m.group(7)))
            self.known = None
            self.status = "Measured: left edge %d mm, right edge %d mm, width %s" % (
                self.edges[0], self.edges[1],
                ("%d mm" % self.edges[2]) if self.edges[2] > 0 else "not known")
            if any(self.oov):
                self.status += "  (an edge is out of view: move the box or the car back)"
            return True
        if R_ESWEEP_NONE.search(line):
            self.status = "Nothing straight ahead."
            return True
        m = R_RUN.search(line)
        if m:
            n = int(m.group(1))
            self.face = int(m.group(2))
            self.known = (int(m.group(4)), int(m.group(7)))
            self.edges = (int(m.group(5)), int(m.group(8)), int(m.group(5)) + int(m.group(8)))
            self.runs[n] = {"face": int(m.group(2)), "ldeg": m.group(3),
                            "lknown": int(m.group(4)), "lreads": int(m.group(5)),
                            "rdeg": m.group(6), "rknown": int(m.group(7)),
                            "rreads": int(m.group(8))}
            self.status = "Calibration run %d added." % n
            return True
        m = R_FIT.search(line)
        if m:
            self.fit = tuple(int(v) for v in m.groups())
            self.status = "Fitted over %d runs: copy the constants below." % self.fit[0]
            return True
        m = R_CHECK.search(line)
        if m:
            n = int(m.group(1))
            if n in self.runs:
                self.runs[n]["lreads"] = int(m.group(2))
                self.runs[n]["rreads"] = int(m.group(4))
            return True
        m = R_NEED.search(line)
        if m:
            self.status = "%s run(s): %s" % (m.group(1), m.group(2))
            return True
        m = R_FAIL.search(line)
        if m:
            self.status = "Calibration run not added: " + m.group(1)
            return True
        if R_CLEARED.search(line):
            self.runs = {}
            self.fit = None
            self.status = "Calibration runs cleared on the car."
            return True
        m = R_IGNORED.search(line)
        if m:
            self.status = ("The car ignored it: a sweep is already running."
                           if "already running" in m.group(2) else
                           "The car ignored it: it must be in Idle (blue LED).")
            return True
        return False
