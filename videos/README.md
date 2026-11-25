# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date       | Subsystem | Test ID / demo                 | Commit  | Student ID | Link                           | Notes                                 |
| ---------- | --------- | ------------------------------ | ------- | ---------- | ------------------------------ | ------------------------------------- |
| 2026-11-25 | S1, S2    | S1-U06, INT-03                 | 67cb365 | -          | <https://youtu.be/qcS9_0VE9O8> | Wheel drive and steering on the track |
| 2026-11-25 | S5        | S5-U05, S5-U08                 | 67cb365 | -          | <https://youtu.be/MFoleEEiH54> | Status LED colours and MODE hold      |
| 2026-11-25 | S4        | S4-U10                         | 67cb365 | -          | <https://youtu.be/BUdd4fKt3JU> | Servo sweep                           |
| 2026-11-25 | S2, S5    | S2-U09, INT-09                 | 67cb365 | -          | <https://youtu.be/4dCMt9Nr6f4> | Line lost and line search             |
| 2026-11-25 | SYS       | INT-02, SYS-03, SYS-04, S5-U05 | 67cb365 | -          | <https://youtu.be/92C_4sAY2dE> | Junction moves: left, right, U-turn   |
| 2026-11-25 | SYS       | SYS-09, SYS-07                 | 67cb365 | -          | <https://youtu.be/WCOPnbTaFRE> | Right obstacle bypass                 |
| 2026-11-25 | SYS       | SYS-09, SYS-07                 | 67cb365 | -          | <https://youtu.be/VTZhVHhzdKM> | Left obstacle bypass                  |
| 2026-11-25 | SYS       | SYS-16                         | 67cb365 | -          | <https://youtu.be/lAEoPwiIDgA> | Hotspot off while driving             |
| 2026-11-25 | SYS       | SYS-17                         | 67cb365 | -          | <https://youtu.be/9aZXYI_PDPk> | Stop moving: Lifting the car          |
