#!/usr/bin/env python3
"""Packs frames with the car's C code (frames_dump) and decodes them with
tools/comms/comms_codec.py: every field must come back unchanged."""
import os, subprocess, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, "..", "..", "tools", "comms"))
import comms_codec as c

out = subprocess.check_output([os.path.join(here, "frames_dump")], text=True)
rows = [l.split() for l in out.splitlines()]
speedcmd = int([v for k, v in rows if k == "speedcmd"][0])
fr = {k: bytes.fromhex(v) for k, v in rows if k != "speedcmd"}
want = {
    "status":   (c.decode_status,   {"seq": 7, "up_s": 123456, "mode": "AVOID", "faults": 0x203, "tx_drop": True}),
    "fault":    (c.decode_fault,    {"t_ms": 70000 & 0xFFFF, "raised": 0x088}),
    "motion":   (c.decode_motion,   {"t_ms": 1234, "vl": -110, "vr": 115, "dist_cm": 532, "owner": "MISSION", "enc_ok": 1,
                                     "enc_l": 1, "enc_r": 1}),
    "line":     (c.decode_line,     {"t_ms": 999, "mode": "pivot", "pos": -64 * 8, "see": 1, "bc_char": "B", "bc_action": "RIGHT", "speed_mms": 80,
                                     "bend_pct": 45, "kp": 1800, "pivot": 1}),
    "terrain":  (c.decode_terrain,  {"t_ms": 4321, "pitch_d10": -73, "roll_d10": 125, "hump_mm": 23, "heading": 271, "events": 5}),
    "obstacle": (c.decode_obstacle, {"t_ms": 65535, "dist_cm": 12, "bearing_deg": -5, "width_cm": 10,
                                     "need_left_cm": 19, "need_right_cm": 21, "action": "LEFT", "stage": "SWEPT"}),
    "resources": (c.decode_resources, {"t_ms": 3210, "cpu0": 23, "cpu1": 41, "tasks": 13, "tasks_max": 32,
                                       "flags": 7, "flags_max": 16, "sems": 2, "sems_max": 16, "mbfs": 2,
                                       "mbfs_max": 12, "cycs": 1, "cycs_max": 12, "queue_pct": 5, "drops": 0,
                                       "motion_us": 187, "line_us": 96}),
    "saturate": (c.decode_motion,   {"vl": 1023, "vr": -1024, "dist_cm": 16383}),
}
fails = 0
ok_sp = (speedcmd == 120 and c.encode_tune(120, 40, 1800) == bytes([0, 12, 40, 18]) and
         c.encode_tune(120, 40, 1800, True) == bytes([0, 12, 40, 18, 2]) and
         c.encode_tune(120, 40, 1800, False) == bytes([0, 12, 40, 18, 1]))
print("tuning command      %s" % ("OK" if ok_sp else "WRONG (%d)" % speedcmd))
fails += not ok_sp
for k, (dec, exp) in want.items():
    got = dec(fr[k])
    bad = {f: (got[f], v) for f, v in exp.items() if got[f] != v}
    print("%-9s %2d bytes  %s" % (k, len(fr[k]), "OK" if not bad else "WRONG %s" % bad))
    fails += bool(bad)
print("total JSON-free size: %d bytes for one of each telemetry frame" %
      sum(len(fr[k]) for k in ("status", "motion", "line", "terrain", "obstacle")))
sys.exit(1 if fails else 0)
