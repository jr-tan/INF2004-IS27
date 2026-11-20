"""
comms_codec.py: decoder/encoder for the IS27 car's binary MQTT frames.

Mirrors app_program/subsystem1/comms_frames.h.  Each frame is one
little-endian bit string; the first field takes the LOWEST bits:
    value = int.from_bytes(frame, "little");  field = (value >> offset) & mask
Signed fields are two's complement in their own width.
testing/comms_test/ checks this file against the C packer.
"""

SYS_MODES = ["INIT", "IDLE", "LINE", "BARCODE", "AVOID", "RECOVERY", "PAUSED", "SAFE_STOP", "COMPLETE"]
FAULT_NAMES = ["MOTOR_STALL", "ENCODER", "IMU", "NO_ECHO", "WIFI_LOSS", "MQTT_LOSS",
               "LINE", "AVOID", "MOTION", "SUDDEN_IMPACT", "COMMS", "(reserved)"]
CMD_CODES = {"START": 1, "STOP": 2, "PAUSE": 3, "RESUME": 4, "PING": 5, "UTURN": 6, "REVERSE": 7}
LINE_MODES = ["off", "track", "centred", "recover", "junction", "LOST", "bend stop", "pivot"]
BC_ACTIONS = ["none", "LEFT", "RIGHT", "U-TURN", "END", "?", "?", "?"]
OWNERS = ["NONE", "LINE", "MISSION", "?"]
OBST_ACTIONS = ["none", "LEFT", "RIGHT", "BLOCKED"]
OBST_STAGES = ["FAILED", "DETECTED", "FALSE_ALARM", "SWEPT", "GONE", "BLOCKED", "PASSED", "REJOINED"]
IMU_EVENTS = ["HUMP", "IMPACT", "TILT"]

SIZES = {"status": 6, "fault": 4, "motion": 7, "line": 8, "terrain": 7, "obstacle": 8, "resources": 15}


def _fields(frame, spec):
    """spec: list of (name, width, signed)"""
    v = int.from_bytes(frame, "little")
    out, pos = {}, 0
    for name, w, signed in spec:
        x = (v >> pos) & ((1 << w) - 1)
        if signed and x & (1 << (w - 1)):
            x -= 1 << w
        out[name] = x
        pos += w
    return out


def _check(frame, kind):
    if len(frame) != SIZES[kind]:
        raise ValueError("%s frame must be %d bytes, got %d" % (kind, SIZES[kind], len(frame)))


def faults_to_names(mask):
    return [n for i, n in enumerate(FAULT_NAMES) if mask & (1 << i)]


def decode_status(frame):
    _check(frame, "status")
    f = _fields(frame, [("seq", 4, 0), ("up_s", 20, 0), ("rssi", 7, 0), ("mode", 4, 0),
                        ("faults", 12, 0), ("tx_drop", 1, 0)])
    r = f.pop("rssi")
    f["rssi_dbm"] = -r if r else None
    f["mode"] = SYS_MODES[f["mode"]] if f["mode"] < len(SYS_MODES) else "?"
    f["fault_names"] = faults_to_names(f["faults"])
    f["tx_drop"] = bool(f["tx_drop"])
    return f


def decode_fault(frame):
    _check(frame, "fault")
    f = _fields(frame, [("t_ms", 16, 0), ("raised", 12, 0), ("pad", 4, 0)])
    f["raised_names"] = faults_to_names(f["raised"])
    return f


def decode_motion(frame):
    _check(frame, "motion")
    f = _fields(frame, [("t_ms", 16, 0), ("vl", 11, 1), ("vr", 11, 1), ("dist_cm", 14, 0),
                        ("owner", 2, 0), ("enc_l", 1, 0), ("enc_r", 1, 0)])
    f["owner"] = OWNERS[f["owner"]]
    f["enc_ok"] = 1 if (f["enc_l"] and f["enc_r"]) else 0
    return f


def decode_line(frame):
    _check(frame, "line")
    f = _fields(frame, SPECS["line"])
    f["speed_mms"] = f.pop("speed_cs") * 10
    f["kp"] = f.pop("kp100") * 100
    f["mode"] = LINE_MODES[f["mode"]]
    f["pos"] = f.pop("pos8") * 8
    f["bc_char"] = chr(f["bc_char"] - 1 + 32) if f["bc_char"] else ""
    f["bc_action"] = BC_ACTIONS[f["bc_action"]]
    return f


def decode_terrain(frame):
    _check(frame, "terrain")
    f = _fields(frame, [("t_ms", 16, 0), ("pitch_d10", 10, 1), ("roll_d10", 10, 1),
                        ("hump_mm", 8, 0), ("heading", 9, 0), ("events", 3, 0)])
    if f["heading"] == 511:
        f["heading"] = None
    f["event_names"] = [n for i, n in enumerate(IMU_EVENTS) if f["events"] & (1 << i)]
    return f


def decode_obstacle(frame):
    _check(frame, "obstacle")
    f = _fields(frame, SPECS["obstacle"])
    f["action"] = OBST_ACTIONS[f["action"]]
    f["stage"] = OBST_STAGES[f["stage"]]
    return f


def decode_resources(frame):
    _check(frame, "resources")
    return _fields(frame, SPECS["resources"])


# Field layouts (name, bits, signed), lowest bits first: same as comms_frames.h
SPECS = {
    "status":   [("seq", 4, 0), ("up_s", 20, 0), ("rssi", 7, 0), ("mode", 4, 0), ("faults", 12, 0), ("tx_drop", 1, 0)],
    "fault":    [("t_ms", 16, 0), ("raised", 12, 0), ("pad", 4, 0)],
    "motion":   [("t_ms", 16, 0), ("vl", 11, 1), ("vr", 11, 1), ("dist_cm", 14, 0), ("owner", 2, 0), ("enc_l", 1, 0), ("enc_r", 1, 0)],
    "line":     [("t_ms", 16, 0), ("mode", 3, 0), ("pos8", 8, 1), ("see", 2, 0), ("bc_char", 6, 0), ("bc_action", 3, 0),
                 ("speed_cs", 8, 0), ("bend_pct", 7, 0), ("kp100", 5, 0), ("pivot", 1, 0)],
    "terrain":  [("t_ms", 16, 0), ("pitch_d10", 10, 1), ("roll_d10", 10, 1), ("hump_mm", 8, 0), ("heading", 9, 0), ("events", 3, 0)],
    "obstacle": [("t_ms", 16, 0), ("dist_cm", 8, 0), ("bearing_deg", 8, 1), ("width_cm", 8, 0),
                 ("need_left_cm", 8, 0), ("need_right_cm", 8, 0), ("action", 2, 0), ("stage", 3, 0)],
    "resources": [("t_ms", 16, 0), ("cpu0", 7, 0), ("cpu1", 7, 0), ("tasks", 6, 0), ("tasks_max", 6, 0),
                  ("flags", 5, 0), ("flags_max", 5, 0), ("sems", 5, 0), ("sems_max", 5, 0),
                  ("mbfs", 4, 0), ("mbfs_max", 4, 0), ("cycs", 4, 0), ("cycs_max", 4, 0),
                  ("queue_pct", 7, 0), ("drops", 8, 0), ("motion_us", 12, 0), ("line_us", 12, 0)],
}


def encode(kind, **values):
    """Pack raw field values (as the car would); missing fields are 0.
    Used by the GUI's demo mode and by tests."""
    v, pos = 0, 0
    for name, w, signed in SPECS[kind]:
        x = int(values.get(name, 0))
        lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if signed else (0, (1 << w) - 1)
        x = max(lo, min(hi, x))
        v |= (x & ((1 << w) - 1)) << pos
        pos += w
    return v.to_bytes(SIZES[kind], "little")


def encode_speed(mms):
    """robot/cmd 2-byte form: 0, line speed in cm/s"""
    return bytes([0, max(3, min(30, round(mms / 10)))])


def encode_tune(mms, bend_pct, kp, pivot=None):
    """robot/cmd tuning: 0, speed cm/s, bend %, KP/100 (0 = unchanged),
    then optionally pivot at bends: 1 off, 2 on (5 bytes)"""
    b = [0, max(3, min(30, round(mms / 10))), max(20, min(100, int(bend_pct))),
         max(3, min(30, round(kp / 100)))]
    if pivot is not None:
        b.append(2 if pivot else 1)
    return bytes(b)


def encode_cmd(name, cmd_id=0):
    """robot/cmd: code 3 | id 5 (1 byte)"""
    if not 0 <= cmd_id < 32:
        raise ValueError("id must be 0..31")
    return bytes([(cmd_id << 3) | CMD_CODES[name.upper()]])


DECODERS = {
    "status": decode_status, "event/fault": decode_fault, "tlm/motion": decode_motion,
    "tlm/line": decode_line, "tlm/terrain": decode_terrain, "tlm/obstacle": decode_obstacle,
    "tlm/resources": decode_resources,
}
