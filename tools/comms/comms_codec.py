"""
comms_codec.py -- decoder/encoder for the robot's binary MQTT frames.

Mirrors app_program/subsystem1/comms_frames.h.  The wire layout is one
little-endian bit string: the first field occupies the LOWEST bits.
    value = int.from_bytes(frame, "little");  field = (value >> offset) & mask
Keep this file in step with comms_frames.h (the unit test checks both).
"""

SYS_MODES = ["INIT", "IDLE", "LINE", "BARCODE", "AVOID", "RECOVERY", "PAUSED", "SAFE_STOP", "COMPLETE"]

FAULT_NAMES = [  # bit 0 .. bit 11
    "MOTOR_STALL", "ENCODER", "IMU", "NO_ECHO", "WIFI_LOSS", "MQTT_LOSS",
    "LINE", "AVOID", "MOTION", "SUDDEN_IMPACT", "COMMS", "(reserved)",
]

CMD_CODES = {"START": 1, "STOP": 2, "PAUSE": 3, "RESUME": 4, "PING": 5}

STATUS_BYTES = 6
FAULT_BYTES = 4


def _fields(frame, widths):
    value = int.from_bytes(frame, "little")
    out, pos = [], 0
    for w in widths:
        out.append((value >> pos) & ((1 << w) - 1))
        pos += w
    return out


def faults_to_names(mask):
    return [n for i, n in enumerate(FAULT_NAMES) if mask & (1 << i)]


def decode_status(frame):
    """robot/status: seq 4 | up_s 20 | rssi 7 | mode 4 | faults 12 | tx_drop 1  (48 bits)"""
    if len(frame) != STATUS_BYTES:
        raise ValueError("status frame must be %d bytes, got %d" % (STATUS_BYTES, len(frame)))
    seq, up_s, rssi, mode, faults, drop = _fields(frame, [4, 20, 7, 4, 12, 1])
    if mode >= len(SYS_MODES):
        raise ValueError("bad mode %d" % mode)
    return {
        "seq": seq, "up_s": up_s,
        "rssi_dbm": (-rssi if rssi else None),   # None = unknown
        "mode": SYS_MODES[mode], "faults": faults,
        "fault_names": faults_to_names(faults), "tx_drop": bool(drop),
    }


def decode_fault(frame):
    """robot/event/fault: t 16 (ms, low 16 bits) | raised 12 | pad 4  (32 bits)"""
    if len(frame) != FAULT_BYTES:
        raise ValueError("fault frame must be %d bytes, got %d" % (FAULT_BYTES, len(frame)))
    t, raised, _pad = _fields(frame, [16, 12, 4])
    return {"t_ms": t, "raised": raised, "raised_names": faults_to_names(raised)}


def encode_cmd(name, cmd_id):
    """robot/cmd: code 3 | id 5  (1 byte)"""
    code = CMD_CODES[name.upper()]
    if not 0 <= cmd_id < 32:
        raise ValueError("id must be 0..31")
    return bytes([(cmd_id << 3) | code])


def encode_status(seq, up_s, rssi_neg_dbm, mode, faults, tx_drop=False):
    """Used by tests and by the offline simulator."""
    v = seq | (up_s << 4) | (rssi_neg_dbm << 24) | (mode << 31) | (faults << 35) | (int(tx_drop) << 47)
    return v.to_bytes(STATUS_BYTES, "little")
