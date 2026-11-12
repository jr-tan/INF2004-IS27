#!/usr/bin/env python3
"""
comms_dashboard.py: PC monitor and remote control for the IS27 car.

    pip install paho-mqtt
    python3 comms_dashboard.py --broker 192.168.1.10 monitor
    python3 comms_dashboard.py --broker 192.168.1.10 send START    (STOP PAUSE RESUME PING)

--root must match MQTT_TOPIC_ROOT in config/wifi_secrets.h (default "robot").
Commands are never retained (a retained START would fire on every reconnect).
"""
import argparse
import time

import paho.mqtt.client as mqtt

from comms_codec import DECODERS, CMD_CODES, encode_cmd


def make_client(name):
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=name)
    except AttributeError:          # paho-mqtt 1.x
        return mqtt.Client(client_id=name)


def show(sub, d):
    if sub == "status":
        return "STATUS   seq %2d up %5ds mode %-9s faults %s%s" % (
            d["seq"], d["up_s"], d["mode"], ",".join(d["fault_names"]) or "none",
            "  TX_DROP" if d["tx_drop"] else "")
    if sub == "event/fault":
        return "FAULT    raised %s" % ",".join(d["raised_names"])
    if sub == "tlm/motion":
        return "MOTION   L %4d R %4d mm/s  %5d cm  owner %-7s encoders %s" % (
            d["vl"], d["vr"], d["dist_cm"], d["owner"], "ok" if d["enc_ok"] else "FAULT")
    if sub == "tlm/line":
        bc = (' barcode "%s" -> %s' % (d["bc_char"], d["bc_action"])) if d["bc_char"] else ""
        return "LINE     %-8s pos %5d  sensors %s%s" % (
            d["mode"], d["pos"], "L" * (d["see"] & 1) + "R" * (d["see"] >> 1) or "-", bc)
    if sub == "tlm/terrain":
        hd = "--" if d["heading"] is None else "%3d" % d["heading"]
        return "TERRAIN  pitch %5.1f roll %5.1f deg  heading %s  last hump %3d mm  %s" % (
            d["pitch_d10"] / 10, d["roll_d10"] / 10, hd, d["hump_mm"], " ".join(d["event_names"]))
    if sub == "tlm/obstacle":
        return "OBSTACLE %3d cm at %+3d deg, width %3d cm, room L %3d / R %3d cm -> %s" % (
            d["dist_cm"], d["bearing_deg"], d["width_cm"], d["need_left_cm"], d["need_right_cm"], d["action"])
    return str(d)


def run_monitor(a):
    def on_connect(c, u, flags, rc, props=None):
        print("connected to %s, listening on %s/#" % (a.broker, a.root))
        c.subscribe(a.root + "/#", qos=1)

    def on_message(c, u, msg):
        sub = msg.topic[len(a.root) + 1:]
        dec = DECODERS.get(sub)
        if dec is None:
            return
        try:
            print(time.strftime("%H:%M:%S"), show(sub, dec(bytes(msg.payload))))
        except ValueError as e:
            print(time.strftime("%H:%M:%S"), sub, "BAD FRAME:", e, bytes(msg.payload).hex())

    c = make_client("is27-dashboard")
    c.on_connect, c.on_message = on_connect, on_message
    c.connect(a.broker, a.port, 30)
    c.loop_forever()


def run_send(a):
    c = make_client("is27-sender")
    c.connect(a.broker, a.port, 30)
    c.loop_start()
    info = c.publish(a.root + "/cmd", encode_cmd(a.command), qos=1, retain=False)
    info.wait_for_publish(5)
    print("sent", a.command)
    c.loop_stop()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--broker", required=True)
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--root", default="robot")
    s = p.add_subparsers(dest="what", required=True)
    s.add_parser("monitor")
    sp = s.add_parser("send")
    sp.add_argument("command", choices=sorted(CMD_CODES))
    a = p.parse_args()
    run_monitor(a) if a.what == "monitor" else run_send(a)


if __name__ == "__main__":
    main()
