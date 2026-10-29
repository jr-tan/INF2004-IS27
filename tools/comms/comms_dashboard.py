#!/usr/bin/env python3
"""
comms_dashboard.py -- PC-side monitor and command sender for the robot car.

    pip install paho-mqtt
    python3 comms_dashboard.py --broker 192.168.1.10 monitor
    python3 comms_dashboard.py --broker 192.168.1.10 send START
    python3 comms_dashboard.py --broker 192.168.1.10 send PING
    python3 comms_dashboard.py --broker 192.168.1.10 latency --count 20

Options:  --root robot    topic root (must match MQTT_TOPIC_ROOT in wifi_secrets.h)
          --port 1883

Commands are NEVER sent retained: a retained START would fire again every time
the robot reconnects.
"""
import argparse
import statistics
import sys
import threading
import time

import paho.mqtt.client as mqtt

from comms_codec import CMD_CODES, decode_fault, decode_status, encode_cmd


def make_client():
    # paho-mqtt 2.x needs the callback API version; 1.x does not have it.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="comms-dashboard")
    except AttributeError:
        return mqtt.Client(client_id="comms-dashboard")


def run_monitor(args):
    root = args.root
    last_status = {"t": None}

    def on_connect(client, userdata, flags, reason_code, properties=None):
        print("connected to broker (%s)" % reason_code)
        client.subscribe(root + "/#", qos=1)

    def on_message(client, userdata, msg):
        now = time.strftime("%H:%M:%S")
        payload = bytes(msg.payload)
        topic = msg.topic
        try:
            if topic == root + "/status":
                s = decode_status(payload)
                rssi = "?" if s["rssi_dbm"] is None else "%d dBm" % s["rssi_dbm"]
                print("%s STATUS seq=%2d up=%ds rssi=%s mode=%s faults=%s%s%s"
                      % (now, s["seq"], s["up_s"], rssi, s["mode"],
                         ",".join(s["fault_names"]) or "none",
                         " TX_DROP" if s["tx_drop"] else "",
                         "  (retained)" if msg.retain else ""))
            elif topic == root + "/event/fault":
                f = decode_fault(payload)
                print("%s FAULT  t=%dms raised=%s" % (now, f["t_ms"], ",".join(f["raised_names"])))
            else:
                print("%s %s  %d bytes: %s" % (now, topic, len(payload), payload.hex(" ")))
        except ValueError as e:
            print("%s %s  BAD FRAME (%s): %s" % (now, topic, e, payload.hex(" ")))

    c = make_client()
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(args.broker, args.port, keepalive=30)
    try:
        c.loop_forever()
    except KeyboardInterrupt:
        print()


def run_send(args):
    c = make_client()
    c.connect(args.broker, args.port, keepalive=30)
    c.loop_start()
    cid = int(time.time()) % 32
    frame = encode_cmd(args.command, cid)
    info = c.publish(args.root + "/cmd", frame, qos=1, retain=False)
    info.wait_for_publish(5)
    print("sent %s id=%d -> %s/cmd  (byte 0x%02X)" % (args.command.upper(), cid, args.root, frame[0]))
    c.loop_stop()
    c.disconnect()


def run_latency(args):
    """Measure PING -> STATUS round trip (PC -> broker -> robot -> broker -> PC).

    A PING makes the robot publish a status frame at once.  Each sample is sent
    0.3 s after a regular 1 Hz status arrives, so the next regular frame is
    ~0.7 s away; the first status arriving within 0.55 s of the PING is its reply.
    The RTT covers two WiFi legs, so one leg (robot -> broker, what NFR-B1-1
    limits to 100 ms) is roughly half of it.
    """
    arrivals = []                      # perf_counter() of every status frame
    lock = threading.Lock()

    def on_connect(client, userdata, flags, reason_code, properties=None):
        client.subscribe(args.root + "/status", qos=0)

    def on_message(client, userdata, msg):
        with lock:
            arrivals.append(time.perf_counter())

    c = make_client()
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(args.broker, args.port, keepalive=30)
    c.loop_start()

    samples, skipped = [], 0
    deadline = time.time() + 15
    while time.time() < deadline and not arrivals:
        time.sleep(0.1)
    if not arrivals:
        print("no status frames from the robot -- is it connected?")
        return 1

    for i in range(args.count):
        with lock:                     # wait for the next regular frame
            seen = len(arrivals)
        t_wait = time.time() + 3
        while time.time() < t_wait:
            with lock:
                if len(arrivals) > seen:
                    break
            time.sleep(0.005)
        time.sleep(0.3)
        with lock:
            seen = len(arrivals)
        t0 = time.perf_counter()
        c.publish(args.root + "/cmd", encode_cmd("PING", i % 32), qos=0, retain=False)
        reply = None
        while time.perf_counter() - t0 < 0.55:
            with lock:
                if len(arrivals) > seen:
                    reply = arrivals[seen]
                    break
            time.sleep(0.001)
        if reply is None:
            skipped += 1
            print("%2d  no reply within 550 ms" % (i + 1))
        else:
            samples.append((reply - t0) * 1000.0)
            print("%2d  %6.1f ms" % (i + 1, samples[-1]))
    c.loop_stop()
    c.disconnect()

    if samples:
        med = statistics.median(samples)
        print("\n%d samples (%d without reply): min %.0f  median %.0f  max %.0f ms"
              % (len(samples), skipped, min(samples), med, max(samples)))
        print("approx. one leg (robot -> broker): median %.0f ms, max %.0f ms   [NFR-B1-1 limit: 100 ms]"
              % (med / 2.0, max(samples) / 2.0))
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--broker", required=True)
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--root", default="robot")
    sub = p.add_subparsers(dest="mode", required=True)
    sub.add_parser("monitor")
    lt = sub.add_parser("latency")
    lt.add_argument("--count", type=int, default=20)
    s = sub.add_parser("send")
    s.add_argument("command", choices=sorted(CMD_CODES, key=CMD_CODES.get), type=str.upper)
    args = p.parse_args()
    return {"monitor": run_monitor, "send": run_send, "latency": run_latency}[args.mode](args)


if __name__ == "__main__":
    sys.exit(main())
