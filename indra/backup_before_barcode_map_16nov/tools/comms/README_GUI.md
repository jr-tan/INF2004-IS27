# IS27 dashboard (GUI)

A desktop window that shows everything the car sends over MQTT, sorted into one tab per dataset, and lets you send commands.

| Tab | Shows | Data from the car |
|---|---|---|
| Overview | mode, uptime, messages per second, the 11 fault lamps, live values, event log | everything |
| Motion | left/right wheel speed chart, distance chart, table | `robot/tlm/motion` (Buddy 2) |
| Line + barcode | line position chart, sensor mode table, list of barcodes read | `robot/tlm/line` (Buddy 3) |
| Terrain (IMU) | pitch/roll chart, compass, list of humps with heights | `robot/tlm/terrain` (Buddy 4) |
| Obstacle | every step of every obstacle (stopped, false alarm, swept, blocked, passed, rejoined, failed) and a top view of the last sweep | `robot/tlm/obstacle` (Buddy 5) |
| Faults | every fault event, and every change of the fault bitmask | `robot/event/fault`, `robot/status` (Buddy 1) |
| Resources | CPU load per core, longest motion/line loop each second, kernel objects in use (bars), MQTT queue fill and drops, task table | `robot/tlm/resources` (firmware v3.6+) |
| Raw frames | every frame as bytes + decoded, filter by topic | all |

Every tab has **Export CSV**. **Export all CSV** saves one file per dataset, which is handy for the report. The **Chart window** picks 30 s to 5 min, and **Freeze charts** stops them scrolling.

The **Send to car** buttons are START, STOP, PAUSE, RESUME, **U-TURN** (turn round on the line and follow it back), **REVERSE** (back up 20 cm slowly, then stop) and PING.

**Line tuning (live)** changes the line following while the car runs. Set the values, then press **Apply**:

- **speed**: the line speed on straights.
- **bends at**: the bend speed, as a % of the line speed.
- **steering KP**: how hard it steers.
- **stop + pivot at bends** (firmware v3.12+): at a bend the car stops, then turns with the inner wheel stopped and the outer wheel at the bend speed. The Line tab shows `bend stop` and `pivot` while it does this. Untick it and press Apply to steer through bends instead.

The car goes back to its defaults (30 mm/s, 50 %, pivot off, and its own KP for the layout: 1000 for EDGE) when it restarts.

## Run it

1. Install Python 3 (it includes tkinter; on Linux install `python3-tk`) and the MQTT library:
   ```
   pip install paho-mqtt
   ```
2. **Without the car**, to see it working:
   ```
   python3 is27_gui.py --demo
   ```
3. **With the car:**
   - Start Mosquitto on the laptop: `mosquitto -c mosquitto.conf -v`.
   - Allow TCP 1883 through the firewall.
   - Flash the car with the MQTT build, using your WiFi and this laptop's IP in `config/wifi_secrets.h`.
   - Then run:
     ```
     python3 is27_gui.py --broker <laptop IP>
     ```
   - Or start it with no options, type the IP in the **Broker** box and press **Connect**.

The **Topic root** must match `MQTT_TOPIC_ROOT` on the car (default `robot`).

## Files

- `is27_gui.py`: the window.
- `comms_codec.py`: decodes and encodes the binary frames, mirroring `app_program/subsystem1/comms_frames.h`. It is checked against the car's C code by `testing/comms_test`.
- `comms_dashboard.py`: the old text-only monitor, still works.
