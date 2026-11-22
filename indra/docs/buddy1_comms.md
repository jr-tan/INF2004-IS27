# Subsystem 1 — WiFi, MQTT and telemetry (Buddy 1)

Bring-up slice: WiFi join, MQTT connect, 1 Hz binary heartbeat, command receive, shared fault bitmask.
Not yet built: `telemetry_dispatch_task` and `tlm_send()` (need the other buddies' record formats).

## 1. Architecture (and one deviation from the design review)

TEAM.md section 3 forbids student tasks from calling lwIP or MQTT. So `wifi_link_task` / `mqtt_client_task`
from the design review are split like this:

| Design-review task | Where it runs now |
|---|---|
| WiFi join + reconnect backoff | kernel WiFi task `cyw43_utk_task` (`cyw43_utk.c`, edited) |
| `mqtt_client_task` (connect, publish, subscribe) | same kernel task, via `mqtt_utk.c` (edited) |
| `wifi_link_task` | real task: reads link state, sets `CF_WIFI_UP`, `FAULT_WIFI_LOSS`, `FAULT_COMMS` |
| `heartbeat_task` | real task: `robot/status` every 1 s, CONN_LOST |
| `cmd_dispatch_task` | real task: validates `robot/cmd`, sets `CMDF_` bits for mission control |

The two sides talk only through non-blocking kernel objects (`comms_port.h`): `mbf_pub` (outgoing frames) and `mbf_rx` (inbound commands).

## 2. Files

New in `app_program/subsystem1/`: `bitpack.[ch]`, `comms_frames.[ch]`, `comms_port.h`, `comms.c`,
`task_wifi_link.c`, `task_heartbeat.c`, `task_cmd_dispatch.c`. Rewritten: `subsystem1.[ch]`, `task_led.c`, `subsystem1_test.c`.
Tools/tests: `tools/comms/`, `testing/subsystem1/host/`.

**Shared files edited (agree with team/instructor):**
- `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` — binary publish pump, `robot/cmd` subscribe, 1–10 s backoff.
- `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` — 1–10 s rejoin backoff; rejoin after a *dropped* link (template only rejoined on negative status, so an AP power-cycle never recovered).
- `build_make/pico_rp2040.mk` — one `-I../app_program/subsystem1` in `LWIP_INCLUDES`.
- `config/wifi_secrets.example.h` — optional `MQTT_TOPIC_ROOT`.

## 3. Setup and test

### 3.1 Broker (on a PC on the same 2.4 GHz WiFi as the robot)
1. Install Mosquitto (https://mosquitto.org/download/). On Windows run it natively, not inside WSL (WSL's NAT address is not reachable from the robot).
2. `mosquitto -c tools/comms/mosquitto.conf -v`
3. Allow inbound TCP 1883 in the firewall (private network).
4. Note the PC's IP (`ipconfig` / `ip a`). The Pico W is **2.4 GHz only**; a phone hotspot must be set to 2.4 GHz. School/enterprise WiFi usually will not work (client isolation, no WPA2-PSK).

### 3.2 Credentials
```sh
cp config/wifi_secrets.example.h config/wifi_secrets.h     # gitignored
```
Set `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER_IP` (the PC's IP), `MQTT_BROKER_PORT 1883`. Keep `MQTT_CLIENT_ID` unique per robot. If several robots share a broker, define `MQTT_TOPIC_ROOT` (e.g. `"robot/team27"`) and pass the same to the dashboard with `--root`.

### 3.3 Build, flash, watch
```sh
cd build_make
make NET=lwip MQTT=1 -j8        # -> mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2
```
Hold BOOTSEL, plug in, copy the `.uf2` (README 2.3). Open the serial port at 115200. Expected, in order:
```
wifi: joining "<ssid>" ...
wifi: link up, ip=192.168.x.y
comms: WiFi up
mqtt: connected to <broker ip>
mqtt: subscribed to robot/cmd
mqtt: TX robot/status (6 bytes)        <- once per second
```
GP19 LED: blinking = connecting, solid = MQTT up, short flash every 1 s = comms fault.

### 3.4 Dashboard (Python)
```sh
pip install paho-mqtt
cd tools/comms
python3 comms_dashboard.py --broker <PC_IP> monitor          # decoded live view
python3 comms_dashboard.py --broker <PC_IP> send PING        # robot answers with a status frame at once
python3 comms_dashboard.py --broker <PC_IP> send START       # also STOP / PAUSE / RESUME
```
Expected monitor line: `STATUS seq= 3 up=42s rssi=? mode=INIT faults=none` (rssi unknown for now). Commands are never retained.

### 3.5 Tests
- **On PC (no hardware):** `cd testing/subsystem1/host && make run && python3 -m unittest test_codec_py` — expect `15 passed, 0 failed` and 7 Python tests OK.
- **On target, no network needed:** `make TEST=1 -j8`, flash, save the console; expect `[TEST] S1-U01 … S1-U18` PASS and `subsystem1 SUMMARY`.
- **Manual (WiFi + broker):** S1-M01 to S1-M09 in `testing/subsystem1/TEST_LOG.md`.

### 3.6 Troubleshooting
| Console shows | Cause |
|---|---|
| `SSID not found` | wrong name, out of range, or 5 GHz only |
| `authentication rejected` | wrong password / WPA3-only |
| link up, then `mqtt: connection status … retrying in N ms` repeating | broker not running, wrong `MQTT_BROKER_IP`, firewall, or Mosquitto missing `listener`/`allow_anonymous` |
| MQTT connected, monitor shows nothing | `--root` differs from `MQTT_TOPIC_ROOT` |

## 4. Wire format
One little-endian bit string; the first field takes the lowest bits (`v = int.from_bytes(frame,"little"); field = (v>>offset)&mask`). Built with `bitpack.c`, not C bit-fields (BARR-C 5.5).

| Topic | QoS | Fields (width in bits) | Size |
|---|---|---|---|
| `robot/status` (retained) | 0 | seq 4, uptime s 20, −RSSI dBm 7 (0=unknown), mode 4, faults 12, tx_drop 1 | 6 B |
| `robot/event/fault` | 1 | t ms (low 16) 16, raised faults 12, pad 4 | 4 B |
| `robot/cmd` (inbound) | 1 | code 3 (1 START, 2 STOP, 3 PAUSE, 4 RESUME, 5 PING), id 5 | 1 B |

## 5. Data representation table (Week 13)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| System mode | 0–8 (9 modes) | 4 | `SYS_MODE_*`; 1 byte in RAM (single writer), 4 bits on the wire |
| Fault conditions | 11 defined | 12 | one bit each in one shared mask: event flag `flg_fault` (atomic set/clear); 12 bits on the wire |
| Connection state | 6 facts | 6 | bits of event flag `flg_comms` (`CF_*`) |
| Command code / id | 1–5 / 0–31 | 3 / 5 | one byte |
| Status seq | 0–15 | 4 | wraps; written only by `heartbeat_task` |
| Uptime | 0–1 048 575 s | 20 | seconds, wraps after 12.1 days |
| RSSI | −127…−1 dBm, 0 = unknown | 7 | negated |
| Topic id (queue entry) | 0–5 | 3 (stored in 1 byte) | first byte of a queue entry; the kernel rounds entries up to 4 B anyway |
| Telemetry drop flag | 0/1 | 1 | `CF_TX_DROP`, read-and-clear atomically |

Not shrinkable: a kernel event flag is a 32-bit word (only the needed bits are used), and message-buffer entries are rounded to 4 bytes by the kernel.

**Race safety:** every word shared between tasks is a kernel event flag (atomic). Variables with one writer are plain `volatile` and the writer is named in a comment. Test S1-U15 hammers different fault bits from both cores at once.

## 6. Measurements so far
- **Message size** (host test, payload bytes; JSON = design-review examples): status 43→6, fault 43→4, cmd 22→1; total 108→11 (−90 %).
- **Static RAM** (`NET=lwip MQTT=1`, template vs this slice): BSS 82 248 → 82 092 B; flash +3 784 B. Run-time kernel objects added: queues 320 B + 128 B, three more 2 KB task stacks. The design review planned a 16 × 128 B = 2 048 B message pool for the same job.
- **Still to measure:** a true JSON "before" build (not written), state-change latency (S1-M08), CPU/timing impact with the other buddies' tasks running.

## 7. Planned telemetry frames (PROPOSAL — not implemented, needs each buddy's agreement)
| Topic | Fields (bits) | Size |
|---|---|---|
| `tlm/motion` | t 16, vl 10 (0.1 cm/s), vr 10, dist 14 (cm), state 3 | 7 B |
| `tlm/line` | t 16, line 2, barcode action 3 (needs Buddy 3) | 3 B |
| `tlm/terrain` | t 16, pitch s10 (0.1°), peak 8 (0.1 cm), yaw s12 (0.1°/s), move 3 | 7 B |
| `tlm/obstacle` | t 16, dist 8, bearing s8, width 8, clear L 8, clear R 8, action 2 | 8 B |
