# Subsystem 1 — test log

**Subsystem:** WiFi, MQTT and telemetry (Buddy 1)  ·  **Student ID:** 2500667  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | cyclic handler is running | | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | LED task sleeps between ticks (not busy) | | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | *dropped: LED example replaced by comms status LED (see S1-M09)* | | | | |
| S1-U10 | status frame survives publish → queue → pop → decode | CR-11 | auto | `subsystem1_test.c` | PASS |
| S1-U11 | bad topic/length/pointer rejected | | auto | `subsystem1_test.c` | PASS |
| S1-U12 | full queue never blocks; drop flagged | NFR-B1-4 | auto | `subsystem1_test.c` | PASS |
| S1-U13 | fault mask raise/clear/get; invalid bits ignored | CR-11 | auto | `subsystem1_test.c` | PASS |
| S1-U14 | new fault → exactly one event frame | FR-B1-5 | auto | `subsystem1_test.c` | PASS |
| S1-U15 | fault mask race-free across both cores | CR-11 | auto | `subsystem1_test.c` | PASS |
| S1-U16 | valid cmd forwarded; invalid ones rejected | FR-B1-3 | auto | `subsystem1_test.c` | PASS |
| S1-U17 | heartbeat_task runs ≥ 2 beats in 2.5 s | FR-B1-4 | auto | `subsystem1_test.c` | PASS |
| S1-U18 | mode change and PING each give a status frame ≤ 100 ms | NFR-B1-1 | auto | `subsystem1_test.c` | PASS |
| S1-H01…H14 | bitpack + frame codec on PC (round trips, limits, layout, sizes) | CR-11 | auto (PC) | `cd testing/subsystem1/host && make run` | `15 passed, 0 failed` |
| S1-P01 | Python decoder matches C encoder bytes | CR-11 | auto (PC) | `python3 -m unittest test_codec_py` | 7 tests OK |
| S1-M01 | WiFi joins and gets an IP | FR-B1-1 | manual | flash `NET=lwip MQTT=1`, watch console | `wifi: link up, ip=…` |
| S1-M02 | MQTT connects and subscribes | FR-B1-2/3 | manual | as above, broker running | `mqtt: connected`, `subscribed to robot/cmd` |
| S1-M03 | heartbeat 1 Hz, 6 bytes, seq +1 mod 16 | FR-B1-4, NFR-B1-2 | manual | dashboard `monitor` for 30 s | 1 line/s, seq increments |
| S1-M04 | commands START/STOP/PAUSE/RESUME reach the robot | FR-B1-3 | manual | dashboard `send X`; console | `comms: cmd N id=M` for each |
| S1-M05 | AP power-cycle: reconnect with 1,2,4…10 s backoff, ≤ 10 s after AP returns | FR-B1-1, NFR-B1-3 | manual | switch AP off 30 s, then on | `wifi: retry in 1/2/4/8/10 s`, link up, status resumes |
| S1-M06 | broker stop/start: MQTT reconnects with backoff; CONN_LOST after 3 missed beats | NFR-B1-2/3 | manual | stop broker 20 s, restart | `retrying in …`, `CONN_LOST`, then status shows MQTT_LOSS for ~5 s |
| S1-M07 | comms never blocks other subsystems | NFR-B1-4 | manual | switch AP off; watch other tasks' console output | other tasks keep their normal rate |
| S1-M08 | state-change latency ≤ 100 ms | NFR-B1-1 | manual | dashboard `send PING`, note delay to STATUS line (several runs) | < 100 ms on a quiet LAN |
| S1-M09 | status LED patterns | | manual | GP19 LED | blink connecting / solid up / short flash on fault |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
