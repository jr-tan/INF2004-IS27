# Team and subsystems

The single place that says **who owns what** in the IS27 car firmware:
subsystems, pins, peripherals, tasks and the interfaces between them.
The values here are taken from the code; the code is the reference:

- pins, PWM slices, task priorities / cores, kernel object counts:
  `app_program/car_config.h` (checked at **compile time**, and again at
  boot by `resources.c`, which prints the table on the USB console)
- interfaces between subsystems: `app_program/car.h`

If you change one of those, update this file in the same commit.

The code follows BARR-C:2018 and Change Requirement 1 (Tribunal 1): see
`docs/barr_c_conformance.md`.

Rule: **only the owner edits a subsystem's folder.** To change another
subsystem, ask its owner or open a pull request for them to review.
Shared files (`app_main.c`, `car.h`, `car_config.h`, `mission.c`, this
file) change only by team agreement.

## 1. Members and subsystems

| # | Subsystem (what it does) | Owner | Student ID | GitHub | Code |
|---|---|---|---|---|---|
| 1 | **Comms:** WiFi/MQTT telemetry, remote commands, barcode map, shared fault bitmask; status task (console, status LED, RGB LEDs, buzzer) | Nurindra | 2500667 | dra-mars / dra77 | `app_program/subsystem1/` |
| 2 | **Motion:** motors, encoders, per-wheel speed control, stall boost, odometry, heading | Kavinesh | 2501197 | kavi-dev | `app_program/subsystem2/` |
| 3 | **Line + barcode:** IR line following (INSIDE / EDGE / STRADDLE), Code 39 Full ASCII barcodes | James | 2500349 | iamhackerman | `app_program/subsystem3/` |
| 4 | **IMU:** GY-511 pitch / roll, humps, impact, tilt and pick-up detection | Jin Rong | 2501003 | jr-tan | `app_program/subsystem4/` |
| 5 | **Ranger / obstacle:** ultrasonic, servo, guard, fan sweep, obstacle profile | Jiale | 2500170 | koyorii | `app_program/subsystem5/` |
| – | **Mission (shared):** buttons, commands, FOLLOW / TURN / AVOID / climb state machine | team | | | `app_program/mission.c` |

## 2. Pins

From `car_config.h` with **`BARCODE_ANALOG 0`** (barcode D0 on GP2, the
wiring in use). `BARCODE_ANALOG 1` moves the barcode to A0 on GP28 and the
left encoder B to GP3; flash the matching build after rewiring.

| Pin | Owner | Used for |
|---|---|---|
| GP0, GP1 | S2 | right wheel encoder A / B (Grove 1). *Also the `CRASH_REPORT` / UART console pins: don't build with those.* |
| GP2 | S3 | barcode sensor D0 (Grove 2), edge interrupt |
| GP3 | – | free (GY-511 INT1 is not used: the IMU is polled) |
| GP4, GP5 | S4 | I2C0 SDA / SCL to the GY-511 (Grove 3) |
| GP7 | S2 | left wheel encoder A (Grove 7) |
| GP8, GP9 | S2 | left motor M1A / M1B (PWM slice 4) |
| GP10, GP11 | S2 | right motor M2A / M2B (PWM slice 5), mounted mirrored |
| GP13 | S5 | servo (PWM slice 6 B) |
| GP14 | S5 | alert LED (via 220 R) |
| GP16 | S5 | ultrasonic TRIG (Grove 4) |
| GP17 | S5 | ultrasonic ECHO (Grove 4, PWM slice 0 B gated counter) |
| GP18 | S1 | 2 x WS2812 RGB LEDs (PIO0 state machine 3) |
| GP19 | S1 | status LED (via 330 R) |
| GP20 | mission | START button (active low) |
| GP21 | mission | STOP button (active low) |
| GP22 | S1 | buzzer (PWM slice 3 A) |
| GP26 | S3 | left line sensor A0 (ADC0, Grove 6) |
| GP27 | S3 | right line sensor A0 (ADC1, Grove 6) |
| GP28 | S2 | left wheel encoder B (Grove 7, digital) |
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP6, GP12, GP15 | – | free |

## 3. Peripherals

Each peripheral is driven by exactly **one task**; other tasks ask it
through the interfaces in section 5.

| Peripheral | Owner | Owning task | Pins |
|---|---|---|---|
| PWM slice 4, 5 (motors) | S2 | `motion` | GP8–GP11 |
| PWM slice 0 (echo timer) | S5 | `ranger` | GP17 |
| PWM slice 6 (servo) | S5 | `ranger` | GP13 |
| PWM slice 3 (buzzer), PIO0 SM3 (RGB LEDs) | S1 | `status` | GP22, GP18 |
| ADC | S3 | `line` (with `BARCODE_ANALOG 1`: the ADC FIFO interrupt, started by `barcode`) | GP26, GP27 (+ GP28 barcode A0) |
| I2C0 (own polled driver, `drv_i2c.c`) | S4 | `imu` | GP4, GP5 |
| GPIO interrupt (shared dispatcher, core 0) | shared | `gpio_irq.c`: encoders (S2) and barcode (S3) | GP0, GP1, GP2, GP7, GP28 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 (radio on PIO1) |

**WiFi and MQTT:** no application task calls `cyw43_`, lwIP or MQTT
functions. The `comms` task puts frames into `g_mbf_pub`; the kernel's
WiFi/MQTT task sends them and puts received commands into `g_mbf_rx`
(`subsystem1/comms_port.h`).

## 4. Tasks

1 is the highest priority. 1–4 belong to the kernel (init 1, USB console
3, WiFi 4). `TP_PRC1` = core 0, `TP_PRC2` = core 1.

| Task | Subsystem | Priority | Core | Period / trigger |
|---|---|---|---|---|
| `motion` | S2 | 10 | 0 (encoder ISR is on core 0) | 20 ms (`ctl_tick_isr`) or owner change / STOP |
| `mission` | mission.c | 11 | 1 | 20 ms or any mission event |
| `line` | S3 | 12 | 1 | 20 ms (woken by `ctl_tick_isr`) |
| `ranger` | S5 | 13 | 1 | one ping every 60 ms (short 10 ms echo window in sweeps) |
| `imu` | S4 | 14 | 1 | 20 ms (50 Hz) |
| `barcode` | S3 | 15 | 0 (same core as the edge interrupt) | woken by barcode edges |
| `comms` | S1 | 18 | 0 (same core as WiFi) | 100 ms |
| `status` | S1 | 20 | 0 | 500 ms |
| `idle0`, `idle1` | resources.c | lowest | 0 and 1 | CPU-load meters |

## 5. Interfaces between subsystems

Each module declares its interface in its own header (BARR-C 4.2:
`subsystemN.h`, `barcode.h`, `mission.h`, …); `app_program/car.h`
includes them all and holds the shared types and constants.

| ID | From → To | Mechanism | Name | Data / meaning |
|---|---|---|---|---|
| IF-01 | mission, S3 → S2 | per-owner speed slot + owner switch | `motion_wheels()`, `motion_drive()`, `motion_set_owner()`, `motion_stop()` | wheel speeds in mm/s; owner `MOT_OWNER_LINE` or `MOT_OWNER_MISSION`; only the owner's slot drives the motors |
| IF-02 | S2 → all | function calls | `motion_odometry_mm()`, `motion_heading_d10()`, `motion_heading_ok()`, `motion_get_status()` | distance, encoder heading (0.1 deg), wheel status |
| IF-03 | S2 → S3 | cyclic handler + task wake-up | `ctl_tick_isr` (20 ms) wakes `motion` (event flag `g_motion_flg`) and `line` | same 20 ms beat for both control loops |
| IF-04 | mission → S3 | function calls | `line_follow()`, `line_climb()`, `line_set_tuning()`, `line_set_pivot()` | start / stop following, seed side, climb mode, live tuning |
| IF-05 | S3 → mission | function calls + event bits | `line_get()`, `line_see_bits()`, `barcode_get()`, `barcode_busy()`; `EV_LINE_LOST`, `EV_JUNCTION`, `EV_BARCODE` | line state, decoded barcode and its action |
| IF-06 | mission → S5 | function calls | `ranger_aim()`, `ranger_guard()`, `ranger_fast()` | servo angle, guard on/off, sweep mode |
| IF-07 | S5 → mission | packed word + event bits | `ranger_word()`, `ranger_guard_cm()`; `EV_OBSTACLE`, `EV_PING` | distance, angle, sequence, valid |
| IF-08 | S4 → mission | function call + event bits | `imu_get()`; `EV_TILT`, `EV_LIFT` | pitch / roll / humps; tipped over or picked up -> safe stop |
| IF-09 | S1 → mission | event bits | `EV_CMD_START` … `EV_CMD_REVERSE` (event flag `g_mission_flg`) | remote commands from `robot/cmd` |
| IF-10 | S1 → S3 | function calls | `barcode_set_map()`, `barcode_get_map()` | letter → action map from `robot/map` |
| IF-11 | all ↔ S1 | event flag + calls | `comms_fault_raise()`, `comms_fault_clear()`, `comms_fault_get()` (`g_fault_flg`) | fault bitmask, `FAULT_*` in `car.h` |
| IF-12 | all → S1 | function calls (read by `comms`) | `mission_state()`, `mission_sys_mode()`, `mission_obstacle_pop()`, `resources_usage()` … | telemetry sources |
| IF-13 | S1 ↔ kernel WiFi/MQTT task | message buffers | `g_mbf_pub` (frames out), `g_mbf_rx` (commands in) | binary frames, `subsystem1/comms_frames.h` |
| IF-14 | S3 → S1, S5 | packed 32-bit words (Change Requirement 1) | `line_raw_word()`, `line_dark_word()`; unpack with `LINE_WORD_LEFT/RIGHT/SEQ` | left 12 + right 12 + sequence 8 bits, one store per update |

**MQTT topics** (binary, layouts in `subsystem1/comms_frames.h`):
`robot/status` (retained), `robot/event/fault`, `robot/tlm/motion`,
`robot/tlm/line` (also barcodes), `robot/tlm/terrain`,
`robot/tlm/obstacle`, `robot/tlm/resources`, `robot/tlm/map`; in:
`robot/cmd` (1 START, 2 STOP, 3 PAUSE, 4 RESUME, 5 PING, 6 U-TURN,
7 REVERSE, or a tuning frame) and `robot/map` (barcode map, retained).

**Fault bits** (wire format): 0 motor stall, 1 encoder, 2 IMU, 3 no echo,
4 WiFi loss, 5 MQTT loss, 6 line, 7 avoid, 8 motion (tilt / pick-up /
stuck on ramp), 9 impact, 10 comms.

## 6. Kernel object budget

Limits are in `config/config.h`; `car_config.h` checks that ours plus the
kernel's own fit, and `resources.c` prints the real numbers at boot.

| Object | Used by the car | Who |
|---|---|---|
| Task | 10 | 8 in the table above + 2 idle meters |
| Event flag | 3 | `g_motion_flg` (S2), `g_mission_flg` (mission), `g_fault_flg` (S1) |
| Message buffer | 2 | `g_mbf_pub`, `g_mbf_rx` (S1) |
| Cyclic handler | 1 | `ctl_tick_isr` (S2) |
| Semaphore, mutex, mailbox, memory pools, alarm | 0 | – |

(The kernel's USB console, WiFi and device drivers use a few more.)
