# IS27 car firmware: fresh rewrite (fresh_v3.5, all five buddies)

## v3.5: line following on tight bends

Our car's two line sensors sit **on** the line (the gap between them is narrower than the line). The controller was tuned for sensors **either side** of it, so:

- on the line, both sensors read black and the car thought it was at a junction, so it stopped steering;
- on a tight bend it only reacted once a sensor had nearly left the line, so it ran off the outside.

v3.5 adds an **INSIDE layout** and picks the layout automatically at START (both black = INSIDE, both white = STRADDLE, otherwise `LINE_LAYOUT_DEFAULT` in `car_config.h`).

In INSIDE mode:
- both black = centred;
- both white = off the line, always turn back toward the side it was last seen;
- steering is stronger (KP 1.5, KD 0.3).

Simulator, sensors on a 25 mm line, tight bends of 60 to 120 mm radius: the old tuning passed 152 to 202 of 216 runs, the new one 210 to 216 of 216.

Barcode turns in INSIDE mode can't see a junction (both sensors are always black on the line), so a left/right barcode turns **`TURN_AFTER_MM` (150 mm, in `mission.c`) after the barcode**. Measure that on your course.

**Put the car centred on the line before START.**



## What's new in v3

| Buddy | Folder | What it does |
|---|---|---|
| 1 Comms | `subsystem1/` | MQTT telemetry (binary frames, no JSON), remote START / STOP / PAUSE / RESUME / PING, shared fault bitmask, status LED + console |
| 2 Motion | `subsystem2/` | motors, encoders, hybrid control, **odometry** (for hump height and telemetry), encoder fault bit |
| 3 Line + barcode | `subsystem3/` | line following; **Code 39 barcodes** on GP2 (A = left, B = right at the next junction, C = U-turn, D = end of course) |
| 4 IMU | `subsystem4/` | **GY-511** over I2C0 (GP4/GP5): pitch, roll, compass, **hump height**, **impact**, **tilt** (stops the car) |
| 5 Obstacle | `subsystem5/` + `mission.c` | ultrasonic guard, fan sweep, go round, find the line, no-echo fault bit |

Two firmware files:

- `IS27_fresh_v3.uf2`: everything, **no WiFi** (works anywhere).
- `IS27_fresh_v3_mqtt.uf2`: everything **plus WiFi/MQTT**. It was built with placeholder WiFi details, so it will not connect until you build it with yours: copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h`, fill in SSID, password and your laptop's IP, then `make NET=lwip MQTT=1`. Or send me those details and I'll build it.

PC dashboard: `tools/comms/comms_dashboard.py --broker <laptop IP> monitor` (and `send START` etc). It decodes every frame; the status, fault and command frames are the team's original layout, so the old dashboard still works for those.

PC tests (all pass): `testing/barcode_test` (3240 barcodes with speed drift, noise and backwards reads, 0 false reads from 2000 noise frames), `testing/imu_test` (maths vs PC library, hump heights within 15 %), `testing/comms_test` (C packer vs Python decoder, every field), plus `scan_test` and `line_sim` from v2.

Resources now: 8 tasks, 3 event flags, 2 message buffers, 1 cyclic handler, all checked at build time and printed at boot.

---



A new `app_program/` written from scratch on the team's μT-Kernel 3.0 template (the kernel, drivers, makefile and USB console are unchanged). It covers **core driving**: motors and encoders, line following, ultrasonic and servo obstacle avoidance, START/STOP buttons and a console readout. The IMU, barcode and WiFi/MQTT parts are left as hooks.

Wiring is exactly **IS27_Wiring_Guide.md v2**: left encoder on Grove 7, right encoder on Grove 1, 3.3 V ultrasonic on Grove 4, line sensors A0 on Grove 6, servo on GP13, alert LED on GP14 and status LED on GP19.

---

## 1. How to use it

1. Flash `IS27_fresh_v2.uf2`: hold BOOTSEL, plug in the USB cable, then drag the file onto the RPI-RP2 drive.
2. Open a serial terminal at 115200. The boot shows the **resource check** table (section 4).
3. Put the car on the line with **both sensors on white** (straddling the line) and wait 1 s. It learns each sensor's white level while it is stopped.
4. Press **GP20 = START**. Press **GP21 = STOP** at any time to brake and go back to IDLE.

| Status LED (GP19) | Meaning |
|---|---|
| Slow blink (1 Hz) | IDLE, waiting for START |
| On | Following the line |
| Fast blink | Going round an obstacle (the alert LED on GP14 is on too) |
| Double blink | FAULT. The console says why. Press START to try again |

---

## 2. Design

### Tasks

| Task | File | Priority | Core | Runs | Job |
|---|---|---|---|---|---|
| motion | subsystem2/subsystem2.c | 10 | 0 | every 20 ms + on STOP | Only writer of the motor PWM. Ramps, open-loop duty, encoder health, trim |
| mission | mission.c | 11 | 1 | every 20 ms + on events | Buttons, IDLE / FOLLOW / AVOID / FAULT, the bypass manoeuvre |
| line | subsystem3/subsystem3.c | 12 | 1 | every 20 ms | Reads the IR sensors, runs the line controller |
| ranger | subsystem5/subsystem5.c | 13 | 1 | one ping per 60 ms | Ultrasonic, servo, obstacle guard |
| status | subsystem1/subsystem1.c | 20 | 0 | 100 ms LED, 500 ms print | Console line, status LED, keeps the buzzer quiet |

The kernel's own tasks sit above all of these: USB console at priority 3 and WiFi at 4, both on core 0. The motion task **must** be on core 0, because the encoder interrupt is routed to core 0 and the encoder snapshot disables interrupts on that core only.

### How the modules talk (car.h)

- **No mutexes, no message queues.** Each shared value has **one writer** and is a single aligned 32-bit store, which is atomic on the RP2040.
- **One slot per motor owner.** The line task writes slot LINE and the mission writes slot MISSION. The mission picks which slot drives the motors (`motion_set_owner`). A late write from the line task can't overwrite a mission command, because it lands in a different slot.
- **STOP always wins.** `motion_stop()` sets the owner to NONE, brakes with one register store (A = B = 0) and wakes the motion task through `motion_flg`. Driving again needs the mission to pick an owner again.
- **Events.** The ranger and line tasks set bits in `mission_flg`: `EV_OBSTACLE`, `EV_LINE_LOST` and `EV_PING`.
- **One 20 ms cyclic handler (`ctl_tick`)** wakes both the motion task (flag bit) and the line task (`tk_wup_tsk`), so the two control loops run in step with no drift.

### Motors: "hybrid"

- `duty = (D0 + K x speed) x wheel gain + kick + encoder trim`
- **Open loop is always on.** The left gain (1.15) makes up for the weaker left motor.
- **Encoder trim** learns slowly (at most ±15 % duty), and only while **both** encoders are counting and both wheels drive the same way.
- **If an encoder goes silent while its wheel is driven**, it is marked DEAD, its trim is reset to 0 and learning stops. The car keeps driving on the open-loop part. This is why the flaky left encoder can no longer make the car spin.

### Line following (subsystem3/line_ctl.c)

The controller is pure C with no kernel calls, so the **same file** runs in the PC simulator (section 5).

| Mode | When | What it does |
|---|---|---|
| track | a sensor sees the line | steer = 1.0 x pos + 0.1 x change. If the line went right under a sensor ("deep") and is now fading, keep at least 45 % steer, because on a curve the line is sliding outside the sensor |
| centred | both white after a small wobble | small hold toward the last side for 0.2 s, then straight |
| recover | both white after a deep excursion | turn toward the line, 45 % rising to 90 % over 0.6 s, at the slow speed |
| junction | both black | straight on |
| lost | recover for 2.5 s without finding the line | stop, FAULT "line lost" |

- **Speed** is 120 mm/s on straights, dropping toward 70 mm/s as the steering gets harder.
- **Overshoot guard:** if a recover ends with the line on the **other** sensor, it was an overshoot and not a curve. The next excursion is then treated as small, which stops a left-right pivot loop.

### Obstacle: go round and find the line again (mission.c)

Every step uses the **sensors and timing, not wheel distance**. The encoder-measured turns are what made the old code circle.

1. **Guard:** while following, 3 pings in a row under **10 cm**, with the sensor straight ahead, means brake.
2. **Confirm:** wait 300 ms and take 3 pings. If the median is over 25 cm, it was a false alarm: carry on following.
3. **Fan scan:** the servo swings the ultrasonic like a fan, from 80° left to 80° right in 20° steps and back in between. That gives one reading every 10°, 17 in all, in about 3 s. `scan_profile()` turns the sweep into:
   - the obstacle's **distance** and **bearing**;
   - its **width** and the sideways position of its **left and right edges** (pulled in by 8° because the sound cone makes objects look wider);
   - whether the sweep **saw past** each edge (a side is only "open" if it did);
   - the **sideways move** needed on each side: edge + half the car + 5 cm.

   It picks the open side that needs the smaller move. If neither side is open, it waits until the obstacle is gone. If nothing is closer than 100 cm after all, it carries on following.
4. **Out:** spin away until the front is clear, plus 150 ms. The spin time `t_out` stands for the angle turned.
5. **Widen:** drive straight out far enough to clear that edge: time = sideways move / sin(turn angle) / speed. The turn angle is estimated from the spin time (`AV_SPIN_DPS`).
6. **Align:** spin back for `t_out`. The car is now parallel to the line.
7. **Pass:** drive on with the servo at 80° toward the obstacle. When it has gone by, drive 0.6 s more for the car's length.
8. **Back:** spin toward the line for `t_out`.
9. **Seek:** drive until a line sensor sees black. Timeout 4 s gives FAULT.
10. **Rejoin:** spin for `t_out` to face along the line, then follow, starting with "the line is on this side, turn to it".

---

## 3. Files

| File | What |
|---|---|
| `car_config.h` | **Every** pin, PWM slice, priority, core, period and object count, plus the compile-time checks |
| `car.h` | The interfaces between modules |
| `app_main.c`, `app.h` | Template entry point, now also calling `mission_init/start` |
| `gpio_irq.c/.h` | The team's shared GPIO interrupt dispatcher (unchanged) |
| `resources.c` | Boot-time resource check |
| `mission.c` | Buttons, states, avoidance |
| `subsystem1/` | Status task (WiFi/MQTT hook later) |
| `subsystem2/` | `drv_motor.c`, `drv_encoder.c`, `subsystem2.c` (motion task), tunables in `subsystem2.h` |
| `subsystem3/` | `line_ctl.c` (pure controller), `subsystem3.c` (ADC + line task) |
| `subsystem4/` | IMU hook only |
| `subsystem5/` | `subsystem5.c` (ultrasonic, servo, ranger task), `scan_profile.c` (fan sweep to obstacle width and edges, pure logic), tunables in `subsystem5.h` |
| `testing/line_sim/line_sim.c` | PC simulator for the line controller |
| `testing/scan_test/scan_test.c` | PC test for the fan sweep: fake boxes, checks width and side |

---

## 4. Resource allocation

### Checked when you build (car_config.h)

The build **fails with a message** if any of these is broken. Each check was tested by breaking it on purpose.

- No GPIO used twice; no Pico W radio pin (GP23, 24, 25, 29)
- Line sensors on ADC pins (GP26 to GP28)
- Motor, servo and echo pins on their claimed PWM slices; no slice with two owners; echo on a **B** input (only B can gate the counter)
- Every task priority below the kernel's 1 to 4, in the order motion/mission > line > ranger > status
- Every period a whole number of 10 ms kernel ticks; ping period at least 60 ms
- Our kernel objects plus the kernel's own fit under every `CNF_MAX_*` in `config/config.h`

### Budget

| Object | Ours | Kernel's own (reserved) | Limit |
|---|---|---|---|
| Tasks | 5 | 3 (init, USB console, WiFi) | 32 |
| Event flags | 2 (`motion_flg`, `mission_flg`) | 4 (device lock, serial x2, ADC) | 16 |
| Cyclic handlers | 1 (`ctl_tick`) | 0 | 12 |
| Semaphores | 0 | up to 4 (one lock per device) | 16 |
| Mutex / mailbox / msg buffer / pools / alarms | 0 | 0 | 12 each |
| Task stacks | 5 x 2 KB = 10 KB | | v47 created over a dozen tasks |

| Hardware | Owner |
|---|---|
| PWM slice 0 | ultrasonic echo width counter (GP17 = input B) |
| PWM slices 4, 5 | left and right motor |
| PWM slice 6 | servo (GP13) |
| ADC0, ADC1 | line sensors, line task only |
| IRQ 13 (GPIO bank) | shared dispatcher: encoders GP0, GP1, GP7, GP28 |

### Checked at boot (resources.c)

The firmware asks the kernel which object IDs **really exist** (`tk_ref_tsk`, `tk_ref_flg` and so on) and prints, for example:

```
=== IS27 resource check (kernel's own view) ===
  tasks             8 used (ours  5, kernel  3) of  32  OK
  event flags       6 used (ours  2, kernel  4) of  16  OK
  ...
=== ALL WITHIN LIMITS ===
```

**Please send us this table from the first boot.** It is the evidence that the allocation is valid on the real board.

---

## 5. Simulator results (line controller)

`testing/line_sim/line_sim.c` drives a model of the car around 5 tracks: a 3 m straight, an R300 oval, mixed R250/R200/R150 curves, S-bends and tight R120/R100 bends. Each track is run 36 times, with:

- sensor spacing of 24, 40 or 60 mm;
- sensors 50 or 80 mm ahead of the axle;
- ±10 % motor mismatch;
- the car starting centred, or 8 mm off and 6° skewed.

| Conditions (motor deadband, lag, sensor blur, delay, speed) | New controller | v47-style controller |
|---|---|---|
| 20 mm/s, 0.08 s, 3 mm, 1 tick, 120 mm/s | 180/180 | 176/180 |
| 40 mm/s, 0.15 s, 5 mm, 1 tick, 120 mm/s | 180/180 | 180/180 |
| 50 mm/s, 0.20 s, 5 mm, 2 ticks, 150 mm/s | **177/180** | **89/180** |
| 60 mm/s, 0.25 s, 6 mm, 2 ticks, 120 mm/s | 179/180 | 180/180 |

- The v47-style controller fails on curves once the motors get sluggish, and still passes on straights. That is the same pattern the car showed.
- The simulator is a model, not the mat. Tune on the real track with the values in section 6.

Run it yourself:

```
gcc -O2 -I../../app_program/subsystem3 line_sim.c ../../app_program/subsystem3/line_ctl.c -lm -o line_sim
./line_sim 2          # 2 = also list each failing case
```

---

### Fan sweep test (scan_test.c)

Fake sweeps of boxes 10 to 25 cm away, 6 to 60 cm wide, centred or off to one side, with a 15° sound cone and ±1 cm noise. Each box was run twice, the second time with one random missed echo per sweep. All 18 cases passed:

- the width estimate was within half the true width plus 3 cm;
- the car chose the shorter way round (either side for a centred box);
- the sideways move always cleared the **true** edge plus half the car.

```
gcc -O2 -I../../app_program/subsystem5 scan_test.c ../../app_program/subsystem5/scan_profile.c -lm -o scan_test
./scan_test
```

## 6. Tuning knobs

| What | Where | Default | Change it when |
|---|---|---|---|
| Left motor gain | `MOT_GAIN_L_X1000` (subsystem2.h) | 1150 | the car drifts on a straight: drifts left, raise it; drifts right, lower it |
| Line speed | `.speed` (line_ctl.c) | 120 mm/s | it runs well and you want it faster |
| Curve speed | `.speed_min` | 70 | inner wheel stalls in bends: raise to 80 or 90 |
| Steering strength | `.kp_x1000` | 1000 | wobbles on straights: 800; too lazy in curves: 1200 |
| "Sees the line" level | `.see` | 1200 | compare with the console `IR L raw/dark` values |
| Obstacle stop distance | `GUARD_STOP_CM` (subsystem5.h) | 10 cm | |
| Spin speed | `AV_SPIN_MMS` | 110 | spins are jerky: 130 |
| Spin rate estimate | `AV_SPIN_DPS` | 90 °/s | the widen leg is too short (clips the box): lower it; too long: raise it |
| Passing room | `AV_MARGIN_CM` | 5 cm | the car passes too close: raise it |
| Car half width | `AV_HALF_CAR_CM` | 9 cm | measure the car and set half its width |
| Sound cone correction | `AV_BEAM_DEG` | 8° | printed width is much bigger than the real box: raise it; smaller: lower it |
| Servo direction | `SERVO_REVERSE` | 0 | the scan says "left" but the sensor looks right |
| Sensor polarity | `LINE_BLACK_IS_HIGH` (car_config.h) | 1 | `dark` goes **down** over the black line |

---

## 7. Test plan, in this order

1. **Boot table:** the resource check says ALL WITHIN LIMITS. Send the table.
2. **Encoders:** with the car stopped, turn each wheel by hand. `enc L ok` / `enc R ok` counts go up. `noise` stays small.
3. **IR:** move the car across the line by hand. `IR L raw/dark` and `R` change, and `pos` goes negative with the line under the left sensor and positive under the right.
4. **Ultrasonic:** `US __ cm @ 0` matches a box at 10, 30 and 50 cm.
5. **Straight:** START on a long straight. If it drifts, change `MOT_GAIN_L_X1000` in steps of 50.
6. **Curves:** run the circuit. Send the console log of any place where it leaves the line. The second console line shows the mode (track / centred / recover).
7. **Fan scan, wheels lifted:** lift the car so the wheels spin freely, hold a box of known width (say 15 cm) about 8 cm in front of the sensor and press START. The car stops itself and sweeps. Press STOP once the `OBSTACLE` line appears. Check the `SCAN deg:cm` line and the `OBSTACLE ... width __ cm` line against the real box. Adjust `AV_BEAM_DEG` until the width is close.
8. **Obstacle on the track:** put the box on a straight part. Watch `MISSION going round on the LEFT/RIGHT`, `turned out __ ms (~__ deg), widen __ ms`, `passed`, `FOLLOW`. Tune `AV_SPIN_DPS` and `AV_PASS_EXTRA_MS` first.
9. **STOP:** press GP21 during each phase. The car must brake at once every time.

---

## 8. Known limits

- **Not run on the car yet.** It builds cleanly (also with `-Wall -Wextra`), the compile-time checks are proven, and the line controller is simulator-tested. The avoidance timings are first guesses for the mat.
- **The bypass spins are timed**, so they depend on battery charge. A fresh battery turns further. If this becomes a problem, the IMU (Buddy 4) is the proper fix for measuring the angle.
- **"Line lost" only triggers after a lost curve.** It does not trigger on a straight, because a well-centred car can see white on both sensors for metres. Set `.blind_ms` to 8000 if you want a timeout there too.
- **Long `tm_printf` lines briefly mask interrupts on core 0.** This is fine at our speed. If the encoder `noise` climbs, shorten the print.
- **Hooks only:** barcode (GP2), IMU (GP3, GP4, GP5) and WiFi/MQTT. Their pins are claimed in `car_config.h`. `subsystem4.c` lists what to change when the IMU is added.
