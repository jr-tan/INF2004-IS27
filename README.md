# IS27 robot car (INF2004)

Firmware for the IS27 line-following robot car: a **Raspberry Pi Pico W**
on a Robo Pico board, running **micro T-Kernel 3.0 on both RP2040 cores**.

The car follows a black line, reads **Code 39 (Full ASCII) barcodes** and
acts on them, goes round **obstacles** with an ultrasonic sensor on a
servo, handles **humps and ramps**, safe-stops if it is **tipped over or
picked up**, and sends **telemetry over WiFi/MQTT** to a desktop GUI that
can also send it commands.

Who owns what (pins, tasks, interfaces) is in **[`TEAM.md`](TEAM.md)**.

---

## 1. What the car does

| Feature | How | Code |
|---|---|---|
| Line following | 2 analog IR sensors, PD steering; layout (INSIDE / EDGE / STRADDLE) picked at START from where the sensors are | `subsystem3/line_ctl.c`, `subsystem3.c` |
| Barcodes | separate IR sensor beside the line (D0 on GP2), Code 39 Full ASCII, read forwards or backwards; each letter mapped to an action | `subsystem3/barcode.c`, `code39.c` |
| Barcode actions | `L` left at the next junction, `R` right, `U` U-turn, `S` stop, `-` report only. Set from the GUI ("Barcode map") or `BC_ACTION_MAP` in `car_config.h` | `mission.c` |
| Obstacles | guard stops at 25 cm; servo sweep picks a side; the car turns on the spot to find **both edges**, goes round the shorter side, watches the obstacle while passing, looks before turning back, re-finds the line | `mission.c`, `subsystem5/` |
| Ramps | climb mode: steady speed, no pivoting, stall boost; backs off for a run-up if stuck | `mission.c`, `subsystem2/` |
| Safety | tipped over (> 35 deg) or picked up (IMU) -> brake, SAFE_STOP; STOP button works in every state | `subsystem4/`, `mission.c` |
| Telemetry | binary MQTT frames: status, motion, line/barcode, terrain, obstacle, faults, resources | `subsystem1/`, `docs/buddy1_comms.md` |
| Remote commands | START, STOP, PAUSE, RESUME, PING, U-TURN, REVERSE, line tuning, barcode map | `subsystem1/subsystem1.c` |

---

## 2. Repository layout

| Folder | What is in it |
|---|---|
| `app_program/` | **the car's code.** `app_main.c` (entry point), `mission.c` (state machine), `car.h` (interfaces), `car_config.h` (pins, priorities, budgets), `subsystem1/` … `subsystem5/` |
| `config/` | kernel configuration, and `wifi_secrets.example.h` (copy it, see 3.2) |
| `build_make/` | makefiles; the firmware is built here |
| `tools/comms/` | desktop GUI (`is27_gui.py`), text dashboard, frame codec, Mosquitto config |
| `tools/elf2uf2/` | source of the ELF -> UF2 converter (the build compiles it for your computer) |
| `tools/barrc_check.py` | automated BARR-C rule check over `app_program/` (see 7) |
| `testing/` | PC tests: barcode decoder, comms frames, IMU maths, line simulator, obstacle scan |
| `docs/` | `buddy1_comms.md` (comms design notes), `barr_c_conformance.md` (coding standard and Change Requirement 1), the BARR-C:2018 standard (PDF) and the Week 8 Tribunal 1 document (docx) |
| `kernel/`, `include/`, `device/`, `lib/`, `etc/` | micro T-Kernel 3.0, its drivers, the WiFi / lwIP / MQTT glue |

---

## 3. Build the firmware

Needs a computer with **macOS, Linux, or Windows + WSL (Ubuntu)**. On
Windows run every command in the WSL terminal.

### 3.1 One-time setup: Pico SDK and compiler

```sh
# basic tools (Linux / WSL)
sudo apt update && sudo apt install -y git make g++ gcc-arm-none-eabi
#   macOS: xcode-select --install, and the Arm GNU Toolchain
#   (AArch32 bare-metal, arm-none-eabi) from developer.arm.com

# Pico C SDK, with the USB, WiFi and lwIP submodules
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip

# tell the build where the SDK is (add to ~/.bashrc to keep it)
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
arm-none-eabi-gcc --version     # must print a version
```

### 3.2 WiFi and MQTT broker settings

```sh
cp config/wifi_secrets.example.h config/wifi_secrets.h
```

Edit `config/wifi_secrets.h`: `WIFI_SSID`, `WIFI_PASSWORD`, and
`MQTT_BROKER_IP` = **the IP address of the laptop running Mosquitto** on
that network (`ipconfig` / `ip addr`). The Pico W only joins **2.4 GHz**
networks (on an iPhone hotspot, turn on *Maximise Compatibility*).

`wifi_secrets.h` is in `.gitignore`: **never commit it**.

### 3.3 Build

```sh
cd build_make
make -j8 NET=lwip MQTT=1
```

The firmware is **`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`**.
Without `NET=lwip MQTT=1` the car still drives, but sends no telemetry.

If `make` says `arm-none-eabi-gcc not found`, fix your `PATH`; if it says
`PICO_SDK_PATH is not set`, export it (3.1). Changing the WiFi settings
needs a rebuild.

Other options (`SMP=0` single core, `CONSOLE=uart`, `CRASH_REPORT=1` for
crash output on GP0/GP1) are listed at the top of `build_make/pico_rp2040.mk`.
**Note:** `CRASH_REPORT=1` and `CONSOLE=uart` use GP0/GP1, which this car
uses for the right wheel encoder.

### 3.4 Flash

1. Hold **BOOTSEL** on the Pico W and plug in the USB cable. A drive
   called **RPI-RP2** appears.
2. Copy the `.uf2` onto it (WSL: `explorer.exe .` in `build_make`). The
   Pico reboots by itself.

---

## 4. Run the car

1. Put the car **on the line, still and level**, and wait a second (the
   line sensors learn the white level while parked):
   - sensors closer together than the tape is wide: **both on the line** (INSIDE, smoothest)
   - otherwise: **one sensor on the line, one beside it** (EDGE)
2. Press **START** (GP20) or START in the GUI. **STOP** (GP21) brakes at
   once, in any state.

USB console (115200 baud; PuTTY on Windows, `screen /dev/ttyACM0 115200`
on Linux) prints the boot resource table, the line layout picked at START,
every barcode, every obstacle step and every fault. While the car is
**IDLE** it also prints an ultrasonic bench line every 2 s
(`RANGE bench: median .. min .. max ..`) for checking the sensor.

---

## 5. Desktop GUI and MQTT broker

On the laptop whose IP is in `wifi_secrets.h`:

```sh
pip install paho-mqtt
cd tools/comms
mosquitto -c mosquitto.conf -v          # window 1: the broker (port 1883)
python3 is27_gui.py --broker 127.0.0.1  # window 2: the GUI
python3 is27_gui.py --demo              # GUI without a car (simulated data)
```

- **Windows:** `& "C:\Program Files\mosquitto\mosquitto.exe" -c mosquitto.conf -v`.
  If port 1883 is busy, stop the Mosquitto service (`net stop mosquitto`,
  as Administrator). Allow TCP 1883 through the firewall once:
  `New-NetFirewallRule -DisplayName "Mosquitto 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow`
- The GUI has a tab per dataset (motion, line + barcode, terrain,
  obstacle, faults, resources, raw frames), **Export CSV**, the command
  buttons, live line tuning and the **Barcode map** box
  (e.g. `A=L B=R C=U D=S`). See `tools/comms/README_GUI.md`.

---

## 6. Settings you are likely to change

| What | Where |
|---|---|
| WiFi / broker IP | `config/wifi_secrets.h` |
| Pins, task priorities, barcode wiring switch, default barcode map | `app_program/car_config.h` |
| Motor power balance (`MOT_GAIN_L/R_X1000`), wheel track, speed control | `app_program/subsystem2/subsystem2.h` |
| Line speed default | `LINE_SPEED_DEFAULT` in `app_program/car.h` (also live from the GUI) |
| Obstacle: stop distance, car size, beam correction (`AV_CONE_DEG`), margins | `app_program/subsystem5/subsystem5.h` |
| Climb mode thresholds, distance from barcode to turn (`TURN_AFTER_MM`) | `app_program/mission.c` |
| Tilt / pick-up thresholds | `app_program/subsystem4/subsystem4.c` |

Measured on this car (12 Nov): width with wheels 12.5 cm, wheel track
10.8 cm, ultrasonic 8.5 cm ahead of the axle. The servo turns only
+-25 deg (a brace is in the way).

---

## 7. Coding standard

`app_program/` follows the **BARR-C:2018 Embedded C Coding Standard**
(`docs/barr_c_coding_standard_2018.pdf`) and the Tribunal 1 **Change
Requirement 1** (shared data packed into words with one writer each).
What was applied, the documented deviations and how behaviour was
checked are in **`docs/barr_c_conformance.md`**.

After editing a file:

```sh
clang-format --style=file -i app_program/<file>.c   # app_program/.clang-format
python3 tools/barrc_check.py app_program            # rules a tool can check
```

clang-format can't line `break;` up with its `case` (rule 8.3.a): do that
by hand. `.gitattributes` keeps C files on LF line endings (rule 3.6.a).

## 8. PC tests

Each test compiles the car's own C file on your computer:

```sh
cd testing/barcode_test
gcc -O2 -I../../app_program/subsystem3 barcode_test.c ../../app_program/subsystem3/code39.c -o barcode_test && ./barcode_test
gcc -O2 -std=c99 -I../../app_program/subsystem3 barcode_sizes_test.c ../../app_program/subsystem3/code39.c -lm -o sizes && ./sizes
```

| Folder | Tests |
|---|---|
| `testing/barcode_test/` | Code 39 decoder: random codes with speed drift and noise; the course barcodes A-Z at 17 / 25 / 34 cm print sizes; Full ASCII |
| `testing/comms_test/` | C frame packer vs the Python decoder (`test_frames.py`) |
| `testing/imu_test/` | pitch / roll / hump maths |
| `testing/line_sim/` | line follower on simulated bends |
| `testing/scan_test/` | ultrasonic sweep -> obstacle profile |

---

*micro T-Kernel 3.0 (c) Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. The dual-core RP2040 port is derived from the official
uT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
