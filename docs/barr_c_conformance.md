# BARR-C:2018 conformance and Change Requirement 1 – IS27 RoboCar

Version: the current `main` source (the "morning Call me daddy climbfix"
build: commit 34d8d5d, the 20 Nov plug-swap wiring files, and the climb
fix), converted to BARR-C:2018.

Scope: all team-written firmware under `app_program/` (42 files).
Out of scope: the micro T-Kernel 3.0 kernel, Pico SDK, lwIP, CYW43 driver
and TinyUSB under `kernel/`, `lib/`, `device/`, `include/` (third-party
code, kept as supplied), and the host-side test harnesses in `testing/`.

## How it was made

1. **Starting point:** the team's BARR-C version of candidate B
   (12 Nov 22:36, `IS27_barrc_source.zip`). It was rebuilt and checked
   first: all 842 functions have the same size as the original candidate B
   build except `comms_task` (0x4F2 -> 0x506 bytes), the one change that
   version declares (rule 8.2.c, see below).
2. **Carried over** everything `main` added since candidate B, written in
   BARR-C style with the same naming: analog barcode input (A0, ADC scan
   interrupt, peak/dip filter), debounce and sensor diagnostics, the
   barcode letter -> action map set from the GUI (`robot/map`,
   `robot/tlm/map`), the `BARCODE_ANALOG` wiring switch, and the climb-mode
   fix (only wheels driven at >= 25 mm/s count as stalled).
3. **Fixed** BARR-C issues the earlier version still had (next section).
4. **Change Requirement 1:** audited every piece of data shared between
   tasks, and packed the one item the Tribunal had agreed but the code
   did not do yet (the line readings; section "Change Requirement 1").

## How behaviour was kept identical

Built with the same toolchain, both with **0 warnings**:

* **Function by function** against the current `main` build: 838
  functions have exactly the same size (783 with the same name, 55
  renamed). Every difference is a function changed on purpose:

  | Function | Why it differs |
  |---|---|
  | `comms_task` | candidate B's rule 8.2.c fix (assignment taken out of the `while` test); robot/map handling as an `if` / `else if` / `else` chain instead of `continue` (1.7.d) |
  | `barcode_task`, `barcode_start`, `barcode_set_map`, `barcode_get_map` | the inline nested `?:` default map became the function `map_boot_default()` (a `switch`), so the compiler inlines differently; one exit per function (6.2.c) |
  | `db_settle`, `next_edge` (inlined) | written with one exit and no `continue` |
  | `comms_frames_map_decode` (was `fr_map_decode`) | one exit (6.2.c) |
  | `line_task` | single-exit `adc_read()`; publishes the packed readings (Change Requirement 1) |
  | `send_telemetry`, `status_task`, `ranger_task` | read the packed line readings instead of the status struct (Change Requirement 1) |
  | `line_raw_word`, `line_dark_word` | new (Change Requirement 1) |

* **Barcode subsystem, both versions side by side on a PC** (a fake clock
  and sensor driving the original and the BARR-C `barcode.c`): 50 random
  tracks (bar sizes 3.8–7.2 mm, 25–31 mm/s, ±20 % speed wobble, noise and
  comparator chatter), for the D0 path and for the A0 path. Every console
  line and every barcode event is identical; 93 of 100 barcodes decode in
  both, and the GUI map gives the same actions.
* **Map frames:** the original and BARR-C encoder/decoder agree on 100 000
  random inputs (33 334 of them valid maps).
* **Host tests give exactly the same output** for the original and the
  BARR-C sources: `testing/barcode_test` (2), `testing/comms_test`,
  `testing/imu_test`, `testing/scan_test`, `testing/line_sim`; and
  `test_frames.py` decodes every frame the BARR-C C code packs, including
  `map_ack`.

## Checking it again after an edit

```sh
# formatting (clang-format 18, rules 1.2, 1.3, 3.1, 3.2, 3.4, 3.5, 3.6)
clang-format --style=file -i app_program/<file>.c
# then line each 'break;' up with its 'case' (8.3.a; clang-format can't)

# the rules a tool can check
python3 tools/barrc_check.py app_program
```

`tools/barrc_check.py` checks line length, tabs, CR-LF, `auto`/`register`,
`goto`, `short`/`long`, `abort`/`exit`/`setjmp`/`longjmp`, backslashes in
comments, the end-of-file marker, file names, include guards and
`for (;;)` infinite loops. All 42 files pass.

## Fixes made in this version

| Rule | What was fixed |
|---|---|
| 2.1.b | 36 `#define` lines whose long trailing comment clang-format had wrapped with `\` continuations (so the comment contained backslashes); each comment now sits on its own lines above its `#define` (code tokens checked unchanged) |
| 2.1.c.ii | the barcode sensor diagnostics are debug output: `BC_DIAG` now follows `NDEBUG` |
| 5.3.b | the A0 filter's arithmetic shifts of signed values became `floor_div()`, which gives the same results without a bitwise operator on signed data |
| 1.8.a | `adc_resync` was exported but used only in `subsystem3.c`: now `LOCAL` |
| 6.1.i | `adc_scan_start()` -> `line_adc_scan_start()` (module prefix) |
| 7.1.e | no new names shorter than 3 characters (`s`, `v`, `h`, `d` in the A0 filter renamed) |
| 8.4.a | magic numbers in new loops and tests became named constants (`ADC_GPIO_CHANNELS`, `ADC_SCAN_WAIT_TRIES`, `BC_MAX_WIDTHS_SHOWN`, `MAP_ACK_TICKS`, `CLIMB_STALL_MIN_MMS`, …) |
| 6.2.c, 1.7.d | new and changed functions have one exit; the comms receive loop no longer uses `continue` |

## Rules applied (whole code base)

| Area | Rule(s) | What was done |
|---|---|---|
| Braces | 1.3.a–d | Allman braces on their own lines; braces added to every `if`, `else`, `for`, `while`, `do` body |
| Parentheses | 1.4.a–b | Each operand of `&&`/`\|\|`, mixed-precedence expressions every `?:` condition, function-like macros and compile-time checks fully bracketed (about 1 150 edits) |
| Abbreviations | 1.5 | Short names expanded (`i`→`idx`, `n`→`num`, `er`→`err`, …); remaining abbreviations listed below |
| Comments | 2.1, 2.2 | `/* */` only; long trailing comments moved above the line; every file and every function has a Doxygen header (`@file`, `@brief`) |
| White space | 3.1, 3.2, 3.3 | One space around binary operators, after keywords and after commas; space between a function name and `(` in declarations and definitions; no space inside parentheses |
| Indentation | 3.4 | 4 spaces per level; `case` labels indented in a `switch` |
| Tabs | 3.5.a | No tab characters anywhere |
| Line length / endings | 1.2.a, 3.6.a | All lines ≤ 80 characters; LF endings; newline at end of file |
| Modules | 4.1, 4.2, 4.3 | Lower-case file names; one header per source file (new: `mission.h`, `resources.h`, `app_main.h`, `drv_motor.h`, `drv_encoder.h`, `barcode.h`, `subsystem1/3/4.h`); include guards; each `.c` includes its own header; prototypes moved out of `.c` files into headers |
| File templates | 4.4 | Every file starts with a `@file` header and ends with `/*** end of file ***/` |
| Types | 5.1, 5.2 | User type names lower case ending `_t` (`enc_snap_t`, `gpio_irq_cb_t`, `scan_profile_t`, …); C99 fixed-width types (`int32_t`, `uint32_t`, `int16_t`, `uint8_t`, `int64_t`) in place of `INT/UINT/W/UW/H/UH/B/UB/long long` |
| Declarations | 8.1.a, 3.3.a | One variable per declaration (struct fields included); one statement per line |
| Functions | 6.1 | Function names lower case with a module prefix (`board_io_`, `comms_frames_`, `comms_fault_`, `drv_i2c_`, `imu_math_`, `scan_profile_`, `motion_`, …); task entry functions end in `_task` (6.4.a); interrupt handlers end in `_isr` (6.5.b); all private functions and module variables `static` via T-Kernel `LOCAL` (1.8.a, 6.2.e); a prototype in the header for every public function (6.2.d) |
| Variables | 7.1 | Lower case, at least 3 characters; prefixes `g_` (file/global scope), `p_`/`pp_` (pointers), `b_` (Boolean), combined as `gp_`, `gb_` (922 renames) |
| `if`/`else` | 8.2.c, 8.2.d, 8.6.a | Every `else if` chain ends in `else`; constant on the left of `==` (`0 == x`, 352 edits); no assignment inside an `if` or loop test |
| `switch` | 8.3.a, 8.3.b | Every `switch` has a `default`; `break` aligned with `case` |
| Loops | 8.4.a–c | Infinite loops written `for (;;)`; no comma operator or extra assignment in a `for` header; named constants for loop bounds (`MOT_WHEELS`, `IMU_AXES`, `I2C_SCAN_FIRST/END`, `HALF_TURN_D10`, `WS2812_PROG_LEN`, `IRQ_REG_COUNT`, `PINS_PER_REG`, `FIRST_OBJ_ID`, `CORE_COUNT`) |

Formatting is fixed by `app_program/.clang-format` (clang-format 18),
so later edits can be re-checked with one command.

## Deviations

| ID | Rule | Deviation | Reason |
|---|---|---|---|
| D-1 | 1.1.a | Built with `-std=gnu11`, not plain C99 | Pico SDK and T-Kernel headers need GNU extensions |
| D-2 | 4.2.a | Project-wide headers `app.h`, `car.h`, `car_config.h`, `comms_port.h` | Shared kernel object IDs, pin map and tuning constants; each module also has its own header |
| D-3 | 6.1 | Subsystem modules use functional prefixes (`motion_`, `line_`, `imu_`, `ranger_`) not the file name | These names are the subsystem interfaces in the design and report; renaming them adds no clarity |
| D-4 | 6.5.a | Interrupt handlers are plain C functions | micro T-Kernel saves context and calls them; there is no `__interrupt` keyword on Cortex-M0+ |
| D-5 | 5.2.a | `INT`/`UINT` kept in five places (`tk_wai_flg` pattern, `line_task` raw/dark arrays, `motion_drive` speed/steer, `motion_task` trim/boost) | Matches the T-Kernel API; `int32_t` (long) here changes the generated code, which would break the identical-binary guarantee of a tested build |
| D-6 | 5.6.a | T-Kernel `BOOL`/`TRUE`/`FALSE` used instead of `bool` | Kernel API returns and expects `BOOL` |
| D-7 | 5.3.c | Some unsigned constants lack a `U` suffix | Values are small and positive; changing them changes no behaviour but was not needed for safety |
| D-8 | 1.6.a | Casts are not each commented | Most casts are kernel API conversions (`(VP)`, `(FP)`) or fixed-point scaling |
| D-9 | 6.2.a | A few functions exceed 100 lines (`mission_task` avoid state machine, `comms_task`, `line_task`) | State machines kept in one place so every transition is visible; splitting would change timing-tested code |
| D-10 | 8.2.b, 6.2.c | Some deep nesting, `continue`, and more than one `return` in a function | Early exits make fault handling clearer; preferred-practice rules |
| D-11 | 1.5.b | Domain abbreviations kept: `imu`, `enc`, `odo`, `mms` (mm/s), `d10` (0.1 deg), `x1000`, `pwm`, `adc`, `rssi`, `mqtt`, `ctl` | Standard in robotics/embedded; listed here as the project's abbreviation table |
| D-12 | 6.3.a | Small function-like macros kept (`ABS`, `CLAMP`, `RW_*`, register-address helpers, `CHECK`) | Used in constant expressions and register addresses or need the caller's line; all follow 6.3.b (every parameter and the whole body bracketed, `do { } while (0)` for statements) |

## Change Requirement 1 (Tribunal 1)

> All shared application data must be packed to minimum bits, meaning
> multiple fields now co-exist in the same uint32_t word, turning
> previously independent writes into read-modify-write operations on
> shared memory across tasks running at different scheduling rates.

Decisions taken at the Tribunal and how the code meets them:

* MQTT application data is **binary**, packed with **explicit masks and
  shifts** at fixed, documented bit positions, **little-endian**
  (`comms_frames_bits_put/get`, layouts at the top of `comms_frames.h`).
  **No C bit-fields** are used anywhere in `app_program/` (checked).
* **One consolidated fault bitmask.** It lives in one kernel event flag
  (`g_fault_flg`); `comms_fault_raise/clear()` are `tk_set_flg` /
  `tk_clr_flg`, which the kernel performs atomically across both cores,
  so no task does its own read-modify-write on the bitmask.
* **One writer per packed word** wherever possible; readers take the whole
  word with one aligned 32-bit load.

| Shared data | Writer | Readers | Packing | Why a reader never sees a torn value | Status |
|---|---|---|---|---|---|
| MQTT frames: status, fault, motion, line, terrain, obstacle, resources, map ack | comms task | GUI (laptop) | bit-packed, LE, explicit masks/shifts (e.g. obstacle 8 B, map 26 x 3 bits) | built in one buffer, sent whole | Met; round trip in `testing/comms_test` |
| Commands in: `robot/cmd` (code 3 + id 5 bits), tuning, `robot/map` (26 x 3 bits) | kernel WiFi task | comms task | bit-packed | message buffer `g_mbf_rx` | Met |
| Fault bitmask (11 bits) | any task | all | one event flag | kernel-atomic set / clear | Met |
| Motion command per owner: left and right speed, 16 bits each | mission or line task | motion task | one `uint32_t` slot per owner | single store | Met |
| Mission -> line command: on 1, side 2, deep 1, generation 8 bits | mission | line task | `g_line_cmd` | single store | Met |
| **Line readings: left 12, right 12, sequence 8 bits** | line task | comms, status, ranger tasks | `g_line_raw_word`, `g_line_dark_word` (`LINE_WORD_*` in `car.h`) | single store each | **Fixed in this version** (was a struct copied field by field) |
| Line "sees the line" bits (2) | line task | mission, comms | `g_see_bits` | single store | Met (telemetry now uses it, not two struct fields) |
| Ranger reading: cm 8, angle 8, sequence 8, valid 1 | ranger task | mission, status | `ranger_word()` | single store | Met |
| Barcode edge ring | D0 interrupt or A0 handler (head) | barcode task (tail) | time + colour per entry | one writer per index | Met |
| Barcode letter -> action map | comms task | barcode task | 3 bits per letter on the wire, one byte per letter in RAM | byte store is atomic | Met |
| Obstacle step ring | mission (head) | comms (tail) | packed into the 8-byte obstacle frame | one writer per index | Met |
| Line tuning requests + generation count | comms task only | line task | separate words + generation | one writer; the line task re-reads them all when the generation changes | Met |
| Status structs: `line_status_t`, `motion_status_t`, `imu_status_t`, `barcode_info_t` | one owner task each | other tasks (copies) | structs | see CR-D1 | Deviation CR-D1 |

**CR-D1.** The status structs stay structs. A copy may mix two updates,
which every getter's comment already says. The decisions taken from them
read **single aligned 32-bit fields** (line `side`; IMU `pitch` and `roll`,
checked against separate thresholds; per-wheel `healthy`, `tgt_mms`,
`pos`), which cannot tear; a mixed pair there changes nothing. Packing
them would change data the mission logic was tested with, for no safety
gain.

