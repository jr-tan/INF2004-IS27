/*
 *	car_config.h
 *	IS27 robot car: ONE place for every pin, PWM slice, task priority,
 *	core, period and kernel object count.  Change values here only.
 *
 *	The bottom half of this file checks the allocation at COMPILE time:
 *	  - no GPIO is used twice, no radio pin (GP23/24/25/29) is used
 *	  - analog inputs are on ADC-capable pins (GP26..GP28)
 *	  - motor / servo / ultrasonic pins sit on the PWM slices we claim
 *	    and no PWM slice has two owners
 *	  - every task priority is below the kernel's own tasks (1..4)
 *	  - all periods are whole multiples of the 10 ms kernel tick
 *	  - kernel objects (ours + the kernel's own) fit in config.h limits
 *	resources.c checks the same thing again at RUN time by asking the
 *	kernel which objects really exist, and prints the table at boot.
 *
 *	Wiring: IS27_Wiring_Guide.md (v2).
 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <config.h>		/* CNF_MAX_* kernel limits, CNF_TIMER_PERIOD */

/* =====================================================================
 *  1. Pins (GPIO numbers)
 * ===================================================================== */
/* Motors: Robo Pico MOTOR 1 = left, MOTOR 2 = right */
#define PIN_MOTOR_L_A		8	/* M1A  PWM slice 4 A */
#define PIN_MOTOR_L_B		9	/* M1B  PWM slice 4 B */
#define PIN_MOTOR_R_A		10	/* M2A  PWM slice 5 A */
#define PIN_MOTOR_R_B		11	/* M2B  PWM slice 5 B */
#define MOTOR_L_INVERT		0	/* 1 = swap A/B in software */
#define MOTOR_R_INVERT		1	/* right motor is mounted mirrored */

/* Wheel encoders (quadrature, inside the motors) */
#define PIN_ENC_L_A		7	/* Grove 7 */
#define PIN_ENC_L_B		28	/* Grove 7 (digital use of GP28) */
#define PIN_ENC_R_A		0	/* Grove 1 */
#define PIN_ENC_R_B		1	/* Grove 1 */
#define ENC_L_REVERSE		0	/* left counts up going forward */
#define ENC_R_REVERSE		1	/* right counts down going forward */

/* Line sensors (A0 outputs) */
#define PIN_LINE_L		26	/* Grove 6, ADC0 */
#define PIN_LINE_R		27	/* Grove 6, ADC1 (power from Grove 5) */
#define LINE_BLACK_IS_HIGH	1	/* 0 if your boards read LOW on black */
/* Where the two line sensors sit when the car is on the line:
     0 = STRADDLE: either side of the line (both on white)
     1 = INSIDE:   both over the line (gap narrower than the line)
     2 = EDGE:     one on the line, one on the white beside it
   At START the firmware looks: both black -> INSIDE, both white ->
   STRADDLE, one black + one white -> EDGE (IS27 car from 3 Nov).
   This default is only used before the first START. */
#define LINE_LAYOUT_DEFAULT	2
#define LINE_PIVOT_DEFAULT	0	/* bends: 1 = stop, then pivot on the inner wheel
				   (left bend: left wheel stops, right drives);
				   0 = steer through.  Dashboard can switch it.
				   Off: in line_sim the EDGE layout at 30 mm/s
				   passes every bend by steering alone. */

/* Ultrasonic (3.3 V type) + servo + alert LED */
#define PIN_US_TRIG		16	/* Grove 4, plain GPIO output */
#define PIN_US_ECHO		17	/* Grove 4, PWM slice 0 B (hardware timing) */
#define PIN_SERVO		13	/* servo header GP13, PWM slice 6 B */
#define PIN_ALERT_LED		14	/* servo header GP14 (S) via 220 R */

/* Onboard parts */
#define PIN_STATUS_LED		19	/* comms / status LED via 330 R */
#define PIN_BTN_START		20	/* onboard button, active low */
#define PIN_BTN_STOP		21	/* onboard button, active low */
#define PIN_BUZZER		22	/* onboard buzzer: PWM slice 3 A (tones, board_io.c) */
#define PIN_RGB			18	/* onboard 2 x WS2812 RGB LEDs, driven by PIO0 SM3 */
#define RGB_COUNT		2

/* Barcode + IMU */
#define PIN_BARCODE		2	/* Grove 2, barcode sensor D0 (edges via gpio_irq) */
#define BARCODE_BLACK_IS_HIGH	1	/* 0 if D0 reads LOW on black */
/* DEMO DAY: what each letter does, one character per letter A..Z
   (lower case letters act like upper case):
     L = turn left at the next junction    R = turn right
     U = U-turn                            S = stop (end of course)
     - = just report it
   e.g. A = left, B = right, C = U-turn, D = stop, rest report only: */
#define BC_ACTION_MAP		"LRUS----------------------"
/*                             ABCDEFGHIJKLMNOPQRSTUVWXYZ */
#define PIN_IMU_INT		3	/* Grove 2, GY-511 INT1: claimed, IMU is polled */
#define PIN_IMU_SDA		4	/* Grove 3, I2C0 SDA */
#define PIN_IMU_SCL		5	/* Grove 3, I2C0 SCL */
#define I2C_IMU			0	/* GP4/GP5 = I2C0 */

/* =====================================================================
 *  2. PWM slices (RP2040 has 8: slice = (pin / 2) % 8, A = even pin)
 * ===================================================================== */
#define SLICE_US		0	/* echo width counter (B input gated) */
#define SLICE_MOTOR_L		4
#define SLICE_MOTOR_R		5
#define SLICE_SERVO		6
#define SLICE_BUZZER		3	/* GP22 channel A: tones */

/* =====================================================================
 *  3. Tasks: priority (1 = highest), core, period
 *     Kernel tasks: init 1, USB console 3 (core 0), WiFi 4 (core 0).
 * ===================================================================== */
#define PRI_MOTION		10	/* core 0: owns PWM, reads encoders  */
#define PRI_MISSION		11	/* core 1: buttons, modes, avoidance */
#define PRI_LINE		12	/* core 1: IR sensors -> steering    */
#define PRI_RANGER		13	/* core 1: ultrasonic + servo        */
#define PRI_IMU			14	/* core 1: GY-511 over I2C0, 50 Hz   */
#define PRI_BARCODE		15	/* core 0: decodes barcode edges     */
#define PRI_COMMS		18	/* core 0: telemetry frames, commands */
#define PRI_STATUS		20	/* core 0: console + status LED      */
#define PRI_IDLE		CNF_MAX_TSKPRI	/* both cores: CPU-load meters (lowest) */

#define CORE_MOTION		TP_PRC1	/* must be core 0: encoder ISR + DI */
#define CORE_MISSION		TP_PRC2
#define CORE_LINE		TP_PRC2
#define CORE_RANGER		TP_PRC2
#define CORE_IMU		TP_PRC2
#define CORE_BARCODE		TP_PRC1	/* same core as the edge interrupt */
#define CORE_COMMS		TP_PRC1	/* same core as the WiFi/MQTT task */
#define CORE_STATUS		TP_PRC1

#define CTL_PERIOD_MS		20	/* motion + line loops (cyclic handler) */
#define MISSION_PERIOD_MS	20	/* button poll / state machine tick */
#define PING_PERIOD_MS		60	/* HC-SR04 needs >= 60 ms between pings */
#define ECHO_WAIT_MS		40	/* echo window (max echo ~38 ms) */
#define STATUS_PERIOD_MS	500	/* console print */
#define IMU_PERIOD_MS		20	/* 50 Hz */
#define COMMS_PERIOD_MS		100	/* telemetry 5 Hz, status 1 Hz */

/* =====================================================================
 *  4. Kernel objects created by THIS firmware
 * ===================================================================== */
#define APP_NUM_TASKS		10	/* motion mission line ranger imu barcode comms status idle0 idle1 */
#define APP_NUM_FLAGS		3	/* motion_flg, mission_flg, fault_flg */
#define APP_NUM_CYCS		1	/* ctl_tick (20 ms) */
#define APP_NUM_SEMS		0
#define APP_NUM_MTXS		0
#define APP_NUM_MBFS		2	/* mbf_pub (frames out), mbf_rx (commands in) */
#define APP_NUM_MBXS		0
#define APP_NUM_MPFS		0
#define APP_NUM_MPLS		0
#define APP_NUM_ALMS		0

/* What the kernel and its drivers create for themselves (grep of this
   tree): tasks = init + USB console + WiFi; flags = device-manager lock
   + serial x2 + ADC; semaphores = one lock per registered device.
   Rounded UP so a small kernel change can't silently break the budget.
   resources.c prints the real numbers at boot. */
#define KERNEL_RSV_TASKS	3
#define KERNEL_RSV_FLAGS	4
#define KERNEL_RSV_SEMS		4
#define KERNEL_RSV_OTHER	2	/* per type, safety margin */
#define KERNEL_TOP_PRI		4	/* priorities 1..4 belong to the kernel */

/* =====================================================================
 *  5. Compile-time checks.  A failure here stops the build with the
 *     message in quotes.
 * ===================================================================== */
#define CC_ASSERT(cond, msg)	_Static_assert(cond, msg)
#define BIT64(p)		(1ULL << (p))

/* Every pin configured or claimed by this firmware */
#define USED_PINS_SUM	(BIT64(PIN_MOTOR_L_A) + BIT64(PIN_MOTOR_L_B) + \
	BIT64(PIN_MOTOR_R_A) + BIT64(PIN_MOTOR_R_B) + \
	BIT64(PIN_ENC_L_A) + BIT64(PIN_ENC_L_B) + BIT64(PIN_ENC_R_A) + \
	BIT64(PIN_ENC_R_B) + BIT64(PIN_LINE_L) + BIT64(PIN_LINE_R) + \
	BIT64(PIN_US_TRIG) + BIT64(PIN_US_ECHO) + BIT64(PIN_SERVO) + \
	BIT64(PIN_ALERT_LED) + BIT64(PIN_STATUS_LED) + BIT64(PIN_BTN_START) + \
	BIT64(PIN_BTN_STOP) + BIT64(PIN_BUZZER) + BIT64(PIN_BARCODE) + \
	BIT64(PIN_IMU_INT) + BIT64(PIN_IMU_SDA) + BIT64(PIN_IMU_SCL) + BIT64(PIN_RGB))
#define USED_PINS_OR	(BIT64(PIN_MOTOR_L_A) | BIT64(PIN_MOTOR_L_B) | \
	BIT64(PIN_MOTOR_R_A) | BIT64(PIN_MOTOR_R_B) | \
	BIT64(PIN_ENC_L_A) | BIT64(PIN_ENC_L_B) | BIT64(PIN_ENC_R_A) | \
	BIT64(PIN_ENC_R_B) | BIT64(PIN_LINE_L) | BIT64(PIN_LINE_R) | \
	BIT64(PIN_US_TRIG) | BIT64(PIN_US_ECHO) | BIT64(PIN_SERVO) | \
	BIT64(PIN_ALERT_LED) | BIT64(PIN_STATUS_LED) | BIT64(PIN_BTN_START) | \
	BIT64(PIN_BTN_STOP) | BIT64(PIN_BUZZER) | BIT64(PIN_BARCODE) | \
	BIT64(PIN_IMU_INT) | BIT64(PIN_IMU_SDA) | BIT64(PIN_IMU_SCL) | BIT64(PIN_RGB))
#define RADIO_PINS	(BIT64(23) | BIT64(24) | BIT64(25) | BIT64(29))

/* Sum == OR only if no pin appears twice */
CC_ASSERT(USED_PINS_SUM == USED_PINS_OR, "a GPIO is assigned twice in car_config.h");
CC_ASSERT((USED_PINS_OR & RADIO_PINS) == 0, "GP23/24/25/29 belong to the Pico W radio");
CC_ASSERT(USED_PINS_OR < BIT64(30), "GPIO number out of range");

/* Analog inputs must be ADC pins */
CC_ASSERT(PIN_LINE_L >= 26 && PIN_LINE_L <= 28, "left line sensor needs an ADC pin");
CC_ASSERT(PIN_LINE_R >= 26 && PIN_LINE_R <= 28, "right line sensor needs an ADC pin");

/* PWM pins sit on the slices we claim (slice = (pin >> 1) & 7) */
#define PWM_SLICE_OF(p)		(((p) >> 1) & 7)
CC_ASSERT(PWM_SLICE_OF(PIN_MOTOR_L_A) == SLICE_MOTOR_L && PWM_SLICE_OF(PIN_MOTOR_L_B) == SLICE_MOTOR_L,
	  "left motor pins must be the A/B pair of SLICE_MOTOR_L");
CC_ASSERT(PWM_SLICE_OF(PIN_MOTOR_R_A) == SLICE_MOTOR_R && PWM_SLICE_OF(PIN_MOTOR_R_B) == SLICE_MOTOR_R,
	  "right motor pins must be the A/B pair of SLICE_MOTOR_R");
CC_ASSERT((PIN_MOTOR_L_A & 1) == 0 && (PIN_MOTOR_R_A & 1) == 0, "motor A pins must be channel A (even)");
CC_ASSERT(PWM_SLICE_OF(PIN_SERVO) == SLICE_SERVO, "servo pin is not on SLICE_SERVO");
CC_ASSERT(PWM_SLICE_OF(PIN_BUZZER) == SLICE_BUZZER && (PIN_BUZZER & 1) == 0, "buzzer must be channel A of SLICE_BUZZER");
CC_ASSERT(SLICE_BUZZER != SLICE_US && SLICE_BUZZER != SLICE_MOTOR_L && SLICE_BUZZER != SLICE_MOTOR_R && SLICE_BUZZER != SLICE_SERVO, "buzzer PWM slice already in use");
CC_ASSERT(PWM_SLICE_OF(PIN_US_ECHO) == SLICE_US && (PIN_US_ECHO & 1) == 1,
	  "echo must be the B input of SLICE_US (only B can gate the counter)");
CC_ASSERT(PIN_IMU_SDA == 4 && PIN_IMU_SCL == 5 && I2C_IMU == 0,
	  "IMU I2C pins: GP4/GP5 are I2C0 SDA/SCL (drv_i2c.c assumes I2C0)");
/* No slice has two owners */
CC_ASSERT(((1U << SLICE_US) + (1U << SLICE_MOTOR_L) + (1U << SLICE_MOTOR_R) + (1U << SLICE_SERVO)) ==
	  ((1U << SLICE_US) | (1U << SLICE_MOTOR_L) | (1U << SLICE_MOTOR_R) | (1U << SLICE_SERVO)),
	  "a PWM slice has two owners");

/* Priorities: below the kernel's tasks, inside the configured range */
#define PRI_OK(p)	((p) > KERNEL_TOP_PRI && (p) <= CNF_MAX_TSKPRI)
CC_ASSERT(PRI_OK(PRI_MOTION) && PRI_OK(PRI_MISSION) && PRI_OK(PRI_LINE) &&
	  PRI_OK(PRI_RANGER) && PRI_OK(PRI_IMU) && PRI_OK(PRI_BARCODE) &&
	  PRI_OK(PRI_COMMS) && PRI_OK(PRI_STATUS), "task priority clashes with kernel range 1..4");
CC_ASSERT(PRI_MOTION < PRI_LINE && PRI_MISSION < PRI_LINE && PRI_LINE < PRI_RANGER &&
	  PRI_RANGER < PRI_IMU && PRI_IMU < PRI_BARCODE && PRI_BARCODE < PRI_COMMS &&
	  PRI_COMMS < PRI_STATUS && PRI_STATUS < PRI_IDLE,
	  "priority order: motion/mission > line > ranger > imu > barcode > comms > status");

/* Periods: whole ticks */
#define TICK_OK(ms)	((ms) % CNF_TIMER_PERIOD == 0 && (ms) >= CNF_TIMER_PERIOD)
CC_ASSERT(TICK_OK(CTL_PERIOD_MS) && TICK_OK(MISSION_PERIOD_MS) && TICK_OK(PING_PERIOD_MS) &&
	  TICK_OK(ECHO_WAIT_MS) && TICK_OK(STATUS_PERIOD_MS) && TICK_OK(IMU_PERIOD_MS) &&
	  TICK_OK(COMMS_PERIOD_MS), "period is not a multiple of the kernel tick");
CC_ASSERT(PING_PERIOD_MS >= 60 && ECHO_WAIT_MS < PING_PERIOD_MS, "HC-SR04 ping timing");

/* Kernel object budget */
CC_ASSERT(APP_NUM_TASKS + KERNEL_RSV_TASKS <= CNF_MAX_TSKID, "too many tasks for CNF_MAX_TSKID");
CC_ASSERT(APP_NUM_FLAGS + KERNEL_RSV_FLAGS <= CNF_MAX_FLGID, "too many event flags for CNF_MAX_FLGID");
CC_ASSERT(APP_NUM_SEMS  + KERNEL_RSV_SEMS  <= CNF_MAX_SEMID, "too many semaphores for CNF_MAX_SEMID");
CC_ASSERT(APP_NUM_CYCS  + KERNEL_RSV_OTHER <= CNF_MAX_CYCID, "too many cyclic handlers");
CC_ASSERT(APP_NUM_ALMS  + KERNEL_RSV_OTHER <= CNF_MAX_ALMID, "too many alarm handlers");
CC_ASSERT(APP_NUM_MTXS  + KERNEL_RSV_OTHER <= CNF_MAX_MTXID, "too many mutexes");
CC_ASSERT(APP_NUM_MBFS  + KERNEL_RSV_OTHER <= CNF_MAX_MBFID, "too many message buffers");
CC_ASSERT(APP_NUM_MBXS  + KERNEL_RSV_OTHER <= CNF_MAX_MBXID, "too many mailboxes");
CC_ASSERT(APP_NUM_MPFS  + KERNEL_RSV_OTHER <= CNF_MAX_MPFID, "too many fixed memory pools");
CC_ASSERT(APP_NUM_MPLS  + KERNEL_RSV_OTHER <= CNF_MAX_MPLID, "too many variable memory pools");

#endif /* CAR_CONFIG_H */
