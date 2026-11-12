/*
 *	car.h
 *	The interfaces between the car's modules.  Every .c file includes
 *	this one header.
 *
 *	Data flow (who writes -> who reads)
 *	-----------------------------------
 *	  line task   --(slot LINE)-->     motion task  --> motors
 *	  mission     --(slot MISSION)-->  motion task
 *	  mission     --motion_set_owner-> motion task  (picks the slot)
 *	  mission     --line_follow()-->   line task    (enable + seed)
 *	  mission     --ranger_aim()-->    ranger task  (servo angle)
 *	  ranger/line --EV_* bits-->       mission      (mission_flg)
 *	  everyone    <--*_get()--         one 32-bit word or a copy
 *
 *	Rules that make this race-free without mutexes
 *	  - every shared word has ONE writer and is a single aligned 32-bit
 *	    store (atomic on the RP2040); readers take one load
 *	  - each driver of the motors has its OWN slot, so a late write
 *	    from the line task can never overwrite a mission command
 *	  - motion_stop() can be called by anyone, at any time; it wins
 *	    because it switches the owner to NONE and wakes the motion task
 */
#ifndef CAR_H
#define CAR_H

#include "app.h"
#include "car_config.h"

/* Small helpers */
#define ABS(x)		((x) < 0 ? -(x) : (x))
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))
IMPORT UW now_us(void);	/* 1 MHz timer, low 32 bits: wraps every 71 min, differences stay right */
IMPORT UW now_ms(void);	/* from the full 64-bit timer: wraps after 49 days, differences stay right */

/* ---------------------------------------------------------------------
 *  Motion (subsystem2)
 * --------------------------------------------------------------------- */
#define MOT_OWNER_NONE		0	/* braked */
#define MOT_OWNER_LINE		1
#define MOT_OWNER_MISSION	2
#define MOT_NUM_OWNERS		3

/* Arc drive: speed in mm/s (>= 0), steer -1000..+1000 (+ = right).
   |steer| 500 stops the inner wheel (pivot on it). */
IMPORT void motion_drive(UINT owner, INT speed_mms, INT steer_pm);
/* Direct wheel speeds in mm/s, -1000..+1000 (negative = backwards) */
IMPORT void motion_wheels(UINT owner, INT left_mms, INT right_mms);
/* Which slot drives the motors.  Only the mission task calls this. */
IMPORT void motion_set_owner(UINT owner);
/* Brake now (both inputs low) and latch: owner becomes NONE. */
IMPORT void motion_stop(void);

typedef struct {
	UINT	owner;
	INT	tgt_mms[2];	/* after the ramp */
	UINT	duty_pm[2];	/* 0..1000 */
	INT	trim_pm[2];	/* encoder correction */
	INT	v_mms[2];	/* measured, 0 if encoder unhealthy */
	UW	count[2];	/* encoder steps (always increasing) */
	W	pos[2];		/* signed steps */
	UW	glitch[2];
	BOOL	healthy[2];
	BOOL	trim_on;
} MOTION_STATUS;
IMPORT void motion_get_status(MOTION_STATUS *st);	/* debug copy */

/* ---------------------------------------------------------------------
 *  Line (subsystem3)
 * --------------------------------------------------------------------- */
/* line_follow(TRUE, side, deep): line task drives slot LINE.
   side = -1 left / +1 right / 0: where the line was last seen.
   deep = TRUE: the line is OUTSIDE that sensor (turn hard toward it). */
IMPORT void line_follow(BOOL on, INT seed_side, BOOL seed_deep);

#define LINE_M_OFF		0	/* not following */
#define LINE_M_TRACK		1	/* a sensor sees the line */
#define LINE_M_CENTRED		2	/* both white, line between them */
#define LINE_M_RECOVER		3	/* line slid outside: turning back */
#define LINE_M_JUNCTION		4	/* both black: straight on */
#define LINE_M_LOST		5
#define LINE_M_BENDSTOP		6	/* bend found: braking before the pivot */
#define LINE_M_PIVOT		7	/* turning on the inner wheel */

typedef struct {
	UINT	raw[2];		/* ADC 0..4095 (left, right) */
	UINT	dark[2];	/* white level removed, 0..4095 */
	BOOL	see[2];		/* dark >= LINE_SEE */
	INT	pos;		/* -1000 (line under left) .. +1000 (right) */
	INT	steer;		/* last steering output */
	INT	speed;		/* last speed output */
	UINT	mode;		/* LINE_M_* */
	INT	side;		/* last side seen -1/0/+1 */
	INT	set_speed;	/* straight-line speed in use, mm/s */
} LINE_STATUS;
IMPORT void line_get(LINE_STATUS *st);
IMPORT BOOL line_inside(void);		/* TRUE: INSIDE or EDGE layout (both black is
					   not a junction) */
/* Line-following speed, changeable while running (dashboard "Set speed").
   Bends get LINE_BEND_DEFAULT % of it.  Clamped to LINE_SPEED_MIN..MAX. */
#define LINE_SPEED_DEFAULT	30	/* 3 Nov: very slow until the turning works */
#define LINE_SPEED_MIN		30
#define LINE_BEND_FLOOR		25	/* bend speed never below this: motors may stall */
#define LINE_SPEED_MAX		300
IMPORT void line_set_speed(INT mms);
/* Climb mode (12 Nov evening): set by the mission on a slope / one-sided
   ramp / stall.  Line following goes on, but at LINE_CLIMB_MMS and with
   the steering capped so both wheels always drive forward. */
#define LINE_CLIMB_MMS		70
#define LINE_CLIMB_STEER	250	/* inner wheel >= 50 % of the outer one */
IMPORT void line_climb(BOOL on);
IMPORT INT  line_get_speed(void);
/* Live tuning from the dashboard: speed (mm/s), bend speed in % of it,
   steering strength (KP x1000).  0 = leave that one unchanged. */
IMPORT void line_set_tuning(INT mms, INT bend_pct, INT kp_x1000);
IMPORT void line_get_tuning(INT *mms, INT *bend_pct, INT *kp_x1000);
/* Bends: stop, then pivot on the inner wheel (TRUE) or steer through (FALSE).
   Default from LINE_PIVOT_DEFAULT; the dashboard can switch it live. */
IMPORT void line_set_pivot(BOOL on);
IMPORT BOOL line_get_pivot(void);
#define LINE_BEND_DEFAULT	50	/* % of the line speed in bends */
/* Fast: bit0 = left sees line, bit1 = right sees line */
IMPORT UINT line_see_bits(void);

/* ---------------------------------------------------------------------
 *  Ranger: ultrasonic on the servo (subsystem5)
 * --------------------------------------------------------------------- */
#define RANGE_FAR_CM		255	/* nothing in range */
IMPORT void ranger_aim(INT deg);	/* -80..+80, + = left */
IMPORT void ranger_guard(BOOL on);	/* EV_OBSTACLE when front < STOP_CM */
IMPORT INT  ranger_guard_cm(void);	/* last close reading that counted for the guard */
IMPORT void ranger_fast(BOOL on);	/* sweep mode: short echo window (close
					   range only), 30 ms between pings */
/* Latest reading.  Packed word:
     bits 0..7   distance cm (255 = far / no echo)
     bits 8..15  angle + 128 at which it was taken
     bits 16..23 sequence number (changes every ping)
     bit  24     valid (servo had settled, echo seen)  */
IMPORT UW ranger_word(void);
#define RW_CM(w)	((INT)((w) & 0xFFU))
#define RW_DEG(w)	((INT)(((w) >> 8) & 0xFFU) - 128)
#define RW_SEQ(w)	((UINT)(((w) >> 16) & 0xFFU))
#define RW_VALID(w)	((((w) >> 24) & 1U) != 0)
IMPORT UW ranger_no_echo_count(void);

/* ---------------------------------------------------------------------
 *  Barcode (subsystem3, Code 39 on GP2)
 * --------------------------------------------------------------------- */
#define BC_ACT_NONE		0	/* just report it */
#define BC_ACT_LEFT		1	/* turn left at the next junction */
#define BC_ACT_RIGHT		2
#define BC_ACT_UTURN		3
#define BC_ACT_END		4	/* end of course: stop */
#define BC_MAX_TEXT		8
typedef struct {
	char	text[BC_MAX_TEXT + 1];	/* between the '*' start/stop chars */
	UINT	action;			/* BC_ACT_* for text[0] */
	UINT	seq;			/* +1 per decoded barcode */
	BOOL	reversed;		/* read back to front */
	UW	t_ms;
	UW	good, bad;		/* totals */
} BARCODE_INFO;
IMPORT void barcode_get(BARCODE_INFO *b);
IMPORT BOOL barcode_busy(void);		/* bars passing right now (or just did) */

/* ---------------------------------------------------------------------
 *  IMU (subsystem4, GY-511 = LSM303 accel + magnetometer, no gyro)
 * --------------------------------------------------------------------- */
#define IMU_EV_HUMP		(1U << 0)
#define IMU_EV_IMPACT		(1U << 1)
#define IMU_EV_TILT		(1U << 2)
typedef struct {
	BOOL	ok;		/* sensor answering */
	INT	ax, ay, az;	/* mg, board axes */
	INT	pitch_d10;	/* 0.1 deg, nose up +, relative to level at rest */
	INT	roll_d10;	/* 0.1 deg, right side down + */
	INT	heading;	/* 0..359 deg (magnetometer, uncalibrated), -1 = none */
	INT	shock_mg;	/* last |a| - 1 g peak */
	UINT	humps;		/* humps crossed */
	INT	hump_mm;	/* height of the last hump */
	UINT	events;		/* IMU_EV_* latched since boot (bits) */
	UINT	seq;		/* +1 per event */
} IMU_STATUS;
IMPORT void imu_get(IMU_STATUS *s);

/* ---------------------------------------------------------------------
 *  Shared fault bitmask (subsystem1).  One bit per fault, in ONE kernel
 *  event flag, so set/clear from any task or core is atomic.
 *  Bit numbers are the wire format (robot/status, robot/event/fault).
 * --------------------------------------------------------------------- */
#define FAULT_MOTOR_STALL	(1U << 0)
#define FAULT_ENCODER		(1U << 1)
#define FAULT_IMU		(1U << 2)
#define FAULT_NO_ECHO		(1U << 3)
#define FAULT_WIFI_LOSS		(1U << 4)
#define FAULT_MQTT_LOSS		(1U << 5)
#define FAULT_LINE		(1U << 6)
#define FAULT_AVOID		(1U << 7)
#define FAULT_MOTION		(1U << 8)
#define FAULT_IMPACT		(1U << 9)
#define FAULT_COMMS		(1U << 10)
#define FAULT_ALL		0x7FFU
IMPORT void fault_raise(UINT bits);
IMPORT void fault_clear(UINT bits);
IMPORT UINT fault_get(void);

/* Remote commands (robot/cmd) */
#define CMD_START		1
#define CMD_STOP		2
#define CMD_PAUSE		3
#define CMD_RESUME		4
#define CMD_PING		5
#define CMD_UTURN		6	/* turn round on the line and follow it back */
#define CMD_REVERSE		7	/* back up REVERSE_MM, then stop */
/* robot/cmd tuning form: byte 0 = 0, byte 1 = line speed in cm/s,
   [byte 2 = bend speed %], [byte 3 = steering KP / 100]  (0 = unchanged) */

/* odometry for the IMU (hump height) and telemetry */
IMPORT W motion_odometry_mm(void);	/* distance driven, forward + */
/* heading from the two wheel encoders: + = turned LEFT (counter-clockwise),
   0.1 deg, since power-up.  Only trustworthy while motion_heading_ok(). */
IMPORT W motion_heading_d10(void);
IMPORT BOOL motion_heading_ok(void);

/* ---------------------------------------------------------------------
 *  Mission: events into mission_flg
 * --------------------------------------------------------------------- */
#define EV_OBSTACLE		(1U << 0)	/* ranger: front < STOP_CM, N pings */
#define EV_LINE_LOST		(1U << 1)	/* line task gave up */
#define EV_PING			(1U << 2)	/* ranger: new reading */
#define EV_JUNCTION		(1U << 3)	/* line task: both sensors black */
#define EV_BARCODE		(1U << 4)	/* barcode decoded */
#define EV_TILT			(1U << 5)	/* IMU: tipped over / lifted */
#define EV_CMD_START		(1U << 6)	/* remote commands */
#define EV_CMD_STOP		(1U << 7)
#define EV_CMD_PAUSE		(1U << 8)
#define EV_CMD_RESUME		(1U << 9)
#define EV_CMD_UTURN		(1U << 10)
#define EV_CMD_REVERSE		(1U << 11)
#define EV_LIFT			(1U << 12)	/* IMU: lifted off the floor (picked up) */
#define EV_ALL			0x1FFFU
IMPORT void mission_event(UINT ev);

#define MS_IDLE			0
#define MS_FOLLOW		1
#define MS_AVOID		2
#define MS_FAULT		3
#define MS_PAUSED		4
#define MS_DONE			5	/* end-of-course barcode */
#define MS_TURN			6	/* barcode turn at a junction (or U-turn) */
#define MS_REVERSE		7	/* backing up (remote REVERSE) */
IMPORT UINT mission_state(void);
IMPORT UINT mission_sys_mode(void);	/* wire value for robot/status */
IMPORT const char *mission_text(void);	/* human-readable sub-state */

/* Init / start functions (called from usermain) */
IMPORT ER mission_init(void);
IMPORT ER mission_start(void);
IMPORT void resources_report(void);	/* boot table: what really exists */

/* Last obstacle sweep result, for telemetry (copy + sequence number) */
/* Every step of an obstacle is recorded (and sent on robot/tlm/obstacle) */
#define OBS_DETECTED		1	/* stopped for it */
#define OBS_FALSE_ALARM		2	/* gone when we looked again */
#define OBS_SWEPT		3	/* fan sweep done: width / side known */
#define OBS_GONE		4	/* moved away during the sweep */
#define OBS_BLOCKED		5	/* no way round: waiting */
#define OBS_PASSED		6	/* driven past it */
#define OBS_REJOINED		7	/* back on the line */
#define OBS_FAILED		0	/* gave up (fault) */
typedef struct {
	UINT	seq;
	UINT	stage;		/* OBS_* */
	INT	dist_cm, bearing_deg, width_cm, need_left_cm, need_right_cm;
	INT	dir;		/* +1 left, -1 right, 0 blocked / not decided */
} OBSTACLE_INFO;
IMPORT void mission_obstacle(OBSTACLE_INFO *o);	/* latest */
IMPORT BOOL mission_obstacle_pop(OBSTACLE_INFO *o);	/* next unsent step (comms task) */

/* Resource usage (resources.c): live kernel objects + CPU load */
typedef struct {
	UINT	tasks, flags, sems, mbfs, cycs;	/* objects that exist now */
	UINT	cpu_pct[2];			/* load per core, last second */
} RES_USAGE;
IMPORT void resources_usage(RES_USAGE *r);	/* ~100 kernel calls: once a second */
IMPORT ER resources_start(void);		/* the two idle-time meters */
IMPORT UW motion_loop_us_max(void);		/* longest motion loop since last call */
IMPORT UW line_loop_us_max(void);		/* longest line loop since last call */

#endif /* CAR_H */
