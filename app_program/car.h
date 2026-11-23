/**
 * @file    car.h
 *  car.h
 *  The interfaces between the car's modules.  Every .c file includes
 *  this one header.
 *
 *  Data flow (who writes -> who reads)
 *  -----------------------------------
 *    line task   --(slot LINE)-->     motion task  --> motors
 *    mission     --(slot MISSION)-->  motion task
 *    mission     --motion_set_owner-> motion task  (picks the slot)
 *    mission     --line_follow()-->   line task    (enable + seed)
 *    mission     --ranger_aim()-->    ranger task  (servo angle)
 *    ranger/line --EV_* bits-->       mission      (mission_flg)
 *    everyone    <--*_get()--         one 32-bit word or a copy
 *
 *  Rules that make this race-free without mutexes
 *    - every shared word has ONE writer and is a single aligned 32-bit
 *      store (atomic on the RP2040); readers take one load
 *    - each driver of the motors has its OWN slot, so a late write
 *      from the line task can never overwrite a mission command
 *    - motion_stop() can be called by anyone, at any time; it wins
 *      because it switches the owner to NONE and wakes the motion task
 */
#ifndef CAR_H
#define CAR_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "app.h"
#include "car_config.h"

/* Small helpers */
#define ABS(x)           (((x) < 0) ? (-(x)) : (x))
#define CLAMP(x, lo, hi) (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))

/* ---------------------------------------------------------------------
 *  Motion (subsystem2)
 * --------------------------------------------------------------------- */
#define MOT_OWNER_NONE    0 /* braked */
#define MOT_OWNER_LINE    1
#define MOT_OWNER_MISSION 2
#define MOT_NUM_OWNERS    3

/* Arc drive: speed in mm/s (>= 0), steer -1000..+1000 (+ = right).
   |steer| 500 stops the inner wheel (pivot on it). */

typedef struct
{
    uint32_t owner;
    int32_t  tgt_mms[2]; /* after the ramp */
    uint32_t duty_pm[2]; /* 0..1000 */
    int32_t  trim_pm[2]; /* encoder correction */
    int32_t  v_mms[2];   /* measured, 0 if encoder unhealthy */
    uint32_t count[2];   /* encoder steps (always increasing) */
    int32_t  pos[2];     /* signed steps */
    uint32_t glitch[2];
    BOOL     healthy[2];
    BOOL     trim_on;
} motion_status_t;

/* ---------------------------------------------------------------------
 *  Line (subsystem3)
 * --------------------------------------------------------------------- */
/* line_follow(TRUE, side, deep): line task drives slot LINE.
   side = -1 left / +1 right / 0: where the line was last seen.
   deep = TRUE: the line is OUTSIDE that sensor (turn hard toward it). */

#define LINE_M_OFF      0 /* not following */
#define LINE_M_TRACK    1 /* a sensor sees the line */
#define LINE_M_CENTRED  2 /* both white, line between them */
#define LINE_M_RECOVER  3 /* line slid outside: turning back */
#define LINE_M_JUNCTION 4 /* both black: straight on */
#define LINE_M_LOST     5
#define LINE_M_BENDSTOP 6 /* bend found: braking before the pivot */
#define LINE_M_PIVOT    7 /* turning on the inner wheel */

typedef struct
{
    uint32_t raw[2];    /* ADC 0..4095 (left, right) */
    uint32_t dark[2];   /* white level removed, 0..4095 */
    BOOL     see[2];    /* dark >= LINE_SEE */
    int32_t  pos;       /* -1000 (line under left) .. +1000 (right) */
    int32_t  steer;     /* last steering output */
    int32_t  speed;     /* last speed output */
    uint32_t mode;      /* LINE_M_* */
    int32_t  side;      /* last side seen -1/0/+1 */
    int32_t  set_speed; /* straight-line speed in use, mm/s */
} line_status_t;
/* Change Requirement 1 (Tribunal 1, Buddy 3): the line task publishes the
   two 12-bit readings and an 8-bit sequence count packed in ONE 32-bit
   word, written with a single aligned store, so a reader on any task or
   core never pairs a left reading from one cycle with a right reading
   from another.  Layout (fixed, little-endian bit numbers):
     bits 0..11 left, bits 12..23 right, bits 24..31 sequence */
#define LINE_WORD_LEFT(w)  ((w) & 0xFFFU)
#define LINE_WORD_RIGHT(w) (((w) >> 12) & 0xFFFU)
#define LINE_WORD_SEQ(w)   (((w) >> 24) & 0xFFU)
/* Line-following speed, changeable while running (dashboard "Set speed").
   Bends get LINE_BEND_DEFAULT % of it.  Clamped to LINE_SPEED_MIN..MAX. */
#define LINE_SPEED_DEFAULT 30 /* 3 Nov: very slow until the turning works */
#define LINE_SPEED_MIN     30
/* bend speed never below this: motors may stall */
#define LINE_BEND_FLOOR 25
#define LINE_SPEED_MAX  300
/* Climb mode (12 Nov evening): set by the mission on a slope / one-sided
   ramp / stall.  Line following goes on, but at LINE_CLIMB_MMS and with
   the steering capped so both wheels always drive forward. */
#define LINE_CLIMB_MMS   70
#define LINE_CLIMB_STEER 250 /* inner wheel >= 50 % of the outer one */
/* Live tuning from the dashboard: speed (mm/s), bend speed in % of it,
   steering strength (KP x1000).  0 = leave that one unchanged. */
/* Bends: stop, then pivot on the inner wheel (TRUE) or steer through (FALSE).
   Default from LINE_PIVOT_DEFAULT; the dashboard can switch it live. */
#define LINE_BEND_DEFAULT 50 /* % of the line speed in bends */

/* ---------------------------------------------------------------------
 *  Ranger: ultrasonic on the servo (subsystem5)
 * --------------------------------------------------------------------- */
#define RANGE_FAR_CM 255 /* nothing in range */
/* Latest reading.  Packed word:
     bits 0..7   distance cm (255 = far / no echo)
     bits 8..15  angle + 128 at which it was taken
     bits 16..23 sequence number (changes every ping)
     bit  24     valid (servo had settled, echo seen)  */
#define RW_CM(w)    ((int32_t)((w) & 0xFFU))
#define RW_DEG(w)   ((int32_t)(((w) >> 8) & 0xFFU) - 128)
#define RW_SEQ(w)   ((uint32_t)(((w) >> 16) & 0xFFU))
#define RW_VALID(w) ((((w) >> 24) & 1U) != 0)

/* ---------------------------------------------------------------------
 *  Barcode (subsystem3, Code 39 on GP2)
 * --------------------------------------------------------------------- */
#define BC_ACT_NONE    0 /* drive straight on (not mapped) */
#define BC_ACT_LEFT    1 /* turn left at the next junction */
#define BC_ACT_RIGHT   2
#define BC_ACT_UTURN   3
#define BC_ACT_END     4 /* end of course: stop */
#define BC_MAX_TEXT    8
#define BC_MAP_LETTERS 26 /* A..Z: one action each in the barcode map */
typedef struct
{
    char     text[BC_MAX_TEXT + 1]; /* between the '*' start/stop chars */
    uint32_t action;                /* BC_ACT_* for text[0] */
    uint32_t seq;                   /* +1 per decoded barcode */
    BOOL     reversed;              /* read back to front */
    uint32_t t_ms;
    uint32_t good; /* totals */
    uint32_t bad;
} barcode_info_t;

/* ---------------------------------------------------------------------
 *  IMU (subsystem4, GY-511 = LSM303 accel + magnetometer, no gyro)
 * --------------------------------------------------------------------- */
#define IMU_EV_HUMP   (1U << 0)
#define IMU_EV_IMPACT (1U << 1)
#define IMU_EV_TILT   (1U << 2)
typedef struct
{
    BOOL     ok; /* sensor answering */
    int32_t  ax; /* mg, board axes */
    int32_t  ay;
    int32_t  az;
    int32_t  pitch_d10; /* 0.1 deg, nose up +, relative to level at rest */
    int32_t  roll_d10;  /* 0.1 deg, right side down + */
    int32_t  heading;   /* 0..359 deg (magnetometer, uncalibrated), -1 = none */
    int32_t  shock_mg;  /* last |a| - 1 g peak */
    uint32_t humps;     /* humps crossed */
    int32_t  hump_mm;   /* height of the last hump */
    uint32_t events;    /* IMU_EV_* latched since boot (bits) */
    uint32_t seq;       /* +1 per event */
} imu_status_t;

/* ---------------------------------------------------------------------
 *  Shared fault bitmask (subsystem1).  One bit per fault, in ONE kernel
 *  event flag, so set/clear from any task or core is atomic.
 *  Bit numbers are the wire format (robot/status, robot/event/fault).
 * --------------------------------------------------------------------- */
#define FAULT_MOTOR_STALL (1U << 0)
#define FAULT_ENCODER     (1U << 1)
#define FAULT_IMU         (1U << 2)
#define FAULT_NO_ECHO     (1U << 3)
#define FAULT_WIFI_LOSS   (1U << 4)
#define FAULT_MQTT_LOSS   (1U << 5)
#define FAULT_LINE        (1U << 6)
#define FAULT_AVOID       (1U << 7)
#define FAULT_MOTION      (1U << 8)
#define FAULT_IMPACT      (1U << 9)
#define FAULT_COMMS       (1U << 10)
#define FAULT_ALL         0x7FFU

/* Remote commands (robot/cmd) */
#define CMD_START   1
#define CMD_STOP    2
#define CMD_PAUSE   3
#define CMD_RESUME  4
#define CMD_PING    5
#define CMD_UTURN   6 /* turn round on the line and follow it back */
#define CMD_REVERSE 7 /* back up REVERSE_MM, then stop */
/* robot/cmd tuning form: byte 0 = 0, byte 1 = line speed in cm/s,
   [byte 2 = bend speed %], [byte 3 = steering KP / 100]  (0 = unchanged) */

/* heading from the two wheel encoders: + = turned LEFT (counter-clockwise),
   0.1 deg, since power-up.  Only trustworthy while motion_heading_ok(). */

/* ---------------------------------------------------------------------
 *  Mission: events into mission_flg
 * --------------------------------------------------------------------- */
#define EV_OBSTACLE    (1U << 0) /* ranger: front < STOP_CM, N pings */
#define EV_LINE_LOST   (1U << 1) /* line task gave up */
#define EV_PING        (1U << 2) /* ranger: new reading */
#define EV_JUNCTION    (1U << 3) /* line task: both sensors black */
#define EV_BARCODE     (1U << 4) /* barcode decoded */
#define EV_TILT        (1U << 5) /* IMU: tipped over / lifted */
#define EV_CMD_START   (1U << 6) /* remote commands */
#define EV_CMD_STOP    (1U << 7)
#define EV_CMD_PAUSE   (1U << 8)
#define EV_CMD_RESUME  (1U << 9)
#define EV_CMD_UTURN   (1U << 10)
#define EV_CMD_REVERSE (1U << 11)
#define EV_LIFT        (1U << 12) /* IMU: lifted off the floor (picked up) */
#define EV_ALL         0x1FFFU

#define MS_IDLE    0
#define MS_FOLLOW  1
#define MS_AVOID   2
#define MS_FAULT   3
#define MS_PAUSED  4
#define MS_DONE    5 /* end-of-course barcode */
#define MS_TURN    6 /* barcode turn at a junction (or U-turn) */
#define MS_REVERSE 7 /* backing up (remote REVERSE) */

/* Last obstacle sweep result, for telemetry (copy + sequence number) */
/* Every step of an obstacle is recorded (and sent on robot/tlm/obstacle) */
#define OBS_DETECTED    1 /* stopped for it */
#define OBS_FALSE_ALARM 2 /* gone when we looked again */
#define OBS_SWEPT       3 /* fan sweep done: width / side known */
#define OBS_GONE        4 /* moved away during the sweep */
#define OBS_BLOCKED     5 /* no way round: waiting */
#define OBS_PASSED      6 /* driven past it */
#define OBS_REJOINED    7 /* back on the line */
#define OBS_FAILED      0 /* gave up (fault) */
typedef struct
{
    uint32_t seq;
    uint32_t stage; /* OBS_* */
    int32_t  dist_cm;
    int32_t  bearing_deg;
    int32_t  width_cm;
    int32_t  need_left_cm;
    int32_t  need_right_cm;
    int32_t  dir; /* +1 left, -1 right, 0 blocked / not decided */
} obstacle_info_t;

/* Resource usage (resources.c): live kernel objects + CPU load */
typedef struct
{
    uint32_t tasks; /* objects that exist now */
    uint32_t flags;
    uint32_t sems;
    uint32_t mbfs;
    uint32_t cycs;
    uint32_t cpu_pct[2]; /* load per core, last second */
} res_usage_t;

/* Module headers split out of this file (BARR-C 4.2.a). */
#include "mission.h"
#include "resources.h"
#include "subsystem1/subsystem1.h"
#include "subsystem3/subsystem3.h"
#include "subsystem3/barcode.h"
#include "subsystem4/subsystem4.h"
#include "subsystem2/subsystem2.h"
#include "subsystem5/subsystem5.h"

#endif /* CAR_H */

/*** end of file ***/
