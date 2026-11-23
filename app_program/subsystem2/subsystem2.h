/**
 * @file    subsystem2.h
 *  subsystem2.h  (Buddy 2: motion)
 *  Motor PWM, wheel encoders and the 20 ms motion loop.
 *  Public API is in car.h; this file holds the tunables.
 *
 *  Hybrid control
 *    duty = (D0 + K * speed) * wheel gain      (open loop, always on)
 *         + encoder trim                       (only while BOTH encoders
 *                                               are healthy)
 *  The car still drives straight on the open-loop part if an encoder
 *  drops out (our left one does), it just stops learning.
 */
#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */

/* Number of drive wheels (left = 0, right = 1); loop bound (BARR-C 8.4.a). */
#define MOT_WHEELS 2
#include "../car.h"

/* ---- PWM ---- */
#define MOT_PWM_HZ 10000 /* 125 MHz / 12500 */

/* ---- open loop: speed (mm/s) -> duty (per mille) ----
   2 Nov circuit log: 25.5 % duty = 140 mm/s on the mat */
#define MOT_D0_PM        120  /* duty where the wheels start to turn */
#define MOT_K_PM_X1000   986  /* + duty per mm/s, x1000 */
#define MOT_GAIN_L_X1000 1150 /* CAL: left motor is ~15 % weaker */
#define MOT_GAIN_R_X1000 1000
#define MOT_MAX_MMS      600

/* ---- ramps ---- */
#define MOT_ACCEL_MMS2 800 /* set-point ramp, both directions */
#define MOT_KICK_PM    120 /* extra duty when a wheel starts */
#define MOT_KICK_MS    250

/* ---- encoders ----
   x4 quadrature: ~2490 steps per wheel turn, 65 mm wheel
   => pi * 65000 um / 2490 = 82 um per step */
#define ENC_UM_PER_STEP 82
/* CAL: wheel track = distance between the centres of the two wheels
   (where they touch the floor).  Turning heading = (right - left) / track.
   Check: turn the car one full circle on the spot by hand; the console
   "head" value should change by about 3600. */
/* measured 12 Nov (was 125: every encoder turn came out ~16 % too big) */
#define MOT_TRACK_MM     108
#define ENC_WINDOW_TICKS 5    /* speed over 5 x 20 ms = 100 ms */
#define ENC_DEAD_MS      200  /* driven, no steps this long = unhealthy */
#define ENC_HEAL_MS      1000 /* steps again this long = healthy */
#define ENC_MIN_MMS      40   /* below this the speed reading is too coarse */

/* ---- encoder trim (the "hybrid" part): per-wheel speed control ---- */
#define TRIM_KI_X1000 100 /* duty pm per (mm/s error) per tick, x1000 */
/* never more than +-30 % duty (was 15 %: not enough to hold speed up a
   slope) */
#define TRIM_MAX_PM  300
#define TRIM_MIN_MMS 15 /* learns from this speed up (was 40) */

/* ---- stall boost (12 Nov evening): driven, no encoder steps for
   STALL_MS -> add STALL_BOOST_STEP_PM per 20 ms tick, up to
   STALL_BOOST_MAX_PM; fades by STALL_BOOST_FADE_PM per tick once the
   wheel turns.  Still no steps STALL_GIVEUP_MS after reaching the
   maximum = the ENCODER is dead, not the wheel stuck. ---- */
#define STALL_MIN_MMS       15
#define STALL_MS            150
#define STALL_BOOST_STEP_PM 15 /* 0 -> max in ~0.5 s */
#define STALL_BOOST_MAX_PM  350
#define STALL_BOOST_FADE_PM 5 /* max -> 0 in ~1.4 s */
#define STALL_GIVEUP_MS     600

/* ---- motion_flg bits ---- */
#define MF_TICK (1U << 0) /* from the 20 ms cyclic handler */
#define MF_WAKE (1U << 1) /* owner change / STOP */

/* drivers */
#define MOTOR_L 0
#define MOTOR_R 1

typedef struct
{
    uint32_t count;   /* steps, only goes up */
    int32_t  pos;     /* signed steps, + = forward */
    uint32_t last_us; /* time of the last counted step */
    uint32_t glitch;
} enc_snap_t;

/* shared with subsystem3 (line task is woken by the same tick) */
IMPORT ID g_motion_flg;
IMPORT ID g_line_task_id;

/* Public functions of subsystem2.c (moved from car.h / app.h, BARR-C 4.2.a). */
IMPORT uint32_t motion_now_us (void); /* 1 MHz timer, low 32 bits: wraps every
                            71 min, differences stay right */

IMPORT uint32_t motion_now_ms (void); /* from the full 64-bit timer: wraps after
                            49 days, differences stay right */

/* INT (T-Kernel, 32-bit) kept: see docs/barr_c_conformance.md, D-5. */
IMPORT void motion_drive (uint32_t owner, INT speed_mms, INT steer_pm);

/* Direct wheel speeds in mm/s, -1000..+1000 (negative = backwards) */
IMPORT void motion_wheels (uint32_t owner, int32_t left_mms, int32_t right_mms);

/* Which slot drives the motors.  Only the mission task calls this. */
IMPORT void motion_set_owner (uint32_t owner);

/* Brake now (both inputs low) and latch: owner becomes NONE. */
IMPORT void motion_stop (void);

IMPORT void motion_get_status (motion_status_t * p_state); /* debug copy */

/* odometry for the IMU (hump height) and telemetry */
IMPORT int32_t motion_odometry_mm (void); /* distance driven, forward + */

IMPORT int32_t motion_heading_d10 (void);

IMPORT BOOL motion_heading_ok (void);

IMPORT uint32_t
motion_loop_us_max (void); /* longest motion loop since last call */

IMPORT ER subsystem2_init (void);

IMPORT ER subsystem2_start (void);

/* Module headers split out of this file (BARR-C 4.2.a). */
#include "drv_motor.h"
#include "drv_encoder.h"

#endif

/*** end of file ***/
