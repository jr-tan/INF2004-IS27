/*
 *	subsystem2.h  (Buddy 2: motion)
 *	Motor PWM, wheel encoders and the 20 ms motion loop.
 *	Public API is in car.h; this file holds the tunables.
 *
 *	Hybrid control
 *	  duty = (D0 + K * speed) * wheel gain      (open loop, always on)
 *	       + encoder trim                       (only while BOTH encoders
 *	                                             are healthy)
 *	The car still drives straight on the open-loop part if an encoder
 *	drops out (our left one does), it just stops learning.
 */
#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../car.h"

/* ---- PWM ---- */
#define MOT_PWM_HZ		10000		/* 125 MHz / 12500 */

/* ---- open loop: speed (mm/s) -> duty (per mille) ----
   2 Nov circuit log: 25.5 % duty = 140 mm/s on the mat */
#define MOT_D0_PM		120		/* duty where the wheels start to turn */
#define MOT_K_PM_X1000		986		/* + duty per mm/s, x1000 */
/* 20 Nov: was 1150 ("left motor ~15 % weaker").  With the wheels in the
   air the left one ran FASTER, and at 30 mm/s the +15 % cancelled every
   left steer (left 15 mm/s got more duty than right 30 mm/s). */
#define MOT_GAIN_L_X1000	1000
#define MOT_GAIN_R_X1000	1000
#define MOT_MAX_MMS		600

/* ---- ramps ---- */
#define MOT_ACCEL_MMS2		800		/* set-point ramp, both directions */
#define MOT_KICK_PM		120		/* extra duty when a wheel starts */
#define MOT_KICK_MS		250

/* ---- encoders ----
   x4 quadrature: ~2490 steps per wheel turn, 65 mm wheel
   => pi * 65000 um / 2490 = 82 um per step */
#define ENC_UM_PER_STEP		82
/* 20 Nov: per-wheel scale.  Since the plug swap the left encoder reads
   1.3x the right on a straight hand push (2.2x while driven).  CAL: push
   the car straight along a ruler for exactly 500 mm and note how many
   counts each wheel added (console "enc L" / "enc R"); then
   um per step = 500000 / counts.  Both 82 = old behaviour. */
#define ENC_L_UM_PER_STEP	ENC_UM_PER_STEP
#define ENC_R_UM_PER_STEP	ENC_UM_PER_STEP
/* CAL: wheel track = distance between the centres of the two wheels
   (where they touch the floor).  Turning heading = (right - left) / track.
   Check: turn the car one full circle on the spot by hand; the console
   "head" value should change by about 3600. */
#define MOT_TRACK_MM		108		/* measured 12 Nov (was 125: every encoder turn came out ~16 % too big) */
#define ENC_WINDOW_TICKS	5		/* speed over 5 x 20 ms = 100 ms */
#define ENC_DEAD_MS		200		/* driven, no steps this long = unhealthy */
#define ENC_HEAL_MS		1000		/* steps again this long = healthy */
#define ENC_MIN_MMS		40		/* below this the speed reading is too coarse */

/* ---- encoder trim (the "hybrid" part): per-wheel speed control ---- */
/* 20 Nov: 0 = test switch.  No encoder speed correction: the motors run
   on the open-loop duty (+ start kick + stall boost) only, like the car
   did before 12 Nov evening.  If driving gets better with 0, the
   encoder readings are what is breaking it: calibrate the per-wheel
   scale above, then set back to 1. */
#define MOT_TRIM_ON		1
#define TRIM_KI_X1000		100		/* duty pm per (mm/s error) per tick, x1000 */
#define TRIM_MAX_PM		300		/* never more than +-30 % duty (was 15 %:
						   not enough to hold speed up a slope) */
#define TRIM_MIN_MMS		15		/* learns from this speed up (was 40) */

/* ---- stall boost (12 Nov evening): driven, no encoder steps for
   STALL_MS -> add STALL_BOOST_STEP_PM per 20 ms tick, up to
   STALL_BOOST_MAX_PM; fades by STALL_BOOST_FADE_PM per tick once the
   wheel turns.  Still no steps STALL_GIVEUP_MS after reaching the
   maximum = the ENCODER is dead, not the wheel stuck. ---- */
#define STALL_MIN_MMS		15
#define STALL_MS		150
#define STALL_BOOST_STEP_PM	15		/* 0 -> max in ~0.5 s */
#define STALL_BOOST_MAX_PM	350
#define STALL_BOOST_FADE_PM	5		/* max -> 0 in ~1.4 s */
#define STALL_GIVEUP_MS		600

/* ---- motion_flg bits ---- */
#define MF_TICK			(1U << 0)	/* from the 20 ms cyclic handler */
#define MF_WAKE			(1U << 1)	/* owner change / STOP */

/* drivers */
#define MOTOR_L			0
#define MOTOR_R			1
IMPORT void drv_motor_init(void);
IMPORT void drv_motor_set(UINT m, INT dir, UINT duty_pm);	/* dir +1 / -1 */
IMPORT void drv_motor_brake(UINT m);				/* both low */

typedef struct {
	UW	count;		/* steps, only goes up */
	W	pos;		/* signed steps, + = forward */
	UW	last_us;	/* time of the last counted step */
	UW	glitch;
} ENC_SNAP;
IMPORT ER   drv_encoder_init(void);		/* call on CORE 0 */
IMPORT void drv_encoder_snapshot(ENC_SNAP s[2]);	/* call on CORE 0 */

/* shared with subsystem3 (line task is woken by the same tick) */
IMPORT ID motion_flg;
IMPORT ID line_task_id;

#endif
