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
#define MOT_GAIN_L_X1000	1150		/* CAL: left motor is ~15 % weaker */
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
/* CAL: wheel track = distance between the centres of the two wheels
   (where they touch the floor).  Turning heading = (right - left) / track.
   Check: turn the car one full circle on the spot by hand; the console
   "head" value should change by about 3600. */
#define MOT_TRACK_MM		125
#define ENC_WINDOW_TICKS	5		/* speed over 5 x 20 ms = 100 ms */
#define ENC_DEAD_MS		200		/* driven, no steps this long = unhealthy */
#define ENC_HEAL_MS		1000		/* steps again this long = healthy */
#define ENC_MIN_MMS		40		/* below this the speed reading is too coarse */

/* ---- encoder trim (the "hybrid" part) ---- */
#define TRIM_KI_X1000		100		/* duty pm per (mm/s error) per tick, x1000 */
#define TRIM_MAX_PM		150		/* never more than +-15 % duty */

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
