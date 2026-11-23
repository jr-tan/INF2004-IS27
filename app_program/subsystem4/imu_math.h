/**
 * @file    imu_math.h
 *  imu_math.h  (Buddy 4)
 *  Integer maths for the IMU, PURE logic (no kernel calls): no FPU on
 *  the RP2040 and no libm in this build.  testing/imu_test/ checks it
 *  against the PC's libm and runs the hump detector on made-up humps.
 */
#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>

int32_t  imu_math_atan2_d10 (int32_t yval,
                             int32_t xval); /* 0.1 deg, -1800 .. 1800 */
uint32_t imu_math_isqrt (uint32_t val);
int32_t  imu_math_sin_x1000 (int32_t deg_d10); /* sin x 1000 from 0.1 deg */

/* Hump / ramp detector.  Feed pitch (0.1 deg, nose up +, relative to
   level) and distance driven (mm) every sample.  Height = sum of
   sin(pitch) x ds.
     hump or ramp UP:  reported when level again after the top (or on a
                       plateau at the top of a one-sided ramp)
     ramp DOWN only:   reported as a drop when level again at the bottom
   Gives up after max_mm of travel (or max_ms) without finishing. */
typedef struct
{
    int32_t  on_d10;     /* climb starts above this pitch (e.g. 60 = 6 deg) */
    int32_t  off_d10;    /* "level again" below this */
    uint32_t confirm_ms; /* pitch must stay up this long */
    uint32_t level_ms;   /* level this long = hump over */
    uint32_t max_ms;     /* give up after this */
    int32_t  min_mm;     /* smaller "humps" are ignored */
    int32_t  max_mm;     /* give up after this much travel */
} hump_tune_t;

typedef struct
{
    int32_t
        st; /* 0 flat, 1 maybe up, 2 climb, 3 descend, 5 maybe down, 6 drop */
    uint32_t t0;
    uint32_t t_level;
    int32_t  s_prev;
    int32_t  s0;
    int64_t  h_um;
    int64_t  peak_um;
    int64_t  base_um;
} hump_t;

void imu_math_hump_reset (hump_t * p_hnd);
/* returns 1 when a hump / ramp up has just been crossed, 2 when a ramp
   down has (a drop); *height_mm filled (always positive) */
int32_t imu_math_hump_step (hump_t * p_hnd, const hump_tune_t * p_key,
                            int32_t pitch_d10, int32_t odo_mm,
                            uint32_t motion_now_ms, int32_t * p_height_mm);

#endif

/*** end of file ***/
