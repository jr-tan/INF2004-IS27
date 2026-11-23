/**
 * @file    scan_profile.h
 *  scan_profile.h  (Buddy 5)
 *  Turns one fan sweep of the ultrasonic (servo from left to right and
 *  back) into an obstacle profile: distance, bearing, width, where its
 *  left and right edges are, and which way round is shorter.
 *
 *  PURE logic (no kernel calls), so testing/scan_test/scan_test.c runs
 *  the same file on a PC with made-up obstacles.
 *
 *  Angles in degrees, + = LEFT.  Lateral positions in cm, + = left of
 *  the car's centre line.  Distances in cm, 255 = nothing / no echo.
 *
 *  How it works
 *    1. the closest reading is the obstacle (dmin, at the bearing)
 *    2. neighbours within dmin + band_cm belong to the same obstacle;
 *       the run stops at the first reading that is clearly farther
 *       (a single missed echo inside the obstacle is bridged)
 *    3. each edge's sideways position = distance x sin(angle), pulled in
 *       by beam_deg because the sound cone makes things look wider
 *    4. a side is OPEN only if the sweep saw past that edge (the run
 *       did not reach the end of the sweep)
 *    5. sideways move needed = edge + half the car + margin; pick the
 *       open side that needs the smaller move
 */
#ifndef SCAN_PROFILE_H
#define SCAN_PROFILE_H

#include <stdint.h>

#define SCAN_MAX_PTS 32

typedef struct
{
    int32_t band_cm;     /* same obstacle if within dmin + this */
    int32_t far_cm;      /* readings beyond this are "free" */
    int32_t beam_deg;    /* half the extra width the sound cone adds */
    int32_t half_car_cm; /* half the car's width */
    int32_t margin_cm;   /* extra room when passing */
    int32_t prefer_left; /* tie: 1 = left */
} scan_tune_t;

typedef struct
{
    int32_t found; /* 0 = nothing closer than far_cm */
    int32_t dmin_cm;
    int32_t bearing_deg; /* centre of the obstacle */
    int32_t left_deg;    /* edge angles (after beam correction) */
    int32_t right_deg;
    int32_t left_cm; /* edge sideways positions, + = left */
    int32_t right_cm;
    int32_t width_cm;
    int32_t open_left; /* sweep saw free space past that edge */
    int32_t open_right;
    int32_t need_left_cm;  /* sideways move to pass on the left */
    int32_t need_right_cm; /* sideways move to pass on the right */
    int32_t dir;           /* +1 go left, -1 go right, 0 blocked */
    int32_t need_cm;       /* sideways move for the chosen side */
} scan_profile_t;

/* deg[i] must run from the most LEFT angle (i = 0) to the most RIGHT */
void scan_profile_compute (const scan_tune_t * p_key, const int32_t * p_deg,
                           const int32_t * p_cm_val, int32_t num,
                           scan_profile_t * p_pval);

/* sin(deg) x 1000, integer, any angle */
int32_t scan_profile_isin_x1000 (int32_t deg);

#endif

/*** end of file ***/
