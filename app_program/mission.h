/**
 * @file    mission.h
 * @brief   Public interface of mission.c (BARR-C 4.2.a: one header per
 *          source file).
 */
#ifndef MISSION_H
#define MISSION_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "car.h"

IMPORT void mission_event (uint32_t ev_val);

IMPORT uint32_t mission_state (void);

IMPORT uint32_t mission_sys_mode (void); /* wire value for robot/status */

IMPORT const char * mission_text (void); /* human-readable sub-state */

/* Init / start functions (called from usermain) */
IMPORT ER mission_init (void);

IMPORT ER mission_start (void);

IMPORT void mission_obstacle (obstacle_info_t * p_out); /* latest */

IMPORT BOOL mission_obstacle_pop (
    obstacle_info_t * p_out); /* next unsent step (comms task) */

#endif /* MISSION_H */

/*** end of file ***/
