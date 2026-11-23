/**
 * @file    subsystem4.h
 * @brief   Public interface of subsystem4.c (BARR-C 4.2.a: one header per
 *          source file).
 */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "../car.h"

IMPORT void imu_get (imu_status_t * p_str);

IMPORT ER subsystem4_init (void);

IMPORT ER subsystem4_start (void);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
