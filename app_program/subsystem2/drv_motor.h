/**
 * @file    drv_motor.h
 * @brief   Public interface of drv_motor.c (BARR-C 4.2.a: one header per
 *          source file).
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "subsystem2.h"

IMPORT void drv_motor_init (void);

IMPORT void drv_motor_set (uint32_t mval, int32_t dir,
                           uint32_t duty_pm); /* dir +1 / -1 */

IMPORT void drv_motor_brake (uint32_t mval); /* both low */

#endif /* DRV_MOTOR_H */

/*** end of file ***/
