/**
 * @file    subsystem1.h
 * @brief   Public interface of subsystem1.c (BARR-C 4.2.a: one header per
 *          source file).
 */
#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "../car.h"

IMPORT void comms_fault_raise (uint32_t bits);

IMPORT void comms_fault_clear (uint32_t bits);

IMPORT uint32_t comms_fault_get (void);

IMPORT ER subsystem1_init (void);

IMPORT ER subsystem1_start (void);

#endif /* SUBSYSTEM1_H */

/*** end of file ***/
