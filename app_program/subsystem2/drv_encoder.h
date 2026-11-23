/**
 * @file    drv_encoder.h
 * @brief   Public interface of drv_encoder.c (BARR-C 4.2.a: one header per
 *          source file).
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "subsystem2.h"

IMPORT ER drv_encoder_init (void); /* call on CORE 0 */

IMPORT void drv_encoder_snapshot (enc_snap_t p_str[2]); /* call on CORE 0 */

#endif /* DRV_ENCODER_H */

/*** end of file ***/
