/**
 * @file    barcode.h
 * @brief   Public interface of barcode.c (BARR-C 4.2.a: one header per
 *          source file).
 */
#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "../car.h"

IMPORT void barcode_get (barcode_info_t * p_buf);

IMPORT BOOL barcode_busy (void);  /* bars passing right now (or just did) */
IMPORT ER   barcode_start (void); /* start the barcode task (core 0) */
/* BARCODE_ANALOG: the ADC interrupt (subsystem3.c, core 0) hands every
   A0 sample here with its timer stamp, us. */
IMPORT void barcode_adc_sample (uint32_t raw, uint32_t t_us);
/* Letter -> BC_ACT_* map, set from the GUI (robot/map).  Called by the
   comms task only.  get returns TRUE once a map has come from the GUI. */
IMPORT void barcode_set_map (const uint8_t p_act[BC_MAP_LETTERS]);
IMPORT BOOL barcode_get_map (uint8_t p_act[BC_MAP_LETTERS]);

#endif /* BARCODE_H */

/*** end of file ***/
