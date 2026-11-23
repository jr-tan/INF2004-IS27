/**
 * @file    subsystem3.h
 * @brief   Public interface of subsystem3.c (BARR-C 4.2.a: one header per
 *          source file).
 */
#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "../car.h"

IMPORT void line_follow (BOOL b_on, int32_t seed_side, BOOL b_seed_deep);

IMPORT void line_get (line_status_t * p_state);

IMPORT BOOL line_inside (void); /* TRUE: INSIDE or EDGE layout (both black is
                   not a junction) */

IMPORT void line_set_speed (int32_t mms);

IMPORT void line_climb (BOOL b_on);

IMPORT int32_t line_get_speed (void);

IMPORT void line_set_tuning (int32_t mms, int32_t bend_pct, int32_t kp_x1000);

IMPORT void line_get_tuning (int32_t * p_mms, int32_t * p_bend_pct,
                             int32_t * p_kp_x1000);

IMPORT void line_set_pivot (BOOL b_on);

IMPORT BOOL line_get_pivot (void);

/* Fast: bit0 = left sees line, bit1 = right sees line */
IMPORT uint32_t line_see_bits (void);
/* Change Requirement 1: packed readings, see LINE_WORD_* in car.h */
IMPORT uint32_t line_raw_word (void);  /* ADC readings */
IMPORT uint32_t line_dark_word (void); /* white level removed */

IMPORT uint32_t line_loop_us_max (void); /* longest line loop since last call */

/* BARCODE_ANALOG: start the free-running ADC scan (line L/R + barcode
   A0).  Core 0 only (the barcode task).  FALSE = not started. */
IMPORT BOOL line_adc_scan_start (void);
IMPORT ER   subsystem3_init (void);

IMPORT ER subsystem3_start (void);

#endif /* SUBSYSTEM3_H */

/*** end of file ***/
