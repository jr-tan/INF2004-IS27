/**
 * @file    board_io.h
 *  board_io.h  (Buddy 1)
 *  Robo Pico on-board RGB LEDs (2 x WS2812 on GP18) and buzzer (GP22).
 *  Status task only.
 */
#ifndef BOARD_IO_H
#define BOARD_IO_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
/* colours, kept dim (0..255 per channel; full brightness is blinding) */
#define RGB_RED  40, 0, 0  /* no WiFi */
#define RGB_PINK 48, 6, 36 /* WiFi connected (v3.18's 40,4,16 looked red) */
#define RGB_ORANGE \
    40, 12, 0              /* found the hotspot, still connecting (no IP yet) */
#define RGB_GREEN 0, 40, 0 /* power-up colour test only */
#define RGB_BLUE  0, 0, 40
#define RGB_OFF   0, 0, 0

/* tunes for board_io_buzz_play() */
#define TUNE_WIFI      0 /* WiFi connected: three rising notes */
#define TUNE_WIFI_LOST 1 /* WiFi dropped: two falling notes */
#define TUNE_BARCODE   2 /* barcode read: one beep */
#define TUNE_BC_ACTION 3 /* barcode with a turn / stop action: two beeps */
#define TUNE_BOOT      4 /* power-up: short tick */

IMPORT void board_io_init (void);
IMPORT void board_io_rgb_set (uint32_t res, uint32_t gval,
                              uint32_t bval); /* both LEDs */
IMPORT void board_io_buzz_play (uint32_t tune);
IMPORT void board_io_buzz_step (uint32_t ms_val);

#endif

/*** end of file ***/
