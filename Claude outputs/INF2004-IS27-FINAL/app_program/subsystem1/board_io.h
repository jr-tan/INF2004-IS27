/*
 *	board_io.h  (Buddy 1)
 *	Robo Pico on-board RGB LEDs (2 x WS2812 on GP18) and buzzer (GP22).
 *	Status task only.
 */
#ifndef BOARD_IO_H
#define BOARD_IO_H

/* colours, kept dim (0..255 per channel; full brightness is blinding) */
#define RGB_RED		40, 0, 0	/* no WiFi */
#define RGB_PINK	48, 6, 36	/* WiFi connected (v3.18's 40,4,16 looked red) */
#define RGB_ORANGE	40, 12, 0	/* found the hotspot, still connecting (no IP yet) */
#define RGB_GREEN	0, 40, 0	/* power-up colour test only */
#define RGB_BLUE	0, 0, 40
#define RGB_OFF		0, 0, 0

/* tunes for buzz_play() */
#define TUNE_WIFI	0	/* WiFi connected: three rising notes */
#define TUNE_WIFI_LOST	1	/* WiFi dropped: two falling notes */
#define TUNE_BARCODE	2	/* barcode read: one beep */
#define TUNE_BC_ACTION	3	/* barcode with a turn / stop action: two beeps */
#define TUNE_BOOT	4	/* power-up: short tick */

IMPORT void board_io_init(void);
IMPORT void rgb_set(UINT r, UINT g, UINT b);	/* both LEDs */
IMPORT void buzz_play(UINT tune);
IMPORT void buzz_step(UINT ms);

#endif
