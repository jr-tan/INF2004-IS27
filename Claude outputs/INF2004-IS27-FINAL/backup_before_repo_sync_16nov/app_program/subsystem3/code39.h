/*
 *	code39.h  (Buddy 3)
 *	Code 39 decoder as PURE logic (no kernel calls); the PC test
 *	testing/barcode_test/ runs this same file.
 *
 *	Input: the widths of the elements in the order the sensor saw them,
 *	bar, space, bar, space ... (any unit; we use microseconds).
 *	Each character is 9 elements (5 bars, 4 spaces), exactly 3 wide,
 *	then one narrow gap.  A frame is  * TEXT *  so n = 10 x chars - 1.
 *
 *	Speed independent: in every character the 3 LONGEST elements are
 *	the wide ones, so a car that speeds up or slows down still reads.
 *	A frame read back to front (car going the other way) is reversed
 *	and decoded again.
 */
#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>

#define C39_ERR_LENGTH		(-1)	/* element count isn't 10k - 1 */
#define C39_ERR_RATIO		(-2)	/* wide and narrow too alike */
#define C39_ERR_UNKNOWN		(-3)	/* pattern is no Code 39 character */
#define C39_ERR_FRAME		(-4)	/* no '*' at both ends */
#define C39_ERR_SPACE		(-5)	/* output buffer too small */

/* returns the text length (start/stop '*' removed) or a C39_ERR_* value.
   ratio_x10: least wide/narrow ratio x10 (e.g. 15 = 1.5) */
int32_t code39_decode(const uint32_t *w, int32_t n, int32_t ratio_x10,
		      char *out, int32_t cap, int32_t *reversed);

/* 9-element pattern of ch (bit 8 = first element, 1 = wide), or -1 */
int32_t code39_pattern(char ch);

#endif
