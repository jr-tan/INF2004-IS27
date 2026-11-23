/**
 * @file    code39.h
 *  code39.h  (Buddy 3)
 *  Code 39 decoder as PURE logic (no kernel calls); the PC test
 *  testing/barcode_test/ runs this same file.
 *
 *  Input: the widths of the elements in the order the sensor saw them,
 *  bar, space, bar, space ... (any unit; we use microseconds).
 *  Each character is 9 elements (5 bars, 4 spaces), exactly 3 wide,
 *  then one narrow gap.  A frame is  * TEXT *  so n = 10 x chars - 1.
 *
 *  Speed independent: in every character the 3 LONGEST elements are
 *  the wide ones, so a car that speeds up or slows down still reads.
 *  A frame read back to front (car going the other way) is reversed
 *  and decoded again.
 */
#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>

#define C39_ERR_LENGTH    (-1) /* element count isn't 10k - 1 */
#define C39_ERR_RATIO     (-2) /* wide and narrow too alike */
#define C39_ERR_UNKNOWN   (-3) /* pattern is no Code 39 character */
#define C39_ERR_FRAME     (-4) /* no '*' at both ends */
#define C39_ERR_SPACE     (-5) /* output buffer too small */
#define C39_ERR_FULLASCII (-6) /* $ % / + not followed by a valid letter */

/* returns the text length (start/stop '*' removed) or a C39_ERR_* value.
   ratio_x10: least wide/narrow ratio x10 (e.g. 15 = 1.5) */
int32_t code39_decode (const uint32_t * p_wrk, int32_t num, int32_t ratio_x10,
                       char * p_out, int32_t cap, int32_t * p_reversed);

/* Code 39 Full ASCII: turns the pairs $X %X /X +X of a decoded text into
   the 128 ASCII characters (e.g. "+A" -> 'a', "/A" -> '!', "$A" -> 0x01).
   Other characters are copied.  Returns the new length or C39_ERR_*.
   in and out may be the same buffer (out is never longer than in). */
int32_t code39_full_ascii (const char * p_inp, char * p_out, int32_t cap);

/* 9-element pattern of ch (bit 8 = first element, 1 = wide), or -1 */
int32_t code39_pattern (char chr);

#endif

/*** end of file ***/
