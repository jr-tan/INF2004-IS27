/*
 *	bitpack.h
 *	Subsystem 1 -- bit-level packer/unpacker for compact binary frames.
 *
 *	Why not C bit-fields?  BARR-C:2018 rule 5.5 warns that bit-field
 *	ordering and padding are compiler-defined, which is unacceptable for
 *	data that leaves the chip.  This module places every field at an
 *	explicit bit offset instead, so the wire layout is identical on every
 *	compiler and on the host-side decoder.
 *
 *	Wire layout (the same rule for every frame):
 *	  - the frame is one little-endian bit string;
 *	  - stream bit i is bit (i % 8) of byte (i / 8);
 *	  - the first field written occupies the lowest bits, and within a field
 *	    the least-significant bit comes first.
 *	So a host can decode with  v = int.from_bytes(frame, "little")  and
 *	field = (v >> offset) & ((1 << width) - 1).
 *
 *	Pure C99, no kernel dependency, so it is unit-tested on the PC
 *	(testing/subsystem1/host/).
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>
#include <stdbool.h>

#define BP_MAX_FIELD_BITS	(32U)

typedef struct {
	uint8_t		*buf;
	uint16_t	cap_bytes;
	uint16_t	bit_pos;
	bool		ok;		/* false after any capacity/argument error */
	bool		clipped;	/* true if any value had to be clamped */
} bp_writer_t;

typedef struct {
	const uint8_t	*buf;
	uint16_t	len_bytes;
	uint16_t	bit_pos;
	bool		ok;		/* false after reading past the end */
} bp_reader_t;

/* Writer.  bp_wr_init() zeroes the buffer, so unused bits are always 0. */
void		bp_wr_init(bp_writer_t *w, uint8_t *buf, uint16_t cap_bytes);
void		bp_put_u(bp_writer_t *w, uint32_t value, uint8_t nbits);
void		bp_put_s(bp_writer_t *w, int32_t value, uint8_t nbits);
uint16_t	bp_wr_len(const bp_writer_t *w);	/* bytes used, rounded up */

/* Reader. */
void		bp_rd_init(bp_reader_t *r, const uint8_t *buf, uint16_t len_bytes);
uint32_t	bp_get_u(bp_reader_t *r, uint8_t nbits);
int32_t		bp_get_s(bp_reader_t *r, uint8_t nbits);

#endif /* BITPACK_H */
