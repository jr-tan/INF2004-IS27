/*
 *	bitpack.c
 *	Subsystem 1 -- bit-level packer/unpacker.  See bitpack.h.
 */

#include <stddef.h>
#include "bitpack.h"

#define BITS_PER_BYTE	(8U)
#define BYTE_MASK	(0xFFU)

static uint32_t low_mask(uint8_t nbits)
{
	uint32_t mask;

	if (nbits >= BP_MAX_FIELD_BITS) {
		mask = UINT32_MAX;
	} else {
		mask = (UINT32_C(1) << nbits) - UINT32_C(1);
	}
	return mask;
}

static bool width_ok(uint8_t nbits)
{
	return (nbits >= 1U) && (nbits <= BP_MAX_FIELD_BITS);
}

void bp_wr_init(bp_writer_t *w, uint8_t *buf, uint16_t cap_bytes)
{
	uint16_t i;

	w->buf = buf;
	w->cap_bytes = cap_bytes;
	w->bit_pos = 0U;
	w->ok = (buf != NULL);
	w->clipped = false;

	for (i = 0U; (buf != NULL) && (i < cap_bytes); i++) {
		buf[i] = 0U;
	}
}

/* Writes the low nbits of value at the current position (already range-checked). */
static void put_raw(bp_writer_t *w, uint32_t value, uint8_t nbits)
{
	uint8_t remaining = nbits;

	while (remaining > 0U) {
		uint16_t byte_idx = (uint16_t)(w->bit_pos / BITS_PER_BYTE);
		uint8_t  bit_off  = (uint8_t)(w->bit_pos % BITS_PER_BYTE);
		uint8_t  room     = (uint8_t)(BITS_PER_BYTE - bit_off);
		uint8_t  take     = (remaining < room) ? remaining : room;
		uint32_t chunk    = value & low_mask(take);

		w->buf[byte_idx] |= (uint8_t)((chunk << bit_off) & BYTE_MASK);
		value = value >> take;
		remaining = (uint8_t)(remaining - take);
		w->bit_pos = (uint16_t)(w->bit_pos + take);
	}
}

static bool has_room(const bp_writer_t *w, uint8_t nbits)
{
	return w->ok &&
	       (((uint32_t)w->bit_pos + nbits) <= ((uint32_t)w->cap_bytes * BITS_PER_BYTE));
}

void bp_put_u(bp_writer_t *w, uint32_t value, uint8_t nbits)
{
	uint32_t max_val;

	if (!width_ok(nbits) || !has_room(w, nbits)) {
		w->ok = false;
		return;
	}

	max_val = low_mask(nbits);
	if (value > max_val) {
		value = max_val;	/* saturate rather than wrap */
		w->clipped = true;
	}
	put_raw(w, value, nbits);
}

void bp_put_s(bp_writer_t *w, int32_t value, uint8_t nbits)
{
	int32_t  lo;
	int32_t  hi;
	uint32_t raw;

	if (!width_ok(nbits) || !has_room(w, nbits)) {
		w->ok = false;
		return;
	}

	if (nbits == BP_MAX_FIELD_BITS) {
		lo = INT32_MIN;
		hi = INT32_MAX;
	} else {
		hi = (int32_t)((UINT32_C(1) << (nbits - 1U)) - UINT32_C(1));
		lo = -hi - 1;
	}
	if (value > hi) {
		value = hi;
		w->clipped = true;
	} else if (value < lo) {
		value = lo;
		w->clipped = true;
	} else {
		/* in range */
	}

	/* Two's complement: the conversion to unsigned is well defined, and all
	 * bitwise work is then done on unsigned data only (BARR-C 5.3.b). */
	raw = (uint32_t)value & low_mask(nbits);
	put_raw(w, raw, nbits);
}

uint16_t bp_wr_len(const bp_writer_t *w)
{
	return (uint16_t)((w->bit_pos + (BITS_PER_BYTE - 1U)) / BITS_PER_BYTE);
}

void bp_rd_init(bp_reader_t *r, const uint8_t *buf, uint16_t len_bytes)
{
	r->buf = buf;
	r->len_bytes = len_bytes;
	r->bit_pos = 0U;
	r->ok = (buf != NULL);
}

uint32_t bp_get_u(bp_reader_t *r, uint8_t nbits)
{
	uint32_t value = 0UL;
	uint8_t  got = 0U;

	if (!width_ok(nbits) ||
	    !r->ok ||
	    (((uint32_t)r->bit_pos + nbits) > ((uint32_t)r->len_bytes * BITS_PER_BYTE))) {
		r->ok = false;
		return 0UL;
	}

	while (got < nbits) {
		uint16_t byte_idx = (uint16_t)(r->bit_pos / BITS_PER_BYTE);
		uint8_t  bit_off  = (uint8_t)(r->bit_pos % BITS_PER_BYTE);
		uint8_t  room     = (uint8_t)(BITS_PER_BYTE - bit_off);
		uint8_t  left     = (uint8_t)(nbits - got);
		uint8_t  take     = (left < room) ? left : room;
		uint32_t chunk    = ((uint32_t)r->buf[byte_idx] >> bit_off) & low_mask(take);

		value |= (chunk << got);
		got = (uint8_t)(got + take);
		r->bit_pos = (uint16_t)(r->bit_pos + take);
	}
	return value;
}

int32_t bp_get_s(bp_reader_t *r, uint8_t nbits)
{
	uint32_t raw = bp_get_u(r, nbits);
	uint32_t sign;
	int32_t  result;

	if (!r->ok) {
		result = 0;
	} else if (nbits >= BP_MAX_FIELD_BITS) {
		result = (int32_t)raw;
	} else {
		sign = UINT32_C(1) << (nbits - 1U);
		/* (raw ^ sign) < 2^nbits <= 2^31, so both casts are exact. */
		result = (int32_t)(raw ^ sign) - (int32_t)sign;
	}
	return result;
}
