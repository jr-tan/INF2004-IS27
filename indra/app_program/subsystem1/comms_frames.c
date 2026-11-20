/*
 *	comms_frames.c  (Buddy 1)
 *	Packing of the binary MQTT frames.  See comms_frames.h.
 */
#include "comms_frames.h"

void bits_put(uint8_t *buf, uint32_t pos, uint32_t width, uint32_t v)
{
	uint32_t i;
	for (i = 0; i < width; i++) {
		if ((v >> i) & 1U) buf[(pos + i) >> 3] |= (uint8_t)(1U << ((pos + i) & 7U));
	}
}

uint32_t bits_get(const uint8_t *buf, uint32_t pos, uint32_t width)
{
	uint32_t i, v = 0;
	for (i = 0; i < width; i++) {
		if ((buf[(pos + i) >> 3] >> ((pos + i) & 7U)) & 1U) v |= 1U << i;
	}
	return v;
}

static void zero(uint8_t *b, uint32_t n) { while (n--) *b++ = 0; }

static uint32_t sat_u(int32_t v, uint32_t width)
{
	int32_t max = (int32_t)((1U << width) - 1U);
	return (uint32_t)(v < 0 ? 0 : v > max ? max : v);
}

static uint32_t sat_s(int32_t v, uint32_t width)	/* two's complement in width */
{
	int32_t max = (int32_t)((1U << (width - 1)) - 1U), min = -max - 1;
	v = v < min ? min : v > max ? max : v;
	return (uint32_t)v & ((1U << width) - 1U);
}

uint32_t fr_status(uint8_t *b, uint32_t seq, uint32_t up_s, uint32_t rssi_neg,
		   uint32_t mode, uint32_t faults, uint32_t tx_drop)
{
	zero(b, FR_STATUS_BYTES);
	bits_put(b, 0, 4, seq & 0xFU);
	bits_put(b, 4, 20, up_s & 0xFFFFFU);		/* wraps after 12 days */
	bits_put(b, 24, 7, sat_u((int32_t)rssi_neg, 7));
	bits_put(b, 31, 4, sat_u((int32_t)mode, 4));
	bits_put(b, 35, 12, faults & 0xFFFU);
	bits_put(b, 47, 1, tx_drop ? 1U : 0U);
	return FR_STATUS_BYTES;
}

uint32_t fr_fault(uint8_t *b, uint32_t t_ms, uint32_t raised)
{
	zero(b, FR_FAULT_BYTES);
	bits_put(b, 0, 16, t_ms & 0xFFFFU);
	bits_put(b, 16, 12, raised & 0xFFFU);
	return FR_FAULT_BYTES;
}

uint32_t fr_motion(uint8_t *b, uint32_t t_ms, int32_t vl, int32_t vr,
		   int32_t dist_cm, uint32_t owner, uint32_t enc_ok)
{
	zero(b, FR_MOTION_BYTES);
	bits_put(b, 0, 16, t_ms & 0xFFFFU);
	bits_put(b, 16, 11, sat_s(vl, 11));
	bits_put(b, 27, 11, sat_s(vr, 11));
	bits_put(b, 38, 14, sat_u(dist_cm, 14));
	bits_put(b, 52, 2, owner & 3U);
	bits_put(b, 54, 1, enc_ok & 1U);		/* left encoder healthy */
	bits_put(b, 55, 1, (enc_ok >> 1) & 1U);	/* right encoder healthy (was a spare bit) */
	return FR_MOTION_BYTES;
}

uint32_t fr_line(uint8_t *b, uint32_t t_ms, uint32_t mode, int32_t pos,
		 uint32_t see, char bc_char, uint32_t bc_action, int32_t speed_mms,
		 int32_t bend_pct, int32_t kp_x1000, uint32_t pivot)
{
	uint32_t ci = (bc_char >= ' ' && bc_char <= 'Z') ? (uint32_t)(bc_char - ' ' + 1) : 0U;

	zero(b, FR_LINE_BYTES);
	bits_put(b, 0, 16, t_ms & 0xFFFFU);
	bits_put(b, 16, 3, mode & 7U);
	bits_put(b, 19, 8, sat_s(pos / 8, 8));
	bits_put(b, 27, 2, see & 3U);
	bits_put(b, 29, 6, ci);				/* 0 none, 1 = ' ' .. 59 = 'Z' */
	bits_put(b, 35, 3, bc_action & 7U);
	bits_put(b, 38, 8, sat_u(speed_mms / 10, 8));
	bits_put(b, 46, 7, sat_u(bend_pct, 7));
	bits_put(b, 53, 5, sat_u(kp_x1000 / 100, 5));
	bits_put(b, 58, 1, pivot ? 1U : 0U);
	return FR_LINE_BYTES;
}

uint32_t fr_cmd_tune(const uint8_t *b, uint32_t len, int32_t *mms, int32_t *bend, int32_t *kp,
		     int32_t *pivot)
{
	if (len < 2 || len > 5 || b[0] != 0) return 0;
	*mms  = (int32_t)b[1] * 10;
	*bend = len >= 3 ? (int32_t)b[2] : 0;
	*kp   = len >= 4 ? (int32_t)b[3] * 100 : 0;
	*pivot = len >= 5 ? (int32_t)b[4] : 0;		/* 0 unchanged, 1 off, 2 on */
	return 1;
}

uint32_t fr_terrain(uint8_t *b, uint32_t t_ms, int32_t pitch_d10, int32_t roll_d10,
		    int32_t hump_mm, int32_t heading, uint32_t events)
{
	zero(b, FR_TERRAIN_BYTES);
	bits_put(b, 0, 16, t_ms & 0xFFFFU);
	bits_put(b, 16, 10, sat_s(pitch_d10, 10));
	bits_put(b, 26, 10, sat_s(roll_d10, 10));
	bits_put(b, 36, 8, sat_u(hump_mm, 8));
	bits_put(b, 44, 9, heading < 0 ? 511U : sat_u(heading, 9));
	bits_put(b, 53, 3, events & 7U);
	return FR_TERRAIN_BYTES;
}

uint32_t fr_obstacle(uint8_t *b, uint32_t t_ms, int32_t dist, int32_t bearing,
		     int32_t width, int32_t need_l, int32_t need_r, uint32_t action,
		     uint32_t stage)
{
	zero(b, FR_OBSTACLE_BYTES);
	bits_put(b, 0, 16, t_ms & 0xFFFFU);
	bits_put(b, 16, 8, sat_u(dist, 8));
	bits_put(b, 24, 8, sat_s(bearing, 8));
	bits_put(b, 32, 8, sat_u(width, 8));
	bits_put(b, 40, 8, sat_u(need_l, 8));
	bits_put(b, 48, 8, sat_u(need_r, 8));
	bits_put(b, 56, 2, action & 3U);
	bits_put(b, 58, 3, stage & 7U);
	return FR_OBSTACLE_BYTES;
}

uint32_t fr_resources(uint8_t *b, uint32_t t_ms, const FR_RES *r)
{
	zero(b, FR_RESOURCES_BYTES);
	bits_put(b, 0, 16, t_ms & 0xFFFFU);
	bits_put(b, 16, 7, sat_u((int32_t)r->cpu0, 7));
	bits_put(b, 23, 7, sat_u((int32_t)r->cpu1, 7));
	bits_put(b, 30, 6, sat_u((int32_t)r->tasks, 6));
	bits_put(b, 36, 6, sat_u((int32_t)r->tasks_max, 6));
	bits_put(b, 42, 5, sat_u((int32_t)r->flags, 5));
	bits_put(b, 47, 5, sat_u((int32_t)r->flags_max, 5));
	bits_put(b, 52, 5, sat_u((int32_t)r->sems, 5));
	bits_put(b, 57, 5, sat_u((int32_t)r->sems_max, 5));
	bits_put(b, 62, 4, sat_u((int32_t)r->mbfs, 4));
	bits_put(b, 66, 4, sat_u((int32_t)r->mbfs_max, 4));
	bits_put(b, 70, 4, sat_u((int32_t)r->cycs, 4));
	bits_put(b, 74, 4, sat_u((int32_t)r->cycs_max, 4));
	bits_put(b, 78, 7, sat_u((int32_t)r->queue_pct, 7));
	bits_put(b, 85, 8, sat_u((int32_t)r->drops, 8));
	bits_put(b, 93, 12, sat_u((int32_t)r->motion_us, 12));
	bits_put(b, 105, 12, sat_u((int32_t)r->line_us, 12));
	return FR_RESOURCES_BYTES;
}

uint32_t fr_cmd_decode(const uint8_t *b, uint32_t len, uint32_t *id)
{
	uint32_t code;
	if (len != 1) return 0;
	code = b[0] & 7U;
	*id = (uint32_t)b[0] >> 3;
	return (code >= 1 && code <= 7) ? code : 0;
}

uint32_t fr_map_decode(const uint8_t *b, uint32_t len, uint8_t *act)
{
	uint32_t i;
	if (len != FR_MAP_BYTES) return 0;
	for (i = 0; i < FR_MAP_LETTERS; i++) {
		if (bits_get(b, i * 3U, 3U) > FR_MAP_ACT_MAX) return 0;
	}
	for (i = 0; i < FR_MAP_LETTERS; i++) act[i] = (uint8_t)bits_get(b, i * 3U, 3U);
	return 1;
}

uint32_t fr_map_ack(uint8_t *b, const uint8_t *act, uint32_t received)
{
	uint32_t i;
	zero(b, FR_MAP_ACK_BYTES);
	for (i = 0; i < FR_MAP_LETTERS; i++) {
		bits_put(b, i * 3U, 3U, act[i] > FR_MAP_ACT_MAX ? 0U : act[i]);
	}
	b[FR_MAP_BYTES] = (uint8_t)(received ? 1U : 0U);
	return FR_MAP_ACK_BYTES;
}
