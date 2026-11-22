/*
 *	imu_math.c  (Buddy 4)
 *	See imu_math.h.
 */
#include "imu_math.h"

/* atan(z) for z = 0..1000 (x1000), in 0.1 deg:
   pi/4 z + 0.273 z (1 - z) rad, max error ~0.2 deg */
static int32_t atan01_d10(int32_t z)
{
	return (450 * z) / 1000 + (1564 * z * (1000 - z)) / 10000000;
}

int32_t imu_atan2_d10(int32_t y, int32_t x)
{
	int32_t ax = x < 0 ? -x : x, ay = y < 0 ? -y : y, a;

	if (ax == 0 && ay == 0) return 0;
	if (ax >= ay) a = atan01_d10((int32_t)(((int64_t)ay * 1000) / ax));
	else          a = 900 - atan01_d10((int32_t)(((int64_t)ax * 1000) / ay));
	if (x < 0) a = 1800 - a;
	return y < 0 ? -a : a;
}

uint32_t imu_isqrt(uint32_t v)
{
	uint32_t r = 0, bit = 1U << 30;
	while (bit > v) bit >>= 2;
	while (bit) {
		if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
		else r >>= 1;
		bit >>= 2;
	}
	return r;
}

/* sin x 1000 every 5 deg, 0..90 */
static const int16_t sin5[19] = {
	0, 87, 174, 259, 342, 423, 500, 574, 643, 707,
	766, 819, 866, 906, 940, 966, 985, 996, 1000
};

int32_t imu_sin_x1000(int32_t d10)
{
	int32_t s = 1, i, f;
	while (d10 > 1800) d10 -= 3600;
	while (d10 < -1800) d10 += 3600;
	if (d10 < 0) { d10 = -d10; s = -1; }
	if (d10 > 900) d10 = 1800 - d10;
	i = d10 / 50;
	f = d10 % 50;
	if (i >= 18) return s * 1000;
	return s * (sin5[i] + ((sin5[i + 1] - sin5[i]) * f) / 50);
}

static int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

void hump_reset(HUMP *h)
{
	h->st = 0; h->t0 = 0; h->t_level = 0; h->s_prev = 0; h->s0 = 0; h->h_um = 0; h->peak_um = 0;
	h->base_um = 0;
}

/* Height is integrated ALL the time (sum of sin(pitch) x ds).  While the
   car is level, 'base' follows it; once a climb is confirmed, the hump's
   height is the highest point minus that base.  So the start of the
   climb, before the pitch crossed the threshold, is counted too. */
int32_t hump_step(HUMP *h, const HUMP_TUNE *k, int32_t p, int32_t odo, uint32_t now, int32_t *out)
{
	int32_t ds = odo - h->s_prev;
	int32_t level = (p < k->off_d10 && p > -k->off_d10);

	h->s_prev = odo;
	h->h_um += (int64_t)imu_sin_x1000(p) * ds;		/* mm x 1000 = um */
	if ((h->st == 2 || h->st == 3) && h->h_um - h->base_um > h->peak_um) h->peak_um = h->h_um - h->base_um;
	if (h->st == 6 && h->base_um - h->h_um > h->peak_um) h->peak_um = h->base_um - h->h_um;
	/* give up by DISTANCE (a slow car takes long over a hump; time
	   alone used to cut off humps at 30 mm/s), or after max_ms */
	if ((h->st == 2 || h->st == 3 || h->st == 6) &&
	    ((k->max_mm > 0 && iabs32(odo - h->s0) > k->max_mm) || (now - h->t0) > k->max_ms)) {
		h->st = 0; h->base_um = h->h_um; return 0;
	}

	switch (h->st) {
	case 0:
		if (level) h->base_um = h->h_um;
		if (p > k->on_d10) { h->st = 1; h->t0 = now; }
		else if (p < -k->on_d10) { h->st = 5; h->t0 = now; }
		break;
	case 1:
		if (p <= k->on_d10) h->st = 0;
		else if ((now - h->t0) >= k->confirm_ms) {
			h->st = 2; h->peak_um = h->h_um - h->base_um; h->t0 = now; h->s0 = odo; h->t_level = 0;
		}
		break;
	case 5:						/* maybe going down */
		if (p >= -k->on_d10) h->st = 0;
		else if ((now - h->t0) >= k->confirm_ms) {
			h->st = 6; h->peak_um = h->base_um - h->h_um; h->t0 = now; h->s0 = odo; h->t_level = 0;
		}
		break;
	case 6:						/* ramp down, no climb before it */
		if (level) {
			if (h->t_level == 0) h->t_level = now;
			else if ((now - h->t_level) >= k->level_ms) h->st = 7;
		} else h->t_level = 0;
		break;
	case 2:						/* climbing */
		if (p < -k->on_d10) { h->st = 3; h->t_level = 0; }
		else if (level) {
			if (h->t_level == 0) h->t_level = now;
			else if ((now - h->t_level) >= k->level_ms * 3) h->st = 4;	/* plateau / ramp top */
		} else h->t_level = 0;
		break;
	case 3:						/* coming down */
		if (level) {
			if (h->t_level == 0) h->t_level = now;
			else if ((now - h->t_level) >= k->level_ms) h->st = 4;
		} else h->t_level = 0;
		break;
	}
	if (h->st == 4 || h->st == 7) {
		int32_t mm = (int32_t)(h->peak_um / 1000);
		int32_t r = h->st == 4 ? 1 : 2;
		h->st = 0;
		h->base_um = h->h_um;
		h->peak_um = 0;
		if (mm >= k->min_mm) { *out = mm; return r; }
	}
	return 0;
}
