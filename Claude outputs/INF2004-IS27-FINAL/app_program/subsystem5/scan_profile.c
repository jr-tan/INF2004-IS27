/*
 *	scan_profile.c  (Buddy 5)
 *	Fan sweep -> obstacle profile.  Pure logic, see scan_profile.h.
 */
#include "scan_profile.h"

/* sin x 1000 for 0, 5, 10 ... 90 degrees */
static const int16_t sin5[19] = {
	   0,   87,  174,  259,  342,  423,  500,  574,  643,  707,
	 766,  819,  866,  906,  940,  966,  985,  996, 1000
};

int32_t isin_x1000(int32_t deg)
{
	int32_t s = 1, i, f, v;

	while (deg > 180) deg -= 360;
	while (deg < -180) deg += 360;
	if (deg < 0) { deg = -deg; s = -1; }
	if (deg > 90) deg = 180 - deg;
	i = deg / 5;
	f = deg % 5;
	v = (i >= 18) ? 1000 : sin5[i] + ((sin5[i + 1] - sin5[i]) * f) / 5;
	return s * v;
}

static int32_t lat_cm(int32_t cm, int32_t deg)	/* sideways position, + = left */
{
	return (cm * isin_x1000(deg)) / 1000;
}

void scan_profile(const SCAN_TUNE *k, const int32_t *deg, const int32_t *cm,
		  int32_t n, SCAN_PROFILE *p)
{
	int32_t i, imin = -1, il, ir, lim;

	p->found = 0;
	p->dir = 0;
	p->need_cm = 0;
	if (n <= 0 || n > SCAN_MAX_PTS) return;

	/* 1. closest reading (ties: the one nearest straight ahead) */
	for (i = 0; i < n; i++) {
		int32_t a = deg[i] < 0 ? -deg[i] : deg[i];
		if (cm[i] >= k->far_cm) continue;
		if (imin < 0 || cm[i] < cm[imin] ||
		    (cm[i] == cm[imin] && a < (deg[imin] < 0 ? -deg[imin] : deg[imin])))
			imin = i;
	}
	if (imin < 0) return;			/* nothing in front */
	p->found = 1;
	p->dmin_cm = cm[imin];

	/* 2. grow the run both ways while readings stay near dmin */
	lim = cm[imin] + k->band_cm;
	/* (one missed echo inside the obstacle is bridged) */
	il = imin;
	while (il > 0 && (cm[il - 1] <= lim || (il > 1 && cm[il - 2] <= lim)))
		il -= (cm[il - 1] <= lim) ? 1 : 2;
	ir = imin;
	while (ir < n - 1 && (cm[ir + 1] <= lim || (ir < n - 2 && cm[ir + 2] <= lim)))
		ir += (cm[ir + 1] <= lim) ? 1 : 2;
	p->open_left  = il > 0;			/* saw past the left edge */
	p->open_right = ir < n - 1;

	/* 3. edges, pulled in by the sound cone, never crossing */
	p->left_deg  = deg[il] - k->beam_deg;
	p->right_deg = deg[ir] + k->beam_deg;
	if (p->left_deg < p->right_deg) {
		int32_t mid = (deg[il] + deg[ir]) / 2;
		p->left_deg = p->right_deg = mid;
	}
	p->left_cm  = lat_cm(cm[il], p->left_deg);
	p->right_cm = lat_cm(cm[ir], p->right_deg);
	if (p->right_cm > p->left_cm) {		/* edges crossed: one narrow thing */
		int32_t mid = (p->left_cm + p->right_cm) / 2;
		p->left_cm = p->right_cm = mid;
	}
	p->width_cm = p->left_cm - p->right_cm;
	p->bearing_deg = (deg[il] + deg[ir]) / 2;

	/* 4./5. sideways move needed for each side (never negative) */
	p->need_left_cm  = p->left_cm  + k->half_car_cm + k->margin_cm;
	p->need_right_cm = -p->right_cm + k->half_car_cm + k->margin_cm;
	if (p->need_left_cm < 0)  p->need_left_cm = 0;
	if (p->need_right_cm < 0) p->need_right_cm = 0;

	if (p->open_left && p->open_right) {
		if (p->need_left_cm == p->need_right_cm) p->dir = k->prefer_left ? 1 : -1;
		else p->dir = (p->need_left_cm < p->need_right_cm) ? 1 : -1;
	} else if (p->open_left) {
		p->dir = 1;
	} else if (p->open_right) {
		p->dir = -1;
	}
	p->need_cm = p->dir > 0 ? p->need_left_cm : p->dir < 0 ? p->need_right_cm : 0;
}
