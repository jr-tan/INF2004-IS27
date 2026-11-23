/*
 *	scan_test.c: PC test for app_program/subsystem5/scan_profile.c
 *	Makes fake fan sweeps of a box in front of the car (with the
 *	ultrasonic's ~15 deg sound cone) and checks the width estimate and
 *	which way round the car chooses.
 *
 *	gcc -O2 -I../../app_program/subsystem5 scan_test.c \
 *	    ../../app_program/subsystem5/scan_profile.c -lm -o scan_test && ./scan_test
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scan_profile.h"

#define CONE 15.0	/* half-angle of the HC-SR04 sound cone, deg */

/* box front face at forward distance d (cm), from x = c - w/2 to c + w/2 */
static int reading(double th_deg, double d, double w, double c)
{
	double best = 1e9;
	for (double ph = th_deg - CONE; ph <= th_deg + CONE; ph += 0.5) {
		double t = tan(ph * M_PI / 180), x = d * t;	/* + = left */
		if (fabs(ph) < 89 && x >= c - w / 2 && x <= c + w / 2) {
			double r = d / cos(ph * M_PI / 180);
			if (r < best) best = r;
		}
	}
	if (best > 254) return 255;
	return (int)(best + (rand() % 3) - 1);		/* +-1 cm noise */
}

int main(void)
{
	scan_tune_t k = { 20, 100, 0, 9, 5, 1 };	/* same values as the firmware (12 Nov: beam 0) */
	if (getenv("BEAM")) k.beam_deg = atoi(getenv("BEAM"));
	struct { double d, w, c; int want_dir; } t[] = {
		{12, 10,  0,  1}, {12, 10,  4, -1}, {12, 10, -4,  1},
		{15, 20,  0,  1}, {10,  6,  0,  1}, {20, 30, -8,  1},
		{15, 20,  6, -1}, {25,  8,  0,  1}, {10, 60, 20, -1},
	};
	int fails = 0;
	int smax = getenv("SWEEP_MAX") ? atoi(getenv("SWEEP_MAX")) : 24;	/* firmware v3.16: +-24 */
	int sstep = getenv("SWEEP_STEP") ? atoi(getenv("SWEEP_STEP")) : 8;	/* in 8 deg steps */
	srand(7);
	printf("  box (dist, width, offset)  | est width  edges L/R cm   go     move  ok\n");
	for (int rep = 0; rep < 2; rep++) {
	if (rep) printf("\n  ... same boxes, one random missed echo per sweep:\n");
	else printf("  clean sweeps (+%d to -%d in %d deg steps):\n", smax, smax, sstep);
	for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
		int32_t deg[SCAN_MAX_PTS], cm[SCAN_MAX_PTS], n = 0;
		scan_profile_t p;
		for (int a = smax; a >= -smax; a -= sstep) { deg[n] = a; cm[n] = reading(a, t[i].d, t[i].w, t[i].c); n++; }
		if (rep) cm[(rand() % (n - 2)) + 1] = 255;	/* one missed echo */
		scan_profile_compute(&k, deg, cm, n, &p);
		double true_l = t[i].c + t[i].w / 2, true_r = t[i].c - t[i].w / 2;
		/* a box this wide this close fills the whole +-60 sweep: the
		   car can't see past either edge, so it must wait (BLOCK) */
		if (t[i].w >= 60) {
			int ok0 = p.found && p.dir == 0;
			fails += !ok0;
			printf("  %3.0f cm %3.0f cm %+3.0f cm      | wider than the sweep -> %s  %s\n", t[i].d, t[i].w, t[i].c,
			       p.dir == 0 ? "BLOCK (waits)" : "goes round", ok0 ? "yes" : "NO");
			continue;
		}
		if (p.found && p.dir == 0) {
			/* the box fills the narrow view: the car guesses the roomier
			   end of the sweep (or left) and moves 35 cm sideways.  Safe
			   if that clears the TRUE edge + half the car. */
			int g = cm[0] > cm[n - 1] ? 1 : cm[0] < cm[n - 1] ? -1 : 1;
			double need_g = (g > 0) ? t[i].c + t[i].w / 2 + 9 : -(t[i].c - t[i].w / 2) + 9;
			int okg = need_g <= 35;	/* AV_UNKNOWN_NEED_CM */
			fails += !okg;
			printf("  %3.0f cm %3.0f cm %+3.0f cm      | fills the view -> guesses %s 35 cm (needs %2.0f)  %s\n",
			       t[i].d, t[i].w, t[i].c, g > 0 ? "LEFT" : "RIGHT", need_g, okg ? "yes" : "NO");
			continue;
		}
		int ok = p.found && (p.dir == t[i].want_dir || (t[i].c == 0 && p.dir != 0)) &&	/* centred: either side is fine */
			 fabs(p.width_cm - t[i].w) <= 0.5 * t[i].w + 3;
		/* safety: the chosen move must clear the TRUE edge + half car */
		double need_true = (p.dir > 0) ? true_l + 9 : -true_r + 9;
		if (p.need_cm < need_true) ok = 0;
		fails += !ok;
		printf("  %3.0f cm %3.0f cm %+3.0f cm      | %3d cm   %+4d / %+4d    %-5s  %3d  %s\n",
		       t[i].d, t[i].w, t[i].c, p.width_cm, p.left_cm, p.right_cm,
		       p.dir > 0 ? "LEFT" : p.dir < 0 ? "RIGHT" : "BLOCK", p.need_cm, ok ? "yes" : "NO");
	}
	}
	printf("%s\n", fails ? "FAILURES" : "all passed");
	return fails != 0;
}
