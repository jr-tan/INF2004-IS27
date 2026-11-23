/*
 *	imu_test.c: PC test for app_program/subsystem4/imu_math.c
 *	  1. integer atan2 / sqrt / sin against the PC's maths library
 *	  2. the hump detector on made-up humps (smooth bump, given height
 *	     and length, car at 120 mm/s, 50 Hz samples, sensor noise)
 *
 *	gcc -O2 -I../../app_program/subsystem4 imu_test.c \
 *	    ../../app_program/subsystem4/imu_math.c -lm -o imu_test && ./imu_test
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "imu_math.h"

int main(void)
{
	double worst = 0;
	int fails = 0;

	for (int y = -2000; y <= 2000; y += 37)
		for (int x = -2000; x <= 2000; x += 41) {
			if (!x && !y) continue;
			double e = fabs(imu_atan2_d10(y, x) / 10.0 - atan2(y, x) * 180 / M_PI);
			if (e > 180) e = 360 - e;
			if (e > worst) worst = e;
		}
	printf("atan2: worst error %.2f deg %s\n", worst, worst < 0.5 ? "OK" : "TOO BIG");
	fails += worst >= 0.5;

	worst = 0;
	for (unsigned v = 0; v < 40000000; v += 9973) {
		double e = fabs((double)imu_isqrt(v) - floor(sqrt(v)));
		if (e > worst) worst = e;
	}
	printf("isqrt: worst error %.0f %s\n", worst, worst == 0 ? "OK" : "WRONG");
	fails += worst != 0;

	worst = 0;
	for (int d = -1800; d <= 1800; d++) {
		double e = fabs(imu_sin_x1000(d) / 1000.0 - sin(d / 10.0 * M_PI / 180));
		if (e > worst) worst = e;
	}
	printf("sin: worst error %.4f %s\n", worst, worst < 0.005 ? "OK" : "TOO BIG");
	fails += worst >= 0.005;

	/* humps: height h, length L, shape h * (1 - cos(2 pi x / L)) / 2 */
	HUMP_TUNE k = { 35, 20, 100, 200, 60000, 5, 1000 };	/* same as subsystem4.c */
	double hs[] = { 15, 25, 40, 60 }, Ls[] = { 150, 250, 400 };
	printf("hump    length  -> detected height\n");
	srand(3);
	static const double speeds[] = { 120, 30 };	/* mm/s: v3.13 drives at 30 */
	for (int v = 0; v < 2; v++)
	for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) {
		HUMP h; hump_reset(&h);
		int32_t got = -1, mm;
		if (atan(hs[i] * M_PI / Ls[j]) * 180 / M_PI > 35) {	/* steeper than the tilt limit */
			printf("  %3.0f mm %4.0f mm -> (slope over 35 deg: the car would stop for tilt)\n", hs[i], Ls[j]);
			continue;
		}
		if (atan(hs[i] * M_PI / Ls[j]) * 180 / M_PI > 35) {	/* steeper than the tilt limit */
			printf("  %3.0f mm %4.0f mm -> (slope over 35 deg: the car would stop for tilt)\n", hs[i], Ls[j]);
			continue;
		}
		double s = 0;
		for (int t = 0; t < 40000; t += 20) {
			double x = s - 300, slope = 0;		/* hump starts 30 cm in */
			if (x > 0 && x < Ls[j]) slope = hs[i] * M_PI / Ls[j] * sin(2 * M_PI * x / Ls[j]);
			int32_t p = (int32_t)(atan(slope) * 1800 / M_PI) + (rand() % 21) - 10;	/* +-1 deg noise */
			if (hump_step(&h, &k, p, (int32_t)s, (uint32_t)t, &mm)) got = mm;
			s += speeds[v] * 0.02;
			if (s > 300 + Ls[j] + 400) break;
		}
		int ok = got > 0 && fabs(got - hs[i]) <= 0.15 * hs[i] + 3;
		fails += !ok;
		printf("  %3.0f mm %4.0f mm at %3.0f mm/s -> %3d mm  %s\n", hs[i], Ls[j], speeds[v], (int)got, ok ? "OK" : "OFF");
	}
	/* one-sided ramps: up onto a plateau, and down off one */
	printf("ramp    length  -> detected\n");
	for (int v = 0; v < 2; v++)
	for (int dn = 0; dn < 2; dn++)
	for (int i = 0; i < 3; i++) {
		static const double rh[] = { 20, 40, 60 }, rl = 300;
		HUMP h; hump_reset(&h);
		int32_t got = -1, kind = 0, mm, r;
		double s = 0, ang = atan(rh[i] / rl);
		for (int t = 0; t < 60000; t += 20) {
			double x = s - 300, slope = (x > 0 && x < rl) ? (dn ? -ang : ang) : 0;
			int32_t p = (int32_t)(slope * 1800 / M_PI) + (rand() % 21) - 10;
			if ((r = hump_step(&h, &k, p, (int32_t)s, (uint32_t)t, &mm))) { got = mm; kind = r; }
			s += speeds[v] * 0.02;
			if (s > 300 + rl + 400) break;
		}
		int ok = got > 0 && kind == (dn ? 2 : 1) && fabs(got - rh[i]) <= 0.15 * rh[i] + 3;
		fails += !ok;
		printf("  %s %3.0f mm at %3.0f mm/s -> %s %3d mm  %s\n", dn ? "down" : "up  ", rh[i], speeds[v],
		       kind == 2 ? "drop" : kind == 1 ? "rise" : "none", (int)got, ok ? "OK" : "OFF");
	}
	/* flat floor with noise: no hump */
	{
		HUMP h; hump_reset(&h); int32_t mm, n = 0; double s = 0;
		for (int t = 0; t < 60000; t += 20) { n += hump_step(&h, &k, (rand() % 41) - 20, (int32_t)s, t, &mm); s += 2.4; }
		printf("flat floor, 60 s, +-2 deg noise: %d false humps %s\n", (int)n, n == 0 ? "OK" : "WRONG");
		fails += n != 0;
	}
	printf("%s\n", fails ? "FAILURES" : "all passed");
	return fails != 0;
}
