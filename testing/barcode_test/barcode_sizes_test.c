/*
 *	barcode_sizes_test.c: PC test for the course barcodes (12 Nov)
 *	docs/Barcode sample/BARCODE.pptx prints each one-letter barcode
 *	(*X*) 17, 25 and 33.87 cm wide.  Measured from the 33.87 cm print:
 *	narrow 7.29 mm, wide 21.4 mm (2.94 : 1).  This test:
 *	  - every letter A..Z, all three sizes, read forwards and backwards
 *	  - car at 25 and 30 mm/s with the speed wobbling +-WOB (zig-zag)
 *	  - checks the timings against barcode.c's limits
 *	  - Full ASCII pairs (+A -> a, /A -> !, ...)
 *
 *	gcc -O2 -I../../app_program/subsystem3 barcode_sizes_test.c \
 *	    ../../app_program/subsystem3/code39.c -lm -o barcode_sizes_test && ./barcode_sizes_test
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "code39.h"

#define BC_MAX_EL_MS	1500	/* keep in step with barcode.c */
#define BC_MAX_FRAME_MS	16000
#define RATIO_X10	15
/* speed wobble while reading.  The line task keeps the inner wheel at
   >= 50 % of the outer one over a barcode, about +-15 %; 0.25 leaves
   room.  (At +-40 %, a hard zig-zag, half the small codes fail.) */
#ifndef WOB
#define WOB 0.25
#endif

int main(void)
{
	const double sizes_cm[3] = { 17.0, 25.0, 33.87 };
	const double speeds[2] = { 25.0, 30.0 };
	int pass = 0, total = 0, lim_fail = 0;
	double worst_el = 0, worst_frame = 0;

	for (int si = 0; si < 3; si++)
	for (int vi = 0; vi < 2; vi++)
	for (char ch = 'A'; ch <= 'Z'; ch++)
	for (int rev = 0; rev < 2; rev++) {
		double scale = sizes_cm[si] / 33.87;
		double narrow = 7.29 * scale, wide = 21.4 * scale;
		char full[4] = { '*', ch, '*', 0 };
		double mm[40], pos = 0, t = 0, el_max = 0;
		uint32_t w[40];
		int n = 0;
		for (int c = 0; c < 3; c++) {
			int p = code39_pattern(full[c]);
			for (int i = 0; i < 9; i++) mm[n++] = ((p >> (8 - i)) & 1) ? wide : narrow;
			if (c < 2) mm[n++] = narrow;		/* gap between characters */
		}
		if (rev) for (int i = 0; i < n / 2; i++) { double x = mm[i]; mm[i] = mm[n - 1 - i]; mm[n - 1 - i] = x; }
		/* time each element with a wobbling speed (period ~1.2 s) */
		for (int i = 0; i < n; i++) {
			double start = t, left = mm[i];
			while (left > 0) {
				double v = speeds[vi] * (1.0 + WOB * sin(2 * M_PI * t / 1.2));
				double step = v * 0.001;		/* 1 ms */
				left -= step; t += 0.001;
			}
			w[i] = (uint32_t)((t - start) * 1e6);
			if ((t - start) * 1000 > el_max) el_max = (t - start) * 1000;
			pos += mm[i];
		}
		if (el_max > worst_el) worst_el = el_max;
		if (t * 1000 > worst_frame) worst_frame = t * 1000;
		if (el_max > BC_MAX_EL_MS || t * 1000 > BC_MAX_FRAME_MS) lim_fail++;
		{
			char out[16]; int32_t rv;
			int32_t r = code39_decode(w, n, RATIO_X10, out, sizeof out, &rv);
			if (r > 0) r = code39_full_ascii(out, out, sizeof out);
			total++;
			if (r == 1 && out[0] == ch) pass++;
			else printf("  miss: %c size %.0f cm %.0f mm/s %s -> %d\n", ch, sizes_cm[si], speeds[vi],
				    rev ? "backwards" : "forwards", r);
		}
	}
	printf("course barcodes A..Z, 3 sizes, 2 speeds (+-25%% speed wobble), both ways: %d / %d decoded\n", pass, total);
	printf("slowest bar %.0f ms (limit %d), longest barcode %.1f s (limit %.1f s): %s\n",
	       worst_el, BC_MAX_EL_MS, worst_frame / 1000, BC_MAX_FRAME_MS / 1000.0,
	       lim_fail ? "SOME OVER THE LIMITS" : "all within the limits");

	/* Full ASCII */
	{
		const char *in[]  = { "+A", "+Z", "/A", "/Z", "%V", "A+B", "IS27", "+I+S27" };
		const char *exp[] = { "a",  "z",  "!",  ":",  "@",  "Ab",  "IS27", "is27" };
		int ok = 0;
		for (int i = 0; i < 8; i++) {
			char out[16];
			int32_t r = code39_full_ascii(in[i], out, sizeof out);
			if (r >= 0 && strcmp(out, exp[i]) == 0) ok++;
			else printf("  full ascii: \"%s\" -> \"%s\" (want \"%s\")\n", in[i], r >= 0 ? out : "ERR", exp[i]);
		}
		printf("Full ASCII pairs: %d / 8 correct; bad pair \"+1\" -> %d (error expected)\n",
		       ok, code39_full_ascii("+1", (char[8]){0}, 8));
	}
	return 0;
}
