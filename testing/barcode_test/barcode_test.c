/*
 *	barcode_test.c: PC test for app_program/subsystem3/code39.c
 *	Prints Code 39 barcodes as bar/space widths the way the car would
 *	time them: narrow ~ 30 ms, wide 2.5x, car speed drifting up to
 *	+-25 % across the barcode, +-15 % noise on every element, and read
 *	forwards or backwards.
 *
 *	gcc -O2 -I../../app_program/subsystem3 barcode_test.c \
 *	    ../../app_program/subsystem3/code39.c -o barcode_test && ./barcode_test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "code39.h"

static int make(const char *text, double narrow_us, double ratio, double drift,
		double noise, int reverse, uint32_t *w)
{
	char full[32];
	int n = 0, k, c, i;
	snprintf(full, sizeof full, "*%s*", text);
	k = (int)strlen(full);
	for (c = 0; c < k; c++) {
		int p = code39_pattern(full[c]);
		for (i = 0; i < 9; i++) w[n++] = (p >> (8 - i)) & 1 ? 1 : 0;
		if (c < k - 1) w[n++] = 0;	/* narrow gap */
	}
	for (i = 0; i < n; i++) {
		double speed = 1.0 + drift * ((double)i / n - 0.5) * 2;	/* linear drift */
		double base = (w[i] ? ratio : 1.0) * narrow_us / speed;
		double nz = 1.0 + noise * (((rand() % 2001) - 1000) / 1000.0);
		w[i] = (uint32_t)(base * nz);
	}
	if (reverse) for (i = 0; i < n / 2; i++) { uint32_t t = w[i]; w[i] = w[n - 1 - i]; w[n - 1 - i] = t; }
	return n;
}

int main(void)
{
	const char *texts[] = { "A", "B", "C", "D", "Z", "IS27", "0123", "HELLO", "X-1" };
	double drifts[] = { 0, 0.15, 0.25 }, noises[] = { 0, 0.10, 0.15 };
	int pass = 0, total = 0;
	srand(42);
	for (unsigned t = 0; t < sizeof texts / sizeof texts[0]; t++)
	for (int d = 0; d < 3; d++)
	for (int z = 0; z < 3; z++)
	for (int rev = 0; rev < 2; rev++)
	for (int rep = 0; rep < 20; rep++) {
		uint32_t w[200];
		char out[16];
		int32_t r, reversed;
		int n = make(texts[t], 30000, 2.5, drifts[d], noises[z], rev, w);
		r = code39_decode(w, n, 15, out, sizeof out, &reversed);
		total++;
		if (r > 0 && strcmp(out, texts[t]) == 0 && reversed == rev) pass++;
		else if (rep == 0) printf("  miss: \"%s\" drift %.0f%% noise %.0f%% %s -> %d \"%s\"\n",
				texts[t], drifts[d] * 100, noises[z] * 100, rev ? "reversed" : "forward",
				(int)r, r > 0 ? out : "");
	}
	printf("decoded %d / %d barcodes correctly\n", pass, total);
	/* garbage must not decode */
	{
		int false_hits = 0;
		for (int i = 0; i < 2000; i++) {
			uint32_t w[200]; char out[16]; int32_t rv;
			int n = 29 + 10 * (rand() % 4);
			for (int j = 0; j < n; j++) w[j] = 20000 + rand() % 60000;
			if (code39_decode(w, n, 15, out, sizeof out, &rv) > 0) false_hits++;
		}
		printf("random noise decoded as a barcode: %d / 2000\n", false_hits);
	}
	return 0;
}
