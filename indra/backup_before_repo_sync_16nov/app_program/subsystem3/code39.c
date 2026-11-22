/*
 *	code39.c  (Buddy 3)
 *	Code 39 decoding.  See code39.h.
 */
#include "code39.h"

/* standard Code 39 table: 9 elements b s b s b s b s b, bit 8 first, 1 = wide */
static const struct { char ch; uint16_t pat; } tab[] = {
	{'0',0x034},{'1',0x121},{'2',0x061},{'3',0x160},{'4',0x031},{'5',0x130},
	{'6',0x070},{'7',0x025},{'8',0x124},{'9',0x064},{'A',0x109},{'B',0x049},
	{'C',0x148},{'D',0x019},{'E',0x118},{'F',0x058},{'G',0x00D},{'H',0x10C},
	{'I',0x04C},{'J',0x01C},{'K',0x103},{'L',0x043},{'M',0x142},{'N',0x013},
	{'O',0x112},{'P',0x052},{'Q',0x007},{'R',0x106},{'S',0x046},{'T',0x016},
	{'U',0x181},{'V',0x0C1},{'W',0x1C0},{'X',0x091},{'Y',0x190},{'Z',0x0D0},
	{'-',0x085},{'.',0x184},{' ',0x0C4},{'*',0x094},{'$',0x0A8},{'/',0x0A2},
	{'+',0x08A},{'%',0x02A},
};
#define NTAB	((int32_t)(sizeof tab / sizeof tab[0]))

int32_t code39_pattern(char ch)
{
	int32_t i;
	for (i = 0; i < NTAB; i++) if (tab[i].ch == ch) return tab[i].pat;
	return -1;
}

/* one character from 9 widths */
static int32_t one_char(const uint32_t *w, int32_t ratio_x10, char *ch)
{
	uint32_t pat = 0, wide_min = 0xFFFFFFFFU, narrow_max = 0;
	int32_t i, j, rank;

	for (i = 0; i < 9; i++) {
		rank = 0;			/* how many are longer than w[i] */
		for (j = 0; j < 9; j++)
			if (w[j] > w[i] || (w[j] == w[i] && j < i)) rank++;
		if (rank < 3) { pat |= 1U << (8 - i); if (w[i] < wide_min) wide_min = w[i]; }
		else if (w[i] > narrow_max) narrow_max = w[i];
	}
	if ((uint64_t)wide_min * 10U < (uint64_t)narrow_max * (uint32_t)ratio_x10) return C39_ERR_RATIO;
	for (i = 0; i < NTAB; i++)
		if (tab[i].pat == pat) { *ch = tab[i].ch; return 0; }
	return C39_ERR_UNKNOWN;
}

static int32_t decode_dir(const uint32_t *w, int32_t n, int32_t ratio_x10, char *out, int32_t cap)
{
	int32_t k = (n + 1) / 10, c, len = 0, r;
	char ch;

	for (c = 0; c < k; c++) {
		if ((r = one_char(&w[10 * c], ratio_x10, &ch)) < 0) return r;
		if (c == 0 || c == k - 1) {
			if (ch != '*') return C39_ERR_FRAME;
		} else {
			if (ch == '*') return C39_ERR_FRAME;
			if (len >= cap - 1) return C39_ERR_SPACE;
			out[len++] = ch;
		}
	}
	out[len] = '\0';
	return len;
}

int32_t code39_decode(const uint32_t *w, int32_t n, int32_t ratio_x10,
		      char *out, int32_t cap, int32_t *reversed)
{
	uint32_t rev[10 * 12];
	int32_t r, i;

	*reversed = 0;
	if (n < 19 || (n + 1) % 10 != 0 || n > (int32_t)(sizeof rev / sizeof rev[0]))
		return C39_ERR_LENGTH;
	r = decode_dir(w, n, ratio_x10, out, cap);
	if (r >= 0) return r;
	for (i = 0; i < n; i++) rev[i] = w[n - 1 - i];
	if (decode_dir(rev, n, ratio_x10, out, cap) >= 0) {
		*reversed = 1;
		for (i = 0; out[i]; i++) { }
		return i;
	}
	return r;			/* the forward error is the more useful one */
}
