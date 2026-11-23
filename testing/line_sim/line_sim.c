/*
 *	line_sim.c: PC simulator for app_program/subsystem3/line_ctl.c
 *
 *	Drives a model of the car (2 wheels, 120 mm track, open-loop motors
 *	with lag and a left/right mismatch, 2 analog IR sensors straddling a
 *	19 mm black line) around test tracks and reports whether the
 *	controller stays on the line.  The controller file is the SAME one
 *	the firmware uses.
 *
 *	Build + run:  gcc -O2 -I../../app_program/subsystem3 line_sim.c \
 *	                  ../../app_program/subsystem3/line_ctl.c -lm -o line_sim
 *	              ./line_sim
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "line_ctl.h"

#define MAXP 20000
static double px[MAXP], py[MAXP];
static int np;
/* crossing lines (cross junctions): centre + track heading there */
#define MAXX 16
static double xcx[MAXX], xcy[MAXX], xch[MAXX];
static int nx;
static const double STEP = 2.0;		/* mm between track points */

static void add_straight(double len, double *x, double *y, double *h)
{
	for (double s = 0; s < len; s += STEP) {
		*x += STEP * cos(*h); *y += STEP * sin(*h);
		px[np] = *x; py[np] = *y; np++;
	}
}
static void add_arc(double r, double deg, double *x, double *y, double *h)
{
	double total = fabs(deg) * M_PI / 180.0 * r;
	double dir = deg > 0 ? 1 : -1;		/* + = left */
	for (double s = 0; s < total; s += STEP) {
		*h += dir * STEP / r;
		*x += STEP * cos(*h); *y += STEP * sin(*h);
		px[np] = *x; py[np] = *y; np++;
	}
}

/* track = list of (radius, degrees); radius 0 = straight of 'deg' mm */
static void build(const double *seg, int n)
{
	double x = 0, y = 0, h = M_PI / 2;	/* start heading +y */
	np = 0;
	px[np] = x; py[np] = y; np++;
	nx = 0;
	for (int i = 0; i < n; i++) {
		if (seg[2 * i] < 0) {		/* (-1, 0): a line crossing here */
			if (nx < MAXX) { xcx[nx] = x; xcy[nx] = y; xch[nx] = h; nx++; }
			continue;
		}
		if (seg[2 * i] == 0) add_straight(seg[2 * i + 1], &x, &y, &h);
		else add_arc(seg[2 * i], seg[2 * i + 1], &x, &y, &h);
	}
}

static int near_idx(double x, double y, int hint, double *dmin)
{
	int lo = hint - 150, hi = hint + 150, best = hint;
	double bd = 1e18;
	if (lo < 0) lo = 0;
	if (hi > np - 1) hi = np - 1;
	for (int i = lo; i <= hi; i++) {
		double dx = px[i] - x, dy = py[i] - y, d = dx * dx + dy * dy;
		if (d < bd) { bd = d; best = i; }
	}
	*dmin = sqrt(bd);
	return best;
}

/* sensor model: darkness 0..4095 from distance to the line centre */
static double LINE_W = 19.0, BLUR = 3.0;
static double EDGE_X = 0;		/* edge layout: start on the line's edge */
static double DEADBAND = 20, TAU = 0.08; static int LAT_TICKS = 1;
static int sense(double x, double y, int hint)
{
	double d;
	near_idx(x, y, hint, &d);
	for (int i = 0; i < nx; i++) {	/* distance to a crossing line */
		double dx = x - xcx[i], dy = y - xcy[i];
		double along = fabs(dx * cos(xch[i]) + dy * sin(xch[i]));
		double across = fabs(-dx * sin(xch[i]) + dy * cos(xch[i]));
		if (across < 300 && along < d) d = along;
	}
	double v = 220 + 3800.0 / (1.0 + exp(-(LINE_W / 2 - d) / BLUR));
	v += (rand() % 61) - 30;
	if (v < 0) v = 0;
	if (v > 4095) v = 4095;
	return (int)v;
}

typedef struct {
	double fwd, half;	/* sensor position: ahead of axle, half spacing (mm) */
	double mis;		/* right/left real motor strength ratio */
	double tau;		/* motor lag (s) */
	double off_x, off_h;	/* start offset (mm, deg) */
} CAR;

/* copy of motion_drive() + the ramp + open-loop duty model */
static void wheels(int speed, int steer, int *l, int *r)
{
	int s = abs(steer), inner;
	if (steer > 1000) steer = 1000;
	inner = speed - (speed * 2 * s) / 1000;
	if (inner < -speed / 2) inner = -speed / 2;
	if (steer >= 0) { *l = speed; *r = inner; } else { *l = inner; *r = speed; }
}

static int run(const double *seg, int nseg, const CAR *car, const line_tune_t *k,
	       int verbose, double *max_err, double *t_end)
{
	build(seg, nseg);
	double hd = M_PI / 2 + car->off_h * M_PI / 180;
	double x = -car->off_x + EDGE_X, y = 0;	/* axle centre */
	double vl = 0, vr = 0;			/* real wheel speeds */
	double cl = 0, cr = 0;			/* ramped set-points */
	int tl = 0, tr = 0;
	int hint = 0;
	line_ctl_t c;
	line_out_t o;
	line_ctl_reset(&c, k, 0, 0, 0);
	*max_err = 0;
	for (int ms = 0; ms < 400000; ms++) {
		if (ms % 20 == 0) {
			double sx = x + car->fwd * cos(hd), sy = y + car->fwd * sin(hd);
			double lx = sx - car->half * sin(hd) * -1, ly = sy - car->half * cos(hd);
			/* left of heading h is (cos(h+90), sin(h+90)) = (-sin h, cos h) */
			lx = sx - car->half * sin(hd);  ly = sy + car->half * cos(hd);
			double rx = sx + car->half * sin(hd), ry = sy - car->half * cos(hd);
			int dl = sense(lx, ly, hint), dr = sense(rx, ry, hint);
			line_ctl_step(&c, dl, dr, (uint32_t)ms, &o);
			if (o.lost) { if (verbose) printf("  LOST at %d ms idx %d/%d\n", ms, hint, np); *t_end = ms / 1000.0; return 0; }
			{ static int ql[8], qr[8], qi; int nl, nr; wheels(o.speed, o.steer, &nl, &nr);
			  if (ms == 0) { for (int q = 0; q < 8; q++) ql[q] = qr[q] = 0; qi = 0; }
			  ql[qi] = nl; qr[qi] = nr; qi = (qi + 1) % 8;
			  int j = (qi - 1 - LAT_TICKS + 16) % 8; tl = ql[j]; tr = qr[j]; }
			if (verbose > 1 && ms % 20 == 0)
				printf("%6d idx %5d dl %4d dr %4d pos %5d steer %5d mode %d sp %d err %.1f\n",
				       ms, hint, dl, dr, o.pos, o.steer, o.mode, o.speed, *max_err);
		}
		/* ramp 800 mm/s^2 */
		double st = 0.8;
		cl += (tl > cl) ? fmin(st, tl - cl) : -fmin(st, cl - tl);
		cr += (tr > cr) ? fmin(st, tr - cr) : -fmin(st, cr - tr);
		/* duty deadband: below ~20 mm/s the wheel does not turn */
		double wl = fabs(cl) < DEADBAND ? 0 : cl, wr = fabs(cr) < DEADBAND ? 0 : cr * car->mis;
		vl += (wl - vl) * (0.001 / TAU);
		vr += (wr - vr) * (0.001 / TAU);
		double v = (vl + vr) / 2, w = (vr - vl) / 120.0;
		x += v * 0.001 * cos(hd); y += v * 0.001 * sin(hd); hd += w * 0.001;
		double d;
		double sx = x + car->fwd * cos(hd), sy = y + car->fwd * sin(hd);
		hint = near_idx(sx, sy, hint, &d);
		if (d > *max_err) *max_err = d;
		if (d > 80) { if (verbose) printf("  OFF TRACK at %d ms idx %d/%d\n", ms, hint, np); *t_end = ms / 1000.0; return 0; }
		if (hint >= np - 5) { *t_end = ms / 1000.0; return 1; }
	}
	*t_end = 400;
	return 0;
}

/* Test tracks */
static const double T_OVAL[] = { 0,600, 300,180, 0,600, 300,180 };
static const double T_MIX[]  = { 0,500, 250,90, 0,300, 200,-180, 0,300, 150,90, 0,400 };
static const double T_TIGHT[] = { 0,400, 120,90, 0,300, 120,-90, 0,300, 100,90, 0,300 };
static const double T_VTIGHT[] = { 0,300, 80,90, 0,250, 60,-90, 0,250, 80,180, 0,300 };
static const double T_S[]    = { 0,300, 180,60, 180,-120, 180,120, 180,-60, 0,300 };
static const double T_STRAIGHT[] = { 0,3000 };
/* sharp corners: tighter than the car can follow by steering alone */
static const double T_SHARP[] = { 0,400, 40,90, 0,300, 30,-90, 0,300, 40,90, 0,300 };
static const double T_CORNER[] = { 0,400, 20,90, 0,300, 15,-90, 0,300, 20,90, 0,300 };

/* cross junctions on a straight and in the middle of an S */
static const double T_CROSS[] = { 0,500, -1,0, 0,500, -1,0, 0,500, 150,90, 0,300, -1,0, 0,300 };
struct track { const char *name; const double *s; int n; };
static const struct track tracks[] = {
	{"straight 3 m", T_STRAIGHT, 1},
	{"oval R300", T_OVAL, 4},
	{"mix R250/200/150", T_MIX, 7},
	{"S-bends R180", T_S, 6},
	{"tight R120/100", T_TIGHT, 7},
	{"very tight R80/60", T_VTIGHT, 7},
	{"sharp R40/30", T_SHARP, 7},
	{"corners R20/15", T_CORNER, 7},
	{"cross junctions x3", T_CROSS, 10},
};

static int suite(const line_tune_t *k, int verbose, int print)
{
	static const double fwd[] = {50, 80};
	static const double half_out[] = {12, 20, 30};		/* gap wider than the line */
	static const double half_in[] = {4, 6.5, 9};		/* gap narrower than a 25 mm line */
	const double *half = getenv("INSIDE_GEOM") ? half_in : half_out;
	static const double mis[] = {0.9, 1.0, 1.1};
	int pass = 0, total = 0;
	for (unsigned t = 0; t < sizeof tracks / sizeof tracks[0]; t++) {
		int tp = 0, tt = 0;
		double worst = 0;
		for (unsigned a = 0; a < 2; a++)
		for (unsigned b = 0; b < 3; b++)
		for (unsigned m = 0; m < 3; m++)
		for (int rep = 0; rep < 2; rep++) {
			CAR car = { fwd[a], half[b], mis[m], 0.08, rep ? 8 : 0, rep ? 6 : 0 };
			double err, te;
			srand(1234 + rep);
			int vb = verbose;
			{ const char *tr = getenv("TRACE"); int tt2, aa, bb, mm, rr;
			  if (tr && sscanf(tr, "%d %d %d %d %d", &tt2, &aa, &bb, &mm, &rr) == 5 &&
			      tt2 == (int)t && aa == (int)a && bb == (int)b && mm == (int)m && rr == rep) vb = 2; }
			int ok = run(tracks[t].s, tracks[t].n, &car, k, vb, &err, &te);
			tp += ok; tt++;
			if (ok && err > worst) worst = err;
			if (!ok && print > 1)
				printf("    fail: %s fwd %.0f half %.0f mis %.1f rep %d\n", tracks[t].name, fwd[a], half[b], mis[m], rep);
		}
		if (print) printf("  %-18s %2d/%2d pass, worst error when passing %.0f mm\n", tracks[t].name, tp, tt, worst);
		pass += tp; total += tt;
	}
	if (print) printf("  TOTAL %d/%d\n", pass, total);
	return pass;
}

int main(int argc, char **argv)
{
	line_tune_t k = getenv("INSIDE_TUNE") ? g_line_tune_inside : g_line_tune_default;	/* INSIDE_TUNE=1: firmware's inside tuning */
	if (getenv("EDGE")) {			/* EDGE=1: line under the left sensor, -1 right */
		k = g_line_tune_edge;
		k.edge_dir = atoi(getenv("EDGE")) >= 0 ? 1 : -1;
	}
	int print = (argc > 1) ? atoi(argv[1]) : 1;

	if (argc > 2) { DEADBAND = atof(argv[2]); TAU = atof(argv[3]); BLUR = atof(argv[4]); LAT_TICKS = atoi(argv[5]); }
	if (argc > 6) { k.speed = atoi(argv[6]); }
	{ const char *e;
	  if ((e = getenv("KP"))) k.kp_x1000 = atoi(e);
	  if ((e = getenv("KD"))) k.kd_x1000 = atoi(e);
	  if ((e = getenv("DEEP"))) k.deep_pos = atoi(e);
	  if ((e = getenv("HDEEP"))) k.hold_deep = atoi(e);
	  if ((e = getenv("HMAX"))) k.hold_max = atoi(e);
	  if ((e = getenv("SMIN"))) k.speed_min = atoi(e);
	  if ((e = getenv("SEE"))) k.see = atoi(e);
	  if ((e = getenv("HSMALL"))) k.hold_small = atoi(e);
	  if ((e = getenv("CALM"))) k.calm_ms = atoi(e);
	  if ((e = getenv("LINEW"))) LINE_W = atof(e);
	  if (getenv("EDGE")) EDGE_X = (k.edge_dir > 0 ? 1 : -1) * LINE_W / 2;	/* right of the line if it is under the left sensor */
	  if ((e = getenv("INSIDE"))) k.inside = atoi(e);
	  if ((e = getenv("FALL"))) k.fall_pos = atoi(e);
	  if ((e = getenv("SPEED"))) k.speed = atoi(e);
	  if ((e = getenv("SLOWST"))) k.slow_steer = atoi(e);
	  if ((e = getenv("SLOWHOLD"))) k.slow_hold_ms = atoi(e);
	  if ((e = getenv("PIVOT"))) k.pivot_steer = atoi(e);		/* 0 = no stop + pivot */
	  if ((e = getenv("PSTOP"))) k.pivot_stop_ms = atoi(e);
	  if ((e = getenv("PEXIT"))) k.pivot_exit_pos = atoi(e);
	  if ((e = getenv("ONLYNEW"))) print = -print; }
	printf("deadband %.0f mm/s, tau %.2f s, blur %.1f mm, latency %d ticks, speed %d\n", DEADBAND, TAU, BLUR, LAT_TICKS, k.speed);
	printf("Firmware defaults:\n");
	suite(&k, 0, print < 0 ? -print : print);

	if (print < 0) return 0;
	/* v47-like reference: P + KD only, small hold, no deep logic */
	line_tune_t v47 = g_line_tune_default; v47.speed = k.speed;
	v47.deep_pos = 5000;		/* never deep */
	v47.hold_small = 280; v47.wobble_ms = 2000;
	v47.speed_min = v47.speed;	/* no slow-down */
	printf("\nReference (v47-style: P + fixed 28%% hold for 2 s, no slow-down):\n");
	suite(&v47, 0, print);
	return 0;
}
