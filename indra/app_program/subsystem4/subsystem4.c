/*
 *	subsystem4.c  (Buddy 4: IMU, terrain, hump / impact / tilt)
 *
 *	GY-511 = LSM303DLHC (or the newer LSM303AGR): accelerometer at I2C
 *	0x19, magnetometer at 0x1E.  It has NO gyro.  Some boards sold as
 *	GY-511 carry another chip (IS27's answers at 0x1D only), so the
 *	setup also finds an LSM303D, ADXL345 or MMA845x (imu_setup below).
 *
 *	imu task (priority 14, core 1), every 20 ms (50 Hz):
 *	  1. read the accelerometer (+-4 g, 2 mg per count), every 5th
 *	     sample the magnetometer
 *	  2. pitch / roll from gravity, smoothed (1/4 new)
 *	  3. while the car is stopped (mission IDLE) learn "level", so the
 *	     sensor doesn't have to be mounted perfectly flat
 *	  4. hump: pitch up, then down, then level; height = sum of
 *	     sin(pitch) x distance (odometry from Buddy 2)
 *	  5. impact: |a| more than IMU_IMPACT_MG away from 1 g -> FAULT_IMPACT
 *	  6. tilt: pitch or roll over IMU_TILT_D10 for 300 ms -> EV_TILT
 *	     (the mission stops the car: it has tipped or been picked up)
 *	A sensor that stops answering raises FAULT_IMU and is re-initialised
 *	every second; everything else keeps running without it.
 *
 *	INT1 (GP3) is not needed: the task polls at a fixed 50 Hz.
 */
#include "../car.h"
#include "drv_i2c.h"
#include "imu_math.h"

/* ---- mounting: which board axis points forward / left / up (CAL) ----
   0 = X, 1 = Y, 2 = Z of the sensor board; sign +1 or -1.
   Check: tilt the nose up -> pitch goes +; lift the left side -> roll goes +. */
#define IMU_FWD_AXIS		0
#define IMU_FWD_SIGN		(+1)
#define IMU_LEFT_AXIS		1
#define IMU_LEFT_SIGN		(+1)
#define IMU_UP_AXIS		2
#define IMU_UP_SIGN		(+1)	/* flipped at run time if it reads -1 g (upside down) */
#define IMU_SETTLE_SAMPLES	25	/* 0.5 s at 50 Hz before orientation / level are decided */
LOCAL INT up_sign = IMU_UP_SIGN, left_sign = IMU_LEFT_SIGN;

#define IMU_IMPACT_MG		1500	/* |a| - 1 g above this = impact */
#define IMU_TILT_D10		350	/* 35 deg */
#define IMU_TILT_MS		300
/* picked up (12 Nov evening): lifting the car pushes the accelerometer
   above 1 g for a while.  IMU_LIFT_HITS of the last IMU_LIFT_WIN raw
   samples (20 ms each) more than IMU_LIFT_MG above 1 g -> EV_LIFT (the
   mission safe-stops while driving).  A bump is a 1-sample spike, so it
   does not count.  CAL: the console prints "IMU lift" with the value. */
#define IMU_LIFT_MG		250
#define IMU_LIFT_HITS		3
#define IMU_LIFT_WIN		4
#define IMU_EVENT_SHOW_MS	1000	/* event bits stay set this long (telemetry) */

static const HUMP_TUNE hump_tune = {
	35,	/* climb above 3.5 deg (a 2 cm rise over a 30 cm ramp is 3.8 deg) */
	20,	/* level below 2 deg */
	100,	/* confirm ms */
	200,	/* level ms */
	60000,	/* give up ms (the distance limit below normally ends it first) */
	5,	/* min height mm */
	1000	/* give up after 1 m of travel without finishing */
};

#define ACC_ADDR		0x19	/* SA0 high: most GY-511 boards */
#define ACC_ADDR_ALT		0x18	/* SA0 low: some clones */
#define MAG_ADDR		0x1E
/* accelerometer (same map on DLHC and AGR) */
#define CTRL_REG1_A		0x20
#define CTRL_REG4_A		0x23
#define OUT_X_L_A		0x28
#define AUTO_INC		0x80
/* magnetometer, DLHC */
#define CRA_REG_M		0x00
#define CRB_REG_M		0x01
#define MR_REG_M		0x02
#define OUT_X_H_M		0x03
#define IRA_REG_M		0x0A
/* magnetometer, AGR */
#define WHO_AM_I_M		0x4F
#define CFG_REG_A_M		0x60
#define CFG_REG_C_M		0x62
#define OUTX_L_REG_M		0x68

LOCAL IMU_STATUS st;		/* written by the imu task only */
LOCAL INT mag_type;		/* 0 none, 1 DLHC, 2 AGR, 3 LSM303D */
LOCAL UB  acc_addr = ACC_ADDR;
LOCAL UINT setup_tries;

/* Nothing answered: list every address that does, so the console shows
   whether it is wiring (nothing at all) or a different chip. */
LOCAL void i2c_scan_report(void)
{
	UB a, b;
	INT found = 0;

	tm_printf((UB*)"IMU: no known accelerometer found (SDA GP%d, SCL GP%d). I2C scan:",
		  PIN_IMU_SDA, PIN_IMU_SCL);
	for (a = 0x08; a < 0x78; a++) {
		if (i2c_read_regs(a, 0x0F, &b, 1) == E_OK) { tm_printf((UB*)" 0x%02x", a); found++; }
	}
	if (found == 0)
		tm_printf((UB*)" nothing.\n    -> wiring: check SDA/SCL not swapped, VIN and GND connected, the Grove 3 port\n");
	else
		tm_printf((UB*)"\n    -> something answers, but not a chip this firmware knows (see the next line)\n");
}

EXPORT void imu_get(IMU_STATUS *s) { *s = st; }

/* ---- which accelerometer chip is fitted ----
   Boards sold as "GY-511" are not all the same chip, so look for each:
     DLHC/AGR  LSM303DLHC or LSM303AGR: accel 0x19 (or 0x18), mag 0x1E
     LSM303D   accel + mag in one at 0x1D (or 0x1E), WHO_AM_I 0x0F = 0x49
     ADXL345   accel only, 0x1D (or 0x53), DEVID 0x00 = 0xE5
     MMA845x   accel only, 0x1D (or 0x1C), WHO_AM_I 0x0D = 0x1A/0x2A/0x3A */
enum { CHIP_NONE, CHIP_DLHC, CHIP_LSM303D, CHIP_ADXL345, CHIP_MMA845X };
LOCAL INT chip = CHIP_NONE;
LOCAL UB  mag_addr = MAG_ADDR;

/* LSM303D registers */
#define D_WHO_AM_I	0x0F
#define D_CTRL1		0x20
#define D_CTRL2		0x21
#define D_CTRL5		0x24
#define D_CTRL6		0x25
#define D_CTRL7		0x26
#define D_OUT_X_L_M	0x08
#define D_OUT_X_L_A	0x28
/* ADXL345 */
#define X_DEVID		0x00
#define X_BW_RATE	0x2C
#define X_POWER_CTL	0x2D
#define X_DATA_FORMAT	0x31
#define X_DATAX0	0x32
/* MMA845x */
#define M_OUT_X_MSB	0x01
#define M_WHO_AM_I	0x0D
#define M_XYZ_CFG	0x0E
#define M_CTRL1		0x2A

LOCAL BOOL rd1(UB addr, UB reg, UB *v) { return i2c_read_regs(addr, reg, v, 1) == E_OK; }

LOCAL ER setup_dlhc(void)
{
	UB id;
	ER er;
	/* accel: 100 Hz, X/Y/Z on; block update, +-4 g, high resolution */
	acc_addr = ACC_ADDR;
	if (i2c_write_reg(acc_addr, CTRL_REG1_A, 0x57) < E_OK) {
		acc_addr = ACC_ADDR_ALT;
		if ((er = i2c_write_reg(acc_addr, CTRL_REG1_A, 0x57)) < E_OK) return er;
	}
	if ((er = i2c_write_reg(acc_addr, CTRL_REG4_A, 0x98)) < E_OK) return er;
	mag_type = 0;
	mag_addr = MAG_ADDR;
	if (rd1(MAG_ADDR, IRA_REG_M, &id) && id == 0x48) {
		mag_type = 1;					/* DLHC: 'H' */
		(void)i2c_write_reg(MAG_ADDR, CRA_REG_M, 0x10);	/* 15 Hz */
		(void)i2c_write_reg(MAG_ADDR, CRB_REG_M, 0x20);	/* +-1.3 gauss */
		(void)i2c_write_reg(MAG_ADDR, MR_REG_M, 0x00);	/* continuous */
	} else if (rd1(MAG_ADDR, WHO_AM_I_M, &id) && id == 0x40) {
		mag_type = 2;					/* AGR */
		(void)i2c_write_reg(MAG_ADDR, CFG_REG_A_M, 0x80);	/* temp comp, 10 Hz, continuous */
		(void)i2c_write_reg(MAG_ADDR, CFG_REG_C_M, 0x10);	/* block update */
	}
	chip = CHIP_DLHC;
	tm_printf((UB*)"IMU found: LSM303DLHC/AGR accelerometer at 0x%02x, magnetometer: %s\n",
		  acc_addr, mag_type == 1 ? "LSM303DLHC" : mag_type == 2 ? "LSM303AGR" : "not found");
	return E_OK;
}

LOCAL ER setup_lsm303d(UB addr)
{
	ER er;
	acc_addr = mag_addr = addr;
	if ((er = i2c_write_reg(addr, D_CTRL1, 0x6F)) < E_OK) return er;	/* 100 Hz, block update, XYZ */
	if ((er = i2c_write_reg(addr, D_CTRL2, 0x08)) < E_OK) return er;	/* +-4 g */
	(void)i2c_write_reg(addr, D_CTRL5, 0x64);	/* mag high resolution, 6.25 Hz */
	(void)i2c_write_reg(addr, D_CTRL6, 0x20);	/* mag +-4 gauss */
	(void)i2c_write_reg(addr, D_CTRL7, 0x00);	/* mag continuous */
	chip = CHIP_LSM303D;
	mag_type = 3;
	tm_printf((UB*)"IMU found: LSM303D (accelerometer + magnetometer) at 0x%02x\n", addr);
	return E_OK;
}

LOCAL ER setup_adxl345(UB addr)
{
	ER er;
	acc_addr = addr;
	if ((er = i2c_write_reg(addr, X_BW_RATE, 0x0A)) < E_OK) return er;	/* 100 Hz */
	if ((er = i2c_write_reg(addr, X_DATA_FORMAT, 0x09)) < E_OK) return er;	/* full res, +-4 g */
	if ((er = i2c_write_reg(addr, X_POWER_CTL, 0x08)) < E_OK) return er;	/* measure */
	chip = CHIP_ADXL345;
	mag_type = 0;
	tm_printf((UB*)"IMU found: ADXL345 accelerometer at 0x%02x (no compass)\n", addr);
	return E_OK;
}

LOCAL ER setup_mma845x(UB addr, UB who)
{
	ER er;
	acc_addr = addr;
	if ((er = i2c_write_reg(addr, M_CTRL1, 0x00)) < E_OK) return er;	/* standby to configure */
	if ((er = i2c_write_reg(addr, M_XYZ_CFG, 0x01)) < E_OK) return er;	/* +-4 g */
	if ((er = i2c_write_reg(addr, M_CTRL1, 0x19)) < E_OK) return er;	/* 100 Hz, active */
	chip = CHIP_MMA845X;
	mag_type = 0;
	tm_printf((UB*)"IMU found: MMA845x accelerometer (id 0x%02x) at 0x%02x (no compass)\n", who, addr);
	return E_OK;
}

LOCAL ER imu_setup(void)
{
	static const UB alt[] = { 0x1D, 0x1E, 0x53, 0x1C };
	UB v;
	UINT i;

	i2c_init();
	chip = CHIP_NONE;
	mag_type = 0;
	if (setup_dlhc() == E_OK) return E_OK;
	for (i = 0; i < sizeof alt; i++) {
		UB a = alt[i];
		if ((a == 0x1D || a == 0x1E) && rd1(a, D_WHO_AM_I, &v) && v == 0x49) return setup_lsm303d(a);
		if ((a == 0x1D || a == 0x53) && rd1(a, X_DEVID, &v) && v == 0xE5) return setup_adxl345(a);
		if ((a == 0x1D || a == 0x1C) && rd1(a, M_WHO_AM_I, &v) &&
		    (v == 0x1A || v == 0x2A || v == 0x3A || v == 0x4A || v == 0x5A)) return setup_mma845x(a, v);
	}
	if ((setup_tries++ % 10U) == 0) {			/* every ~10 s */
		i2c_scan_report();
		/* say what an unknown chip answers, so it can be added */
		for (i = 0; i < sizeof alt; i++) {
			UB a = alt[i], w0f = 0, w00 = 0, w0d = 0;
			BOOL ok = rd1(a, 0x0F, &w0f);
			if (!ok) continue;
			(void)rd1(a, 0x00, &w00);
			(void)rd1(a, 0x0D, &w0d);
			tm_printf((UB*)"    chip at 0x%02x: reg 0x0F = 0x%02x, reg 0x00 = 0x%02x, reg 0x0D = 0x%02x"
				  " (send this line to identify it)\n", a, w0f, w00, w0d);
		}
	}
	return E_NOEXS;
}

LOCAL ER read_accel(INT a[3])
{
	UB b[6];
	INT i;
	ER er;

	switch (chip) {
	case CHIP_DLHC:		/* 12 bit left-justified, +-4 g high res: 2 mg per count */
		if ((er = i2c_read_regs(acc_addr, OUT_X_L_A | AUTO_INC, b, 6)) < E_OK) return er;
		for (i = 0; i < 3; i++) a[i] = ((INT)(H)(b[2 * i] | (b[2 * i + 1] << 8)) >> 4) * 2;
		break;
	case CHIP_LSM303D:	/* 16 bit, +-4 g: 0.122 mg per count */
		if ((er = i2c_read_regs(acc_addr, D_OUT_X_L_A | AUTO_INC, b, 6)) < E_OK) return er;
		for (i = 0; i < 3; i++) a[i] = ((INT)(H)(b[2 * i] | (b[2 * i + 1] << 8)) * 122) / 1000;
		break;
	case CHIP_ADXL345:	/* full resolution: 3.9 mg per count, low byte first */
		if ((er = i2c_read_regs(acc_addr, X_DATAX0, b, 6)) < E_OK) return er;
		for (i = 0; i < 3; i++) a[i] = ((INT)(H)(b[2 * i] | (b[2 * i + 1] << 8)) * 39) / 10;
		break;
	case CHIP_MMA845X:	/* 12 bit left-justified, high byte first, +-4 g: 512 per g */
		if ((er = i2c_read_regs(acc_addr, M_OUT_X_MSB, b, 6)) < E_OK) return er;
		for (i = 0; i < 3; i++) a[i] = (((INT)(H)((b[2 * i] << 8) | b[2 * i + 1]) >> 4) * 1000) / 512;
		break;
	default:
		return E_NOEXS;
	}
	return E_OK;
}

LOCAL ER read_mag(INT m[3])
{
	UB b[6];
	ER er;

	if (mag_type == 1) {				/* X, Z, Y, high byte first */
		if ((er = i2c_read_regs(MAG_ADDR, OUT_X_H_M, b, 6)) < E_OK) return er;
		m[0] = (H)((b[0] << 8) | b[1]);
		m[2] = (H)((b[2] << 8) | b[3]);
		m[1] = (H)((b[4] << 8) | b[5]);
	} else if (mag_type == 2) {			/* X, Y, Z, low byte first */
		if ((er = i2c_read_regs(MAG_ADDR, OUTX_L_REG_M | AUTO_INC, b, 6)) < E_OK) return er;
		m[0] = (H)(b[0] | (b[1] << 8));
		m[1] = (H)(b[2] | (b[3] << 8));
		m[2] = (H)(b[4] | (b[5] << 8));
	} else if (mag_type == 3) {			/* LSM303D: X, Y, Z, low byte first */
		if ((er = i2c_read_regs(mag_addr, D_OUT_X_L_M | AUTO_INC, b, 6)) < E_OK) return er;
		m[0] = (H)(b[0] | (b[1] << 8));
		m[1] = (H)(b[2] | (b[3] << 8));
		m[2] = (H)(b[4] | (b[5] << 8));
	} else return E_NOEXS;
	return E_OK;
}

LOCAL void imu_task(INT stacd, void *exinf)
{
	HUMP hump;
	INT f[3] = {0, 0, 1000};		/* filtered accel, mg */
	INT lvl_p = 0, lvl_r = 0;		/* learned level, 0.1 deg */
	INT fails = 0, n = 0;
	UW tilt_t0 = 0, impact_t = 0, ev_t[3] = {0, 0, 0};
	BOOL tilted = FALSE, have_level = FALSE;
	UINT lift_hist = 0;
	UW lift_t = 0;
	W odo_last = 0;

	(void)stacd; (void)exinf;
	hump_reset(&hump);
	st.heading = -1;
	while (imu_setup() < E_OK) {
		fault_raise(FAULT_IMU);
		tk_dly_tsk(1000);
	}
	fault_clear(FAULT_IMU);
	st.ok = TRUE;

	for (;; n++) {
		INT a[3], fwd, left, up, pitch, roll, mag_g;
		UW t;
		UINT i;

		tk_dly_tsk(IMU_PERIOD_MS);
		t = now_ms();

		if (read_accel(a) < E_OK) {
			if (++fails >= 5) {			/* ~100 ms of silence */
				st.ok = FALSE;
				fault_raise(FAULT_IMU);
				tk_dly_tsk(1000);
				if (imu_setup() == E_OK) { fails = 0; st.ok = TRUE; fault_clear(FAULT_IMU); }
			}
			continue;
		}
		fails = 0;

		/* impact: on the raw sample, before smoothing */
		mag_g = (INT)imu_isqrt((UW)(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));
		st.shock_mg = ABS(mag_g - 1000);
		if (st.shock_mg > IMU_IMPACT_MG && (t - impact_t) > 500U) {
			impact_t = t;
			ev_t[1] = t;
			st.events |= IMU_EV_IMPACT;
			st.seq++;
			fault_raise(FAULT_IMPACT);
			tm_printf((UB*)"IMU impact %d mg\n", st.shock_mg);
		}
		if (impact_t && (t - impact_t) > 3000U) { fault_clear(FAULT_IMPACT); impact_t = 0; }

		/* picked up: sustained push above 1 g */
		{
			UINT k, h;
			lift_hist = ((lift_hist << 1) | (mag_g - 1000 > IMU_LIFT_MG ? 1U : 0U)) &
				    ((1U << IMU_LIFT_WIN) - 1U);
			for (h = lift_hist, k = 0; h; h >>= 1) k += h & 1U;
			if (k >= IMU_LIFT_HITS && n > IMU_SETTLE_SAMPLES && (t - lift_t) > 1000U) {
				lift_t = t;
				mission_event(EV_LIFT);
				tm_printf((UB*)"IMU lift: %d mg above 1 g on %d of %d samples\n",
					  mag_g - 1000, (INT)k, IMU_LIFT_WIN);
			}
		}

		for (i = 0; i < 3; i++) f[i] += (a[i] - f[i]) / 4;	/* smoothing */
		st.ax = f[0]; st.ay = f[1]; st.az = f[2];
		/* mounted upside down?  (IS27: the board reads -1 g on Z.)
		   Decided once, after 0.5 s of smoothing: gravity pointing
		   "down" on the up axis = flip up AND left (a 180 deg turn
		   about the forward axis), so roll reads 0, not 180 deg. */
		if (n == IMU_SETTLE_SAMPLES && up_sign * f[IMU_UP_AXIS] < -500) {
			up_sign = -up_sign;
			left_sign = -left_sign;
			tm_printf((UB*)"IMU is mounted upside down (up axis %d mg): flipping up and left axes\n",
				  f[IMU_UP_AXIS]);
		}
		fwd  = IMU_FWD_SIGN * f[IMU_FWD_AXIS];
		left = left_sign    * f[IMU_LEFT_AXIS];
		up   = up_sign      * f[IMU_UP_AXIS];
		pitch = imu_atan2_d10(fwd, (INT)imu_isqrt((UW)(left * left + up * up)));
		roll  = imu_atan2_d10(left, up);

		/* level: learnt while parked, steady and NOT being moved, and
		   only small corrections (under 3 deg).  So a car pushed by
		   hand over a hump, or set down on a ramp, still reads the
		   slope instead of "learning" it away within a second. */
		{
			W odo = motion_odometry_mm();
			BOOL still = (odo == odo_last);
			odo_last = odo;
			if (!have_level) {
				/* first level: after the smoothing has settled and
				   the orientation is known (not from sample 1, which
				   started from a made-up 0, 0, 1 g) */
				if (n < IMU_SETTLE_SAMPLES + 5) continue;
				lvl_p = pitch; lvl_r = roll; have_level = TRUE;
				tm_printf((UB*)"IMU level learnt: pitch %d roll %d (0.1 deg) = 0\n", pitch, roll);
			} else if (mission_state() == MS_IDLE && st.shock_mg < 60 && still &&
				 ABS(pitch - lvl_p) < 30 && ABS(roll - lvl_r) < 30) {
				lvl_p += (pitch - lvl_p) / 16;
				lvl_r += (roll - lvl_r) / 16;
			}
		}
		st.pitch_d10 = pitch - lvl_p;
		st.roll_d10  = roll - lvl_r;

		/* hump */
		{
			int32_t h_mm, r;
			r = hump_step(&hump, &hump_tune, st.pitch_d10, (INT)motion_odometry_mm(), t, &h_mm);
			if (r) {
				st.humps++;
				st.hump_mm = (INT)h_mm;
				st.events |= IMU_EV_HUMP;
				ev_t[0] = t;
				st.seq++;
				tm_printf((UB*)"IMU %s #%d, %d mm %s\n", r == 1 ? "hump / ramp up" : "ramp down",
					  (INT)st.humps, (INT)h_mm, r == 1 ? "high" : "drop");
			}
		}

		/* tilt: tipped over or picked up */
		if (ABS(st.pitch_d10) > IMU_TILT_D10 || ABS(st.roll_d10) > IMU_TILT_D10) {
			if (tilt_t0 == 0) tilt_t0 = t;
			if (!tilted && (t - tilt_t0) > IMU_TILT_MS) {
				tilted = TRUE;
				st.events |= IMU_EV_TILT;
				ev_t[2] = t;
				st.seq++;
				mission_event(EV_TILT);
				tm_printf((UB*)"IMU tilt: pitch %d roll %d (0.1 deg)\n", st.pitch_d10, st.roll_d10);
			}
		} else {
			tilt_t0 = 0;
			tilted = FALSE;
		}

		/* event bits fade after a second (they ride on robot/tlm/terrain) */
		for (i = 0; i < 3; i++)
			if ((st.events & (1U << i)) && (t - ev_t[i]) > IMU_EVENT_SHOW_MS) st.events &= ~(1U << i);

		/* compass, 10 Hz (uncalibrated: close to the motors it is rough) */
		if ((n % 5) == 0 && mag_type) {
			INT m[3];
			if (read_mag(m) == E_OK) {
				INT mf = IMU_FWD_SIGN * m[IMU_FWD_AXIS], ml = left_sign * m[IMU_LEFT_AXIS];
				INT hd = imu_atan2_d10(-ml, mf) / 10;	/* clockwise from north */
				st.heading = hd < 0 ? hd + 360 : hd;
			}
		}
	}
}

EXPORT ER subsystem4_init(void) { return E_OK; }	/* no kernel objects */

EXPORT ER subsystem4_start(void)
{
	ID tid = app_start_task((FP)imu_task, PRI_IMU, CORE_IMU, "imu");
	return tid < E_OK ? tid : E_OK;
}

#if APP_TEST
EXPORT void subsystem4_test(void) { }
#endif
