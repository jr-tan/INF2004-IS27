/**
 * @file    subsystem4.c
 *  subsystem4.c  (Buddy 4: IMU, terrain, hump / impact / tilt)
 *
 *  GY-511 = LSM303DLHC (or the newer LSM303AGR): accelerometer at I2C
 *  0x19, magnetometer at 0x1E.  It has NO gyro.  Some boards sold as
 *  GY-511 carry another chip (IS27's answers at 0x1D only), so the
 *  setup also finds an LSM303D, ADXL345 or MMA845x (imu_setup below).
 *
 *  imu task (priority 14, core 1), every 20 ms (50 Hz):
 *    1. read the accelerometer (+-4 g, 2 mg per count), every 5th
 *       sample the magnetometer
 *    2. pitch / roll from gravity, smoothed (1/4 new)
 *    3. while the car is stopped (mission IDLE) learn "level", so the
 *       sensor doesn't have to be mounted perfectly flat
 *    4. hump: pitch up, then down, then level; height = sum of
 *       sin(pitch) x distance (odometry from Buddy 2)
 *    5. impact: |a| more than IMU_IMPACT_MG away from 1 g -> FAULT_IMPACT
 *    6. tilt: pitch or roll over IMU_TILT_D10 for 300 ms -> EV_TILT
 *       (the mission stops the car: it has tipped or been picked up)
 *  A sensor that stops answering raises FAULT_IMU and is re-initialised
 *  every second; everything else keeps running without it.
 *
 *  INT1 (GP3) is not needed: the task polls at a fixed 50 Hz.
 */
#include "../car.h"
#include "drv_i2c.h"
#include "imu_math.h"
#include "subsystem4.h" /* own header (BARR-C 4.3.c) */

/* ---- mounting: which board axis points forward / left / up (CAL) ----
   0 = X, 1 = Y, 2 = Z of the sensor board; sign +1 or -1.
   Check: tilt the nose up -> pitch goes +; lift the left side -> roll goes +.
 */
#define IMU_FWD_AXIS  0
#define IMU_FWD_SIGN  (+1)
#define IMU_LEFT_AXIS 1
#define IMU_LEFT_SIGN (+1)
#define IMU_UP_AXIS   2
#define IMU_UP_SIGN \
    (+1) /* flipped at run time if it reads -1 g (upside down) */
#define IMU_SETTLE_SAMPLES \
    25 /* 0.5 s at 50 Hz before orientation / level are decided */
LOCAL int32_t g_up_sign   = IMU_UP_SIGN;
LOCAL int32_t g_left_sign = IMU_LEFT_SIGN;

#define IMU_IMPACT_MG 1500 /* |a| - 1 g above this = impact */
#define IMU_TILT_D10  350  /* 35 deg */
#define IMU_TILT_MS   300
/* picked up (12 Nov evening): lifting the car pushes the accelerometer
   above 1 g for a while.  IMU_LIFT_HITS of the last IMU_LIFT_WIN raw
   samples (20 ms each) more than IMU_LIFT_MG above 1 g -> EV_LIFT (the
   mission safe-stops while driving).  A bump is a 1-sample spike, so it
   does not count.  CAL: the console prints "IMU lift" with the value. */
#define IMU_LIFT_MG       250
#define IMU_LIFT_HITS     3
#define IMU_LIFT_WIN      4
#define IMU_EVENT_SHOW_MS 1000 /* event bits stay set this long (telemetry) */
/* Accelerometer axes x, y, z (loop bound, BARR-C 8.4.a). */
#define IMU_AXES 3
/* 7-bit I2C addresses probed by the bus scan: 0x08..0x77 (0x00-0x07 and
   0x78-0x7F are reserved by the I2C specification). */
#define I2C_SCAN_FIRST 0x08
#define I2C_SCAN_END   0x78

static const hump_tune_t g_hump_tune = {
    35,    /* climb above 3.5 deg (a 2 cm rise over a 30 cm ramp is 3.8 deg) */
    20,    /* level below 2 deg */
    100,   /* confirm ms */
    200,   /* level ms */
    60000, /* give up ms (the distance limit below normally ends it first) */
    5,     /* min height mm */
    1000   /* give up after 1 m of travel without finishing */
};

#define ACC_ADDR     0x19 /* SA0 high: most GY-511 boards */
#define ACC_ADDR_ALT 0x18 /* SA0 low: some clones */
#define MAG_ADDR     0x1E
/* accelerometer (same map on DLHC and AGR) */
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define OUT_X_L_A   0x28
#define AUTO_INC    0x80
/* magnetometer, DLHC */
#define CRA_REG_M 0x00
#define CRB_REG_M 0x01
#define MR_REG_M  0x02
#define OUT_X_H_M 0x03
#define IRA_REG_M 0x0A
/* magnetometer, AGR */
#define WHO_AM_I_M   0x4F
#define CFG_REG_A_M  0x60
#define CFG_REG_C_M  0x62
#define OUTX_L_REG_M 0x68

LOCAL imu_status_t g_stat_val; /* written by the imu task only */
LOCAL int32_t      g_mag_type; /* 0 none, 1 DLHC, 2 AGR, 3 LSM303D */
LOCAL uint8_t      g_acc_addr = ACC_ADDR;
LOCAL uint32_t     g_setup_tries;

/** Nothing answered: list every address that does, so the console shows
   whether it is wiring (nothing at all) or a different chip. */
LOCAL void i2c_scan_report (void)
{
    uint8_t aval;
    uint8_t bval;
    int32_t found = 0;

    tm_printf(
        (uint8_t *)"IMU: no known accelerometer found (SDA GP%d, SCL GP%d). "
                   "I2C scan:",
        PIN_IMU_SDA, PIN_IMU_SCL);
    for (aval = I2C_SCAN_FIRST; aval < I2C_SCAN_END; aval++)
    {
        if (E_OK == drv_i2c_read_regs(aval, 0x0F, &bval, 1))
        {
            tm_printf((uint8_t *)" 0x%02x", aval);
            found++;
        }
    }
    if (0 == found)
    {
        tm_printf(
            (uint8_t *)" nothing.\n    -> wiring: check SDA/SCL not swapped, "
                       "VIN and GND connected, the Grove 3 port\n");
    }
    else
    {
        tm_printf((uint8_t *)"\n    -> something answers, but not a chip this "
                             "firmware knows (see the next line)\n");
    }
}

/**
 * @brief  Imu get.
 */
EXPORT void imu_get (imu_status_t * p_str)
{
    *p_str = g_stat_val;
}

/* ---- which accelerometer chip is fitted ----
   Boards sold as "GY-511" are not all the same chip, so look for each:
     DLHC/AGR  LSM303DLHC or LSM303AGR: accel 0x19 (or 0x18), mag 0x1E
     LSM303D   accel + mag in one at 0x1D (or 0x1E), WHO_AM_I 0x0F = 0x49
     ADXL345   accel only, 0x1D (or 0x53), DEVID 0x00 = 0xE5
     MMA845x   accel only, 0x1D (or 0x1C), WHO_AM_I 0x0D = 0x1A/0x2A/0x3A */
enum
{
    CHIP_NONE,
    CHIP_DLHC,
    CHIP_LSM303D,
    CHIP_ADXL345,
    CHIP_MMA845X
};
LOCAL int32_t g_chip     = CHIP_NONE;
LOCAL uint8_t g_mag_addr = MAG_ADDR;

/* LSM303D registers */
#define D_WHO_AM_I  0x0F
#define D_CTRL1     0x20
#define D_CTRL2     0x21
#define D_CTRL5     0x24
#define D_CTRL6     0x25
#define D_CTRL7     0x26
#define D_OUT_X_L_M 0x08
#define D_OUT_X_L_A 0x28
/* ADXL345 */
#define X_DEVID       0x00
#define X_BW_RATE     0x2C
#define X_POWER_CTL   0x2D
#define X_DATA_FORMAT 0x31
#define X_DATAX0      0x32
/* MMA845x */
#define M_OUT_X_MSB 0x01
#define M_WHO_AM_I  0x0D
#define M_XYZ_CFG   0x0E
#define M_CTRL1     0x2A

/**
 * @brief  Rd1.
 * @return Result as described above.
 */
LOCAL BOOL rd1 (uint8_t addr, uint8_t reg, uint8_t * p_val)
{
    return E_OK == drv_i2c_read_regs(addr, reg, p_val, 1);
}

/**
 * @brief  Setup dlhc.
 * @return Result as described above.
 */
LOCAL ER setup_dlhc (void)
{
    uint8_t obj_id;
    ER      err;
    /* accel: 100 Hz, X/Y/Z on; block update, +-4 g, high resolution */
    g_acc_addr = ACC_ADDR;
    if (drv_i2c_write_reg(g_acc_addr, CTRL_REG1_A, 0x57) < E_OK)
    {
        g_acc_addr = ACC_ADDR_ALT;
        err        = drv_i2c_write_reg(g_acc_addr, CTRL_REG1_A, 0x57);
        if (err < E_OK)
        {
            return err;
        }
    }
    err = drv_i2c_write_reg(g_acc_addr, CTRL_REG4_A, 0x98);
    if (err < E_OK)
    {
        return err;
    }
    g_mag_type = 0;
    g_mag_addr = MAG_ADDR;
    if ((rd1(MAG_ADDR, IRA_REG_M, &obj_id)) && (0x48 == obj_id))
    {
        g_mag_type = 1;                                     /* DLHC: 'H' */
        (void)drv_i2c_write_reg(MAG_ADDR, CRA_REG_M, 0x10); /* 15 Hz */
        (void)drv_i2c_write_reg(MAG_ADDR, CRB_REG_M, 0x20); /* +-1.3 gauss */
        (void)drv_i2c_write_reg(MAG_ADDR, MR_REG_M, 0x00);  /* continuous */
    }
    else if ((rd1(MAG_ADDR, WHO_AM_I_M, &obj_id)) && (0x40 == obj_id))
    {
        g_mag_type = 2; /* AGR */
        (void)drv_i2c_write_reg(MAG_ADDR, CFG_REG_A_M,
                                0x80); /* temp comp, 10 Hz, continuous */
        (void)drv_i2c_write_reg(MAG_ADDR, CFG_REG_C_M, 0x10); /* block update */
    }
    else
    {
        /* No action is required in this case. */
    }
    g_chip = CHIP_DLHC;
    tm_printf((uint8_t *)"IMU found: LSM303DLHC/AGR accelerometer at 0x%02x, "
                         "magnetometer: %s\n",
              g_acc_addr,
              (1 == g_mag_type)   ? "LSM303DLHC"
              : (2 == g_mag_type) ? "LSM303AGR"
                                  : "not found");
    return E_OK;
}

/**
 * @brief  Setup lsm303d.
 * @return Result as described above.
 */
LOCAL ER setup_lsm303d (uint8_t addr)
{
    ER err;
    g_acc_addr = g_mag_addr = addr;
    err                     = drv_i2c_write_reg(addr, D_CTRL1, 0x6F);
    if (err < E_OK)
    {
        return err; /* 100 Hz, block update, XYZ */
    }
    err = drv_i2c_write_reg(addr, D_CTRL2, 0x08);
    if (err < E_OK)
    {
        return err; /* +-4 g */
    }
    (void)drv_i2c_write_reg(addr, D_CTRL5,
                            0x64); /* mag high resolution, 6.25 Hz */
    (void)drv_i2c_write_reg(addr, D_CTRL6, 0x20); /* mag +-4 gauss */
    (void)drv_i2c_write_reg(addr, D_CTRL7, 0x00); /* mag continuous */
    g_chip     = CHIP_LSM303D;
    g_mag_type = 3;
    tm_printf((uint8_t *)"IMU found: LSM303D (accelerometer + magnetometer) at "
                         "0x%02x\n",
              addr);
    return E_OK;
}

/**
 * @brief  Setup adxl345.
 * @return Result as described above.
 */
LOCAL ER setup_adxl345 (uint8_t addr)
{
    ER err;
    g_acc_addr = addr;
    err        = drv_i2c_write_reg(addr, X_BW_RATE, 0x0A);
    if (err < E_OK)
    {
        return err; /* 100 Hz */
    }
    err = drv_i2c_write_reg(addr, X_DATA_FORMAT, 0x09);
    if (err < E_OK)
    {
        return err; /* full res, +-4 g */
    }
    err = drv_i2c_write_reg(addr, X_POWER_CTL, 0x08);
    if (err < E_OK)
    {
        return err; /* measure */
    }
    g_chip     = CHIP_ADXL345;
    g_mag_type = 0;
    tm_printf(
        (uint8_t *)"IMU found: ADXL345 accelerometer at 0x%02x (no compass)\n",
        addr);
    return E_OK;
}

/**
 * @brief  Setup mma845x.
 * @return Result as described above.
 */
LOCAL ER setup_mma845x (uint8_t addr, uint8_t who)
{
    ER err;
    g_acc_addr = addr;
    err        = drv_i2c_write_reg(addr, M_CTRL1, 0x00);
    if (err < E_OK)
    {
        return err; /* standby to configure */
    }
    err = drv_i2c_write_reg(addr, M_XYZ_CFG, 0x01);
    if (err < E_OK)
    {
        return err; /* +-4 g */
    }
    err = drv_i2c_write_reg(addr, M_CTRL1, 0x19);
    if (err < E_OK)
    {
        return err; /* 100 Hz, active */
    }
    g_chip     = CHIP_MMA845X;
    g_mag_type = 0;
    tm_printf(
        (uint8_t *)"IMU found: MMA845x accelerometer (id 0x%02x) at 0x%02x "
                   "(no compass)\n",
        who, addr);
    return E_OK;
}

/**
 * @brief  Imu setup.
 * @return Result as described above.
 */
LOCAL ER imu_setup (void)
{
    static const uint8_t alt[] = {0x1D, 0x1E, 0x53, 0x1C};
    uint8_t              val;
    uint32_t             idx;

    drv_i2c_init();
    g_chip     = CHIP_NONE;
    g_mag_type = 0;
    if (E_OK == setup_dlhc())
    {
        return E_OK;
    }
    for (idx = 0; idx < sizeof alt; idx++)
    {
        uint8_t aval = alt[idx];
        if ((((0x1D == aval) || (0x1E == aval)) &&
             (rd1(aval, D_WHO_AM_I, &val))) &&
            (0x49 == val))
        {
            return setup_lsm303d(aval);
        }
        if ((((0x1D == aval) || (0x53 == aval)) &&
             (rd1(aval, X_DEVID, &val))) &&
            (0xE5 == val))
        {
            return setup_adxl345(aval);
        }
        if ((((0x1D == aval) || (0x1C == aval)) &&
             (rd1(aval, M_WHO_AM_I, &val))) &&
            (((((0x1A == val) || (0x2A == val)) || (0x3A == val)) ||
              (0x4A == val)) ||
             (0x5A == val)))
        {
            return setup_mma845x(aval, val);
        }
    }
    if (0 == (g_setup_tries++ % 10U))
    { /* every ~10 s */
        i2c_scan_report();
        /* say what an unknown chip answers, so it can be added */
        for (idx = 0; idx < sizeof alt; idx++)
        {
            uint8_t aval = alt[idx];
            uint8_t w0f  = 0;
            uint8_t w00  = 0;
            uint8_t w0d  = 0;
            BOOL    b_ok = rd1(aval, 0x0F, &w0f);
            if (!b_ok)
            {
                continue;
            }
            (void)rd1(aval, 0x00, &w00);
            (void)rd1(aval, 0x0D, &w0d);
            tm_printf(
                (uint8_t *)"    chip at 0x%02x: reg 0x0F = 0x%02x, reg 0x00 = "
                           "0x%02x, reg 0x0D = 0x%02x"
                           " (send this line to identify it)\n",
                aval, w0f, w00, w0d);
        }
    }
    return E_NOEXS;
}

/**
 * @brief  Read accel.
 * @return Result as described above.
 */
LOCAL ER read_accel (int32_t p_arr[3])
{
    uint8_t bval[6];
    int32_t idx;
    ER      err;

    switch (g_chip)
    {
        case CHIP_DLHC: /* 12 bit left-justified, +-4 g high res: 2 mg per count
                         */
            err = drv_i2c_read_regs(g_acc_addr, OUT_X_L_A | AUTO_INC, bval, 6);
            if (err < E_OK)
            {
                return err;
            }
            for (idx = 0; idx < IMU_AXES; idx++)
            {
                p_arr[idx] = ((int32_t)(int16_t)(bval[2 * idx] |
                                                 (bval[(2 * idx) + 1] << 8)) >>
                              4) *
                             2;
            }
            break;
        case CHIP_LSM303D: /* 16 bit, +-4 g: 0.122 mg per count */
            err =
                drv_i2c_read_regs(g_acc_addr, D_OUT_X_L_A | AUTO_INC, bval, 6);
            if (err < E_OK)
            {
                return err;
            }
            for (idx = 0; idx < IMU_AXES; idx++)
            {
                p_arr[idx] = ((int32_t)(int16_t)(bval[2 * idx] |
                                                 (bval[(2 * idx) + 1] << 8)) *
                              122) /
                             1000;
            }
        break;
        case CHIP_ADXL345: /* full resolution: 3.9 mg per count, low byte first
                            */
            err = drv_i2c_read_regs(g_acc_addr, X_DATAX0, bval, 6);
            if (err < E_OK)
            {
                return err;
            }
            for (idx = 0; idx < IMU_AXES; idx++)
            {
                p_arr[idx] = ((int32_t)(int16_t)(bval[2 * idx] |
                                                 (bval[(2 * idx) + 1] << 8)) *
                              39) /
                             10;
            }
        break;
        case CHIP_MMA845X: /* 12 bit left-justified, high byte first, +-4 g: 512
                              per g */
            err = drv_i2c_read_regs(g_acc_addr, M_OUT_X_MSB, bval, 6);
            if (err < E_OK)
            {
                return err;
            }
            for (idx = 0; idx < IMU_AXES; idx++)
            {
                p_arr[idx] = (((int32_t)(int16_t)((bval[2 * idx] << 8) |
                                                  bval[(2 * idx) + 1]) >>
                               4) *
                              1000) /
                             512;
            }
        break;
        default:
            return E_NOEXS;
    }
    return E_OK;
}

/**
 * @brief  Read mag.
 * @return Result as described above.
 */
LOCAL ER read_mag (int32_t p_mem[3])
{
    uint8_t bval[6];
    ER      err;

    if (1 == g_mag_type)
    { /* X, Z, Y, high byte first */
        err = drv_i2c_read_regs(MAG_ADDR, OUT_X_H_M, bval, 6);
        if (err < E_OK)
        {
            return err;
        }
        p_mem[0] = (int16_t)((bval[0] << 8) | bval[1]);
        p_mem[2] = (int16_t)((bval[2] << 8) | bval[3]);
        p_mem[1] = (int16_t)((bval[4] << 8) | bval[5]);
    }
    else if (2 == g_mag_type)
    { /* X, Y, Z, low byte first */
        err = drv_i2c_read_regs(MAG_ADDR, OUTX_L_REG_M | AUTO_INC, bval, 6);
        if (err < E_OK)
        {
            return err;
        }
        p_mem[0] = (int16_t)(bval[0] | (bval[1] << 8));
        p_mem[1] = (int16_t)(bval[2] | (bval[3] << 8));
        p_mem[2] = (int16_t)(bval[4] | (bval[5] << 8));
    }
    else if (3 == g_mag_type)
    { /* LSM303D: X, Y, Z, low byte first */
        err = drv_i2c_read_regs(g_mag_addr, D_OUT_X_L_M | AUTO_INC, bval, 6);
        if (err < E_OK)
        {
            return err;
        }
        p_mem[0] = (int16_t)(bval[0] | (bval[1] << 8));
        p_mem[1] = (int16_t)(bval[2] | (bval[3] << 8));
        p_mem[2] = (int16_t)(bval[4] | (bval[5] << 8));
    }
    else
    {
        return E_NOEXS;
    }
    return E_OK;
}

/**
 * @brief  Task body (micro T-Kernel task entry).
 * @param  stacd  Required by the micro T-Kernel signature; not used.
 * @param  p_exinf  Required by the micro T-Kernel signature; not used.
 */
LOCAL void imu_task (int32_t stacd, void * p_exinf)
{
    hump_t   hump;
    int32_t  fval[3]      = {0, 0, 1000}; /* filtered accel, mg */
    int32_t  lvl_p        = 0;
    int32_t  lvl_r        = 0; /* learned level, 0.1 deg */
    int32_t  fails        = 0;
    int32_t  num          = 0;
    uint32_t tilt_t0      = 0;
    uint32_t impact_t     = 0;
    uint32_t ev_t[3]      = {0, 0, 0};
    BOOL     b_tilted     = FALSE;
    BOOL     b_have_level = FALSE;
    uint32_t lift_hist    = 0;
    uint32_t lift_t       = 0;
    int32_t  odo_last     = 0;

    (void)stacd;
    (void)p_exinf;
    imu_math_hump_reset(&hump);
    g_stat_val.heading = -1;
    while (imu_setup() < E_OK)
    {
        comms_fault_raise(FAULT_IMU);
        tk_dly_tsk(1000);
    }
    comms_fault_clear(FAULT_IMU);
    g_stat_val.ok = TRUE;

    for (;; num++)
    {
        int32_t  aval[3];
        int32_t  fwd;
        int32_t  left;
        int32_t  up;
        int32_t  pitch;
        int32_t  roll;
        int32_t  mag_g;
        uint32_t tval;
        uint32_t idx;

        tk_dly_tsk(IMU_PERIOD_MS);
        tval = motion_now_ms();

        if (read_accel(aval) < E_OK)
        {
            if (++fails >= 5)
            { /* ~100 ms of silence */
                g_stat_val.ok = FALSE;
                comms_fault_raise(FAULT_IMU);
                tk_dly_tsk(1000);
                if (E_OK == imu_setup())
                {
                    fails         = 0;
                    g_stat_val.ok = TRUE;
                    comms_fault_clear(FAULT_IMU);
                }
            }
            continue;
        }
        fails = 0;

        /* impact: on the raw sample, before smoothing */
        mag_g = (int32_t)imu_math_isqrt((uint32_t)((aval[0] * aval[0]) +
                                                   (aval[1] * aval[1]) +
                                                   (aval[2] * aval[2])));
        g_stat_val.shock_mg = ABS(mag_g - 1000);
        if ((g_stat_val.shock_mg > IMU_IMPACT_MG) && ((tval - impact_t) > 500U))
        {
            impact_t = tval;
            ev_t[1]  = tval;
            g_stat_val.events |= IMU_EV_IMPACT;
            g_stat_val.seq++;
            comms_fault_raise(FAULT_IMPACT);
            tm_printf((uint8_t *)"IMU impact %d mg\n", g_stat_val.shock_mg);
        }
        if (impact_t && ((tval - impact_t) > 3000U))
        {
            comms_fault_clear(FAULT_IMPACT);
            impact_t = 0;
        }

        /* picked up: sustained push above 1 g */
        {
            uint32_t kdx;
            uint32_t hval;
            lift_hist = ((lift_hist << 1) |
                         (((mag_g - 1000) > IMU_LIFT_MG) ? 1U : 0U)) &
                        ((1U << IMU_LIFT_WIN) - 1U);
            kdx = 0;
            for (hval = lift_hist; 0U != hval; hval >>= 1)
            {
                kdx += hval & 1U;
            }
            if (((kdx >= IMU_LIFT_HITS) && (num > IMU_SETTLE_SAMPLES)) &&
                ((tval - lift_t) > 1000U))
            {
                lift_t = tval;
                mission_event(EV_LIFT);
                tm_printf((uint8_t *)"IMU lift: %d mg above 1 g on %d of %d "
                                     "samples\n",
                          mag_g - 1000, (int32_t)kdx, IMU_LIFT_WIN);
            }
        }

        for (idx = 0; idx < IMU_AXES; idx++)
        {
            fval[idx] += (aval[idx] - fval[idx]) / 4; /* smoothing */
        }
        g_stat_val.ax = fval[0];
        g_stat_val.ay = fval[1];
        g_stat_val.az = fval[2];
        /* mounted upside down?  (IS27: the board reads -1 g on Z.)
           Decided once, after 0.5 s of smoothing: gravity pointing
           "down" on the up axis = flip up AND left (a 180 deg turn
           about the forward axis), so roll reads 0, not 180 deg. */
        if ((IMU_SETTLE_SAMPLES == num) &&
            ((g_up_sign * fval[IMU_UP_AXIS]) < -500))
        {
            g_up_sign   = -g_up_sign;
            g_left_sign = -g_left_sign;
            tm_printf((uint8_t *)"IMU is mounted upside down (up axis %d mg): "
                                 "flipping up and left axes\n",
                      fval[IMU_UP_AXIS]);
        }
        fwd   = IMU_FWD_SIGN * fval[IMU_FWD_AXIS];
        left  = g_left_sign * fval[IMU_LEFT_AXIS];
        up    = g_up_sign * fval[IMU_UP_AXIS];
        pitch = imu_math_atan2_d10(
            fwd,
            (int32_t)imu_math_isqrt((uint32_t)((left * left) + (up * up))));
        roll = imu_math_atan2_d10(left, up);

        /* level: learnt while parked, steady and NOT being moved, and
           only small corrections (under 3 deg).  So a car pushed by
           hand over a hump, or set down on a ramp, still reads the
           slope instead of "learning" it away within a second. */
        {
            int32_t odo     = motion_odometry_mm();
            BOOL    b_still = (odo == odo_last);
            odo_last        = odo;
            if (!b_have_level)
            {
                /* first level: after the smoothing has settled and
                   the orientation is known (not from sample 1, which
                   started from a made-up 0, 0, 1 g) */
                if (num < (IMU_SETTLE_SAMPLES + 5))
                {
                    continue;
                }
                lvl_p        = pitch;
                lvl_r        = roll;
                b_have_level = TRUE;
                tm_printf((uint8_t *)"IMU level learnt: pitch %d roll %d (0.1 "
                                     "deg) = 0\n",
                          pitch, roll);
            }
            else if (((((MS_IDLE == mission_state()) &&
                        (g_stat_val.shock_mg < 60)) &&
                       b_still) &&
                      (ABS(pitch - lvl_p) < 30)) &&
                     (ABS(roll - lvl_r) < 30))
            {
                lvl_p += (pitch - lvl_p) / 16;
                lvl_r += (roll - lvl_r) / 16;
            }
            else
            {
                /* No action is required in this case. */
            }
        }
        g_stat_val.pitch_d10 = pitch - lvl_p;
        g_stat_val.roll_d10  = roll - lvl_r;

        /* hump */
        {
            int32_t h_mm;
            int32_t res;
            res =
                imu_math_hump_step(&hump, &g_hump_tune, g_stat_val.pitch_d10,
                                   (int32_t)motion_odometry_mm(), tval, &h_mm);
            if (res)
            {
                g_stat_val.humps++;
                g_stat_val.hump_mm = (int32_t)h_mm;
                g_stat_val.events |= IMU_EV_HUMP;
                ev_t[0] = tval;
                g_stat_val.seq++;
                tm_printf((uint8_t *)"IMU %s #%d, %d mm %s\n",
                          (1 == res) ? "hump / ramp up" : "ramp down",
                          (int32_t)g_stat_val.humps, (int32_t)h_mm,
                          (1 == res) ? "high" : "drop");
            }
        }

        /* tilt: tipped over or picked up */
        if ((ABS(g_stat_val.pitch_d10) > IMU_TILT_D10) ||
            (ABS(g_stat_val.roll_d10) > IMU_TILT_D10))
        {
            if (0 == tilt_t0)
            {
                tilt_t0 = tval;
            }
            if ((!b_tilted) && ((tval - tilt_t0) > IMU_TILT_MS))
            {
                b_tilted = TRUE;
                g_stat_val.events |= IMU_EV_TILT;
                ev_t[2] = tval;
                g_stat_val.seq++;
                mission_event(EV_TILT);
                tm_printf((uint8_t *)"IMU tilt: pitch %d roll %d (0.1 deg)\n",
                          g_stat_val.pitch_d10, g_stat_val.roll_d10);
            }
        }
        else
        {
            tilt_t0  = 0;
            b_tilted = FALSE;
        }

        /* event bits fade after a second (they ride on robot/tlm/terrain) */
        for (idx = 0; idx < IMU_AXES; idx++)
        {
            if ((g_stat_val.events & (1U << idx)) &&
                ((tval - ev_t[idx]) > IMU_EVENT_SHOW_MS))
            {
                g_stat_val.events &= ~(1U << idx);
            }
        }

        /* compass, 10 Hz (uncalibrated: close to the motors it is rough) */
        if ((0 == (num % 5)) && g_mag_type)
        {
            int32_t mval[3];
            if (E_OK == read_mag(mval))
            {
                int32_t mf_val = IMU_FWD_SIGN * mval[IMU_FWD_AXIS];
                int32_t ml_val = g_left_sign * mval[IMU_LEFT_AXIS];
                int32_t hd_val = imu_math_atan2_d10(-ml_val, mf_val) /
                                 10; /* clockwise from north */
                g_stat_val.heading = (hd_val < 0) ? hd_val + 360 : hd_val;
            }
        }
    }
}

/**
 * @brief  Initialise the module (create its kernel objects).
 * @return Result as described above.
 */
EXPORT ER subsystem4_init (void)
{
    return E_OK;
} /** no kernel objects */

EXPORT ER subsystem4_start (void)
{
    ID tid = app_main_task_start((FP)imu_task, PRI_IMU, CORE_IMU, "imu");
    return (tid < E_OK) ? tid : E_OK;
}

#if APP_TEST
/**
 * @brief  On-target self-test of this subsystem (built only when
 *         APP_TEST = 1).
 */
EXPORT void subsystem4_test (void)
{
}
#endif

/*** end of file ***/
