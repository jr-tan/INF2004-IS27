/**
 * @file    imu_math.c
 *  imu_math.c  (Buddy 4)
 *  See imu_math.h.
 */
#include "imu_math.h"

/* Half a turn, 180.0 degrees, in 0.1 degree units (BARR-C 8.4.a). */
#define HALF_TURN_D10 1800

/** atan(z) for z = 0..1000 (x1000), in 0.1 deg:
   pi/4 z + 0.273 z (1 - z) rad, max error ~0.2 deg */
static int32_t atan01_d10 (int32_t zval)
{
    return ((450 * zval) / 1000) + ((1564 * zval * (1000 - zval)) / 10000000);
}

/**
 * @brief  Imu math atan2 d10.
 * @return Result as described above.
 */
int32_t imu_math_atan2_d10 (int32_t yval, int32_t xval)
{
    int32_t ax_val = (xval < 0) ? -xval : xval;
    int32_t ay_val = (yval < 0) ? -yval : yval;
    int32_t aval;

    if ((0 == ax_val) && (0 == ay_val))
    {
        return 0;
    }
    if (ax_val >= ay_val)
    {
        aval = atan01_d10((int32_t)(((int64_t)ay_val * 1000) / ax_val));
    }
    else
    {
        aval = 900 - atan01_d10((int32_t)(((int64_t)ax_val * 1000) / ay_val));
    }
    if (xval < 0)
    {
        aval = 1800 - aval;
    }
    return (yval < 0) ? -aval : aval;
}

/**
 * @brief  Imu math isqrt.
 * @return Result as described above.
 */
uint32_t imu_math_isqrt (uint32_t val)
{
    uint32_t res = 0;
    uint32_t bit = 1U << 30;
    while (bit > val)
    {
        bit >>= 2;
    }
    while (bit)
    {
        if (val >= (res + bit))
        {
            val -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/* sin x 1000 every 5 deg, 0..90 */
static const int16_t g_sin5[19] = {0,   87,  174, 259, 342, 423, 500,
                                   574, 643, 707, 766, 819, 866, 906,
                                   940, 966, 985, 996, 1000};

/**
 * @brief  Imu math sin x1000.
 * @return Result as described above.
 */
int32_t imu_math_sin_x1000 (int32_t d10)
{
    int32_t sval = 1;
    int32_t idx;
    int32_t fval;
    while (d10 > HALF_TURN_D10)
    {
        d10 -= 3600;
    }
    while (d10 < -HALF_TURN_D10)
    {
        d10 += 3600;
    }
    if (d10 < 0)
    {
        d10  = -d10;
        sval = -1;
    }
    if (d10 > 900)
    {
        d10 = 1800 - d10;
    }
    idx  = d10 / 50;
    fval = d10 % 50;
    if (idx >= 18)
    {
        return sval * 1000;
    }
    return sval *
           (g_sin5[idx] + (((g_sin5[idx + 1] - g_sin5[idx]) * fval) / 50));
}

/**
 * @brief  Iabs32.
 * @return Result as described above.
 */
static int32_t iabs32 (int32_t val)
{
    return (val < 0) ? -val : val;
}

/**
 * @brief  Imu math hump reset.
 */
void imu_math_hump_reset (hump_t * p_hnd)
{
    p_hnd->st      = 0;
    p_hnd->t0      = 0;
    p_hnd->t_level = 0;
    p_hnd->s_prev  = 0;
    p_hnd->s0      = 0;
    p_hnd->h_um    = 0;
    p_hnd->peak_um = 0;
    p_hnd->base_um = 0;
}

/** Height is integrated ALL the time (sum of sin(pitch) x ds).  While the
   car is level, 'base' follows it; once a climb is confirmed, the hump's
   height is the highest point minus that base.  So the start of the
   climb, before the pitch crossed the threshold, is counted too. */
int32_t imu_math_hump_step (hump_t * p_hnd, const hump_tune_t * p_key,
                            int32_t pval, int32_t odo, uint32_t now,
                            int32_t * p_out)
{
    int32_t ds_val = odo - p_hnd->s_prev;
    int32_t level  = ((pval < p_key->off_d10) && (pval > -p_key->off_d10));

    p_hnd->s_prev = odo;
    p_hnd->h_um +=
        (int64_t)imu_math_sin_x1000(pval) * ds_val; /* mm x 1000 = um */
    if (((2 == p_hnd->st) || (3 == p_hnd->st)) &&
        ((p_hnd->h_um - p_hnd->base_um) > p_hnd->peak_um))
    {
        p_hnd->peak_um = p_hnd->h_um - p_hnd->base_um;
    }
    if ((6 == p_hnd->st) && ((p_hnd->base_um - p_hnd->h_um) > p_hnd->peak_um))
    {
        p_hnd->peak_um = p_hnd->base_um - p_hnd->h_um;
    }
    /* give up by DISTANCE (a slow car takes long over a hump; time
       alone used to cut off humps at 30 mm/s), or after max_ms */
    if ((((2 == p_hnd->st) || (3 == p_hnd->st)) || (6 == p_hnd->st)) &&
        (((p_key->max_mm > 0) && (iabs32(odo - p_hnd->s0) > p_key->max_mm)) ||
         ((now - p_hnd->t0) > p_key->max_ms)))
    {
        p_hnd->st      = 0;
        p_hnd->base_um = p_hnd->h_um;
        return 0;
    }

    switch (p_hnd->st)
    {
        case 0:
            if (level)
            {
                p_hnd->base_um = p_hnd->h_um;
            }
            if (pval > p_key->on_d10)
            {
                p_hnd->st = 1;
                p_hnd->t0 = now;
            }
            else if (pval < -p_key->on_d10)
            {
                p_hnd->st = 5;
                p_hnd->t0 = now;
            }
            else
            {
                /* No action is required in this case. */
            }
        break;
        case 1:
            if (pval <= p_key->on_d10)
            {
                p_hnd->st = 0;
            }
            else if ((now - p_hnd->t0) >= p_key->confirm_ms)
            {
                p_hnd->st      = 2;
                p_hnd->peak_um = p_hnd->h_um - p_hnd->base_um;
                p_hnd->t0      = now;
                p_hnd->s0      = odo;
                p_hnd->t_level = 0;
            }
            else
            {
                /* No action is required in this case. */
            }
        break;
        case 5: /* maybe going down */
            if (pval >= -p_key->on_d10)
            {
                p_hnd->st = 0;
            }
            else if ((now - p_hnd->t0) >= p_key->confirm_ms)
            {
                p_hnd->st      = 6;
                p_hnd->peak_um = p_hnd->base_um - p_hnd->h_um;
                p_hnd->t0      = now;
                p_hnd->s0      = odo;
                p_hnd->t_level = 0;
            }
            else
            {
                /* No action is required in this case. */
            }
        break;
        case 6: /* ramp down, no climb before it */
            if (level)
            {
                if (0 == p_hnd->t_level)
                {
                    p_hnd->t_level = now;
                }
                else if ((now - p_hnd->t_level) >= p_key->level_ms)
                {
                    p_hnd->st = 7;
                }
                else
                {
                    /* No action is required in this case. */
                }
            }
            else
            {
                p_hnd->t_level = 0;
            }
        break;
        case 2: /* climbing */
            if (pval < -p_key->on_d10)
            {
                p_hnd->st      = 3;
                p_hnd->t_level = 0;
            }
            else if (level)
            {
                if (0 == p_hnd->t_level)
                {
                    p_hnd->t_level = now;
                }
                else if ((now - p_hnd->t_level) >= (p_key->level_ms * 3))
                {
                    p_hnd->st = 4; /* plateau / ramp top */
                }
                else
                {
                    /* No action is required in this case. */
                }
            }
            else
            {
                p_hnd->t_level = 0;
            }
        break;
        case 3: /* coming down */
            if (level)
            {
                if (0 == p_hnd->t_level)
                {
                    p_hnd->t_level = now;
                }
                else if ((now - p_hnd->t_level) >= p_key->level_ms)
                {
                    p_hnd->st = 4;
                }
                else
                {
                    /* No action is required in this case. */
                }
            }
            else
            {
                p_hnd->t_level = 0;
            }
        break;
        default:
        {
            /* No other values are expected. */
        break;
        }
    }
    if ((4 == p_hnd->st) || (7 == p_hnd->st))
    {
        int32_t mm_val = (int32_t)(p_hnd->peak_um / 1000);
        int32_t res    = (4 == p_hnd->st) ? 1 : 2;
        p_hnd->st      = 0;
        p_hnd->base_um = p_hnd->h_um;
        p_hnd->peak_um = 0;
        if (mm_val >= p_key->min_mm)
        {
            *p_out = mm_val;
            return res;
        }
    }
    return 0;
}

/*** end of file ***/
