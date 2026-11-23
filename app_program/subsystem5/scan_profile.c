/**
 * @file    scan_profile.c
 *  scan_profile.c  (Buddy 5)
 *  Fan sweep -> obstacle profile.  Pure logic, see scan_profile.h.
 */
#include "scan_profile.h"

/* sin x 1000 for 0, 5, 10 ... 90 degrees */
static const int16_t g_sin5[19] = {0,   87,  174, 259, 342, 423, 500,
                                   574, 643, 707, 766, 819, 866, 906,
                                   940, 966, 985, 996, 1000};

/**
 * @brief  Scan profile isin x1000.
 * @return Result as described above.
 */
int32_t scan_profile_isin_x1000 (int32_t deg)
{
    int32_t sval = 1;
    int32_t idx;
    int32_t fval;
    int32_t val;

    while (deg > 180)
    {
        deg -= 360;
    }
    while (deg < -180)
    {
        deg += 360;
    }
    if (deg < 0)
    {
        deg  = -deg;
        sval = -1;
    }
    if (deg > 90)
    {
        deg = 180 - deg;
    }
    idx  = deg / 5;
    fval = deg % 5;
    val  = (idx >= 18)
               ? 1000
               : g_sin5[idx] + (((g_sin5[idx + 1] - g_sin5[idx]) * fval) / 5);
    return sval * val;
}

/**
 * @brief  Lat cm.
 * @return Result as described above.
 */
static int32_t lat_cm (int32_t cm_val,
                       int32_t deg) /* sideways position, + = left */
{
    return (cm_val * scan_profile_isin_x1000(deg)) / 1000;
}

/**
 * @brief  Scan profile compute.
 */
void scan_profile_compute (const scan_tune_t * p_key, const int32_t * p_deg,
                           const int32_t * p_cm_val, int32_t num,
                           scan_profile_t * p_pval)
{
    int32_t idx;
    int32_t imin = -1;
    int32_t il_val;
    int32_t ir_val;
    int32_t lim;

    p_pval->found   = 0;
    p_pval->dir     = 0;
    p_pval->need_cm = 0;
    if ((num <= 0) || (num > SCAN_MAX_PTS))
    {
        return;
    }

    /* 1. closest reading (ties: the one nearest straight ahead) */
    for (idx = 0; idx < num; idx++)
    {
        int32_t aval = (p_deg[idx] < 0) ? -p_deg[idx] : p_deg[idx];
        if (p_cm_val[idx] >= p_key->far_cm)
        {
            continue;
        }
        if (((imin < 0) || (p_cm_val[idx] < p_cm_val[imin])) ||
            ((p_cm_val[idx] == p_cm_val[imin]) &&
             (aval < ((p_deg[imin] < 0) ? -p_deg[imin] : p_deg[imin]))))
        {
            imin = idx;
        }
    }
    if (imin < 0)
    {
        return; /* nothing in front */
    }
    p_pval->found   = 1;
    p_pval->dmin_cm = p_cm_val[imin];

    /* 2. grow the run both ways while readings stay near dmin */
    lim = p_cm_val[imin] + p_key->band_cm;
    /* (one missed echo inside the obstacle is bridged) */
    il_val = imin;
    while ((il_val > 0) && ((p_cm_val[il_val - 1] <= lim) ||
                            ((il_val > 1) && (p_cm_val[il_val - 2] <= lim))))
    {
        il_val -= (p_cm_val[il_val - 1] <= lim) ? 1 : 2;
    }
    ir_val = imin;
    while ((ir_val < (num - 1)) &&
           ((p_cm_val[ir_val + 1] <= lim) ||
            ((ir_val < (num - 2)) && (p_cm_val[ir_val + 2] <= lim))))
    {
        ir_val += (p_cm_val[ir_val + 1] <= lim) ? 1 : 2;
    }
    p_pval->open_left  = il_val > 0; /* saw past the left edge */
    p_pval->open_right = ir_val < (num - 1);

    /* 3. edges, pulled in by the sound cone, never crossing */
    p_pval->left_deg  = p_deg[il_val] - p_key->beam_deg;
    p_pval->right_deg = p_deg[ir_val] + p_key->beam_deg;
    if (p_pval->left_deg < p_pval->right_deg)
    {
        int32_t mid      = (p_deg[il_val] + p_deg[ir_val]) / 2;
        p_pval->left_deg = p_pval->right_deg = mid;
    }
    p_pval->left_cm  = lat_cm(p_cm_val[il_val], p_pval->left_deg);
    p_pval->right_cm = lat_cm(p_cm_val[ir_val], p_pval->right_deg);
    if (p_pval->right_cm > p_pval->left_cm)
    { /* edges crossed: one narrow thing */
        int32_t mid     = (p_pval->left_cm + p_pval->right_cm) / 2;
        p_pval->left_cm = p_pval->right_cm = mid;
    }
    p_pval->width_cm    = p_pval->left_cm - p_pval->right_cm;
    p_pval->bearing_deg = (p_deg[il_val] + p_deg[ir_val]) / 2;

    /* 4./5. sideways move needed for each side (never negative) */
    p_pval->need_left_cm =
        p_pval->left_cm + p_key->half_car_cm + p_key->margin_cm;
    p_pval->need_right_cm =
        -p_pval->right_cm + p_key->half_car_cm + p_key->margin_cm;
    if (p_pval->need_left_cm < 0)
    {
        p_pval->need_left_cm = 0;
    }
    if (p_pval->need_right_cm < 0)
    {
        p_pval->need_right_cm = 0;
    }

    if ((p_pval->open_left) && (p_pval->open_right))
    {
        if (p_pval->need_left_cm == p_pval->need_right_cm)
        {
            p_pval->dir = (p_key->prefer_left) ? 1 : -1;
        }
        else
        {
            p_pval->dir =
                (p_pval->need_left_cm < p_pval->need_right_cm) ? 1 : -1;
        }
    }
    else if (p_pval->open_left)
    {
        p_pval->dir = 1;
    }
    else if (p_pval->open_right)
    {
        p_pval->dir = -1;
    }
    else
    {
        /* No action is required in this case. */
    }
    p_pval->need_cm = (p_pval->dir > 0)   ? p_pval->need_left_cm
                      : (p_pval->dir < 0) ? p_pval->need_right_cm
                                          : 0;
}

/*** end of file ***/
