/**
 * @file    comms_frames.c
 *  comms_frames.c  (Buddy 1)
 *  Packing of the binary MQTT frames.  See comms_frames.h.
 */
#include "comms_frames.h"

/**
 * @brief  Comms frames bits put.
 */
void comms_frames_bits_put (uint8_t * p_buf, uint32_t pos, uint32_t width,
                            uint32_t val)
{
    uint32_t idx;
    for (idx = 0; idx < width; idx++)
    {
        if ((val >> idx) & 1U)
        {
            p_buf[(pos + idx) >> 3] |= (uint8_t)(1U << ((pos + idx) & 7U));
        }
    }
}

/**
 * @brief  Comms frames bits get.
 * @return Result as described above.
 */
uint32_t comms_frames_bits_get (const uint8_t * p_buf, uint32_t pos,
                                uint32_t width)
{
    uint32_t idx;
    uint32_t val = 0;
    for (idx = 0; idx < width; idx++)
    {
        if ((p_buf[(pos + idx) >> 3] >> ((pos + idx) & 7U)) & 1U)
        {
            val |= 1U << idx;
        }
    }
    return val;
}

/**
 * @brief  Zero.
 */
static void zero (uint8_t * p_buf, uint32_t num)
{
    while (num--)
    {
        *p_buf++ = 0;
    }
}

/**
 * @brief  Sat u.
 * @return Result as described above.
 */
static uint32_t sat_u (int32_t val, uint32_t width)
{
    int32_t max = (int32_t)((1U << width) - 1U);
    return (uint32_t)((val < 0) ? 0 : (val > max) ? max : val);
}

/**
 * @brief  Sat s.
 * @return Result as described above.
 */
static uint32_t sat_s (int32_t  val,
                       uint32_t width) /* two's complement in width */
{
    int32_t max = (int32_t)((1U << (width - 1)) - 1U);
    int32_t min = -max - 1;
    val         = (val < min) ? min : (val > max) ? max : val;
    return (uint32_t)val & ((1U << width) - 1U);
}

/**
 * @brief  Comms frames status.
 * @return Result as described above.
 */
uint32_t comms_frames_status (uint8_t * p_buf, uint32_t seq, uint32_t up_s,
                              uint32_t rssi_neg, uint32_t mode, uint32_t faults,
                              uint32_t tx_drop)
{
    zero(p_buf, FR_STATUS_BYTES);
    comms_frames_bits_put(p_buf, 0, 4, seq & 0xFU);
    comms_frames_bits_put(p_buf, 4, 20,
                          up_s & 0xFFFFFU); /* wraps after 12 days */
    comms_frames_bits_put(p_buf, 24, 7, sat_u((int32_t)rssi_neg, 7));
    comms_frames_bits_put(p_buf, 31, 4, sat_u((int32_t)mode, 4));
    comms_frames_bits_put(p_buf, 35, 12, faults & 0xFFFU);
    comms_frames_bits_put(p_buf, 47, 1, tx_drop ? 1U : 0U);
    return FR_STATUS_BYTES;
}

/**
 * @brief  Comms frames fault.
 * @return Result as described above.
 */
uint32_t comms_frames_fault (uint8_t * p_buf, uint32_t t_ms, uint32_t raised)
{
    zero(p_buf, FR_FAULT_BYTES);
    comms_frames_bits_put(p_buf, 0, 16, t_ms & 0xFFFFU);
    comms_frames_bits_put(p_buf, 16, 12, raised & 0xFFFU);
    return FR_FAULT_BYTES;
}

/**
 * @brief  Comms frames motion.
 * @return Result as described above.
 */
uint32_t comms_frames_motion (uint8_t * p_buf, uint32_t t_ms, int32_t vel_left,
                              int32_t vel_right, int32_t dist_cm,
                              uint32_t owner, uint32_t enc_ok)
{
    zero(p_buf, FR_MOTION_BYTES);
    comms_frames_bits_put(p_buf, 0, 16, t_ms & 0xFFFFU);
    comms_frames_bits_put(p_buf, 16, 11, sat_s(vel_left, 11));
    comms_frames_bits_put(p_buf, 27, 11, sat_s(vel_right, 11));
    comms_frames_bits_put(p_buf, 38, 14, sat_u(dist_cm, 14));
    comms_frames_bits_put(p_buf, 52, 2, owner & 3U);
    comms_frames_bits_put(p_buf, 54, 1, enc_ok & 1U); /* left encoder healthy */
    comms_frames_bits_put(p_buf, 55, 1,
                          (enc_ok >> 1) &
                              1U); /* right encoder healthy (was a spare bit) */
    return FR_MOTION_BYTES;
}

/**
 * @brief  Comms frames line.
 * @return Result as described above.
 */
uint32_t comms_frames_line (uint8_t * p_buf, uint32_t t_ms, uint32_t mode,
                            int32_t pos, uint32_t see, char bc_char,
                            uint32_t bc_action, int32_t speed_mms,
                            int32_t bend_pct, int32_t kp_x1000, uint32_t pivot)
{
    uint32_t ci_val = ((bc_char >= ' ') && (bc_char <= 'Z'))
                          ? (uint32_t)(bc_char - ' ' + 1)
                          : 0U;

    zero(p_buf, FR_LINE_BYTES);
    comms_frames_bits_put(p_buf, 0, 16, t_ms & 0xFFFFU);
    comms_frames_bits_put(p_buf, 16, 3, mode & 7U);
    comms_frames_bits_put(p_buf, 19, 8, sat_s(pos / 8, 8));
    comms_frames_bits_put(p_buf, 27, 2, see & 3U);
    comms_frames_bits_put(p_buf, 29, 6,
                          ci_val); /* 0 none, 1 = ' ' .. 59 = 'Z' */
    comms_frames_bits_put(p_buf, 35, 3, bc_action & 7U);
    comms_frames_bits_put(p_buf, 38, 8, sat_u(speed_mms / 10, 8));
    comms_frames_bits_put(p_buf, 46, 7, sat_u(bend_pct, 7));
    comms_frames_bits_put(p_buf, 53, 5, sat_u(kp_x1000 / 100, 5));
    comms_frames_bits_put(p_buf, 58, 1, pivot ? 1U : 0U);
    return FR_LINE_BYTES;
}

/**
 * @brief  Comms frames cmd tune.
 * @return Result as described above.
 */
uint32_t comms_frames_cmd_tune (const uint8_t * p_buf, uint32_t len,
                                int32_t * p_mms, int32_t * p_bend,
                                int32_t * p_kp_val, int32_t * p_pivot)
{
    if (((len < 2) || (len > 5)) || (p_buf[0] != 0))
    {
        return 0;
    }
    *p_mms    = (int32_t)p_buf[1] * 10;
    *p_bend   = (len >= 3) ? (int32_t)p_buf[2] : 0;
    *p_kp_val = (len >= 4) ? (int32_t)p_buf[3] * 100 : 0;
    *p_pivot =
        (len >= 5) ? (int32_t)p_buf[4] : 0; /* 0 unchanged, 1 off, 2 on */
    return 1;
}

/**
 * @brief  Comms frames terrain.
 * @return Result as described above.
 */
uint32_t comms_frames_terrain (uint8_t * p_buf, uint32_t t_ms,
                               int32_t pitch_d10, int32_t roll_d10,
                               int32_t hump_mm, int32_t heading,
                               uint32_t events)
{
    zero(p_buf, FR_TERRAIN_BYTES);
    comms_frames_bits_put(p_buf, 0, 16, t_ms & 0xFFFFU);
    comms_frames_bits_put(p_buf, 16, 10, sat_s(pitch_d10, 10));
    comms_frames_bits_put(p_buf, 26, 10, sat_s(roll_d10, 10));
    comms_frames_bits_put(p_buf, 36, 8, sat_u(hump_mm, 8));
    comms_frames_bits_put(p_buf, 44, 9,
                          (heading < 0) ? 511U : sat_u(heading, 9));
    comms_frames_bits_put(p_buf, 53, 3, events & 7U);
    return FR_TERRAIN_BYTES;
}

/**
 * @brief  Comms frames obstacle.
 * @return Result as described above.
 */
uint32_t comms_frames_obstacle (uint8_t * p_buf, uint32_t t_ms, int32_t dist,
                                int32_t bearing, int32_t width, int32_t need_l,
                                int32_t need_r, uint32_t action, uint32_t stage)
{
    zero(p_buf, FR_OBSTACLE_BYTES);
    comms_frames_bits_put(p_buf, 0, 16, t_ms & 0xFFFFU);
    comms_frames_bits_put(p_buf, 16, 8, sat_u(dist, 8));
    comms_frames_bits_put(p_buf, 24, 8, sat_s(bearing, 8));
    comms_frames_bits_put(p_buf, 32, 8, sat_u(width, 8));
    comms_frames_bits_put(p_buf, 40, 8, sat_u(need_l, 8));
    comms_frames_bits_put(p_buf, 48, 8, sat_u(need_r, 8));
    comms_frames_bits_put(p_buf, 56, 2, action & 3U);
    comms_frames_bits_put(p_buf, 58, 3, stage & 7U);
    return FR_OBSTACLE_BYTES;
}

/**
 * @brief  Comms frames resources.
 * @return Result as described above.
 */
uint32_t comms_frames_resources (uint8_t * p_buf, uint32_t t_ms,
                                 const fr_res_t * p_res)
{
    zero(p_buf, FR_RESOURCES_BYTES);
    comms_frames_bits_put(p_buf, 0, 16, t_ms & 0xFFFFU);
    comms_frames_bits_put(p_buf, 16, 7, sat_u((int32_t)p_res->cpu0, 7));
    comms_frames_bits_put(p_buf, 23, 7, sat_u((int32_t)p_res->cpu1, 7));
    comms_frames_bits_put(p_buf, 30, 6, sat_u((int32_t)p_res->tasks, 6));
    comms_frames_bits_put(p_buf, 36, 6, sat_u((int32_t)p_res->tasks_max, 6));
    comms_frames_bits_put(p_buf, 42, 5, sat_u((int32_t)p_res->flags, 5));
    comms_frames_bits_put(p_buf, 47, 5, sat_u((int32_t)p_res->flags_max, 5));
    comms_frames_bits_put(p_buf, 52, 5, sat_u((int32_t)p_res->sems, 5));
    comms_frames_bits_put(p_buf, 57, 5, sat_u((int32_t)p_res->sems_max, 5));
    comms_frames_bits_put(p_buf, 62, 4, sat_u((int32_t)p_res->mbfs, 4));
    comms_frames_bits_put(p_buf, 66, 4, sat_u((int32_t)p_res->mbfs_max, 4));
    comms_frames_bits_put(p_buf, 70, 4, sat_u((int32_t)p_res->cycs, 4));
    comms_frames_bits_put(p_buf, 74, 4, sat_u((int32_t)p_res->cycs_max, 4));
    comms_frames_bits_put(p_buf, 78, 7, sat_u((int32_t)p_res->queue_pct, 7));
    comms_frames_bits_put(p_buf, 85, 8, sat_u((int32_t)p_res->drops, 8));
    comms_frames_bits_put(p_buf, 93, 12, sat_u((int32_t)p_res->motion_us, 12));
    comms_frames_bits_put(p_buf, 105, 12, sat_u((int32_t)p_res->line_us, 12));
    return FR_RESOURCES_BYTES;
}

/**
 * @brief  Comms frames cmd decode.
 * @return Result as described above.
 */
uint32_t comms_frames_cmd_decode (const uint8_t * p_buf, uint32_t len,
                                  uint32_t * p_ident)
{
    uint32_t code;
    if (len != 1)
    {
        return 0;
    }
    code     = p_buf[0] & 7U;
    *p_ident = (uint32_t)p_buf[0] >> 3;
    return ((code >= 1) && (code <= 7)) ? code : 0;
}

/**
 * @brief  Decode a robot/map frame: one 3-bit action per letter A..Z.
 * @param  p_buf  Received payload.
 * @param  len    Payload length, bytes.
 * @param  p_act  Output, FR_MAP_LETTERS actions; written only if valid.
 * @return 1 if the frame is valid, else 0.
 */
uint32_t comms_frames_map_decode (const uint8_t * p_buf, uint32_t len,
                                  uint8_t * p_act)
{
    uint32_t idx;
    uint32_t result = 1U;

    if (FR_MAP_BYTES != len)
    {
        result = 0U;
    }

    /* Check every action first, so a bad frame changes nothing. */
    for (idx = 0U; (1U == result) && (idx < FR_MAP_LETTERS); idx++)
    {
        if (comms_frames_bits_get(p_buf, idx * FR_MAP_ACT_BITS,
                                  FR_MAP_ACT_BITS) > FR_MAP_ACT_MAX)
        {
            result = 0U;
        }
    }

    for (idx = 0U; (1U == result) && (idx < FR_MAP_LETTERS); idx++)
    {
        p_act[idx] = (uint8_t)comms_frames_bits_get(
            p_buf, idx * FR_MAP_ACT_BITS, FR_MAP_ACT_BITS); /* 3 bits: fits */
    }

    return result;
}

/**
 * @brief  Build a robot/tlm/map frame: the map in use, then a flags byte.
 * @param  p_buf     Output buffer, at least FR_MAP_ACK_BYTES.
 * @param  p_act     FR_MAP_LETTERS actions (out-of-range sent as 0).
 * @param  received  Non-zero once a map has arrived from the GUI.
 * @return Frame length, FR_MAP_ACK_BYTES.
 */
uint32_t comms_frames_map_ack (uint8_t * p_buf, const uint8_t * p_act,
                               uint32_t received)
{
    uint32_t idx;

    zero(p_buf, FR_MAP_ACK_BYTES);
    for (idx = 0U; idx < FR_MAP_LETTERS; idx++)
    {
        comms_frames_bits_put(p_buf, idx * FR_MAP_ACT_BITS, FR_MAP_ACT_BITS,
                              (p_act[idx] > FR_MAP_ACT_MAX) ? 0U : p_act[idx]);
    }
    p_buf[FR_MAP_BYTES] = (uint8_t)((0U != received) ? 1U : 0U);

    return FR_MAP_ACK_BYTES;
}

/*** end of file ***/
