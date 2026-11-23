/**
 * @file    code39.c
 *  code39.c  (Buddy 3)
 *  Code 39 decoding.  See code39.h.
 */
#include "code39.h"

/* standard Code 39 table: 9 elements b s b s b s b s b, bit 8 first, 1 = wide
 */
static const struct
{
    char     ch;
    uint16_t pat;
} g_tab[] = {
    {'0', 0x034}, {'1', 0x121}, {'2', 0x061}, {'3', 0x160}, {'4', 0x031},
    {'5', 0x130}, {'6', 0x070}, {'7', 0x025}, {'8', 0x124}, {'9', 0x064},
    {'A', 0x109}, {'B', 0x049}, {'C', 0x148}, {'D', 0x019}, {'E', 0x118},
    {'F', 0x058}, {'G', 0x00D}, {'H', 0x10C}, {'I', 0x04C}, {'J', 0x01C},
    {'K', 0x103}, {'L', 0x043}, {'M', 0x142}, {'N', 0x013}, {'O', 0x112},
    {'P', 0x052}, {'Q', 0x007}, {'R', 0x106}, {'S', 0x046}, {'T', 0x016},
    {'U', 0x181}, {'V', 0x0C1}, {'W', 0x1C0}, {'X', 0x091}, {'Y', 0x190},
    {'Z', 0x0D0}, {'-', 0x085}, {'.', 0x184}, {' ', 0x0C4}, {'*', 0x094},
    {'$', 0x0A8}, {'/', 0x0A2}, {'+', 0x08A}, {'%', 0x02A},
};
#define NTAB ((int32_t)(sizeof g_tab / sizeof g_tab[0]))

/**
 * @brief  Code39 pattern.
 * @return Result as described above.
 */
int32_t code39_pattern (char chr)
{
    int32_t idx;
    for (idx = 0; idx < NTAB; idx++)
    {
        if (g_tab[idx].ch == chr)
        {
            return g_tab[idx].pat;
        }
    }
    return -1;
}

/** one character from 9 widths */
static int32_t one_char (const uint32_t * p_wrk, int32_t ratio_x10,
                         char * p_chr)
{
    uint32_t pat        = 0;
    uint32_t wide_min   = 0xFFFFFFFFU;
    uint32_t narrow_max = 0;
    int32_t  idx;
    int32_t  jdx;
    int32_t  rank;

    for (idx = 0; idx < 9; idx++)
    {
        rank = 0; /* how many are longer than w[i] */
        for (jdx = 0; jdx < 9; jdx++)
        {
            if ((p_wrk[jdx] > p_wrk[idx]) ||
                ((p_wrk[jdx] == p_wrk[idx]) && (jdx < idx)))
            {
                rank++;
            }
        }
        if (rank < 3)
        {
            pat |= 1U << (8 - idx);
            if (p_wrk[idx] < wide_min)
            {
                wide_min = p_wrk[idx];
            }
        }
        else if (p_wrk[idx] > narrow_max)
        {
            narrow_max = p_wrk[idx];
        }
        else
        {
            /* No action is required in this case. */
        }
    }
    if (((uint64_t)wide_min * 10U) <
        ((uint64_t)narrow_max * (uint32_t)ratio_x10))
    {
        return C39_ERR_RATIO;
    }
    for (idx = 0; idx < NTAB; idx++)
    {
        if (g_tab[idx].pat == pat)
        {
            *p_chr = g_tab[idx].ch;
            return 0;
        }
    }
    return C39_ERR_UNKNOWN;
}

/**
 * @brief  Decode dir.
 * @return Result as described above.
 */
static int32_t decode_dir (const uint32_t * p_wrk, int32_t num,
                           int32_t ratio_x10, char * p_out, int32_t cap)
{
    int32_t kdx = (num + 1) / 10;
    int32_t cval;
    int32_t len = 0;
    int32_t res;
    char    chr;

    for (cval = 0; cval < kdx; cval++)
    {
        res = one_char(&p_wrk[10 * cval], ratio_x10, &chr);
        if (res < 0)
        {
            return res;
        }
        if ((0 == cval) || (cval == (kdx - 1)))
        {
            if (chr != '*')
            {
                return C39_ERR_FRAME;
            }
        }
        else
        {
            if ('*' == chr)
            {
                return C39_ERR_FRAME;
            }
            if (len >= (cap - 1))
            {
                return C39_ERR_SPACE;
            }
            p_out[len++] = chr;
        }
    }
    p_out[len] = '\0';
    return len;
}

/** Full ASCII pairs (standard table) */
static int32_t full_pair (char pval, char lval)
{
    if ((lval < 'A') || (lval > 'Z'))
    {
        return -1;
    }
    switch (pval)
    {
        case '+':
            return 'a' + (lval - 'A'); /* +A..+Z  a..z */
        case '$':
            return 0x01 + (lval - 'A'); /* $A..$Z  ^A..^Z */
        case '/':
            if (lval <= 'O')
            {
                return '!' + (lval - 'A'); /* /A../O  ! .. / */
            }
            if ('Z' == lval)
            {
                return ':';
            }
            return -1;
        case '%':
            if (lval <= 'E')
            {
                return 0x1B + (lval - 'A'); /* ESC FS GS RS US */
            }
            if (lval <= 'J')
            {
                return ';' + (lval - 'F'); /* ; < = > ? */
            }
            if (lval <= 'O')
            {
                return '[' + (lval - 'K'); /* [ \ ] ^ _ */
            }
            if (lval <= 'T')
            {
                return '{' + (lval - 'P'); /* { | } ~ DEL */
            }
            if ('U' == lval)
            {
                return 0x00;
            }
            if ('V' == lval)
            {
                return '@';
            }
            if ('W' == lval)
            {
                return '`';
            }
            return 0x7F; /* %X %Y %Z */
        default:
            return -1;
    }
}

/**
 * @brief  Code39 full ascii.
 * @return Result as described above.
 */
int32_t code39_full_ascii (const char * p_inp, char * p_out, int32_t cap)
{
    int32_t idx = 0;
    int32_t len = 0;
    int32_t val;

    while (p_inp[idx])
    {
        if (len >= (cap - 1))
        {
            return C39_ERR_SPACE;
        }
        if (((('$' == p_inp[idx]) || ('%' == p_inp[idx])) ||
             ('/' == p_inp[idx])) ||
            ('+' == p_inp[idx]))
        {
            val = full_pair(p_inp[idx], p_inp[idx + 1]);
            if (val < 0)
            {
                return C39_ERR_FULLASCII;
            }
            p_out[len++] = (char)val;
            idx += 2;
        }
        else
        {
            p_out[len++] = p_inp[idx++];
        }
    }
    p_out[len] = '\0';
    return len;
}

/**
 * @brief  Code39 decode.
 * @return Result as described above.
 */
int32_t code39_decode (const uint32_t * p_wrk, int32_t num, int32_t ratio_x10,
                       char * p_out, int32_t cap, int32_t * p_reversed)
{
    uint32_t rev[10 * 12];
    int32_t  res;
    int32_t  idx;

    *p_reversed = 0;
    if (((num < 19) || (((num + 1) % 10) != 0)) ||
        (num > (int32_t)(sizeof rev / sizeof rev[0])))
    {
        return C39_ERR_LENGTH;
    }
    res = decode_dir(p_wrk, num, ratio_x10, p_out, cap);
    if (res >= 0)
    {
        return res;
    }
    for (idx = 0; idx < num; idx++)
    {
        rev[idx] = p_wrk[num - 1 - idx];
    }
    if (decode_dir(rev, num, ratio_x10, p_out, cap) >= 0)
    {
        *p_reversed = 1;
        for (idx = 0; p_out[idx]; idx++)
        {
        }
        return idx;
    }
    return res; /* the forward error is the more useful one */
}

/*** end of file ***/
