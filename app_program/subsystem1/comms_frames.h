/**
 * @file    comms_frames.h
 *  comms_frames.h  (Buddy 1)
 *  Binary MQTT payloads (Change Requirement 1: minimum bits, packed,
 *  no JSON).  Each frame is ONE little-endian bit string; the first
 *  field takes the LOWEST bits:
 *      v = int.from_bytes(frame, "little");  field = (v >> offset) & mask
 *  Built with explicit shifts and masks (comms_frames.c), never C
 *  bit-fields, so the PC dashboard can rely on the layout.
 *  Signed fields are two's complement in their width.  Values that
 *  don't fit are saturated, never wrapped.
 *
 *  Topic              Fields (bits)                                   Bytes
 *  robot/status       seq 4, uptime s 20, -rssi 7, mode 4, faults 12,
 *                     tx_drop 1                                         6
 *  robot/event/fault  t ms 16, raised faults 12, pad 4                  4
 *  robot/cmd (in)     code 3, id 5                                      1
 *  robot/tlm/motion   t 16, vl s11 (mm/s), vr s11, dist 14 (cm),
 *                     owner 2, enc L ok 1, enc R ok 1                   7
 *  robot/tlm/line     t 16, mode 3, pos s8 (/8), see 2, barcode char 6,
 *                     barcode action 3, set speed 8 (cm/s), bend % 7,
 *                     steering KP/100 5, bend stop + pivot on 1         8
 *  robot/tlm/terrain  t 16, pitch s10 (0.1 deg), roll s10, hump 8 (mm),
 *                     heading 9 (deg, 511 = none), events 3             7
 *  robot/tlm/obstacle t 16, dist 8, bearing s8, width 8, need L 8,
 *                     need R 8, action 2 (0 none 1 L 2 R 3 blocked),
 *                     stage 3 (OBS_* in car.h)                          8
 *  robot/tlm/resources t 16, cpu0 % 7, cpu1 % 7, tasks 6, tasks max 6,
 *                     flags 5, flags max 5, sems 5, sems max 5, mbfs 4,
 *                     mbfs max 4, cycs 4, cycs max 4, out-queue fill % 7,
 *                     tx drops 8, motion loop us 12, line loop us 12     15
 *  robot/cmd codes: 1 START 2 STOP 3 PAUSE 4 RESUME 5 PING 6 U-TURN 7 REVERSE
 *  robot/map (in,     26 x action 3 (letter A first), pad 2            10
 *    GUI, retained)   action: 0 straight 1 left 2 right 3 U-turn 4 stop
 *  robot/tlm/map      the map the car is using (same 10 bytes),
 *                     then flags 8 (bit 0 = a map arrived since boot)   11
 *
 *  The status / fault / cmd layouts are the team's (docs/buddy1_comms.md),
 *  so the existing dashboard still decodes them.
 *
 *  Pure C (stdint only): testing/comms_test/ checks it against the
 *  Python decoder.
 */
#ifndef COMMS_FRAMES_H
#define COMMS_FRAMES_H

#include <stdint.h>

#define FR_STATUS_BYTES    6
#define FR_FAULT_BYTES     4
#define FR_MOTION_BYTES    7
#define FR_LINE_BYTES      8
#define FR_TERRAIN_BYTES   7
#define FR_OBSTACLE_BYTES  8
#define FR_RESOURCES_BYTES 15
#define FR_MAP_BYTES       10
#define FR_MAP_ACK_BYTES   11
#define FR_MAP_LETTERS     26
#define FR_MAP_ACT_BITS    3
#define FR_MAP_ACT_MAX     4 /* = BC_ACT_END */

/* system mode on the wire (team's SYS_MODES list) */
#define SYS_INIT      0
#define SYS_IDLE      1
#define SYS_LINE      2
#define SYS_BARCODE   3
#define SYS_AVOID     4
#define SYS_RECOVERY  5
#define SYS_PAUSED    6
#define SYS_SAFE_STOP 7
#define SYS_COMPLETE  8

/* low-level: put 'width' bits of v at bit 'pos' (LE); buf must be zeroed */
void     comms_frames_bits_put (uint8_t * p_buf, uint32_t pos, uint32_t width,
                                uint32_t val);
uint32_t comms_frames_bits_get (const uint8_t * p_buf, uint32_t pos,
                                uint32_t width);

uint32_t comms_frames_status (uint8_t * p_buf, uint32_t seq, uint32_t up_s,
                              uint32_t rssi_neg, uint32_t mode, uint32_t faults,
                              uint32_t tx_drop);
uint32_t comms_frames_fault (uint8_t * p_buf, uint32_t t_ms, uint32_t raised);
uint32_t comms_frames_motion (uint8_t * p_buf, uint32_t t_ms, int32_t vel_left,
                              int32_t vel_right, int32_t dist_cm,
                              uint32_t owner, uint32_t enc_ok);
/* enc_ok: bit 0 = left encoder healthy, bit 1 = right (12 Nov: was one
   bit for both, so a fault didn't say which wheel) */
uint32_t comms_frames_line (uint8_t * p_buf, uint32_t t_ms, uint32_t mode,
                            int32_t pos, uint32_t see, char bc_char,
                            uint32_t bc_action, int32_t speed_mms,
                            int32_t bend_pct, int32_t kp_x1000, uint32_t pivot);
/* robot/cmd tuning form (2..5 bytes, byte 0 = 0):
   0, speed cm/s, bend %, KP/100, pivot (1 off, 2 on).  Returns 1 if this
   is one.  Missing or 0 fields come back as 0 (= unchanged). */
uint32_t comms_frames_cmd_tune (const uint8_t * p_buf, uint32_t len,
                                int32_t * p_mms, int32_t * p_bend,
                                int32_t * p_kp_val, int32_t * p_pivot);
uint32_t comms_frames_terrain (uint8_t * p_buf, uint32_t t_ms,
                               int32_t pitch_d10, int32_t roll_d10,
                               int32_t hump_mm, int32_t heading,
                               uint32_t events);
uint32_t comms_frames_obstacle (uint8_t * p_buf, uint32_t t_ms, int32_t dist,
                                int32_t bearing, int32_t width, int32_t need_l,
                                int32_t need_r, uint32_t action,
                                uint32_t stage);
typedef struct
{
    uint32_t cpu0;
    uint32_t cpu1;
    uint32_t tasks;
    uint32_t tasks_max;
    uint32_t flags;
    uint32_t flags_max;
    uint32_t sems;
    uint32_t sems_max;
    uint32_t mbfs;
    uint32_t mbfs_max;
    uint32_t cycs;
    uint32_t cycs_max;
    uint32_t queue_pct;
    uint32_t drops;
    uint32_t motion_us;
    uint32_t line_us;
} fr_res_t;
uint32_t comms_frames_resources (uint8_t * p_buf, uint32_t t_ms,
                                 const fr_res_t * p_res);

/* robot/cmd: returns code (1..7) or 0 if invalid; *id = 0..31 */
uint32_t comms_frames_cmd_decode (const uint8_t * p_buf, uint32_t len,
                                  uint32_t * p_ident);

/* robot/map: fills p_act[FR_MAP_LETTERS] and returns 1 if p_buf is
   exactly FR_MAP_BYTES with every action 0..FR_MAP_ACT_MAX; else 0 and
   p_act is untouched. */
uint32_t comms_frames_map_decode (const uint8_t * p_buf, uint32_t len,
                                  uint8_t * p_act);
/* robot/tlm/map: the map in use + flags (bit 0 = received) */
uint32_t comms_frames_map_ack (uint8_t * p_buf, const uint8_t * p_act,
                               uint32_t received);

#endif

/*** end of file ***/
