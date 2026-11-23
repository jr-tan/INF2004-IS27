/**
 * @file    subsystem1.c
 *  subsystem1.c  (Buddy 1: WiFi / MQTT telemetry, commands, faults, status)
 *
 *  Kernel objects (created in subsystem1_init, before any task runs):
 *    fault_flg  event flag: the shared fault bitmask (FAULT_* in car.h)
 *    mbf_pub    message buffer: frames OUT  (comms task -> MQTT client)
 *    mbf_rx     message buffer: commands IN (MQTT client -> comms task)
 *
 *  Tasks
 *    comms  (18, core 0)  every 100 ms: commands in, faults out,
 *                         status 1 Hz, telemetry 5 Hz while driving
 *    status (20, core 0)  status LED + console lines every 500 ms
 *
 *  lwIP/MQTT are NEVER called from here.  They run inside the kernel's
 *  WiFi task; mqtt_utk.c pops mbf_pub and pushes mbf_rx through the
 *  comms_port_* functions at the bottom of this file (TMO_POL only).
 *
 *  Default build (NET=none): no network, the comms task still runs
 *  and decodes nothing; frames are simply not queued.
 *  Network build: make NET=lwip MQTT=1 (fill in config/wifi_secrets.h).
 */
#include "../car.h"
#include "comms_port.h"
#include "comms_frames.h"
#include "board_io.h"
#if TM_NET_LWIP
#include "cyw43_utk.h"
#include "subsystem1.h" /* own header (BARR-C 4.3.c) */
#endif

#define PUB_BUFSZ (24U * 20U)
LOCAL ID            g_fault_flg;
LOCAL ID            g_mbf_pub;
LOCAL ID            g_mbf_rx;
LOCAL volatile BOOL gb_mqtt_up;
LOCAL volatile BOOL gb_wifi_up; /* comms task writes, status task reads */
/* raw CYW43_LINK_* (3 = up), for the console */
LOCAL volatile int32_t  g_wifi_link;
LOCAL volatile uint32_t g_pub_ok;
LOCAL volatile uint32_t g_pub_fail;
LOCAL volatile uint32_t g_tx_drops;
LOCAL volatile uint32_t g_rx_cmds;
LOCAL volatile BOOL     gb_tx_drop_flag;

/** ===================== shared fault bitmask ===================== */
EXPORT void comms_fault_raise (uint32_t bits)
{
    if (g_fault_flg > 0)
    {
        tk_set_flg(g_fault_flg, bits & FAULT_ALL);
    }
}
/**
 * @brief  Comms fault clear.
 */
EXPORT void comms_fault_clear (uint32_t bits)
{
    if (g_fault_flg > 0)
    {
        tk_clr_flg(g_fault_flg, ~bits);
    }
}
/**
 * @brief  Comms fault get.
 * @return Result as described above.
 */
EXPORT uint32_t comms_fault_get (void)
{
    T_RFLG res;
    if ((g_fault_flg <= 0) || (tk_ref_flg(g_fault_flg, &res) != E_OK))
    {
        return 0;
    }
    return res.flgptn & FAULT_ALL;
}

/** ===================== frames out ===================== */
LOCAL void publish (uint32_t topic, const uint8_t * p_frame, uint32_t len)
{
    uint8_t  buf[1 + COMMS_FRAME_MAX];
    uint32_t idx;

    if ((!gb_mqtt_up) || (len > COMMS_FRAME_MAX))
    {
        return; /* stale data is worthless */
    }
    buf[0] = (uint8_t)topic;
    for (idx = 0; idx < len; idx++)
    {
        buf[1 + idx] = p_frame[idx];
    }
    if (tk_snd_mbf(g_mbf_pub, buf, (int32_t)(len + 1U), TMO_POL) != E_OK)
    {
        g_tx_drops++;
        gb_tx_drop_flag = TRUE;
    }
}

/**
 * @brief  Send status.
 */
LOCAL void send_status (uint32_t seq)
{
    uint8_t fval[FR_STATUS_BYTES];
    comms_frames_status(fval, seq, motion_now_ms() / 1000U, 0,
                        mission_sys_mode(), comms_fault_get(), gb_tx_drop_flag);
    gb_tx_drop_flag = FALSE;
    publish(COMMS_TOPIC_STATUS, fval, FR_STATUS_BYTES);
}

/**
 * @brief  Send telemetry.
 */
LOCAL void send_telemetry (void)
{
    uint8_t         fval[COMMS_FRAME_MAX];
    motion_status_t mval;
    line_status_t   lval;
    imu_status_t    im_val;
    barcode_info_t  bc_val;
    uint32_t        tval = motion_now_ms();

    motion_get_status(&mval);
    comms_frames_motion(fval, tval, mval.tgt_mms[0], mval.tgt_mms[1],
                        motion_odometry_mm() / 10, mval.owner,
                        ((mval.healthy[0]) ? 1U : 0U) |
                            ((mval.healthy[1]) ? 2U : 0U));
    publish(COMMS_TOPIC_TLM_MOTION, fval, FR_MOTION_BYTES);

    line_get(&lval);
    barcode_get(&bc_val);
    {
        int32_t sp_val;
        int32_t bend;
        int32_t kp_val;
        line_get_tuning(&sp_val, &bend, &kp_val);
        comms_frames_line(fval, tval, lval.mode, lval.pos, line_see_bits(),
                          (bc_val.seq) ? bc_val.text[0] : 0,
                          (bc_val.seq) ? bc_val.action : 0, sp_val, bend,
                          kp_val, (line_get_pivot()) ? 1U : 0U);
    }
    publish(COMMS_TOPIC_TLM_LINE, fval, FR_LINE_BYTES);

    imu_get(&im_val);
    comms_frames_terrain(fval, tval, im_val.pitch_d10, im_val.roll_d10,
                         im_val.hump_mm, im_val.heading, im_val.events);
    publish(COMMS_TOPIC_TLM_TERRAIN, fval, FR_TERRAIN_BYTES);
}

LOCAL uint32_t g_motion_us_last;
LOCAL uint32_t g_line_us_last;

/**
 * @brief  Send resources.
 */
LOCAL void send_resources (void)
{
    uint8_t     fval[COMMS_FRAME_MAX];
    res_usage_t uval;
    fr_res_t    res;
    T_RMBF      mb_val;

    resources_usage(&uval);
    res.cpu0      = uval.cpu_pct[0];
    res.cpu1      = uval.cpu_pct[1];
    res.tasks     = uval.tasks;
    res.tasks_max = CNF_MAX_TSKID;
    res.flags     = uval.flags;
    res.flags_max = CNF_MAX_FLGID;
    res.sems      = uval.sems;
    res.sems_max  = CNF_MAX_SEMID;
    res.mbfs      = uval.mbfs;
    res.mbfs_max  = CNF_MAX_MBFID;
    res.cycs      = uval.cycs;
    res.cycs_max  = CNF_MAX_CYCID;
    res.queue_pct = (E_OK == tk_ref_mbf(g_mbf_pub, &mb_val))
                        ? 100U - ((uint32_t)(mb_val.frbufsz * 100) / PUB_BUFSZ)
                        : 0U;
    res.drops     = g_tx_drops;
    res.motion_us = g_motion_us_last = motion_loop_us_max();
    res.line_us = g_line_us_last = line_loop_us_max();
    comms_frames_resources(fval, motion_now_ms(), &res);
    publish(COMMS_TOPIC_RESOURCES, fval, FR_RESOURCES_BYTES);
}

/* robot/tlm/map: the barcode map the car is really using, so the GUI can
   show it was received.  Sent on a new map, on (re)connect and every
   MAP_ACK_S seconds. */
#define MAP_ACK_S 5U
/* comms ticks (COMMS_PERIOD_MS) between two map acknowledgements */
#define MAP_ACK_TICKS ((MAP_ACK_S * 1000U) / COMMS_PERIOD_MS)
/* tick within that period on which it is sent (spreads the frames) */
#define MAP_ACK_PHASE 7U

/**
 * @brief  Publish the barcode map in use on robot/tlm/map.
 */
LOCAL void send_map_ack (void)
{
    uint8_t fval[COMMS_FRAME_MAX];
    uint8_t act[BC_MAP_LETTERS];
    BOOL    b_from_gui = barcode_get_map(act);

    comms_frames_map_ack(fval, act, b_from_gui ? 1U : 0U);
    publish(COMMS_TOPIC_MAP_ACK, fval, FR_MAP_ACK_BYTES);
}

/**
 * @brief  Apply a barcode map received on robot/map, print it and
 *         acknowledge it.
 * @param  p_buf  Payload after the COMMS_RX_MAP_MARK byte.
 * @param  len    Payload length, bytes.
 */
LOCAL void on_map (const uint8_t * p_buf, uint32_t len)
{
    static const char letter[] = "-LRUS"; /* indexed by BC_ACT_* */
    uint8_t           act[BC_MAP_LETTERS];
    uint32_t          idx;
    uint32_t          mapped = 0U;

    if (0U == comms_frames_map_decode(p_buf, len, act))
    {
        tm_printf((uint8_t *)"COMMS barcode map rejected (%d bytes, need %d)\n",
                  (int32_t)len, FR_MAP_BYTES);
    }
    else
    {
        barcode_set_map(act);
        tm_printf((uint8_t *)"COMMS barcode map from GUI:");
        for (idx = 0U; idx < BC_MAP_LETTERS; idx++)
        {
            if (BC_ACT_NONE != act[idx])
            {
                tm_printf((uint8_t *)" %c=%c", (int32_t)('A' + idx),
                          (int32_t)letter[act[idx]]);
                mapped++;
            }
        }
        tm_printf((uint8_t *)"%s\n",
                  (mapped > 0U) ? " (rest straight)" : " all straight");
        send_map_ack();
    }
}

/** ===================== comms task ===================== */
LOCAL void comms_task (int32_t stacd, void * p_exinf)
{
    uint32_t tick          = 0;
    uint32_t seq           = 0;
    uint32_t last_faults   = 0;
    uint32_t last_obs      = 0;
    uint32_t last_bc       = 0;
    BOOL     b_mqtt_was_up = FALSE;
    uint8_t  fval[COMMS_FRAME_MAX];
    uint8_t  rx_val[COMMS_RX_MAX];
#if TM_NET_LWIP
    T_CYW43_UTK_STATUS ws_val;
    BOOL               b_wifi_was_up = FALSE;
#endif

    (void)stacd;
    (void)p_exinf;
    for (;; tick++)
    {
        int32_t         num;
        uint32_t        faults;
        uint32_t        stat_val = mission_state();
        obstacle_info_t ob_val;
        barcode_info_t  bc_val;

        /* commands in (robot/cmd) */
        for (;;)
        {
            /* BARR-C 8.4.b: no assignment in the loop's controlling expression.
             */
            num = tk_rcv_mbf(g_mbf_rx, rx_val, TMO_POL);
            if (num <= 0)
            {
                break;
            }
            uint32_t obj_id;
            int32_t  mms;
            int32_t  bend;
            int32_t  kp_val;
            int32_t  pivot;
            uint32_t code =
                comms_frames_cmd_decode(rx_val, (uint32_t)num, &obj_id);
            g_rx_cmds++;
            if (COMMS_RX_MAP_MARK == rx_val[0])
            {
                /* robot/map: a marker byte, then the map */
                on_map(&rx_val[1], (uint32_t)num - 1U);
            }
            else if (comms_frames_cmd_tune(rx_val, (uint32_t)num, &mms, &bend,
                                           &kp_val, &pivot))
            {
                line_set_tuning((int32_t)mms, (int32_t)bend, (int32_t)kp_val);
                if ((1 == pivot) || (2 == pivot))
                {
                    line_set_pivot(2 == pivot);
                }
                tm_printf(
                    (uint8_t *)"COMMS tuning: speed %d mm/s, bend %d %%, KP "
                               "%d, bend pivot %s\n",
                    (int32_t)mms, (int32_t)bend, (int32_t)kp_val,
                    (2 == pivot)   ? "ON"
                    : (1 == pivot) ? "OFF"
                                   : "unchanged");
            }
            else
            {
                tm_printf((uint8_t *)"COMMS cmd %d (id %d)\n", code,
                          (int32_t)obj_id);
                switch (code)
                {
                    case CMD_START:
                        mission_event(EV_CMD_START);
                    break;
                    case CMD_STOP:
                        mission_event(EV_CMD_STOP);
                    break;
                    case CMD_PAUSE:
                        mission_event(EV_CMD_PAUSE);
                    break;
                    case CMD_RESUME:
                        mission_event(EV_CMD_RESUME);
                    break;
                    case CMD_PING:
                        send_status(seq++);
                    break;
                    case CMD_UTURN:
                        mission_event(EV_CMD_UTURN);
                    break;
                    case CMD_REVERSE:
                        mission_event(EV_CMD_REVERSE);
                    break;
                    default:
                    break;
                }
            }
        }

#if TM_NET_LWIP
        /* link supervision -> fault bits */
        cyw43_utk_get_status(&ws_val);
        g_wifi_link = ws_val.link_status;
        gb_wifi_up =
            3 == ws_val.link_status; /* CYW43_LINK_UP: joined AND has an IP */
        if (ws_val.link_status >= 2)
        {
            if (!b_wifi_was_up)
            {
                tm_printf((uint8_t *)"COMMS WiFi up\n");
            }
            b_wifi_was_up = TRUE;
            comms_fault_clear(FAULT_WIFI_LOSS);
        }
        else if (b_wifi_was_up || (motion_now_ms() > 30000U))
        {
            comms_fault_raise(FAULT_WIFI_LOSS);
        }
        else
        {
            /* No action is required in this case. */
        }
        if (gb_mqtt_up)
        {
            comms_fault_clear(FAULT_MQTT_LOSS);
        }
        else if (b_wifi_was_up)
        {
            comms_fault_raise(FAULT_MQTT_LOSS);
        }
        else
        {
            /* No action is required in this case. */
        }
#endif

        /* new faults -> robot/event/fault, at once */
        faults = comms_fault_get();
        if (faults & ~last_faults)
        {
            comms_frames_fault(fval, motion_now_ms(), faults & ~last_faults);
            publish(COMMS_TOPIC_FAULT, fval, FR_FAULT_BYTES);
            tm_printf((uint8_t *)"FAULT raised 0x%03x (now 0x%03x)\n",
                      faults & ~last_faults, faults);
        }
        last_faults = faults;

        /* status 1 Hz; telemetry 5 Hz while driving, 1 Hz otherwise */
        if (0 == (tick % (1000 / COMMS_PERIOD_MS)))
        {
            send_status(seq++);
        }
        if (((MS_FOLLOW == stat_val) || (MS_AVOID == stat_val))
                ? 0 == (tick % 2)
                : 5 == (tick % (1000 / COMMS_PERIOD_MS)))
        {
            send_telemetry();
        }

        /* one-off events: every obstacle step */
        while (mission_obstacle_pop(&ob_val))
        {
            comms_frames_obstacle(
                fval, motion_now_ms(), ob_val.dist_cm, ob_val.bearing_deg,
                ob_val.width_cm, ob_val.need_left_cm, ob_val.need_right_cm,
                (ob_val.dir > 0)   ? 1U
                : (ob_val.dir < 0) ? 2U
                                   : ((OBS_BLOCKED == ob_val.stage) ? 3U : 0U),
                ob_val.stage);
            publish(COMMS_TOPIC_TLM_OBSTACLE, fval, FR_OBSTACLE_BYTES);
        }
        (void)last_obs;

        /* barcode map in use: on connect, then every MAP_ACK_S */
        if (gb_mqtt_up &&
            ((!b_mqtt_was_up) || (MAP_ACK_PHASE == (tick % MAP_ACK_TICKS))))
        {
            send_map_ack();
        }
        b_mqtt_was_up = gb_mqtt_up;

        /* resource usage, once a second */
        if (3 == (tick % (1000 / COMMS_PERIOD_MS)))
        {
            send_resources();
        }
        barcode_get(&bc_val);
        if (bc_val.seq != last_bc)
        {
            last_bc = bc_val.seq;
            send_telemetry();
        }

        if (g_tx_drops > 0)
        {
            comms_fault_raise(FAULT_COMMS);
        }
        tk_dly_tsk(COMMS_PERIOD_MS);
    }
}

/* ===================== status task: LED + console ===================== */
#define LED_STEP_MS 100

/**
 * @brief  Mode name.
 * @return Result as described above.
 */
LOCAL const char * mode_name (uint32_t mval)
{
    static const char * p_num[] = {"off",      "track", "centred",   "recover",
                                   "junction", "LOST",  "BEND STOP", "PIVOT"};
    return (mval < 8) ? p_num[mval] : "?";
}
/**
 * @brief  Owner name.
 * @return Result as described above.
 */
LOCAL const char * owner_name (uint32_t obj)
{
    return (MOT_OWNER_LINE == obj)      ? "LINE"
           : (MOT_OWNER_MISSION == obj) ? "MISSION"
                                        : "NONE";
}
/**
 * @brief  Led pattern.
 * @return Result as described above.
 */
LOCAL BOOL led_pattern (uint32_t state, uint32_t step)
{
    switch (state)
    {
        case MS_IDLE:
        case MS_PAUSED:
            return (step % 10U) < 5U; /* 1 Hz */
        case MS_FOLLOW:
        case MS_TURN:
            return TRUE;
        case MS_AVOID:
            return (step & 1U) != 0; /* 5 Hz */
        case MS_DONE:
            return 0 == (step % 20U); /* short blink */
        default:
            return (0 == (step % 10U)) || (2 == (step % 10U)); /* double */
    }
}

/**
 * @brief  Task body (micro T-Kernel task entry).
 * @param  stacd  Required by the micro T-Kernel signature; not used.
 * @param  p_exinf  Required by the micro T-Kernel signature; not used.
 */
LOCAL void status_task (int32_t stacd, void * p_exinf)
{
    uint32_t        step = 0;
    line_status_t   lval;
    motion_status_t mval;
    imu_status_t    im_val;
    barcode_info_t  bc_val;
    uint32_t        wval;
    uint32_t        raw_word; /* packed line readings */
    uint32_t        dark_word;
    BOOL            b_led_wifi = FALSE;
    uint32_t        bc_seen    = 0xFFFFFFFFU;

    (void)stacd;
    (void)p_exinf;
    /* power-up colour test: you should see red, green, blue, then pink */
    tm_printf((
        uint8_t *)"LED test: red, green, blue, pink (pink = WiFi connected)\n");
    board_io_rgb_set(RGB_RED);
    tk_dly_tsk(400);
    board_io_rgb_set(RGB_GREEN);
    tk_dly_tsk(400);
    board_io_rgb_set(RGB_BLUE);
    tk_dly_tsk(400);
    board_io_rgb_set(RGB_PINK);
    tk_dly_tsk(800);
    board_io_rgb_set(RGB_RED);
    tk_dly_tsk(100);
    resources_report();
    tm_printf(
        (uint8_t *)"\nReady. Put the car centred on the line and press GP20 = "
                   "START, GP21 = STOP.\n"
                   "(At START it checks the sensors: both on black = INSIDE "
                   "layout, both on white = STRADDLE.)\n\n");

    board_io_buzz_play(TUNE_BOOT);
    for (;; step++)
    {
        gpio_set_val(PIN_STATUS_LED,
                     (led_pattern(mission_state(), step)) ? 1 : 0);

        /* on-board RGB LEDs + buzzer: red = no WiFi, pink = WiFi up,
           a tune when it connects / drops, a beep per barcode */
        {
            BOOL b_up = gb_wifi_up || gb_mqtt_up; /* MQTT talking = connected,
                                                     whatever the link says */
            if (b_up != b_led_wifi)
            {
                b_led_wifi = b_up;
                tm_printf((uint8_t *)"LEDs %s\n",
                          b_up ? "PINK (WiFi connected)" : "RED (no WiFi)");
                if (b_up)
                {
                    board_io_buzz_play(TUNE_WIFI);
                }
                else if (step > 0)
                {
                    board_io_buzz_play(TUNE_WIFI_LOST);
                }
                else
                {
                    /* No action is required in this case. */
                }
            }
            /* pink = connected; orange = found the hotspot but no IP
               yet (link 1 or 2); red = nothing */
            if (b_up || gb_mqtt_up)
            {
                board_io_rgb_set(RGB_PINK);
            }
            else if ((1 == g_wifi_link) || (2 == g_wifi_link))
            {
                board_io_rgb_set(RGB_ORANGE);
            }
            else
            {
                board_io_rgb_set(RGB_RED);
            }
            barcode_get(&bc_val);
            if (bc_val.seq != bc_seen)
            {
                if (bc_seen != 0xFFFFFFFFU)
                {
                    board_io_buzz_play((bc_val.action != BC_ACT_NONE)
                                           ? TUNE_BC_ACTION
                                           : TUNE_BARCODE);
                }
                bc_seen = bc_val.seq;
            }
            board_io_buzz_step(LED_STEP_MS);
        }

        if (0 == (step % (STATUS_PERIOD_MS / LED_STEP_MS)))
        {
            line_get(&lval);
            raw_word  = line_raw_word(); /* one store: a consistent pair */
            dark_word = line_dark_word();
            motion_get_status(&mval);
            imu_get(&im_val);
            barcode_get(&bc_val);
            wval = ranger_word();
            tm_printf(
                (uint8_t *)"[%6d] %s | IR L %4d/%4d R %4d/%4d pos %5d steer "
                           "%5d sp %3d %s"
                           " | US %3d cm @%3d%s\n",
                (int32_t)(motion_now_ms() / 100U), mission_text(),
                (int32_t)LINE_WORD_LEFT(raw_word),
                (int32_t)LINE_WORD_LEFT(dark_word),
                (int32_t)LINE_WORD_RIGHT(raw_word),
                (int32_t)LINE_WORD_RIGHT(dark_word), lval.pos, lval.steer,
                lval.speed, mode_name(lval.mode), RW_CM(wval), RW_DEG(wval),
                RW_VALID(wval) ? "" : " (invalid)");
            tm_printf(
                (uint8_t *)"         motor %s L %4d mm/s %3d pm trim %4d | R "
                           "%4d "
                           "mm/s %3d pm trim %4d%s"
                           " | enc L %s %d (v %d) R %s %d (v %d) noise %d/%d | "
                           "odo "
                           "%d cm head %d (0.1 deg)%s\n",
                owner_name(mval.owner), mval.tgt_mms[0], mval.duty_pm[0],
                mval.trim_pm[0], mval.tgt_mms[1], mval.duty_pm[1],
                mval.trim_pm[1], (mval.trim_on) ? " LEARN" : "",
                (mval.healthy[0]) ? "ok" : "DEAD", (int32_t)mval.count[0],
                mval.v_mms[0], (mval.healthy[1]) ? "ok" : "DEAD",
                (int32_t)mval.count[1], mval.v_mms[1], (int32_t)mval.glitch[0],
                (int32_t)mval.glitch[1], (int32_t)(motion_odometry_mm() / 10),
                (int32_t)motion_heading_d10(),
                (motion_heading_ok()) ? "" : " (encoder fault: not trusted)");
            tm_printf(
                (uint8_t *)"         imu %s a %5d %5d %5d mg pitch %4d roll "
                           "%4d (0.1 deg) head %3d"
                           " humps %d (last %d mm) shock %d mg | barcode "
                           "\"%s\" ok %d bad %d"
                           " | wifi %s (link %d) mqtt %s tx %d drop %d | "
                           "faults 0x%03x\n",
                (im_val.ok) ? "ok" : "--", im_val.ax, im_val.ay, im_val.az,
                im_val.pitch_d10, im_val.roll_d10, im_val.heading,
                (int32_t)im_val.humps, im_val.hump_mm, im_val.shock_mg,
                (bc_val.seq) ? bc_val.text : "", (int32_t)bc_val.good,
                (int32_t)bc_val.bad, gb_wifi_up ? "UP" : "down",
                (int32_t)g_wifi_link, gb_mqtt_up ? "UP" : "down",
                (int32_t)g_pub_ok, (int32_t)g_tx_drops, comms_fault_get());
        }
        tk_dly_tsk(LED_STEP_MS);
    }
}

/** ===================== init / start ===================== */
EXPORT ER subsystem1_init (void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};

    gpio_set_pin(PIN_STATUS_LED, GPIO_MODE_OUT);
    gpio_set_val(PIN_STATUS_LED, 0);
    board_io_init(); /* RGB LEDs red, buzzer silent */

    cflg.flgatr = TA_TFIFO | TA_WMUL; /* anyone may wait on faults */
    g_fault_flg = tk_cre_flg(&cflg);
    if (g_fault_flg < E_OK)
    {
        return g_fault_flg;
    }

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = PUB_BUFSZ; /* ~24 frames queued */
    cmbf.maxmsz = 1 + COMMS_FRAME_MAX;
    g_mbf_pub   = tk_cre_mbf(&cmbf);
    if (g_mbf_pub < E_OK)
    {
        return g_mbf_pub;
    }
    cmbf.bufsz  = 8 * (COMMS_RX_MAX + 4U); /* 8 messages + kernel headers */
    cmbf.maxmsz = COMMS_RX_MAX;
    g_mbf_rx    = tk_cre_mbf(&cmbf);
    if (g_mbf_rx < E_OK)
    {
        return g_mbf_rx;
    }
    return E_OK;
}

/**
 * @brief  Start the module's task(s).
 * @return Result as described above.
 */
EXPORT ER subsystem1_start (void)
{
    ID tid =
        app_main_task_start((FP)comms_task, PRI_COMMS, CORE_COMMS, "comms");
    if (tid < E_OK)
    {
        return tid;
    }
    tid =
        app_main_task_start((FP)status_task, PRI_STATUS, CORE_STATUS, "status");
    return (tid < E_OK) ? tid : E_OK;
}

/** ===================== bridge to mqtt_utk.c (WiFi task) =====================
 */
EXPORT uint16_t comms_port_tx_pop (uint8_t * p_topic, uint8_t * p_frame,
                                   uint16_t cap)
{
    uint8_t buf[1 + COMMS_FRAME_MAX];
    int32_t num = tk_rcv_mbf(g_mbf_pub, buf, TMO_POL);
    int32_t idx;

    if (num < 2)
    {
        return 0;
    }
    *p_topic = buf[0];
    num -= 1;
    if (num > (int32_t)cap)
    {
        num = cap;
    }
    for (idx = 0; idx < num; idx++)
    {
        p_frame[idx] = buf[1 + idx];
    }
    return (uint16_t)num;
}

/**
 * @brief  Comms port rx push.
 */
EXPORT void comms_port_rx_push (const uint8_t * p_frame, uint16_t len)
{
    if ((0 == len) || (len > COMMS_RX_MAX))
    {
        return;
    }
    (void)tk_snd_mbf(g_mbf_rx, (void *)p_frame, len, TMO_POL);
}

/**
 * @brief  Comms port mqtt state.
 */
EXPORT void comms_port_mqtt_state (uint8_t up)
{
    gb_mqtt_up = up ? TRUE : FALSE;
}
/**
 * @brief  Comms port publish result.
 */
EXPORT void comms_port_publish_result (uint8_t ok)
{
    if (ok)
    {
        g_pub_ok++;
    }
    else
    {
        g_pub_fail++;
    }
}

#if APP_TEST
/**
 * @brief  On-target self-test of this subsystem (built only when
 *         APP_TEST = 1).
 */
EXPORT void subsystem1_test (void)
{
}
#endif

/*** end of file ***/
