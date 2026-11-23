/*
 *	subsystem1.c  (Buddy 1: WiFi / MQTT telemetry, commands, faults, status)
 *
 *	Kernel objects (created in subsystem1_init, before any task runs):
 *	  fault_flg  event flag: the shared fault bitmask (FAULT_* in car.h)
 *	  mbf_pub    message buffer: frames OUT  (comms task -> MQTT client)
 *	  mbf_rx     message buffer: commands IN (MQTT client -> comms task)
 *
 *	Tasks
 *	  comms  (18, core 0)  every 100 ms: commands in, faults out,
 *	                       status 1 Hz, telemetry 5 Hz while driving
 *	  status (20, core 0)  status LED + console lines every 500 ms
 *
 *	lwIP/MQTT are NEVER called from here.  They run inside the kernel's
 *	WiFi task; mqtt_utk.c pops mbf_pub and pushes mbf_rx through the
 *	comms_port_* functions at the bottom of this file (TMO_POL only).
 *
 *	Default build (NET=none): no network, the comms task still runs
 *	and decodes nothing; frames are simply not queued.
 *	Network build: make NET=lwip MQTT=1 (fill in config/wifi_secrets.h).
 */
#include "../car.h"
#include "comms_port.h"
#include "comms_frames.h"
#include "board_io.h"
#if TM_NET_LWIP
#include "cyw43_utk.h"
#endif

#define PUB_BUFSZ	(24U * 20U)
LOCAL ID fault_flg;
LOCAL ID mbf_pub, mbf_rx;
LOCAL volatile BOOL mqtt_up;
LOCAL volatile BOOL wifi_up;		/* comms task writes, status task reads */
LOCAL volatile INT wifi_link;		/* raw CYW43_LINK_* (3 = up), for the console */
LOCAL volatile UW pub_ok, pub_fail, tx_drops, rx_cmds;
LOCAL volatile BOOL tx_drop_flag;

/* ===================== shared fault bitmask ===================== */
EXPORT void fault_raise(UINT bits) { if (fault_flg > 0) tk_set_flg(fault_flg, bits & FAULT_ALL); }
EXPORT void fault_clear(UINT bits) { if (fault_flg > 0) tk_clr_flg(fault_flg, ~bits); }
EXPORT UINT fault_get(void)
{
	T_RFLG r;
	if (fault_flg <= 0 || tk_ref_flg(fault_flg, &r) != E_OK) return 0;
	return r.flgptn & FAULT_ALL;
}

/* ===================== frames out ===================== */
LOCAL void publish(UINT topic, const UB *frame, UINT len)
{
	UB buf[1 + COMMS_FRAME_MAX];
	UINT i;

	if (!mqtt_up || len > COMMS_FRAME_MAX) return;	/* stale data is worthless */
	buf[0] = (UB)topic;
	for (i = 0; i < len; i++) buf[1 + i] = frame[i];
	if (tk_snd_mbf(mbf_pub, buf, (INT)(len + 1U), TMO_POL) != E_OK) {
		tx_drops++;
		tx_drop_flag = TRUE;
	}
}

LOCAL void send_status(UINT seq)
{
	UB f[FR_STATUS_BYTES];
	fr_status(f, seq, now_ms() / 1000U, 0, mission_sys_mode(), fault_get(), tx_drop_flag);
	tx_drop_flag = FALSE;
	publish(COMMS_TOPIC_STATUS, f, FR_STATUS_BYTES);
}

LOCAL void send_telemetry(void)
{
	UB f[COMMS_FRAME_MAX];
	MOTION_STATUS m;
	LINE_STATUS l;
	IMU_STATUS im;
	BARCODE_INFO bc;
	UW t = now_ms();

	motion_get_status(&m);
	fr_motion(f, t, m.tgt_mms[0], m.tgt_mms[1], motion_odometry_mm() / 10,
		  m.owner, (m.healthy[0] ? 1U : 0U) | (m.healthy[1] ? 2U : 0U));
	publish(COMMS_TOPIC_TLM_MOTION, f, FR_MOTION_BYTES);

	line_get(&l);
	barcode_get(&bc);
	{
		INT sp, bend, kp;
		line_get_tuning(&sp, &bend, &kp);
		fr_line(f, t, l.mode, l.pos, (l.see[0] ? 1U : 0U) | (l.see[1] ? 2U : 0U),
			bc.seq ? bc.text[0] : 0, bc.seq ? bc.action : 0, sp, bend, kp,
			line_get_pivot() ? 1U : 0U);
	}
	publish(COMMS_TOPIC_TLM_LINE, f, FR_LINE_BYTES);

	imu_get(&im);
	fr_terrain(f, t, im.pitch_d10, im.roll_d10, im.hump_mm, im.heading, im.events);
	publish(COMMS_TOPIC_TLM_TERRAIN, f, FR_TERRAIN_BYTES);
}

LOCAL UW motion_us_last, line_us_last;

LOCAL void send_resources(void)
{
	UB f[COMMS_FRAME_MAX];
	RES_USAGE u;
	FR_RES r;
	T_RMBF mb;

	resources_usage(&u);
	r.cpu0 = u.cpu_pct[0];
	r.cpu1 = u.cpu_pct[1];
	r.tasks = u.tasks;  r.tasks_max = CNF_MAX_TSKID;
	r.flags = u.flags;  r.flags_max = CNF_MAX_FLGID;
	r.sems  = u.sems;   r.sems_max  = CNF_MAX_SEMID;
	r.mbfs  = u.mbfs;   r.mbfs_max  = CNF_MAX_MBFID;
	r.cycs  = u.cycs;   r.cycs_max  = CNF_MAX_CYCID;
	r.queue_pct = (tk_ref_mbf(mbf_pub, &mb) == E_OK)
		? 100U - (UW)(mb.frbufsz * 100) / PUB_BUFSZ : 0U;
	r.drops = tx_drops;
	r.motion_us = motion_us_last = motion_loop_us_max();
	r.line_us = line_us_last = line_loop_us_max();
	fr_resources(f, now_ms(), &r);
	publish(COMMS_TOPIC_RESOURCES, f, FR_RESOURCES_BYTES);
}

/* robot/tlm/map: the barcode map the car is really using, so the GUI
   can show it was received.  Sent on a new map, on (re)connect and every
   MAP_ACK_S seconds. */
#define MAP_ACK_S	5U
LOCAL void send_map_ack(void)
{
	UB f[COMMS_FRAME_MAX], act[BC_MAP_LETTERS];
	BOOL got = barcode_get_map(act);
	fr_map_ack(f, act, got ? 1U : 0U);
	publish(COMMS_TOPIC_MAP_ACK, f, FR_MAP_ACK_BYTES);
}

LOCAL void on_map(const UB *b, UINT len)
{
	static const char letter[] = "-LRUS";
	UB act[BC_MAP_LETTERS];
	UINT i, k = 0;

	if (!fr_map_decode(b, len, act)) {
		tm_printf((UB*)"COMMS barcode map rejected (%d bytes, need %d)\n", (INT)len, FR_MAP_BYTES);
		return;
	}
	barcode_set_map(act);
	tm_printf((UB*)"COMMS barcode map from GUI:");
	for (i = 0; i < BC_MAP_LETTERS; i++) {
		if (act[i] != BC_ACT_NONE) { tm_printf((UB*)" %c=%c", (INT)('A' + i), (INT)letter[act[i]]); k++; }
	}
	tm_printf((UB*)"%s\n", k ? " (rest straight)" : " all straight");
	send_map_ack();
}

/* ===================== comms task ===================== */
LOCAL void comms_task(INT stacd, void *exinf)
{
	UINT tick = 0, seq = 0, last_faults = 0, last_obs = 0, last_bc = 0;
	BOOL mqtt_was_up = FALSE;
	UB f[COMMS_FRAME_MAX], rx[COMMS_RX_MAX];
#if TM_NET_LWIP
	T_CYW43_UTK_STATUS ws;
	BOOL wifi_was_up = FALSE;
#endif

	(void)stacd; (void)exinf;
	for (;; tick++) {
		INT n;
		UINT faults, st = mission_state();
		OBSTACLE_INFO ob;
		BARCODE_INFO bc;

		/* commands in (robot/cmd) */
		while ((n = tk_rcv_mbf(mbf_rx, rx, TMO_POL)) > 0) {
			UW id;
			int32_t mms, bend, kp, pivot;
			UINT code = fr_cmd_decode(rx, (UW)n, &id);
			rx_cmds++;
			if (n >= 1 && rx[0] == COMMS_RX_MAP_MARK) {	/* robot/map */
				on_map(&rx[1], (UINT)n - 1U);
				continue;
			}
			if (fr_cmd_tune(rx, (UW)n, &mms, &bend, &kp, &pivot)) {
				line_set_tuning((INT)mms, (INT)bend, (INT)kp);
				if (pivot == 1 || pivot == 2) line_set_pivot(pivot == 2);
				tm_printf((UB*)"COMMS tuning: speed %d mm/s, bend %d %%, KP %d, bend pivot %s\n",
					  (INT)mms, (INT)bend, (INT)kp,
					  pivot == 2 ? "ON" : pivot == 1 ? "OFF" : "unchanged");
				continue;
			}
			tm_printf((UB*)"COMMS cmd %d (id %d)\n", code, (INT)id);
			switch (code) {
			case CMD_START:  mission_event(EV_CMD_START);  break;
			case CMD_STOP:   mission_event(EV_CMD_STOP);   break;
			case CMD_PAUSE:  mission_event(EV_CMD_PAUSE);  break;
			case CMD_RESUME: mission_event(EV_CMD_RESUME); break;
			case CMD_PING:   send_status(seq++);           break;
			case CMD_UTURN:  mission_event(EV_CMD_UTURN);  break;
			case CMD_REVERSE: mission_event(EV_CMD_REVERSE); break;
			default: break;
			}
		}

#if TM_NET_LWIP
		/* link supervision -> fault bits */
		cyw43_utk_get_status(&ws);
		wifi_link = ws.link_status;
		wifi_up = ws.link_status == 3;		/* CYW43_LINK_UP: joined AND has an IP */
		if (ws.link_status >= 2) {
			if (!wifi_was_up) tm_printf((UB*)"COMMS WiFi up\n");
			wifi_was_up = TRUE;
			fault_clear(FAULT_WIFI_LOSS);
		} else if (wifi_was_up || now_ms() > 30000U) {
			fault_raise(FAULT_WIFI_LOSS);
		}
		if (mqtt_up) fault_clear(FAULT_MQTT_LOSS);
		else if (wifi_was_up) fault_raise(FAULT_MQTT_LOSS);
#endif

		/* new faults -> robot/event/fault, at once */
		faults = fault_get();
		if (faults & ~last_faults) {
			fr_fault(f, now_ms(), faults & ~last_faults);
			publish(COMMS_TOPIC_FAULT, f, FR_FAULT_BYTES);
			tm_printf((UB*)"FAULT raised 0x%03x (now 0x%03x)\n", faults & ~last_faults, faults);
		}
		last_faults = faults;

		/* status 1 Hz; telemetry 5 Hz while driving, 1 Hz otherwise */
		if ((tick % (1000 / COMMS_PERIOD_MS)) == 0) send_status(seq++);
		if ((st == MS_FOLLOW || st == MS_AVOID) ? (tick % 2) == 0
							: (tick % (1000 / COMMS_PERIOD_MS)) == 5)
			send_telemetry();

		/* one-off events: every obstacle step */
		while (mission_obstacle_pop(&ob)) {
			fr_obstacle(f, now_ms(), ob.dist_cm, ob.bearing_deg, ob.width_cm,
				    ob.need_left_cm, ob.need_right_cm,
				    ob.dir > 0 ? 1U : ob.dir < 0 ? 2U : (ob.stage == OBS_BLOCKED ? 3U : 0U),
				    ob.stage);
			publish(COMMS_TOPIC_TLM_OBSTACLE, f, FR_OBSTACLE_BYTES);
		}
		(void)last_obs;

		/* barcode map in use: on connect, then every MAP_ACK_S */
		if (mqtt_up && (!mqtt_was_up || (tick % (MAP_ACK_S * 1000U / COMMS_PERIOD_MS)) == 7))
			send_map_ack();
		mqtt_was_up = mqtt_up;

		/* resource usage, once a second */
		if ((tick % (1000 / COMMS_PERIOD_MS)) == 3) send_resources();
		barcode_get(&bc);
		if (bc.seq != last_bc) { last_bc = bc.seq; send_telemetry(); }

		if (tx_drops > 0) fault_raise(FAULT_COMMS);
		tk_dly_tsk(COMMS_PERIOD_MS);
	}
}

/* ===================== status task: LED + console ===================== */
#define LED_STEP_MS	100

LOCAL const char *mode_name(UINT m)
{
	static const char *n[] = { "off", "track", "centred", "recover", "junction", "LOST",
				   "BEND STOP", "PIVOT" };
	return m < 8 ? n[m] : "?";
}
LOCAL const char *owner_name(UINT o)
{
	return o == MOT_OWNER_LINE ? "LINE" : o == MOT_OWNER_MISSION ? "MISSION" : "NONE";
}
LOCAL BOOL led_pattern(UINT state, UINT step)
{
	switch (state) {
	case MS_IDLE: case MS_PAUSED: return (step % 10U) < 5U;	/* 1 Hz */
	case MS_FOLLOW: case MS_TURN: return TRUE;
	case MS_AVOID:  return (step & 1U) != 0;		/* 5 Hz */
	case MS_DONE:   return (step % 20U) == 0;		/* short blink */
	default:        return (step % 10U) == 0 || (step % 10U) == 2;	/* double */
	}
}

LOCAL void status_task(INT stacd, void *exinf)
{
	UINT step = 0;
	LINE_STATUS l;
	MOTION_STATUS m;
	IMU_STATUS im;
	BARCODE_INFO bc;
	UW w;
	BOOL led_wifi = FALSE;
	UINT bc_seen = 0xFFFFFFFFU;

	(void)stacd; (void)exinf;
	/* power-up colour test: you should see red, green, blue, then pink */
	tm_printf((UB*)"LED test: red, green, blue, pink (pink = WiFi connected)\n");
	rgb_set(RGB_RED);   tk_dly_tsk(400);
	rgb_set(RGB_GREEN); tk_dly_tsk(400);
	rgb_set(RGB_BLUE);  tk_dly_tsk(400);
	rgb_set(RGB_PINK);  tk_dly_tsk(800);
	rgb_set(RGB_RED);
	tk_dly_tsk(100);
	resources_report();
	tm_printf((UB*)"\nReady. Put the car centred on the line and press GP20 = START, GP21 = STOP.\n"
		  "(At START it checks the sensors: both on black = INSIDE layout, both on white = STRADDLE.)\n\n");

	buzz_play(TUNE_BOOT);
	for (;; step++) {
		gpio_set_val(PIN_STATUS_LED, led_pattern(mission_state(), step) ? 1 : 0);

		/* on-board RGB LEDs + buzzer: red = no WiFi, pink = WiFi up,
		   a tune when it connects / drops, a beep per barcode */
		{
			BOOL up = wifi_up || mqtt_up;	/* MQTT talking = connected, whatever the link says */
			if (up != led_wifi) {
				led_wifi = up;
				tm_printf((UB*)"LEDs %s\n", up ? "PINK (WiFi connected)" : "RED (no WiFi)");
				if (up) buzz_play(TUNE_WIFI);
				else if (step > 0) buzz_play(TUNE_WIFI_LOST);
			}
			/* pink = connected; orange = found the hotspot but no IP
			   yet (link 1 or 2); red = nothing */
			if (up || mqtt_up) rgb_set(RGB_PINK);
			else if (wifi_link == 1 || wifi_link == 2) rgb_set(RGB_ORANGE);
			else rgb_set(RGB_RED);
			barcode_get(&bc);
			if (bc.seq != bc_seen) {
				if (bc_seen != 0xFFFFFFFFU)
					buzz_play(bc.action != BC_ACT_NONE ? TUNE_BC_ACTION : TUNE_BARCODE);
				bc_seen = bc.seq;
			}
			buzz_step(LED_STEP_MS);
		}

		if ((step % (STATUS_PERIOD_MS / LED_STEP_MS)) == 0) {
			line_get(&l);
			motion_get_status(&m);
			imu_get(&im);
			barcode_get(&bc);
			w = ranger_word();
			tm_printf((UB*)"[%6d] %s | IR L %4d/%4d R %4d/%4d pos %5d steer %5d sp %3d %s"
				  " | US %3d cm @%3d%s\n",
				  (INT)(now_ms() / 100U), mission_text(),
				  (INT)l.raw[0], (INT)l.dark[0], (INT)l.raw[1], (INT)l.dark[1],
				  l.pos, l.steer, l.speed, mode_name(l.mode),
				  RW_CM(w), RW_DEG(w), RW_VALID(w) ? "" : " (invalid)");
			tm_printf((UB*)"         motor %s L %4d mm/s %3d pm trim %4d | R %4d mm/s %3d pm trim %4d%s"
				  " | enc L %s %d (v %d) R %s %d (v %d) noise %d/%d | odo %d cm head %d (0.1 deg)%s\n",
				  owner_name(m.owner),
				  m.tgt_mms[0], m.duty_pm[0], m.trim_pm[0],
				  m.tgt_mms[1], m.duty_pm[1], m.trim_pm[1], m.trim_on ? " LEARN" : "",
				  m.healthy[0] ? "ok" : "DEAD", (INT)m.count[0], m.v_mms[0],
				  m.healthy[1] ? "ok" : "DEAD", (INT)m.count[1], m.v_mms[1],
				  (INT)m.glitch[0], (INT)m.glitch[1], (INT)(motion_odometry_mm() / 10),
				  (INT)motion_heading_d10(), motion_heading_ok() ? "" : " (encoder fault: not trusted)");
			tm_printf((UB*)"         imu %s a %5d %5d %5d mg pitch %4d roll %4d (0.1 deg) head %3d"
				  " humps %d (last %d mm) shock %d mg | barcode \"%s\" ok %d bad %d"
				  " | wifi %s (link %d) mqtt %s tx %d drop %d | faults 0x%03x\n",
				  im.ok ? "ok" : "--", im.ax, im.ay, im.az, im.pitch_d10, im.roll_d10,
				  im.heading, (INT)im.humps, im.hump_mm, im.shock_mg,
				  bc.seq ? bc.text : "", (INT)bc.good, (INT)bc.bad,
				  wifi_up ? "UP" : "down", (INT)wifi_link, mqtt_up ? "UP" : "down", (INT)pub_ok, (INT)tx_drops, fault_get());
		}
		tk_dly_tsk(LED_STEP_MS);
	}
}

/* ===================== init / start ===================== */
EXPORT ER subsystem1_init(void)
{
	T_CFLG cflg = {0};
	T_CMBF cmbf = {0};

	gpio_set_pin(PIN_STATUS_LED, GPIO_MODE_OUT);
	gpio_set_val(PIN_STATUS_LED, 0);
	board_io_init();			/* RGB LEDs red, buzzer silent */

	cflg.flgatr = TA_TFIFO | TA_WMUL;	/* anyone may wait on faults */
	if ((fault_flg = tk_cre_flg(&cflg)) < E_OK) return fault_flg;

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = PUB_BUFSZ;		/* ~24 frames queued */
	cmbf.maxmsz = 1 + COMMS_FRAME_MAX;
	if ((mbf_pub = tk_cre_mbf(&cmbf)) < E_OK) return mbf_pub;
	cmbf.bufsz  = 8 * (COMMS_RX_MAX + 4U);
	cmbf.maxmsz = COMMS_RX_MAX;
	if ((mbf_rx = tk_cre_mbf(&cmbf)) < E_OK) return mbf_rx;
	return E_OK;
}

EXPORT ER subsystem1_start(void)
{
	ID tid = app_start_task((FP)comms_task, PRI_COMMS, CORE_COMMS, "comms");
	if (tid < E_OK) return tid;
	tid = app_start_task((FP)status_task, PRI_STATUS, CORE_STATUS, "status");
	return tid < E_OK ? tid : E_OK;
}

/* ===================== bridge to mqtt_utk.c (WiFi task) ===================== */
EXPORT uint16_t comms_port_tx_pop(uint8_t *topic, uint8_t *frame, uint16_t cap)
{
	UB buf[1 + COMMS_FRAME_MAX];
	INT n = tk_rcv_mbf(mbf_pub, buf, TMO_POL);
	INT i;

	if (n < 2) return 0;
	*topic = buf[0];
	n -= 1;
	if (n > (INT)cap) n = cap;
	for (i = 0; i < n; i++) frame[i] = buf[1 + i];
	return (uint16_t)n;
}

EXPORT void comms_port_rx_push(const uint8_t *frame, uint16_t len)
{
	if (len == 0 || len > COMMS_RX_MAX) return;
	(void)tk_snd_mbf(mbf_rx, (void *)frame, len, TMO_POL);
}

EXPORT void comms_port_mqtt_state(uint8_t up)    { mqtt_up = up ? TRUE : FALSE; }
EXPORT void comms_port_publish_result(uint8_t ok) { if (ok) pub_ok++; else pub_fail++; }

#if APP_TEST
EXPORT void subsystem1_test(void) { }
#endif
