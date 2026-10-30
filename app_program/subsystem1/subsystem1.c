/*
 *	subsystem1.c
 *	Subsystem 1 -- WiFi / MQTT / telemetry: creates its objects, starts its tasks.
 *
 *	Tasks (design review section 3):
 *	  wifi_link_task     watches the link, publishes connection state
 *	  heartbeat_task     robot/status every 1 s, CONN_LOST detection
 *	  cmd_dispatch_task  validates robot/cmd frames, forwards to mission control
 *	  led_task           shows the comms state on GP19
 *	mqtt_client_task is not a separate task: the MQTT session must run inside
 *	the kernel's WiFi service task (see comms_port.h).
 *	telemetry_dispatch_task comes with the next slice, once the other buddies'
 *	telemetry record formats are agreed.
 */

#include "subsystem1.h"

EXPORT ID	tid_led;	/* IF-05, see subsystem1.h */
EXPORT ID	cyc_led;

static ID	tid_wifi;
static ID	tid_hb;
static ID	tid_cmd;

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};
	ER	er;

	er = comms_init();
	if (er < E_OK) {
		return er;
	}

	/* LED pacing: cyclic handler, created stopped, started in _start() */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;
	ccyc.cycphs = S1_TICK_MS;
	cyc_led = tk_cre_cyc(&ccyc);
	return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_cmd = app_start_task((FP)cmd_dispatch_task, S1_PRI_CMD, 0, "cmd_dispatch_task");
	if (tid_cmd < E_OK) {
		return tid_cmd;
	}
	tid_wifi = app_start_task((FP)wifi_link_task, S1_PRI_WIFI, 0, "wifi_link_task");
	if (tid_wifi < E_OK) {
		return tid_wifi;
	}
	tid_hb = app_start_task((FP)heartbeat_task, S1_PRI_HB, 0, "heartbeat_task");
	if (tid_hb < E_OK) {
		return tid_hb;
	}
	tid_led = app_start_task((FP)led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}
	return tk_sta_cyc(cyc_led);
}
