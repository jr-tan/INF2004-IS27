/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the publish pump below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	BUDDY 1 CHANGES (WiFi + telemetry subsystem):
 *	  - the demo "hb N" text heartbeat is replaced by a PUMP: every
 *	    COMMS_PUMP_MS it takes frames that application tasks queued with
 *	    comms_publish() and publishes them as BINARY payloads (Change
 *	    Requirement 11: no text values) on the robot/... topics;
 *	  - the client subscribes to robot/cmd and hands each complete payload to
 *	    cmd_dispatch_task;
 *	  - reconnect uses 1 s, 2 s, 4 s ... capped at 10 s backoff (NFR-B1-3).
 *	All hand-over to the rest of the firmware goes through comms_port.h
 *	(non-blocking kernel objects), never through direct calls into tasks.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "comms_port.h"
#include <tm/tmonitor.h>

/* Topic root.  Override MQTT_TOPIC_ROOT in config/wifi_secrets.h when several
 * robots share one broker (e.g. "robot/team27"). */
#ifndef MQTT_TOPIC_ROOT
#define MQTT_TOPIC_ROOT		"robot"
#endif

/* Set to 0 to silence the per-frame console trace once the link is proven. */
#ifndef COMMS_DEBUG
#define COMMS_DEBUG		1
#endif

#define MQTT_UTK_KEEPALIVE_S	10	/* short, so a dead broker/link is noticed in ~15 s */
#define MQTT_UTK_RETRY_MIN_MS	1000	/* backoff: 1 s, 2 s, 4 s, 8 s, then 10 s (NFR-B1-3) */
#define MQTT_UTK_RETRY_MAX_MS	10000
#define COMMS_PUMP_MS		10	/* publish pump period; keeps state-change latency << 100 ms */
#define COMMS_PUMP_BURST	4	/* frames taken per pump call, bounds time spent in one call */

#define MQTT_UTK_RX_TOPIC_MAX	64
#define MQTT_UTK_RX_DATA_MAX	COMMS_RX_MAX

#define MQTT_CMD_TOPIC		MQTT_TOPIC_ROOT "/cmd"

/* One row per comms_topic_t, in the same order. */
static const struct {
	const char	*name;
	u8_t		qos;
	u8_t		retain;
} topic_tab[COMMS_TOPIC_COUNT] = {
	{ MQTT_TOPIC_ROOT "/tlm/motion",   0, 0 },	/* COMMS_TOPIC_TLM_MOTION   */
	{ MQTT_TOPIC_ROOT "/tlm/line",     0, 0 },	/* COMMS_TOPIC_TLM_LINE     */
	{ MQTT_TOPIC_ROOT "/tlm/terrain",  0, 0 },	/* COMMS_TOPIC_TLM_TERRAIN  */
	{ MQTT_TOPIC_ROOT "/tlm/obstacle", 0, 0 },	/* COMMS_TOPIC_TLM_OBSTACLE */
	{ MQTT_TOPIC_ROOT "/status",       0, 1 },	/* COMMS_TOPIC_STATUS (retained) */
	{ MQTT_TOPIC_ROOT "/event/fault",  1, 0 },	/* COMMS_TOPIC_FAULT (QoS 1) */
};

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;
static u8_t		pump_armed;
static u32_t		retry_ms = MQTT_UTK_RETRY_MIN_MS;

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks; the payload is kept until the fragment carrying
 * MQTT_DATA_FLAG_LAST shows up. The payload is BINARY: it is never printed as
 * text and never altered. A payload longer than the buffer is discarded whole. */
static u8_t		rx_is_cmd;
static u8_t		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_overflow;

static void pump_timeout(void *arg);

static u8_t str_equal(const char *a, const char *b)
{
	while((*a != '\0') && (*a == *b)) {
		a++;
		b++;
	}
	return (*a == *b);
}

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	(void)arg;
	(void)tot_len;

	rx_is_cmd = str_equal(topic, MQTT_CMD_TOPIC);
	rx_len = 0;
	rx_overflow = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	if(!rx_is_cmd) {
		return;
	}

	for(i = 0; i < len; i++) {
		if(rx_len < MQTT_UTK_RX_DATA_MAX) {
			rx_data[rx_len++] = data[i];
		} else {
			rx_overflow = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		if(!rx_overflow) {
			comms_port_rx_push(rx_data, rx_len);
#if COMMS_DEBUG
			tm_printf((UB*)"mqtt: RX cmd %d byte(s), first=0x%02x\n",
				  (int)rx_len, (unsigned)rx_data[0]);
#endif
		} else {
			tm_printf((UB*)"mqtt: RX cmd too long, discarded\n");
		}
		rx_is_cmd = 0;
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " MQTT_CMD_TOPIC "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

/* The pump: runs every COMMS_PUMP_MS inside the network task.  While connected
 * it publishes queued frames.  While NOT connected it discards them: stale
 * telemetry is worthless and must not pile up or fire in a burst after the
 * link returns (the next status frame carries the latest state). */
static void pump_timeout(void *arg)
{
	u8_t	topic;
	u8_t	frame[COMMS_FRAME_MAX];
	u16_t	len;
	int	burst;
	u8_t	connected;

	(void)arg;

	connected = (client != NULL) && mqtt_client_is_connected(client);

	for(burst = 0; burst < COMMS_PUMP_BURST; burst++) {
		len = comms_port_tx_pop(&topic, frame, (u16_t)sizeof(frame));
		if(len == 0) {
			break;
		}
		if(!connected || (topic >= COMMS_TOPIC_COUNT)) {
			continue;	/* dropped (see above) */
		}
		{
			err_t err = mqtt_publish(client, topic_tab[topic].name, frame, len,
						 topic_tab[topic].qos, topic_tab[topic].retain,
						 NULL, NULL);
			comms_port_publish_result((err == ERR_OK)? 1: 0);
#if COMMS_DEBUG
			if(err == ERR_OK) {
				tm_printf((UB*)"mqtt: TX %s (%d bytes)\n",
					  topic_tab[topic].name, (int)len);
			} else {
				tm_printf((UB*)"mqtt: publish %s failed, err=%d\n",
					  topic_tab[topic].name, (int)err);
			}
#endif
		}
	}

	sys_timeout(COMMS_PUMP_MS, pump_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;
		retry_ms = MQTT_UTK_RETRY_MIN_MS;	/* healthy again: restart the backoff */

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Callbacks must be registered before subscribing, or the first
		 * delivery has nowhere to go. QoS 1 so a command is not lost to a
		 * single dropped packet. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, MQTT_CMD_TOPIC, 1 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		comms_port_mqtt_state(1);
	} else {
		comms_port_mqtt_state(0);

		/* mqtt_client_connect() does not retry a rejected, timed-out or dropped
		 * connection itself, so do it here -- on an exponential backoff, so a
		 * broker that keeps rejecting can't turn this into a tight loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %lu ms\n",
			  (int)status, (unsigned long)retry_ms);
		sys_timeout(retry_ms, retry_timeout, NULL);
		retry_ms *= 2;
		if(retry_ms > MQTT_UTK_RETRY_MAX_MS) {
			retry_ms = MQTT_UTK_RETRY_MAX_MS;
		}
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(!pump_armed) {
		/* One self-re-arming chain for the life of the program. */
		pump_armed = 1;
		sys_timeout(COMMS_PUMP_MS, pump_timeout, NULL);
	}

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}
