/*
 *	comms_port.h  (Buddy 1)
 *	Bridge between the MQTT client and the rest of the firmware.
 *
 *	lwIP and MQTT may only be called from the kernel's WiFi task
 *	(cyw43_utk_task).  The MQTT client (lib/liblwip/.../mqtt_utk.c) runs
 *	there and talks to us ONLY through these four functions, which use
 *	non-blocking kernel message buffers (subsystem1.c).
 *
 *	stdint only: mqtt_utk.c cannot include <tk/tkernel.h>.
 *	Same names and meaning as the team's comms_port.h, so the existing
 *	mqtt_utk.c works unchanged.
 */
#ifndef COMMS_PORT_H
#define COMMS_PORT_H

#include <stdint.h>

/* Topic ids: order must match topic_tab[] in mqtt_utk.c */
typedef enum {
	COMMS_TOPIC_TLM_MOTION   = 0,	/* robot/tlm/motion */
	COMMS_TOPIC_TLM_LINE     = 1,	/* robot/tlm/line (also barcodes) */
	COMMS_TOPIC_TLM_TERRAIN  = 2,	/* robot/tlm/terrain */
	COMMS_TOPIC_TLM_OBSTACLE = 3,	/* robot/tlm/obstacle */
	COMMS_TOPIC_STATUS       = 4,	/* robot/status (retained) */
	COMMS_TOPIC_FAULT        = 5,	/* robot/event/fault (QoS 1) */
	COMMS_TOPIC_RESOURCES    = 6,	/* robot/tlm/resources */
	COMMS_TOPIC_MAP_ACK      = 7,	/* robot/tlm/map (barcode map in use) */
	COMMS_TOPIC_COUNT        = 8
} comms_topic_t;

#define COMMS_FRAME_MAX		(16U)	/* largest payload, bytes (resources = 15) */
#define COMMS_RX_MAX		(12U)	/* largest inbound: marker + 10-byte map */
#define COMMS_RX_MAP_MARK	(0xFFU)	/* rx[0] of a robot/map payload (never a valid cmd) */

uint16_t comms_port_tx_pop(uint8_t *topic, uint8_t *frame, uint16_t cap);
void comms_port_rx_push(const uint8_t *frame, uint16_t len);
void comms_port_mqtt_state(uint8_t up);
void comms_port_publish_result(uint8_t ok);

#endif
