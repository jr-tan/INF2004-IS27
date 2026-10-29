/*
 *	comms_port.h
 *	Subsystem 1 -- bridge between the network stack and the rest of the firmware.
 *
 *	WHY THIS EXISTS.  TEAM.md section 3 forbids any student task from calling
 *	lwIP or MQTT functions: they must run inside the kernel's WiFi service
 *	task (cyw43_utk_task).  The MQTT client (lib/liblwip/.../mqtt_utk.c) runs
 *	there, and it talks to subsystem 1 only through the four functions below.
 *	They are implemented in comms.c using kernel objects, never block, and
 *	are called ONLY from cyw43_utk_task.
 *
 *	This header must stay free of kernel and lwIP types (stdint only), because
 *	mqtt_utk.c cannot include <tk/tkernel.h> (see its header comment).
 */

#ifndef COMMS_PORT_H
#define COMMS_PORT_H

#include <stdint.h>

/* MQTT topics this robot publishes.  The id travels in the first byte of every
 * queued frame; mqtt_utk.c maps it to the topic string, QoS and retain flag. */
typedef enum {
	COMMS_TOPIC_TLM_MOTION   = 0,
	COMMS_TOPIC_TLM_LINE     = 1,
	COMMS_TOPIC_TLM_TERRAIN  = 2,
	COMMS_TOPIC_TLM_OBSTACLE = 3,
	COMMS_TOPIC_STATUS       = 4,
	COMMS_TOPIC_FAULT        = 5,
	COMMS_TOPIC_COUNT        = 6
} comms_topic_t;

/* Largest binary frame (payload only) in bytes.  The biggest planned frame
 * (obstacle) is 9 bytes; see docs/buddy1_comms.md. */
#define COMMS_FRAME_MAX		(12U)

/* Inbound command frames are 1 byte; a little slack is allowed and rejected
 * later by the decoder. */
#define COMMS_RX_MAX		(8U)

/* Pop the oldest queued outgoing frame.  Returns the payload length (0 = queue
 * empty).  *topic receives a comms_topic_t value. */
uint16_t comms_port_tx_pop(uint8_t *topic, uint8_t *frame, uint16_t cap);

/* Hand one complete inbound robot/cmd payload to cmd_dispatch_task. */
void comms_port_rx_push(const uint8_t *frame, uint16_t len);

/* MQTT session came up (1) or went down (0). */
void comms_port_mqtt_state(uint8_t up);

/* Result of one mqtt_publish() call: 1 = accepted by the stack, 0 = failed. */
void comms_port_publish_result(uint8_t ok);

#endif /* COMMS_PORT_H */
