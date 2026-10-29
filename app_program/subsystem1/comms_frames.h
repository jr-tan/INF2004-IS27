/*
 *	comms_frames.h
 *	Subsystem 1 -- application data fields and binary MQTT frame codecs.
 *
 *	CHANGE REQUIREMENT 11 (minimum-bit data representation): every field uses
 *	only as many bits as its valid range needs.  The widths are named
 *	constants here so the "data representation table" in docs/ and the code
 *	cannot drift apart.  Frames are built with bitpack.h, never with C
 *	bit-fields (BARR-C 5.5), so the layout is compiler independent.
 *
 *	Pure C, no kernel dependency -- unit-tested on the PC.
 */

#ifndef COMMS_FRAMES_H
#define COMMS_FRAMES_H

#include <stdint.h>
#include <stdbool.h>
#include "comms_port.h"

/* ------------------------------------------------------------------ */
/*  System mode: 9 values (0..8)  ->  4 bits                           */
/* ------------------------------------------------------------------ */
#define SYS_MODE_BITS		(4U)
#define SYS_MODE_INIT		(0U)
#define SYS_MODE_IDLE		(1U)
#define SYS_MODE_LINE		(2U)
#define SYS_MODE_BARCODE	(3U)
#define SYS_MODE_AVOID		(4U)
#define SYS_MODE_RECOVERY	(5U)
#define SYS_MODE_PAUSED		(6U)
#define SYS_MODE_SAFE_STOP	(7U)
#define SYS_MODE_COMPLETE	(8U)
#define SYS_MODE_MAX		(8U)

/* ------------------------------------------------------------------ */
/*  Fault bitmask: one bit per fault condition, shared by every buddy  */
/*  (replaces separate integer variables and the old "last_fault").    */
/* ------------------------------------------------------------------ */
#define FAULT_BITS		(12U)
#define FAULT_MOTOR_STALL	(1U << 0)
#define FAULT_ENCODER		(1U << 1)
#define FAULT_IMU		(1U << 2)
#define FAULT_NO_ECHO		(1U << 3)
#define FAULT_WIFI_LOSS		(1U << 4)
#define FAULT_MQTT_LOSS		(1U << 5)	/* CONN_LOST: 3 missed heartbeats */
#define FAULT_LINE		(1U << 6)	/* LINE_FAULT   (Buddy 3) */
#define FAULT_AVOID		(1U << 7)	/* AVOID_FAULT  (Buddy 5) */
#define FAULT_MOTION		(1U << 8)	/* MOTION_FAULT (Buddy 2) */
#define FAULT_SUDDEN_IMPACT	(1U << 9)	/* Buddy 4 */
#define FAULT_COMMS		(1U << 10)	/* repeated reconnect failure */
/* bit 11 is reserved */
#define FAULT_MASK_ALL		((1U << FAULT_BITS) - 1U)

/* ------------------------------------------------------------------ */
/*  Commands (robot/cmd): code 3 bits + id 5 bits = 8 bits = 1 byte     */
/* ------------------------------------------------------------------ */
#define CMD_CODE_BITS		(3U)
#define CMD_ID_BITS		(5U)
#define CMD_NONE		(0U)	/* invalid / never sent */
#define CMD_START		(1U)
#define CMD_STOP		(2U)
#define CMD_PAUSE		(3U)
#define CMD_RESUME		(4U)
#define CMD_PING		(5U)	/* robot answers with a status frame at once */
#define CMD_CODE_MAX		(5U)
#define CMD_FRAME_BYTES		(1U)

typedef struct {
	uint8_t code;		/* CMD_* */
	uint8_t id;		/* 0..31, increments per command sent */
} cmd_frame_t;

/* ------------------------------------------------------------------ */
/*  robot/status: 48 bits = 6 bytes                                     */
/*    seq 4 | up_s 20 | rssi 7 | mode 4 | faults 12 | tx_drop 1         */
/* ------------------------------------------------------------------ */
#define ST_SEQ_BITS		(4U)
#define ST_UP_BITS		(20U)	/* uptime in seconds, wraps after 12.1 days */
#define ST_RSSI_BITS		(7U)	/* -RSSI in dBm, 1..127; 0 = unknown */
#define ST_FAULT_BITS		FAULT_BITS
#define ST_DROP_BITS		(1U)
#define ST_TOTAL_BITS		(ST_SEQ_BITS + ST_UP_BITS + ST_RSSI_BITS + \
				 SYS_MODE_BITS + ST_FAULT_BITS + ST_DROP_BITS)
#define ST_FRAME_BYTES		((ST_TOTAL_BITS + 7U) / 8U)

typedef struct {
	uint8_t  seq;		/* 0..15, wraps */
	uint32_t up_s;
	uint8_t  rssi_neg_dbm;
	uint8_t  mode;
	uint16_t faults;
	bool     tx_drop;	/* telemetry was dropped since the last status */
} status_frame_t;

/* ------------------------------------------------------------------ */
/*  robot/event/fault: 28 bits -> 4 bytes (4 pad bits)                  */
/*    t 16 (ms since boot, low 16 bits) | raised 12 | pad 4            */
/* ------------------------------------------------------------------ */
#define FE_T_BITS		(16U)
#define FE_PAD_BITS		(4U)
#define FE_TOTAL_BITS		(FE_T_BITS + FAULT_BITS + FE_PAD_BITS)
#define FE_FRAME_BYTES		((FE_TOTAL_BITS + 7U) / 8U)

typedef struct {
	uint16_t t_ms;		/* low 16 bits of the uptime in ms */
	uint16_t raised;	/* fault bits that were newly raised */
} fault_frame_t;

_Static_assert(ST_FRAME_BYTES <= COMMS_FRAME_MAX, "status frame too large");
_Static_assert(FE_FRAME_BYTES <= COMMS_FRAME_MAX, "fault frame too large");
_Static_assert(CMD_FRAME_BYTES <= COMMS_RX_MAX, "cmd frame too large");
_Static_assert(CMD_CODE_BITS + CMD_ID_BITS == 8U, "cmd must be one byte");

/* Encoders return the frame length in bytes, or 0 on error.  Decoders return
 * true only if the frame is the right length and every field is in range. */
uint16_t comms_enc_status(uint8_t *buf, uint16_t cap, const status_frame_t *f);
bool     comms_dec_status(const uint8_t *buf, uint16_t len, status_frame_t *f);
uint16_t comms_enc_fault(uint8_t *buf, uint16_t cap, const fault_frame_t *f);
bool     comms_dec_fault(const uint8_t *buf, uint16_t len, fault_frame_t *f);
uint16_t comms_enc_cmd(uint8_t *buf, uint16_t cap, const cmd_frame_t *c);
bool     comms_dec_cmd(const uint8_t *buf, uint16_t len, cmd_frame_t *c);

#endif /* COMMS_FRAMES_H */
