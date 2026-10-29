/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi, MQTT and telemetry communication (Buddy 1).
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 *
 *	Other buddies need only three things from here:
 *	    comms_publish()   queue a binary frame for MQTT         (non-blocking)
 *	    comms_is_up()     is the MQTT session up?               (non-blocking)
 *	    fault_raise() / fault_clear() / fault_get()   shared fault bitmask
 *	and, for mission_ctrl_task, comms_set_mode() and flg_cmd.
 *
 *	All of them are safe to call from any task on either core and never block.
 *	None may be called from an interrupt handler.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "comms_frames.h"	/* field widths, fault bits, frame codecs */

/* ---------------------------------------------------------------------
 *  Task priorities.  1 is the highest, 32 the lowest this kernel build allows
 *  (CNF_MAX_TSKPRI = 32 in config/config.h; more would need kernel bit-search
 *  support that is not enabled).  The design review (section 3.2) used
 *  numbers up to 41, which do not fit, so subsystem 1 keeps the same RELATIVE
 *  order (cmd < wifi < heartbeat < LED) inside 26..29: below every control
 *  and sensing task (<= 20), above the two slow event tasks (30, 31).
 *  telemetry_dispatch_task (next slice) takes 28 together with heartbeat.
 *  mqtt_client_task runs inside the kernel's WiFi service task (priority 4):
 *  see comms_port.h for why.
 * --------------------------------------------------------------------- */
#define S1_PRI_CMD	26	/* cmd_dispatch_task */
#define S1_PRI_WIFI	27	/* wifi_link_task */
#define S1_PRI_HB	28	/* heartbeat_task */
#define S1_PRI_LED	29	/* comms status LED task (was the LED example) */

#define S1_LED_PIN	19	/* GP19: LED + 330R resistor to GND */
#define S1_TICK_MS	250	/* LED pattern tick */

/* ---------------------------------------------------------------------
 *  comms_flg: connection state, one bit per fact (kernel event flag, so
 *  every set/clear is an atomic read-modify-write -- Change Requirement 11).
 * --------------------------------------------------------------------- */
#define CF_WIFI_UP	(1U << 0)	/* associated and has an IP address */
#define CF_MQTT_UP	(1U << 1)	/* MQTT CONNACK received, session open */
#define CF_CONN_LOST	(1U << 2)	/* 3 consecutive missed heartbeats (NFR-B1-2) */
#define CF_COMMS_FAULT	(1U << 3)	/* link down too long: LED fault pattern */
#define CF_TX_DROP	(1U << 4)	/* a frame was dropped (queue full); sticky until reported */
#define CF_STATUS_NOW	(1U << 5)	/* request: publish a status frame now */

/* Bits of flg_cmd (TEMPORARY stand-in for CMD_ bits of sys_evt_flg: repoint
 * cmd_dispatch_task to sys_evt_flg once mission_ctrl_task exists). */
#define CMDF_START	(1U << 0)
#define CMDF_STOP	(1U << 1)
#define CMDF_PAUSE	(1U << 2)
#define CMDF_RESUME	(1U << 3)

/* ---------------------------------------------------------------------
 *  Kernel objects owned by subsystem 1
 * --------------------------------------------------------------------- */
IMPORT ID	flg_comms;	/* connection state, CF_ bits */
IMPORT ID	flg_fault;	/* the shared fault bitmask, FAULT_ bits */
IMPORT ID	flg_cmd;	/* commands for mission control, CMDF_ bits */
IMPORT ID	mbf_pub;	/* outgoing frames: [topic][payload]  (comms_publish) */
IMPORT ID	mbf_rx;		/* inbound robot/cmd payloads (from the MQTT client) */
IMPORT ID	tid_led;	/* IF-05: the LED task, so subsystem 4 may pause it */

/* ---------------------------------------------------------------------
 *  Communication API for the other subsystems
 * --------------------------------------------------------------------- */

/* Queue one binary frame for publishing on a COMMS_TOPIC_xxx topic.
 * Never blocks: returns E_TMOUT (and sets CF_TX_DROP) if the queue is full,
 * E_PAR for a bad topic or length.  Frames queued while MQTT is down are
 * discarded -- stale telemetry is worthless; the next status frame carries
 * the latest state. */
IMPORT ER	comms_publish(UB topic, const UB *frame, UINT len);

/* True if the MQTT session is up and the link is not considered lost. */
IMPORT BOOL	comms_is_up(void);

/* Mission control reports the system mode (SYS_MODE_xxx).  Publishes a status
 * frame right away so the dashboard sees every mode change. */
IMPORT void	comms_set_mode(UB mode);
IMPORT UB	comms_get_mode(void);

/* Shared fault bitmask (FAULT_xxx).  Atomic: any task may raise or clear any
 * bits at any time.  fault_raise() also publishes a robot/event/fault frame
 * for the bits that were newly set. */
IMPORT void	fault_raise(UH mask);
IMPORT void	fault_clear(UH mask);
IMPORT UH	fault_get(void);

/* ---------------------------------------------------------------------
 *  Internal to subsystem 1 (shared between its own .c files)
 * --------------------------------------------------------------------- */
IMPORT ER	comms_init(void);
IMPORT ID	cyc_led;
IMPORT void	led_task(INT stacd, void *exinf);
IMPORT void	led_cyclic_handler(void *exinf);
IMPORT void	wifi_link_task(INT stacd, void *exinf);
IMPORT void	heartbeat_task(INT stacd, void *exinf);
IMPORT void	cmd_dispatch_task(INT stacd, void *exinf);
IMPORT UW	comms_uptime_ms(void);		/* ms since boot, low 32 bits (wraps after 49 days) */
IMPORT UB	comms_pub_fail_streak(void);	/* consecutive failed MQTT publishes */
IMPORT void	comms_cmd_note(BOOL accepted);	/* cmd_dispatch_task only */
IMPORT UW	comms_hb_count(void);		/* status frames built so far */
IMPORT UW	comms_cmd_count(void);		/* valid commands accepted so far */
IMPORT UW	comms_cmd_reject_count(void);	/* malformed commands discarded */

#if APP_TEST
IMPORT INT	comms_test_pop(UB *topic, UB *frame, UINT cap);	/* raw queue pop */
IMPORT void	comms_test_hold(BOOL hold);	/* stop the MQTT pump taking frames */
#endif

#endif /* SUBSYSTEM1_H */
