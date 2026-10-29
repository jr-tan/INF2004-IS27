/*
 *	test_codec.c
 *	Host-side (PC) unit tests for bitpack.c and comms_frames.c, plus the
 *	message-size comparison (JSON vs binary) required by Change Requirement 11.
 *
 *	Build and run from this folder:   make        (see Makefile)
 *	Test IDs S1-H01.. are recorded in testing/subsystem1/TEST_LOG.md.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "../../../app_program/subsystem1/bitpack.h"
#include "../../../app_program/subsystem1/comms_frames.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond, desc)						\
	do {								\
		if (cond) { g_pass++; printf("[HOST] %s PASS  %s\n", id, desc); } \
		else { g_fail++; printf("[HOST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); } \
	} while (0)

/* Reference model: decode a frame the way the Python tool does. */
static unsigned long long ref_field(const uint8_t *b, unsigned len, unsigned off, unsigned width)
{
	unsigned long long v = 0ULL;
	unsigned i;
	for (i = 0; i < len; i++) { v |= ((unsigned long long)b[i]) << (8U * i); }
	return (v >> off) & ((1ULL << width) - 1ULL);
}

static void test_bitpack(void)
{
	uint8_t buf[8];
	bp_writer_t w;
	bp_reader_t r;
	unsigned width;
	bool all_ok = true;

	/* S1-H01: every unsigned width 1..32, values 0 / max / pattern, round trip at
	 * every start offset 0..7 (so fields straddle byte boundaries). */
	for (width = 1; width <= 32; width++) {
		uint32_t max = (width == 32) ? UINT32_MAX : ((UINT32_C(1) << width) - UINT32_C(1));
		uint32_t vals[3];
		unsigned i, pre;
		vals[0] = 0; vals[1] = max; vals[2] = 0xA5A5A5A5UL & max;
		for (pre = 0; pre < 8; pre++) {
			for (i = 0; i < 3; i++) {
				uint8_t buf2[12];
				bp_wr_init(&w, buf2, sizeof buf2);
				if (pre) { bp_put_u(&w, 0, (uint8_t)pre); }
				bp_put_u(&w, vals[i], (uint8_t)width);
				bp_rd_init(&r, buf2, sizeof buf2);
				if (pre) { (void)bp_get_u(&r, (uint8_t)pre); }
				if (!(w.ok && r.ok && bp_get_u(&r, (uint8_t)width) == vals[i] && !w.clipped)) {
					all_ok = false;
				}
				/* and against the independent reference decoder */
				if (ref_field(buf2, sizeof buf2, pre, width) != vals[i]) { all_ok = false; }
			}
		}
	}
	CHECK("S1-H01", all_ok, "unsigned round trip, widths 1..32, all bit offsets, matches reference decoder");

	/* S1-H02: signed round trip incl. extremes and sign extension */
	all_ok = true;
	for (width = 2; width <= 32; width++) {
		int32_t hi = (width == 32) ? INT32_MAX : (int32_t)((UINT32_C(1) << (width - 1)) - UINT32_C(1));
		int32_t lo = -hi - 1;
		int32_t vals[5] = { 0, 1, -1, hi, lo };
		unsigned i;
		for (i = 0; i < 5; i++) {
			bp_wr_init(&w, buf, sizeof buf);
			bp_put_s(&w, vals[i], (uint8_t)width);
			bp_rd_init(&r, buf, sizeof buf);
			if (!(w.ok && !w.clipped && bp_get_s(&r, (uint8_t)width) == vals[i])) { all_ok = false; }
		}
	}
	CHECK("S1-H02", all_ok, "signed round trip incl. min/max for widths 2..32");

	/* S1-H03: out-of-range values saturate and flag clipped (no wrap-around) */
	bp_wr_init(&w, buf, sizeof buf);
	bp_put_u(&w, 40, 5);          /* max 31 */
	bp_put_s(&w, 100, 7);         /* max 63 */
	bp_put_s(&w, -100, 7);        /* min -64 */
	bp_rd_init(&r, buf, sizeof buf);
	CHECK("S1-H03", w.clipped && bp_get_u(&r, 5) == 31 && bp_get_s(&r, 7) == 63 && bp_get_s(&r, 7) == -64,
	      "out-of-range values saturate, clipped flag set");

	/* S1-H04: writer refuses to overflow its buffer and never writes past it */
	{
		uint8_t guard[4] = { 0, 0, 0xEE, 0xEE };
		bp_wr_init(&w, guard, 2);
		bp_put_u(&w, 0xFFFF, 16);
		bp_put_u(&w, 1, 1);       /* no room */
		CHECK("S1-H04", !w.ok && guard[2] == 0xEE && guard[3] == 0xEE, "buffer overflow rejected, no write past end");
	}

	/* S1-H05: reader refuses to read past the end and returns 0 */
	bp_rd_init(&r, buf, 1);
	(void)bp_get_u(&r, 8);
	CHECK("S1-H05", bp_get_u(&r, 1) == 0 && !r.ok, "read past end fails safely");

	/* S1-H06: invalid widths rejected */
	bp_wr_init(&w, buf, sizeof buf);
	bp_put_u(&w, 1, 0);
	CHECK("S1-H06", !w.ok, "zero width rejected");
	bp_wr_init(&w, buf, sizeof buf);
	bp_put_u(&w, 1, 33);
	CHECK("S1-H06b", !w.ok, "width > 32 rejected");
}

static void test_frames(void)
{
	uint8_t b[COMMS_FRAME_MAX];
	status_frame_t s_in, s_out;
	fault_frame_t f_in, f_out;
	cmd_frame_t c_in, c_out;
	unsigned seq;
	bool ok = true;
	uint16_t n;

	/* S1-H07: status round trip over the full range of each field */
	for (seq = 0; seq < 16; seq++) {
		s_in.seq = (uint8_t)seq;
		s_in.up_s = (seq == 15) ? 0xFFFFFU : seq * 70001U;
		s_in.rssi_neg_dbm = (uint8_t)(seq * 8U);
		s_in.mode = (uint8_t)(seq % 9U);
		s_in.faults = (uint16_t)((seq * 0x111U) & FAULT_MASK_ALL);
		s_in.tx_drop = (seq & 1U) != 0U;
		n = comms_enc_status(b, sizeof b, &s_in);
		if (n != ST_FRAME_BYTES || !comms_dec_status(b, n, &s_out) ||
		    s_out.seq != s_in.seq || s_out.up_s != s_in.up_s || s_out.rssi_neg_dbm != s_in.rssi_neg_dbm ||
		    s_out.mode != s_in.mode || s_out.faults != s_in.faults || s_out.tx_drop != s_in.tx_drop) { ok = false; }
	}
	CHECK("S1-H07", ok, "status frame round trip");
	CHECK("S1-H08", ST_FRAME_BYTES == 6U && ST_TOTAL_BITS == 48U, "status frame is exactly 48 bits / 6 bytes");

	/* S1-H09: wire layout is the documented one (guards the Python decoder) */
	s_in.seq = 5; s_in.up_s = 312; s_in.rssi_neg_dbm = 61; s_in.mode = 2; s_in.faults = 0x040; s_in.tx_drop = false;
	n = comms_enc_status(b, sizeof b, &s_in);
	CHECK("S1-H09", ref_field(b, n, 0, 4) == 5 && ref_field(b, n, 4, 20) == 312 && ref_field(b, n, 24, 7) == 61 &&
	      ref_field(b, n, 31, 4) == 2 && ref_field(b, n, 35, 12) == 0x040 && ref_field(b, n, 47, 1) == 0,
	      "status field offsets match docs (seq0 up4 rssi24 mode31 fault35 drop47)");
	printf("       status example bytes:");
	{ unsigned i; for (i = 0; i < n; i++) { printf(" %02X", b[i]); } printf("\n"); }

	/* S1-H10: a corrupt mode value (9..15) is rejected */
	s_in.mode = 9;
	n = comms_enc_status(b, sizeof b, &s_in);
	CHECK("S1-H10", !comms_dec_status(b, n, &s_out), "status with mode > 8 rejected");

	/* S1-H11: wrong lengths rejected */
	CHECK("S1-H11", !comms_dec_status(b, 5, &s_out) && !comms_dec_status(b, 7, &s_out) &&
	      !comms_dec_fault(b, 3, &f_out) && !comms_dec_cmd(b, 0, &c_out) && !comms_dec_cmd(b, 2, &c_out),
	      "frames of the wrong length are rejected");

	/* S1-H12: fault frame */
	f_in.t_ms = 13550U; f_in.raised = FAULT_AVOID | FAULT_NO_ECHO;
	n = comms_enc_fault(b, sizeof b, &f_in);
	printf("       fault example bytes:");
	{ unsigned i; for (i = 0; i < n; i++) { printf(" %02X", b[i]); } printf("\n"); }
	CHECK("S1-H12", n == 4U && comms_dec_fault(b, n, &f_out) && f_out.t_ms == 13550U && f_out.raised == f_in.raised,
	      "fault frame round trip, 4 bytes");

	/* S1-H13: command frames: every valid code/id, invalid codes rejected */
	ok = true;
	for (c_in.code = 0; c_in.code < 8; c_in.code++) {
		for (c_in.id = 0; c_in.id < 32; c_in.id++) {
			bool valid = (c_in.code >= CMD_START && c_in.code <= CMD_CODE_MAX);
			n = comms_enc_cmd(b, sizeof b, &c_in);
			if (n != 1U) { ok = false; }
			if (comms_dec_cmd(b, n, &c_out) != valid) { ok = false; }
			if (valid && (c_out.code != c_in.code || c_out.id != c_in.id)) { ok = false; }
		}
	}
	CHECK("S1-H13", ok, "cmd frame: all code/id combinations; codes 0,6,7 rejected");

	/* S1-H14: documented example bytes for the dashboard/tools: START id 7 */
	c_in.code = CMD_START; c_in.id = 7;
	(void)comms_enc_cmd(b, sizeof b, &c_in);
	CHECK("S1-H14", b[0] == 0x39U, "START id=7 encodes to 0x39 (id<<3 | code)");
}

/* Message size comparison for the Week 13 report: the JSON examples are the
 * ones in Design Review section 3.3, the binary sizes come from the codec. */
static void size_comparison(void)
{
	struct { const char *topic; const char *json; unsigned bin; } rows[3];
	unsigned i, tj = 0, tb = 0;
	uint8_t b[COMMS_FRAME_MAX];
	status_frame_t s = { 5, 312, 61, SYS_MODE_LINE, 0, false };
	fault_frame_t f = { 13550U, FAULT_AVOID };
	cmd_frame_t c = { CMD_START, 7 };

	rows[0].topic = "robot/status";
	rows[0].json = "{\"up\":312,\"rssi\":-61,\"st\":\"LINE\",\"fault\":0}";
	rows[0].bin = comms_enc_status(b, sizeof b, &s);
	rows[1].topic = "robot/event/fault";
	rows[1].json = "{\"t\":13550,\"src\":\"B5\",\"code\":\"AVOID_FAULT\"}";
	rows[1].bin = comms_enc_fault(b, sizeof b, &f);
	rows[2].topic = "robot/cmd";
	rows[2].json = "{\"cmd\":\"START\",\"id\":7}";
	rows[2].bin = comms_enc_cmd(b, sizeof b, &c);

	printf("\nMessage size comparison (payload bytes, MQTT headers excluded)\n");
	printf("%-20s %8s %8s %8s\n", "topic", "JSON", "binary", "saving");
	for (i = 0; i < 3; i++) {
		unsigned j = (unsigned)strlen(rows[i].json);
		printf("%-20s %8u %8u %7u%%\n", rows[i].topic, j, rows[i].bin, 100U - (100U * rows[i].bin) / j);
		tj += j; tb += rows[i].bin;
	}
	printf("%-20s %8u %8u %7u%%\n", "total", tj, tb, 100U - (100U * tb) / tj);
}

int main(void)
{
	test_bitpack();
	test_frames();
	size_comparison();
	printf("\n[HOST] SUMMARY %d passed, %d failed\n", g_pass, g_fail);
	return (g_fail == 0) ? 0 : 1;
}
