/*
 *	comms_frames.c
 *	Subsystem 1 -- binary frame encoders/decoders.  See comms_frames.h.
 */

#include "comms_frames.h"
#include "bitpack.h"

uint16_t comms_enc_status(uint8_t *buf, uint16_t cap, const status_frame_t *f)
{
	bp_writer_t w;

	bp_wr_init(&w, buf, cap);
	bp_put_u(&w, f->seq,          ST_SEQ_BITS);
	bp_put_u(&w, f->up_s,         ST_UP_BITS);
	bp_put_u(&w, f->rssi_neg_dbm, ST_RSSI_BITS);
	bp_put_u(&w, f->mode,         SYS_MODE_BITS);
	bp_put_u(&w, f->faults,       ST_FAULT_BITS);
	bp_put_u(&w, f->tx_drop ? 1UL : 0UL, ST_DROP_BITS);

	return w.ok ? bp_wr_len(&w) : 0U;
}

bool comms_dec_status(const uint8_t *buf, uint16_t len, status_frame_t *f)
{
	bp_reader_t r;

	if (len != ST_FRAME_BYTES) {
		return false;
	}
	bp_rd_init(&r, buf, len);
	f->seq          = (uint8_t)bp_get_u(&r, ST_SEQ_BITS);
	f->up_s         = bp_get_u(&r, ST_UP_BITS);
	f->rssi_neg_dbm = (uint8_t)bp_get_u(&r, ST_RSSI_BITS);
	f->mode         = (uint8_t)bp_get_u(&r, SYS_MODE_BITS);
	f->faults       = (uint16_t)bp_get_u(&r, ST_FAULT_BITS);
	f->tx_drop      = (bp_get_u(&r, ST_DROP_BITS) != 0UL);

	return r.ok && (f->mode <= SYS_MODE_MAX);
}

uint16_t comms_enc_fault(uint8_t *buf, uint16_t cap, const fault_frame_t *f)
{
	bp_writer_t w;

	bp_wr_init(&w, buf, cap);
	bp_put_u(&w, f->t_ms,   FE_T_BITS);
	bp_put_u(&w, f->raised, FAULT_BITS);
	bp_put_u(&w, 0UL,       FE_PAD_BITS);

	return w.ok ? bp_wr_len(&w) : 0U;
}

bool comms_dec_fault(const uint8_t *buf, uint16_t len, fault_frame_t *f)
{
	bp_reader_t r;

	if (len != FE_FRAME_BYTES) {
		return false;
	}
	bp_rd_init(&r, buf, len);
	f->t_ms   = (uint16_t)bp_get_u(&r, FE_T_BITS);
	f->raised = (uint16_t)bp_get_u(&r, FAULT_BITS);
	(void)bp_get_u(&r, FE_PAD_BITS);

	return r.ok;
}

uint16_t comms_enc_cmd(uint8_t *buf, uint16_t cap, const cmd_frame_t *c)
{
	bp_writer_t w;

	bp_wr_init(&w, buf, cap);
	bp_put_u(&w, c->code, CMD_CODE_BITS);
	bp_put_u(&w, c->id,   CMD_ID_BITS);

	return w.ok ? bp_wr_len(&w) : 0U;
}

bool comms_dec_cmd(const uint8_t *buf, uint16_t len, cmd_frame_t *c)
{
	bp_reader_t r;

	if (len != CMD_FRAME_BYTES) {
		return false;
	}
	bp_rd_init(&r, buf, len);
	c->code = (uint8_t)bp_get_u(&r, CMD_CODE_BITS);
	c->id   = (uint8_t)bp_get_u(&r, CMD_ID_BITS);

	/* Only codes 1..CMD_CODE_MAX are valid commands. */
	return r.ok && (c->code >= CMD_START) && (c->code <= CMD_CODE_MAX);
}
