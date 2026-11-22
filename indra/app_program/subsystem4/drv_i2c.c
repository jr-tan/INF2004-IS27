/*
 *	drv_i2c.c  (Buddy 4)
 *	Minimal I2C0 master (GP4 SDA, GP5 SCL), 400 kHz, polled, direct
 *	registers.  Owner: the IMU task only (one owner per peripheral), so
 *	no lock is needed and the kernel's I2C driver stays switched off
 *	(no extra kernel objects).
 *	Every wait has a deadline: a missing or stuck sensor returns an
 *	error after a few ms instead of hanging the task.
 */
#include "../car.h"
#include "drv_i2c.h"

#define I2C0_BASE		0x40044000U
#define IC_CON			(I2C0_BASE + 0x00U)
#define IC_TAR			(I2C0_BASE + 0x04U)
#define IC_DATA_CMD		(I2C0_BASE + 0x10U)
#define IC_FS_SCL_HCNT		(I2C0_BASE + 0x1CU)
#define IC_FS_SCL_LCNT		(I2C0_BASE + 0x20U)
#define IC_RAW_INTR_STAT	(I2C0_BASE + 0x34U)
#define IC_RX_TL		(I2C0_BASE + 0x38U)
#define IC_TX_TL		(I2C0_BASE + 0x3CU)
#define IC_CLR_TX_ABRT		(I2C0_BASE + 0x54U)
#define IC_ENABLE		(I2C0_BASE + 0x6CU)
#define IC_STATUS		(I2C0_BASE + 0x70U)
#define IC_RXFLR		(I2C0_BASE + 0x78U)
#define IC_SDA_HOLD		(I2C0_BASE + 0x7CU)
#define IC_ENABLE_STATUS	(I2C0_BASE + 0x9CU)
#define IC_FS_SPKLEN		(I2C0_BASE + 0xA0U)

#define CON_MASTER		(1U << 0)
#define CON_SPEED_FAST		(2U << 1)
#define CON_RESTART_EN		(1U << 5)
#define CON_SLAVE_DISABLE	(1U << 6)
#define CON_TX_EMPTY_CTRL	(1U << 8)
#define IC_CMD_READ		(1U << 8)
#define IC_CMD_STOP		(1U << 9)
#define IC_CMD_RESTART		(1U << 10)
#define RAW_TX_ABRT		(1U << 6)
#define ST_TFNF			(1U << 1)
#define ST_TFE			(1U << 2)
#define ST_MST_ACTIVITY		(1U << 5)

#define TIMEOUT_US		3000U
#define PAD_SCHMITT		(1U << 1)

/* 400 kHz from clk_sys 125 MHz, as the Pico SDK computes it */
#define PERIOD			313U
#define LCNT			(PERIOD * 3U / 5U)	/* 187 */
#define HCNT			(PERIOD - LCNT)		/* 126 */

LOCAL UB cur_addr = 0xFF;

LOCAL void pin_i2c(UINT pin)
{
	out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_I2C);
	set_w(GPIO(pin), GPIO_IE | GPIO_PUE | PAD_SCHMITT);	/* GY-511 has pull-ups too */
	clr_w(GPIO(pin), GPIO_PDE | GPIO_OD);
}

EXPORT void i2c_init(void)
{
	set_w(RESETS_RESET, RESETS_RESET_I2C0);		/* clean restart */
	clr_w(RESETS_RESET, RESETS_RESET_I2C0);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C0) == 0) { }

	out_w(IC_ENABLE, 0);
	out_w(IC_CON, CON_MASTER | CON_SPEED_FAST | CON_RESTART_EN |
		      CON_SLAVE_DISABLE | CON_TX_EMPTY_CTRL);
	out_w(IC_TX_TL, 0);
	out_w(IC_RX_TL, 0);
	out_w(IC_FS_SCL_HCNT, HCNT);
	out_w(IC_FS_SCL_LCNT, LCNT);
	out_w(IC_FS_SPKLEN, LCNT < 16U ? 1U : LCNT / 16U);
	out_w(IC_SDA_HOLD, (125000000U * 3U) / 10000000U + 1U);	/* 300 ns */
	pin_i2c(PIN_IMU_SDA);
	pin_i2c(PIN_IMU_SCL);
	cur_addr = 0xFF;
}

LOCAL ER set_target(UB addr)
{
	UW t0 = now_us();

	if (addr == cur_addr) return E_OK;
	out_w(IC_ENABLE, 0);
	while (in_w(IC_ENABLE_STATUS) & 1U) {
		if ((now_us() - t0) > TIMEOUT_US) return E_TMOUT;
	}
	out_w(IC_TAR, addr);
	out_w(IC_ENABLE, 1);
	cur_addr = addr;
	return E_OK;
}

LOCAL ER aborted(void)
{
	if (in_w(IC_RAW_INTR_STAT) & RAW_TX_ABRT) {
		(void)in_w(IC_CLR_TX_ABRT);		/* reading clears it */
		return E_IO;				/* no ACK: sensor missing */
	}
	return E_OK;
}

LOCAL ER push(UW cmd)
{
	UW t0 = now_us();
	while (!(in_w(IC_STATUS) & ST_TFNF)) {
		if (aborted() < E_OK) return E_IO;
		if ((now_us() - t0) > TIMEOUT_US) return E_TMOUT;
	}
	out_w(IC_DATA_CMD, cmd);
	return E_OK;
}

LOCAL ER wait_idle(void)
{
	UW t0 = now_us();
	while (!(in_w(IC_STATUS) & ST_TFE) || (in_w(IC_STATUS) & ST_MST_ACTIVITY)) {
		if (aborted() < E_OK) return E_IO;
		if ((now_us() - t0) > TIMEOUT_US) return E_TMOUT;
	}
	return aborted();
}

EXPORT ER i2c_write_reg(UB addr, UB reg, UB val)
{
	ER er;
	if ((er = set_target(addr)) < E_OK) return er;
	if ((er = push(reg)) < E_OK) return er;
	if ((er = push(val | IC_CMD_STOP)) < E_OK) return er;
	return wait_idle();
}

EXPORT ER i2c_read_regs(UB addr, UB reg, UB *buf, UINT len)
{
	UINT sent = 0, got = 0;
	UW t0 = now_us();
	ER er;

	if (len == 0) return E_PAR;
	if ((er = set_target(addr)) < E_OK) return er;
	if ((er = push(reg)) < E_OK) return er;
	while (got < len) {
		if (sent < len && (in_w(IC_STATUS) & ST_TFNF)) {
			UW cmd = IC_CMD_READ;
			if (sent == 0) cmd |= IC_CMD_RESTART;
			if (sent == len - 1U) cmd |= IC_CMD_STOP;
			out_w(IC_DATA_CMD, cmd);
			sent++;
		}
		if (in_w(IC_RXFLR) > 0) {
			buf[got++] = (UB)(in_w(IC_DATA_CMD) & 0xFFU);
			t0 = now_us();
		}
		if (aborted() < E_OK) return E_IO;
		if ((now_us() - t0) > TIMEOUT_US) return E_TMOUT;
	}
	return E_OK;
}
