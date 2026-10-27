/**
 * @file subsystem4_gy511.c
 * @brief Minimal RP2040 MMIO I2C0 driver for the GY-511 (LSM303DLHC).
 *
 * I2C0 wiring: GP4 = SDA, GP5 = SCL. This driver deliberately avoids Pico SDK
 * headers to remain compatible with the supplied μT-Kernel makefile.
 *
 * Hardware assumptions:
 * - RP2040 peripheral clock is 125 MHz unless S4_SYS_CLOCK_HZ is overridden.
 * - External I2C pull-ups are present or the board's pull-ups are adequate.
 * - The LSM303DLHC accelerometer address is 0x19 and magnetometer address 0x1E.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "subsystem4_gy511.h"
#include <stdint.h>

#ifndef S4_SYS_CLOCK_HZ
#define S4_SYS_CLOCK_HZ (125000000UL)
#endif

#define S4_RESETS_RESET      MMIO32(RP2040_RESETS_BASE + 0x00UL)
#define S4_RESETS_RESET_DONE MMIO32(RP2040_RESETS_BASE + 0x08UL)
#define S4_RESETS_I2C0_BIT   (1UL << 3)
#define RP2040_RESETS_BASE       (0x4000C000UL)
#define RP2040_IO_BANK0_BASE     (0x40014000UL)
#define RP2040_PADS_BANK0_BASE   (0x4001C000UL)
#define RP2040_I2C0_BASE         (0x40044000UL)

#define MMIO32(addr) (*(volatile uint32_t *)(addr))
#define REG32(address) (*(volatile uint32_t *)(uintptr_t)(address))
#define I2C_REG(offset) REG32(RP2040_I2C0_BASE + (offset))
#define GPIO_PAD(gpio)           REG32(RP2040_PADS_BANK0_BASE + 0x04UL + ((uint32_t)(gpio) * 4UL))

#define GPIO_FUNC_I2C            (3UL)
#define GPIO_PAD_IE              (1UL << 6)
#define GPIO_PAD_PUE             (1UL << 3)
#define GPIO_PAD_PDE              (1UL << 2)

#define I2C_IC_CON               (0x00UL)
#define I2C_IC_TAR               (0x04UL)
#define I2C_IC_DATA_CMD          (0x10UL)
#define I2C_IC_SS_SCL_HCNT       (0x14UL)
#define I2C_IC_SS_SCL_LCNT       (0x18UL)
#define I2C_IC_RAW_INTR_STAT     (0x34UL)
#define I2C_IC_RX_TL             (0x38UL)
#define I2C_IC_TX_TL             (0x3CUL)
#define I2C_IC_CLR_TX_ABRT       (0x54UL)
#define I2C_IC_ENABLE            (0x6CUL)
#define I2C_IC_STATUS            (0x70UL)
#define I2C_IC_TXFLR             (0x74UL)
#define I2C_IC_RXFLR             (0x78UL)
#define I2C_IC_TX_ABRT_SOURCE    (0x80UL)

#define I2C_CON_MASTER           (1UL << 0)
#define I2C_CON_SPEED_STANDARD   (1UL << 1)
#define I2C_CON_RESTART_EN       (1UL << 5)
#define I2C_CON_SLAVE_DISABLE    (1UL << 6)
#define I2C_CMD_READ             (1UL << 8)
#define I2C_CMD_STOP             (1UL << 9)
#define I2C_CMD_RESTART          (1UL << 10)
#define I2C_INTR_TX_ABRT          (1UL << 6)
#define I2C_STATUS_TFNF           (1UL << 1)
#define I2C_STATUS_TFE            (1UL << 2)
#define I2C_STATUS_RFNE           (1UL << 3)
#define I2C_STATUS_ACTIVITY       (1UL << 0)
#define I2C_STATUS_MST_ACTIVITY   (1UL << 5)
#define I2C_WAIT_LIMIT            (200000UL)

#define LSM303_REG_CTRL1_A        (0x20U)
#define LSM303_REG_CTRL4_A        (0x23U)
#define LSM303_REG_OUT_X_L_A      (0x28U)
#define LSM303_REG_WHO_AM_I_A     (0x0FU)
#define LSM303_REG_CRA_M          (0x00U)
#define LSM303_REG_CRB_M          (0x01U)
#define LSM303_REG_MR_M           (0x02U)
#define LSM303_REG_OUT_X_H_M      (0x03U)

static bool g_i2c_initialized = false;
static bool g_gy511_ready = false;
static uint8_t g_accel_who_am_i = 0U;
static uint32_t g_error_stage = 0U;
static uint32_t g_sample_timestamp_ms = 0U;

static bool i2c_wait_tx_ready(void)
{
    uint32_t wait;

    for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
        if ((I2C_REG(I2C_IC_RAW_INTR_STAT) & I2C_INTR_TX_ABRT) != 0U) {
            uint32_t abort_src = I2C_REG(I2C_IC_TX_ABRT_SOURCE);
            tm_printf((UB *)"TX_ABRT source: 0x%08lx\n", (UW)abort_src);
            (void)I2C_REG(I2C_IC_CLR_TX_ABRT);
            return false;
        }
        if ((I2C_REG(I2C_IC_STATUS) & I2C_STATUS_TFNF) != 0U) {
            return true;
        }
    }
    return false;
}

static bool i2c_wait_complete(void)
{
    uint32_t wait;

    for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
        if ((I2C_REG(I2C_IC_RAW_INTR_STAT) & I2C_INTR_TX_ABRT) != 0U) {
            (void)I2C_REG(I2C_IC_CLR_TX_ABRT);
            return false;
        }
        if (((I2C_REG(I2C_IC_STATUS) & I2C_STATUS_TFE) != 0U) &&
            ((I2C_REG(I2C_IC_STATUS) &
              (I2C_STATUS_ACTIVITY | I2C_STATUS_MST_ACTIVITY)) == 0U)) {
            return true;
        }
    }
    return false;
}

static bool i2c0_init(void)
{
    uint32_t wait;
    uint32_t high_count;
    uint32_t low_count;

    /* Put I2C0 into reset */
    S4_RESETS_RESET |= S4_RESETS_I2C0_BIT;
    /* Take I2C0 out of reset */
    S4_RESETS_RESET &= ~S4_RESETS_I2C0_BIT;

    /* Wait until reset cycle completes */
    for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
        if ((S4_RESETS_RESET_DONE & S4_RESETS_I2C0_BIT) != 0U) {
            break;
        }
    }

    if (wait >= I2C_WAIT_LIMIT) {
        return false;
    }

    GPIO_PAD(4U) = (GPIO_PAD(4U) | GPIO_PAD_IE | GPIO_PAD_PUE) & ~GPIO_PAD_PDE;
    GPIO_PAD(5U) = (GPIO_PAD(5U) | GPIO_PAD_IE | GPIO_PAD_PUE) & ~GPIO_PAD_PDE;
    REG32(RP2040_IO_BANK0_BASE + 0x04UL + (4UL * 8UL)) =
        (REG32(RP2040_IO_BANK0_BASE + 0x04UL + (4UL * 8UL)) & ~0x1FUL) |
        GPIO_FUNC_I2C;
    REG32(RP2040_IO_BANK0_BASE + 0x04UL + (5UL * 8UL)) =
        (REG32(RP2040_IO_BANK0_BASE + 0x04UL + (5UL * 8UL)) & ~0x1FUL) |
        GPIO_FUNC_I2C;

    I2C_REG(I2C_IC_ENABLE) = 0U;
    I2C_REG(I2C_IC_CON) = I2C_CON_MASTER | I2C_CON_SPEED_STANDARD |
                          I2C_CON_RESTART_EN | I2C_CON_SLAVE_DISABLE;

    /* Conservative standard-mode timing for a 125 MHz peripheral clock. */
    high_count = ((S4_SYS_CLOCK_HZ / 1000000UL) * 4UL) - 8UL;
    low_count = ((S4_SYS_CLOCK_HZ / 1000000UL) * 47UL / 10UL) - 1UL;
    I2C_REG(I2C_IC_SS_SCL_HCNT) = high_count;
    I2C_REG(I2C_IC_SS_SCL_LCNT) = low_count;
    I2C_REG(I2C_IC_RX_TL) = 0U;
    I2C_REG(I2C_IC_TX_TL) = 0U;
    I2C_REG(I2C_IC_ENABLE) = 1U;
    g_i2c_initialized = true;
    return true;
}

static bool i2c0_set_target(uint8_t address)
{
    uint32_t wait;

    /* Clear any lingering abort flags */
    (void)I2C_REG(I2C_IC_CLR_TX_ABRT);

    /* Disable controller */
    I2C_REG(I2C_IC_ENABLE) = 0U;

    /* Wait until the controller finishes any current byte transmission */
    for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
        if ((I2C_REG(I2C_IC_STATUS) & (I2C_STATUS_ACTIVITY | I2C_STATUS_MST_ACTIVITY)) == 0U) {
            break;
        }
    }

    I2C_REG(I2C_IC_TAR) = (uint32_t)(address & 0x7FU);
    I2C_REG(I2C_IC_ENABLE) = 1U;
    return true;
}
static bool i2c0_write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    if (!i2c0_set_target(address) || !i2c_wait_tx_ready()) {
        return false;
    }
    I2C_REG(I2C_IC_DATA_CMD) = (uint32_t)reg;
    if (!i2c_wait_tx_ready()) {
        return false;
    }
    I2C_REG(I2C_IC_DATA_CMD) = (uint32_t)value | I2C_CMD_STOP;
    return i2c_wait_complete();
}

static bool i2c0_read_regs(uint8_t address,
                           uint8_t reg,
                           uint8_t *p_data,
                           uint32_t length)
{
    uint32_t index;
    uint32_t wait;

    if ((p_data == (uint8_t *)0) || (length == 0U) || (length > 16U)) {
        return false;
    }

    if (!i2c0_set_target(address)) {
        return false;
    }

    /* Send the register address */
    if (!i2c_wait_tx_ready()) {
        return false;
    }
    I2C_REG(I2C_IC_DATA_CMD) = (uint32_t)reg;

    /* Queue all read requests */
    for (index = 0U; index < length; ++index) {
        uint32_t command = I2C_CMD_READ;

        if (index == 0U) {
            command |= I2C_CMD_RESTART;
        }
        if (index == (length - 1U)) {
            command |= I2C_CMD_STOP;
        }

        if (!i2c_wait_tx_ready()) {
            return false;
        }
        I2C_REG(I2C_IC_DATA_CMD) = command;
    }

    /* Read incoming bytes out of RX FIFO */
    for (index = 0U; index < length; ++index) {
        for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
            if ((I2C_REG(I2C_IC_RAW_INTR_STAT) & I2C_INTR_TX_ABRT) != 0U) {
                (void)I2C_REG(I2C_IC_CLR_TX_ABRT);
                return false;
            }
            if ((I2C_REG(I2C_IC_STATUS) & I2C_STATUS_RFNE) != 0U) {
                break;
            }
        }

        if (wait >= I2C_WAIT_LIMIT) {
            return false;
        }

        p_data[index] = (uint8_t)(I2C_REG(I2C_IC_DATA_CMD) & 0xFFU);
    }

    return i2c_wait_complete();
}

static int16_t signed_le16(uint8_t low, uint8_t high)
{
    return (int16_t)((((uint16_t)high) << 8) | (uint16_t)low);
}

static int16_t signed_be16(uint8_t high, uint8_t low)
{
    return (int16_t)((((uint16_t)high) << 8) | (uint16_t)low);
}

bool GY511_Init(void)
{
    uint8_t who_am_i;
    uint8_t mag_id[3];

    g_gy511_ready = false;
    g_error_stage = 0U;

    if (!g_i2c_initialized && !i2c0_init()) {
        g_error_stage = 1U;
        return false;
    }

    if (!i2c0_read_regs(GY511_ACCEL_I2C_ADDRESS,
                        LSM303_REG_WHO_AM_I_A, &who_am_i, 1U)) {
        g_error_stage = 2U;
        return false;
    }
    g_accel_who_am_i = who_am_i;
    /* WHO_AM_I check for LSM303D (0x49) */
    if (who_am_i != GY511_ACCEL_WHO_AM_I_VALUE) {
        g_error_stage = 3U;
        return false;
    }

    /* LSM303D unified configuration */
    /* CTRL1 (0x20): 100 Hz accelerometer, all axes enabled -> 0x57 */
    if (!i2c0_write_reg(GY511_ACCEL_I2C_ADDRESS, 0x20U, 0x57U)) {
        g_error_stage = 6U;
        return false;
    }

    /* CTRL2 (0x21): +/- 2g scale -> 0x00 */
    if (!i2c0_write_reg(GY511_ACCEL_I2C_ADDRESS, 0x21U, 0x00U)) {
        g_error_stage = 6U;
        return false;
    }

    /* CTRL7 (0x26): Magnetometer continuous-conversion mode -> 0x00 */
    if (!i2c0_write_reg(GY511_ACCEL_I2C_ADDRESS, 0x26U, 0x00U)) {
        g_error_stage = 7U;
        return false;
    }

    g_sample_timestamp_ms = 0U;
    g_gy511_ready = true;
    return true;
}

bool GY511_ReadRawSample(gy511_raw_sample_t *p_sample)
{
    uint8_t accel_bytes[6];
    uint8_t mag_bytes[6];

    if ((p_sample == (gy511_raw_sample_t *)0) || !g_gy511_ready) {
        return false;
    }

    /* Read 6 accelerometer bytes (0x28 with auto-increment bit 7 -> 0xA8) */
    if (!i2c0_read_regs(GY511_ACCEL_I2C_ADDRESS, 0xA8U, accel_bytes, 6U)) {
        return false;
    }

    /* Read 6 magnetometer bytes (0x08 with auto-increment bit 7 -> 0x88) from the SAME address */
    if (!i2c0_read_regs(GY511_ACCEL_I2C_ADDRESS, 0x88U, mag_bytes, 6U)) {
        return false;
    }

    p_sample->accel_x = signed_le16(accel_bytes[0], accel_bytes[1]);
    p_sample->accel_y = signed_le16(accel_bytes[2], accel_bytes[3]);
    p_sample->accel_z = signed_le16(accel_bytes[4], accel_bytes[5]);

    /* LSM303D magnetometer output is also little-endian */
    p_sample->mag_x = signed_le16(mag_bytes[0], mag_bytes[1]);
    p_sample->mag_y = signed_le16(mag_bytes[2], mag_bytes[3]);
    p_sample->mag_z = signed_le16(mag_bytes[4], mag_bytes[5]);

    g_sample_timestamp_ms += SUBSYSTEM4_SAMPLE_PERIOD_MS;
    p_sample->timestamp_ms = g_sample_timestamp_ms;
    return true;
}

bool GY511_IsReady(void)
{
    return g_gy511_ready;
}

uint8_t GY511_Get_Accel_WhoAmI(void)
{
    return g_accel_who_am_i;
}

uint32_t GY511_Get_Error_Stage(void)
{
    return g_error_stage;
}
