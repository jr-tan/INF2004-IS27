/**
 * @file subsystem4_gy511.c
 * @brief Minimal RP2040 MMIO I2C0 driver for the GY-511 protected by an
 *        RTOS priority-inheritance mutex (i2c_mtx with TA_INHERIT).
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "subsystem4_gy511.h"

#ifndef S4_SYS_CLOCK_HZ
#define S4_SYS_CLOCK_HZ (125000000UL)
#endif

#define RP2040_RESETS_BASE       (0x4000C000UL)
#define RP2040_IO_BANK0_BASE     (0x40014000UL)
#define RP2040_PADS_BANK0_BASE   (0x4001C000UL)
#define RP2040_I2C0_BASE         (0x40044000UL)

#define MMIO32(addr)             (*(volatile uint32_t *)(addr))
#define REG32(address)           (*(volatile uint32_t *)(uintptr_t)(address))
#define I2C_REG(offset)          REG32(RP2040_I2C0_BASE + (offset))
#define GPIO_PAD(gpio)           REG32(RP2040_PADS_BANK0_BASE + 0x04UL + ((uint32_t)(gpio) * 4UL))

#define S4_RESETS_RESET          MMIO32(RP2040_RESETS_BASE + 0x00UL)
#define S4_RESETS_RESET_DONE     MMIO32(RP2040_RESETS_BASE + 0x08UL)
#define S4_RESETS_I2C0_BIT       (1UL << 3)

#define GPIO_FUNC_I2C            (3UL)
#define GPIO_PAD_IE              (1UL << 6)
#define GPIO_PAD_PUE             (1UL << 3)
#define GPIO_PAD_PDE             (1UL << 2)

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
#define I2C_IC_TX_ABRT_SOURCE    (0x80UL)

#define I2C_CON_MASTER           (1UL << 0)
#define I2C_CON_SPEED_STANDARD   (1UL << 1)
#define I2C_CON_RESTART_EN       (1UL << 5)
#define I2C_CON_SLAVE_DISABLE    (1UL << 6)
#define I2C_CMD_READ             (1UL << 8)
#define I2C_CMD_STOP             (1UL << 9)
#define I2C_CMD_RESTART          (1UL << 10)
#define I2C_INTR_TX_ABRT         (1UL << 6)
#define I2C_STATUS_TFNF          (1UL << 1)
#define I2C_STATUS_TFE           (1UL << 2)
#define I2C_STATUS_RFNE          (1UL << 3)
#define I2C_STATUS_ACTIVITY      (1UL << 0)
#define I2C_STATUS_MST_ACTIVITY  (1UL << 5)
#define I2C_WAIT_LIMIT           (200000UL)
#define S4_I2C_MUTEX_TIMEOUT_MS  (10)

#define LSM303_REG_WHO_AM_I_A    (0x0FU)

static bool g_i2c_initialized = false;
static bool g_gy511_ready = false;
static uint8_t g_accel_who_am_i = 0U;
static uint32_t g_error_stage = 0U;
static uint32_t g_sample_timestamp_ms = 0U;
static ID g_i2c0_mtx_id = 0;

static bool i2c0_ensure_mutex(void)
{
    T_CMTX mtx_cfg;

    if (g_i2c0_mtx_id > 0) {
        return true;
    }

    /* Tribunal 1 NFR-SYS-2: i2c_mtx initialized with TA_INHERIT */
    mtx_cfg.exinf = NULL;
    mtx_cfg.mtxatr = TA_INHERIT;
    mtx_cfg.ceilpri = 0;
    g_i2c0_mtx_id = tk_cre_mtx(&mtx_cfg);
    return (g_i2c0_mtx_id > 0);
}

static bool i2c_wait_tx_ready(void)
{
    uint32_t wait;

    for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
        if ((I2C_REG(I2C_IC_RAW_INTR_STAT) & I2C_INTR_TX_ABRT) != 0U) {
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

    S4_RESETS_RESET |= S4_RESETS_I2C0_BIT;
    S4_RESETS_RESET &= ~S4_RESETS_I2C0_BIT;

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

    (void)I2C_REG(I2C_IC_CLR_TX_ABRT);
    I2C_REG(I2C_IC_ENABLE) = 0U;

    for (wait = 0U; wait < I2C_WAIT_LIMIT; ++wait) {
        if ((I2C_REG(I2C_IC_STATUS) &
             (I2C_STATUS_ACTIVITY | I2C_STATUS_MST_ACTIVITY)) == 0U) {
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

    if (!i2c_wait_tx_ready()) {
        return false;
    }
    I2C_REG(I2C_IC_DATA_CMD) = (uint32_t)reg;

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

bool GY511_Init(void)
{
    uint8_t who_am_i = 0U;
    bool ok = false;

    g_gy511_ready = false;
    g_error_stage = 0U;

    if (!i2c0_ensure_mutex()) {
        g_error_stage = 1U;
        return false;
    }

    if (tk_loc_mtx(g_i2c0_mtx_id, (TMO)S4_I2C_MUTEX_TIMEOUT_MS) < E_OK) {
        g_error_stage = 1U;
        return false;
    }

    do {
        if (!g_i2c_initialized && !i2c0_init()) {
            g_error_stage = 1U;
            break;
        }

        if (!i2c0_read_regs(GY511_ACCEL_I2C_ADDRESS,
                            LSM303_REG_WHO_AM_I_A, &who_am_i, 1U)) {
            g_error_stage = 2U;
            break;
        }
        g_accel_who_am_i = who_am_i;
        if ((who_am_i != GY511_ACCEL_WHO_AM_I_VALUE) &&
            (who_am_i != GY511_DLHC_WHO_AM_I_VALUE)) {
            g_error_stage = 3U;
            break;
        }

        /* CTRL1 (0x20): 100 Hz accelerometer, all axes enabled -> 0x57 */
        if (!i2c0_write_reg(GY511_ACCEL_I2C_ADDRESS, 0x20U, 0x57U)) {
            g_error_stage = 6U;
            break;
        }

        /* CTRL2 (0x21): +/- 2g scale -> 0x00 */
        if (!i2c0_write_reg(GY511_ACCEL_I2C_ADDRESS, 0x21U, 0x00U)) {
            g_error_stage = 6U;
            break;
        }

        /* CTRL7 (0x26): Magnetometer continuous-conversion mode -> 0x00 */
        if (!i2c0_write_reg(GY511_ACCEL_I2C_ADDRESS, 0x26U, 0x00U)) {
            g_error_stage = 7U;
            break;
        }

        g_sample_timestamp_ms = 0U;
        g_gy511_ready = true;
        ok = true;
    } while (0);

    (void)tk_unl_mtx(g_i2c0_mtx_id);

    if (!ok) {
        Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_IMU_I2C);
    } else {
        Subsystem4_Fault_Clear_Bits(SYS_FAULT_BIT_IMU_I2C);
    }
    return ok;
}

bool GY511_ReadRawSample(gy511_raw_sample_t *p_sample)
{
    uint8_t accel_bytes[6];
    uint8_t mag_bytes[6];
    bool ok = false;

    if ((p_sample == (gy511_raw_sample_t *)0) || !g_gy511_ready) {
        return false;
    }

    if (tk_loc_mtx(g_i2c0_mtx_id, (TMO)S4_I2C_MUTEX_TIMEOUT_MS) < E_OK) {
        Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_IMU_I2C);
        return false;
    }

    if (i2c0_read_regs(GY511_ACCEL_I2C_ADDRESS, 0xA8U, accel_bytes, 6U) &&
        i2c0_read_regs(GY511_ACCEL_I2C_ADDRESS, 0x88U, mag_bytes, 6U)) {
        ok = true;
    }

    (void)tk_unl_mtx(g_i2c0_mtx_id);

    if (!ok) {
        Subsystem4_Fault_Set_Bits(SYS_FAULT_BIT_IMU_I2C);
        return false;
    }

    Subsystem4_Fault_Clear_Bits(SYS_FAULT_BIT_IMU_I2C);

    /* 
     * FIX: Right-shift by 4 (>> 4) to convert LSM303D left-justified 
     * data into the correct range where ~1000 counts = 1.0g.
     */
    p_sample->accel_x = (int16_t)(signed_le16(accel_bytes[0], accel_bytes[1]) >> 4);
    p_sample->accel_y = (int16_t)(signed_le16(accel_bytes[2], accel_bytes[3]) >> 4);
    p_sample->accel_z = (int16_t)(signed_le16(accel_bytes[4], accel_bytes[5]) >> 4);
    
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

ID GY511_Get_I2C_Mutex_ID(void)
{
    (void)i2c0_ensure_mutex();
    return g_i2c0_mtx_id;
}