/*
 *	drv_imu.c
 *	GY-511 (LSM303DLHC) on I2C1, GP2 = SDA, GP3 = SCL, 400 kHz.
 *	SDK side, option (a) of drv_imu.h: register-level, polled.
 *	                                                       Owner: Jian Yi
 *
 *	Accelerometer 0x19: 50 Hz, high resolution, +-4 g (2 mg / LSB after
 *	  the 4-bit right shift), block data update.
 *	Magnetometer  0x1E: 30 Hz, +-1.3 gauss (1100 LSB/gauss X/Y, 980 Z),
 *	  continuous.  Its data registers are X, Z, Y, big-endian.
 *	One read = two transfers (6 + 6 bytes): ~0.4 ms of busy time per
 *	20 ms.  The Wi-Fi task (core 0, priority 4) may pre-empt mid-transfer;
 *	the master just holds the clock, so timeouts are generous.
 */

#include "drv_imu.h"
#include "../hal.h"
#include "../hw_alloc.h"
#include "hardware/gpio.h"
#include "hardware/structs/i2c.h"
#include "hardware/regs/i2c.h"
#include "hardware/structs/resets.h"

#define I2C_TIMEOUT_US		20000U

/* LSM303DLHC registers */
#define A_CTRL_REG1		0x20U
#define A_CTRL_REG4		0x23U
#define A_OUT_X_L		0x28U
#define A_AUTO_INC		0x80U	/* MSB of the sub-address */
#define M_CRA_REG		0x00U
#define M_CRB_REG		0x01U
#define M_MR_REG		0x02U
#define M_OUT_X_H		0x03U
#define M_IRA_REG		0x0AU	/* "H43" identification */

static i2c_hw_t *const h = i2c1_hw;

static int32_t bus_error(void)
{
	(void)h->clr_tx_abrt;
	(void)h->clr_stop_det;
	return DRV_IMU_ERR_BUS;
}

static int32_t wait_tx_room(uint32_t t0)
{
	while ((h->status & I2C_IC_STATUS_TFNF_BITS) == 0) {
		if (h->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) return DRV_IMU_ERR_BUS;
		if ((hal_now_us() - t0) > I2C_TIMEOUT_US) return DRV_IMU_ERR_BUS;
	}
	return DRV_IMU_OK;
}

/* write wn bytes, then (repeated start) read rn bytes, then stop */
static int32_t xfer(uint8_t addr, const uint8_t *w, uint32_t wn, uint8_t *r, uint32_t rn)
{
	uint32_t i, t0 = hal_now_us();

	h->enable = 0;
	h->tar = addr;
	h->enable = 1;
	(void)h->clr_tx_abrt;
	(void)h->clr_stop_det;

	for (i = 0; i < wn; i++) {
		uint32_t cmd = w[i];
		if (i == wn - 1U && rn == 0U) cmd |= I2C_IC_DATA_CMD_STOP_BITS;
		if (wait_tx_room(t0) != DRV_IMU_OK) return bus_error();
		h->data_cmd = cmd;
	}
	for (i = 0; i < rn; i++) {
		uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS;
		if (i == 0U && wn > 0U) cmd |= I2C_IC_DATA_CMD_RESTART_BITS;
		if (i == rn - 1U)       cmd |= I2C_IC_DATA_CMD_STOP_BITS;
		if (wait_tx_room(t0) != DRV_IMU_OK) return bus_error();
		h->data_cmd = cmd;
		while (h->rxflr == 0U) {
			if (h->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) return bus_error();
			if ((hal_now_us() - t0) > I2C_TIMEOUT_US) return bus_error();
		}
		r[i] = (uint8_t)h->data_cmd;
	}
	while ((h->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) == 0) {
		if (h->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) return bus_error();
		if ((hal_now_us() - t0) > I2C_TIMEOUT_US) return bus_error();
	}
	(void)h->clr_stop_det;
	if (h->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) return bus_error();
	return DRV_IMU_OK;
}

static int32_t wr_reg(uint8_t addr, uint8_t reg, uint8_t v)
{
	uint8_t b[2];
	b[0] = reg;
	b[1] = v;
	return xfer(addr, b, 2, 0, 0);
}

int32_t drv_imu_init(void)
{
	uint32_t freq = 125000000U, baud = 400000U, period, lcnt, hcnt, hold;
	uint8_t  reg, id[3], v;

	/* release I2C1 from reset */
	hw_set_bits(&resets_hw->reset, RESETS_RESET_I2C1_BITS);
	hw_clear_bits(&resets_hw->reset, RESETS_RESET_I2C1_BITS);
	while ((resets_hw->reset_done & RESETS_RESET_I2C1_BITS) == 0) {
		/* wait */
	}

	/* same timing as the SDK's i2c_init() / i2c_set_baudrate() */
	h->enable = 0;
	h->con = I2C_IC_CON_SPEED_VALUE_FAST << I2C_IC_CON_SPEED_LSB |
		 I2C_IC_CON_MASTER_MODE_BITS | I2C_IC_CON_IC_SLAVE_DISABLE_BITS |
		 I2C_IC_CON_IC_RESTART_EN_BITS | I2C_IC_CON_TX_EMPTY_CTRL_BITS;
	h->tx_tl = 0;
	h->rx_tl = 0;
	period = (freq + baud / 2U) / baud;
	lcnt = period * 3U / 5U;
	hcnt = period - lcnt;
	h->fs_scl_hcnt = hcnt;
	h->fs_scl_lcnt = lcnt;
	h->fs_spklen = (lcnt < 16U) ? 1U : lcnt / 16U;
	hold = ((freq * 3U) / 10000000U) + 1U;
	h->sda_hold = (h->sda_hold & ~I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS) | hold;
	h->enable = 1;

	gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
	gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(PIN_IMU_SDA);
	gpio_pull_up(PIN_IMU_SCL);

	/* magnetometer identification "H43" */
	reg = M_IRA_REG;
	if (xfer(IMU_ADDR_MAG, &reg, 1, id, 3) != DRV_IMU_OK) return DRV_IMU_ERR_BUS;
	if (id[0] != 'H' || id[1] != '4' || id[2] != '3') return DRV_IMU_ERR_ID;

	/* accelerometer: 50 Hz, XYZ on; BDU, +-4 g, high resolution */
	if (wr_reg(IMU_ADDR_ACC, A_CTRL_REG1, 0x47U) != DRV_IMU_OK) return DRV_IMU_ERR_BUS;
	if (wr_reg(IMU_ADDR_ACC, A_CTRL_REG4, 0x98U) != DRV_IMU_OK) return DRV_IMU_ERR_BUS;
	reg = A_CTRL_REG1;
	if (xfer(IMU_ADDR_ACC, &reg, 1, &v, 1) != DRV_IMU_OK) return DRV_IMU_ERR_BUS;
	if (v != 0x47U) return DRV_IMU_ERR_ID;		/* accel not answering right */

	/* magnetometer: 30 Hz, +-1.3 gauss, continuous */
	if (wr_reg(IMU_ADDR_MAG, M_CRA_REG, 0x14U) != DRV_IMU_OK) return DRV_IMU_ERR_BUS;
	if (wr_reg(IMU_ADDR_MAG, M_CRB_REG, 0x20U) != DRV_IMU_OK) return DRV_IMU_ERR_BUS;
	if (wr_reg(IMU_ADDR_MAG, M_MR_REG,  0x00U) != DRV_IMU_OK) return DRV_IMU_ERR_BUS;
	return DRV_IMU_OK;
}

int32_t drv_imu_read(IMU_RAW *out)
{
	uint8_t reg, b[6];

	reg = A_OUT_X_L | A_AUTO_INC;
	if (xfer(IMU_ADDR_ACC, &reg, 1, b, 6) != DRV_IMU_OK) return DRV_IMU_ERR_BUS;
	/* 12-bit left justified, little-endian: >> 4 = 2 mg per count at +-4 g */
	out->ax = (int16_t)((int16_t)(b[0] | (b[1] << 8)) >> 4);
	out->ay = (int16_t)((int16_t)(b[2] | (b[3] << 8)) >> 4);
	out->az = (int16_t)((int16_t)(b[4] | (b[5] << 8)) >> 4);

	reg = M_OUT_X_H;
	if (xfer(IMU_ADDR_MAG, &reg, 1, b, 6) != DRV_IMU_OK) return DRV_IMU_ERR_BUS;
	out->mx = (int16_t)((b[0] << 8) | b[1]);	/* X, Z, Y order */
	out->mz = (int16_t)((b[2] << 8) | b[3]);
	out->my = (int16_t)((b[4] << 8) | b[5]);
	return DRV_IMU_OK;
}
