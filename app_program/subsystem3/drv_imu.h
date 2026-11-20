/*
 *      drv_imu.h
 *      GY-511 (LSM303DLHC) on I2C1, GP2 / GP3, 400 kHz.  Owner: Jian Yi.
 *      Plan refs: JY-1.3..JY-1.6.
 *
 *      TWO WAYS TO GET I2C1 -- pick one (JY-1.5), then fill in drv_imu.c:
 *       (a) Register-level I2C1 on the SDK side (hardware/structs/i2c.h),
 *           polled.  12 bytes at 400 kHz is ~0.4 ms busy time per 20 ms.
 *       (b) The kernel's I2C device driver (device/i2c), interrupt driven.
 *           NEEDS template patch P3: as shipped it puts I2C0 on GP8/GP9
 *           (your LEFT MOTOR) and I2C1 on GP6/GP7 (GP7 = LEFT ENCODER A), and
 *           devinit only starts unit 0.  Then drv_imu.c must be kernel side.
 *
 *      The interface stays the same either way: raw counts, no units.
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <stdint.h>

#define IMU_ADDR_ACC 0x19U
#define IMU_ADDR_MAG 0x1EU

#define DRV_IMU_OK 0
/* NACK / timeout: counts toward IMU_SENSOR */
#define DRV_IMU_ERR_BUS      (-1)
#define DRV_IMU_ERR_ID       (-2) /* answered, but not an LSM303DLHC */
#define IMU_ACC_MG_PER_LSB   2    /* +-4 g, high resolution, after >> 4 */
#define IMU_MAG_XY_PER_GAUSS 1100 /* +-1.3 gauss */
#define IMU_MAG_Z_PER_GAUSS  980
#define DRV_IMU_ERR_TODO     (-100) /* driver not written yet */

typedef struct
{
    int16_t ax, ay, az; /* raw accelerometer */
    int16_t mx, my, mz; /* raw magnetometer */
} IMU_RAW;

int32_t drv_imu_init(void); /* check device IDs (JY-1.3) */
int32_t drv_imu_read(IMU_RAW * out);

#endif /* DRV_IMU_H */

/*** end of file ***/
