/*
 *      subsystem3.c
 *      IMU: hardware init and task start.                      Owner: Jian Yi
 */

#include "subsystem3.h"
#include "drv_imu.h"

EXPORT ID  tid_imu;
EXPORT INT imu_driver_status;

EXPORT ER subsystem3_init (void)
{
    imu_driver_status = drv_imu_init();
    if (imu_driver_status == DRV_IMU_OK)
    {
        LOG("imu", "GY-511 (LSM303DLHC) found on I2C1 GP2/GP3\n");
        return E_OK;
    }
    LOG("imu", "GY-511 NOT found on I2C1 GP2/GP3 (%s): %s\n",
        (imu_driver_status == DRV_IMU_ERR_ID) ? "wrong chip ID" : "no answer",
        IMU_REQUIRED ? "INIT FAIL" : "running without IMU");
    return IMU_REQUIRED ? E_SYS : E_OK;
}

EXPORT ER subsystem3_start (void)
{
    tid_imu = app_start_periodic(imu_task, PRI_IMU, CORE_IMU, PERIOD_IMU_MS,
                                 "imu", NULL);
    return (tid_imu < E_OK) ? tid_imu : E_OK;
}

/*** end of file ***/
