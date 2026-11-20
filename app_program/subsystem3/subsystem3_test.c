/*
 *      subsystem3_test.c
 *      On-target tests for the IMU.  Built only with: make TEST=1
 *      Record results in testing/subsystem3/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "drv_imu.h"
#include "../nav_state.h"
#include "../wcet.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem3_test (void)
{
    NAV_STATE ns;

    TEST_INIT();

    /* S3-U01 (JY-1.3): device IDs read back.  Expected FAIL until written. */
    TEST_CHECK("S3-U01", imu_driver_status == DRV_IMU_OK,
               "GY-511 found on I2C1");

    /* S3-U02: the IMU task publishes every 20 ms */
    nav_read(&ns);
    {
        UW t1 = ns.i.t_us;
        tk_dly_tsk(60);
        nav_read(&ns);
        TEST_CHECK("S3-U02", ns.i.t_us != t1, "nav_state IMU half updating");
    }
    TEST_CHECK("S3-U03", wcet_tab[WCET_IMU].overruns == 0,
               "IMU loop never overran");

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/*** end of file ***/
