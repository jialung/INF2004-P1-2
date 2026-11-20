/*
 *	subsystem4_test.c
 *	On-target tests for Obstacle Scan.  Built only with: make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md.
 */

#include "subsystem4.h"
#include "../gpio_irq.h"
#include "../msg_bus.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	UW	cm = 0, isr0;
	US_RESULT r;

	TEST_INIT();

	/* S4-U01 (R5): ECHO edges reach our handler through gpio_irq.c.
	   Point the sensor at a wall ~30 cm away. */
	isr0 = echo_isr_count;
	r = us_ping(&cm);
	tm_printf((UB*)"[TEST] S4-U01 ping result %d, %d cm, isr %d\n",
		  (INT)r, (INT)cm, (INT)(echo_isr_count - isr0));
	TEST_CHECK("S4-U01", echo_isr_count != isr0, "ECHO interrupt fired");
	TEST_CHECK("S4-U02", r != US_NO_RISE, "HC-SR04 answered");
	tm_printf((UB*)"[TEST] S4-U03 GPIO handler max %d us over %d entries\n",
		  (INT)gpio_irq_stats.max_us, (INT)gpio_irq_stats.count);
	TEST_CHECK("S4-U03", gpio_irq_stats.max_us < 20, "GPIO handler < 20 us");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
