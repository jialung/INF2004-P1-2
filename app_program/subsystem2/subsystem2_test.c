/*
 *	subsystem2_test.c
 *	On-target tests for Line / Barcode.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 */

#include "subsystem2.h"
#include "drv_adc.h"
#include "../hal.h"
#include "../msg_bus.h"
#include "../wcet.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	UW	le1, le2, v, b;

	TEST_INIT();

	/* S2-U01: raw readings of all three IR sensors (RY-1.3).  Run once
	   over white and once over black tape and record both. */
	v = drv_adc_read(ADC_CH_LINE_L);
	b = drv_adc_read(ADC_CH_BARCODE);
	tm_printf((UB*)"[TEST] S2-U01 line L (ADC0) = %d, line R (GP17) = %d, "
		  "barcode (ADC1) = %d\n",
		  (INT)v, (INT)hal_gpio_get(PIN_IR_LINE_R), (INT)b);
	TEST_CHECK("S2-U01", v <= 4095 && b <= 4095, "ADC0 / ADC1 readings in range");

	/* S2-U02: the line error word is fresh every 5 ms */
	le1 = line_err_read();
	tk_dly_tsk(20);
	le2 = line_err_read();
	TEST_CHECK("S2-U02", LE_SEQ(le1) != LE_SEQ(le2) && (LE_FLAGS(le2) & LE_VALID),
		   "line error published and updating");

	/* S2-U03: Line Following keeps its 5 ms period */
	TEST_CHECK("S2-U03", wcet_tab[WCET_LINE].jitter_max_us <= 500,
		   "line loop jitter <= 500 us");

	/* S2-U04 (RY-1.26 seed): a burst of droppable events never costs a
	   must-keep event */
	{
		INT i;
		UW lost0 = bus_stats.sup_lost_keep;
		for (i = 0; i < 40; i++) {
			(void)sup_post(SRC_LINE, EV_LINE_SAMPLE, 0, 0);
		}
		(void)sup_post(SRC_LINE, EV_BARCODE, BC_A(0, 0), 0);
		TEST_CHECK("S2-U04", bus_stats.sup_lost_keep == lost0,
			   "barcode event survives a burst of samples");
	}

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
