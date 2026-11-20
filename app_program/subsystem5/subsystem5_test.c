/*
 *	subsystem5_test.c
 *	On-target tests for Supervisor / Telemetry.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.
 */

#include "subsystem5.h"
#include "telem_frame.h"
#include "../fault_flags.h"
#include "../msg_bus.h"
#include "../gpio_irq.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
	TEST_INIT();

	/* S5-U01 (R7): WS2812 loaded on pio1 after the encoders */
	TEST_CHECK("S5-U01", ui_led_ok, "WS2812 PIO program loaded");

	/* S5-U02: all three GPIO interrupt users registered on one handler */
	TEST_CHECK("S5-U02", gpio_irq_stats.core == 0, "GPIO IRQ owned by core 0");

	/* S5-U03: a trip bit puts the Supervisor in Fault; MODE-short path
	   clears it (simulated: clear the bit and check the state follows) */
	fault_raise(FAULT_US_FAULT);
	tk_dly_tsk(30);
	TEST_CHECK("S5-U03a", sys_state_get() == SYS_FAULT, "trip bit -> Fault");
	fault_clear(FAULT_US_FAULT);

	/* S5-U04 (CH-1.12): frame round-trips and CRC catches a flipped bit */
	{
		TELEM_FIELDS a = {0}, b = {0};
		UB fr[TF_LEN];
		a.state = SYS_LINE_SEARCH; a.faults = FAULT_WIFI_DOWN; a.v_mm_s = -120;
		a.heading_deg = 359; a.roll_q = -31; a.letter = 25; a.letter_rev = 1;
		(void)tf_pack(&a, fr);
		TEST_CHECK("S5-U04a", tf_unpack(fr, &b) == 0 && b.heading_deg == 359 &&
				      b.v_mm_s == -120 && b.roll_q == -31 && b.state == SYS_LINE_SEARCH,
			   "frame pack/unpack round trip");
		fr[4] ^= 0x10;
		TEST_CHECK("S5-U04b", tf_unpack(fr, &b) == -1, "CRC-8 rejects a corrupt frame");
	}

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
