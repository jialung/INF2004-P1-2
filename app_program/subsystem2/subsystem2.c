/*
 *	subsystem2.c
 *	Line / barcode: hardware init and task start.               Owner: Ryan
 */

#include "subsystem2.h"
#include "drv_adc.h"
#include "../hal.h"

EXPORT ID tid_line;
EXPORT ID tid_barcode;
/* Power-up thresholds, from the calibration after the sensor height was
 * changed, 30 Oct (evening):
 *   "[line] calibrate OK: left 117..2784 -> thr 1450,
 *    barcode 110..2336 -> thr 1223, right (GP17) black 173 of 3000"
 * A calibration (MODE held 2 s) still replaces them until power-off.
 * Recalibrate and update these if the floor, lighting or sensor height
 * changes.
 *
 * Barcode: with BC_ADAPTIVE (task_barcode.c) edges come from rises and
 * falls of BC_SWING; thr_bc only sets the floor a bar must reach
 * (thr_bc - 200 = 1023) and the colour at the start.  A low floor is the
 * safe side for weak narrow bars; white paper reads 110-190.
 * (Earlier height: paper 177, wide bar 3043, narrow bar 2036, narrow gap
 * 990; re-measure these four at the new height to check BC_SWING.) */
EXPORT IR_CAL ir_cal = { 1450, 1223, 1 };

EXPORT ER subsystem2_init(void)
{
	INT r;

	/* Right line sensor: analog output on a digital pin (hw_alloc.h) */
	hal_gpio_in(PIN_IR_LINE_R, HAL_PULL_NONE);

	/* Left line sensor (ADC0) and barcode sensor (ADC1) */
	if (drv_adc_init() != DRV_ADC_OK) {
		return E_SYS;
	}
	r = drv_adc_ring_start();
	if (r == DRV_ADC_ERR_TODO) {
		LOG("line", "ADC sampled every %d ms by the barcode task "
		    "(Code 39 A-Z, both directions)\n", (INT)PERIOD_BARCODE_MS);
	} else if (r < 0) {
		return E_SYS;
	} else {
		LOG("line", "ADC ring on DMA ch %d (R6)\n", (INT)drv_adc_ring_dma_channel());
	}
	return E_OK;
}

EXPORT ER subsystem2_start(void)
{
	tid_line = app_start_periodic(line_task, PRI_LINE, CORE_LINE,
				      PERIOD_LINE_MS, "line", NULL);
	if (tid_line < E_OK) return tid_line;
	tid_barcode = app_start_periodic(barcode_task, PRI_BARCODE, CORE_BARCODE,
					 PERIOD_BARCODE_MS, "barcode", NULL);
	return (tid_barcode < E_OK) ? tid_barcode : E_OK;
}
