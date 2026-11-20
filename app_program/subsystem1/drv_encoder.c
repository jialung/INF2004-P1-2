/*
 *	drv_encoder.c
 *	Right encoder on PIO.  SDK side.                      Owner: Jia Lung
 *	Init follows pico-examples quadrature_encoder.c, without float or
 *	clock_get_hz (hardware_clocks is not linked here).  The read never
 *	blocks (rule R-4, docs/SKELETON.md).
 */

#include "drv_encoder.h"
#include "../hw_alloc.h"
#include "../hal.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "quadrature_encoder.pio.h"

static int	ready;
static uint32_t	last;
static uint32_t	fresh;

int32_t drv_encoder_init(void)
{
	pio_sm_config	c;
	const uint	sm = PIO_SM_ENC_R, pin = PIN_ENC_R_A;

	if (PIN_ENC_R_B != PIN_ENC_R_A + 1) {
		return ENC_ERR_PINS;		/* the program reads pin and pin + 1 */
	}
	if (!pio_can_add_program_at_offset(HW_PIO_APP, &quadrature_encoder_program, 0)) {
		return ENC_ERR_NO_SPACE;	/* R7: log it */
	}
	if (pio_sm_is_claimed(HW_PIO_APP, sm)) {
		return ENC_ERR_SM_BUSY;
	}
	pio_sm_claim(HW_PIO_APP, sm);
	(void)pio_add_program_at_offset(HW_PIO_APP, &quadrature_encoder_program, 0);

	pio_sm_set_consecutive_pindirs(HW_PIO_APP, sm, pin, 2, false);
	pio_gpio_init(HW_PIO_APP, pin);
	pio_gpio_init(HW_PIO_APP, pin + 1);
	gpio_pull_up(pin);
	gpio_pull_up(pin + 1);

	c = quadrature_encoder_program_get_default_config(0);
	sm_config_set_in_pins(&c, pin);			/* WAIT, IN */
	sm_config_set_jmp_pin(&c, pin);			/* JMP PIN */
	sm_config_set_in_shift(&c, false, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
	sm_config_set_clkdiv_int_frac8(&c, 1, 0);	/* full speed */
	pio_sm_init(HW_PIO_APP, sm, 0, &c);
	pio_sm_set_enabled(HW_PIO_APP, sm, true);
	ready = 1;
	return ENC_OK;
}

/* Pop one RX FIFO word, only if there is one.  Two tasks read this
   encoder (Motion Control every 10 ms, Barcode every 1 ms for its
   distance-based widths, 30 Oct), so a task switch between "not empty"
   and the read could let the other task empty the FIFO first, and a read
   from an empty RX FIFO returns undefined data (RP2040 datasheet, RXF).
   Interrupts (so task switches) are off for the check and the read only,
   a few cycles.  PRIMASK by hand: this file is SDK-side and must not
   need the kernel's DI() or a profile-dependent SDK include. */
static int fifo_pop(uint32_t *v)
{
	uint32_t primask;
	int ok;

	__asm volatile ("mrs %0, primask\n\tcpsid i" : "=r"(primask) :: "memory");
	ok = !pio_sm_is_rx_fifo_empty(HW_PIO_APP, PIO_SM_ENC_R);
	if (ok) {
		*v = pio_sm_get(HW_PIO_APP, PIO_SM_ENC_R);
	}
	__asm volatile ("msr primask, %0" :: "r"(primask) : "memory");
	return ok;
}

int32_t drv_encoder_count(void)
{
	uint32_t n, t0, v;

	if (!ready) {
		return 0;
	}
	/* The program pushes the count continuously with "push noblock": the
	   FIFO fills within a microsecond and then holds 4 STALE values while
	   new pushes are dropped.  So:
	     1. discard what is queued (a fixed number: never "until empty" --
	        the state machine refills it faster than we read, which hung
	        Motion Control on the first bench run, 23 Oct);
	     2. wait for one fresh push, at most ENC_FRESH_TIMEOUT_US.
	   pico-examples does step 2 with pio_sm_get_blocking(), which would
	   hang the safety loop for ever if the state machine stopped. */
	n = pio_sm_get_rx_fifo_level(HW_PIO_APP, PIO_SM_ENC_R);
	while (n-- > 0) {
		(void)fifo_pop(&v);
	}
	t0 = hal_now_us();
	while (!fifo_pop(&v)) {
		if ((hal_now_us() - t0) > ENC_FRESH_TIMEOUT_US) {
			fresh = 0;			/* SM stopped: keep the last value */
			return (int32_t)last;
		}
	}
	last = v;
	fresh = 1;
	return (int32_t)v;
}

uint32_t drv_encoder_fresh(void)
{
	return fresh;
}

uint32_t drv_encoder_program_len(void)
{
	return quadrature_encoder_program.length;
}
