/*
 *	hal_rp2040.c
 *	SDK SIDE (see hal.h): Pico SDK headers only, no micro T-Kernel headers.
 *	                                                      Owner: Jia Lung
 */

#include "hal.h"
#include "hardware/gpio.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/pwm.h"
#include "hardware/structs/resets.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/nvic.h"
#include "hardware/structs/psm.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/vreg_and_chip_reset.h"

RAMFUNC uint32_t hal_now_us(void)
{
	return timer_hw->timerawl;
}

uint32_t hal_core_id(void)
{
	return sio_hw->cpuid;
}

void hal_busy_wait_us(uint32_t us)
{
	uint32_t t0 = timer_hw->timerawl;
	while ((timer_hw->timerawl - t0) < us) {
		/* spin */
	}
}

/* ---- GPIO ---------------------------------------------------------- */
void hal_gpio_out(uint32_t pin, uint32_t level)
{
	gpio_init(pin);
	gpio_put(pin, level != 0);
	gpio_set_dir(pin, true);
}

void hal_gpio_in(uint32_t pin, uint32_t pull)
{
	gpio_init(pin);
	gpio_set_dir(pin, false);
	gpio_set_pulls(pin, pull == HAL_PULL_UP, pull == HAL_PULL_DOWN);
}

void hal_gpio_put(uint32_t pin, uint32_t level)
{
	if (level) {
		sio_hw->gpio_set = 1UL << pin;
	} else {
		sio_hw->gpio_clr = 1UL << pin;
	}
}

RAMFUNC uint32_t hal_gpio_get(uint32_t pin)
{
	return (sio_hw->gpio_in >> pin) & 1U;
}

RAMFUNC uint32_t hal_gpio_get_all(void)
{
	return sio_hw->gpio_in;
}

#define IRQ_IDX(p)	((p) >> 3)
#define IRQ_SHIFT(p)	(4U * ((p) & 7U))

static RAMFUNC io_bank0_irq_ctrl_hw_t *irq_ctrl(uint32_t core)
{
	return (core == 0) ? &io_bank0_hw->proc0_irq_ctrl
			   : &io_bank0_hw->proc1_irq_ctrl;
}

RAMFUNC uint32_t hal_gpio_irq_status(uint32_t core, uint32_t pin)
{
	return (irq_ctrl(core)->ints[IRQ_IDX(pin)] >> IRQ_SHIFT(pin)) & 0xFU;
}

RAMFUNC void hal_gpio_irq_ack(uint32_t pin, uint32_t events)
{
	/* edges are latched (write 1 to clear); level events are not */
	io_bank0_hw->intr[IRQ_IDX(pin)] = (events & 0xCU) << IRQ_SHIFT(pin);
}

void hal_gpio_irq_enable(uint32_t core, uint32_t pin, uint32_t events)
{
	hal_gpio_irq_ack(pin, 0xFU);
	hw_set_bits(&irq_ctrl(core)->inte[IRQ_IDX(pin)],
		    (events & 0xFU) << IRQ_SHIFT(pin));
}

RAMFUNC void hal_gpio_irq_disable(uint32_t core, uint32_t pin, uint32_t events)
{
	hw_clear_bits(&irq_ctrl(core)->inte[IRQ_IDX(pin)],
		      (events & 0xFU) << IRQ_SHIFT(pin));
	hal_gpio_irq_ack(pin, events);
}

RAMFUNC uint32_t hal_gpio_irq_ints(uint32_t core, uint32_t reg)
{
	return irq_ctrl(core)->ints[reg & 3U];
}

uint32_t hal_gpio_irq_inte(uint32_t core, uint32_t reg)
{
	return irq_ctrl(core)->inte[reg & 3U];
}

/* ---- PWM ----------------------------------------------------------- */
int32_t hal_pwm_slice_init(uint32_t slice, uint32_t div16, uint32_t top)
{
	if (slice > 7 || div16 < 16 || div16 > 0xFFF || top > 0xFFFF) {
		return -1;
	}
	if (resets_hw->reset & RESETS_RESET_PWM_BITS) {
		hw_clear_bits(&resets_hw->reset, RESETS_RESET_PWM_BITS);
		while ((resets_hw->reset_done & RESETS_RESET_PWM_BITS) == 0) {
			/* wait */
		}
	}
	pwm_hw->slice[slice].csr = 0;			/* off while set up */
	pwm_hw->slice[slice].div = div16;		/* INT[11:4] FRAC[3:0] */
	pwm_hw->slice[slice].top = top;
	pwm_hw->slice[slice].cc  = 0;			/* both outputs low */
	pwm_hw->slice[slice].ctr = 0;
	return 0;
}

void hal_pwm_pin(uint32_t pin)
{
	gpio_set_function(pin, GPIO_FUNC_PWM);
}

uint32_t hal_pwm_slice_of(uint32_t pin)
{
	return (pin >> 1) & 7U;
}

void hal_pwm_set(uint32_t pin, uint32_t level)
{
	uint32_t slice = hal_pwm_slice_of(pin);

	if (pin & 1U) {		/* channel B: cc[31:16] */
		hw_write_masked(&pwm_hw->slice[slice].cc, level << 16, 0xFFFF0000U);
	} else {		/* channel A: cc[15:0] */
		hw_write_masked(&pwm_hw->slice[slice].cc, level, 0x0000FFFFU);
	}
}

uint32_t hal_pwm_get(uint32_t pin)
{
	uint32_t cc = pwm_hw->slice[hal_pwm_slice_of(pin)].cc;

	return (pin & 1U) ? (cc >> 16) : (cc & 0xFFFFU);
}

uint32_t hal_pwm_csr(uint32_t slice) { return pwm_hw->slice[slice & 7U].csr; }
uint32_t hal_pwm_top(uint32_t slice) { return pwm_hw->slice[slice & 7U].top; }
uint32_t hal_pwm_div(uint32_t slice) { return pwm_hw->slice[slice & 7U].div; }

RAMFUNC uint32_t hal_gpio_funcsel(uint32_t pin)
{
	return io_bank0_hw->io[pin].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS;
}

/* Sample the pad n times; spacing not a divisor of a PWM period, so a
   toggling pin shows edges whatever its frequency (below ~50 kHz) */
uint32_t hal_gpio_edges(uint32_t pin, uint32_t n, uint32_t spacing_us)
{
	uint32_t i, e = 0, last = gpio_get(pin), v, t;

	for (i = 0; i < n; i++) {
		t = timer_hw->timerawl;
		while ((timer_hw->timerawl - t) < spacing_us) { }
		v = gpio_get(pin);
		if (v != last) e++;
		last = v;
	}
	return e;
}

void hal_pwm_enable(uint32_t slice, uint32_t on)
{
	if (on) {
		hw_set_bits(&pwm_hw->slice[slice].csr, 1U);
	} else {
		hw_clear_bits(&pwm_hw->slice[slice].csr, 1U);
	}
}

/* ---- post-mortem watch ---------------------------------------------- */
RAMFUNC void hal_alarm_set(uint32_t alarm, uint32_t t_us)
{
	timer_hw->alarm[alarm & 3U] = t_us;	/* writing arms it */
}

RAMFUNC void hal_alarm_ack(uint32_t alarm)
{
	timer_hw->intr = 1U << (alarm & 3U);
}

void hal_alarm_irq_on_this_core(uint32_t alarm)
{
	uint32_t irq = alarm & 3U;		/* TIMER_IRQ_n = IRQ n */

	/* priority 0 (highest): IPR byte = 0 */
	nvic_hw->ipr[irq >> 2] &= ~(0xFFU << (8U * (irq & 3U)));
	timer_hw->intr = 1U << irq;
	hw_set_bits(&timer_hw->inte, 1U << irq);
	nvic_hw->icpr = 1U << irq;
	nvic_hw->iser = 1U << irq;		/* this core's NVIC only */
}

void hal_alarm_irq_all_off(void)
{
	hw_clear_bits(&timer_hw->inte, (1U << 2) | (1U << 3));
	timer_hw->intr = (1U << 2) | (1U << 3);
	nvic_hw->icer = (1U << 2) | (1U << 3);
	nvic_hw->icpr = (1U << 2) | (1U << 3);
}

void hal_reboot(void)
{
	/* as the SDK's watchdog_reboot(): reset everything but the oscillators */
	hw_set_bits(&psm_hw->wdsel, PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));
	watchdog_hw->ctrl = WATCHDOG_CTRL_TRIGGER_BITS;
	for (;;) {
		__asm volatile ("nop");
	}
}

uint32_t hal_reset_cause(void)
{
	uint32_t c = vreg_and_chip_reset_hw->chip_reset, r = 0;

	if (c & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) r |= 1U;
	if (c & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS) r |= 2U;
	if (watchdog_hw->reason != 0) r |= 4U;
	return r;
}

uint32_t hal_scratch_get(uint32_t i)
{
	return watchdog_hw->scratch[i & 3U];
}

void hal_scratch_set(uint32_t i, uint32_t v)
{
	watchdog_hw->scratch[i & 3U] = v;
}
