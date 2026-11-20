/*
 *	hal.h
 *	Small hardware layer shared by everyone.               Owner: Jia Lung
 *
 *	THE TWO-SIDES RULE (read this before writing any driver)
 *	-------------------------------------------------------
 *	micro T-Kernel's headers and the Pico SDK's headers cannot be in the
 *	same .c file: tk/syslib.h makes size_t a 'long', the SDK makes it an
 *	'unsigned int', and both define PWM_BASE, TIMER_BASE, SIO_BASE ...
 *	So every .c file is on exactly one side:
 *
 *	  kernel side   task code, kernel objects    #include "app.h"
 *	                (any file not named below)   never "hardware/..." or "pico/..."
 *	  SDK side      hal_*.c and drv_*.c          #include "hardware/..."
 *	                                             never app.h or <tk/...>
 *
 *	The two sides talk only through headers like this one, which use
 *	plain <stdint.h> types and no bool.  (The template's own CYW43 port
 *	does the same: cyw43_utk.c declares the tk_* calls it needs by hand.)
 *	A driver that must signal a task (e.g. ECHO -> semaphore) exposes a
 *	callback; the kernel-side code passes in a function that does the
 *	tk_* call.
 */

#ifndef HAL_H
#define HAL_H

#include <stdint.h>

/* Code that runs in interrupt context and must not wait for a flash (XIP)
   cache miss: copied to RAM at boot (the linker script puts .data.* in
   RAM).  First bench run, 23 Oct: the GPIO handler took up to 36 us from
   flash.  Use it on the dispatcher, ISR callbacks and what they call. */
#define RAMFUNC	__attribute__((noinline, section(".data.ramfunc")))

/* ---- time and cores ------------------------------------------------ */
uint32_t hal_now_us(void);		/* 1 MHz timer, low 32 bits */
uint32_t hal_core_id(void);		/* 0 or 1 */
void     hal_busy_wait_us(uint32_t us);	/* short delays only (TRIG pulse) */

/* ---- GPIO ---------------------------------------------------------- */
#define HAL_PULL_NONE	0
#define HAL_PULL_UP	1
#define HAL_PULL_DOWN	2
void     hal_gpio_out(uint32_t pin, uint32_t level);	/* make output */
void     hal_gpio_in(uint32_t pin, uint32_t pull);	/* make input */
void     hal_gpio_put(uint32_t pin, uint32_t level);
uint32_t hal_gpio_get(uint32_t pin);
uint32_t hal_gpio_get_all(void);		/* all 30 inputs, one read */

/* GPIO interrupt registers, used only by gpio_irq.c */
uint32_t hal_gpio_irq_status(uint32_t core, uint32_t pin);	/* 4 event bits */
void     hal_gpio_irq_ack(uint32_t pin, uint32_t events);
void     hal_gpio_irq_enable(uint32_t core, uint32_t pin, uint32_t events);
void     hal_gpio_irq_disable(uint32_t core, uint32_t pin, uint32_t events);
uint32_t hal_gpio_irq_ints(uint32_t core, uint32_t reg);	/* INTS0..3 raw */
uint32_t hal_gpio_irq_inte(uint32_t core, uint32_t reg);	/* INTE0..3 raw */

/* ---- PWM (clk_sys = 125 MHz) ---------------------------------------
 * f_pwm = 125 MHz / (div * (top + 1)).  div is 8.4 fixed point here as
 * div16 = div * 16, e.g. 125.0 -> 2000.
 * The template leaves PWM in reset (only USE_PTMR releases it), so
 * hal_pwm_slice_init() releases it on first use.                      */
int32_t  hal_pwm_slice_init(uint32_t slice, uint32_t div16, uint32_t top);
void     hal_pwm_pin(uint32_t pin);			/* pin -> PWM function */
void     hal_pwm_set(uint32_t pin, uint32_t level);	/* compare level */
void     hal_pwm_enable(uint32_t slice, uint32_t on);
uint32_t hal_pwm_slice_of(uint32_t pin);
uint32_t hal_pwm_get(uint32_t pin);			/* compare level read back */
uint32_t hal_pwm_csr(uint32_t slice);			/* bit0 = enabled */
uint32_t hal_pwm_top(uint32_t slice);
uint32_t hal_pwm_div(uint32_t slice);			/* 8.4 fixed point */
uint32_t hal_gpio_funcsel(uint32_t pin);		/* 4 PWM, 5 SIO, 6 PIO0, 7 PIO1 */
uint32_t hal_gpio_edges(uint32_t pin, uint32_t n, uint32_t spacing_us);	/* level changes seen */

/* ---- post-mortem watch (diag_pm.c) ---------------------------------
 * A 1 ms timer alarm per core at NVIC priority 0, outside the kernel's
 * interrupt layer, so it still samples when a core is buried in tasks or
 * lower-priority interrupts.  Alarm 2 = core 0, alarm 3 = core 1
 * (pico_time is not linked; nothing else uses the alarms).            */
void     hal_alarm_set(uint32_t alarm, uint32_t t_us);	/* next match time */
void     hal_alarm_ack(uint32_t alarm);
void     hal_alarm_irq_on_this_core(uint32_t alarm);	/* INTE + NVIC prio 0 */
void     hal_alarm_irq_all_off(void);			/* alarms 2 and 3, at boot */
void     hal_reboot(void);				/* watchdog, full chip */
uint32_t hal_reset_cause(void);		/* bit0 power-on, bit1 RUN pin, bit2 watchdog */
uint32_t hal_scratch_get(uint32_t i);	/* watchdog scratch 0..3, survive a reboot */
void     hal_scratch_set(uint32_t i, uint32_t v);

#endif /* HAL_H */
