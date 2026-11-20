/*
 *	gpio_irq.h
 *	The ONE handler for IO_IRQ_BANK0 (all GPIO interrupts).
 *	                                Owner: Christian (Lui Khee tests, R5)
 *	Plan refs: CH-1.8, CH-1.20, LK-1.8, CH-1.37 / LK-1.27.
 *
 *	CH-1.8 answered: in this port the kernel owns the vector table.
 *	tk_def_int() is the only way in.  The SDK's
 *	gpio_set_irq_enabled_with_callback() and gpio_add_raw_irq_handler()
 *	compile, link, and then never fire (irq_* are stubs in
 *	cyw43_utk_compat.c).  So every GPIO interrupt goes through here.
 *	BANNED in code review: gpio_set_irq_enabled_with_callback,
 *	gpio_set_irq_callback, gpio_add_raw_irq_handler*, irq_set_exclusive_handler.
 *
 *	Usage (at init, before tasks start):
 *	    gpio_irq_register(PIN_US_ECHO, GPIO_EV_RISE | GPIO_EV_FALL, echo_isr);
 *	The dispatcher checks registered pins in registration order, so the
 *	busiest (left encoder) registers first: S1 inits first.
 *	The callback runs in interrupt context: no waiting, no tm_printf.
 *	It gets the edge bits and a microsecond timestamp taken on entry.
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include <tk/tkernel.h>

/* Event bits (RP2040 IO_BANK0 encoding).  Only edges are latched. */
#define GPIO_EV_LEVEL_LOW	0x1U
#define GPIO_EV_LEVEL_HIGH	0x2U
#define GPIO_EV_FALL		0x4U
#define GPIO_EV_RISE		0x8U

typedef void (*GPIO_IRQ_FN)(UINT gpio, UINT events, UW t_us);

/* Pins allowed to interrupt (hw_alloc.h): GP5 echo, GP7 + GP28 left
   encoder, GP20 / GP21 buttons */
#define GPIO_IRQ_ALLOWED_MASK	((1UL << 5) | (1UL << 7) | (1UL << 20) | \
				 (1UL << 21) | (1UL << 28))
#define GPIO_IRQ_MAX_USERS	8

IMPORT ER gpio_irq_init(void);	/* once, from usermain, before registering */
IMPORT ER gpio_irq_register(UINT gpio, UINT events, GPIO_IRQ_FN fn);

typedef struct {
	UW	count;			/* handler entries */
	UW	spurious;		/* entries with no registered pin pending */
	UW	max_us;			/* longest handler run */
	UINT	core;			/* core whose NVIC owns the IRQ */
	UINT	spurious_last_pin;	/* unregistered pin last switched off (0xFF none) */
	UW	killed_mask;		/* every unregistered pin switched off */
} GPIO_IRQ_STATS;
IMPORT volatile GPIO_IRQ_STATS gpio_irq_stats;

#endif /* GPIO_IRQ_H */
