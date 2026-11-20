/*
 *      encoders.h
 *      Both wheel encoders behind one interface.  Kernel side.  Jia Lung.
 *        right: PIO (drv_encoder.c), GP0 / GP1
 *        left : GPIO edge interrupts + x4 decode table (enc_irq.c), GP7 / GP28
 *      Both give x4 quadrature counts, so the same ENC_COUNTS_PER_REV applies.
 */

#ifndef ENCODERS_H
#define ENCODERS_H

#include "../app.h"

#define ENC_LEFT  0U
#define ENC_RIGHT 1U

/* right PIO first (offset 0), then left IRQs */
IMPORT ER encoders_init(void);
IMPORT W  encoders_count(UINT side); /* signed running count, never blocks */
/* impossible transitions seen (missed edges) */
IMPORT UW   encoders_left_errors(void);
IMPORT UW   encoders_left_isr_max_us(void);
IMPORT UINT encoders_pio_len(void);

#endif /* ENCODERS_H */

/*** end of file ***/
