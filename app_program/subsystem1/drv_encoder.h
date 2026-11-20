/*
 *      drv_encoder.h
 *      RIGHT wheel encoder, counted by PIO (pio1 SM0, GP0 / GP1).  Jia Lung.
 *      SDK side (two-sides rule, hal.h).  Plan refs: JL-1.3, JL-1.12, JL-1.13,
 * R7. The LEFT encoder (GP7 / GP28, not consecutive) is in enc_irq.c. Motion
 * Control uses neither directly: it calls encoders.h.
 *
 *      The program is pico-examples' quadrature_encoder.pio (BSD-3-Clause,
 *      unmodified): 24 instructions, must load at offset 0.  With WS2812 (4)
 *      that is 28 of pio1's 32 slots.  CYW43's 6 would NOT also fit, which
 *      is why CYW43 is left alone on pio0 (hw_alloc.h).
 *
 *      The generated quadrature_encoder.pio.h is committed because the
 *      template's makefile has no pioasm step.  Regenerate after editing the
 *      .pio with:  pioasm quadrature_encoder.pio quadrature_encoder.pio.h
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

#define ENC_OK           0
#define ENC_ERR_NO_SPACE (-1) /* offset 0 on pio1 already taken */
#define ENC_ERR_SM_BUSY  (-2)
#define ENC_ERR_PINS     (-3) /* B != A + 1 */

/* Longest drv_encoder_count() waits for a fresh value (normally < 1 us) */
#define ENC_FRESH_TIMEOUT_US 50U

int32_t  drv_encoder_init(void);  /* load FIRST on pio1 */
int32_t  drv_encoder_count(void); /* signed running count; bounded, <= 50 us */
uint32_t drv_encoder_fresh(void); /* 1 = last read got a new value */
uint32_t drv_encoder_program_len(void);

#endif /* DRV_ENCODER_H */

/*** end of file ***/
