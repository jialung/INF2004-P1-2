/*
 *      drv_ui.h
 *      Status RGB LED (WS2812, GP18, pio1 SM2) and piezo buzzer (GP22,
 *      PWM slice 3A).  Owner: Christian.  SDK side.  Plan refs: CH-1.5, CH-1.6,
 * CH-1.9.
 *
 *      ws2812.pio is pico-examples' program (BSD-3-Clause, unmodified, 4
 *      instructions).  It loads on pio1 AFTER the encoders (R7).
 */

#ifndef DRV_UI_H
#define DRV_UI_H

#include <stdint.h>

#define UI_OK           0
#define UI_ERR_NO_SPACE (-1)
#define UI_ERR_SM_BUSY  (-2)

int32_t drv_led_init(void);
void    drv_led_rgb(uint8_t r, uint8_t g, uint8_t b); /* never blocks */

int32_t drv_buzzer_init(void);
void    drv_buzzer(uint32_t on); /* 2 kHz tone on / off */

#endif /* DRV_UI_H */

/*** end of file ***/
