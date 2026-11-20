/*
 *      drv_ui.c
 *      SDK side.                                             Owner: Christian
 */

#include "drv_ui.h"
#include "../hal.h"
#include "../hw_alloc.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

static int led_ready;

int32_t drv_led_init (void)
{
    pio_sm_config c;
    int           offset;

    if (!pio_can_add_program(HW_PIO_APP, &ws2812_program))
    {
        return UI_ERR_NO_SPACE;
    }
    if (pio_sm_is_claimed(HW_PIO_APP, PIO_SM_WS2812))
    {
        return UI_ERR_SM_BUSY;
    }
    pio_sm_claim(HW_PIO_APP, PIO_SM_WS2812);
    offset = pio_add_program(HW_PIO_APP, &ws2812_program);

    pio_gpio_init(HW_PIO_APP, PIN_RGB_LED);
    pio_sm_set_consecutive_pindirs(HW_PIO_APP, PIO_SM_WS2812, PIN_RGB_LED, 1,
                                   true);
    c = ws2812_program_get_default_config((uint) offset);
    sm_config_set_sideset_pins(&c, PIN_RGB_LED);
    sm_config_set_out_shift(&c, false, true, 24); /* GRB, 24 bit autopull */
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    /* 800 kHz x (T1+T2+T3 = 10) cycles = 8 MHz: 125 / 8 = 15.625
       = 15 + 160/256 */
    sm_config_set_clkdiv_int_frac8(&c, 15, 160);
    pio_sm_init(HW_PIO_APP, PIO_SM_WS2812, (uint) offset, &c);
    pio_sm_set_enabled(HW_PIO_APP, PIO_SM_WS2812, true);
    led_ready = 1;
    drv_led_rgb(0, 0, 0);
    return UI_OK;
}

void drv_led_rgb (uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t grb = ((uint32_t) g << 16) | ((uint32_t) r << 8) | b;

    if (led_ready && !pio_sm_is_tx_fifo_full(HW_PIO_APP, PIO_SM_WS2812))
    {
        pio_sm_put(HW_PIO_APP, PIO_SM_WS2812, grb << 8);
    }
}

int32_t drv_buzzer_init (void)
{
    /* 125 MHz / 62.5 (div16 = 1000) / 1000 = 2 kHz */
    if (hal_pwm_slice_init(PWM_SLICE_BUZZER, 1000, 999) != 0)
    {
        return -1;
    }
    hal_pwm_pin(PIN_BUZZER);
    hal_pwm_set(PIN_BUZZER, 0);
    hal_pwm_enable(PWM_SLICE_BUZZER, 1);
    return UI_OK;
}

void drv_buzzer (uint32_t on)
{
    hal_pwm_set(PIN_BUZZER, on ? 500U : 0U); /* 50 % duty */
}

/*** end of file ***/
