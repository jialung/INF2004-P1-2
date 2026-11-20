/*
 *      drv_adc.c
 *      SDK side.                                                Owner: Ryan
 */

#include "drv_adc.h"
#include "../hw_alloc.h"
#include "hardware/gpio.h"
#include "hardware/structs/adc.h"
#include "hardware/structs/resets.h"
#include "hardware/regs/adc.h"

static void pin_analog (uint32_t pin)
{
    /* ADC pins: no digital input, no pulls (pico-sdk adc_gpio_init) */
    gpio_set_function(pin, GPIO_FUNC_NULL);
    gpio_disable_pulls(pin);
    gpio_set_input_enabled(pin, false);
}

int32_t drv_adc_init (void)
{
    if (resets_hw->reset & RESETS_RESET_ADC_BITS)
    {
        hw_clear_bits(&resets_hw->reset, RESETS_RESET_ADC_BITS);
        while ((resets_hw->reset_done & RESETS_RESET_ADC_BITS) == 0)
        {
            /* wait */
        }
    }
    adc_hw->cs = ADC_CS_EN_BITS;
    while ((adc_hw->cs & ADC_CS_READY_BITS) == 0)
    {
        /* wait for the ADC to be ready */
    }
    pin_analog(PIN_IR_LINE_L);  /* GP26 = ADC0 */
    pin_analog(PIN_IR_BARCODE); /* GP27 = ADC1; GP28 stays digital (encoder) */
    return DRV_ADC_OK;
}

uint32_t drv_adc_read (uint32_t ch)
{
    hw_write_masked(&adc_hw->cs, ch << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
    hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
    while ((adc_hw->cs & ADC_CS_READY_BITS) == 0)
    {
        /* ~2 us at 48 MHz */
    }
    return adc_hw->result & 0xFFFU;
}

/* ---- DMA ring: TODO RY-1.5, RY-1.6, RY-1.7 ------------------------- */
int32_t drv_adc_ring_start (void)
{
    return DRV_ADC_ERR_TODO;
}
uint32_t drv_adc_ring_head (void)
{
    return 0;
}
uint16_t drv_adc_ring_at (uint32_t abs_index)
{
    (void) abs_index;
    return 0;
}
int32_t drv_adc_ring_dma_channel (void)
{
    return -1;
}

/*** end of file ***/
