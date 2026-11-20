/*
 *      drv_servo.c
 *      SDK side via hal.h.                                    Owner: Lui Khee
 */

#include "drv_servo.h"
#include "../hal.h"
#include "../hw_alloc.h"

#if SERVO_TRIM_US > SERVO_TRIM_MAX_US || SERVO_TRIM_US < -SERVO_TRIM_MAX_US
#error "SERVO_TRIM_US is outside +-SERVO_TRIM_MAX_US: refit the servo horn"
#endif

int32_t drv_servo_init (void)
{
    /* 125 MHz / 125.0 = 1 MHz tick (div16 = 2000), so the compare level
       is the pulse width in microseconds; top = one frame (SERVO_PWM_HZ:
       9999 at 100 Hz, 19999 at 50 Hz). */
    if (hal_pwm_slice_init(PWM_SLICE_SERVO, 2000,
                           1000000U / SERVO_PWM_HZ - 1U) != 0)
    {
        return -1;
    }
    hal_pwm_pin(PIN_SERVO);
    drv_servo_set_deg(SERVO_AHEAD_DEG);
    hal_pwm_enable(PWM_SLICE_SERVO, 1);
    return 0;
}

void drv_servo_set_deg (uint32_t deg)
{
    (void) drv_servo_set_deg_adj(deg, 0);
}

uint32_t drv_servo_set_deg_adj (uint32_t deg, int32_t adj_us)
{
    int32_t us;

    if (deg > 180U)
    {
        deg = 180U;
    }
    us = (int32_t) (SERVO_MIN_US +
                    (deg * (SERVO_MAX_US - SERVO_MIN_US)) / 180U) +
         SERVO_TRIM_US + adj_us;
    if (us < (int32_t) SERVO_MIN_US)
    {
        us = (int32_t) SERVO_MIN_US;
    }
    if (us > (int32_t) SERVO_MAX_US)
    {
        us = (int32_t) SERVO_MAX_US;
    }
    hal_pwm_set(PIN_SERVO, (uint32_t) us);
    return (uint32_t) us;
}

/*** end of file ***/
