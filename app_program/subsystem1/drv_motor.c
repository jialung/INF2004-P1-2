/*
 *      drv_motor.c
 *      SDK side: hal.h only.                                 Owner: Jia Lung
 */

#include "drv_motor.h"
#include "../hal.h"
#include "../hw_alloc.h"

static int32_t last[2];

static uint32_t level_of (int32_t pm)
{
    if (pm < 0)
    {
        pm = -pm;
    }
    if (pm > MOTOR_FULL)
    {
        pm = MOTOR_FULL;
    }
    return ((uint32_t) pm * (MOTOR_PWM_TOP + 1U)) / (uint32_t) MOTOR_FULL;
}

static void set_one (uint32_t pin_a, uint32_t pin_b, int32_t pm)
{
    uint32_t lvl = level_of(pm);

    if (pm >= 0)
    {
        hal_pwm_set(pin_b, 0);
        hal_pwm_set(pin_a, lvl);
    }
    else
    {
        hal_pwm_set(pin_a, 0);
        hal_pwm_set(pin_b, lvl);
    }
}

int32_t drv_motor_init (void)
{
    /* div 1.0 (div16 = 16), top 6249 -> 20 kHz on both slices */
    if (hal_pwm_slice_init(PWM_SLICE_MOTOR_L, 16, MOTOR_PWM_TOP) != 0)
    {
        return -1;
    }
    if (hal_pwm_slice_init(PWM_SLICE_MOTOR_R, 16, MOTOR_PWM_TOP) != 0)
    {
        return -1;
    }
    hal_pwm_pin(PIN_MOTOR_L_A);
    hal_pwm_pin(PIN_MOTOR_L_B);
    hal_pwm_pin(PIN_MOTOR_R_A);
    hal_pwm_pin(PIN_MOTOR_R_B);
    drv_motor_set(0, 0);
    hal_pwm_enable(PWM_SLICE_MOTOR_L, 1);
    hal_pwm_enable(PWM_SLICE_MOTOR_R, 1);
    return 0;
}

void drv_motor_set (int32_t left, int32_t right)
{
    last[MOTOR_LEFT]  = left;
    last[MOTOR_RIGHT] = right;
    set_one(PIN_MOTOR_L_A, PIN_MOTOR_L_B, left);
    set_one(PIN_MOTOR_R_A, PIN_MOTOR_R_B, right);
}

void drv_motor_brake (void)
{
    last[MOTOR_LEFT]  = 0;
    last[MOTOR_RIGHT] = 0;
    hal_pwm_set(PIN_MOTOR_L_A, MOTOR_PWM_TOP + 1U); /* 100 % high */
    hal_pwm_set(PIN_MOTOR_L_B, MOTOR_PWM_TOP + 1U);
    hal_pwm_set(PIN_MOTOR_R_A, MOTOR_PWM_TOP + 1U);
    hal_pwm_set(PIN_MOTOR_R_B, MOTOR_PWM_TOP + 1U);
}

int32_t drv_motor_get (uint32_t side)
{
    return (side <= MOTOR_RIGHT) ? last[side] : 0;
}

/*** end of file ***/
