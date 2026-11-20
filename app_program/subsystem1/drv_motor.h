/*
 *      drv_motor.h
 *      Robo Pico motor driver, 2 PWM inputs per motor, 20 kHz.  Jia Lung.
 *      SDK side (two-sides rule, hal.h).  Called ONLY by Motion Control.
 *      Plan refs: JL-1.2.
 *
 *      Drive table (check on the bench, JL-1.2):
 *        forward  A = duty, B = 0      reverse  A = 0, B = duty
 *        brake    A = B = 100 %        coast    A = B = 0
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>

#define MOTOR_LEFT    0U
#define MOTOR_RIGHT   1U
#define MOTOR_PWM_TOP 6249U /* 125 MHz / 1.0 / 6250 = 20 kHz */
#define MOTOR_FULL    1000  /* duty unit: per mille */

int32_t drv_motor_init(void);                       /* 0 = ok */
void    drv_motor_set(int32_t left, int32_t right); /* -1000..1000 */
void    drv_motor_brake(void);
int32_t drv_motor_get(uint32_t side); /* last value set (tests) */

#endif /* DRV_MOTOR_H */

/*** end of file ***/
