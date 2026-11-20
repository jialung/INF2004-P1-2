/*
 *	hw_alloc.h
 *	Hardware allocation: every pin, PWM slice, PIO block, DMA rule and
 *	interrupt user in one place.                          Owner: Jia Lung
 *	Changed only by team agreement.  Plan refs: JL-1.11, R5, R6, R7.
 *	Source of the pin numbers: Week 6 Design Review, section 2.
 *
 *	Rule: if it is not in this file, nobody may touch it.
 */

#ifndef HW_ALLOC_H
#define HW_ALLOC_H

/* ---------------------------------------------------------------------
 *  Pins (Robo Pico + Pico W).  Revision 2, 23 Oct 2026: encoders moved to
 *  Grove 1 (right) and Grove 7 (left); IR sensors on GP26 / GP27 / GP17,
 *  AO outputs only (DO left unconnected).
 * --------------------------------------------------------------------- */
/* Right wheel encoder, Grove 1, PIO (B = A + 1)       -- Jia Lung   */
#define PIN_ENC_R_A		0	/* takes UART0 TX: no CRASH_REPORT=1 builds */
#define PIN_ENC_R_B		1	/* takes UART0 RX */
/* IMU GY-511 on I2C1, Maker port                     -- Jian Yi    */
#define PIN_IMU_SDA		2
#define PIN_IMU_SCL		3
/* HC-SR04 ultrasonic                                 -- Lui Khee   */
#define PIN_US_TRIG		4
#define PIN_US_ECHO		5	/* 1 k / 2 k divider to 3.3 V */
/* GP6: free */
/* Left wheel encoder, Grove 7.  GP7 and GP28 are neighbours on the plug
   but NOT consecutive GPIO numbers, so the PIO program cannot count it:
   counted by edge interrupts instead (subsystem1/enc_irq.c). -- Jia Lung */
#define PIN_ENC_L_A		7
#define PIN_ENC_L_B		28	/* ADC-capable pin, used as digital input */
/* Motors, Robo Pico on-board driver: xA = forward, xB = reverse -- Jia Lung */
#define PIN_MOTOR_L_A		8	/* M1A, PWM slice 4A */
#define PIN_MOTOR_L_B		9	/* M1B, PWM slice 4B */
#define PIN_MOTOR_R_A		10	/* M2A, PWM slice 5A */
#define PIN_MOTOR_R_B		11	/* M2B, PWM slice 5B */
/* GP12, GP13, GP14: free (servo headers) */
/* Scan servo SG90                                    -- Lui Khee   */
#define PIN_SERVO		15	/* PWM slice 7B */
/* GP16: free */
/* Right IR line sensor: its ANALOG output on a NON-ADC pin.  GP17 reads
   it as digital: the pad's Schmitt input decides black / white at fixed
   thresholds (low < 0.8 V, high > 2.0 V, undefined between), so it cannot
   be calibrated in software -- adjust sensor height instead.  The module
   MUST be powered from 3.3 V: a 5 V AO would exceed the pin's rating. -- Ryan */
#define PIN_IR_LINE_R		17	/* AO read as digital */
/* Supervisor user interface                          -- Christian  */
#define PIN_RGB_LED		18	/* WS2812, PIO */
#define PIN_WCET_PROBE		19	/* free pin: scope probe for WCET / jitter */
#define PIN_BTN_START		20	/* active low, on-board */
#define PIN_BTN_MODE		21	/* active low, on-board */
#define PIN_BUZZER		22	/* PWM slice 3A */
/* GP23, GP24, GP25, GP29: Pico W radio. NEVER use (GP29 = VSYS/3 too). */
/* IR sensors on ADC (Grove 6), AO only                  -- Ryan       */
#define PIN_IR_LINE_L		26	/* ADC0: left line sensor */
#define PIN_IR_BARCODE		27	/* ADC1: barcode sensor */

#define ADC_CH_LINE_L		0
#define ADC_CH_BARCODE		1

/* ---------------------------------------------------------------------
 *  PWM slices (clk_sys = 125 MHz in this port, cpu_clock.c)
 *  Slice 2 is free.  Slice 3A is the buzzer (GP6, also 3A, is unused);
 *  GP7 is on slice 3B but stays a plain GPIO input.
 *  Keep USE_PTMR = 0 in config.h: the kernel's physical timer would
 *  take a PWM slice of its own.
 * --------------------------------------------------------------------- */
#define PWM_SLICE_BUZZER	3	/* channel A */
#define PWM_SLICE_MOTOR_L	4
#define PWM_SLICE_MOTOR_R	5
#define PWM_SLICE_SERVO		7	/* channel B */

/* ---------------------------------------------------------------------
 *  PIO (R7).  CYW43 claims "the first PIO with a free SM and room for its
 *  6-instruction program", trying pio0 first, from its own kernel task
 *  that may start before or after usermain().  Putting OUR programs on
 *  pio1 makes the result the same whichever runs first:
 *    pio0 : CYW43 (claims itself)
 *    pio1 : SM0 = right encoder (program at offset 0, GP0/GP1),
 *           SM2 = WS2812 status LED.  SM1 free (left encoder is on
 *           interrupts: its pins are not consecutive).
 *  Load order on pio1: encoder first (it needs offset 0), then WS2812.
 * --------------------------------------------------------------------- */
#define HW_PIO_APP		pio1
#define PIO_SM_ENC_R		0
#define PIO_SM_WS2812		2

/* ---------------------------------------------------------------------
 *  DMA (R6).  Never hard-code a channel.  Always
 *      int ch = dma_claim_unused_channel(true);
 *  and log it at boot.  CYW43 claims two channels the same way.
 *  Users: CYW43 (2), ADC ring (Ryan, 1: ADC0 + ADC1 round robin).
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Interrupts.  The vector table belongs to the kernel: use tk_def_int()
 *  and EnableInt() only.  The SDK's irq_* and gpio_set_irq_callback /
 *  gpio_add_raw_irq_handler calls are STUBS in this port and do nothing
 *  (lib/libwifi/.../cyw43_utk_compat.c), so a driver that uses them
 *  builds fine and then never gets an interrupt (R5).
 *
 *  IO_IRQ_BANK0 (13): one handler, gpio_irq.c (Christian). Users:
 *      GP5 ECHO, GP20 START, GP21 MODE, GP7 + GP28 left encoder
 *      -- all both edges.
 *      The left encoder is the busiest user (4 edges per encoder line);
 *      its callback must stay a few microseconds (S1-U04 measures it).
 *      GP24 is NOT a user: CYW43 runs in polling mode in this port.
 *  Interrupt levels for EnableInt(): 1 (highest) .. 3 (= SysTick).
 * --------------------------------------------------------------------- */
#define INTNO_GPIO_BANK0	13	/* IO_IRQ_BANK0 */
#define INTLVL_GPIO		1	/* echo timing needs the best latency */

/* ---------------------------------------------------------------------
 *  Robot geometry.  PLACEHOLDERS until measured (JL-1.4, JL-1.5, D2).
 *  One copy here so Motion (odometry), IMU (hump height) and the bypass
 *  plan (car width W) cannot disagree.
 * --------------------------------------------------------------------- */
/* Distance calibration, 30 Oct (JL-1.4): the car pushed 500 mm along a
   ruler read 842 mm with 1440 counts / rev, so the effective count is
   1440 x 842 / 500 = 2425 (wheel diameter kept at 65).  Every distance on
   the car uses this: odometer, speed, x / y, MC_DRIVE, line lost,
   junction drive, line search, bypass legs, barcode widths. */
#define ENC_COUNTS_PER_REV	2425	/* effective, measured (was 1440) */
#define WHEEL_DIAMETER_MM	65
#define TRACK_WIDTH_MM		113	/* measured: wheel centre to centre */
/* Effective track width for the ENCODER HEADING (pivots, x / y), in
   0.1 mm.  Was 671 (67.1 mm, kept from the old 1440-count turn scale).
   6 Nov: the calibration spin asked for 3 x 200 = 600 deg and the car
   turned only ~330 deg (all 3 pivots finished normally), so the encoders
   counted turns ~1.8 x too high.  671 x 1.8 = 1208 (120.8 mm), a little
   above the 113 mm between the wheels, as expected from wheel scrub in
   pivots.  ESTIMATED by eye: TODO JL-1.5, check a measured 360 deg pivot
   and set  new = 1208 x (angle asked) / (angle turned).                */
#define TURN_TRACK_X10		1208
/* Width.  The body is 150 mm at its widest point (the front), and the
   barcode sensor sticks out 30 mm past the body on the LEFT only, so
   the car is not symmetric about its centre line. */
#define CAR_BODY_WIDTH_MM	150	/* measured: widest point of the body */
#define BARCODE_OUT_LEFT_MM	30	/* measured: barcode sensor past the body */
#define CAR_HALF_LEFT_MM	(CAR_BODY_WIDTH_MM / 2 + BARCODE_OUT_LEFT_MM)
#define CAR_HALF_RIGHT_MM	(CAR_BODY_WIDTH_MM / 2)
#define CAR_WIDTH_MM		(CAR_HALF_LEFT_MM + CAR_HALF_RIGHT_MM)	/* W = 180 */
/* Line sensors to the wheel axle (the pivot point), along the car.  At a
   junction the car drives this far after the sensors see the crossing,
   so it pivots with the wheels over the crossing and the sensors end up
   on the new line. */
#define SENSOR_TO_AXLE_MM	130	/* measured */
/* Barcode sensor spot, sideways from the car's centre line (+ = left).
   Barcode widths are measured as the distance this point travels, from
   both wheel encoders (task_barcode.c): a point left of centre moves
   slower than the centre when the car steers left, faster when it steers
   right.  On a straight run (both wheels equal) the value does not
   matter.  Measured 30 Oct: 80 mm (outside the left wheel, 56.5 mm). */
#define BARCODE_SENSOR_Y_MM	80
#define BYPASS_MARGIN_MM	50	/* D2: M, start value */
/* Ultrasonic sensor and car outline along the car, from the wheel axle
   (the pivot point), measured.  Readings are measured from the front of
   the transducers (US_FACE_TO_AXLE_MM). */
#define US_BACK_TO_AXLE_MM	70	/* back face of the HC-SR04 module */	/* 11 Nov: obstacle bypass */
#define US_FACE_TO_AXLE_MM	81	/* front of the transducers */	/* 11 Nov: obstacle bypass */
#define FRONT_TO_AXLE_MM	132	/* front-most point of the car */	/* 11 Nov: obstacle bypass */
#define REAR_TO_AXLE_MM		56	/* rear-most point (documentation) */	/* 11 Nov: obstacle bypass */
#define REAR_WIDTH_MM		90	/* width at the rear-most point (documentation) */	/* 11 Nov: obstacle bypass */

#endif /* HW_ALLOC_H */
