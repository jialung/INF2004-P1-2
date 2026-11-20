/*
 *	fault_flags.h
 *	The one safety path: a single event flag, one bit per fault.
 *	                                                      Owner: Christian
 *	Plan refs: section 1.2, CH-1.10.  Changed only by team agreement.
 *
 *	  Bit  Name              Set by                Cleared by          Trips Fault?
 *	  0    MOTOR_STALL       Motion Control        Supervisor          yes
 *	  1    ENC_FAULT         Motion Control        Supervisor          yes
 *	  2    IMU_SENSOR        IMU (3 failed reads)  Supervisor          yes
 *	  3    IMPACT            IMU                   Supervisor          yes
 *	  4    US_FAULT          Obstacle Scan         Supervisor          yes
 *	  5    OBST_STOP         Obstacle Scan         Obstacle Scan ONLY  no, stop only
 *	  6    LINE_SEARCH_FAIL  LineSearch            Supervisor          yes
 *	  7    WIFI_DOWN         Telemetry             Telemetry           no, status only
 *	  8    INIT_FAIL         usermain / any init   Supervisor          yes   (new)
 *	  9    NO_BYPASS         Supervisor (bypass)   Supervisor          yes   (new)
 *	  10   PICKUP            IMU (car lifted)      Supervisor          yes   (new)
 *
 *	Rules
 *	- Raise with fault_raise(), clear with fault_clear().  Never call
 *	  tk_set_flg / tk_clr_flg on flg_fault directly: tk_clr_flg takes the
 *	  bits to KEEP, which is easy to get backwards.
 *	- Motion Control reads the flag every 10 ms (fault_peek) and cuts the
 *	  motors within that cycle if any FAULT_STOP_MASK bit is set.
 *	- Nobody WAITS on this flag with TWF_CLR or TWF_BITCLR: that would
 *	  wipe bits Motion Control still needs to see.  The Supervisor does not
 *	  wait on it at all; fault_raise() also posts EV_FAULT to wake it, and
 *	  it re-reads the flag every SUP_POLL_MS anyway.
 *	- OBST_STOP has ONE clearer: Obstacle Scan.  It sets the bit only in
 *	  GUARD scan mode and clears it when the path is clear (hysteresis) or
 *	  when the Supervisor moves it out of GUARD (scan, pass-along, ...).
 *	  This is what lets the bypass start while the box is still ahead.
 *	- "No echo" (nothing in range) is normal, not a fault.
 */

#ifndef FAULT_FLAGS_H
#define FAULT_FLAGS_H

#include <tk/tkernel.h>

#define FAULT_MOTOR_STALL	(1U << 0)
#define FAULT_ENC_FAULT		(1U << 1)
#define FAULT_IMU_SENSOR	(1U << 2)
#define FAULT_IMPACT		(1U << 3)
#define FAULT_US_FAULT		(1U << 4)
#define FAULT_OBST_STOP		(1U << 5)
#define FAULT_LINE_SEARCH_FAIL	(1U << 6)
#define FAULT_WIFI_DOWN		(1U << 7)
#define FAULT_INIT_FAIL		(1U << 8)
#define FAULT_NO_BYPASS		(1U << 9)	/* Supervisor: no side fits after the
						   retries, or the bypass timed out */
#define FAULT_PICKUP		(1U << 10)	/* IMU: car lifted while running */

/* Bits that put the Supervisor into Fault */
#define FAULT_TRIP_MASK		(FAULT_MOTOR_STALL | FAULT_ENC_FAULT |	\
				 FAULT_IMU_SENSOR | FAULT_IMPACT |	\
				 FAULT_US_FAULT | FAULT_LINE_SEARCH_FAIL | \
				 FAULT_INIT_FAIL | FAULT_NO_BYPASS |	\
				 FAULT_PICKUP)

/* Bits that make Motion Control cut the motors this cycle */
#define FAULT_STOP_MASK		(FAULT_TRIP_MASK | FAULT_OBST_STOP)

/* Bits that may be cleared by a short MODE press */
#define FAULT_USER_CLEAR_MASK	(FAULT_TRIP_MASK)

IMPORT ID flg_fault;			/* created by fault_flags_init() */

IMPORT ER   fault_flags_init(void);	/* call once, before any task starts */

/* Set bits.  Callable from tasks AND from interrupt / cyclic handlers. */
IMPORT void fault_raise(UINT bits);

/* Clear bits (only the bits you own, see the table above). */
IMPORT void fault_clear(UINT bits);

/* Current pattern without waiting or clearing anything. */
IMPORT UINT fault_peek(void);

/* Short name of the lowest set bit, for logs ("OBST_STOP"...). */
IMPORT const char *fault_name(UINT bits);

#endif /* FAULT_FLAGS_H */
