/*
 *	fault_flags.c
 *	The fault event flag (fault_flags.h).                 Owner: Christian
 */

#include "app.h"
#include "fault_flags.h"
#include "msg_bus.h"

EXPORT ID flg_fault;

EXPORT ER fault_flags_init(void)
{
	T_CFLG	cflg = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_fault = tk_cre_flg(&cflg);
	return (flg_fault < E_OK) ? flg_fault : E_OK;
}

EXPORT void fault_raise(UINT bits)
{
	UINT	before = fault_peek();

	(void)tk_set_flg(flg_fault, bits);

	/* Wake the Supervisor only for bits that were not already set, so a
	   sensor that keeps failing cannot flood its queue. */
	if ((bits & ~before) != 0) {
		(void)sup_post(SRC_SYSTEM, EV_FAULT, bits & ~before, 0);
	}
}

EXPORT void fault_clear(UINT bits)
{
	/* tk_clr_flg ANDs the pattern: pass the bits to KEEP */
	(void)tk_clr_flg(flg_fault, ~bits);
}

EXPORT UINT fault_peek(void)
{
	T_RFLG	rflg;

	if (tk_ref_flg(flg_fault, &rflg) != E_OK) {
		return FAULT_INIT_FAIL;		/* no flag = treat as fault */
	}
	return rflg.flgptn;
}

EXPORT const char *fault_name(UINT bits)
{
	LOCAL const char *const names[] = {
		"MOTOR_STALL", "ENC_FAULT", "IMU_SENSOR", "IMPACT", "US_FAULT",
		"OBST_STOP", "LINE_SEARCH_FAIL", "WIFI_DOWN", "INIT_FAIL",
		"NO_BYPASS", "PICKUP"
	};
	UINT	i;

	for (i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
		if (bits & (1U << i)) {
			return names[i];
		}
	}
	return "none";
}
