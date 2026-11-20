/*
 *	nav_state.c
 *	Owner: Jia Lung.  See nav_state.h for the rules.
 */

#include "app.h"
#include "nav_state.h"

LOCAL NAV_STATE	nav;
LOCAL ID	mtx_nav;

EXPORT ER nav_state_init(void)
{
	T_CMTX	cmtx = {0};

	/* Priority inheritance: Telemetry (15) holding the lock must not
	   delay Motion Control (8) behind a middle-priority task. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_nav = tk_cre_mtx(&cmtx);
	return (mtx_nav < E_OK) ? mtx_nav : E_OK;
}

EXPORT void nav_motion_write(const NAV_MOTION *m)
{
	if (tk_loc_mtx(mtx_nav, TMO_FEVR) == E_OK) {
		nav.m = *m;
		(void)tk_unl_mtx(mtx_nav);
	}
}

EXPORT void nav_imu_write(const NAV_IMU *i)
{
	if (tk_loc_mtx(mtx_nav, TMO_FEVR) == E_OK) {
		nav.i = *i;
		(void)tk_unl_mtx(mtx_nav);
	}
}

EXPORT void nav_read(NAV_STATE *out)
{
	if (tk_loc_mtx(mtx_nav, TMO_FEVR) == E_OK) {
		*out = nav;
		(void)tk_unl_mtx(mtx_nav);
	}
}
