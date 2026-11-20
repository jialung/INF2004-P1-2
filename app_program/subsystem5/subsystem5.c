/*
 *	subsystem5.c
 *	Supervisor + Telemetry: hardware init and task start. Owner: Christian
 */

#include "subsystem5.h"
#include "drv_ui.h"

EXPORT ID   tid_supervisor;
EXPORT ID   tid_telemetry;
EXPORT BOOL ui_led_ok;

EXPORT ER subsystem5_init(void)
{
	INT r;

	/* WS2812 AFTER the encoders on pio1 (S1 ran first, R7) */
	r = drv_led_init();
	ui_led_ok = (r == UI_OK);
	LOG("sup", "WS2812 PIO load: %s\n",
	    r == UI_OK ? "OK" : (r == UI_ERR_NO_SPACE ? "NO SPACE" : "SM BUSY"));

	if (drv_buzzer_init() != UI_OK) {
		return E_SYS;
	}
	if (buttons_init() < E_OK) {
		return E_SYS;
	}
	return ui_led_ok ? E_OK : E_SYS;
}

EXPORT ER subsystem5_start(void)
{
	tid_supervisor = app_start_task(supervisor_task, PRI_SUPERVISOR,
					CORE_SUPERVISOR, "supervisor");
	if (tid_supervisor < E_OK) return tid_supervisor;
	tid_telemetry = app_start_periodic(telemetry_task, PRI_TELEMETRY,
					   CORE_TELEMETRY, PERIOD_TELEMETRY_MS,
					   "telemetry", NULL);
	return (tid_telemetry < E_OK) ? tid_telemetry : E_OK;
}
