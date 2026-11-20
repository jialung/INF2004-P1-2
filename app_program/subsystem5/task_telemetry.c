/*
 *      task_telemetry.c
 *      Telemetry task, 500 ms, core 0, priority 15.            Owner: Christian
 *      Plan refs: CH-1.11, CH-1.12, CH-1.15, CH-1.16, chain 4.7.
 *
 *      Done:  drains mbf_telem (state changes, map entries), packs the
 *             binary frame with CRC-8, WIFI_DOWN from the CYW43 link status,
 *             console fallback line every 2 s (UART / USB, Week 10 plan B).
 *
 *      Publishing (CH-1.11).  lwIP runs with NO_SYS=1 inside the template's
 *      cyw43_utk_task (core 0, priority 4), so mqtt_publish() may ONLY be
 *      called from that task.  Do not call it from here.  This task leaves
 *      the newest frame in telem_outbox; mqtt_utk.c polls it every 100 ms
 *      from an lwIP timer in that task and publishes it on MQTT_TOPIC_TELEM
 *      (build with NET=lwip MQTT=1).  TODO: snapshot replies (CH-1.15) and
 *      map/set the same way.
 */

#include "subsystem5.h"
#include "telem_frame.h"
#include "../fault_flags.h"
#include "../msg_bus.h"
#include "../nav_state.h"
#include "../wcet.h"
#include "../hal.h"
#include "../diag.h"
#include "cyw43_utk.h" /* template header, plain stdint: kernel-side safe */
#include "map_parse.h"
#include "../subsystem2/subsystem2.h" /* bc_stats */
#include "../subsystem3/subsystem3.h" /* imu_peak_mm */
#include "../subsystem4/subsystem4.h" /* obst_stats */

#define CYW43_LINK_UP_VALUE 3 /* CYW43_LINK_UP in cyw43.h (SDK side) */

#define TELEM_CONSOLE_EVERY 4 /* console line every 4 x 500 ms */

/* Single-slot outbox: newest frame wins.  Read by the P5 hook. */
EXPORT volatile UB telem_outbox[TF_LEN];
EXPORT volatile UW telem_outbox_seq;

/* Event ring for MQTT (telem_frame.h) */
EXPORT volatile uint8_t  telem_evt[TELEM_EVT_SLOTS][TELEM_EVT_LEN];
EXPORT volatile uint32_t telem_evt_head, telem_evt_tail;
LOCAL UW                 evt_dropped;

LOCAL const char * const state_name[SYS_STATE_COUNT] = {
    "init", "idle",     "calibrate",   "line_follow", "junction",
    "hump", "obstacle", "line_search", "finished",    "fault"};

LOCAL void evt_push (const UB * txt)
{
    UW                 h = telem_evt_head, i;
    volatile uint8_t * slot;

    if ((h - telem_evt_tail) >= TELEM_EVT_SLOTS)
    {
        evt_dropped++; /* Wi-Fi down / slow: drop the newest */
        return;
    }
    slot = telem_evt[h % TELEM_EVT_SLOTS];
    for (i = 0; i < TELEM_EVT_LEN - 1U && txt[i] != '\0'; i++)
    {
        slot[i] = txt[i];
    }
    slot[i] = '\0';
    MEM_BARRIER();
    telem_evt_head = h + 1U;
}

/* One Supervisor message -> one JSON event line (FR12, FR14) */
LOCAL void evt_from (const TELEM_MSG * m)
{
    UB  t[TELEM_EVT_LEN];
    INT x = (INT) m->x_mm, y = (INT) m->y_mm;

    switch ((TM_KIND) m->kind)
    {
        case TM_STATE:
            (void) tm_sprintf(
                t,
                (const UB *) "{\"ev\":\"state\",\"state\":\"%s\",\"x\":%d,"
                             "\"y\":%d}",
                state_name[m->value < SYS_STATE_COUNT ? m->value : 0], x, y);
            break;
        case TM_MAP_BARCODE:
            (void) tm_sprintf(t,
                              (const UB *) "{\"ev\":\"barcode\",\"letter\":\"%"
                                           "c\",\"x\":%d,\"y\":%d}",
                              (INT) ('A' + (m->value % 26U)), x, y);
            break;
        case TM_MAP_JUNCTION:
            (void) tm_sprintf(
                t, (const UB *) "{\"ev\":\"junction\",\"x\":%d,\"y\":%d}", x,
                y);
            break;
        case TM_COMMAND:
            (void) tm_sprintf(t,
                              (const UB *) "{\"ev\":\"command\",\"move\":\"%"
                                           "s\",\"x\":%d,\"y\":%d}",
                              map_move_name(m->value), x, y);
            break;
        case TM_MAP_OBSTACLE:
            (void) tm_sprintf(
                t,
                (const UB *) "{\"ev\":\"obstacle\",\"width_mm\":%d,"
                             "\"x\":%d,\"y\":%d}",
                (INT) m->value, x, y);
            break;
        case TM_MAP_HUMP:
            (void) tm_sprintf(t,
                              (const UB *) "{\"ev\":\"hump\",\"height_mm\":%d,"
                                           "\"x\":%d,\"y\":%d}",
                              (INT) m->value, x, y);
            break;
        default:
            return;
    }
    evt_push(t);
    if (m->kind != TM_STATE)
    {
        tm_printf((UB *) "[map] %s\n", t);
    }
}

/* FR13: called by the Wi-Fi task for a snapshot request */
EXPORT uint32_t telem_snapshot (char * out, uint32_t size)
{
    NAV_STATE ns;
    SYS_STATE s = sys_state_get();
    UB        map[26];
    char      mtxt[200];
    UINT      k, mask = 0;
    INT       pend;

    /* worst case ~330 B of fields + 200 B of mapping: no bound in
       tm_sprintf, so insist on room for it */
    if (size < 768U)
    {
        return 0;
    }
    nav_read(&ns);
    sup_map_get(map);
    for (k = 0; k < 26U; k++)
    {
        if (map[k] != MAP_FORWARD)
        {
            mask |= 1UL << k;
        }
    }
    (void) map_format(map, mask, mtxt, sizeof(mtxt));
    pend = sup_pending_letter();
    return (uint32_t) tm_sprintf(
        (UB *) out,
        (const UB *) "{\"state\":\"%s\",\"faults\":%d,\"odo_mm\":%d,\"speed_mm_"
                     "s\":%d,"
                     "\"x_mm\":%d,\"y_mm\":%d,\"heading_deg\":%d,\"roll_q\":%d,"
                     "\"pitch_q\":%d,"
                     "\"humps\":%d,\"hump_peak_mm\":%d,\"barcodes\":%d,"
                     "\"misreads\":%d,"
                     "\"last_letter\":\"%c\",\"pending\":\"%c\",\"obstacle_"
                     "width_mm\":%d,"
                     "\"bypass_side\":\"%s\",\"depth_mm\":%d,\"map\":\"%s\"}",
        state_name[s < SYS_STATE_COUNT ? s : 0], (INT) fault_peek(),
        (INT) ns.m.odo_mm, (INT) ns.m.v_mm_s, (INT) ns.m.x_mm, (INT) ns.m.y_mm,
        (INT) ns.i.heading_deg, (INT) ns.i.roll_q, (INT) ns.i.pitch_q,
        (INT) sup_stats.humps, (INT) imu_peak_mm, (INT) bc_stats.reads,
        (INT) bc_stats.misreads,
        (bc_stats.last_letter < 26U) ? (INT) ('A' + bc_stats.last_letter) : '-',
        (pend >= 0) ? (INT) ('A' + pend) : '-', (INT) sup_obstacle_width_mm(),
        (sup_bypass_side() > 0) ? "left" : "right", (INT) sup_bypass_depth(),
        mtxt);
}

LOCAL SPEED_LEVEL speed_of (SYS_STATE s)
{
    switch (s)
    {
        case SYS_LINE_FOLLOW:
            return SPD_CRUISE;
        case SYS_HUMP_TRAVERSE:
            return SPD_HUMP;
        case SYS_LINE_SEARCH:
        case SYS_JUNCTION_ACTION:
            return SPD_SLOW;
        default:
            return SPD_STOP;
    }
}

EXPORT void telemetry_task (INT stacd, void * exinf)
{
    TELEM_MSG          m;
    TELEM_FIELDS       f;
    NAV_STATE          ns;
    T_CYW43_UTK_STATUS ws;
    UB                 frame[TF_LEN];
    UINT               tick = 0, i;

    (void) stacd;
    (void) exinf;
    LOG("telem", "task on core %d\n", (INT) hal_core_id());
    wcet_set_period(WCET_TELEMETRY, PERIOD_TELEMETRY_MS);

    for (;;)
    {
        UW t0;

        tk_slp_tsk(TMO_FEVR);
        t0 = wcet_release(WCET_TELEMETRY);
        /* shared diag_pm.c: prints once if the watch tripped */
        diag_pm_live_report();

        /* 1. events from the Supervisor -> MQTT "event" topic (CH-2.2) */
        while (tk_rcv_mbf(mbf_telem, &m, TMO_POL) == (INT) sizeof(m))
        {
            evt_from(&m);
        }

        /* 2. Wi-Fi status -> WIFI_DOWN (CH-1.16; status only, NFR8) */
        cyw43_utk_get_status(&ws); /* (diag.c reads it again for [diag]) */
        if (ws.init_complete)
        {
            diag_pm_arm(); /* single core: loop watch after bring-up */
        }
        if (ws.link_status == CYW43_LINK_UP_VALUE)
        {
            fault_clear(FAULT_WIFI_DOWN);
        }
        else if ((fault_peek() & FAULT_WIFI_DOWN) == 0)
        {
            fault_raise(FAULT_WIFI_DOWN);
        }

        /* 3. frame (CH-1.12) */
        nav_read(&ns);
        f.state       = (uint8_t) sys_state_get();
        f.faults      = (uint16_t) fault_peek();
        f.odo_mm      = ns.m.odo_mm;
        f.v_mm_s      = ns.m.v_mm_s;
        f.heading_deg = ns.i.heading_deg;
        f.terrain     = ns.i.terrain;
        f.speed_level = (uint8_t) speed_of(sys_state_get());
        f.roll_q      = ns.i.roll_q;
        f.pitch_q     = ns.i.pitch_q;
        f.humps       = (uint8_t) sup_stats.humps;
        f.letter      = sup_stats.last_letter;
        f.letter_rev  = sup_stats.last_letter_rev;
        (void) tf_pack(&f, frame);

        /* Sequence lock: odd while writing, even when complete.  The
           reader (mqtt_utk.c, Wi-Fi task, priority 4) preempts this
           task, so it can run in the middle of the copy; it skips odd
           values and re-checks the number after copying. */
        telem_outbox_seq++;
        MEM_BARRIER();
        for (i = 0; i < TF_LEN; i++)
        {
            telem_outbox[i] = frame[i];
        }
        MEM_BARRIER();
        telem_outbox_seq++;

        /* 4. liveness, judged by TIME (hardware timer), not by run count:
           after this task was held off it runs several times back to
           back to catch up, and a 1 ms gap is not a stalled 10 ms loop.
           A hung Motion Control cannot cut the motors, so say it loudly.
           TODO JL-2.7: hardware watchdog (RP2040 watchdog_hw). */
        {
            LOCAL UW last_motion, last_line, last_sup, last_check_us,
                last_run_us;
            LOCAL BOOL in_stall;
            UW         now     = now_us();
            BOOL       stalled = FALSE, checked = FALSE;

            if (tick > 0 && (now - last_run_us) > 1500000U)
            {
                tm_printf((UB *) "[telem] this task (core 0, priority %d) got "
                                 "no CPU for %d ms\n",
                          (INT) PRI_TELEMETRY,
                          (INT) ((now - last_run_us) / 1000U));
            }
            last_run_us = now;

            if (tick == 0 || (now - last_check_us) >= 200000U)
            {
                if (tick > 0)
                {
                    if (wcet_tab[WCET_MOTION].runs == last_motion)
                    {
                        tm_printf((UB *) "[telem] WARNING motion loop STALLED "
                                         "(runs %d)\n",
                                  (INT) last_motion);
                        stalled = TRUE;
                    }
                    if (wcet_tab[WCET_LINE].runs == last_line)
                    {
                        tm_printf((UB *) "[telem] WARNING line loop STALLED "
                                         "(runs %d)\n",
                                  (INT) last_line);
                        stalled = TRUE;
                    }
                    if (wcet_tab[WCET_SUPERVISOR].runs == last_sup)
                    {
                        tm_printf((UB *) "[telem] WARNING supervisor STALLED "
                                         "(runs %d)\n",
                                  (INT) last_sup);
                        stalled = TRUE;
                    }
                }
                last_motion   = wcet_tab[WCET_MOTION].runs;
                last_line     = wcet_tab[WCET_LINE].runs;
                last_sup      = wcet_tab[WCET_SUPERVISOR].runs;
                last_check_us = now;
                checked       = (tick > 0);
            }
            /* counters every 10 s; full task dump once per stall episode */
            if (stalled || (tick % 20) == 0)
            {
                diag_kernel_line();
            }
            if (stalled && !in_stall)
            {
                diag_task_dump("a core-1 loop stalled");
            }
            if (checked)
            {
                in_stall = stalled; /* only a real 200 ms check changes it */
            }
        }

        /* 5. console fallback (Week 10 plan B if Wi-Fi is not ready) */
        if ((++tick % TELEM_CONSOLE_EVERY) == 0)
        {
            tm_printf((UB *) "[telem] st %d flt 0x%03x odo %d v %d hdg %d "
                             "wifi %d lost %d/%d\n",
                      (INT) f.state, (INT) f.faults, (INT) f.odo_mm,
                      (INT) f.v_mm_s, (INT) f.heading_deg, (INT) ws.link_status,
                      (INT) bus_stats.sup_lost_keep,
                      (INT) bus_stats.sup_dropped_low);
        }
        wcet_end(WCET_TELEMETRY, t0);
    }
}

/*** end of file ***/
