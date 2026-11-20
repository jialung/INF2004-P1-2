/*
 *	telnet_utk.h
 *	Console over Wi-Fi: a Telnet server on port 23 that sends a copy of
 *	everything printed on the USB console.  Output only.  One client at
 *	a time (a new one replaces the old).  NO_SYS=1: both calls only from
 *	cyw43_utk_task, like the rest of lwIP.
 */

#ifndef TELNET_UTK_H
#define TELNET_UTK_H

#define TELNET_UTK_PORT		23

void telnet_utk_start(void);	/* idempotent: call when the link is up */
void telnet_utk_poll(void);	/* every poll loop: send what is queued */

#endif /* TELNET_UTK_H */
