/*
 *      remote_cmd.h
 *      START / STOP from the PC over MQTT (topic <client id>/cmd).
 *      Plain <stdint.h>: called by mqtt_utk.c (SDK side, Wi-Fi task).
 *                                                            Owner: Christian
 *
 *      Payload (case-insensitive, leading / trailing spaces ignored):
 *        "start", "stop"   like the START button
 *        "mode"            like a short MODE press (clear Fault, leave
 * Calibrate) "calibrate"       like holding MODE 2 s (Calibrate, from Idle
 * only) "depth <mm>"      obstacle depth the bypass assumes, 10..300, "mm"
 *                          after the number allowed (any state; beeps)
 *        "wcal <l> <r>"    width calibration run, each box edge from the
 *                          centre line in mm (Idle only)
 *        "wcal clear"      forget the calibration runs (any state)
 *        "esweep"          width sweep, measure only (Idle only)
 *      The bypass side is not a command: the car goes round the side with
 *      more room, from the width sweep (20 Nov).
 *      The PC must publish it NOT retained: a retained "start" would start
 *      the car again every time it reconnects to the broker.
 */

#ifndef REMOTE_CMD_H
#define REMOTE_CMD_H

#include <stdint.h>

/* Parses the payload and hands it to the Supervisor (EV_REMOTE).  Writes
   a short reply ("ok ..." / "refused ..." / "error ...") for the ack
   topic.  Defined in supervisor.c.  Returns 0 if sent, < 0 if not. */
int32_t remote_cmd_rx(const uint8_t * p, uint32_t len, char * reply,
                      uint32_t reply_size);

#endif /* REMOTE_CMD_H */

/*** end of file ***/
