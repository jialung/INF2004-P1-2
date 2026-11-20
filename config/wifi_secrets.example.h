/*
 *	wifi_secrets.example.h
 *
 *	Copy this file to wifi_secrets.h (same directory) and fill in your own
 *	network's credentials. wifi_secrets.h is listed in .gitignore -- it
 *	will never be committed, so it's safe to put your real password there.
 *
 *	Only used when built with NET=lwip (see build_make/pico_rp2040.mk and
 *	lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c).
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

/* Only used when built with MQTT=1 (also requires NET=lwip). */
#define MQTT_BROKER_IP   "192.168.1.10"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "mtk3smp-pico"
#define MQTT_TOPIC       "mtk3smp-pico/heartbeat"
/* Optional: robot telemetry frames (14 bytes, app_program/subsystem5/
   telem_frame.h).  Default: MQTT_CLIENT_ID "/telem". */
/* #define MQTT_TOPIC_TELEM "mtk3smp-pico/telem" */
/* Optional: barcode letter -> movement mapping from the PC, text such as
   "A=forward,B=left,C=right,D=uturn".  The car replies on <topic>/ack.
   Default: MQTT_CLIENT_ID "/map". */
/* #define MQTT_TOPIC_MAP "mtk3smp-pico/map" */
/* Optional: JSON events (barcode, junction, command, obstacle, hump,
   state, with x/y position) and the snapshot request topic (any payload;
   the reply goes to <topic>/reply).  Defaults: "/event", "/snapshot". */
/* #define MQTT_TOPIC_EVENT    "mtk3smp-pico/event" */
/* #define MQTT_TOPIC_SNAPSHOT "mtk3smp-pico/snapshot" */

#endif /* WIFI_SECRETS_H */
