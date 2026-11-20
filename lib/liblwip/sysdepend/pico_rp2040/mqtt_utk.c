/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_HEARTBEAT_MS   5000
#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000

/* Robot telemetry (CH-1.11, patch P5 done as an lwIP timer instead of a
 * hook in cyw43_utk.c's loop -- same task, so the R-5 rule still holds).
 * task_telemetry.c (kernel side) writes the newest 14-byte frame into
 * telem_outbox every 500 ms under a sequence lock: telem_outbox_seq is odd
 * while it is writing and even when the frame is complete.  The frame
 * format is in app_program/subsystem5/telem_frame.h (plain stdint, safe
 * here).  Published binary, QoS 0, not retained. */
#include "../../../../app_program/subsystem5/telem_frame.h"

#ifndef MQTT_TOPIC_TELEM
#define MQTT_TOPIC_TELEM        MQTT_CLIENT_ID "/telem"
#endif
#define MQTT_UTK_OUTBOX_MS      100

extern volatile uint8_t  telem_outbox[TF_LEN];
extern volatile uint32_t telem_outbox_seq;

/* Barcode letter -> movement mapping from the PC (FR4, CH-1.14).  The text
 * payload is parsed by map_rx_payload() (app_program/subsystem5, plain
 * stdint interface) which hands the table to the Supervisor.  The result
 * is published on MQTT_TOPIC_MAP "/ack" so the sender can see it landed.
 * Subscribed with QoS 1; a retained message is picked up on (re)connect. */
#include "../../../../app_program/subsystem5/map_parse.h"

#ifndef MQTT_TOPIC_MAP
#define MQTT_TOPIC_MAP          MQTT_CLIENT_ID "/map"
#endif
#define MQTT_TOPIC_MAP_ACK      MQTT_TOPIC_MAP "/ack"

/* Events (FR12 / FR14 map) and snapshot request / reply (FR13) */
#ifndef MQTT_TOPIC_EVENT
#define MQTT_TOPIC_EVENT        MQTT_CLIENT_ID "/event"
#endif
#ifndef MQTT_TOPIC_SNAPSHOT
#define MQTT_TOPIC_SNAPSHOT     MQTT_CLIENT_ID "/snapshot"
#endif
#define MQTT_TOPIC_SNAP_REPLY   MQTT_TOPIC_SNAPSHOT "/reply"

/* START / STOP from the PC (app_program/subsystem5/remote_cmd.h).  The
 * answer goes to "/ack".  The PC must NOT retain these: a retained
 * "start" would restart the car on every reconnect. */
#include "../../../../app_program/subsystem5/remote_cmd.h"

#ifndef MQTT_TOPIC_CMD
#define MQTT_TOPIC_CMD          MQTT_CLIENT_ID "/cmd"
#endif
#define MQTT_TOPIC_CMD_ACK      MQTT_TOPIC_CMD "/ack"

/* Console log: a copy of everything printed on the USB console, filled
 * by usb_console.c while the client is connected.  Published as text
 * chunks of whole lines, QoS 0, after telemetry and events so a busy log
 * never delays them.  Text that does not fit is dropped, never waited
 * for (same rule as USB and Telnet). */
#ifndef MQTT_TOPIC_LOG
#define MQTT_TOPIC_LOG          MQTT_CLIENT_ID "/log"
#endif
#define MQTT_UTK_LOG_CHUNK      400	/* bytes per publish */
#define MQTT_UTK_LOG_PER_POLL   2	/* publishes per 100 ms poll */
#define MQTT_UTK_LOG_HOLD       5	/* polls a partial line may wait */

extern void tm_mqtt_log_enable(int on);
extern int  tm_mqtt_log_get(unsigned char *out, int max);

static int topic_is(const char *a, const char *b)
{
	while(*a != '\0' && *a == *b) { a++; b++; }
	return *a == *b;
}

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past the
 * heartbeat this demo publishes; anything longer is truncated and flagged
 * rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_truncated;

static UB		log_pend[MQTT_UTK_LOG_CHUNK];
static u16_t		log_n;
static u8_t		log_age;

static void heartbeat_timeout(void *arg);
static void publish_done_cb(void *arg, err_t result);

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	(void)tot_len;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_truncated = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for(i = 0; i < len; i++) {
		if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1)) {
			/* The console is a terminal -- keep raw bytes from a
			 * payload this demo did not publish from scrambling it. */
			UB c = data[i];
			rx_data[rx_len++] = (c >= 0x20 && c < 0x7f)? c: '.';
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		rx_data[rx_len] = '\0';
		tm_printf((UB*)"mqtt: RX [%s] \"%s\"%s\n", rx_topic, rx_data,
			  rx_truncated? " (truncated)": "");

		if(topic_is(rx_topic, MQTT_TOPIC_SNAPSHOT)) {
			static char	snap[768];
			uint32_t	n = telem_snapshot(snap, sizeof(snap));

			if(n > 0 && mqtt_client_is_connected(client)) {
				(void)mqtt_publish(client, MQTT_TOPIC_SNAP_REPLY, snap, (u16_t)n,
						   0 /* qos */, 0 /* retain */,
						   publish_done_cb, NULL);
			}
		}

		if(topic_is(rx_topic, MQTT_TOPIC_MAP)) {
			static char	reply[200];
			u16_t		n = 0;

			if(rx_truncated) {
				(void)tm_sprintf((UB*)reply, (const UB*)"error: mapping longer than %d bytes",
						 MQTT_UTK_RX_DATA_MAX - 1);
			} else {
				(void)map_rx_payload(rx_data, rx_len, reply, sizeof(reply));
			}
			while(n < sizeof(reply) - 1 && reply[n] != '\0') n++;
			tm_printf((UB*)"mqtt: map %s\n", reply);
			if(mqtt_client_is_connected(client)) {
				(void)mqtt_publish(client, MQTT_TOPIC_MAP_ACK, reply, n,
						   1 /* qos */, 0 /* retain */,
						   publish_done_cb, NULL);
			}
		}

		if(topic_is(rx_topic, MQTT_TOPIC_CMD)) {
			static char	reply[160];
			u16_t		n = 0;

			reply[0] = '\0';
			(void)remote_cmd_rx(rx_data, rx_len, reply, sizeof(reply));
			while(n < sizeof(reply) - 1 && reply[n] != '\0') n++;
			tm_printf((UB*)"mqtt: cmd %s\n", reply);
			if(mqtt_client_is_connected(client)) {
				(void)mqtt_publish(client, MQTT_TOPIC_CMD_ACK, reply, n,
						   1 /* qos */, 0 /* retain */,
						   publish_done_cb, NULL);
			}
		}
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to %s\n",
			  (arg != NULL)? (const char*)arg: MQTT_TOPIC);
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}

static void heartbeat_timeout(void *arg)
{
	static unsigned int	count = 0;
	UB			payload[48];
	int			len;

	(void)arg;

	if(mqtt_client_is_connected(client)) {
		len = tm_sprintf(payload, (const UB*)"hb %u", count++);
		tm_printf((UB*)"mqtt: TX [" MQTT_TOPIC "] \"%s\"\n", payload);
		(void)mqtt_publish(client, MQTT_TOPIC, payload, (u16_t)len,
				   0 /* qos */, 0 /* retain */,
				   publish_done_cb, NULL);
	}

	/* Re-arms itself every call -- mqtt.c's connection callback (below)
	 * only starts this chain once, on the transition into MQTT_CONNECT_
	 * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
	sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

/* Console log -> MQTT_TOPIC_LOG.  Text stays in log_pend until a publish
 * accepts it, so a full output buffer delays it instead of losing it;
 * only the console ring itself drops (when it fills). */
static void log_poll(void)
{
	int	k;

	for(k = 0; k < MQTT_UTK_LOG_PER_POLL && mqtt_client_is_connected(client); k++) {
		u16_t	cut = 0, i;

		log_n = (u16_t)(log_n + tm_mqtt_log_get(log_pend + log_n,
							(int)(MQTT_UTK_LOG_CHUNK - log_n)));
		if(log_n == 0) {
			log_age = 0;
			return;
		}
		/* send whole lines; a partial line waits a little for its end */
		for(i = log_n; i > 0; i--) {
			if(log_pend[i - 1] == '\n') { cut = i; break; }
		}
		if(cut == 0) {
			if(log_n < MQTT_UTK_LOG_CHUNK && ++log_age < MQTT_UTK_LOG_HOLD) return;
			cut = log_n;
		}
		if(mqtt_publish(client, MQTT_TOPIC_LOG, log_pend, cut, 0 /* qos */,
				0 /* retain */, publish_done_cb, NULL) != ERR_OK) {
			return;				/* output buffer full: next poll */
		}
		for(i = cut; i < log_n; i++) log_pend[i - cut] = log_pend[i];
		log_n = (u16_t)(log_n - cut);
		log_age = 0;
	}
}

static void outbox_timeout(void *arg)
{
	static uint32_t	last_seq;
	static uint32_t	sent;
	uint8_t		frame[TF_LEN];
	uint32_t	s1, s2, i;

	(void)arg;

	s1 = telem_outbox_seq;
	if(mqtt_client_is_connected(client) && (s1 & 1u) == 0 && s1 != last_seq) {
		__asm__ volatile("dmb" ::: "memory");
		for(i = 0; i < TF_LEN; i++) {
			frame[i] = telem_outbox[i];
		}
		__asm__ volatile("dmb" ::: "memory");
		s2 = telem_outbox_seq;
		if(s1 == s2) {			/* not overwritten while copying */
			if(mqtt_publish(client, MQTT_TOPIC_TELEM, frame, TF_LEN,
					0 /* qos */, 0 /* retain */,
					publish_done_cb, NULL) == ERR_OK) {
				last_seq = s1;
				if(sent++ == 0) {
					tm_printf((UB*)"mqtt: telemetry on [" MQTT_TOPIC_TELEM
						  "], %d-byte frames\n", (int)TF_LEN);
				}
			}
		}
		/* else: the writer was mid-copy; try again on the next poll */
	}

	/* event ring: publish what is waiting, oldest first; stop at the
	   first refusal (output buffer full) and retry on the next poll */
	while(mqtt_client_is_connected(client) && telem_evt_tail != telem_evt_head) {
		uint32_t	t = telem_evt_tail;
		uint8_t		ev[TELEM_EVT_LEN];
		u16_t		n = 0;

		__asm__ volatile("dmb" ::: "memory");
		while(n < TELEM_EVT_LEN - 1U &&
		      telem_evt[t % TELEM_EVT_SLOTS][n] != '\0') {
			ev[n] = telem_evt[t % TELEM_EVT_SLOTS][n];
			n++;
		}
		if(mqtt_publish(client, MQTT_TOPIC_EVENT, ev, n, 0, 0,
				publish_done_cb, NULL) != ERR_OK) {
			break;
		}
		telem_evt_tail = t + 1U;
	}

	log_poll();

	sys_timeout(MQTT_UTK_OUTBOX_MS, outbox_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Subscribe to the same topic this client publishes to, so every
		 * heartbeat comes back from the broker and is printed by
		 * incoming_data_cb(). A matching RX for each TX is end-to-end
		 * proof of the round trip without needing mosquitto_sub on the
		 * broker host. Callbacks must be registered before subscribing,
		 * or the first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, MQTT_TOPIC, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}
		err = mqtt_subscribe(client, MQTT_TOPIC_SNAPSHOT, 0 /* qos */,
				     subscribe_done_cb, (void*)MQTT_TOPIC_SNAPSHOT);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: subscribe to " MQTT_TOPIC_SNAPSHOT
				  " failed, err=%d\n", (int)err);
		}
		err = mqtt_subscribe(client, MQTT_TOPIC_MAP, 1 /* qos */,
				     subscribe_done_cb, (void*)MQTT_TOPIC_MAP);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: subscribe to " MQTT_TOPIC_MAP
				  " failed, err=%d\n", (int)err);
		}
		err = mqtt_subscribe(client, MQTT_TOPIC_CMD, 1 /* qos */,
				     subscribe_done_cb, (void*)MQTT_TOPIC_CMD);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: subscribe to " MQTT_TOPIC_CMD
				  " failed, err=%d\n", (int)err);
		}

		/* console copy on: from here on every console line also goes
		   to MQTT_TOPIC_LOG */
		log_n = 0;
		log_age = 0;
		tm_mqtt_log_enable(1);
		tm_printf((UB*)"mqtt: console log on [" MQTT_TOPIC_LOG "]\n");

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating heartbeat, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(heartbeat_timeout, NULL);
		sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
		/* same rule for the telemetry poll chain (one slot of the +4
		   MEMP_NUM_SYS_TIMEOUT budget in lwipopts.h) */
		sys_untimeout(outbox_timeout, NULL);
		sys_timeout(MQTT_UTK_OUTBOX_MS, outbox_timeout, NULL);
	} else {
		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_mqtt_log_enable(0);		/* nobody to send it to */
		log_n = 0;
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}
