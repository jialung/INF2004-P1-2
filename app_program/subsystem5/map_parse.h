/*
 *	map_parse.h
 *	Barcode letter -> junction movement mapping, received over MQTT.
 *	Plain <stdint.h>: used by mqtt_utk.c (SDK side, Wi-Fi task) and the
 *	laptop tests.                                         Owner: Christian
 *	Plan refs: FR4, CH-1.13, CH-1.14.
 *
 *	Payload: text, pairs of one letter and one movement word, in either
 *	order, any separators, case-insensitive.  All of these work:
 *	    A=forward,B=left,C=right,D=uturn
 *	    left=K right=M forward=X u-turn=Q
 *	    {"A":"forward","B":"left","C":"right","D":"U-turn"}
 *	Movement words: FORWARD / STRAIGHT, LEFT, RIGHT, UTURN (also U-TURN,
 *	U_TURN).  Letters not named are FORWARD.  The whole table is replaced
 *	in one step; a payload with any error changes nothing.
 *
 *	DEFAULT (or ELSE) in place of a letter sets what the car does at a
 *	junction it reaches with NO barcode pending -- a barcode counts for
 *	ONE junction only, then the car is back to this default:
 *	    A=uturn,default=left
 *	Left out, the default is FORWARD (drive straight across), which is
 *	what the car did before this existed.
 */

#ifndef MAP_PARSE_H
#define MAP_PARSE_H

#include <stdint.h>

/* same values as MAP_CMD in subsystem5.h */
#define MP_FORWARD		0U
#define MP_LEFT			1U
#define MP_RIGHT		2U
#define MP_UTURN		3U

#define MP_ERR_EMPTY		(-1)	/* no pair found */
#define MP_ERR_WORD		(-2)	/* unknown word (typo?) */
#define MP_ERR_PAIR		(-3)	/* a letter or a word without its partner */

/* Where the junction default sits in the packed 'b' word.  26 letters x
   2 bits fill all of 'a' and bits 0..19 of 'b', so bits 20..31 were spare.
   A payload from before this existed packs 0 there, which reads back as
   MP_FORWARD: the old behaviour. */
#define MP_DEF_SHIFT		20

/* Parse into table[26]; *assigned = bit i set if letter i was named;
   *deflt = the DEFAULT / ELSE movement (MP_FORWARD if the payload does
   not name one).  Returns the number of pairs, or MP_ERR_*. */
int32_t  map_parse(const uint8_t *p, uint32_t len, uint8_t table[26],
		   uint32_t *assigned, uint8_t *deflt);

/* "A=forward B=left ..." for the letters in 'mask'; returns the length */
uint32_t map_format(const uint8_t table[26], uint32_t mask, char *out, uint32_t size);

const char *map_move_name(uint32_t move);

/* 26 x 2 bits + the junction default into two words (EV_MAP_NEW payload
   a, b) and back */
void map_pack(const uint8_t table[26], uint8_t deflt, uint32_t *a, uint32_t *b);
void map_unpack(uint32_t a, uint32_t b, uint8_t table[26], uint8_t *deflt);

/* Called by mqtt_utk.c with a payload from the map topic.  Parses it and
   hands the table to the Supervisor (EV_MAP_NEW).  Writes a short text
   reply ("ok ..." / "error ...") for the ack topic.  Defined in
   supervisor.c (kernel side).  Returns pairs applied, or < 0. */
int32_t map_rx_payload(const uint8_t *p, uint32_t len, char *reply, uint32_t reply_size);

#endif /* MAP_PARSE_H */
