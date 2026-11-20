/*
 *	telem_frame.c
 *	Plain C, no kernel or SDK headers.                    Owner: Christian
 */

#include "telem_frame.h"

uint8_t tf_crc8(const uint8_t *p, uint32_t n)
{
	uint8_t crc = 0;
	uint32_t i;
	int b;

	for (i = 0; i < n; i++) {
		crc ^= p[i];
		for (b = 0; b < 8; b++) {
			crc = (uint8_t)((crc & 0x80U) ? ((uint32_t)crc << 1) ^ 0x07U : ((uint32_t)crc << 1));
		}
	}
	return crc;
}

static void put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)(v >> 8);
}

static uint16_t get16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

uint32_t tf_pack(const TELEM_FIELDS *f, uint8_t out[TF_LEN])
{
	out[0] = (uint8_t)((TF_TYPE_STATUS & 0x7U) | ((TF_VERSION & 0x1U) << 3) |
			   ((f->state & 0xFU) << 4));
	put16(&out[1], f->faults);
	put16(&out[3], f->odo_mm);
	put16(&out[5], (uint16_t)f->v_mm_s);
	put16(&out[7], (uint16_t)((f->heading_deg & 0x1FFU) |
				  ((f->terrain & 0x7U) << 9) |
				  ((f->speed_level & 0x3U) << 12)));
	out[9]  = (uint8_t)f->roll_q;
	out[10] = (uint8_t)f->pitch_q;
	out[11] = f->humps;
	out[12] = (uint8_t)((f->letter & 0x1FU) | ((f->letter_rev & 0x1U) << 5));
	out[13] = tf_crc8(out, TF_LEN - 1U);
	return TF_LEN;
}

int32_t tf_unpack(const uint8_t in[TF_LEN], TELEM_FIELDS *f)
{
	uint16_t w;

	if (tf_crc8(in, TF_LEN - 1U) != in[13])        return -1;	/* CRC */
	if ((in[0] & 0x7U) != TF_TYPE_STATUS)           return -2;
	if (((in[0] >> 3) & 0x1U) != TF_VERSION)        return -3;
	f->state       = (uint8_t)(in[0] >> 4);
	f->faults      = get16(&in[1]);
	f->odo_mm      = get16(&in[3]);
	f->v_mm_s      = (int16_t)get16(&in[5]);
	w              = get16(&in[7]);
	f->heading_deg = (uint16_t)(w & 0x1FFU);
	f->terrain     = (uint8_t)((w >> 9) & 0x7U);
	f->speed_level = (uint8_t)((w >> 12) & 0x3U);
	f->roll_q      = (int8_t)in[9];
	f->pitch_q     = (int8_t)in[10];
	f->humps       = in[11];
	f->letter      = (uint8_t)(in[12] & 0x1FU);
	f->letter_rev  = (uint8_t)((in[12] >> 5) & 0x1U);
	return 0;
}
