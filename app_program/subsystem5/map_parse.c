/*
 *	map_parse.c
 *	See map_parse.h.  No kernel calls.                    Owner: Christian
 */

#include "map_parse.h"

#define TOK_MAX		16U
/* 'letter' slot standing for DEFAULT / ELSE: past the 26 real letters */
#define DEF_SLOT	26

static const char *const move_name[4] = { "forward", "left", "right", "uturn" };

const char *map_move_name(uint32_t move)
{
	return (move < 4U) ? move_name[move] : "?";
}

static int is_tok(uint8_t c)
{
	return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
	       (c >= '0' && c <= '9') || c == '-' || c == '_';
}

static int streq(const char *a, const char *b)
{
	while (*a != '\0' && *a == *b) { a++; b++; }
	return *a == *b;
}

/* token (upper case, '-' and '_' removed) -> movement, or -1 */
static int32_t move_of(const char *t)
{
	if (streq(t, "FORWARD") || streq(t, "STRAIGHT")) return (int32_t)MP_FORWARD;
	if (streq(t, "LEFT"))   return (int32_t)MP_LEFT;
	if (streq(t, "RIGHT"))  return (int32_t)MP_RIGHT;
	if (streq(t, "UTURN"))  return (int32_t)MP_UTURN;
	return -1;
}

int32_t map_parse(const uint8_t *p, uint32_t len, uint8_t table[26],
		  uint32_t *assigned, uint8_t *deflt)
{
	uint8_t		t[26], d = (uint8_t)MP_FORWARD;
	uint32_t	i = 0, k, mask = 0;
	int32_t		pairs = 0, letter = -1, move = -1;

	for (k = 0; k < 26U; k++) t[k] = (uint8_t)MP_FORWARD;

	while (i < len) {
		char		tok[TOK_MAX + 1U];
		uint32_t	tl = 0, raw = 0;

		while (i < len && !is_tok(p[i])) i++;		/* separators */
		if (i >= len) break;
		while (i < len && is_tok(p[i])) {
			uint8_t c = p[i++];
			raw++;
			if (c == '-' || c == '_') continue;
			if (c >= 'a' && c <= 'z') c = (uint8_t)(c - 'a' + 'A');
			if (tl < TOK_MAX) tok[tl++] = (char)c;
		}
		tok[tl] = '\0';
		if (raw > TOK_MAX) return MP_ERR_WORD;

		if (tl == 1U && tok[0] >= 'A' && tok[0] <= 'Z') {
			if (letter >= 0) return MP_ERR_PAIR;	/* two letters */
			letter = tok[0] - 'A';
		} else if (streq(tok, "DEFAULT") || streq(tok, "ELSE")) {
			if (letter >= 0) return MP_ERR_PAIR;	/* two letters */
			letter = DEF_SLOT;		/* the no-barcode default */
		} else {
			int32_t m = move_of(tok);
			if (m < 0) return MP_ERR_WORD;
			if (move >= 0) return MP_ERR_PAIR;	/* two words */
			move = m;
		}
		if (letter >= 0 && move >= 0) {
			if (letter == DEF_SLOT) {
				d = (uint8_t)move;
			} else {
				t[letter] = (uint8_t)move;
				mask |= 1UL << (uint32_t)letter;
			}
			pairs++;
			letter = -1;
			move = -1;
		}
	}
	if (letter >= 0 || move >= 0) return MP_ERR_PAIR;
	if (pairs == 0) return MP_ERR_EMPTY;

	for (k = 0; k < 26U; k++) table[k] = t[k];
	*assigned = mask;
	*deflt = d;
	return pairs;
}

uint32_t map_format(const uint8_t table[26], uint32_t mask, char *out, uint32_t size)
{
	uint32_t	k, n = 0;

	if (size == 0U) return 0;
	for (k = 0; k < 26U; k++) {
		const char	*m;
		if ((mask & (1UL << k)) == 0) continue;
		m = map_move_name(table[k]);
		if (n + 12U > size) break;	/* ' ' + "X=" + up to 7 + NUL */
		if (n > 0) out[n++] = ' ';
		out[n++] = (char)('A' + k);
		out[n++] = '=';
		while (*m != '\0') out[n++] = *m++;
	}
	out[n] = '\0';
	return n;
}

void map_pack(const uint8_t table[26], uint8_t deflt, uint32_t *a, uint32_t *b)
{
	uint32_t k;

	*a = 0;
	*b = 0;
	for (k = 0; k < 16U; k++)  *a |= (uint32_t)(table[k] & 3U) << (2U * k);
	for (k = 16; k < 26U; k++) *b |= (uint32_t)(table[k] & 3U) << (2U * (k - 16U));
	*b |= (uint32_t)(deflt & 3U) << MP_DEF_SHIFT;
}

void map_unpack(uint32_t a, uint32_t b, uint8_t table[26], uint8_t *deflt)
{
	uint32_t k;

	for (k = 0; k < 16U; k++)  table[k] = (uint8_t)((a >> (2U * k)) & 3U);
	for (k = 16; k < 26U; k++) table[k] = (uint8_t)((b >> (2U * (k - 16U))) & 3U);
	*deflt = (uint8_t)((b >> MP_DEF_SHIFT) & 3U);
}
