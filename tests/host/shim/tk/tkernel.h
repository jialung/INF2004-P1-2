/* Host-test shim: just enough of micro T-Kernel's types and calls to
   compile the pure-logic files on a laptop.  NOT the real kernel. */
#ifndef SHIM_TKERNEL_H
#define SHIM_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int8_t B; typedef int16_t H; typedef int32_t W;
typedef uint8_t UB; typedef uint16_t UH; typedef uint32_t UW;
typedef int INT; typedef unsigned int UINT; typedef INT ID; typedef INT ER;
typedef INT BOOL; typedef INT PRI; typedef UW ATR; typedef W TMO; typedef UW RELTIM;
typedef long SZ; typedef void (*FP)();
#define TRUE 1
#define FALSE 0
#define LOCAL static
#define EXPORT
#define IMPORT extern
#define E_OK 0
#define E_SYS (-5)
#define E_PAR (-17)
#define E_LIMIT (-34)
#define E_OBJ (-41)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_TFIFO 0
#define TA_WMUL 8
#define TP_PRC1 1
#define TP_PRC2 2
typedef struct { void *exinf; ATR mbfatr; SZ bufsz; INT maxmsz; void *bufptr; } T_CMBF;
typedef struct { void *exinf; ID wtsk; ID stsk; INT msgsz; SZ frbufsz; INT maxmsz; } T_RMBF;
typedef struct { void *exinf; ATR flgatr; UINT iflgptn; } T_CFLG;
typedef struct { void *exinf; ID wtsk; UINT flgptn; } T_RFLG;
ID tk_cre_mbf(const T_CMBF *p); ER tk_snd_mbf(ID id, const void *m, INT sz, TMO t);
ER tk_ref_mbf(ID id, T_RMBF *r); ID tk_cre_flg(const T_CFLG *p);
ER tk_set_flg(ID id, UINT p); ER tk_clr_flg(ID id, UINT p); ER tk_ref_flg(ID id, T_RFLG *r);
#endif
