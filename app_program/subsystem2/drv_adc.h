/*
 *      drv_adc.h
 *      ADC for the left line sensor (GP26 = ADC0) and the barcode sensor
 *      (GP27 = ADC1).  Owner: Ryan.
 *      SDK side (two-sides rule, hal.h).  Plan refs: RY-1.5..RY-1.7, R6.
 *
 *      Pin revision 2: only ADC0 and ADC1 are left (GP28 = left encoder B).
 *      The right line sensor is on GP17, read as digital (hw_alloc.h).
 *
 *      Now:   drv_adc_read() = one polled conversion (~2 us).
 *      Next:  RY-1.5/1.6/1.7: free-running ADC0/ADC1 round robin + DMA ring.
 *             NEEDED FOR BARCODES: a 10 ms poll is far too slow (a 2 mm bar
 *             at 0.5 m/s lasts 4 ms); the ring samples at tens of kHz.
 *             The ring API below is the agreed interface; it returns
 *             DRV_ADC_ERR_TODO until written.
 *
 *      Set DEVCNF_USE_ADC to 0 in config/config_device.h (patch P2).
 */

#ifndef DRV_ADC_H
#define DRV_ADC_H

#include <stdint.h>

#define DRV_ADC_OK       0
#define DRV_ADC_ERR_TODO (-100)

int32_t  drv_adc_init(void);        /* GP26 + GP27 analog */
uint32_t drv_adc_read(uint32_t ch); /* 0..4095, blocking ~2 us */

/* ---- DMA ring (TODO RY-1.5..1.7) ----------------------------------- */
#define ADC_RING_SLOTS 1024U /* power of two, 16-bit slots */
/* claims DMA with
   dma_claim_unused_channel(true) */
int32_t  drv_adc_ring_start(void);
uint32_t drv_adc_ring_head(void);             /* absolute sample count */
uint16_t drv_adc_ring_at(uint32_t abs_index); /* channel = abs_index % 2 */
int32_t  drv_adc_ring_dma_channel(void);      /* for the boot log (R6) */

#endif /* DRV_ADC_H */

/*** end of file ***/
