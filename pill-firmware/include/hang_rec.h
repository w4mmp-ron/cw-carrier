#ifndef HANG_REC_H
#define HANG_REC_H

#include <stdint.h>

/*
 * Hang recorder (3.13). TIM11 interrupts every 100 ms at the highest priority,
 * so it still runs when the USB or I2S interrupt never returns. If the main
 * loop has not called hang_rec_alive() for HANG_REC_TICKS ticks, it saves where
 * the CPU was and the EP0 registers in RAM that survives the watchdog reset.
 * After that reset the record is read with CMD_AUDIO_STATS, wIndex 1.
 */
#define HANG_REC_WORDS 16u

/** At power-up, before anything else uses RAM state: keep last boot's record
 *  if this start was a watchdog reset. reset_flags = RCC_CSR >> 24. */
void hang_rec_boot(uint8_t reset_flags);
void hang_rec_start(void);
void hang_rec_alive(void);
/** Last boot's record (HANG_REC_WORDS LE uint32, word 0 bit 0 = valid). */
uint16_t hang_rec_read(uint8_t *buf, uint16_t cap);

#endif /* HANG_REC_H */
