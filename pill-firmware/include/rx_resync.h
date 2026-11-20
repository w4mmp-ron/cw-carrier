/**
 * Receive slip recovery (3.18). See rx_resync.c.
 */
#ifndef RX_RESYNC_H
#define RX_RESYNC_H

#include <stdint.h>

/** Main loop, every 7 ms (beater case 3). One step per call, never waits. */
void     rx_resync_poll(uint32_t now);

/** CMD_AUDIO_STATS wIndex 4: fills buf with RX_RESYNC_STATS_WORDS
 *  little-endian uint32, returns the byte count. wValue 2 = test: fake a
 *  slip first (the pill should find and fix it within ~0.2 s).
 *  3.21: wValue 3 / 4 = test: next poll while receiving, 3 false pulses on
 *  the bit clock / word select pin (as RF would), which can put the receive
 *  out of step for real (the pill should fix it within ~0.3 s).
 *  Word 7: state | waiting<<8 | slipped<<9 | armed<<10 | FRE run<<11 |
 *  test pending<<12 | restarts by FRE<<16 (layout unchanged, 8 words). */
#define RX_RESYNC_STATS_WORDS 8u
uint16_t rx_resync_stats(uint8_t *buf, uint16_t cap, uint16_t wValue);

#endif /* RX_RESYNC_H */
