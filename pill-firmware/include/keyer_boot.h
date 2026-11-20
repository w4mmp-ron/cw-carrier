#ifndef KEYER_BOOT_H
#define KEYER_BOOT_H

/*
 * 3.16: CMD_KEYER_BOOT 0x9F, I2C pass-through to the PIC keyer, used by the Pi's
 * keyer-flash.py (with mscc stopped) to update the keyer through its I2C bootloader.
 *
 * OUT 0x9F  wValue = 7-bit I2C address (0x40 keyer app or 0x5C keyer bootloader only)
 *           wIndex = bytes to read after the write (0..16)
 *           data   = bytes to write (0..18); 0 bytes and 0 to read = address probe
 * IN  0x9F  returns state, count, then count bytes read
 *           state 0 idle, 1 pending, 2 done OK, 3 failed (NACK / timeout), 4 refused
 * The transfer runs on the next main-loop beat (case 3); the host asks again until
 * the state is not 1. Each transfer is short (18 bytes ~2 ms at 100 kHz, blocking HAL).
 */
#include <stdint.h>

#define KEYER_BOOT_ADDR     0x5C
#define KEYER_BOOT_MAX_WR   18u
#define KEYER_BOOT_MAX_RD   16u

#define KB_IDLE     0u
#define KB_PENDING  1u
#define KB_OK       2u
#define KB_FAIL     3u
#define KB_REFUSED  4u

/* USB setup stage of OUT 0x9F: returns the write buffer (NULL = refuse). A request with
 * wLength 0 is queued here at once. */
uint8_t *keyer_boot_setup(uint16_t wValue, uint16_t wIndex, uint16_t wLength);
/* USB data stage of OUT 0x9F done: queue the transfer */
void     keyer_boot_out_done(uint16_t len);
/* IN 0x9F: state, count, data. Returns bytes put in buf. */
uint16_t keyer_boot_result(uint8_t *buf, uint16_t cap);
/* Main loop, one call per beat: does at most one queued transfer */
void     keyer_boot_poll(void);

#endif /* KEYER_BOOT_H */
