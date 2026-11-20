#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "board_pins.h"

void board_init(void);
void board_led_on(void);
void board_led_off(void);
void board_led_toggle(void);
void board_delay_ms(uint32_t ms);
uint32_t board_millis(void);

/** RCC_CSR reset flags (bits 24-31) seen at power-up: 0x80 low power, 0x40 window
 *  watchdog, 0x20 independent watchdog, 0x10 software, 0x08 power-on, 0x04 pin, 0x02 brown-out. */
uint8_t board_reset_flags(void);
void board_watchdog_start(void);
void board_watchdog_kick(void);

#endif /* BOARD_H */
