/**
 * App helpers for the flash DFU bootloader (KEY+NRST entry).
 * USB 0xFE → BL DFU flag + SystemReset.
 */
#ifndef SYSTEM_BOOT_H
#define SYSTEM_BOOT_H

#include <stdint.h>

void system_boot_request_reset(void);
uint8_t system_boot_pending(void);
void system_boot_reset_into_dfu(void) __attribute__((noreturn));

#endif /* SYSTEM_BOOT_H */
