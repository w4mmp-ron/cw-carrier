/**
 * Proficio STM32F411 entry.
 * Linked at 0x08008000 (32KB flash DFU bootloader at 0x08000000).
 * Field DFU: KEY (PA0) + NRST.
 */
#include "proficio_config.h"
#include "board.h"
#include "app.h"
#include "system_boot.h"
#include "usb_device.h"
#include "stm32f4xx.h"

#define APP_VECTOR_TABLE 0x08008000u

int main(void)
{
    SCB->VTOR = APP_VECTOR_TABLE;
    __DSB();
    __ISB();

    board_init();
    app_init();
    board_watchdog_start();   /* main loop stuck ~2 s -> reset (3.12) */

    for (;;) {
        board_watchdog_kick();

        if (system_boot_pending()) {
            board_delay_ms(50);
            system_boot_reset_into_dfu();
        }

        app_poll();

        /* PC13 belongs to app.c's diagnostic Morse player from boot (the
         * codec/I2S subsystem runs independently of USB). Do not add
         * board_led_*() calls here: they write the pin directly and would
         * fight Control_Write(CONTROL_LED). */
    }
}
