/**
 * App-side DFU request for flash bootloader (KEY+NRST is the normal entry).
 * 0xFE → DFU flag at 0x20000000 + SystemReset → BL stays in DFU.
 */
#include "system_boot.h"
#include "stm32f4xx_hal.h"

#define BL_BOOT_FLAG_ADDR  0x20000000UL
#define BL_DFU_FLAG        0xA5A5A5A5UL
#define BL_RESERVED        0x0U

typedef struct {
    volatile uint32_t flag;
    volatile uint32_t reserved;
} bl_boot_flag_t;

static volatile uint8_t s_enter_pending;

void system_boot_request_reset(void)
{
    s_enter_pending = 1;
}

uint8_t system_boot_pending(void)
{
    return s_enter_pending;
}

void system_boot_reset_into_dfu(void)
{
    bl_boot_flag_t *f = (bl_boot_flag_t *)BL_BOOT_FLAG_ADDR;
    f->flag = BL_DFU_FLAG;
    f->reserved = BL_RESERVED;
    __DSB();
    __ISB();
    NVIC_SystemReset();
    for (;;) {
    }
}
