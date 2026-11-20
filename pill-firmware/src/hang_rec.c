/**
 * Hang recorder (3.13), see hang_rec.h.
 *
 * Why: 3.12's watchdog reset clears the hang, and the evidence with it. Seen
 * 2026-10-24 19:39 on the Pi: EP0 timeouts for 11 minutes, then a watchdog reset.
 *
 * Record words (CMD_AUDIO_STATS wIndex 1, audio-stats.py prints them):
 *   0  bit 0 valid, bits 8-15 stall ticks (100 ms) at the last update
 *   1  ms since power-up at the capture (stops if SysTick is starved)
 *   2  USB interrupts at the capture
 *   3  USB interrupts at the last update (3 - 2 = interrupts while stuck)
 *   4  PC, 5 LR, 6 xPSR of the code TIM11 interrupted
 *      (xPSR bits 0-8: 0 = main loop, else exception number; OTG_FS = 83)
 *   7-9   NVIC active bits IRQ 0-31 / 32-63 / 64-95
 *   10 GINTSTS, 11 DIEPEMPMSK, 12 DIEPCTL0, 13 DIEPTSIZ0, 14 DIEPINT0, 15 DOEPINT0
 */
#include "hang_rec.h"
#include "usb_device.h"
#include "stm32f4xx_hal.h"

#define HANG_REC_TICKS 5u            /* 0.5 s, under the watchdog's 1.4 s minimum */
#define HANG_REC_MAGIC 0x48414E47u   /* "HANG" */

typedef struct {
    uint32_t magic;
    uint32_t w[HANG_REC_WORDS];
} hang_rec_t;

/* Not cleared by the startup code (linker .noinit, after .bss, clear of the
 * bootloader's RAM at the bottom and its stack at the top). */
static hang_rec_t s_rec __attribute__((section(".noinit")));
static uint32_t s_last[HANG_REC_WORDS];      /* last boot's record, for the host */
static volatile uint8_t s_stall;

#define OTG_DEV  ((USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define OTG_IN0  ((USB_OTG_INEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE))
#define OTG_OUT0 ((USB_OTG_OUTEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE))

void hang_rec_boot(uint8_t reset_flags)
{
    uint32_t i;

    if (s_rec.magic == HANG_REC_MAGIC && (reset_flags & 0x20u) != 0u) {
        for (i = 0; i < HANG_REC_WORDS; i++) {
            s_last[i] = s_rec.w[i];
        }
        s_last[0] |= 1u;
    }
    s_rec.magic = 0u;
}

void hang_rec_start(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    (void)RCC->APB2ENR;
    TIM11->PSC = 8400u - 1u;         /* 84 MHz -> 10 kHz */
    TIM11->ARR = 1000u - 1u;         /* 10 Hz */
    TIM11->EGR = TIM_EGR_UG;
    TIM11->SR = 0u;
    TIM11->DIER = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);   /* above USB / I2S (1) */
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    TIM11->CR1 = TIM_CR1_CEN;
}

void hang_rec_alive(void)
{
    s_stall = 0u;
}

uint16_t hang_rec_read(uint8_t *buf, uint16_t cap)
{
    uint32_t i;

    if (cap < sizeof(s_last)) {
        return 0;
    }
    for (i = 0; i < HANG_REC_WORDS; i++) {
        buf[i * 4u + 0u] = (uint8_t)s_last[i];
        buf[i * 4u + 1u] = (uint8_t)(s_last[i] >> 8);
        buf[i * 4u + 2u] = (uint8_t)(s_last[i] >> 16);
        buf[i * 4u + 3u] = (uint8_t)(s_last[i] >> 24);
    }
    return (uint16_t)sizeof(s_last);
}

/* frame = the exception stack frame of the interrupted code (r0-r3, r12, LR, PC, xPSR) */
void hang_rec_tick(const uint32_t *frame)
{
    uint8_t n;

    TIM11->SR = ~TIM_SR_UIF;
    n = s_stall;
    if (n < 255u) {
        s_stall = ++n;
    }
    if (n == HANG_REC_TICKS) {
        s_rec.w[0]  = (uint32_t)n << 8;
        s_rec.w[1]  = HAL_GetTick();
        s_rec.w[2]  = usb_otg_irq_count();
        s_rec.w[3]  = s_rec.w[2];
        s_rec.w[4]  = frame[6];
        s_rec.w[5]  = frame[5];
        s_rec.w[6]  = frame[7];
        s_rec.w[7]  = NVIC->IABR[0];
        s_rec.w[8]  = NVIC->IABR[1];
        s_rec.w[9]  = NVIC->IABR[2];
        s_rec.w[10] = USB_OTG_FS->GINTSTS;
        s_rec.w[11] = OTG_DEV->DIEPEMPMSK;
        s_rec.w[12] = OTG_IN0->DIEPCTL;
        s_rec.w[13] = OTG_IN0->DIEPTSIZ;
        s_rec.w[14] = OTG_IN0->DIEPINT;
        s_rec.w[15] = OTG_OUT0->DOEPINT;
        s_rec.magic = HANG_REC_MAGIC;
    } else if (n > HANG_REC_TICKS && s_rec.magic == HANG_REC_MAGIC) {
        s_rec.w[0] = (uint32_t)n << 8;
        s_rec.w[3] = usb_otg_irq_count();
    }
}

/* Find the interrupted code's stack frame (MSP or PSP), then the C handler. */
__attribute__((naked)) void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    __asm volatile(
        "tst   lr, #4          \n"
        "ite   eq              \n"
        "mrseq r0, msp         \n"
        "mrsne r0, psp         \n"
        "b     hang_rec_tick   \n");
}
