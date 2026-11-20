/**
 * PCM3060 I2C control + triple audio buffers (PSoC pcm3060.c port).
 */
#include "pcm3060.h"
#include "board.h"
#include "board_pins.h"
#include "i2c_bus.h"
#include "i2s_audio.h"
#include <string.h>

/*
 * PCM3060 start-up timing (TI datasheet):
 *  - RST must be held low for at least 2048/fS (~22 ms at 93.75 kHz), and the
 *    internal reset only releases with the system clock (SCKI) running.
 *  - Leaving power-save (ADPSV/DAPSV) needs more than 2048/fS to settle.
 * On this board SCKI = I2S2_MCK (PA3), which only runs while I2S is enabled,
 * so the I2S clocks must be started BEFORE the reset pulse and register
 * writes. (The PSoC's codec clock ran from power-up, so it never had to care.)
 */
/* Timing defines (PCM3060_CLOCK_LOCK_MS etc.) are in pcm3060.h (3.18). */

/*
 * Experiment 2: reg 72 (0x48) bit 7 CSEL1 = 1 -> the ADC runs from the "2"
 * clock pins (SCKI2/BCK2/LRCK2, pins 9/10/11) instead of the "1" set (pins
 * 6/5/4). Both sets are driven from the same STM32 pins on the daughter board.
 * Other bits 0 = slave mode, 24-bit I2S (the reset defaults).
 * Set to 0 to go back to the PSoC behaviour.
 */
#define PCM3060_ADC_USE_CLK2     1

/* U2/J5 A28 RESET → PA2: hold PCM3060 in reset, then release (active-low).
 * Call only with the I2S clocks running (see above). */
void PCM3060_ResetPin(uint8_t released)
{
    HAL_GPIO_WritePin(BOARD_CODEC_RESET_GPIO, BOARD_CODEC_RESET_PIN,
                      released ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void pcm3060_hw_reset_pulse(void)
{
    PCM3060_ResetPin(0);
    board_delay_ms(PCM3060_RESET_LOW_MS);
    PCM3060_ResetPin(1);
    board_delay_ms(PCM3060_RESET_SETTLE_MS);
}

uint8_t RxI2S[USB_AUDIO_BUFS][I2S_BUF_SIZE];
uint8_t TxI2S[USB_AUDIO_BUFS][I2S_BUF_SIZE];

static volatile uint8_t s_buf_idx = 0;
static uint8_t *s_tx_override = 0;
/* 0 = not started, 1 = I2C OK, 2 = I2C failed */
static uint8_t s_i2c_status = 0;
/* See PCM3060_readback_status() */
static uint8_t s_readback_status = 0;

/* Registers written by PCM3060_Start(), with the bits we expect to read back.
 * Reg 0x40: only bits 7:4 (MRST, SRST, ADPSV, DAPSV) are compared. */
static const struct {
    uint8_t reg;
    uint8_t val;
    uint8_t mask;
} s_readback[] = {
    { 0x40, 0xC0, 0xF0 },
    { 0x45, 0x80, 0xFF },
    { 0x41, 0xFF, 0xFF },
    { 0x42, 0xFF, 0xFF },
#if PCM3060_ADC_USE_CLK2
    { 0x48, 0x80, 0x80 },
#endif
};

static uint8_t pcm3060_readback(void)
{
    uint8_t i, attempt, val;
    uint8_t status = 1;

    for (i = 0; i < sizeof(s_readback) / sizeof(s_readback[0]); i++) {
        for (attempt = 0; attempt < 8u; attempt++) {
            if (i2c_read_reg(PCM3060_I2C_ADDR7, s_readback[i].reg, &val)) {
                break;
            }
        }
        if (attempt >= 8u) {
            return 3;
        }
        if ((val & s_readback[i].mask) != (s_readback[i].val & s_readback[i].mask)) {
            status = 2;
        }
    }
    return status;
}

uint8_t audio_buf_index(void)
{
    return s_buf_idx;
}

void audio_buf_advance(void)
{
    s_buf_idx = (uint8_t)((s_buf_idx + 1u) % USB_AUDIO_BUFS);
}

uint8_t *PCM3060_TxBuf(void)
{
    if (s_tx_override) {
        return s_tx_override;
    }
    return TxI2S[s_buf_idx];
}

uint8_t *PCM3060_RxBuf(void)
{
    return RxI2S[s_buf_idx];
}

void PCM3060_SetTxBufAddress(uint8_t *src)
{
    s_tx_override = src;
}

void PCM3060_SetTxBufAddressDefault(void)
{
    s_tx_override = 0;
}

uint8_t PCM3060_SetRegister(uint8_t reg, uint8_t val)
{
    uint8_t attempt;
    for (attempt = 0; attempt < 8u; attempt++) {
        if (i2c_write_reg(PCM3060_I2C_ADDR7, reg, val)) {
            return 0; /* success like PSoC (0 = ok) */
        }
    }
    return 1;
}

uint8_t PCM3060_Init(void)
{
    memset(RxI2S, 0, sizeof(RxI2S));
    memset(TxI2S, 0, sizeof(TxI2S));
    s_buf_idx = 0;
    s_tx_override = 0;
    s_i2c_status = 0;
    s_readback_status = 0;
    /* Configure I2S/DMA only. Clocks, codec reset and codec registers are
     * all handled by PCM3060_Start(), in that order. */
    i2s_audio_init();
    return 0;
}

/* Step 3: same register writes as the PSoC. 0 = all OK, 1 = a write failed. */
uint8_t PCM3060_WriteSetup(void)
{
    uint8_t fail = 0;

    /* Non-zero from SetRegister = I2C NACK / bus fail */
    if (PCM3060_SetRegister(0x40, 0xC0) != 0) { /* Wakeup */
        fail = 1;
    }
    if (PCM3060_SetRegister(0x45, 0x80) != 0) { /* Slow rolloff */
        fail = 1;
    }
    if (PCM3060_SetRegister(0x41, 0xFF) != 0) { /* Volume L full */
        fail = 1;
    }
    if (PCM3060_SetRegister(0x42, 0xFF) != 0) { /* Volume R full */
        fail = 1;
    }
#if PCM3060_ADC_USE_CLK2
    if (PCM3060_SetRegister(0x48, 0x80) != 0) { /* CSEL1: ADC uses clock set 2 */
        fail = 1;
    }
#endif
    return fail;
}

/* Step 5, after the power-save settle time: read back, mark the codec status. */
void PCM3060_FinishSetup(uint8_t fail)
{
    /* Experiment 1: confirm the codec holds what was written. */
    s_readback_status = fail ? 0u : pcm3060_readback();

    /* Anything received during reset/settle doesn't count as codec data. */
    i2s_audio_rx_nonzero_clear();
    s_i2c_status = fail ? 2u : 1u;
}

/* Steps 2-5 of PCM3060_Start(): reset and configure the codec, clocks running */
static void pcm3060_configure(void)
{
    uint8_t fail;

    /* 2. Hardware reset of the codec, with the clocks running. */
    pcm3060_hw_reset_pulse();

    /* 3. Same register writes as the PSoC. */
    fail = PCM3060_WriteSetup();

    /* 4. Let the ADC/DAC leave power-save before anyone looks at the data. */
    board_delay_ms(PCM3060_PSV_SETTLE_MS);

    /* 5. */
    PCM3060_FinishSetup(fail);
}

void PCM3060_Start(void)
{
    /* 1. Clocks first: MCLK/BCK/LRCK run continuously from here on. */
    i2s_audio_start();
    board_delay_ms(PCM3060_CLOCK_LOCK_MS);
    pcm3060_configure();
}

/* Power-up retry (3.4): codec reset + setup again, clocks already running.
 * Blocks ~95 ms (reset and settle times); only called after a failed start. */
uint8_t PCM3060_Retry(void)
{
    pcm3060_configure();
    return (s_i2c_status == 1u) ? 1u : 0u;
}

uint8_t PCM3060_Stop(void)
{
    uint8_t ret = PCM3060_SetRegister(0x40, 0xF0); /* Sleep */
    i2s_audio_stop();
    s_i2c_status = 0;
    s_readback_status = 0;
    return ret;
}

uint8_t PCM3060_i2c_status(void)
{
    return s_i2c_status;
}

uint8_t PCM3060_readback_status(void)
{
    return s_readback_status;
}

void PCM3060_Adj_Output_Volume(uint8_t level)
{
    /* PSoC used band volume tables; simple dual-channel set here */
    (void)PCM3060_SetRegister(0x41, level);
    (void)PCM3060_SetRegister(0x42, level);
}
