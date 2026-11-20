/**
 * I2S2 + I2S2ext full duplex to PCM3060 (Black Pill pin map).
 */
#ifndef I2S_AUDIO_H
#define I2S_AUDIO_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

extern I2S_HandleTypeDef hi2s2;
extern I2S_HandleTypeDef hi2s2ext;

void    i2s_audio_init(void);
void    i2s_audio_start(void);
void    i2s_audio_stop(void);
uint8_t i2s_audio_running(void);

/** 0 = not started, 1 = full-duplex DMA OK, 2 = FDX failed (TX-only fallback). */
uint8_t i2s_audio_fdx_status(void);

/** How many DMA buffer-complete swaps have run (Tx-only or full-duplex). */
uint32_t i2s_audio_cplt_count(void);

/** 1 if any non-zero sample seen in s_rx_dma since start. */
uint8_t i2s_audio_rx_saw_nonzero(void);

/** 1 once any RX sample has differed from the previous one (same channel). */
uint8_t i2s_audio_rx_saw_change(void);

/** 1 if RX has looked like live ADC data (at least 1/8 of the samples in
 *  each half-buffer changing) continuously for about the last second.
 *  Not latched. */
uint8_t i2s_audio_rx_steady(void);

/** Clear the rx_saw_nonzero latch and the steady-RX run (after codec reset/settle, so "O" only
 *  reflects real ADC data, not start-up glitches). */
void    i2s_audio_rx_nonzero_clear(void);

/** 1 if SPI2 TX DMA NDTR has changed since start (I2S clocks shifting). */
uint8_t i2s_audio_dma_tx_moving(void);

/** 1 if I2S2ext RX DMA NDTR has changed since start. */
uint8_t i2s_audio_dma_rx_moving(void);

/** Call from main loop to sample DMA counters. */
void i2s_audio_dma_poll(void);

/** 1 if RCC PLLI2S ready flag is set. */
uint8_t i2s_audio_plli2s_ready(void);

/** 1 if SPI2 I2SCFGR.I2SE is set (I2S block enabled). */
uint8_t i2s_audio_i2se_enabled(void);

/** 1 if polled TXE+DR writes succeeded with I2SE on (bit clock alive). */
uint8_t i2s_audio_txe_poll_ok(void);

/** Receive slip recovery (3.18): stop + start the full-duplex DMA, no TXE
 *  probe. The codec must be reset and set up again after this. */
void    i2s_audio_restart(void);

/** 1 when ~100 ms of live RX had no negative sample on I or Q (slipped). */
uint8_t i2s_audio_rx_slipped(void);

/** Test: shift RX right one bit (fake slip) until the next restart. */
void    i2s_audio_rx_sim_slip(void);

/** Test (3.21): 3 false pulses on the bit clock (0) or word select (1) pin,
 *  as RF on that line would make; can put the receive out of step for real. */
void    i2s_audio_test_glitch(uint8_t ws);

/** I2S2ext status flags now (reading clears FRE). */
#define I2S_EXT_FRE 0x01u
#define I2S_EXT_OVR 0x02u
uint8_t i2s_audio_ext_flags(void);

/** SPI2_IRQHandler entry count since start. */
uint32_t i2s_audio_spi2_irq_count(void);

/** Called from DMA complete to rotate buffers / notify USB. */
void    i2s_audio_on_tx_half(void);
void    i2s_audio_on_tx_cplt(void);
void    i2s_audio_on_rx_half(void);
void    i2s_audio_on_rx_cplt(void);

#endif /* I2S_AUDIO_H */
