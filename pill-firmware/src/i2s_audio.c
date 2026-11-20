/**
 * I2S2 full-duplex to PCM3060.
 * Pins: PB12 WS, PB13 CK, PB15 SD (TX→DIN), PB14 ext SD (RX←DOUT), PA3 MCK
 * (PA3 = I2S2_MCK — WeAct Black Pill has no PC6 header pin)
 *
 * Master transmit @ ~96 kHz, 16-bit data in 32-bit slots (BCK = 64 fS).
 * I2S2ext for full-duplex RX.
 * DMA circular double-buffer into/out of current PCM3060 Tx/Rx buffers.
 */
#include "i2s_audio.h"
#include "pcm3060.h"
#include "audio.h"
#include "sync_sof.h"
#include "board_pins.h"
#include <string.h>

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s2ext;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2ext_rx;

static uint8_t s_running = 0;
/* 0 = not started, 1 = full-duplex DMA OK, 2 = FDX failed (TX-only fallback). */
static uint8_t s_fdx_status = 0;

static void i2s_audio_swap_usb_buffers(uint16_t *tx_half, uint16_t *rx_half);
static volatile uint32_t s_cplt_count = 0;
static volatile uint8_t s_rx_saw_nonzero = 0;
/* 1 once any RX sample has differed from the previous one (same channel) */
static volatile uint8_t s_rx_saw_change = 0;
/* Consecutive RX half-buffers that look like live ADC data (see
 * i2s_audio_rx_steady()). Saturates at RX_STEADY_HALVES. */
static volatile uint16_t s_rx_live_run = 0;
static uint16_t s_last_tx_ndtr = 0;
static uint16_t s_last_rx_ndtr = 0;
static uint8_t s_ndtr_primed = 0;
static uint8_t s_tx_dma_moving = 0;
static uint8_t s_rx_dma_moving = 0;
static uint8_t s_txe_poll_ok = 0;
static volatile uint32_t s_spi2_irq_count = 0;
/* Receive slip check (3.18). Consecutive live RX half-buffers in which
 * neither channel had a single negative sample. Saturates at 0xFFFF. */
static volatile uint16_t s_rx_noneg_run = 0;
/* Test (3.18): 1 = shift every RX sample right one bit, as a real slip does.
 * Cleared by i2s_audio_restart() / start / stop. */
static volatile uint8_t s_rx_sim_slip = 0;

/*
 * DMA circular double buffer: two halves, each one PCM3060 audio quantum
 * (I2S_BUF_SIZE bytes ≈ 1 ms @ 96 kHz). HAL_I2SEx_TransmitReceive_DMA() is
 * started ONCE with the full buffer; DMA_CIRCULAR mode then loops it in
 * hardware forever. HAL_I2SEx_TxRxHalfCpltCallback refills/drains the first
 * half while DMA plays/records the second, and HAL_I2SEx_TxRxCpltCallback
 * does the same for the second half while DMA wraps back to the first —
 * classic ping-pong buffering. Neither callback restarts the transfer.
 */
#define I2S_DMA_HALF_WORDS ((uint16_t)(I2S_BUF_SIZE / 2u))
#define I2S_DMA_FULL_WORDS ((uint16_t)(I2S_DMA_HALF_WORDS * 2u))
/* A half-buffer counts as live ADC data when at least 1/8 of its samples
 * differ from the previous sample of the same channel. Real ADC noise keeps
 * the values moving; a stuck ADC gives a constant (seen: every sample -335,
 * which the old "non-zero" test wrongly passed). 1000 live halves in a row
 * ~= 1 s. */
#define RX_LIVE_MIN_CHANGES ((uint16_t)(I2S_DMA_HALF_WORDS / 8u))
#define RX_STEADY_HALVES    1000u
/*
 * Receive slip (found 2026-11-17, Pi capture after a high-SWR TUNE): the
 * codec -> STM32 data arrived one bit clock late, so each 16-bit sample read
 * as the real one shifted right one bit with a 0 on top: never negative,
 * small negatives read near +32767. Only a codec + I2S restart (NRST) cleared
 * it. Real receive (ADC noise, high-pass filtered) is negative about half the
 * time, so ~100 ms of live data with no negative sample on I or Q = slipped.
 * 3.19: and each channel must show the slip's own pattern in the same 1 ms:
 * values near 0 (real small positives) AND near +32767 (real small
 * negatives). 3.18 also fired once at power-up, when the codec input was
 * still settling and every sample sat on one side of zero; a settling offset
 * or a transient stays on one side and has only one of the two.
 */
#define RX_SLIP_HALVES      100u
#define RX_SLIP_LOW         0x1000u  /* below: "near 0" */
#define RX_SLIP_HIGH        0x7000u  /* at or above (and not negative): "near +32767" */

static uint16_t s_tx_dma[I2S_DMA_FULL_WORDS];
static uint16_t s_rx_dma[I2S_DMA_FULL_WORDS];

static void i2s_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB12 WS, PB13 CK, PB15 SD — AF5 I2S2 */
    g.Pin = BOARD_I2S_LRCK_PIN | BOARD_I2S_BCK_PIN | BOARD_I2S_DIN_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);

    /* PB14 I2S2ext_SD — AF6 on F411
     * Experiment 3 (DOUT_PULLUP=1): weak pull-up on PB14. If the codec is
     * not driving DOUT, RX reads all ones (-1); if it is, the data is
     * unchanged. Set to 0 to go back to no pull. */
#define DOUT_PULLUP 1
    g.Pin = BOARD_I2S_DOUT_PIN;
#if DOUT_PULLUP
    g.Pull = GPIO_PULLUP;
#endif
#ifdef GPIO_AF6_SPI2
    g.Alternate = GPIO_AF6_SPI2;
#else
    g.Alternate = GPIO_AF6_I2S2ext;
#endif
    HAL_GPIO_Init(GPIOB, &g);
    g.Pull = GPIO_NOPULL;

    /* PA3 MCK — AF5 I2S2_MCK (Black Pill: PC6 not on headers)
     * HIGH (not VERY_HIGH): softer edges on the 24.6 MHz MCLK through J5 to
     * cut ringing at codec SCKI (pins 6/9). HIGH is still rated for 50 MHz. */
    g.Pin = BOARD_I2S_MCLK_PIN;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(BOARD_I2S_MCLK_GPIO, &g);
}

static void i2s_dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* SPI2_TX = DMA1 Stream4 Channel0 */
    hdma_spi2_tx.Instance = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_spi2_tx);
    __HAL_LINKDMA(&hi2s2, hdmatx, hdma_spi2_tx);

    /* I2S2_EXT_RX = DMA1 Stream3 Channel3 */
    hdma_i2s2ext_rx.Instance = DMA1_Stream3;
    hdma_i2s2ext_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_i2s2ext_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2s2ext_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s2ext_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2s2ext_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2ext_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2ext_rx.Init.Mode = DMA_CIRCULAR;
    hdma_i2s2ext_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2s2ext_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_i2s2ext_rx);
    /*
     * HAL_I2SEx_TransmitReceive_DMA() uses hi2s->hdmarx on the primary handle.
     * DMA Parent must stay hi2s2 (link only once — a second LINKDMA would
     * overwrite Parent and break TxRx callbacks).
     */
    __HAL_LINKDMA(&hi2s2, hdmarx, hdma_i2s2ext_rx);
    hi2s2ext.hdmarx = &hdma_i2s2ext_rx;

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

void i2s_audio_init(void)
{
    __HAL_RCC_SPI2_CLK_ENABLE();

    i2s_gpio_init();

    /*
     * I2S master TX, Philips standard, 16-bit data in a 32-bit channel slot.
     * AudioFreq 96k — PLLI2S configured below.
     *
     * 16B_EXTENDED (not 16B): PCM3060 datasheet, "Audio Interface Mode and
     * Timing": in slave mode "BCK1/2 must be either 64 fS or 48 fS". Plain
     * 16B gives 16-bit slots = BCK 32 fS, which the codec doesn't support.
     * 16B_EXTENDED gives 32-bit slots = BCK 64 fS. The codec's default
     * 24-bit I2S format then takes our 16 bits as the MSBs (TX), and we keep
     * the 16 MSBs of its 24-bit word (RX). Still one 16-bit DR access per
     * channel, so the HAL DMA Size and buffer sizes are unchanged (HAL only
     * doubles Size for 24B/32B). The prescaler math also comes out the same
     * (MCLK = 256 fS in both cases), so the sample rate is unchanged.
     */
    hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
    hi2s2.Init.CPOL = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    /*
     * ENABLE: this is what makes HAL_I2S_Init() program the I2S2ext register
     * block (via its internal I2S_FullDuplexConfig() step) to slave-receive,
     * mirroring this Standard/DataFormat/CPOL. Left DISABLE, I2S2ext is never
     * configured at all and the RX (codec→MCU) side stays permanently dead —
     * that was the state this file shipped in during TX-only IT bring-up.
     */
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;

    /*
     * PLLI2S for 96 kHz I2S clock.
     * F411 uses its own PLLI2SM (not main PLLM). VCO = HSE/M * N; I2SCLK = VCO/R.
     * HSE 25 MHz: M=22, N=173, R=2 → 25/22 * 173 / 2 = 98.30 MHz.
     * With MCLK on, fS = I2SCLK / (256 * 4) = 95.99 kHz (HAL picks DIV=2,
     * ODD=0). MCLK = I2SCLK/4 = 24.57 MHz with a 50 % duty cycle.
     * Was M=25, N=344 (172 MHz, divide by 7): odd divider → ~43 % MCLK duty,
     * only ~1 ns above the PCM3060's 40 % minimum high time.
     * N=192 gave 96 MHz → /(256*4) = 93.75 kHz.
     * (PLLI2SM was previously left 0 → PLLI2S never produced a valid clock.)
     */
    {
        RCC_PeriphCLKInitTypeDef p = {0};
        p.PeriphClockSelection = RCC_PERIPHCLK_I2S;
        p.PLLI2S.PLLI2SM = 22;
        p.PLLI2S.PLLI2SN = 173;
        p.PLLI2S.PLLI2SR = 2;
        if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) {
            return;
        }
    }

    if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
        /* Leave unusable; Start will no-op */
        return;
    }

    /*
     * hi2s2ext is intentionally never HAL_I2S_Init()'d or passed to any
     * HAL_I2SEx_* call directly — the extended full-duplex API takes only
     * the primary handle (hi2s2) and derives the I2S2ext register config
     * from HAL_I2S_Init()'s FullDuplexMode step above. This handle exists
     * only as documentation of the ext instance/mode; kept for reference.
     */
    hi2s2ext.Instance = I2S2ext;
    memcpy(&hi2s2ext.Init, &hi2s2.Init, sizeof(hi2s2.Init));
    hi2s2ext.Init.Mode = I2S_MODE_SLAVE_RX;

    i2s_dma_init();
    s_running = 0;
}

/*
 * Bring-up: polled-TXE probe first (proves the I2S bit/word clock is alive
 * without relying on NVIC or DMA — a hardware/clock-config sanity check),
 * then hand off to full-duplex DMA for the real, continuous stream.
 */
void i2s_audio_start(void)
{
    uint32_t spins;
    uint32_t saw;

    s_cplt_count = 0;
    s_rx_saw_nonzero = 0;
    s_rx_saw_change = 0;
    s_rx_live_run = 0;
    s_tx_dma_moving = 0;
    s_rx_dma_moving = 0;
    s_ndtr_primed = 0;
    s_last_tx_ndtr = 0;
    s_last_rx_ndtr = 0;
    s_txe_poll_ok = 0;
    s_spi2_irq_count = 0;
    s_rx_noneg_run = 0;
    s_rx_sim_slip = 0;

    /* Prime both halves of the circular buffer before DMA starts. */
    memcpy(&s_tx_dma[0], PCM3060_TxBuf(), I2S_BUF_SIZE);
    memcpy(&s_tx_dma[I2S_DMA_HALF_WORDS], PCM3060_TxBuf(), I2S_BUF_SIZE);
    memset(s_rx_dma, 0, sizeof(s_rx_dma));

    /* Polled TXE: proves I2S bit clock without relying on NVIC */
    __HAL_I2S_ENABLE(&hi2s2);
    saw = 0;
    for (spins = 0; spins < 800000u; spins++) {
        if ((SPI2->SR & SPI_SR_TXE) != 0u) {
            SPI2->DR = 0u;
            if (++saw >= 256u) {
                s_txe_poll_ok = 1;
                break;
            }
        }
    }
    __HAL_I2S_DISABLE(&hi2s2);
    hi2s2.State = HAL_I2S_STATE_READY;

    HAL_NVIC_SetPriority(SPI2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);

    /*
     * Single call — DMA_CIRCULAR loops s_tx_dma/s_rx_dma forever in hardware.
     * Do NOT call this again from the completion callbacks; that would
     * restart/reconfigure the stream every ~1 ms instead of letting DMA run.
     */
    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s2, s_tx_dma, s_rx_dma, I2S_DMA_FULL_WORDS) != HAL_OK) {
        s_fdx_status = 2;
        s_running = 0;
        return;
    }
    s_fdx_status = 1;
    s_running = 1;
}

/*
 * Receive slip recovery (3.18): stop and start the full-duplex DMA again,
 * without the polled TXE probe (that spins ~1.3 ms; the bit clock was already
 * proven at power-up, and its result is kept for the LED). The caller resets
 * and sets up the codec afterwards, with these clocks running.
 *
 * 3.22: SPI2 (I2S2 and its receive half I2S2ext) is also reset in the RCC and
 * set up again, as NRST does. 3.21's false-pulse test showed stop / start alone
 * leaves the receive out of step (frame errors came straight back, 16
 * restarts, only NRST cleared it). PLLI2S, the pins and DMA are left as they
 * are; HAL_I2S_Init() with State READY only writes the I2S registers.
 */
void i2s_audio_restart(void)
{
    uint8_t txe_ok = s_txe_poll_ok;

    i2s_audio_stop();
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    hi2s2.State = HAL_I2S_STATE_READY;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
        s_fdx_status = 2;
        s_running = 0;
        return;
    }
    s_txe_poll_ok = txe_ok;
    s_rx_noneg_run = 0;
    s_rx_sim_slip = 0;
    memcpy(&s_tx_dma[0], PCM3060_TxBuf(), I2S_BUF_SIZE);
    memcpy(&s_tx_dma[I2S_DMA_HALF_WORDS], PCM3060_TxBuf(), I2S_BUF_SIZE);
    memset(s_rx_dma, 0, sizeof(s_rx_dma));
    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s2, s_tx_dma, s_rx_dma, I2S_DMA_FULL_WORDS) != HAL_OK) {
        s_fdx_status = 2;
        s_running = 0;
        return;
    }
    s_fdx_status = 1;
    s_running = 1;
}

uint8_t i2s_audio_rx_slipped(void)
{
    return (s_rx_noneg_run >= RX_SLIP_HALVES) ? 1u : 0u;
}

void i2s_audio_rx_sim_slip(void)
{
    s_rx_sim_slip = 1;
}

/*
 * 3.21 test: copy what RF on a clock line most likely does. For under 1 us the
 * pin (PB13 bit clock or PB12 word select) is taken from I2S, driven as a
 * plain output and toggled 3 times (~100 ns per level), then handed back to
 * I2S. The codec and the pill's receive can see the false edges, as with
 * interference. (3.20 tried re-enabling I2S2ext mid-frame instead: the STM32
 * lined itself up again every time.) Only from rx_resync.c, on request.
 */
static void glitch_wait(void)
{
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
}

void i2s_audio_test_glitch(uint8_t ws)
{
    uint32_t pin = ws ? 12u : 13u;
    uint32_t bit = 1u << pin;
    uint32_t moder, mout;
    uint8_t i;

    __disable_irq();
    moder = GPIOB->MODER;
    mout = (moder & ~(3u << (pin * 2u))) | (1u << (pin * 2u));
    GPIOB->BSRR = (GPIOB->IDR & bit) ? bit : (bit << 16);   /* start at the level it has */
    GPIOB->MODER = mout;
    for (i = 0; i < 6u; i++) {          /* 6 changes = 3 pulses */
        GPIOB->ODR ^= bit;
        glitch_wait();
    }
    GPIOB->MODER = moder;               /* back to I2S (alternate function) */
    __enable_irq();
}

uint8_t i2s_audio_ext_flags(void)
{
    /* Reading SR clears FRE. OVR needs a DR read to clear, which the DMA does. */
    uint32_t sr = I2S2ext->SR;
    return (uint8_t)(((sr & SPI_SR_FRE) ? I2S_EXT_FRE : 0u) | ((sr & SPI_SR_OVR) ? I2S_EXT_OVR : 0u));
}

void i2s_audio_stop(void)
{
    s_running = 0;
    (void)HAL_I2S_DMAStop(&hi2s2);
    __HAL_I2S_DISABLE_IT(&hi2s2, I2S_IT_TXE | I2S_IT_RXNE | I2S_IT_ERR);
    __HAL_I2SEXT_DISABLE_IT(&hi2s2, I2S_IT_TXE | I2S_IT_RXNE | I2S_IT_ERR);
    __HAL_I2S_DISABLE(&hi2s2);
    __HAL_I2SEXT_DISABLE(&hi2s2);
    hi2s2.State = HAL_I2S_STATE_READY;
    s_fdx_status = 0;
    s_cplt_count = 0;
    s_rx_saw_nonzero = 0;
    s_rx_saw_change = 0;
    s_rx_live_run = 0;
    s_tx_dma_moving = 0;
    s_rx_dma_moving = 0;
    s_ndtr_primed = 0;
    s_txe_poll_ok = 0;
    s_spi2_irq_count = 0;
    s_rx_noneg_run = 0;
    s_rx_sim_slip = 0;
}

uint8_t i2s_audio_running(void)
{
    return s_running;
}

uint8_t i2s_audio_fdx_status(void)
{
    return s_fdx_status;
}

uint32_t i2s_audio_cplt_count(void)
{
    return s_cplt_count;
}

uint8_t i2s_audio_rx_saw_nonzero(void)
{
    return s_rx_saw_nonzero;
}

uint8_t i2s_audio_rx_saw_change(void)
{
    return s_rx_saw_change;
}

uint8_t i2s_audio_rx_steady(void)
{
    return (s_rx_live_run >= RX_STEADY_HALVES) ? 1u : 0u;
}

void i2s_audio_rx_nonzero_clear(void)
{
    s_rx_saw_nonzero = 0;
    s_rx_saw_change = 0;
    s_rx_live_run = 0;
}

void i2s_audio_dma_poll(void)
{
    /* Busy-wait NDTR sample — old 1 ms poll aliased against 1 ms buffer period. */
    uint16_t tx0;
    uint16_t tx1;
    uint16_t rx0;
    uint16_t rx1;
    volatile uint32_t spin;

    if (!s_running) {
        return;
    }
    tx0 = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_spi2_tx);
    rx0 = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_i2s2ext_rx);
    for (spin = 0; spin < 2000u; spin++) {
    }
    tx1 = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_spi2_tx);
    rx1 = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_i2s2ext_rx);
    if (tx0 != tx1) {
        s_tx_dma_moving = 1;
    }
    if (rx0 != rx1) {
        s_rx_dma_moving = 1;
    }
    (void)s_ndtr_primed;
    (void)s_last_tx_ndtr;
    (void)s_last_rx_ndtr;
}

uint8_t i2s_audio_dma_tx_moving(void)
{
    return s_tx_dma_moving;
}

uint8_t i2s_audio_dma_rx_moving(void)
{
    return s_rx_dma_moving;
}

uint8_t i2s_audio_plli2s_ready(void)
{
    return (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY) != RESET) ? 1u : 0u;
}

uint8_t i2s_audio_i2se_enabled(void)
{
    return ((SPI2->I2SCFGR & SPI_I2SCFGR_I2SE) != 0u) ? 1u : 0u;
}

uint8_t i2s_audio_txe_poll_ok(void)
{
    return s_txe_poll_ok;
}

uint32_t i2s_audio_spi2_irq_count(void)
{
    return s_spi2_irq_count;
}

/* Refill/drain one half of the circular DMA buffer (ping-pong). */
static void i2s_audio_swap_usb_buffers(uint16_t *tx_half, uint16_t *rx_half)
{
    uint16_t i;
    const uint16_t nwords = I2S_DMA_HALF_WORDS;
    uint16_t nonzero = 0;
    uint16_t changes = 0;
    uint16_t neg_l = 0;
    uint16_t neg_r = 0;
    uint8_t  ends = 0;      /* bits: 1 L low, 2 L high, 4 R low, 8 R high */

    s_cplt_count++;
    if (s_rx_sim_slip) {    /* test: make the data look like a real slip */
        for (i = 0; i < nwords; i++) {
            rx_half[i] = (uint16_t)(rx_half[i] >> 1);
        }
    }
    for (i = 0; i < nwords; i++) {
        if (rx_half[i] != 0) {
            nonzero++;
        }
        if (rx_half[i] & 0x8000u) {
            if (i & 1u) {
                neg_r++;
            } else {
                neg_l++;
            }
        } else if (rx_half[i] < RX_SLIP_LOW) {
            ends |= (uint8_t)((i & 1u) ? 4u : 1u);
        } else if (rx_half[i] >= RX_SLIP_HIGH) {
            ends |= (uint8_t)((i & 1u) ? 8u : 2u);
        }
        /* Words alternate L, R: compare with the same channel's last sample */
        if (i >= 2u && rx_half[i] != rx_half[i - 2u]) {
            changes++;
        }
    }
    if (nonzero != 0u) {
        s_rx_saw_nonzero = 1;
    }
    if (changes != 0u) {
        s_rx_saw_change = 1;
    }
    if (changes >= RX_LIVE_MIN_CHANGES) {
        if (s_rx_live_run < RX_STEADY_HALVES) {
            s_rx_live_run++;
        }
    } else {
        s_rx_live_run = 0;
    }
    /* Slip check: live data (so a silent or held codec never counts) with no
     * negative sample on either channel, and both ends on both channels. */
    if (changes >= RX_LIVE_MIN_CHANGES && neg_l == 0u && neg_r == 0u && ends == 0x0Fu) {
        if (s_rx_noneg_run < 0xFFFFu) {
            s_rx_noneg_run++;
        }
    } else {
        s_rx_noneg_run = 0;
    }

    /* Asynchronous USB: RX block into the USB IN FIFO, next TX block out of
     * the USB OUT FIFO (audio.c). */
    audio_i2s_rx_block((const uint8_t *)rx_half);
    audio_i2s_tx_block((uint8_t *)tx_half);
    sync_sof_on_i2s_frame(); /* one buffer ≈ 1 ms @ 96 kHz */
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    (void)hi2s;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    (void)hi2s;
}

/* DMA wrapped back to the start of the buffer — second half just finished. */
void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance != SPI2) {
        return;
    }
    i2s_audio_swap_usb_buffers(&s_tx_dma[I2S_DMA_HALF_WORDS], &s_rx_dma[I2S_DMA_HALF_WORDS]);
    /* DMA_CIRCULAR — hardware already wrapped and is playing/recording the
     * first half again. No restart call here. */
}

/* DMA reached the midpoint — first half just finished. */
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance != SPI2) {
        return;
    }
    i2s_audio_swap_usb_buffers(&s_tx_dma[0], &s_rx_dma[0]);
}

void SPI2_IRQHandler(void)
{
    s_spi2_irq_count++;
    HAL_I2S_IRQHandler(&hi2s2);
}

void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_i2s2ext_rx);
}

void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi2_tx);
}

void i2s_audio_on_tx_half(void) {}
void i2s_audio_on_tx_cplt(void) {}
void i2s_audio_on_rx_half(void) {}
void i2s_audio_on_rx_cplt(void) {}
