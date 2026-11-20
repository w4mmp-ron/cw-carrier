/**
 * PCM3060 codec — port of PSoC pcm3060.c control path.
 * Data plane is I2S (see i2s_audio.h); this module is I2C register control.
 */
#ifndef PCM3060_H
#define PCM3060_H

#include <stdint.h>

/*
 * PCM3060 7-bit I2C address = 0x46 (1000110b, ADR pin low).
 * The PSoC source passed 0x46 to I2C_DISPLAY_MasterWriteBuf(), which takes a
 * right-justified 7-bit address and shifts it left itself
 * (slaveAddress << I2C_DISPLAY_SLAVE_ADDR_SHIFT) -- so 0x46 was already the
 * 7-bit address, not an 8-bit write address. i2c_bus.c shifts addr7 << 1.
 * (Was 0x23 here, which addressed nothing -> Morse "C" in the transceiver.)
 */
#define PCM3060_I2C_ADDR7   0x46u

#define USB_AUDIO_BUFS      3u
/* 96 stereo frames × 2 ch × 2 bytes = 384 = 1 ms @ 96 kHz (matches USB max packet) */
#define I2S_BUF_SIZE        (96u * 2u * 2u)
#define AUDIO_SAMPLE_RATE   96000u

uint8_t  PCM3060_SetRegister(uint8_t reg, uint8_t val);
uint8_t  PCM3060_Init(void);
void     PCM3060_Start(void);
uint8_t  PCM3060_Retry(void);   /* power-up retry after a failed start (3.4) */
uint8_t  PCM3060_Stop(void);
void     PCM3060_Adj_Output_Volume(uint8_t level);

/* Start-up timing (pcm3060.c has the datasheet notes). Also used by the
 * receive slip recovery (rx_resync.c), which runs the same steps without
 * blocking. */
#define PCM3060_CLOCK_LOCK_MS    5u   /* codec locks onto MCLK/BCK/LRCK */
#define PCM3060_RESET_LOW_MS     30u  /* >= 2048/fS */
#define PCM3060_RESET_SETTLE_MS  30u
#define PCM3060_PSV_SETTLE_MS    30u  /* > 2048/fS after power-save release */

/* The steps of the codec set-up, for rx_resync.c (3.18), clocks running:
 * reset pin low, wait, high, wait, PCM3060_WriteSetup(), wait, then
 * PCM3060_FinishSetup() with its result. */
void     PCM3060_ResetPin(uint8_t released);
uint8_t  PCM3060_WriteSetup(void);          /* 0 = OK, 1 = an I2C write failed */
void     PCM3060_FinishSetup(uint8_t fail);

/** 0 = not started, 1 = I2C wake/config ACK OK, 2 = I2C failed (no talk to PCM3060). */
uint8_t  PCM3060_i2c_status(void);

/** Register read-back after start-up (experiment 1):
 *  0 = not done, 1 = all match, 2 = read OK but a value differs,
 *  3 = codec did not answer the read. */
uint8_t  PCM3060_readback_status(void);

uint8_t *PCM3060_TxBuf(void);  /* host → radio (USB OUT / I2S TX to codec DIN) */
uint8_t *PCM3060_RxBuf(void);  /* radio → host (I2S RX from codec DOUT / USB IN) */

void     PCM3060_SetTxBufAddress(uint8_t *src); /* CW tone inject hook */
void     PCM3060_SetTxBufAddressDefault(void);

/** Triple buffers (USB_AUDIO_BUFS × I2S_BUF_SIZE). */
extern uint8_t RxI2S[USB_AUDIO_BUFS][I2S_BUF_SIZE];
extern uint8_t TxI2S[USB_AUDIO_BUFS][I2S_BUF_SIZE];

/** Index selected by SOF / audio tick (0..USB_AUDIO_BUFS-1). */
uint8_t  audio_buf_index(void);
void     audio_buf_advance(void);

#endif /* PCM3060_H */
