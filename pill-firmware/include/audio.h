/**
 * USB ↔ I2S audio path — port of PSoC audio.c (pio-async: asynchronous USB)
 */
#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>

#define RX_ENDPOINT   2u  /* USB IN  0x82 — device → host (RX IQ) */
#define TX_ENDPOINT   3u  /* USB OUT 0x03 — host → device (TX IQ) */
#define RX_INTERFACE  2u
#define TX_INTERFACE  3u

/* Largest USB OUT audio packet in bytes: (96 + 1) stereo 16-bit frames. */
#define CFG_AUDIO_EP_OUT_SZ  388u

extern uint8_t Audio_IQ_Channels;
extern uint8_t E_Audio_running;

void Audio_Start(void);
void Audio_Main(void);
void Audio_Stop(void);

/** Build the CW raised-cosine keying table (call once at start-up). */
void audio_cw_shape_init(void);

/** Main loop (TX_Main): TX envelope target outside CW, 1 = ramp up / stay up. */
void audio_tx_envelope(uint8_t on);

/** USB stack: an OUT (TX IQ) packet arrived (LED diagnostic only). */
void audio_usb_out_packet_seen(void);

/** I2S interrupt, once per DMA half-buffer (I2S_BUF_SIZE bytes). */
void audio_i2s_tx_block(uint8_t *dst);
void audio_i2s_rx_block(const uint8_t *src);

/*
 * cw-carrier (3.90): stored CW carrier. The host sends one 1 ms block (I2S_BUF_SIZE = 384
 * bytes = 96 stereo 16-bit frames, exactly 12 cycles of the 12 kHz carrier) in
 * CW_CARRIER_PIECES pieces of CW_CARRIER_PIECE bytes (CMD_SET_CW_CARRIER, wValue = piece).
 * The last piece makes the block active. In CW with an active block the pill plays it instead
 * of the USB transmit audio (so the host need not stream in CW). No block = as before.
 */
#define CW_CARRIER_PIECE   64u
#define CW_CARRIER_PIECES  6u    /* 6 x 64 = 384 = I2S_BUF_SIZE */

/** USB (main loop): store piece n (0..CW_CARRIER_PIECES-1); 1 = block now active. */
uint8_t audio_cw_carrier_piece(uint8_t n, const uint8_t *data, uint16_t len);
/** USB (main loop): forget the stored block (back to the USB carrier). */
void audio_cw_carrier_clear(void);
/** Status for the host: bit 0 = feature present (always 1), bit 1 = a block is active. */
uint8_t audio_cw_carrier_status(void);

/** TX diagnostic, last 100 ms: 0 = no USB OUT packets, 1 = packets all
 *  zero, 2 = packets with live audio. */
uint8_t audio_tx_diag(void);

/** Counters since power-up (should stay 0 once a stream has settled). */
uint32_t audio_tx_underruns(void);
uint32_t audio_tx_dropped(void);
uint32_t audio_rx_dropped(void);

/** OUT FIFO fill (bytes) min / max since the last audio_out_level_reset(). */
uint16_t audio_out_level_min(void);
uint16_t audio_out_level_max(void);
void audio_out_level_reset(void);

#endif /* AUDIO_H */
