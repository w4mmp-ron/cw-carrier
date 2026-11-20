/**
 * Audio path main — USB <-> I2S, asynchronous (pio-async).
 *
 * The codec clock (95.99 kHz) is the master. Each I2S DMA half-buffer
 * (96 stereo frames, ~1 ms) the I2S interrupt:
 *   - puts the received block into TinyUSB's IN FIFO  (audio_i2s_rx_block)
 *   - takes the next block from TinyUSB's OUT FIFO    (audio_i2s_tx_block)
 * TinyUSB matches the USB side to that rate: flow control on the IN endpoint
 * (95/96/97 frames per packet) and a feedback endpoint for the OUT endpoint.
 * No sample is dropped or repeated when the two clocks drift.
 */
#include "audio.h"
#include "pcm3060.h"
#include "usb_device.h"
#include "control.h"
#include "cw.h"
#include "radio_state.h"
#include "proficio_config.h"
#include "stm32f4xx.h"
#include <string.h>
#include <math.h>

uint8_t Audio_IQ_Channels = 0;
uint8_t E_Audio_running = 0;

extern uint32_t board_millis(void);

/* TX diagnostic (LED letters M / I / T): when the last USB OUT packet
 * arrived, and when the last block holding any non-zero sample went out. */
#define TX_DIAG_WINDOW_MS 100u
static volatile uint32_t s_out_pkt_ms = 0;
static volatile uint32_t s_out_live_ms = 0;
static volatile uint8_t  s_out_pkt_seen = 0;
static volatile uint8_t  s_out_live_seen = 0;

/* OUT FIFO -> I2S: 1 once the FIFO has filled to half and blocks are being
 * taken; back to 0 on an underrun (then it refills to half first). */
static volatile uint8_t  s_tx_primed = 0;
static volatile uint32_t s_tx_underruns = 0; /* FIFO ran empty while primed */
static volatile uint32_t s_tx_dropped = 0;   /* blocks dropped, FIFO nearly full */
static volatile uint32_t s_rx_dropped = 0;   /* blocks dropped, IN FIFO full */
/* OUT FIFO fill (bytes) seen by the I2S interrupt since the last reset (3.11) */
static volatile uint16_t s_out_lvl_min = 0xFFFFu;
static volatile uint16_t s_out_lvl_max = 0u;

/* Last first-word of the previous TX block (one-sample delay, see
 * audio_i2s_tx_block). */
static uint16_t s_tx_first_prev = 0;

static uint8_t s_rx_block[I2S_BUF_SIZE];
static uint8_t s_tx_discard[I2S_BUF_SIZE];

/* cw-carrier (3.90): stored CW carrier, two copies. USB (main loop) writes the spare copy;
 * the I2S interrupt plays s_car[s_car_active]. CAR_NONE = no block, use the USB audio. */
#define CAR_NONE 0xFFu
static int16_t s_car[2][I2S_BUF_SIZE / 2u];
static volatile uint8_t s_car_active = CAR_NONE;
static uint8_t s_car_fill = 0u;     /* copy being written */
static uint8_t s_car_got = 0u;      /* pieces received, one bit each */

/*
 * TX raised-cosine envelope. Each TX sample is multiplied by s_cw_ramp[s_cw_ramp_pos]; the
 * position moves one step per sample toward the end the envelope target asks for, so rise
 * and fall each take CW_RAMP_MS. Table: gain = (1 - cos(pi * n / N)) / 2, Q15.
 * Target: in CW with the key delay running (cw.c, old-series 3.6) the CW key envelope;
 * otherwise s_tx_env_on, set by app.c TX_Main() (3.6): up after PA on, down before PA off,
 * so the PA never switches with drive on it (TUN, SSB, AM, digital). Gain 0 while receiving.
 */
#define CW_RAMP_SAMPLES (CW_RAMP_MS * 96u)   /* 96 kHz codec rate */
static int16_t  s_cw_ramp[CW_RAMP_SAMPLES + 1u];
static uint16_t s_cw_ramp_pos = 0;
static volatile uint8_t s_tx_env_on = 0;

void audio_cw_shape_init(void)
{
    uint16_t n;

    for (n = 0; n <= CW_RAMP_SAMPLES; n++) {
        float g = 0.5f * (1.0f - cosf(3.14159265f * (float)n / (float)CW_RAMP_SAMPLES));
        s_cw_ramp[n] = (int16_t)(g * 32767.0f + 0.5f);
    }
    s_cw_ramp_pos = 0;
}

/* Main loop (TX_Main): non-CW envelope target, 1 = ramp up / stay up */
void audio_tx_envelope(uint8_t on)
{
    s_tx_env_on = on;
}

/* I2S ISR: apply the TX envelope to one block of stereo 16-bit frames. Also called on
 * silent blocks (underrun, no stream) so the ramp keeps time. */
static void audio_cw_shape_block(uint8_t *dst)
{
    int16_t *w = (int16_t *)dst;
    const uint16_t frames = (uint16_t)(I2S_BUF_SIZE / 4u);
    uint8_t on;
    uint16_t i;
    int32_t g;

    if (cw_shaping_active()) {
        on = cw_envelope_on();
    } else if (E_host_mode == 'C') {
        s_cw_ramp_pos = 0;      /* CW idle or QSK CW: pass through (as before 3.6) */
        return;
    } else {
        on = s_tx_env_on;
    }
    for (i = 0; i < frames; i++) {
        if (on) {
            if (s_cw_ramp_pos < CW_RAMP_SAMPLES) {
                s_cw_ramp_pos++;
            }
        } else if (s_cw_ramp_pos > 0u) {
            s_cw_ramp_pos--;
        }
        g = s_cw_ramp[s_cw_ramp_pos];
        w[i * 2u]      = (int16_t)(((int32_t)w[i * 2u] * g) >> 15);
        w[i * 2u + 1u] = (int16_t)(((int32_t)w[i * 2u + 1u] * g) >> 15);
    }
}

uint8_t audio_tx_diag(void)
{
    uint32_t now = board_millis();

    if (!s_out_pkt_seen || (now - s_out_pkt_ms) > TX_DIAG_WINDOW_MS) {
        return 0u; /* no packets from USB */
    }
    if (!s_out_live_seen || (now - s_out_live_ms) > TX_DIAG_WINDOW_MS) {
        return 1u; /* packets, all zero */
    }
    return 2u;     /* live audio */
}

uint32_t audio_tx_underruns(void) { return s_tx_underruns; }
uint32_t audio_tx_dropped(void)   { return s_tx_dropped; }
uint32_t audio_rx_dropped(void)   { return s_rx_dropped; }
uint16_t audio_out_level_min(void) { return s_out_lvl_min; }
uint16_t audio_out_level_max(void) { return s_out_lvl_max; }

/* Main loop: start a new min / max window (the I2S interrupt updates them). */
void audio_out_level_reset(void)
{
    __disable_irq();
    s_out_lvl_min = 0xFFFFu;
    s_out_lvl_max = 0u;
    __enable_irq();
}

void Audio_Start(void)
{
    s_tx_primed = 0;
    E_Audio_running = 1;
    memset(TxI2S, 0, sizeof(TxI2S));
    memset(RxI2S, 0, sizeof(RxI2S));
}

void Audio_Stop(void)
{
    E_Audio_running = 0;
    s_tx_primed = 0;
    /* cw-carrier (3.90): no host = no stored carrier, so a key can't send RF on its own */
    audio_cw_carrier_clear();
    /* Codec + I2S keep running across USB disconnects (started once at boot
     * in app_init()); only the USB audio side stops here. */
}

void Audio_Main(void)
{
    /* Nothing to do: the I2S interrupt moves the audio (see top of file). */
}

void audio_usb_out_packet_seen(void)
{
    s_out_pkt_ms = board_millis();
    s_out_pkt_seen = 1;
}

/* I2S interrupt: a silent block (no stream, priming, underrun); the TX envelope still
 * steps, so a ramp down finishes on time and the next key starts from 0. */
static void tx_block_silent(uint8_t *dst)
{
    memset(dst, 0, I2S_BUF_SIZE);
    audio_cw_shape_block(dst);
}

static void tx_block_finish(uint8_t *dst);

/* cw-carrier (3.90): USB main loop writes the spare copy; the last piece makes it active. */
uint8_t audio_cw_carrier_piece(uint8_t n, const uint8_t *data, uint16_t len)
{
    if (n >= CW_CARRIER_PIECES || !data || len != CW_CARRIER_PIECE) {
        return 0u;
    }
    if (n == 0u) {
        s_car_got = 0u;
        s_car_fill = (s_car_active == 0u) ? 1u : 0u;
    } else if (s_car_fill == s_car_active) {
        return 0u;      /* no piece 0 since the last block: never write the playing copy */
    }
    memcpy((uint8_t *)s_car[s_car_fill] + (uint16_t)n * CW_CARRIER_PIECE, data,
           CW_CARRIER_PIECE);
    s_car_got |= (uint8_t)(1u << n);
    if (n == (uint8_t)(CW_CARRIER_PIECES - 1u) &&
        s_car_got == (uint8_t)((1u << CW_CARRIER_PIECES) - 1u)) {
        s_car_active = s_car_fill;      /* one byte write: the I2S interrupt sees old or new */
        s_car_got = 0u;
        return 1u;
    }
    return 0u;
}

void audio_cw_carrier_clear(void)
{
    s_car_active = CAR_NONE;
    s_car_got = 0u;
}

uint8_t audio_cw_carrier_status(void)
{
    return (uint8_t)(0x01u | ((s_car_active != CAR_NONE) ? 0x02u : 0x00u));
}

/* I2S interrupt: fill one DMA half-buffer (I2S_BUF_SIZE bytes) for the codec. */
void audio_i2s_tx_block(uint8_t *dst)
{
    uint16_t avail;
    uint16_t depth;
    uint16_t i;
    const uint8_t car = s_car_active;

    /* cw-carrier (3.90): CW with a stored block = play it; USB transmit audio is not used
     * (a host that still streams is read and thrown away, so its FIFO never fills). */
    if (E_host_mode == 'C' && car != CAR_NONE) {
        if (E_Audio_running && usb_audio_tx_enabled()) {
            usb_audio_out_clear();
        }
        s_tx_primed = 0;
        memcpy(dst, s_car[car], I2S_BUF_SIZE);
        s_out_live_ms = board_millis();
        s_out_live_seen = 1;
        tx_block_finish(dst);
        return;
    }

    if (!E_Audio_running || !usb_audio_tx_enabled()) {
        s_tx_primed = 0;
        tx_block_silent(dst);
        return;
    }

    avail = usb_audio_out_available();
    depth = usb_audio_out_depth();
    if (avail < s_out_lvl_min) {
        s_out_lvl_min = avail;
    }
    if (avail > s_out_lvl_max) {
        s_out_lvl_max = avail;
    }

    /* Not a whole number of stereo frames: the FIFO overflowed and cut a
     * packet short. Start again rather than play swapped/shifted channels. */
    if (avail & 3u) {
        usb_audio_out_clear();
        avail = 0;
        s_tx_primed = 0;
    }

    if (!s_tx_primed) {
        if (avail < (uint16_t)(depth / 2u)) {
            tx_block_silent(dst);
            return;
        }
        s_tx_primed = 1;
    }

    if (avail < I2S_BUF_SIZE) {
        s_tx_primed = 0;
        s_tx_underruns++;
        tx_block_silent(dst);
        return;
    }

    /* Host not following the feedback: drop one block while there is still
     * room for two more packets, so a packet is never cut short. */
    if (avail > (uint16_t)(depth - 2u * CFG_AUDIO_EP_OUT_SZ)) {
        (void)usb_audio_out_read(s_tx_discard, I2S_BUF_SIZE);
        s_tx_dropped++;
    }

    if (usb_audio_out_read(dst, I2S_BUF_SIZE) != I2S_BUF_SIZE) {
        s_tx_primed = 0;
        s_tx_underruns++;
        tx_block_silent(dst);
        return;
    }

    for (i = 0; i < I2S_BUF_SIZE; i++) {
        if (dst[i] != 0u) {
            s_out_live_ms = board_millis();
            s_out_live_seen = 1;
            break;
        }
    }

    tx_block_finish(dst);
}

/* I2S interrupt: shaping, IQ order and the one-sample delay for a block of carrier / audio
 * (the host's, or the stored CW carrier). Same steps and order as before 3.90. */
static void tx_block_finish(uint8_t *dst)
{
    uint16_t i;

    /* Raised-cosine TX envelope (CW keying; PA on / off in the other modes) */
    audio_cw_shape_block(dst);

    /*
     * IQ order: bit 0x02 clear (default) = straight order, bit set = channels
     * swapped. Unlike RX, the default must NOT swap: tested on air 2026-10-04,
     * swapped TX landed 24 kHz low (mirror image about the LO).
     */
    if (Audio_IQ_Channels & 0x02u) {
        uint16_t *w = (uint16_t *)dst;
        const uint16_t frames = (uint16_t)(I2S_BUF_SIZE / 4u);
        for (i = 0; i < frames; i++) {
            uint16_t t = w[i * 2u];
            w[i * 2u] = w[i * 2u + 1u];
            w[i * 2u + 1u] = t;
        }
    }

    /*
     * Delay the first word of each frame by one sample. The I2S TX stream
     * goes out one slot late (first word lands in the codec's right slot,
     * second word in the NEXT frame's left slot), so without this I and Q
     * reach the DAC one sample apart: 45 deg at the 12 kHz offset, TX image
     * only 8 dB down (Stew, spectrum analyzer, 2026-10-05).
     */
    {
        uint16_t *w = (uint16_t *)dst;
        const uint16_t frames = (uint16_t)(I2S_BUF_SIZE / 4u);
        for (i = 0; i < frames; i++) {
            uint16_t t = w[i * 2u];
            w[i * 2u] = s_tx_first_prev;
            s_tx_first_prev = t;
        }
    }
}

/* I2S interrupt: one DMA half-buffer (I2S_BUF_SIZE bytes) received from the
 * codec, on its way to the host. */
void audio_i2s_rx_block(const uint8_t *src)
{
    if (!E_Audio_running || !usb_audio_rx_enabled()) {
        return;
    }

    /*
     * PSoC CONTROL_DOUT fabric gate: clear bit = mute PCM3060 RX → host.
     * CW clears DOUT on key-down, sets it when hang ends (cw.c).
     */
    if ((Control_Read() & CONTROL_DOUT) == 0u) {
        memset(s_rx_block, 0, I2S_BUF_SIZE);
    } else if ((Audio_IQ_Channels & 0x01u) == 0u) {
        /* IQ order, as the PSoC: bit 0x01 clear (default) = channels swapped. */
        uint16_t i;
        const uint16_t *s = (const uint16_t *)src;
        uint16_t *d = (uint16_t *)s_rx_block;
        const uint16_t frames = (uint16_t)(I2S_BUF_SIZE / 4u);
        for (i = 0; i < frames; i++) {
            d[i * 2u] = s[i * 2u + 1u];
            d[i * 2u + 1u] = s[i * 2u];
        }
    } else {
        memcpy(s_rx_block, src, I2S_BUF_SIZE);
    }

    if (!usb_audio_in_write(s_rx_block, I2S_BUF_SIZE)) {
        s_rx_dropped++;
    }
}
