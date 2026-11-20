/**
 * Receive slip recovery (3.18).
 *
 * Found 2026-11-17 on the Pi (and 2026-11-16 on Windows), each time after a
 * TUNE into a high SWR: the spectrum went all yellow, S-meter S9+40, no audio,
 * until NRST. A raw capture showed every I and Q sample shifted right one bit
 * (never negative): the codec -> STM32 receive data was one bit clock out of
 * step, most likely from RF on the bit clock line. USB, the LO and transmit
 * were fine.
 *
 * i2s_audio.c flags a slip (~100 ms of live receive with no negative sample).
 * This file then does what NRST did for the receive path, one step per call:
 * restart the I2S DMA, reset the codec and write its registers again, with
 * the same waits as power-up. Receive is lost for ~100 ms. It only starts
 * while the radio is receiving (PA off, TX_Main idle, no CW over running), so
 * a transmission is never cut; a slip found while transmitting is fixed at
 * key-up. At most one restart every 2 s.
 *
 * 3.20: a second kind (2026-11-17 22:42, Pi, normal FT8 on 20 m): the receive
 * data ~8-10 bit clocks out of step, half the samples negative, so the check
 * above never fired; the I2S2ext frame error flag (FRE) was set on every poll
 * (~140 a second) and is 0 in normal use, transmit included (40 min of FT8
 * checked). So FRE seen on RESYNC_FRE_POLLS polls in a row is a slip too,
 * with the same restart and the same guards. Both kinds only ever followed a
 * transmission (RF from the transmitter, most likely).
 * The overrun flag (OVR) is only counted, for the stats.
 */
#include "rx_resync.h"
#include "i2s_audio.h"
#include "pcm3060.h"
#include "proficio_config.h"
#include "radio_state.h"
#include "control.h"
#include "cw.h"
#include "app.h"
#include "board.h"

#define RESYNC_MIN_GAP_MS 2000u
/* 3.19: the check is armed only after receive has looked steady (live ADC
 * data, i2s_audio_rx_steady) for this long after power-up; stays armed. */
#define RESYNC_ARM_MS     5000u
/* 3.20: FRE on this many 7 ms polls in a row (~0.1 s) = slipped. */
#define RESYNC_FRE_POLLS  15u

enum {
    RS_IDLE = 0,
    RS_CLOCKS,      /* I2S restarted, codec locking onto the clocks */
    RS_RESET_LOW,   /* codec reset pin low */
    RS_RESET_HIGH,  /* reset released, settling */
    RS_PSV,         /* registers written, ADC/DAC leaving power-save */
};

static uint8_t  s_state = RS_IDLE;
static uint32_t s_step_ms = 0;
static uint8_t  s_fail = 0;
static uint32_t s_count = 0;        /* restarts done */
static uint32_t s_last_ms = 0;      /* when the last one started (0 = never) */
static uint32_t s_fre = 0;          /* I2S2ext frame errors seen */
static uint32_t s_ovr = 0;          /* I2S2ext overruns seen (rising edges) */
static uint8_t  s_ovr_prev = 0;
static uint8_t  s_waiting = 0;      /* 1 = slip seen, waiting for receive */
static uint8_t  s_armed = 0;        /* 1 = receive was steady RESYNC_ARM_MS after power-up */
static uint32_t s_steady_ms = 0;    /* when receive turned steady (0 = not steady) */
static uint16_t s_fre_run = 0;      /* 3.20: polls in a row with FRE set */
static uint16_t s_fre_restarts = 0; /* 3.20: restarts started by the FRE check */
static uint8_t  s_misframe_req = 0; /* 3.21 test: 1 = false pulses on bit clock, 2 = on word select */

static uint8_t receiving(void)
{
    if (!app_tx_idle()) {
        return 0;
    }
    if ((Control_Read() & CONTROL_RX) == 0u) {  /* PA on */
        return 0;
    }
#if PROFICIO_FEAT_CW
    if (E_host_mode == 'C' && cw_shaping_active()) {    /* CW over running */
        return 0;
    }
#endif
    return 1;
}

void rx_resync_poll(uint32_t now)
{
#if defined(PROFICIO_BENCH_NO_CODEC) && PROFICIO_BENCH_NO_CODEC
    (void)now;
#else
    uint8_t f;

    if (!i2s_audio_running() && s_state == RS_IDLE) {
        return;
    }
    f = i2s_audio_ext_flags();
    if (f & I2S_EXT_FRE) {
        s_fre++;
        if (s_fre_run < 0xFFFFu) {
            s_fre_run++;
        }
    } else {
        s_fre_run = 0;
    }
    if ((f & I2S_EXT_OVR) && !s_ovr_prev) {
        s_ovr++;
    }
    s_ovr_prev = (uint8_t)((f & I2S_EXT_OVR) ? 1u : 0u);

    if (!s_armed) {
        if (!i2s_audio_rx_steady()) {
            s_steady_ms = 0;
        } else if (s_steady_ms == 0u) {
            s_steady_ms = now ? now : 1u;
        } else if ((now - s_steady_ms) >= RESYNC_ARM_MS) {
            s_armed = 1;
        }
    }

    switch (s_state) {
    case RS_IDLE:
        if (s_misframe_req != 0u && receiving()) {
            /* Test: false clock pulses, as RF would make (FRE may follow). */
            i2s_audio_test_glitch((uint8_t)(s_misframe_req == 2u));
            s_misframe_req = 0;
            s_fre_run = 0;
            break;
        }
        if (!s_armed || PCM3060_i2c_status() != 1u ||
            (!i2s_audio_rx_slipped() && s_fre_run < RESYNC_FRE_POLLS)) {
            s_waiting = 0;
            break;
        }
        s_waiting = 1;
        if (!receiving()) {
            break;
        }
        if (s_last_ms != 0u && (now - s_last_ms) < RESYNC_MIN_GAP_MS) {
            break;
        }
        s_last_ms = now ? now : 1u;
        s_count++;
        if (s_fre_run >= RESYNC_FRE_POLLS && s_fre_restarts < 0xFFFFu) {
            s_fre_restarts++;
        }
        s_fre_run = 0;
        s_waiting = 0;
        i2s_audio_restart();
        s_step_ms = now;
        s_state = RS_CLOCKS;
        break;
    case RS_CLOCKS:
        if ((now - s_step_ms) >= PCM3060_CLOCK_LOCK_MS) {
            PCM3060_ResetPin(0);
            s_step_ms = now;
            s_state = RS_RESET_LOW;
        }
        break;
    case RS_RESET_LOW:
        if ((now - s_step_ms) >= PCM3060_RESET_LOW_MS) {
            PCM3060_ResetPin(1);
            s_step_ms = now;
            s_state = RS_RESET_HIGH;
        }
        break;
    case RS_RESET_HIGH:
        if ((now - s_step_ms) >= PCM3060_RESET_SETTLE_MS) {
            s_fail = PCM3060_WriteSetup();  /* 5 short I2C writes, as at power-up */
            s_step_ms = now;
            s_state = RS_PSV;
        }
        break;
    case RS_PSV:
        if ((now - s_step_ms) >= PCM3060_PSV_SETTLE_MS) {
            PCM3060_FinishSetup(s_fail);
            (void)i2s_audio_ext_flags();    /* flags from the restart don't count */
            s_fre_run = 0;
            s_state = RS_IDLE;
        }
        break;
    default:
        s_state = RS_IDLE;
        break;
    }
#endif
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

uint16_t rx_resync_stats(uint8_t *buf, uint16_t cap, uint16_t wValue)
{
    uint32_t w[RX_RESYNC_STATS_WORDS];
    uint16_t i;

    if (cap < sizeof(w)) {
        return 0;
    }
    if (wValue == 2u) {
        i2s_audio_rx_sim_slip();
    } else if (wValue == 3u || wValue == 4u) {
        s_misframe_req = (uint8_t)(wValue - 2u);   /* 1 = bit clock, 2 = word select */
    }
    w[0] = 1u | (RX_RESYNC_STATS_WORDS << 16);   /* layout version | word count */
    w[1] = board_millis();
    w[2] = s_count;
    w[3] = s_last_ms;
    w[4] = s_fre;
    w[5] = s_ovr;
    w[6] = (uint32_t)PCM3060_i2c_status() | ((uint32_t)PCM3060_readback_status() << 8);
    w[7] = (uint32_t)s_state | ((uint32_t)s_waiting << 8) |
           ((uint32_t)i2s_audio_rx_slipped() << 9) | ((uint32_t)s_armed << 10) |
           ((uint32_t)(s_fre_run >= RESYNC_FRE_POLLS) << 11) |
           ((uint32_t)(s_misframe_req != 0u) << 12) | ((uint32_t)s_fre_restarts << 16);
    for (i = 0; i < RX_RESYNC_STATS_WORDS; i++) {
        put_u32(&buf[i * 4u], w[i]);
    }
    return (uint16_t)sizeof(w);
}
