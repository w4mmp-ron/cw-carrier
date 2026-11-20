/**
 * Main application loop — PSoC-style beater + USB audio path.
 */
#include "app.h"
#include "proficio_config.h"
#include "radio_state.h"
#include "control.h"
#include "board.h"
#include "usb_vendor.h"
#include "usb_device.h"
#include "cw.h"
#include "keyer_boot.h"
#include "si5351a.h"
#include "band.h"
#include "pcm3060.h"
#include "audio.h"
#include "i2s_audio.h"
#include "sync_sof.h"
#include "die_temp.h"
#include "pa_ntc.h"
#include "calibration.h"
#include "adc1_shared.h"
#include "board_pins.h"
#include "loop_prof.h"
#include "i2c_bus.h"
#include "rx_resync.h"

extern uint8_t g_status_beat;

static uint8_t s_beater = 0;
static uint8_t s_prev_host = 0;
static uint8_t s_cw_entry_started = 0;  /* entering CW: TX drain running (3.6) */
static uint32_t s_cw_entry_ms = 0;
static uint8_t s_prev_tx_hold = 0;
static uint32_t s_last_beat_ms = 0;
static uint8_t s_audio_started = 0;
#define PTT_POLL_PASSES 5u     /* beater case 1 passes between PTT reads (35 ms) */
static uint8_t s_ptt_poll_count = PTT_POLL_PASSES;
static uint8_t s_i2c_retries = 0;
static uint32_t s_i2c_retry_ms = 0;
static uint16_t s_i2c_retry_info = 0;    /* see app_i2c_retry_info() */

/*
 * PC13 diagnostic Morse-code letters (priority high -> low).
 *
 * Ownership: this player owns PC13 from boot; main.c no longer touches it.
 * The codec/I2S subsystem starts in app_init(), independent of USB, so the
 * letter shows with or without a USB host connected. On starting it holds
 * the LED dark for one word gap,
 * then sends the letter for the current condition, repeating it with a word
 * gap between repeats. The condition is re-sampled only at the end of each
 * word gap, so a letter is never cut off part-way — a change shows up on the
 * next repeat. Timing matches the bootloader's Morse-B beacon: 5 WPM,
 * unit = 240 ms.
 *
 *   C  -.-.   PCM3060 I2C failed
 *   U  ..-    PCM3060 register read-back: codec did not answer the read
 *   R  .-.    PCM3060 register read-back: a value differs from what was written
 *   L  .-..   SI5351 LO: last I2C write not acknowledged (no LO -> no signals)
 *   F  ..-.   I2S full-duplex DMA start failed
 *   P  .--.   PLLI2S clock not ready
 *   N  -.     SPI2 I2SE not enabling
 *   D  -..    No bit clock (polled TXE never asserted)
 *   S  ...    Bit clock OK, but no DMA half-buffer has completed (stalled)
 *   Z  --..   Streaming; RX still all zero
 *   K  -.-    Streaming; RX non-zero but frozen (every sample the same value:
 *             ADC not converting)
 *   G  --.    Streaming; RX values change, but not steadily (glitch/noise?)
 *   O  ---    Streaming; RX steadily live (>= 1/8 of samples in each buffer
 *             changing for ~1 s, re-checked every repeat) -- success
 *   solid ON  = normal (3.15): USB enumerated and O (RX) or T (TX) -- the
 *               letter is not sent. Not enumerated, O / T are sent as letters.
 *               Also solid: none of the above (SPI2 IRQ fired but no DMA
 *               completion; not expected in DMA mode, SPI2 error IRQs are off)
 *
 * While keyed by the host (RX line low, not CW) the RX letters above are
 * replaced by the TX audio condition:
 *   M  --     No audio packets arriving from USB
 *   I  ..     Packets arriving, all zero (host sending silence)
 *   T  -      Live audio from the host going out to the codec
 *
 * Bench build (pio run -e bench, PROFICIO_BENCH_NO_CODEC=1): for a bare
 * Black Pill with no codec. The codec I2C result is ignored, so "C" never
 * shows and the I2S/DMA letters below it do. The STM32 is I2S master and
 * makes all the clocks itself, so DMA runs without a codec: expect Z (RX pin
 * floating low) or O (pin picking up noise) -- either proves the full-duplex
 * DMA is completing. S would mean it isn't.
 */
#if defined(PROFICIO_BENCH_NO_CODEC) && PROFICIO_BENCH_NO_CODEC
#define DIAG_CODEC_FAILED(st)  (0)
#define DIAG_CODEC_STARTED(st) ((st) != 0u)
#define DIAG_READBACK_ST()     (0u)
#define DIAG_LO_FAILED()       (0)
#else
#define DIAG_CODEC_FAILED(st)  ((st) == 2u)
#define DIAG_CODEC_STARTED(st) ((st) == 1u)
#define DIAG_READBACK_ST()     PCM3060_readback_status()
#define DIAG_LO_FAILED()       (si5351_i2c_status() == 2u)
#endif
#define MORSE_UNIT_MS      (240u)

/*
 * Power-up I2C retry (3.4). At power-up the codec / Si5351 sometimes do not
 * answer (LED C), most likely another part (the PIC keyer) holding I2C1 while
 * it starts; NRST then clears it. If the codec or LO failed at start, try
 * again from the main loop at enumeration (the Pi has booted by then) or
 * I2C_RETRY_START_MS after power-up with no host, then every
 * I2C_RETRY_EVERY_MS, I2C_RETRIES times in all. A probe that finds the bus
 * still held costs ~1 ms; only a probe that ACKs runs the setup again.
 * After the last try the C (or L) stays: a real fault.
 */
#define I2C_RETRY_START_MS  3000u
#define I2C_RETRY_EVERY_MS  500u
#define I2C_RETRIES         10u
#define MORSE_DOT_MS       (MORSE_UNIT_MS)
#define MORSE_DASH_MS      (3u * MORSE_UNIT_MS)
#define MORSE_GAP_MS       (MORSE_UNIT_MS)
#define MORSE_WORD_GAP_MS  (7u * MORSE_UNIT_MS)
#define MORSE_MAX_ELEMENTS 4u

typedef enum {
    DIAG_I2C_FAIL = 0,
    DIAG_READ_FAIL,
    DIAG_READ_MISMATCH,
    DIAG_LO_FAIL,
    DIAG_FDX_FAIL,
    DIAG_PLL_NOT_READY,
    DIAG_I2S_NOT_ENABLED,
    DIAG_NO_CLOCK,
    DIAG_STALLED,
    DIAG_RX_ZERO,
    DIAG_RX_STUCK,
    DIAG_RX_SPORADIC,
    DIAG_RX_OK,
    DIAG_TX_NO_USB,
    DIAG_TX_SILENT,
    DIAG_TX_LIVE,
    DIAG_NONE,   /* fallback: started, nothing matched -- solid ON */
    DIAG_ALL_OK, /* USB enumerated and O or T -- solid ON (3.15) */
    DIAG_IDLE    /* audio not started -- leave PC13 alone */
} diag_led_state_t;

typedef struct {
    uint8_t elements[MORSE_MAX_ELEMENTS]; /* 0 = dot, 1 = dash */
    uint8_t count;
} morse_pattern_t;

static const morse_pattern_t s_diag_morse[DIAG_NONE] = {
    [DIAG_I2C_FAIL]        = { {1, 0, 1, 0}, 4 }, /* C */
    [DIAG_READ_FAIL]       = { {0, 0, 1},    3 }, /* U */
    [DIAG_READ_MISMATCH]   = { {0, 1, 0},    3 }, /* R */
    [DIAG_LO_FAIL]         = { {0, 1, 0, 0}, 4 }, /* L */
    [DIAG_FDX_FAIL]        = { {0, 0, 1, 0}, 4 }, /* F */
    [DIAG_PLL_NOT_READY]   = { {0, 1, 1, 0}, 4 }, /* P */
    [DIAG_I2S_NOT_ENABLED] = { {1, 0},       2 }, /* N */
    [DIAG_NO_CLOCK]        = { {1, 0, 0},    3 }, /* D */
    [DIAG_STALLED]         = { {0, 0, 0},    3 }, /* S */
    [DIAG_RX_ZERO]         = { {1, 1, 0, 0}, 4 }, /* Z */
    [DIAG_RX_STUCK]        = { {1, 0, 1},    3 }, /* K */
    [DIAG_RX_SPORADIC]     = { {1, 1, 0},    3 }, /* G */
    [DIAG_RX_OK]           = { {1, 1, 1},    3 }, /* O */
    [DIAG_TX_NO_USB]       = { {1, 1},       2 }, /* M */
    [DIAG_TX_SILENT]       = { {0, 0},       2 }, /* I */
    [DIAG_TX_LIVE]         = { {1},          1 }, /* T */
};

typedef enum {
    PH_IDLE = 0,   /* not driving PC13 */
    PH_WORD_GAP,   /* LED off; at the end, sample condition and start a letter */
    PH_ON,         /* LED on for the current dot/dash */
    PH_OFF,        /* LED off between elements of one letter */
    PH_SOLID       /* DIAG_NONE / DIAG_ALL_OK: LED held on */
} diag_phase_t;

typedef struct {
    diag_phase_t     phase;
    diag_led_state_t letter;  /* latched for the whole letter */
    uint8_t          element; /* index into s_diag_morse[letter].elements */
    uint32_t         phase_start;
} diag_led_player_t;

static diag_led_player_t s_diag_led = { PH_IDLE, DIAG_IDLE, 0, 0 };

/* Same priority chain as the old blink-rate ladder; it now just classifies
 * the condition into one enum value instead of toggling the LED inline. */
static diag_led_state_t compute_diag_led_state(void)
{
    uint8_t i2c_st = PCM3060_i2c_status();
    uint8_t fdx_st = i2s_audio_fdx_status();

    i2s_audio_dma_poll();

    if (DIAG_CODEC_FAILED(i2c_st)) {
        return DIAG_I2C_FAIL;
    }
    if (DIAG_READBACK_ST() == 3u) {
        return DIAG_READ_FAIL;
    }
    if (DIAG_READBACK_ST() == 2u) {
        return DIAG_READ_MISMATCH;
    }
    if (DIAG_LO_FAILED()) {
        return DIAG_LO_FAIL;
    }
    if (fdx_st == 2u) {
        return DIAG_FDX_FAIL;
    }
    if (DIAG_CODEC_STARTED(i2c_st) && i2s_audio_running()) {
        if (!i2s_audio_plli2s_ready()) {
            return DIAG_PLL_NOT_READY;
        }
        if (!i2s_audio_i2se_enabled()) {
            return DIAG_I2S_NOT_ENABLED;
        }
        if (!i2s_audio_txe_poll_ok()) {
            return DIAG_NO_CLOCK;
        }
        if (E_host_mode != 'C' && (Control_Read() & CONTROL_RX) == 0u) {
            switch (audio_tx_diag()) {
            case 0u:  return DIAG_TX_NO_USB;
            case 1u:  return DIAG_TX_SILENT;
            default:  return DIAG_TX_LIVE;
            }
        }
        if (i2s_audio_rx_steady()) {
            return DIAG_RX_OK;
        }
        if (i2s_audio_rx_saw_change()) {
            return DIAG_RX_SPORADIC;
        }
        if (i2s_audio_rx_saw_nonzero()) {
            return DIAG_RX_STUCK;
        }
        if (i2s_audio_cplt_count() > 0u) {
            return DIAG_RX_ZERO;
        }
        if (i2s_audio_spi2_irq_count() == 0u) {
            return DIAG_STALLED;
        }
        return DIAG_NONE;
    }
    return DIAG_IDLE;
}

static void diag_led_set(uint8_t on)
{
    Control_Write((uint8_t)(on ? (Control_Read() | CONTROL_LED)
                                : (Control_Read() & (uint8_t)~CONTROL_LED)));
}

static void diag_led_enter_word_gap(uint32_t now)
{
    diag_led_set(0);
    s_diag_led.phase = PH_WORD_GAP;
    s_diag_led.phase_start = now;
}

static void diag_led_begin_letter(diag_led_state_t st, uint32_t now)
{
    s_diag_led.letter = st;
    s_diag_led.element = 0;
    s_diag_led.phase_start = now;
    s_diag_led.phase = (st == DIAG_NONE || st == DIAG_ALL_OK) ? PH_SOLID : PH_ON;
    diag_led_set(1);
}

static void diag_led_poll(uint32_t now)
{
    diag_led_state_t st;
    uint32_t elapsed;

    st = compute_diag_led_state();
    if (st == DIAG_IDLE) {
        s_diag_led.phase = PH_IDLE; /* don't touch PC13 */
        return;
    }
    /* Keyed + silent (I) is normal with a host (SSB pause, mic muted, digital mode
     * not sending): solid too. I still shows with no host; M (no packets) always. */
    if ((st == DIAG_RX_OK || st == DIAG_TX_LIVE || st == DIAG_TX_SILENT) &&
        usb_device_configured()) {
        st = DIAG_ALL_OK; /* normal: solid on instead of O / T */
    }

    elapsed = now - s_diag_led.phase_start;

    switch (s_diag_led.phase) {
    case PH_IDLE:
        /* Just took over PC13: go dark for one word gap so the first letter
         * is clearly framed (also lets start-up transients settle). */
        diag_led_enter_word_gap(now);
        break;

    case PH_WORD_GAP:
        if (elapsed >= MORSE_WORD_GAP_MS) {
            diag_led_begin_letter(st, now); /* condition sampled here only */
        }
        break;

    case PH_ON: {
        const morse_pattern_t *pat = &s_diag_morse[s_diag_led.letter];
        uint32_t on_ms = pat->elements[s_diag_led.element] ? MORSE_DASH_MS
                                                            : MORSE_DOT_MS;
        if (elapsed >= on_ms) {
            if ((uint8_t)(s_diag_led.element + 1u) >= pat->count) {
                diag_led_enter_word_gap(now); /* letter finished */
            } else {
                diag_led_set(0);
                s_diag_led.phase = PH_OFF;
                s_diag_led.phase_start = now;
            }
        }
        break;
    }

    case PH_OFF:
        if (elapsed >= MORSE_GAP_MS) {
            s_diag_led.element++;
            s_diag_led.phase = PH_ON;
            s_diag_led.phase_start = now;
            diag_led_set(1);
        }
        break;

    case PH_SOLID:
        if (st != s_diag_led.letter) {
            diag_led_enter_word_gap(now);
        } else if ((Control_Read() & CONTROL_LED) == 0u) {
            diag_led_set(1); /* a host mode change clears CONTROL_LED (3.16) */
        }
        break;

    default:
        s_diag_led.phase = PH_IDLE;
        break;
    }
}

void app_init(void)
{
    loop_prof_init();   /* main loop timing (3.20) */
    radio_state_init();
    cal_init();     /* flash read (erase only when the log is 3/4 full) */
    (void)cal_ppm_get(&ee_ppm_int, &ee_ppm_dec);   /* Si5351 PPM (3.9), 0 if never set */

#if PROFICIO_FEAT_PCM3060 || PROFICIO_FEAT_USB_AUDIO
    /*
     * Codec + I2S run from boot, independent of USB -- like the PSoC, whose
     * codec system clock ran from power-up. PCM3060_Start() starts the I2S
     * clocks first, then resets and configures the codec with the clocks
     * running (~100 ms). Done before usb_device_init() so that settling time
     * can never stall USB enumeration.
     */
    (void)PCM3060_Init();
    PCM3060_Start();
#endif

#if PROFICIO_FEAT_USB_VENDOR || PROFICIO_FEAT_USB_AUDIO
    usb_device_init();
#endif
#if PROFICIO_FEAT_CW
    cw_init();
#if PROFICIO_FEAT_USB_AUDIO
    audio_cw_shape_init();
#endif
#endif
#if PROFICIO_FEAT_SI5351
    si5351a_init();
    (void)si5351aSetFrequency_wait(E_current_LO_freq);
#endif
#if PROFICIO_FEAT_BAND
    Band_Control_Write(CONTROL_BAND_20_30);
#endif
    die_temp_init();
    pa_ntc_init();

    s_prev_host = E_host_mode;
    s_prev_tx_hold = E_TX_Hold;
    s_last_beat_ms = board_millis();
}

/*
 * Host TX request -> RX / AMP lines (PSoC tx.c TX_Main()). One step per call.
 * Non-QSK amp (PSoC MKII 3.234): AMP low, wait E_key_delay_ms for the amp relays,
 * then RX low (PA on); on release RX high, wait AMP_RELEASE_MS, then AMP high.
 * QSK amp (E_QSK TRUE): original order, RX low then AMP low; AMP high then RX high.
 * TX envelope (3.6): after PA on (and AMP) the TX I/Q ramps up over CW_RAMP_MS
 * (raised cosine, audio.c); on release it ramps down first and the PA goes off only
 * after TX_RAMP_DONE_MS, so the PA never switches with drive on it (TUN, SSB, AM,
 * digital). A new request during the ramp down ramps back up.
 * Called every 7 ms (beater; every 1 ms while entering CW), so the waits can run up
 * to 7 ms long.
 * The PSoC also cleared CONTROL_LED on release; not done here (PC13 is the Morse player's).
 */
#define TX_RAMP_DONE_MS (CW_RAMP_MS + 1u)
static uint8_t s_tx_state = 0;

static void TX_Main(void)
{
    static uint32_t start_ms = 0;
    static uint8_t delay_ms = 0;

    switch (s_tx_state) {
    case 0: /* receiving */
        if (TX_Request && !TX_Inhibit) {
            if (E_QSK == TRUE) {
                s_tx_state = 2;
                Control_Write((uint8_t)(Control_Read() & ~CONTROL_RX));
            } else {
                Control_Write((uint8_t)(Control_Read() & ~CONTROL_AMP)); /* AMP on first */
                delay_ms = E_key_delay_ms;
                start_ms = board_millis();
                s_tx_state = 3;
            }
        }
        break;
    case 2:
        Control_Write((uint8_t)(Control_Read() & ~CONTROL_AMP));
        audio_tx_envelope(1);   /* ramp up */
        s_tx_state = 10;
        break;
    case 3: /* non-QSK: AMP on, waiting for the amp relays before PA on */
        if (!TX_Request || TX_Inhibit) {
            Control_Write((uint8_t)(Control_Read() | CONTROL_AMP)); /* PTT dropped before PA on */
            s_tx_state = 0;
        } else if ((board_millis() - start_ms) >= delay_ms) {
            Control_Write((uint8_t)(Control_Read() & ~CONTROL_RX)); /* PA on */
            audio_tx_envelope(1);   /* then ramp up */
            s_tx_state = 10;
        }
        break;
    case 10: /* transmitting */
        if (!TX_Request && !TX_Inhibit) {
            audio_tx_envelope(0);   /* ramp down before PA off */
            start_ms = board_millis();
            s_tx_state = 13;
        }
        break;
    case 13: /* ramping down */
        if (TX_Request && !TX_Inhibit) {
            audio_tx_envelope(1);   /* request again: back up */
            s_tx_state = 10;
        } else if ((board_millis() - start_ms) >= TX_RAMP_DONE_MS) {
            if (E_QSK == TRUE) {
                Control_Write((uint8_t)(Control_Read() | CONTROL_AMP));
                s_tx_state = 11;
            } else {
                Control_Write((uint8_t)(Control_Read() | CONTROL_RX)); /* PA off first */
                start_ms = board_millis();
                s_tx_state = 12;
            }
        }
        break;
    case 11:
        Control_Write((uint8_t)(Control_Read() | CONTROL_RX));
        s_tx_state = 0;
        break;
    case 12: /* non-QSK: PA off, waiting for RF to die before AMP off */
        if ((board_millis() - start_ms) >= AMP_RELEASE_MS) {
            Control_Write((uint8_t)(Control_Read() | CONTROL_AMP));
            s_tx_state = 0;
        }
        break;
    default:
        s_tx_state = 0;
        break;
    }
}

uint8_t app_tx_idle(void)
{
    return (uint8_t)(s_tx_state == 0u);
}

/* Hard stop: TX lines off, envelope down, TX_Main idle (power-up, leaving CW, drain timeout) */
static void TX_Reset(void)
{
    uint8_t c = Control_Read();

    c = (uint8_t)(c & ~CONTROL_LED);
    c = (uint8_t)(c | CONTROL_RX | CONTROL_AMP | CONTROL_DOUT);
    Control_Write(c);
    audio_tx_envelope(0);
    TX_Request = 0;
    s_tx_state = 0;
}

/*
 * Temperatures, one step per call and never waiting (called every 7 ms).
 * A conversion is started on one call and collected on the next; then idle,
 * so each sensor is read (and the PA maths done) about once a second.
 * 3.11: the PA NTC is the average of 16 conversions, one per call (~110 ms),
 * so a single glitched sample no longer moves the reading by 2-3 C.
 */
#define TEMPS_IDLE_CALLS 140u
#define PA_NTC_SAMPLES   16u

static void temps_poll(void)
{
    static uint8_t step = 0;
    static uint8_t idle = 0;
    static uint8_t n = 0;
    static uint32_t sum = 0;
    uint32_t raw;

    switch (step) {
    case 0:
        adc1_shared_start(ADC_CHANNEL_TEMPSENSOR);
        step = 1;
        break;
    case 1:
        if (adc1_shared_result(&raw)) {
            die_temp_update(raw);
        }
        adc1_shared_start(BOARD_PA_NTC_ADC_CHANNEL);
        n = 0;
        sum = 0;
        idle = 0;   /* guard: give up after 2 x PA_NTC_SAMPLES calls */
        step = 2;
        break;
    case 2:
        if (adc1_shared_result(&raw)) {
            sum += raw;
            n++;
        }
        if (n < PA_NTC_SAMPLES && ++idle < 2u * PA_NTC_SAMPLES) {
            adc1_shared_start(BOARD_PA_NTC_ADC_CHANNEL);
            break;
        }
        if (n > 0u) {
            pa_ntc_update((sum + n / 2u) / n);
        }
        idle = 0;
        step = 3;
        break;
    default:
        if (++idle >= TEMPS_IDLE_CALLS) {
            step = 0;
        }
        break;
    }
}

static void i2c_retry_poll(uint32_t now)
{
#if !(defined(PROFICIO_BENCH_NO_CODEC) && PROFICIO_BENCH_NO_CODEC)
    uint8_t codec_bad = (PCM3060_i2c_status() == 2u) ? 1u : 0u;
    uint8_t lo_bad = 0;

#if PROFICIO_FEAT_SI5351
    lo_bad = (si5351_i2c_status() == 2u) ? 1u : 0u;
#endif
    if ((!codec_bad && !lo_bad) || s_i2c_retries >= I2C_RETRIES) {
        return;
    }
    if (s_i2c_retries == 0u) {      /* what failed at start */
        s_i2c_retry_info |= (uint16_t)((codec_bad ? 0x100u : 0u) | (lo_bad ? 0x200u : 0u));
    }
    if (!usb_device_configured() && now < I2C_RETRY_START_MS) {
        return;
    }
    if (s_i2c_retries != 0u && (now - s_i2c_retry_ms) < I2C_RETRY_EVERY_MS) {
        return;
    }
    s_i2c_retry_ms = now;
    s_i2c_retries++;
    s_i2c_retry_info = (uint16_t)((s_i2c_retry_info & 0xFF00u) | s_i2c_retries);
    if (codec_bad && i2c_bus_probe(PCM3060_I2C_ADDR7)) {
        if (PCM3060_Retry()) {
            s_i2c_retry_info |= 0x400u;
        }
    }
#if PROFICIO_FEAT_SI5351
    if (lo_bad && i2c_bus_probe(SI5351_I2C_ADDR)) {
        si5351a_init();
        (void)si5351aSetFrequency_wait(E_current_LO_freq);
        if (si5351_i2c_status() != 2u) {
            s_i2c_retry_info |= 0x800u;
        }
    }
#endif
#else
    (void)now;
#endif
}

uint16_t app_i2c_retry_info(void)
{
    return s_i2c_retry_info;
}

void app_poll(void)
{
    uint32_t now = board_millis();
    uint8_t beat;

    loop_prof_begin();
    cw_hold_poll();
    cal_poll();     /* writes calibration values set by the host to flash */
    loop_prof_mark(PROF_CAL);

#if PROFICIO_FEAT_USB_VENDOR || PROFICIO_FEAT_USB_AUDIO
    usb_device_poll();
#endif
    loop_prof_mark(PROF_USB);

#if PROFICIO_FEAT_USB_AUDIO
    /* USB connect/disconnect only switches the USB audio streams on/off.
     * The codec and I2S keep running (started once in app_init()). */
    if (usb_device_configured() && !s_audio_started) {
        Audio_Start();
        sync_sof_start();
        s_audio_started = 1;
    }
    if (!usb_device_configured() && s_audio_started) {
        Audio_Stop();
        sync_sof_stop();
        s_audio_started = 0;
    }
    Audio_Main();
#endif
    loop_prof_mark(PROF_AUDIO);

    if ((now - s_last_beat_ms) < 1u) {
        loop_prof_end();
        return;
    }
    s_last_beat_ms = now;
    g_status_beat ^= 1u;

    diag_led_poll(now);
    loop_prof_mark(PROF_LED);

    beat = s_beater;
    switch (s_beater++) {
    case 0:
        i2c_retry_poll(now);    /* power-up I2C retry (3.4) */
        break;
    case 1:
        if (E_host_mode != 'C') {
            if (s_prev_host != E_host_mode) {
                /* Leaving CW, or first pass after power-up: hard stop. A change between
                 * two non-CW modes (3.6, as PSoC MKII 3.242) leaves TX alone: the client
                 * sends the mode before TUNE / PTT off, and a hard drop here would cut
                 * the carrier at full level instead of ramping it down. */
                if (s_prev_host == 'C' || s_prev_host == 0u) {
#if PROFICIO_FEAT_CW
                    cw_key_delay_stop();    /* give CONTROL_RX back before PA off */
#endif
                    TX_Reset();
                }
                s_prev_host = E_host_mode;
            }
            TX_Main();
        }   /* entering CW: handled below, before Manage_Paddles_Port() */
        /* PTT footswitch (PA6) -> E_PTT, read by the host with CMD_GET_PTT.
         * PSoC: every 10th pass of its 0.5 ms beater (~35 ms, debounces the
         * switch); here every 5th 7 ms pass = 35 ms. Was missing: E_PTT only
         * changed as a side effect of other requests, so PTT was not seen. */
        if (--s_ptt_poll_count == 0u) {
            (void)Status_Read();    /* updates E_PTT */
            s_ptt_poll_count = PTT_POLL_PASSES;
        }
        break;
    case 2:
        /* PPM from the host (frequency calibration), as the PSoC's Settings_Main /
         * Set_PPM (3.9): both bytes in, use them, retune (+100 Hz bump), keep in
         * flash. Before 3.9 they were received and never used (PPM stayed 0).
         * One byte each into an int32, so the low byte is the signed value. */
        if (E_PPM_needs_updated == (PPM_INT_RECEIVED | PPM_DEC_RECEIVED)) {
            E_PPM_needs_updated = 0;
            ee_ppm_int = (int8_t)E_calibration_int;
            ee_ppm_dec = (int8_t)E_calibration_dec;
            E_PPM_needs_set = E_PPM_NEEDS_SET_STEP_1;
            cal_ppm_set(ee_ppm_int, ee_ppm_dec);
        }
        if (s_prev_tx_hold != E_TX_Hold) {
            s_prev_tx_hold = E_TX_Hold;
            /* TX hold is in 10 ms units */
            E_key_delay_ms = (uint8_t)(((uint16_t)s_prev_tx_hold * 10u) / KEY_DELAY_DIVIDER);
            if (E_key_delay_ms < KEY_DELAY_MIN_MS) {
                E_key_delay_ms = KEY_DELAY_MIN_MS;
            }
            if (E_key_delay_ms > KEY_DELAY_MAX_MS) {
                E_key_delay_ms = KEY_DELAY_MAX_MS;
            }
        }
#if PROFICIO_FEAT_KEYER_I2C
        if (E_keyer_installed == TRUE) {
            (void)Configure_CW();
        }
#endif
        break;
    case 3:
#if PROFICIO_FEAT_KEYER_I2C
        keyer_boot_poll();      /* 3.16: CMD_KEYER_BOOT 0x9F, only busy during keyer-flash.py */
#endif
#if PROFICIO_FEAT_PCM3060
        rx_resync_poll(now);    /* 3.18: receive slip found -> restart codec + I2S */
#endif
        break;
    case 4:
#if PROFICIO_FEAT_BAND
        if (E_si5351_status == 0) {
            (void)Band_Main();
        }
#endif
        break;
    case 5:
        temps_poll();
        break;
    case 6:
#if PROFICIO_FEAT_SI5351
        if (E_host_mode != 'C') {
            E_si5351_status = si5351aSetFrequency(E_current_LO_freq);
        }
#endif
        s_beater = 0;
        break;
    default:
        s_beater = 0;
        break;
    }
    loop_prof_mark((uint8_t)(beat < 7u ? PROF_BEAT0 + beat : PROF_PARTS));

    if (E_host_mode == 'C') {
        if (s_prev_host != E_host_mode) {
            /* Entering CW (PSoC MKII 3.233): drop any host PTT / TUNE TX. TX_Main()
             * does not run in CW, so a PTT off sent after the mode change would never
             * be acted on. 3.6: if transmitting, ramp down and release through
             * TX_Main() (every 1 ms here) first; hard stop after MODE_DRAIN_MAX_MS.
             * Paddles wait until this is done. */
            if (!s_cw_entry_started) {
                s_cw_entry_started = 1;
                s_cw_entry_ms = now;
            }
            TX_Request = 0;
            if (s_tx_state != 0u && (now - s_cw_entry_ms) < MODE_DRAIN_MAX_MS) {
                TX_Main();
            } else {
                TX_Reset();
                s_cw_entry_started = 0;
                s_prev_host = E_host_mode;
            }
        } else {
#if PROFICIO_FEAT_CW
            Manage_Paddles_Port();
#endif
        }
    }
    loop_prof_mark(PROF_TAIL);
    loop_prof_end();
}
