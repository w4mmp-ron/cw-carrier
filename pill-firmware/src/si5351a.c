/**
 * SI5351A LO — port of soft/hard Multisynth strategy from PSoC si5351a.c.
 * Uses blocking I2C (setupPLL/MS complete in one poll when bus is free).
 */
#include "si5351a.h"
#include "i2c_bus.h"
#include "radio_state.h"
#include "app.h"
#include "proficio_config.h"

#define SI5351_VCO_MIN     600000000UL
#define SI5351_VCO_MAX     900000000UL
#define SI5351_VCO_CENTER  750000000UL
#define SI5351_MS_MIN      6UL
#define SI5351_MS_MAX      1800UL
#define SI5351_HARD_BLANK_HOLD  6u
#define SI5351_CLK0_OFF         0x80u
#define SI5351_CLK0_ON          (0x4Fu | SI_CLK_SRC_PLL_A)

uint32_t E_l_freq = 0;
uint32_t E_l_freq_temp = 0;

/* See si5351_i2c_status() */
static uint8_t s_i2c_status = 0;

uint8_t si5351_write(uint8_t reg, uint8_t val)
{
    uint8_t ok = i2c_write_reg(SI5351_I2C_ADDR, reg, val);
    s_i2c_status = ok ? 1u : 2u;
    return ok;
}

uint8_t si5351_i2c_status(void)
{
    return s_i2c_status;
}

/* 3.22: 8 registers in one I2C burst (the Si5351 auto-increments the register
 * address). One transaction is ~1 ms at 100 kHz; 8 single writes took ~3 ms of
 * main loop, long enough to miss USB audio packets at every band change. It
 * also updates the PLL / Multisynth in one go instead of register by register. */
static void si5351_write8(uint8_t reg, const uint8_t *v)
{
    uint8_t buf[9];
    uint8_t i;
    uint8_t ok;

    buf[0] = reg;
    for (i = 0; i < 8u; i++) {
        buf[1u + i] = v[i];
    }
    ok = i2c_write_buf(SI5351_I2C_ADDR, buf, sizeof(buf));
    s_i2c_status = ok ? 1u : 2u;
}

static void setupPLL_blocking(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
    uint32_t P1, P2, P3;
    P1 = (uint32_t)(128.0f * ((float)num / (float)denom));
    P1 = (uint32_t)(128u * (uint32_t)mult + P1 - 512u);
    P2 = (uint32_t)(128.0f * ((float)num / (float)denom));
    P2 = (uint32_t)(128u * num - denom * P2);
    P3 = denom;
    {
        const uint8_t v[8] = {
            (uint8_t)((P3 & 0x0000FF00u) >> 8),
            (uint8_t)(P3 & 0xFFu),
            (uint8_t)((P1 & 0x00030000u) >> 16),
            (uint8_t)((P1 & 0x0000FF00u) >> 8),
            (uint8_t)(P1 & 0xFFu),
            (uint8_t)(((P3 & 0x000F0000u) >> 12) | ((P2 & 0x000F0000u) >> 16)),
            (uint8_t)((P2 & 0x0000FF00u) >> 8),
            (uint8_t)(P2 & 0xFFu)
        };
        si5351_write8(pll, v);
    }
}

static void setupMultisynth_blocking(uint8_t synth, uint32_t divider, uint8_t rDiv)
{
    uint32_t P1 = 128u * divider - 512u;
    uint32_t P2 = 0;
    uint32_t P3 = 1;
    const uint8_t v[8] = {
        (uint8_t)((P3 & 0x0000FF00u) >> 8),
        (uint8_t)(P3 & 0xFFu),
        (uint8_t)(((P1 & 0x00030000u) >> 16) | rDiv),
        (uint8_t)((P1 & 0x0000FF00u) >> 8),
        (uint8_t)(P1 & 0xFFu),
        (uint8_t)(((P3 & 0x000F0000u) >> 12) | ((P2 & 0x000F0000u) >> 16)),
        (uint8_t)((P2 & 0x0000FF00u) >> 8),
        (uint8_t)(P2 & 0xFFu)
    };
    si5351_write8(synth, v);
}

void si5351aOutputOff(uint8_t clk)
{
    (void)si5351_write(clk, 0x80);
}

#define SI_OUTPUT_ENABLE_CTRL 3u   /* bit set = output disabled */

void si5351a_init(void)
{
    /* Crystal load 10 pF, outputs off until first tune */
    (void)si5351_write(183, 0xD2); /* CL=10pF typical */
    si5351aOutputOff(SI_CLK0_CONTROL);
    /*
     * 3.8: CLK1 / CLK2 powered down and their outputs disabled, as the PSoC's
     * si5351_init() does. Before, they were left as the chip powered up; an
     * output running next to CLK0 leaks into it and puts spurs on the LO
     * (comb around the LO, worst on some frequencies, e.g. WWV 15.000 MHz).
     * CLK0 stays enabled (bit 0 clear); it is switched on by the first tune.
     */
    si5351aOutputOff(SI_CLK1_CONTROL);
    si5351aOutputOff(SI_CLK2_CONTROL);
    (void)si5351_write(SI_OUTPUT_ENABLE_CTRL, 0xFEu);
}

uint8_t si5351aSetFrequency(uint32_t LO_freq)
{
    static uint32_t pllFreq;
    uint32_t xtalFreq = SI5351_XTAL_FREQ;
    static uint32_t l;
    static uint8_t mult;
    static uint32_t num, denom, divider;
    static uint32_t freq_previous = 0;
    static uint32_t prev_ms_divider = 0;
    static uint8_t soft_ms_hold = 0;
    static uint8_t clk0_blanked = 0;
    static uint8_t hard_blank_hold = 0;
    float delta_freq, ppm, f;
    int32_t delta_freq_int;
    static uint8_t state = 0;
    static int8_t l_ppm_int;
    static int8_t l_ppm_dec;
    uint8_t try_soft;

    switch (state) {
    case 0:
        switch (E_PPM_needs_set) {
        case 0:
            E_l_freq_temp = LO_freq;
            break;
        case E_PPM_NEEDS_SET_STEP_1:
            E_l_freq_temp = LO_freq + 100u;
            E_PPM_needs_set = E_PPM_NEEDS_SET_STEP_2;
            break;
        case E_PPM_NEEDS_SET_STEP_2:
            E_l_freq_temp = LO_freq;
            E_PPM_needs_set = 0;
            break;
        default:
            E_l_freq_temp = LO_freq;
            break;
        }
        /* RX frequency (with RIT) only once TX_Main is idle (3.7): since 3.6 the PA
         * stays on through the ramp down after TX_Request drops, and moving the LO
         * by the RIT then would shift the end of the transmission. */
        if (!TX_Request && app_tx_idle()) {
            if (freq_previous != (E_l_freq_temp + E_current_rit_freq)) {
                E_l_freq = E_l_freq_temp + E_current_rit_freq;
                freq_previous = E_l_freq;
                state = 1;
            }
        } else {
            if (freq_previous != E_l_freq_temp) {
                E_l_freq = E_l_freq_temp;
                freq_previous = E_l_freq_temp;
                state = 1;
            }
        }
        break;

    case 1:
        l_ppm_int = ee_ppm_int;
        l_ppm_dec = ee_ppm_dec;
        ppm = (float)l_ppm_dec / 100.0f;
        ppm = ppm + (float)l_ppm_int;
        delta_freq = (float)E_l_freq / 1000000.0f;
        delta_freq = ((delta_freq * ppm) * 4.0f) * -1.0f;
        delta_freq_int = (int32_t)delta_freq;
        if (delta_freq_int < 0) {
            while ((delta_freq_int % 4) != 0) {
                delta_freq_int++;
            }
        } else if (delta_freq_int > 0) {
            while ((delta_freq_int % 4) != 0) {
                delta_freq_int--;
            }
        }
        E_l_freq = E_l_freq * 4u;
        E_l_freq = (uint32_t)((int32_t)E_l_freq + delta_freq_int);

        soft_ms_hold = 0;
        try_soft = 0;
        if (prev_ms_divider >= SI5351_MS_MIN && E_l_freq > 0UL) {
            if (E_l_freq <= (SI5351_VCO_MAX / prev_ms_divider)) {
                pllFreq = prev_ms_divider * E_l_freq;
                if (pllFreq >= SI5351_VCO_MIN) {
                    mult = (uint8_t)(pllFreq / xtalFreq);
                    if (mult >= 15u && mult <= 90u) {
                        try_soft = 1;
                    }
                }
            }
        }

        if (try_soft) {
            divider = prev_ms_divider;
            soft_ms_hold = 1;
            E_smooth = TRUE;
        } else {
            divider = SI5351_VCO_CENTER / E_l_freq;
            if (divider < SI5351_MS_MIN) {
                divider = SI5351_MS_MIN;
            }
            if (divider > SI5351_MS_MAX) {
                divider = SI5351_MS_MAX;
            }
            if (divider % 2UL) {
                divider--;
            }
            if (divider < SI5351_MS_MIN) {
                divider = SI5351_MS_MIN;
            }
            pllFreq = divider * E_l_freq;
            if (pllFreq > SI5351_VCO_MAX || pllFreq < SI5351_VCO_MIN) {
                divider = SI5351_VCO_MAX / E_l_freq;
                if (divider % 2UL) {
                    divider--;
                }
                if (divider < SI5351_MS_MIN) {
                    divider = SI5351_MS_MIN;
                }
                pllFreq = divider * E_l_freq;
            }
            soft_ms_hold = 0;
            E_smooth = FALSE;
        }

        mult = (uint8_t)(pllFreq / xtalFreq);
        l = pllFreq % xtalFreq;
        f = (float)l;
        f *= 1048575.0f;
        f /= (float)xtalFreq;
        num = (uint32_t)f;
        denom = 1048575u;
        hard_blank_hold = 0;
        state = 2;
        break;

    case 2:
        if (!soft_ms_hold && !clk0_blanked) {
            (void)si5351_write(SI_CLK0_CONTROL, SI5351_CLK0_OFF);
            clk0_blanked = 1;
        }
        setupPLL_blocking(SI_SYNTH_PLL_A, mult, num, denom);
        state = 3;
        break;

    case 3:
        if (!soft_ms_hold) {
            setupMultisynth_blocking(SI_SYNTH_MS_0, divider, SI_R_DIV_1);
        }
        state = 4;
        break;

    case 4:
        if (!E_smooth) {
            (void)si5351_write(SI_PLL_RESET, 0xA0);
            E_smooth = TRUE;
            hard_blank_hold = SI5351_HARD_BLANK_HOLD;
        } else {
            hard_blank_hold = 0;
        }
        prev_ms_divider = divider;
        state = 5;
        break;

    case 5:
        if (hard_blank_hold != 0u) {
            hard_blank_hold--;
            break;
        }
        (void)si5351_write(SI_CLK0_CONTROL, SI5351_CLK0_ON);
        clk0_blanked = 0;
        state = 0;
        break;

    default:
        state = 0;
        break;
    }
    return state;
}

uint8_t si5351aSetFrequency_wait(uint32_t LO_freq)
{
    uint16_t guard = 64;
    uint8_t st;
    do {
        st = si5351aSetFrequency(LO_freq);
    } while (st != 0 && --guard);
    return (st == 0) ? 0 : 1;
}
