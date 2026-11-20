/**
 * Main loop timing (3.20, evidence only), DWT cycle counter. See loop_prof.h.
 * Times include interrupts that ran during the part (USB, I2S).
 */
#include "loop_prof.h"
#include "board.h"
#include "stm32f4xx_hal.h"

static uint32_t s_pass_start;
static uint32_t s_mark;
static uint32_t s_part_max[PROF_PARTS];   /* cycles */
static uint32_t s_pass_max;               /* cycles */
static uint8_t  s_pass_max_part;          /* longest part in the longest pass */
static uint32_t s_pass_max_ms;            /* when it happened */
static uint8_t  s_cur_part;
static uint32_t s_cur_part_cyc;

void loop_prof_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void loop_prof_begin(void)
{
    s_pass_start = DWT->CYCCNT;
    s_mark = s_pass_start;
    s_cur_part_cyc = 0;
    s_cur_part = 0;
}

void loop_prof_mark(uint8_t part)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t d = now - s_mark;

    s_mark = now;
    if (part >= PROF_PARTS) {
        return;
    }
    if (d > s_part_max[part]) {
        s_part_max[part] = d;
    }
    if (d > s_cur_part_cyc) {
        s_cur_part_cyc = d;
        s_cur_part = part;
    }
}

void loop_prof_end(void)
{
    uint32_t d = DWT->CYCCNT - s_pass_start;

    if (d > s_pass_max) {
        s_pass_max = d;
        s_pass_max_part = s_cur_part;
        s_pass_max_ms = board_millis();
    }
}

static uint32_t cyc_to_us(uint32_t c)
{
    uint32_t per_us = SystemCoreClock / 1000000u;

    return per_us ? c / per_us : c;
}

uint16_t loop_prof_stats(uint8_t *buf, uint16_t cap, uint8_t clear)
{
    uint32_t w[LOOP_PROF_STATS_WORDS];
    uint16_t i;

    if (cap < sizeof(w)) {
        return 0;
    }
    w[0] = 1u | (LOOP_PROF_STATS_WORDS << 16);  /* layout version | word count */
    w[1] = board_millis();
    w[2] = cyc_to_us(s_pass_max);               /* longest app_poll pass, us */
    w[3] = (uint32_t)s_pass_max_part | (s_pass_max_ms << 8); /* its part | ms (24 bit) */
    for (i = 0; i < PROF_PARTS; i++) {
        w[4u + i] = cyc_to_us(s_part_max[i]);   /* longest of each part, us */
    }
    for (i = 0; i < LOOP_PROF_STATS_WORDS; i++) {
        buf[i * 4u]      = (uint8_t)w[i];
        buf[i * 4u + 1u] = (uint8_t)(w[i] >> 8);
        buf[i * 4u + 2u] = (uint8_t)(w[i] >> 16);
        buf[i * 4u + 3u] = (uint8_t)(w[i] >> 24);
    }
    if (clear) {
        for (i = 0; i < PROF_PARTS; i++) {
            s_part_max[i] = 0;
        }
        s_pass_max = 0;
    }
    return (uint16_t)sizeof(w);
}
