/**
 * Shared ADC1 instance for internal die sensor and external PA NTC.
 */
#include "adc1_shared.h"

static ADC_HandleTypeDef hadc1;
static uint8_t s_ready;

void adc1_shared_init(void)
{
    if (s_ready) {
        return;
    }

    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        return;
    }
    s_ready = 1;
}

/*
 * The main loop must never wait for the ADC: a stall there delays TinyUSB's
 * re-arming of the isochronous IN endpoint and costs received audio (found
 * 2026-10-05). So: channels are set up once at start-up, the ADC stays on,
 * and a conversion is started on one pass and collected on a later one.
 */
void adc1_shared_prepare(uint32_t channel)
{
    ADC_ChannelConfTypeDef c = {0};

    if (!s_ready) {
        return;
    }
    c.Channel = channel;
    c.Rank = 1;
    c.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    (void)HAL_ADC_ConfigChannel(&hadc1, &c);
    __HAL_ADC_ENABLE(&hadc1);
}

void adc1_shared_start(uint32_t channel)
{
    if (!s_ready) {
        return;
    }
    /* One conversion, rank 1 = this channel (low 16 bits hold the number). */
    ADC1->SQR3 = (uint32_t)(uint16_t)channel;
    (void)ADC1->DR; /* clears a stale end-of-conversion flag */
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint8_t adc1_shared_result(uint32_t *raw)
{
    if (!s_ready || (ADC1->SR & ADC_SR_EOC) == 0u) {
        return 0;
    }
    *raw = ADC1->DR & 0x0FFFu;
    return 1;
}
