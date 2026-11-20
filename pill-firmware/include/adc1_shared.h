/**
 * Shared ADC1 for die temp sensor + PA NTC (PA4 / ADC1_IN4).
 */
#ifndef ADC1_SHARED_H
#define ADC1_SHARED_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

void     adc1_shared_init(void);
/** Set up one channel (sampling time). Call once at start-up; may wait a few us. */
void     adc1_shared_prepare(uint32_t channel);
/** Start one conversion on a prepared channel. Never waits. */
void     adc1_shared_start(uint32_t channel);
/** 1 + 12-bit raw value if the conversion has finished, else 0. Never waits. */
uint8_t  adc1_shared_result(uint32_t *raw);

#endif /* ADC1_SHARED_H */
