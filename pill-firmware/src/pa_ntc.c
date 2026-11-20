/**
 * Proficio PA / board NTC on PA4 (ADC1_IN4).
 * USB: CMD_GET_POTENTIA_TEMPERATURE (0x06) — Solidus retired; opcode reused.
 *
 * Default circuit (tune to Stew's schematic):
 *   3.3V -- Rfixed(10k) -- ADC node -- NTC(10k@25C, B=3950) -- GND
 * NTC to ground; pull-up to 3V3. Any solid board GND is fine for NTC return.
 */
#include "pa_ntc.h"
#include "adc1_shared.h"
#include "board_pins.h"
#include "radio_state.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#ifndef PA_NTC_R_FIXED_OHMS
#define PA_NTC_R_FIXED_OHMS 10000.0f
#endif
#ifndef PA_NTC_R25_OHMS
#define PA_NTC_R25_OHMS     10000.0f
#endif
#ifndef PA_NTC_BETA
#define PA_NTC_BETA         3950.0f
#endif
#ifndef PA_NTC_VREF_MV
#define PA_NTC_VREF_MV      3300.0f
#endif

void pa_ntc_init(void)
{
    GPIO_InitTypeDef g = {0};

    adc1_shared_init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin = BOARD_PA_NTC_PIN;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOARD_PA_NTC_GPIO, &g);
    adc1_shared_prepare(BOARD_PA_NTC_ADC_CHANNEL);
}

void pa_ntc_update(uint32_t raw)
{
    float v_mv;
    float r_ntc;
    float t_k;
    float t_c;

    if (raw > 4095u) {
        return;
    }
    /* Open / short: report it (3.12), and avoid divide-by-zero at the extremes */
    if (raw > 4087u) {
        E_pa_temp = PA_NTC_OPEN;
        return;
    }
    if (raw < 8u) {
        E_pa_temp = PA_NTC_SHORT;
        return;
    }

    v_mv = (PA_NTC_VREF_MV * (float)raw) / 4095.0f;
    /* V = Vref * Rntc / (Rfixed + Rntc)  =>  Rntc = Rfixed * V / (Vref - V) */
    r_ntc = PA_NTC_R_FIXED_OHMS * v_mv / (PA_NTC_VREF_MV - v_mv);

    /* Beta: 1/T = 1/T25 + (1/B)*ln(R/R25) */
    t_k = 1.0f / ((1.0f / 298.15f) + (logf(r_ntc / PA_NTC_R25_OHMS) / PA_NTC_BETA));
    t_c = t_k - 273.15f;

    /* 3.15: out of range = broken circuit, not a temperature (was: clamped to -40 / 150).
     * An unplugged thermistor reads just under full scale (pull-up on the board), which
     * came out as -40 C instead of OPEN. A real heat sink is never below -40 or above 150. */
    if (t_c < -40.0f) {
        E_pa_temp = PA_NTC_OPEN;
        return;
    }
    if (t_c > 150.0f) {
        E_pa_temp = PA_NTC_SHORT;
        return;
    }

    E_pa_temp = (int32_t)(t_c + (t_c >= 0.0f ? 0.5f : -0.5f));
}
