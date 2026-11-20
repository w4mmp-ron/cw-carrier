#ifndef PA_NTC_H
#define PA_NTC_H

#include <stdint.h>

/* 3.12: sent on 0x06 in place of a temperature when the thermistor circuit is
 * broken (was: the last good value kept, 0 C if broken from power-up). */
#define PA_NTC_OPEN   (-99)   /* PA4 at full scale: thermistor open / unplugged */
#define PA_NTC_SHORT  999     /* PA4 at 0 V: thermistor shorted */

void pa_ntc_init(void);
/** Take one raw ADC reading of the PA/board NTC on PA4; updates E_pa_temp
 *  (°C, integer). */
void pa_ntc_update(uint32_t raw);

#endif /* PA_NTC_H */
