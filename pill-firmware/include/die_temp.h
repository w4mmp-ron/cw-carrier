#ifndef DIE_TEMP_H
#define DIE_TEMP_H

#include <stdint.h>

void    die_temp_init(void);
/** Take one raw ADC reading of the STM32 internal sensor; updates
 *  E_transceiver_temp (°C, integer). */
void    die_temp_update(uint32_t raw);

#endif
