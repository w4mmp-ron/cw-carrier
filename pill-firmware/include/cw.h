#ifndef CW_H
#define CW_H

#include <stdint.h>

uint8_t keyer_write(uint8_t byte);
uint8_t Configure_CW(void);
void    Manage_Paddles_Port(void);
void    cw_init(void);
void    cw_on_hold_expired(void);  /* TIM / ISR → clear E_cw_hold */
void    cw_key_delay_tick(void);   /* SysTick ISR, every 1 ms */
void    cw_key_delay_stop(void);   /* main loop, on leaving CW */
uint8_t cw_shaping_active(void);   /* I2S ISR: 1 = shape TX I/Q with the CW envelope */
uint8_t cw_envelope_on(void);      /* I2S ISR: envelope target (1 = ramp up / stay up) */

#endif /* CW_H */
