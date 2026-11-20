#ifndef APP_H
#define APP_H

#include <stdint.h>

void app_init(void);
void app_poll(void);
/* Power-up I2C retry record (3.5), CMD_AUDIO_STATS word 13 bits 16-27:
 * bits 0-7 tries run, 8 codec failed at start, 9 LO failed at start,
 * 10 codec answered on a retry, 11 LO answered on a retry. */
uint16_t app_i2c_retry_info(void);
/* 1 when TX_Main is idle (PA and AMP off, ramp done); 0 while keying or releasing (3.7) */
uint8_t app_tx_idle(void);

#endif /* APP_H */
