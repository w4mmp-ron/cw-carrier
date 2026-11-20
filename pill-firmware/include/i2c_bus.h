#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void    i2c_bus_init(void);
uint8_t i2c_bus_probe(uint8_t addr7);
/** Write one byte to 7-bit addr. Returns 1 on ACK success. */
uint8_t i2c_write_byte(uint8_t addr7, uint8_t data);
/** Write reg+val to device (SI5351 style). Returns 1 on success. */
uint8_t i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val);
/** Write buffer. Returns 1 on success. */
uint8_t i2c_write_buf(uint8_t addr7, const uint8_t *buf, uint16_t len);
/** Read one register (reg write, repeated start, 1-byte read). Returns 1 on success. */
uint8_t i2c_read_reg(uint8_t addr7, uint8_t reg, uint8_t *val);
/** Plain read of len bytes (3.16, keyer bootloader). Returns 1 on success. */
uint8_t i2c_read_buf(uint8_t addr7, uint8_t *buf, uint16_t len);
/** One address probe, no peripheral reset (3.16). Returns 1 on ACK. */
uint8_t i2c_ready(uint8_t addr7);

#endif /* I2C_BUS_H */
