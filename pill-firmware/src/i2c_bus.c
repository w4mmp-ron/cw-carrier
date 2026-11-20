#include "i2c_bus.h"
#include "board_pins.h"

I2C_HandleTypeDef hi2c1;

void i2c_bus_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    g.Pin = BOARD_I2C1_SCL_PIN | BOARD_I2C1_SDA_PIN;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        /* Leave bus unusable; callers see NACK */
    }
}

/*
 * Power-up retry (3.4): is addr7 answering? Returns 0 at once if SDA is held
 * low (another part still holding the bus). Otherwise resets the I2C1
 * peripheral (clears a BUSY flag left by a bus held at power-up) and sends
 * one address probe, ~1 ms. Never stalls the main loop for long.
 */
uint8_t i2c_bus_probe(uint8_t addr7)
{
    if (HAL_GPIO_ReadPin(GPIOB, BOARD_I2C1_SDA_PIN) == GPIO_PIN_RESET) {
        return 0;
    }
    (void)HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        return 0;
    }
    return (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 1, 2) == HAL_OK)
        ? 1u : 0u;
}

uint8_t i2c_write_byte(uint8_t addr7, uint8_t data)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1), &data, 1, 20)
        == HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1), buf, 2, 20)
        == HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t i2c_read_reg(uint8_t addr7, uint8_t reg, uint8_t *val)
{
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, val, 1, 20) == HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t i2c_read_buf(uint8_t addr7, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(addr7 << 1), buf, len, 20) == HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t i2c_ready(uint8_t addr7)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 1, 2) == HAL_OK)
        ? 1u : 0u;
}

uint8_t i2c_write_buf(uint8_t addr7, const uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1),
                                (uint8_t *)buf, len, 50) == HAL_OK) {
        return 1;
    }
    return 0;
}
