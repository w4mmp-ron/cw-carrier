#include "board.h"
#include "control.h"
#include "i2c_bus.h"
#include "cw.h"
#include "proficio_config.h"
#include "hang_rec.h"
#include "stm32f4xx_hal.h"

static void SystemClock_Config(void);
static void Error_Handler(void);

static uint8_t s_reset_flags;   /* RCC_CSR bits 24-31 at power-up (3.12) */

void board_init(void)
{
    /* Why we started (watchdog / pin / power / software), then clear the flags */
    s_reset_flags = (uint8_t)(RCC->CSR >> 24);
    RCC->CSR |= RCC_CSR_RMVF;
    hang_rec_boot(s_reset_flags);   /* keep the hang record of a watchdog reset (3.13) */

    HAL_Init();
    SystemClock_Config();
    control_init();
    i2c_bus_init();
}

void board_led_on(void)
{
    HAL_GPIO_WritePin(BOARD_LED_GPIO, BOARD_LED_PIN, GPIO_PIN_RESET);
}

void board_led_off(void)
{
    HAL_GPIO_WritePin(BOARD_LED_GPIO, BOARD_LED_PIN, GPIO_PIN_SET);
}

void board_led_toggle(void)
{
    HAL_GPIO_TogglePin(BOARD_LED_GPIO, BOARD_LED_PIN);
}

void board_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

uint32_t board_millis(void)
{
    return HAL_GetTick();
}

uint8_t board_reset_flags(void)
{
    return s_reset_flags;
}

/*
 * Independent watchdog (3.12): resets the pill if the main loop stops for about
 * 2 s (LSI ~32 kHz / 64 = 500 Hz, reload 1000; LSI spread gives ~1.4-3.8 s).
 * Started after app_init() (its flash erase can take 1-2 s). Stopped by any
 * reset, so the DFU bootloader is not affected. Frozen while the ST-Link halts
 * the core.
 */
void board_watchdog_start(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    IWDG->KR = 0xCCCCu;            /* start (also turns on LSI) */
    IWDG->KR = 0x5555u;            /* unlock PR / RLR */
    IWDG->PR = 4u;                 /* /64 */
    IWDG->RLR = 1000u - 1u;
    while (IWDG->SR != 0u) {
    }
    IWDG->KR = 0xAAAAu;
    hang_rec_start();
}

void board_watchdog_kick(void)
{
    IWDG->KR = 0xAAAAu;
    hang_rec_alive();
}

/**
 * Black Pill clock — same recipe as TinyUSB stm32f411blackpill BSP (known-good USB).
 * HSE 25 MHz: PLLM=25, N=336, P=/4 → 84 MHz SYSCLK; Q=7 → 48 MHz USB.
 * (HSE_VALUE must match crystal; we define 25000000 in platformio.ini.)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = (uint32_t)(HSE_VALUE / 1000000U);
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV4;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    /*
     * PLLI2S for I2S2 @ ~96 kHz (F411 has its own PLLI2SM).
     * HSE 25 MHz: M=22, N=173, R=2 → 98.30 MHz I2SCLK → fS = 98.30M/(256*4)
     * = 95.99 kHz, MCLK 50 % duty (even divider). See i2s_audio.c.
     */
    {
        RCC_PeriphCLKInitTypeDef per = {0};
        per.PeriphClockSelection = RCC_PERIPHCLK_I2S;
        per.PLLI2S.PLLI2SM = 22;
        per.PLLI2S.PLLI2SN = 173;
        per.PLLI2S.PLLI2SR = 2;
        if (HAL_RCCEx_PeriphCLKConfig(&per) != HAL_OK) {
            Error_Handler();
        }
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
#if PROFICIO_FEAT_CW
    cw_key_delay_tick();    /* 1 ms: non-QSK amp CW key delay */
#endif
}

void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

static void Error_Handler(void)
{
    __disable_irq();
    for (;;) {
    }
}
