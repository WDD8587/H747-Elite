/**
 * @file    gpio_config.c
 * @brief   All GPIO pin configurations for STM32H747 system.
 *
 * Every pin used in the system is configured here. Each GPIO is annotated
 * with its purpose, alternate function where applicable, and electrical
 * characteristics (pull-up/down, speed, output type).
 */

#include "stm32h7xx_hal.h"
#include "gpio_config.h"
#include "m7_config.h"

/* ---------------------------------------------------------------------------
 * GPIO pin mapping table
 *
 * For each pin:
 *   - Port, Pin
 *   - Mode (INPUT, OUTPUT, AF)
 *   - Pull (NOPULL, PULLUP, PULLDOWN)
 *   - Speed (LOW, MEDIUM, HIGH, VERY_HIGH)
 *   - Alternate function (0 if GPIO)
 *   - Output type (PushPull / OpenDrain)
 *   - Initial output state (0/1 for outputs)
 * -------------------------------------------------------------------------*/

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      mode;       /* GPIO_MODE_INPUT, _OUTPUT_PP, _AF_PP    */
    uint32_t      pull;       /* GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN */
    uint32_t      speed;      /* GPIO_SPEED_FREQ_LOW/MEDIUM/HIGH/VERY_HIGH */
    uint32_t      alternate;  /* AF number or 0 for GPIO mode            */
    uint8_t       init_state; /* 0/1 — initial output state (outputs)    */
    const char   *purpose;    /* Human-readable description              */
} gpio_pin_cfg_t;

/* ---------------------------------------------------------------------------
 * Peripherals address mapping
 *
 *          INPUTS
 * Bumper L       PA0     GPIO input, pull-down (normally GND, active HIGH)
 * Bumper R       PA1     GPIO input, pull-down
 * IR dock        PA3     GPIO input, pull-up (active LOW sensor)
 * Button         PC13    GPIO input, pull-up (active LOW)
 *
 *          OUTPUTS
 * PWM_SHTDN      PD12    GPIO output, push-pull, HIGH = motors enabled
 * LED_R          PB0     GPIO output, push-pull, active HIGH
 * LED_G          PB1     GPIO output, push-pull, active HIGH
 * LED_B          PB2     GPIO output, push-pull, active HIGH
 * DRV_nSLEEP     PA4     GPIO output, push-pull, HIGH = DRV awake
 * DRV_CS         PB12    GPIO output, push-pull, HIGH = deselect
 * IMU_CS         PA15    GPIO output, push-pull, HIGH = deselect
 * FLASH_CS       PE0     GPIO output, push-pull, HIGH = deselect
 * -------------------------------------------------------------------------*/

static const gpio_pin_cfg_t gpio_config[] = {
    /* ---- INPUTS ---- */
    {GPIOA, GPIO_PIN_0,  GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW,
     0, 0, "Bumper L — PA0, collision sensor left, active HIGH"},
    {GPIOA, GPIO_PIN_1,  GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW,
     0, 0, "Bumper R — PA1, collision sensor right, active HIGH"},
    {GPIOA, GPIO_PIN_3,  GPIO_MODE_INPUT, GPIO_PULLUP,   GPIO_SPEED_FREQ_LOW,
     0, 0, "IR dock sensor — PA3, active LOW when docked"},
    {GPIOC, GPIO_PIN_13, GPIO_MODE_INPUT, GPIO_PULLUP,   GPIO_SPEED_FREQ_LOW,
     0, 0, "User button — PC13, active LOW"},

    /* ---- OUTPUTS ---- */
    {GPIOD, GPIO_PIN_12, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW,
     0, 0, "PWM shutdown — PD12, HIGH=gate driver enabled, LOW=brake"},
    {GPIOB, GPIO_PIN_0,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW,
     0, 0, "LED R — PB0, red indicator, active HIGH"},
    {GPIOB, GPIO_PIN_1,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW,
     0, 0, "LED G — PB1, green indicator, active HIGH"},
    {GPIOB, GPIO_PIN_2,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW,
     0, 0, "LED B — PB2, blue indicator, active HIGH"},
    {GPIOA, GPIO_PIN_4,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW,
     0, 1, "DRV nSLEEP — PA4, HIGH=driver awake, LOW=sleep"},
    {GPIOB, GPIO_PIN_12, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
     0, 1, "DRV SPI CS — PB12, LOW=select DRV8323S"},
    {GPIOA, GPIO_PIN_15, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
     0, 1, "IMU SPI CS — PA15, LOW=select ICM-20948"},
    {GPIOE, GPIO_PIN_0,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
     0, 1, "Flash SPI CS — PE0, LOW=select MX25L512"},

    /* ---- SPI6 IPC Slave (AF5 on GPIOG) ---- */
    {GPIOG, GPIO_PIN_13, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH,
     IPC_SPI_GPIO_AF, 0, "SPI6 SCK  — PG13, IPC clock from RK3566"},
    {GPIOG, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH,
     IPC_SPI_GPIO_AF, 0, "SPI6 MISO — PG12, STM32->RK3566 data"},
    {GPIOG, GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH,
     IPC_SPI_GPIO_AF, 0, "SPI6 MOSI — PG14, RK3566->STM32 data"},
    {GPIOG, GPIO_PIN_9,  GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH,
     IPC_SPI_GPIO_AF, 0, "SPI6 NSS  — PG9, hardware slave select"},
    {GPIOE, GPIO_PIN_1,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
     0, 0, "SPI READY — PE1, HIGH=STM32 data ready for master"},

    /* ---- USB FS (AF10 on PA9/PA11/PA12, internal PHY) ---- */
    {GPIOA, GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH,
     10, 0, "USB DM — PA11, AF10"},
    {GPIOA, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH,
     10, 0, "USB DP — PA12, AF10"},
    {GPIOA, GPIO_PIN_9,  GPIO_MODE_INPUT,  GPIO_PULLDOWN,  GPIO_SPEED_FREQ_LOW,
     0, 0, "USB VBUS sense — PA9, input"},
};

static const uint32_t gpio_config_count =
    sizeof(gpio_config) / sizeof(gpio_config[0]);

/* ---------------------------------------------------------------------------
 * GPIO clock enable
 * -------------------------------------------------------------------------*/

/**
 * @brief  Enable the clock for a GPIO port.
 *
 * @param  port  GPIO port base address.
 */
static inline void gpio_enable_port_clock(GPIO_TypeDef *port)
{
    if (port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (port == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    } else if (port == GPIOE) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    } else if (port == GPIOF) {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    } else if (port == GPIOG) {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    } else if (port == GPIOH) {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    } else if (port == GPIOI) {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Configure all GPIO pins.
 *
 * Call this once during system initialisation (after HAL_Init() and
 * clock tree setup). Iterates the config table and applies settings.
 */
void gpio_config_init_all(void)
{
    GPIO_InitTypeDef init = {0};
    GPIO_TypeDef    *last_port = NULL;

    for (uint32_t i = 0; i < gpio_config_count; i++)
    {
        const gpio_pin_cfg_t *cfg = &gpio_config[i];

        /* Enable port clock (once per port) */
        if (cfg->port != last_port) {
            gpio_enable_port_clock(cfg->port);
            last_port = cfg->port;
        }

        /* Set initial state for outputs before configuring pin */
        if ((cfg->mode == GPIO_MODE_OUTPUT_PP) ||
            (cfg->mode == GPIO_MODE_OUTPUT_OD)) {
            HAL_GPIO_WritePin(cfg->port, cfg->pin,
                              (cfg->init_state) ? GPIO_PIN_SET
                                                : GPIO_PIN_RESET);
        }

        /* Configure pin */
        init.Pin       = cfg->pin;
        init.Mode      = cfg->mode;
        init.Pull      = cfg->pull;
        init.Speed     = cfg->speed;

        if (cfg->alternate != 0) {
            init.Alternate = cfg->alternate;
        }

        HAL_GPIO_Init(cfg->port, &init);
    }
}

/**
 * @brief  Configure a single GPIO pin (for dynamic use at runtime).
 *
 * @param  port    GPIO port.
 * @param  pin     Pin number (GPIO_PIN_x).
 * @param  mode    GPIO_MODE_INPUT, _OUTPUT_PP, _AF_PP, etc.
 * @param  pull    GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN.
 * @param  speed   GPIO_SPEED_FREQ_*.
 * @param  alternate  AF number or 0.
 */
void gpio_config_pin(GPIO_TypeDef *port, uint16_t pin,
                     uint32_t mode, uint32_t pull,
                     uint32_t speed, uint32_t alternate)
{
    GPIO_InitTypeDef init = {0};

    gpio_enable_port_clock(port);

    init.Pin   = pin;
    init.Mode  = mode;
    init.Pull  = pull;
    init.Speed = speed;
    if (alternate != 0) {
        init.Alternate = alternate;
    }

    HAL_GPIO_Init(port, &init);
}

/**
 * @brief  Return a pointer to the pin configuration table.
 *
 * @param  count  Output: number of entries in the table.
 * @return Pointer to the config table.
 */
const gpio_pin_cfg_t *gpio_config_get_table(uint32_t *count)
{
    if (count != NULL) {
        *count = gpio_config_count;
    }
    return gpio_config;
}
