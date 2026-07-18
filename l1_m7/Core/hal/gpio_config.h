/**
 * @file    gpio_config.h
 * @brief   GPIO configuration API.
 */
#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include "stm32h7xx_hal.h"

void gpio_config_init_all(void);
void gpio_config_pin(GPIO_TypeDef *port, uint16_t pin,
                     uint32_t mode, uint32_t pull,
                     uint32_t speed, uint32_t alternate);

/* Access pin configuration table for inspection */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      mode;
    uint32_t      pull;
    uint32_t      speed;
    uint32_t      alternate;
    uint8_t       init_state;
    const char   *purpose;
} gpio_pin_cfg_t;

const gpio_pin_cfg_t *gpio_config_get_table(uint32_t *count);

#endif /* GPIO_CONFIG_H */
