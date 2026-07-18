/**
 * @file    flash_config.h
 * @brief   Flash configuration API.
 */
#ifndef FLASH_CONFIG_H
#define FLASH_CONFIG_H

#include "stm32h7xx_hal.h"

void              flash_config_init(uint32_t sysclk_mhz);
void              flash_config_art_disable(void);
void              flash_config_art_enable(void);
HAL_StatusTypeDef flash_config_option_bytes(void);

#endif /* FLASH_CONFIG_H */
