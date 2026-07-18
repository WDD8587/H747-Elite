/**
 * @file    nvic_config.h
 * @brief   NVIC configuration API.
 */
#ifndef NVIC_CONFIG_H
#define NVIC_CONFIG_H

#include "stm32h7xx_hal.h"

void     nvic_config_init(void);
void     nvic_config_enable(IRQn_Type irqn, uint32_t preempt_prio,
                             uint32_t sub_prio);
void     nvic_config_disable(IRQn_Type irqn);
uint32_t nvic_config_get_priority(IRQn_Type irqn);

#endif /* NVIC_CONFIG_H */
