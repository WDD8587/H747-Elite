/**
 * @file    dma_config.h
 * @brief   DMA configuration API.
 */
#ifndef DMA_CONFIG_H
#define DMA_CONFIG_H

#include "stm32h7xx_hal.h"

void dma_config_init_all(void);
void dma_config_deinit(DMA_Stream_TypeDef *stream);
void dma_config_register_callbacks(DMA_HandleTypeDef *hdma,
                                    void (*cplt_cb)(DMA_HandleTypeDef *),
                                    void (*half_cb)(DMA_HandleTypeDef *),
                                    void (*err_cb)(DMA_HandleTypeDef *));

#endif /* DMA_CONFIG_H */
