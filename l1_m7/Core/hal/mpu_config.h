/**
 * @file    mpu_config.h
 * @brief   MPU configuration API.
 */
#ifndef MPU_CONFIG_H
#define MPU_CONFIG_H

#include <stdint.h>

void mpu_config_init(void);
void mpu_config_disable(void);
void mpu_config_set_region(uint32_t region, uint32_t base, uint32_t size,
                            uint32_t attrs);

#endif /* MPU_CONFIG_H */
