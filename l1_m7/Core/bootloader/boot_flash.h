/**
 * @file    boot_flash.h
 * @brief   Flash operations API for bootloader.
 */
#ifndef BOOT_FLASH_H
#define BOOT_FLASH_H

#include <stdint.h>

void boot_flash_read(uint32_t addr, void *buf, uint32_t size);
int  boot_flash_erase_region(uint32_t start_addr, uint32_t size);
int  boot_flash_write_region(uint32_t dest_addr, const uint32_t *src,
                              uint32_t size);
int  boot_flash_verify(uint32_t addr, const uint32_t *src, uint32_t size);

#endif /* BOOT_FLASH_H */
