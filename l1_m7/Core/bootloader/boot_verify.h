/**
 * @file    boot_verify.h
 * @brief   Firmware verification API.
 */
#ifndef BOOT_VERIFY_H
#define BOOT_VERIFY_H

#include <stdint.h>

int  boot_verify_image(uint32_t image_base, uint32_t max_size);
void boot_verify_get_expected_hash(uint32_t image_base, uint8_t *out_hash);
void boot_verify_get_signature(uint32_t image_base, uint8_t *out_sig);

#endif /* BOOT_VERIFY_H */
