/**
 * @file    rk_boot.h
 * @brief   RK3566 boot control API.
 */
#ifndef RK_BOOT_H
#define RK_BOOT_H

#include <stdint.h>

/* Environment read/write */
int         rk_boot_env_read(const char *key, char *out_val);
int         rk_boot_env_write(const char *key, const char *value);

/* A/B slot management */
const char *rk_boot_get_slot(void);
int         rk_boot_set_slot(const char *slot);
int         rk_boot_switch_slot(void);

/* Boot count */
uint32_t    rk_boot_get_bootcount(void);
int         rk_boot_set_bootcount(uint32_t count);

/* Upgrade tracking */
int         rk_boot_upgrade_available(void);
int         rk_boot_set_upgrade_available(int available);

/* Lifecycle */
int         rk_boot_mark_successful(void);

#endif /* RK_BOOT_H */
