/**
 * @file    rk_boot.c
 * @brief   RK3566 U-Boot environment manipulation for A/B boot slot management.
 *
 * @details Provides functions to read/write U-Boot environment variables
 *          used for A/B update scheme:
 *            - bootslot:  "a" or "b"
 *            - bootcount: number of boot attempts
 *            - upgrade_available: "1" if OTA staged
 *
 *          Uses fw_setenv / fw_printenv (or direct MTD access via /dev/mtd)
 *          depending on platform configuration.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "rk_boot.h"

/* ---------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------*/

/*
 * Path to fw_printenv / fw_setenv utilities.
 * On RK3566 these are typically in /usr/sbin/.
 */
#define FW_PRINTENV_PATH       "/usr/sbin/fw_printenv"
#define FW_SETENV_PATH         "/usr/sbin/fw_setenv"

/*
 * Fallback: direct MTD device for U-Boot environment.
 * Used if fw_printenv/setenv are unavailable.
 */
#define MTD_UBOOT_ENV_DEV      "/dev/mtd4"
#define MTD_UBOOT_ENV_SIZE     (16UL * 1024UL)   /* 16 KB default env */
#define MTD_UBOOT_ENV_OFFSET   0

/*
 * Environment variable names.
 * The actual variable names depend on the U-Boot configuration.
 */
#define ENV_BOOTSLOT           "bootslot"
#define ENV_BOOTCOUNT          "bootcount"
#define ENV_UPGRADE_AVAILABLE  "upgrade_available"

/* Default values */
#define BOOTSLOT_DEFAULT       "a"
#define BOOTCOUNT_DEFAULT      "0"
#define UPGRADE_AVAILABLE_DEFAULT "0"

/* ---------------------------------------------------------------------------
 * Static helpers for direct MTD access (fallback)
 *
 * U-Boot environment format:
 *   - CRC32 (4 bytes)
 *   - Byte 0x00 terminated key=value pairs
 *   - 0x00 padding to end
 * -------------------------------------------------------------------------*/

/**
 * @brief  Parse a U-Boot environment buffer for a specific key.
 *
 * @param  env_buf  Buffer containing U-Boot env data.
 * @param  env_len  Length of the buffer.
 * @param  key      Key to find.
 * @param  out_val  Output buffer for the value.
 * @param  val_max  Size of output buffer.
 * @return 0 on success, -1 if key not found.
 */
static int uboot_env_parse(const uint8_t *env_buf, size_t env_len,
                            const char *key, char *out_val, size_t val_max)
{
    size_t key_len = strlen(key);
    size_t pos = 4;  /* Skip CRC32 */

    while (pos < env_len) {
        if (env_buf[pos] == '\0') {
            break;  /* End of valid data */
        }

        /* Check if this entry matches our key */
        if ((pos + key_len < env_len) &&
            (memcmp(&env_buf[pos], key, key_len) == 0) &&
            (env_buf[pos + key_len] == '='))
        {
            /* Found key; copy value */
            size_t val_start = pos + key_len + 1;
            size_t val_end = val_start;

            while (val_end < env_len && env_buf[val_end] != '\0') {
                val_end++;
            }

            size_t copy_len = val_end - val_start;
            if (copy_len >= val_max) {
                copy_len = val_max - 1;
            }
            memcpy(out_val, &env_buf[val_start], copy_len);
            out_val[copy_len] = '\0';
            return 0;
        }

        /* Skip to next null terminator */
        while (pos < env_len && env_buf[pos] != '\0') {
            pos++;
        }
        pos++;  /* Skip null */
    }

    return -1;  /* Not found */
}

/**
 * @brief  Read U-Boot environment via MTD device.
 *
 * @param  buf      Output buffer.
 * @param  buf_size Size of buffer.
 * @return Number of bytes read, or -1 on error.
 */
static int uboot_env_mtd_read(uint8_t *buf, size_t buf_size)
{
    int fd = open(MTD_UBOOT_ENV_DEV, O_RDONLY);
    if (fd < 0) {
        return -1;
    }

    ssize_t n = read(fd, buf, buf_size);
    close(fd);

    return (int)n;
}

/**
 * @brief  Write U-Boot environment buffer back to MTD device.
 *
 * @param  buf      Buffer containing modified environment.
 * @param  buf_size Size of data to write.
 * @return 0 on success, -1 on error.
 */
static int uboot_env_mtd_write(const uint8_t *buf, size_t buf_size)
{
    int fd = open(MTD_UBOOT_ENV_DEV, O_WRONLY | O_SYNC);
    if (fd < 0) {
        return -1;
    }

    /* Erase the MTD sector first (assumes erase size = 4KB) */
    uint32_t erase_size = 4096;
    for (size_t off = 0; off < buf_size; off += erase_size) {
        /* MTD erase is done via ioctl; simplified here */
        lseek(fd, (off_t)off, SEEK_SET);
        ssize_t n = write(fd, buf + off, erase_size);
        if (n < 0) {
            close(fd);
            return -1;
        }
    }

    close(fd);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Public API — RK3566 boot control
 * -------------------------------------------------------------------------*/

/**
 * @brief  Read a U-Boot environment variable.
 *
 * Tries fw_printenv first, then falls back to direct MTD access.
 *
 * @param  key     Variable name.
 * @param  out_val Output buffer (must be >= 64 bytes).
 * @return 0 on success, -1 on error.
 */
int rk_boot_env_read(const char *key, char *out_val)
{
    /* Attempt fw_printenv */
    char cmd[256];
    int  ret;

    ret = snprintf(cmd, sizeof(cmd), "%s %s 2>/dev/null",
                   FW_PRINTENV_PATH, key);
    if (ret < 0 || (size_t)ret >= sizeof(cmd)) {
        goto fallback;
    }

    FILE *fp = popen(cmd, "r");
    if (fp != NULL) {
        if (fgets(out_val, 64, fp) != NULL) {
            /* Strip trailing newline */
            size_t len = strlen(out_val);
            if (len > 0 && out_val[len - 1] == '\n') {
                out_val[len - 1] = '\0';
            }
            pclose(fp);
            return 0;
        }
        pclose(fp);
    }

fallback:
    /* Direct MTD access */
    {
        uint8_t env_buf[MTD_UBOOT_ENV_SIZE];
        int n = uboot_env_mtd_read(env_buf, sizeof(env_buf));
        if (n <= 0) {
            return -1;
        }
        if (uboot_env_parse(env_buf, (size_t)n, key, out_val, 64) != 0) {
            return -2;  /* Key not found */
        }
    }

    return 0;
}

/**
 * @brief  Write a U-Boot environment variable.
 *
 * Tries fw_setenv first, then falls back to direct MTD access.
 *
 * @param  key   Variable name.
 * @param  value Value to set.
 * @return 0 on success, -1 on error.
 */
int rk_boot_env_write(const char *key, const char *value)
{
    char cmd[512];
    int  ret;

    /* Attempt fw_setenv */
    ret = snprintf(cmd, sizeof(cmd), "%s %s %s 2>/dev/null",
                   FW_SETENV_PATH, key, value);
    if (ret < 0 || (size_t)ret >= sizeof(cmd)) {
        goto fallback_mtd;
    }

    int exit_code = system(cmd);
    if (exit_code == 0) {
        return 0;
    }

fallback_mtd:
    /* Direct MTD write — simplified; a full implementation would
     * recompute the CRC32 and write the entire env block. */
    {
        /* Read current env */
        uint8_t env_buf[MTD_UBOOT_ENV_SIZE];
        int n = uboot_env_mtd_read(env_buf, sizeof(env_buf));
        if (n <= 0) {
            return -1;
        }

        /* Find and replace the key/value, or append */
        /* This is a simplified stub — use fw_setenv for production */
        (void)env_buf;
        (void)n;
    }

    return -1;
}

/**
 * @brief  Get the current active boot slot.
 *
 * @return "a", "b", or NULL on failure (static storage).
 */
const char *rk_boot_get_slot(void)
{
    static char slot[8] = {0};

    if (rk_boot_env_read(ENV_BOOTSLOT, slot) != 0) {
        return BOOTSLOT_DEFAULT;
    }

    if ((slot[0] != 'a' && slot[0] != 'b')) {
        return BOOTSLOT_DEFAULT;
    }

    return slot;
}

/**
 * @brief  Set the active boot slot for next boot.
 *
 * @param  slot  "a" or "b".
 * @return 0 on success, -1 on error.
 */
int rk_boot_set_slot(const char *slot)
{
    if ((slot == NULL) ||
        (slot[0] != 'a' && slot[0] != 'b') ||
        (slot[1] != '\0')) {
        return -1;
    }

    return rk_boot_env_write(ENV_BOOTSLOT, slot);
}

/**
 * @brief  Get the current boot counter.
 *
 * @return Boot count value (0 if unset or error).
 */
uint32_t rk_boot_get_bootcount(void)
{
    char buf[16] = {0};

    if (rk_boot_env_read(ENV_BOOTCOUNT, buf) != 0) {
        return 0;
    }

    return (uint32_t)strtoul(buf, NULL, 10);
}

/**
 * @brief  Set the boot counter.
 *
 * @param  count  Value to set.
 * @return 0 on success, -1 on error.
 */
int rk_boot_set_bootcount(uint32_t count)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", (unsigned int)count);
    return rk_boot_env_write(ENV_BOOTCOUNT, buf);
}

/**
 * @brief  Check if an upgrade is available.
 *
 * @return 1 if upgrade is pending, 0 otherwise (or on error).
 */
int rk_boot_upgrade_available(void)
{
    char buf[8] = {0};

    if (rk_boot_env_read(ENV_UPGRADE_AVAILABLE, buf) != 0) {
        return 0;
    }

    return (buf[0] == '1') ? 1 : 0;
}

/**
 * @brief  Mark upgrade as available (or clear the flag).
 *
 * @param  available  1 to stage upgrade, 0 to clear.
 * @return 0 on success, -1 on error.
 */
int rk_boot_set_upgrade_available(int available)
{
    const char *val = (available) ? "1" : "0";
    return rk_boot_env_write(ENV_UPGRADE_AVAILABLE, val);
}

/**
 * @brief  Mark the current boot as successful (reset bootcount, clear flag).
 *
 * Call this from application code once the system has booted successfully.
 *
 * @return 0 on success, -1 on error.
 */
int rk_boot_mark_successful(void)
{
    if (rk_boot_set_bootcount(0) != 0) {
        return -1;
    }
    if (rk_boot_set_upgrade_available(0) != 0) {
        return -1;
    }
    return 0;
}

/**
 * @brief  Switch to the alternate boot slot.
 *
 * @return 0 on success, -1 on error.
 */
int rk_boot_switch_slot(void)
{
    const char *current = rk_boot_get_slot();

    if (current == NULL) {
        return -1;
    }

    const char *next = (current[0] == 'a') ? "b" : "a";
    return rk_boot_set_slot(next);
}
