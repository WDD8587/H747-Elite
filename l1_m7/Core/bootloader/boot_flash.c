/**
 * @file    boot_flash.c
 * @brief   Flash operations for bootloader: erase, program, verify.
 *
 * @details Uses HAL flash API with ART accelerator disable/enable wrapping.
 *          Implements retry logic on flash errors and verifies writes.
 */

#include <stdint.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "boot_flash.h"

/* ---------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------*/
#define FLASH_TIMEOUT_MS        5000U
#define FLASH_RETRY_COUNT       3U
#define FLASH_DWORD_SIZE        8U

/* Compare mask for double-word verification */
#define DWORD_MASK              0xFFFFFFFFFFFFFFFFULL

/* ---------------------------------------------------------------------------
 * ART accelerator control
 * -------------------------------------------------------------------------*/
static void art_disable(void)
{
    /* Disable ART: clear EN bit; wait for busy clear */
    FLASH->ARTCR &= ~(FLASH_ARTCR_EN);
    while (FLASH->ARTCR & FLASH_ARTCR_EN) { /* spin */ }
}

static void art_enable(void)
{
    FLASH->ARTCR |= (FLASH_ARTCR_EN | FLASH_ARTCR_DCEN |
                     FLASH_ARTCR_ICEN | FLASH_ARTCR_PFEN);
}

/* ---------------------------------------------------------------------------
 * Internal flash read (word-aligned, 32-bit words)
 * -------------------------------------------------------------------------*/

void boot_flash_read(uint32_t addr, void *buf, uint32_t size)
{
    uint32_t *dst = (uint32_t *)buf;
    uint32_t  words = (size + 3U) / 4U;

    for (uint32_t i = 0; i < words; i++) {
        dst[i] = *(volatile uint32_t *)(addr + i * 4U);
    }
}

/* ---------------------------------------------------------------------------
 * Flash unlock / lock
 * -------------------------------------------------------------------------*/
static HAL_StatusTypeDef flash_unlock(void)
{
    HAL_StatusTypeDef ret;

    /* Unlock main flash */
    ret = HAL_FLASH_Unlock();
    if (ret != HAL_OK) {
        return ret;
    }

    /* Unlock option bytes if needed (not always) -- skip for bootloader */
    return HAL_OK;
}

static HAL_StatusTypeDef flash_lock(void)
{
    return HAL_FLASH_Lock();
}

/* ---------------------------------------------------------------------------
 * Sector-to-bank mapping for STM32H747 dual-bank flash
 *
 * H747 has 2 banks of 1MB each:
 *   Bank 1: sectors 0..7 (128 KB each)
 *   Bank 2: sectors 8..15
 *
 * Bootloader is in sectors 0-1 (0x08000000 - 0x0801FFFF, 128 KB).
 * Application is sectors 2-7 (0x08020000 - 0x080FFFFF, 896 KB).
 * We protect sectors 0-1.
 *
 * For this bootloader, we treat addresses linearly and compute
 * the sector number based on address.
 * -------------------------------------------------------------------------*/

/**
 * @brief  Get the flash sector number for a given address.
 *
 * @param  address  Flash address.
 * @return Sector number (0..15) or -1 if invalid.
 */
static int32_t get_sector_from_addr(uint32_t address)
{
    const uint32_t base  = 0x08000000UL;
    const uint32_t bank1 = 0x080FFFFFUL;
    const uint32_t bank2 = 0x081FFFFFUL;

    if ((address < base) || (address > bank2)) {
        return -1;
    }

    uint32_t offset = address - base;
    uint32_t bank_size = 0x00100000UL; /* 1 MB per bank */

    uint32_t bank = offset / bank_size;
    uint32_t sector_offset = offset % bank_size;

    if (bank == 0) {
        /* Bank 1: 8 sectors of 128 KB */
        return (int32_t)(sector_offset / 0x00020000UL);
    } else {
        /* Bank 2: 8 sectors of 128 KB */
        return (int32_t)(8 + (sector_offset / 0x00020000UL));
    }
}

/* ---------------------------------------------------------------------------
 * Erase region
 * -------------------------------------------------------------------------*/

int boot_flash_erase_region(uint32_t start_addr, uint32_t size)
{
    HAL_StatusTypeDef hal_ret;
    uint32_t          end_addr = start_addr + size;
    int32_t           sector;
    uint32_t          last_sector = 0xFFFFFFFFU;
    int               retry;
    int               first = 1;

    /* Align to 32-byte flash word (not strictly needed for erase) */

    if (flash_unlock() != HAL_OK) {
        return -1;
    }

    /* Iterate sectors in the region */
    uint32_t addr = start_addr;
    while (addr < end_addr) {
        sector = get_sector_from_addr(addr);
        if (sector < 0) {
            flash_lock();
            return -2;
        }

        /* Skip if same sector as last (sector covers > 1 addr) */
        if ((uint32_t)sector == last_sector) {
            addr += 0x00020000UL;
            continue;
        }

        last_sector = (uint32_t)sector;

        /* Erase with retries */
        retry = FLASH_RETRY_COUNT;
        do {
            FLASH_EraseInitTypeDef erase_cfg;
            uint32_t               sector_err = 0;

            erase_cfg.TypeErase   = FLASH_TYPEERASE_SECTORS;
            erase_cfg.Sector      = (uint32_t)sector;
            erase_cfg.NbSectors   = 1;
            erase_cfg.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7V-3.6V */

            art_disable();

            hal_ret = HAL_FLASHEx_Erase(&erase_cfg, &sector_err);

            art_enable();

            if ((hal_ret == HAL_OK) && (sector_err == 0xFFFFFFFFU)) {
                break;  /* Success */
            }

            retry--;

            if (retry > 0) {
                /* Small delay before retry */
                for (volatile uint32_t d = 0; d < 1000; d++) { __NOP(); }
            }
        } while (retry > 0);

        if ((hal_ret != HAL_OK) || (retry == 0)) {
            flash_lock();
            return -3 - (int)(last_sector & 0xFFU);
        }

        if (first) {
            first = 0;
        }
        addr += 0x00020000UL;
    }

    flash_lock();
    return 0;
}

/* ---------------------------------------------------------------------------
 * Program region (double-word writes)
 * -------------------------------------------------------------------------*/

int boot_flash_write_region(uint32_t dest_addr,
                             const uint32_t *src,
                             uint32_t size)
{
    HAL_StatusTypeDef hal_ret;
    uint32_t          end_addr = dest_addr + size;
    int               retry;

    if (flash_unlock() != HAL_OK) {
        return -1;
    }

    uint32_t addr = dest_addr;
    uint32_t src_idx = 0;

    /* Align source to double-word boundary */
    while (addr < end_addr) {
        uint64_t data = 0;

        /* Assemble double-word from two 32-bit words */
        data = ((uint64_t)src[src_idx + 1] << 32) | (uint64_t)src[src_idx];
        src_idx += 2;

        retry = FLASH_RETRY_COUNT;
        do {
            art_disable();

            hal_ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                         addr, data);

            art_enable();

            if (hal_ret == HAL_OK) {
                break;
            }

            retry--;
            if (retry > 0) {
                for (volatile uint32_t d = 0; d < 1000; d++) { __NOP(); }
            }
        } while (retry > 0);

        if (hal_ret != HAL_OK) {
            flash_lock();
            return -10 - (int)((addr - dest_addr) / 0x00020000UL);
        }

        /* Verify the write */
        uint64_t read_back = *(volatile uint64_t *)addr;
        if (read_back != data) {
            flash_lock();
            return -20 - (int)((addr - dest_addr) / 0x00020000UL);
        }

        addr += FLASH_DWORD_SIZE;
    }

    flash_lock();
    return 0;
}

/**
 * @brief  Verify a region of flash against a source buffer.
 *
 * @param  addr   Flash address to verify.
 * @param  src    Source data buffer.
 * @param  size   Number of bytes.
 * @return 0 if match, -1 on mismatch.
 */
int boot_flash_verify(uint32_t addr, const uint32_t *src, uint32_t size)
{
    uint32_t word_count = (size + 3U) / 4U;

    for (uint32_t i = 0; i < word_count; i++) {
        uint32_t flash_word = *(volatile uint32_t *)(addr + i * 4);
        if (flash_word != src[i]) {
            return -1;
        }
    }
    return 0;
}
