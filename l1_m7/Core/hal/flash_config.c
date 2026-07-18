/**
 * @file    flash_config.c
 * @brief   Flash wait state, ART accelerator, option byte configuration.
 *
 * @details Provides flash initialisation for 480 MHz operation:
 *          - 4 wait states (FLASH_LATENCY_4) for 480 MHz @ 3.3V.
 *          - ART accelerator (cache + prefetch) enabled.
 *          - Flash prefetch buffer enabled.
 *          - Option bytes: BOR level 2.7V, WRP sectors 0-1 (bootloader
 *            protected), RDP level 0 (no read protection).
 */

#include "stm32h7xx_hal.h"
#include "flash_config.h"

/* ---------------------------------------------------------------------------
 * FLASH_ACR register bit definitions (RM0455 rev4)
 * -------------------------------------------------------------------------*/
#define FLASH_ACR_LATENCY_MASK  0x0000000FU
#define FLASH_ACR_PRFTEN        (1U << 8)
#define FLASH_ACR_ARTEN         (1U << 9)
#define FLASH_ACR_ARTICEN       (1U << 15)  /* ART instruction cache */
#define FLASH_ACR_ARTDCEN       (1U << 16)  /* ART data cache      */
#define FLASH_ACR_ART_PFEN      (1U << 17)  /* ART prefetch enable */

/* ---------------------------------------------------------------------------
 * Option bytes
 * -------------------------------------------------------------------------*/

/**
 * @brief  Flash option byte configuration structure.
 *
 * Fields:
 *   BOR_LEVEL: 2.7V (BOR level 2)
 *   WRP_SECTORS: Sectors 0-1 protected (bootloader)
 *   RDP: Level 0 (disabled)
 */
#define OPTION_BYTE_BOR_LEV     OB_BOR_LEVEL2        /* ~2.7V brown-out */
#define OPTION_BYTE_WRP_START   0
#define OPTION_BYTE_WRP_END     1
#define OPTION_BYTE_RDP         OB_RDP_LEVEL0        /* No protection   */

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise flash controller for 480 MHz operation.
 *
 * Called early in boot (before clock_tree_init) with default
 * wait states, then again after clock tree is set.
 *
 * @param  sysclk_mhz  System clock frequency in MHz. Use 0 for default (480).
 */
void flash_config_init(uint32_t sysclk_mhz)
{
    if (sysclk_mhz == 0) {
        sysclk_mhz = 480;
    }

    /*
     * Determine required wait states per RM0455 Table 7.
     * Voltage range 1 (VOS1), 3.3V supply:
     *   0 WS:   0-70 MHz
     *   1 WS:  70-140 MHz
     *   2 WS: 140-210 MHz
     *   3 WS: 210-280 MHz
     *   4 WS: 280-350 MHz
     *   5 WS: 350-400 MHz (480 MHz requires 4 WS; actually 5 for 480)
     *   6 WS: 400-480 MHz
     *
     * At 480 MHz we need 5 WS (FLASH_LATENCY_5) for VOS1.
     * Using 4 WS for 400 MHz fallback; adjust based on actual test.
     */
    uint32_t latency = FLASH_LATENCY_4;  /* Safe for 400 MHz */
    if (sysclk_mhz > 400) {
        latency = FLASH_LATENCY_5;
    } else if (sysclk_mhz > 350) {
        latency = FLASH_LATENCY_4;
    } else if (sysclk_mhz > 280) {
        latency = FLASH_LATENCY_3;
    } else if (sysclk_mhz > 210) {
        latency = FLASH_LATENCY_2;
    } else if (sysclk_mhz > 140) {
        latency = FLASH_LATENCY_1;
    } else {
        latency = FLASH_LATENCY_0;
    }

    /* Set wait states via HAL */
    __HAL_FLASH_SET_LATENCY(latency);

    /* Verify the latency was set */
    while (__HAL_FLASH_GET_LATENCY() != latency) {
        /* Spin until latched (should complete immediately) */
    }

    /* ---- ART accelerator ---- */
    /*
     * Enable ART with instruction cache, data cache, and prefetch.
     * The ART accelerator reduces effective flash latency by caching.
     *
     * Register: FLASH_ARTCR (at 0x52002010)
     *   Bit 0: EN      - ART enable
     *   Bit 1: ICEN    - Instruction cache enable
     *   Bit 2: DCEN    - Data cache enable
     *   Bit 3: PFEN    - Prefetch enable
     */
    FLASH->ARTCR |= (FLASH_ARTCR_EN | FLASH_ARTCR_DCEN |
                     FLASH_ARTCR_ICEN | FLASH_ARTCR_PFEN);

    /* ---- Prefetch buffer ---- */
    /*
     * Enable the flash prefetch buffer (separate from ART prefetch;
     * this controls the AHB prefetch).
     */
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);

    /* ---- Data cache enable for D1 domain ---- */
    /* SCB->CACR enables data cache; handled in MPU / startup */
}

/**
 * @brief  Disable ART accelerator (required before flash write/erase).
 */
void flash_config_art_disable(void)
{
    FLASH->ARTCR &= ~FLASH_ARTCR_EN;
    while (FLASH->ARTCR & FLASH_ARTCR_EN) { /* Wait */ }
}

/**
 * @brief  Re-enable ART accelerator after flash write/erase.
 */
void flash_config_art_enable(void)
{
    FLASH->ARTCR |= (FLASH_ARTCR_EN | FLASH_ARTCR_DCEN |
                     FLASH_ARTCR_ICEN | FLASH_ARTCR_PFEN);
}

/**
 * @brief  Configure option bytes.
 *
 * Sets BOR level, WRP sectors, and RDP level.
 * This function triggers a system reset if option bytes differ.
 *
 * @return HAL_OK on success (may not return if reset occurs).
 */
HAL_StatusTypeDef flash_config_option_bytes(void)
{
    HAL_StatusTypeDef ret;
    HAL_FLASH_OBProgramInitTypeDef ob = {0};

    /* Get current option bytes */
    HAL_FLASHEx_OBGetConfig(&ob);

    /* Check if configuration matches desired values */
    uint8_t  desired_bor   = OPTION_BYTE_BOR_LEV;
    uint16_t desired_wrp_s = OPTION_BYTE_WRP_START;
    uint16_t desired_wrp_e = OPTION_BYTE_WRP_END;
    uint8_t  desired_rdp   = OPTION_BYTE_RDP;

    /* Only program if different */
    if ((ob.BORLevel != desired_bor) ||
        (ob.WRPStartSector != desired_wrp_s) ||
        (ob.WRPEndSector != desired_wrp_e) ||
        (ob.RDPLevel != desired_rdp))
    {
        /* Unlock flash and option bytes */
        ret = HAL_FLASH_OB_Unlock();
        if (ret != HAL_OK) return ret;

        /* Prepare new configuration */
        ob.OptionType = OPTIONBYTE_WRP | OPTIONBYTE_RDP | OPTIONBYTE_BOR;
        ob.WRPState   = OB_WRPSTATE_ENABLE;
        ob.WRPSector  = (uint32_t)desired_wrp_s |
                        ((uint32_t)desired_wrp_e << 16);
        ob.RDPLevel   = desired_rdp;
        ob.BORLevel   = desired_bor;

        ret = HAL_FLASHEx_OBProgram(&ob);
        if (ret != HAL_OK) {
            HAL_FLASH_OB_Lock();
            return ret;
        }

        /* Launch option byte loading (triggers reset) */
        HAL_FLASH_OB_Launch();

        /* If we reach here, OB launch did not reset — unexpected */
        HAL_FLASH_OB_Lock();
    }

    return HAL_OK;
}
