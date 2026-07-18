/**
 * @file    boot_main.c
 * @brief   Secure bootloader entry for STM32H747 M7 core.
 * @details Checks boot flag in SRAM2, verifies OTA image (SHA256 + ECDSA),
 *          copies verified image to app flash, clears flag, and jumps.
 *          Also reads reset reason from RCC_CSR.
 *
 * Boot flow:
 *   1. Read RCC_CSR to determine reset cause (POR, WDT, SW, etc.)
 *   2. Check OTA boot flag at SRAM2+0 (4-byte magic)
 *   3. If OTA_FLAG_MAGIC is set:
 *      a. Verify new image in staging area (SHA256 of 512KB app region)
 *      b. ECDSA P-256 signature verification on image header
 *      c. If verification passes, copy staging -> app flash
 *      d. Clear OTA flag
 *      e. Jump to app
 *   4. If no flag, jump directly to app (0x08010000)
 */

#include <stdint.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "boot_verify.h"
#include "boot_flash.h"

/* ---------------------------------------------------------------------------
 * Address definitions
 * -------------------------------------------------------------------------*/
#define APP_FLASH_BASE         0x08010000UL   /* Application flash base   */
#define APP_FLASH_SIZE         0x00080000UL   /* 512 KB                   */
#define STAGING_BASE           0x08020000UL   /* OTA staging area         */
#define STAGING_SIZE           0x00080000UL   /* 512 KB                   */

#define SRAM2_BASE             0x20020000UL
#define OTA_FLAG_OFFSET        0x00000000UL   /* 4 bytes at SRAM2 base    */
#define OTA_FLAG_MAGIC         0xDEADBEAFUL

/* Stack top from linker symbol */
extern uint32_t _estack;

/* Application entry point typedef */
typedef void (*app_entry_t)(void);

/* Application vector table (first entry = SP, second = Reset_Handler) */
typedef struct {
    uint32_t    stack_top;
    app_entry_t reset_handler;
} app_vectors_t;

/* ---------------------------------------------------------------------------
 * Reset reason helpers
 * -------------------------------------------------------------------------*/
#define RCC_CSR_ADDR           0x58024400UL   /* D3 domain CSR            */

typedef enum {
    RESET_REASON_POWER_ON      = 0x01,
    RESET_REASON_PIN_RESET     = 0x02,
    RESET_REASON_WATCHDOG      = 0x04,
    RESET_REASON_SOFTWARE      = 0x08,
    RESET_REASON_BOR           = 0x10,
    RESET_REASON_UNKNOWN       = 0xFF,
} reset_reason_t;

/**
 * @brief  Read RCC CSR register and decode reset reason.
 * @return reset_reason_t
 */
static reset_reason_t boot_read_reset_reason(void)
{
    uint32_t csr = *(volatile uint32_t *)RCC_CSR_ADDR;

    /* LPWRRSTF: Low-power reset */
    if (csr & (1UL << 31)) { (void)csr; } /* WUFI: not used */
    if (csr & (1UL << 30)) { /* Low-power management reset */ }

    /* Check individual flags - RM0455 rev4 */
    if (csr & (1UL << 27)) return RESET_REASON_SOFTWARE;    /* SFTRSTF  */
    if (csr & (1UL << 26)) return RESET_REASON_WATCHDOG;     /* IWDG1RSTF*/
    if (csr & (1UL << 25)) return RESET_REASON_WATCHDOG;     /* WWDG1RSTF*/
    if (csr & (1UL << 24)) return RESET_REASON_BOR;          /* BORRSTF  */
    if (csr & (1UL << 23)) return RESET_REASON_PIN_RESET;    /* PINRF    */
    if (csr & (1UL << 22)) return RESET_REASON_POWER_ON;     /* PORRSTF  */

    return RESET_REASON_UNKNOWN;
}

/* ---------------------------------------------------------------------------
 * Boot flag helpers
 * -------------------------------------------------------------------------*/
static inline uint32_t boot_read_ota_flag(void)
{
    return *(volatile uint32_t *)(SRAM2_BASE + OTA_FLAG_OFFSET);
}

static inline void boot_clear_ota_flag(void)
{
    *(volatile uint32_t *)(SRAM2_BASE + OTA_FLAG_OFFSET) = 0x00000000U;
    __DSB();
    __ISB();
}

/* ---------------------------------------------------------------------------
 * Jump to application
 * -------------------------------------------------------------------------*/
static void boot_jump_to_app(uint32_t app_addr)
{
    const app_vectors_t *vectors = (const app_vectors_t *)app_addr;

    /* Basic sanity: SP must be in SRAM range, reset vector must be thumb */
    if ((vectors->stack_top < 0x20000000UL) ||
        (vectors->stack_top >= 0x30000000UL)) {
        /* Invalid SP — trap */
        while (1) { __NOP(); }
    }
    if ((vectors->reset_handler & 0x01UL) == 0) {
        /* Not a Thumb entry point — trap */
        while (1) { __NOP(); }
    }

    /* Disable interrupts globally */
    __disable_irq();

    /* Disable SysTick */
    SysTick->CTRL = 0U;

    /* Disable all interrupt sources in NVIC */
    for (uint32_t i = 0U; i < 8U; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    /* Set VTOR to new app vector table */
    SCB->VTOR = app_addr;

    /* Reset handler expects stack pointer from vector table */
    __set_MSP(vectors->stack_top);
    __set_PSP(vectors->stack_top);   /* In case app uses process stack */

    /* DSB+ISB before jump */
    __DSB();
    __ISB();

    /* Call the reset handler */
    vectors->reset_handler();

    /* Should never return */
    while (1) { __NOP(); }
}

/* ---------------------------------------------------------------------------
 * HAL initialisation stub (minimal: enable LSE, flash prefetch, wait-states)
 * -------------------------------------------------------------------------*/
static void boot_system_init(void)
{
    HAL_Init();

    /* Enable D3 domain power interface clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Set flash wait-states for 480 MHz operation */
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_4);

    /* Enable ART accelerator for M7 */
    FLASH->ARTCR = (FLASH_ARTCR_EN | FLASH_ARTCR_DCEN |
                    FLASH_ARTCR_ICEN | FLASH_ARTCR_PFEN);
}

/* ---------------------------------------------------------------------------
 * main() — bootloader entry
 * -------------------------------------------------------------------------*/
int main(void)
{
    reset_reason_t reason = boot_read_reset_reason();
    (void)reason;  /* Could be logged in production */

    boot_system_init();

    /* Enable SYSCFG clock (needed for VTOR etc.) */
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    uint32_t ota_flag = boot_read_ota_flag();
    int      result   = 0;

    if (ota_flag == OTA_FLAG_MAGIC)
    {
        /* ---- OTA update flow ---- */
        /* 1. Verify staging image */
        result = boot_verify_image(STAGING_BASE, APP_FLASH_SIZE);
        if (result == 0)
        {
            /* 2. Erase app flash area */
            result = boot_flash_erase_region(APP_FLASH_BASE, APP_FLASH_SIZE);
            if (result == 0)
            {
                /* 3. Copy verified image: staging -> app flash */
                result = boot_flash_write_region(APP_FLASH_BASE,
                                                 (const uint32_t *)STAGING_BASE,
                                                 APP_FLASH_SIZE);
            }

            /* 4. Clear OTA flag (regardless of flash write result —
                  we do NOT retry a corrupt image on next boot) */
            boot_clear_ota_flag();

            /* If copy failed, fall through to boot existing app */
            if (result != 0)
            {
                /* Log failure — application can check last error in SRAM4 */
                *(volatile uint32_t *)(SRAM2_BASE + 0x04) = (uint32_t)result;
            }
        }
        else
        {
            /* Verification failed — clear flag and boot current app */
            boot_clear_ota_flag();
            *(volatile uint32_t *)(SRAM2_BASE + 0x04) = 0xDEAD0001U;
        }
    }

    /* ---- Boot existing app ---- */
    boot_jump_to_app(APP_FLASH_BASE);

    /* Never reached */
    return 0;
}

/* ---------------------------------------------------------------------------
 * HAL weak overrides
 * -------------------------------------------------------------------------*/
void HAL_MspInit(void)     { /* Bootloader uses no HAL MSP */ }
void HAL_MspDeInit(void)   { /* Bootloader uses no HAL MSP */ }
void SysTick_Handler(void) { /* Bootloader does not use SysTick */ }
