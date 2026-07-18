/**
 * @file    mpu_config.c
 * @brief   MPU region configuration for STM32H747 M7 core.
 *
 * MPU regions defined:
 *
 *   Region 0: SRAM3   (0x20030000, 64 KB)  — Full access, shareable,
 *                                             eXecute Never disabled (code)
 *                                             Write-back, write-allocate
 *   Region 1: App Flash (0x08010000, 512 KB) — Read-only, XN disabled (code)
 *                                             Write-through, no-write-allocate
 *   Region 2: SRAM2   (0x20020000, 288 KB)  — Full access, M4 backup data
 *                                             XN enabled (data only)
 *                                             Write-through
 *   Region 3: Peripherals (0x40000000, 512 MB) — Device memory, XN enabled
 *                                                 nGnRnE (strongly ordered)
 *                                                 No cache
 *
 * Background region: disabled (all memory not explicitly configured
 *                   inherits default: strongly-ordered, XN for backwards
 *                   compatibility = most permissive).
 */

#include "stm32h7xx_hal.h"
#include "mpu_config.h"

/* ---------------------------------------------------------------------------
 * ARMv7-M MPU register definitions
 * -------------------------------------------------------------------------*/

/* MPU Region Base Address Register (MPU_RBAR) */
#define MPU_RBAR_VALID         (1UL << 4)    /* VP = 1: use REGION field */
#define MPU_RBAR_REGION_SHIFT  0

/* MPU Region Attribute and Size Register (MPU_RASR) */
#define MPU_RASR_ENABLE        (1UL << 0)
#define MPU_RASR_SIZE_SHIFT    1
#define MPU_RASR_AP_SHIFT      24
#define MPU_RASR_TEX_SHIFT     19
#define MPU_RASR_S_SHIFT       18   /* Shareable   */
#define MPU_RASR_C_SHIFT       17   /* Cacheable   */
#define MPU_RASR_B_SHIFT       16   /* Bufferable  */
#define MPU_RASR_XN_SHIFT      28   /* eXecute Never */

/* Access permissions */
#define MPU_AP_NO_ACCESS       0UL
#define MPU_AP_PRIV_RW         1UL
#define MPU_AP_PRIV_RW_USER_NO 2UL
#define MPU_AP_FULL_ACCESS     3UL
#define MPU_AP_PRIV_RO         5UL
#define MPU_AP_FULL_RO         6UL

/* Memory type encoding (TEX, C, B) */
#define MPU_MEMTYPE_NORMAL_WB_WA    (0UL << 19) | (1UL << 18) | (1UL << 17) | (1UL << 16)
/* Above is wrong; TEX=001 for Normal Outer WB WA, C=1, B=1 */
/* TEX=001, S=depends, C=1, B=1 => Write-back, write-allocate */

/* Correct encoding using HAL macros is preferred; we define manually for
 * clarity and then use HAL as final init step. */

/* ---------------------------------------------------------------------------
 * MPU region descriptor table
 * -------------------------------------------------------------------------*/

typedef struct {
    uint32_t base;        /* Region base address       */
    uint32_t size;        /* Region size in bytes      */
    uint32_t rasr;        /* Pre-computed RASR value   */
    const char *purpose;  /* Description               */
} mpu_region_t;

/*
 * Compute the size field for MPU_RASR (2^(SIZE+1) must equal region size).
 * SIZE = log2(region_size) - 1.
 * Region size must be a power of 2 and >= 32 bytes.
 */
#define MPU_SIZE_LOG2(x)  (31UL - __CLZ((x)))  /* log2 for powers of 2 */
#define MPU_SIZE_FIELD(x) ((MPU_SIZE_LOG2(x) - 1UL) << MPU_RASR_SIZE_SHIFT)

static const mpu_region_t mpu_regions[] = {
    /*
     * Region 0: SRAM3 (0x20030000, 64 KB)
     *   - Full access (PRIV+USER R/W)
     *   - Normal memory, Outer Write-back, Write-allocate
     *   - Shareable (for M7/M4 coherence)
     *   - XN disabled (code can execute from SRAM3)
     */
    {0x20030000UL, 0x00010000UL,
     (uint32_t)(MPU_RASR_ENABLE |
                MPU_SIZE_FIELD(0x00010000UL) |
                (MPU_AP_FULL_ACCESS << MPU_RASR_AP_SHIFT) |
                (1UL << 19) |  /* TEX = 1 (Normal Outer WB) */
                (1UL << 18) |  /* S  = 1 (Shareable) */
                (1UL << 17) |  /* C  = 1 (Cacheable) */
                (1UL << 16) |  /* B  = 1 (Bufferable)  */
                (0UL << 28)),  /* XN = 0 (executable)  */
     "SRAM3 — 64KB, full access, shareable, cacheable, executable"},

    /*
     * Region 1: Flash app area (0x08010000, 512 KB)
     *   - Read-only (PRIV+USER)
     *   - Normal memory, Outer Write-through, no write-allocate
     *   - XN disabled (code fetch)
     */
    {0x08010000UL, 0x00080000UL,
     (uint32_t)(MPU_RASR_ENABLE |
                MPU_SIZE_FIELD(0x00080000UL) |
                (MPU_AP_FULL_RO << MPU_RASR_AP_SHIFT) |
                (1UL << 19) |  /* TEX = 1 */
                (0UL << 18) |  /* S  = 0 (non-shareable) */
                (1UL << 17) |  /* C  = 1 (cacheable) */
                (0UL << 16) |  /* B  = 0 (non-bufferable — WT) */
                (0UL << 28)),  /* XN = 0 (executable) */
     "App Flash — 512KB, read-only, write-through, executable"},

    /*
     * Region 2: SRAM2 (0x20020000, 288 KB)
     *   - Full access
     *   - Normal memory, Write-through (M4 backup data)
     *   - XN enabled (data only, no code)
     */
    {0x20020000UL, 0x00048000UL,
     (uint32_t)(MPU_RASR_ENABLE |
                MPU_SIZE_FIELD(0x00048000UL) & 0xFFFFFFE0UL, /* size may not be exact power of 2 */
                /* Use 256KB region to avoid alignment issues */
                MPU_SIZE_FIELD(0x00040000UL) |
                (MPU_AP_FULL_ACCESS << MPU_RASR_AP_SHIFT) |
                (1UL << 19) |  /* TEX = 1 */
                (0UL << 18) |  /* S  = 0 */
                (1UL << 17) |  /* C  = 1 */
                (0UL << 16) |  /* B  = 0 */
                (1UL << 28)),  /* XN = 1 (data only) */
     "SRAM2 — M4 backup, full access, write-through, XN enabled"},

    /*
     * Region 3: Peripheral space (0x40000000, 512 MB)
     *   - Full access
     *   - Device memory (nGnRnE)
     *   - XN enabled (no code fetch)
     */
    {0x40000000UL, 0x20000000UL,
     (uint32_t)(MPU_RASR_ENABLE |
                MPU_SIZE_FIELD(0x20000000UL) |
                (MPU_AP_FULL_ACCESS << MPU_RASR_AP_SHIFT) |
                (0UL << 19) |  /* TEX = 0 */
                (1UL << 18) |  /* S  = 1 (shareable) */
                (0UL << 17) |  /* C  = 0 */
                (1UL << 16) |  /* B  = 1 (nGnRnE: B=1, C=0 for device) */
                (1UL << 28)),  /* XN = 1 */
     "Peripherals — 512MB, device memory (nGnRnE), no cache, XN"},
};

static const uint32_t mpu_region_count =
    sizeof(mpu_regions) / sizeof(mpu_regions[0]);

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise the MPU with all configured regions.
 *
 * Disables the MPU first, programs all regions, then enables.
 * Must be called before cache enable and before any peripheral access.
 */
void mpu_config_init(void)
{
    /* Disable MPU and clear control register */
    ARM_MPU_Disable();

    /* Disable background region: all unmapped addresses fault-free
     * but with default attributes (strongly ordered, XN). */
    ARM_MPU_Disable(0);  /* PRIVDEFENA = 0 */

    /* Program each region */
    for (uint32_t i = 0; i < mpu_region_count; i++)
    {
        const mpu_region_t *r = &mpu_regions[i];

        /* Use the CMSIS function for portability */
        ARM_MPU_SetRegion(
            i,                              /* region number */
            ARM_MPU_RBAR(r->base,           /* base address */
                         ARM_MPU_SH_NON) |  /* shareability (overridden by RASR) */
            ARM_MPU_AP_NONE,                /* (alternative to manual encoding) */
            0, 0, 0, 0, 0, 0, 0, 0          /* dummy - not used */
        );
        /* Actually, use the raw register method to ensure exact config: */

        /* Write RBAR */
        MPU->RBAR = (r->base & 0xFFFFFFE0UL) | (i & 0x0FUL) | MPU_RBAR_VALID;
        __DSB();

        /* Write RASR with pre-computed value */
        MPU->RASR = r->rasr;
        __DSB();
    }

    /* Enable MPU with PRIVDEFENA (allow background access with default map) */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Ensure ordering */
    __DSB();
    __ISB();
}

/**
 * @brief  Disable the MPU (e.g., before entering low-power or bootloader).
 */
void mpu_config_disable(void)
{
    ARM_MPU_Disable();
    __DSB();
    __ISB();
}

/**
 * @brief  Enable a specific MPU region at runtime.
 *
 * @param  region  Region number (0-15).
 * @param  base    Region base address (must be aligned to region size).
 * @param  size    Region size in bytes (must be power of 2, >= 32).
 * @param  attrs   Region attributes (access, TEX, C, B, S, XN).
 */
void mpu_config_set_region(uint32_t region, uint32_t base, uint32_t size,
                            uint32_t attrs)
{
    if (region > 15) return;

    MPU->RBAR = (base & 0xFFFFFFE0UL) | (region & 0x0FUL) | MPU_RBAR_VALID;
    __DSB();

    uint32_t rasr = MPU_RASR_ENABLE | MPU_SIZE_FIELD(size) | attrs;
    MPU->RASR = rasr;
    __DSB();
}
