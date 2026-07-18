/**
 * @file    anti_brick.c
 * @brief   Anti-brick protection with golden-image fallback
 * @details Bootloader-level safety net:
 *          1. Valid magic number at application entry
 *          2. Stack pointer within valid RAM range
 *          3. CRC32 of vector table (first 16 entries)
 *          If any check fails, the golden image in write-protected flash
 *          is loaded and the cloud is notified.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */
#define ANTI_BRICK_MAGIC        0xDEADBEEFu    /* expected at app entry */
#define APP_START_ADDRESS       0x08020000u    /* application start (flash) */
#define APP_VECTOR_TABLE_SIZE   16             /* first 16 exception vectors */
#define STACK_PTR_MIN           0x20000000u    /* RAM start */
#define STACK_PTR_MAX           0x20040000u    /* RAM end (256KB) */

#define GOLDEN_IMAGE_ADDRESS    0x08000000u    /* golden image in bootloader sector */
#define GOLDEN_IMAGE_MAX_SIZE   (128 * 1024)   /* 128KB max golden image */

#define NOTIFY_BUF_SIZE         128

/* ------------------------------------------------------------------ */
/*  Register access macros (Cortex-M7 / Cortex-M4)                    */
/* ------------------------------------------------------------------ */
#define READ_REG(addr)          (*(volatile uint32_t *)(uintptr_t)(addr))
#define WRITE_REG(addr, val)    ((*(volatile uint32_t *)(uintptr_t)(addr)) = (uint32_t)(val))

/* ------------------------------------------------------------------ */
/*  Forward declarations                                               */
/* ------------------------------------------------------------------ */
static uint32_t anti_crc32(const void *data, size_t len);
static bool     is_stack_valid(uint32_t sp);
static bool     is_vector_crc_valid(uint32_t base_addr);
static bool     is_app_magic_valid(uint32_t base_addr);
static int      anti_notify(const char *msg);
static void     jump_to_image(uint32_t address);

/* ------------------------------------------------------------------ */
/*  CRC32                                                              */
/* ------------------------------------------------------------------ */
static uint32_t anti_crc32(const void *data, size_t len)
{
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t crc = i;
            for (int j = 0; j < 8; j++)
                crc = (crc >> 1) ^ (crc & 1 ? 0xEDB88320u : 0);
            table[i] = crc;
        }
        init = true;
    }
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *buf = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++)
        crc = table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

/* ------------------------------------------------------------------ */
/*  Checks                                                             */
/* ------------------------------------------------------------------ */

/**
 * Check that the initial stack pointer (first word in vector table)
 * points into valid RAM.
 */
bool is_stack_valid(uint32_t base_addr)
{
    uint32_t sp = READ_REG(base_addr);

    if (sp < STACK_PTR_MIN || sp >= STACK_PTR_MAX) {
        fprintf(stderr, "[ANTI-BRICK] stack pointer 0x%08X out of range "
                "[0x%08X, 0x%08X)\n", sp, STACK_PTR_MIN, STACK_PTR_MAX);
        return false;
    }

    /* Stack should be aligned to 8 bytes per AAPCS */
    if (sp & 7) {
        fprintf(stderr, "[ANTI-BRICK] stack pointer 0x%08X not 8-byte aligned\n", sp);
        return false;
    }

    return true;
}

/**
 * Verify CRC32 of the first APP_VECTOR_TABLE_SIZE words of the vector table.
 * The stored CRC is expected at word offset 16 (immediately after the
 * standard vector table entries).
 */
bool is_vector_crc_valid(uint32_t base_addr)
{
    /* Compute CRC of first 16 vector entries (64 bytes) */
    uint32_t computed_crc = anti_crc32(
        (const void *)(uintptr_t)base_addr,
        APP_VECTOR_TABLE_SIZE * sizeof(uint32_t));

    /* Stored CRC is at offset 16 words (64 bytes) into the image */
    uint32_t stored_crc = READ_REG(base_addr + APP_VECTOR_TABLE_SIZE * sizeof(uint32_t));

    if (computed_crc != stored_crc) {
        fprintf(stderr, "[ANTI-BRICK] vector table CRC mismatch: "
                "computed=0x%08X stored=0x%08X\n", computed_crc, stored_crc);
        return false;
    }

    return true;
}

/**
 * Check for valid application magic number.
 * The magic is stored at a known offset in the application header
 * (e.g., at offset 0x100 from the application base).
 */
bool is_app_magic_valid(uint32_t base_addr)
{
    uint32_t magic_offset = 0x100;  /* offset where magic is stored */
    uint32_t magic = READ_REG(base_addr + magic_offset);

    if (magic != ANTI_BRICK_MAGIC) {
        fprintf(stderr, "[ANTI-BRICK] magic mismatch at 0x%08X+0x%04X: "
                "expected 0x%08X, got 0x%08X\n",
                base_addr, magic_offset, ANTI_BRICK_MAGIC, magic);
        return false;
    }

    return true;
}

/* ------------------------------------------------------------------ */
/*  Notify (stub — integrate with cloud reporting / watchdog)          */
/* ------------------------------------------------------------------ */
int anti_notify(const char *msg)
{
    /* In production: POST to cloud endpoint, log to persistent storage,
     * blink LED pattern for visual indicator.
     */
    fprintf(stderr, "[ANTI-BRICK] %s\n", msg ? msg : "(null)");
    (void)msg;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Jump to image                                                      */
/* ------------------------------------------------------------------ */

/**
 * jump_to_image - set stack pointer and jump to reset handler
 */
void jump_to_image(uint32_t address)
{
    /* Cortex-M: vector table at 'address', first entry = SP, second = Reset */
    uint32_t sp    = READ_REG(address);
    uint32_t reset = READ_REG(address + 4);

    /* Set new vector table offset */
    __asm volatile (
        "MSR    MSP, %0\n"         /* set main stack pointer */
        "DSB\n"
        "ISB\n"
        "BX     %1\n"              /* branch to reset handler */
        :
        : "r" (sp), "r" (reset)
        : "memory"
    );

    /* Never reached */
    __builtin_unreachable();
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * anti_brick_check - run all pre-boot safety checks
 *
 * Returns:
 *   0  if all checks pass
 *   1  if golden image fallback was triggered
 *  -1  on critical error (both app and golden image corrupted)
 */
int anti_brick_check(void)
{
    fprintf(stderr, "[ANTI-BRICK] running pre-boot safety checks\n");

    /* Check 1: Valid application magic */
    if (!is_app_magic_valid(APP_START_ADDRESS)) {
        anti_notify("anti-brick: app magic invalid");
        goto fallback;
    }

    /* Check 2: Valid stack pointer */
    if (!is_stack_valid(APP_START_ADDRESS)) {
        anti_notify("anti-brick: stack pointer invalid");
        goto fallback;
    }

    /* Check 3: Vector table CRC */
    if (!is_vector_crc_valid(APP_START_ADDRESS)) {
        anti_notify("anti-brick: vector table CRC invalid");
        goto fallback;
    }

    /* All checks passed — boot normally */
    fprintf(stderr, "[ANTI-BRICK] all checks PASSED, booting application\n");
    return 0;

fallback:
    /* Attempt golden image */
    fprintf(stderr, "[ANTI-BRICK] checks FAILED, attempting golden image\n");

    if (!is_app_magic_valid(GOLDEN_IMAGE_ADDRESS)) {
        anti_notify("anti-brick CRITICAL: golden image also corrupted");
        return -1;
    }

    if (!is_stack_valid(GOLDEN_IMAGE_ADDRESS)) {
        anti_notify("anti-brick CRITICAL: golden image stack invalid");
        return -1;
    }

    anti_notify("anti-brick: loading golden image");
    jump_to_image(GOLDEN_IMAGE_ADDRESS);
    return 1; /* never reached if jump succeeds */
}

/**
 * anti_brick_install_golden - copy current running image to golden region
 *                              (called only from trusted boot context)
 *
 * This should only be invoked after successful OTA verification to update
 * the golden image to the latest known-good version.
 */
int anti_brick_install_golden(const uint8_t *image, size_t size)
{
    if (!image || size == 0 || size > GOLDEN_IMAGE_MAX_SIZE)
        return -1;

    /* In production, this should:
     * 1. Unlock the write-protected flash sector (temporarily)
     * 2. Erase the golden sector
     * 3. Program the new golden image
     * 4. Re-lock the write-protected flash sector
     * 5. Verify CRC of the written data
     */

    fprintf(stderr, "[ANTI-BRICK] installing golden image: %zu bytes\n", size);

    /* STUB: actual flash programming depends on MCU HAL */
    /* HAL_FLASH_Unlock();
     * FLASH_Erase_Sector(GOLDEN_SECTOR);
     * for (size_t i = 0; i < size; i += 8) {
     *     HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
     *                       GOLDEN_IMAGE_ADDRESS + i,
     *                       *(uint64_t *)(image + i));
     * }
     * HAL_FLASH_Lock();
     */

    /* Verify */
    uint32_t golden_crc = anti_crc32(image, size);
    uint32_t written_crc = anti_crc32(
        (const void *)(uintptr_t)GOLDEN_IMAGE_ADDRESS, size);

    if (golden_crc != written_crc) {
        anti_notify("golden image install: CRC verify failed");
        return -1;
    }

    anti_notify("golden image installed successfully");
    return 0;
}

/**
 * anti_brick_get_status - return bitmask of check results
 */
uint32_t anti_brick_get_status(void)
{
    uint32_t status = 0;

    if (is_app_magic_valid(APP_START_ADDRESS))
        status |= (1 << 0);

    if (is_stack_valid(APP_START_ADDRESS))
        status |= (1 << 1);

    if (is_vector_crc_valid(APP_START_ADDRESS))
        status |= (1 << 2);

    if (is_app_magic_valid(GOLDEN_IMAGE_ADDRESS))
        status |= (1 << 3);

    return status;
}

/* ------------------------------------------------------------------ */
/*  Self-test                                                          */
/* ------------------------------------------------------------------ */
#ifdef TEST_ANTI_BRICK
int main(void)
{
    printf("Running anti-brick checks (will fail in test env):\n");
    int rc = anti_brick_check();
    printf("Result: %d\n", rc);
    printf("Status: 0x%08X\n", anti_brick_get_status());
    return 0;
}
#endif /* TEST_ANTI_BRICK */
