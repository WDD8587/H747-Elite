/**
 * @file    crash_dump.c
 * @brief   Crash dump analyzer for STM32H747.
 *
 * @details On a fault, the safety watchdog captures context registers to
 *          a well-known location in SRAM3. This module parses that context,
 *          decodes the fault type (UsageFault, BusFault, HardFault),
 *          extracts the PC address, looks up the function name in the
 *          symbol table (from .map file on RK3566 side), and logs a
 *          structured dump to /var/log/robot/crash.log with ISO 8601
 *          timestamps.
 *
 * Fault context layout in SRAM3 (0x20030000):
 *   Offset  | Size | Field
 *   --------+------+----------------------
 *   0x000   |  32  | r0..r7
 *   0x020   |  32  | r8..r15 (r12, sp, lr, pc)
 *   0x040   |   4  | xPSR
 *   0x044   |   4  | CFSR (Configurable Fault Status Register)
 *   0x048   |   4  | HFSR (HardFault Status Register)
 *   0x04C   |   4  | BFAR (BusFault Address Register)
 *   0x050   |   4  | MMFAR (MemManage Fault Address Register)
 *   0x054   |   4  | AFSR (Auxiliary Fault Status Register)
 *   0x058   |   4  | reserved (fault counter)
 *   0x05C   |   4  | magic (0xDEADBEEF if valid)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include "crash_dump.h"

/* ---------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------*/

#define CRASH_LOG_DIR        "/var/log/robot"
#define CRASH_LOG_PATH       CRASH_LOG_DIR "/crash.log"
#define MAX_SYMBOL_LINE      256
#define MAX_SYMBOLS          4096

#define CRASH_MAGIC          0xDEADBEEFUL

/* Fault context offset in SRAM3 (remote access via sysfs or /dev/mem) */
#define SRAM3_BASE           0x20030000UL
#define CRASH_CTX_OFFSET     0x0000F000UL  /* Offset within SRAM3 */

/* CSR flag bit positions */
#define CFSR_USGFAULT        (1UL << 16)   /* UsageFault asserted */
#define CFSR_BUSFAULT        (1UL << 15)   /* BusFault asserted  */
#define CFSR_MEMFAULT        (1UL << 14)   /* MemManage asserted */
#define CFSR_UNDEFINSTR      (1UL << 24)   /* Undefined instruction */
#define CFSR_INVSTATE        (1UL << 25)   /* Invalid state (EPSR) */
#define CFSR_INVPC           (1UL << 26)   /* Invalid PC */
#define CFSR_NOCPG           (1UL << 27)   /* No coprocessor */
#define CFSR_UNALIGNED       (1UL << 28)   /* Unaligned access */
#define CFSR_DIVBYZERO       (1UL << 25)   /* Divide by zero (with CCR.DIV_0_TRP) */
/* Note: BIT25 is shared; decode both INVSTATE and DIVBYZERO */

/* HFSR bits */
#define HFSR_DEBUGEVT        (1UL << 31)
#define HFSR_FORCED          (1UL << 30)   /* Forced HardFault (subtype in CFSR) */
#define HFSR_VECTBL          (1UL << 1)    /* Vector table read fault */

/* ---------------------------------------------------------------------------
 * Fault context structure (mirrors SRAM3 layout)
 * -------------------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    uint32_t r0_r7[8];       /* 0x000: General registers r0-r7 */
    uint32_t r8_r15[8];      /* 0x020: r8-r15 (r12, sp, lr, pc) */
    uint32_t xpsr;           /* 0x040 */
    uint32_t cfsr;           /* 0x044: Configurable Fault Status Reg */
    uint32_t hfsr;           /* 0x048: HardFault Status Reg */
    uint32_t bfar;           /* 0x04C: BusFault Address Reg */
    uint32_t mmfar;          /* 0x050: MemManage Fault Address Reg */
    uint32_t afsr;           /* 0x054: Auxiliary Fault Status Reg */
    uint32_t fault_count;    /* 0x058: reserved */
    uint32_t magic;          /* 0x05C: 0xDEADBEEF if valid */
} fault_ctx_t;

/* ---------------------------------------------------------------------------
 * Symbol table entry (parsed from .map file)
 * -------------------------------------------------------------------------*/
typedef struct {
    uint32_t addr;
    char     name[128];
} symbol_t;

static symbol_t symbols[MAX_SYMBOLS];
static int      num_symbols = 0;
static int      symbols_loaded = 0;

/* ---------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------*/

/**
 * @brief  Read fault context from SRAM3 via file-backed memory or /dev/mem.
 *
 * On RK3566, the STM32H747 SRAM is mapped via some memory window (e.g.,
 * /sys/kernel/debug/remoteproc/remoteproc0/mem). We attempt to read the
 * context from that file. If unavailable, we read from a crash dump file.
 *
 * @param  ctx  Output: fault context.
 * @return 0 on success, -1 on error.
 */
static int read_fault_context(fault_ctx_t *ctx)
{
    /*
     * Attempt to read via sysfs (remote-proc shared memory).
     * This is platform-specific; adjust for your RK3566 kernel config.
     */
    static const char *paths[] = {
        "/sys/kernel/debug/remoteproc/remoteproc0/mem",  /* rproc shared mem */
        "/tmp/crash_ctx.bin",                             /* fallback file   */
        NULL
    };

    for (int i = 0; paths[i] != NULL; i++) {
        FILE *fp = fopen(paths[i], "rb");
        if (fp == NULL) continue;

        /* Seek to crash context offset within the shared memory region */
        if (fseek(fp, (long)(CRASH_CTX_OFFSET), SEEK_SET) != 0) {
            fclose(fp);
            continue;
        }

        size_t n = fread(ctx, 1, sizeof(fault_ctx_t), fp);
        fclose(fp);

        if (n == sizeof(fault_ctx_t) && ctx->magic == CRASH_MAGIC) {
            return 0;  /* Valid context found */
        }
    }

    return -1;  /* No valid context */
}

/**
 * @brief  Clear the crash context (mark as consumed).
 */
static void clear_fault_context(void)
{
    /* Write magic = 0 at the context offset */
    static const char *paths[] = {
        "/sys/kernel/debug/remoteproc/remoteproc0/mem",
        "/tmp/crash_ctx.bin",
        NULL
    };

    uint32_t zero = 0;
    for (int i = 0; paths[i] != NULL; i++) {
        FILE *fp = fopen(paths[i], "r+b");
        if (fp == NULL) continue;

        if (fseek(fp, (long)(CRASH_CTX_OFFSET + 0x5C), SEEK_SET) == 0) {
            fwrite(&zero, sizeof(zero), 1, fp);
        }
        fclose(fp);
    }
}

/**
 * @brief  Load symbol table from .map file.
 *
 * @param  map_path  Path to the .map file (or symbol file).
 * @return 0 on success, -1 on error.
 */
static int load_symbols(const char *map_path)
{
    if (symbols_loaded) return 0;

    FILE *fp = fopen(map_path, "r");
    if (fp == NULL) {
        return -1;
    }

    char line[MAX_SYMBOL_LINE];
    num_symbols = 0;

    while (fgets(line, sizeof(line), fp) != NULL &&
           num_symbols < MAX_SYMBOLS)
    {
        /*
         * Expect lines like:
         *   0x08001234  main
         *   0x08005678  HAL_Delay
         *
         * Alternative: "    0x08001234                main"
         * Handle both formats.
         */
        uint32_t addr;
        char     name[128];

        if (sscanf(line, " 0x%x %127s", &addr, name) >= 2) {
            symbols[num_symbols].addr = addr;
            strncpy(symbols[num_symbols].name, name,
                    sizeof(symbols[num_symbols].name) - 1);
            symbols[num_symbols].name[sizeof(symbols[num_symbols].name) - 1] = '\0';
            num_symbols++;
        } else if (sscanf(line, "%x %127s", &addr, name) >= 2) {
            /* Try without leading 0x prefix */
            symbols[num_symbols].addr = addr;
            strncpy(symbols[num_symbols].name, name,
                    sizeof(symbols[num_symbols].name) - 1);
            symbols[num_symbols].name[sizeof(symbols[num_symbols].name) - 1] = '\0';
            num_symbols++;
        }
    }

    fclose(fp);
    symbols_loaded = 1;
    return 0;
}

/**
 * @brief  Lookup a function name by address.
 *
 * @param  pc  Program counter value.
 * @return Function name string, or "???" if not found.
 */
static const char *lookup_symbol(uint32_t pc)
{
    if (!symbols_loaded) {
        return "???";
    }

    /* Find the closest symbol at or below PC */
    int best_idx = -1;
    for (int i = 0; i < num_symbols; i++) {
        if (symbols[i].addr <= pc) {
            if (best_idx < 0 ||
                symbols[i].addr > symbols[best_idx].addr) {
                best_idx = i;
            }
        }
    }

    if (best_idx >= 0) {
        return symbols[best_idx].name;
    }

    return "???";
}

/**
 * @brief  Format an ISO 8601 timestamp string.
 *
 * @param  buf   Output buffer (must be >= 32 bytes).
 * @param  len   Buffer length.
 */
static void iso_timestamp(char *buf, size_t len)
{
    time_t now = time(NULL);
    struct tm *tm = gmtime(&now);

    if (tm == NULL) {
        snprintf(buf, len, "1970-01-01T00:00:00Z");
        return;
    }

    strftime(buf, len, "%Y-%m-%dT%H:%M:%SZ", tm);
}

/**
 * @brief  Ensure the log directory exists.
 *
 * @return 0 on success, -1 on error.
 */
static int ensure_log_dir(void)
{
    struct stat st;
    if (stat(CRASH_LOG_DIR, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return 0;
        }
        /* Exists but not a directory — remove it */
        unlink(CRASH_LOG_DIR);
    }

    return mkdir(CRASH_LOG_DIR, 0755);
}

/**
 * @brief  Decode CFSR into a human-readable fault description.
 *
 * @param  cfsr  CFSR value.
 * @param  buf   Output buffer.
 * @param  len   Buffer length.
 */
static void decode_cfsr(uint32_t cfsr, char *buf, size_t len)
{
    const char *desc = "Unknown fault";

    if (cfsr & CFSR_USGFAULT) {
        /* UsageFault */
        if (cfsr & CFSR_DIVBYZERO) {
            desc = "UsageFault: Divide by zero";
        } else if (cfsr & CFSR_UNALIGNED) {
            desc = "UsageFault: Unaligned memory access";
        } else if (cfsr & CFSR_INVPC) {
            desc = "UsageFault: Invalid PC load (EXC_RETURN)";
        } else if (cfsr & CFSR_INVSTATE) {
            desc = "UsageFault: Invalid state (EPSR, not Thumb)";
        } else if (cfsr & CFSR_UNDEFINSTR) {
            desc = "UsageFault: Undefined instruction";
        } else if (cfsr & CFSR_NOCPG) {
            desc = "UsageFault: No coprocessor";
        } else {
            desc = "UsageFault";
        }
    } else if (cfsr & CFSR_BUSFAULT) {
        desc = "BusFault: Data bus error";
    } else if (cfsr & CFSR_MEMFAULT) {
        desc = "MemManage: MPU violation";
    }

    snprintf(buf, len, "%s (CFSR=0x%08X)", desc, (unsigned int)cfsr);
}

/**
 * @brief  Decode HFSR into a human-readable string.
 *
 * @param  hfsr  HFSR value.
 * @param  buf   Output buffer.
 * @param  len   Buffer length.
 */
static void decode_hfsr(uint32_t hfsr, char *buf, size_t len)
{
    const char *desc = "HardFault";

    if (hfsr & HFSR_FORCED) {
        desc = "HardFault (forced): Configurable fault escalated";
    } else if (hfsr & HFSR_DEBUGEVT) {
        desc = "HardFault: Debug event";
    } else if (hfsr & HFSR_VECTBL) {
        desc = "HardFault: Vector table read error";
    } else {
        desc = "HardFault";
    }

    snprintf(buf, len, "%s (HFSR=0x%08X)", desc, (unsigned int)hfsr);
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Analyse a crash dump from the last fault context.
 *
 * Reads fault context from SRAM3 (via sysfs rproc shared memory),
 * decodes it, looks up the faulting function, and writes a structured
 * log entry to /var/log/robot/crash.log.
 *
 * @param  map_path  Path to the firmware .map file (can be NULL to skip
 *                   symbol lookup).
 * @return 0 on success, -1 if no valid crash context found.
 */
int crash_dump_analyse(const char *map_path)
{
    fault_ctx_t ctx;
    char        timestamp[64];
    char        cfsr_desc[128];
    char        hfsr_desc[128];

    /* Read fault context from remote processor memory */
    if (read_fault_context(&ctx) != 0) {
        return -1;  /* No crash context available */
    }

    /* Decode timestamps */
    iso_timestamp(timestamp, sizeof(timestamp));

    /* Decode fault type */
    decode_cfsr(ctx.cfsr, cfsr_desc, sizeof(cfsr_desc));
    decode_hfsr(ctx.hfsr, hfsr_desc, sizeof(hfsr_desc));

    /* Load symbols if map file provided */
    if (map_path != NULL) {
        load_symbols(map_path);
    }

    /* Extract PC (r15 stored at r8_r15[7]) */
    uint32_t pc = ctx.r8_r15[7];
    const char *func_name = lookup_symbol(pc);

    /* Extract LR (r14) and SP (r13) */
    uint32_t lr = ctx.r8_r15[6];
    uint32_t sp = ctx.r8_r15[4];

    /* Extract key registers */
    uint32_t r0 = ctx.r0_r7[0];
    uint32_t r1 = ctx.r0_r7[1];
    uint32_t r2 = ctx.r0_r7[2];
    uint32_t r3 = ctx.r0_r7[3];
    uint32_t r12 = ctx.r8_r15[0];

    /* Ensure log directory exists */
    ensure_log_dir();

    /* Write crash log */
    FILE *fp = fopen(CRASH_LOG_PATH, "a");
    if (fp == NULL) {
        fprintf(stderr, "crash_dump: cannot open %s: %s\n",
                CRASH_LOG_PATH, strerror(errno));
        return -1;
    }

    fprintf(fp, "=== CRASH DUMP === %s ===\n", timestamp);
    fprintf(fp, "Fault:     %s\n", cfsr_desc);
    fprintf(fp, "HardFault: %s\n", hfsr_desc);
    fprintf(fp, "PC:        0x%08X (%s)\n", (unsigned int)pc, func_name);
    fprintf(fp, "LR:        0x%08X\n", (unsigned int)lr);
    fprintf(fp, "SP:        0x%08X\n", (unsigned int)sp);
    fprintf(fp, "xPSR:      0x%08X\n", (unsigned int)ctx.xpsr);
    fprintf(fp, "R0:        0x%08X\n", (unsigned int)r0);
    fprintf(fp, "R1:        0x%08X\n", (unsigned int)r1);
    fprintf(fp, "R2:        0x%08X\n", (unsigned int)r2);
    fprintf(fp, "R3:        0x%08X\n", (unsigned int)r3);
    fprintf(fp, "R12:       0x%08X\n", (unsigned int)r12);
    fprintf(fp, "BFAR:      0x%08X\n", (unsigned int)ctx.bfar);
    fprintf(fp, "MMFAR:     0x%08X\n", (unsigned int)ctx.mmfar);
    fprintf(fp, "Fault cnt: %u\n", (unsigned int)ctx.fault_count);
    fprintf(fp, "==============================\n");
    fclose(fp);

    /* Clear the context so we don't re-analyse */
    clear_fault_context();

    return 0;
}

/**
 * @brief  Check if a crash dump is available (without consuming).
 *
 * @return 1 if crash data is pending, 0 otherwise.
 */
int crash_dump_available(void)
{
    fault_ctx_t ctx;
    if (read_fault_context(&ctx) == 0) {
        return 1;
    }
    return 0;
}

/**
 * @brief  Force-clear any pending crash dump.
 */
void crash_dump_clear(void)
{
    clear_fault_context();
}
