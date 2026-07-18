/**
 * @file    ab_partition.c
 * @brief   A/B partition manager with fail-count tracking
 * @details Manages two firmware slots (A and B).  Tracks:
 *          - Current active slot (boot source)
 *          - Boot attempt count (failed boots increment counter)
 *          - Last known good boot
 *          - Automatic failover after 3 consecutive failures
 *          - Cloud notification on slot switch
 *
 *          Slot metadata stored in a small persistent block on a
 *          dedicated partition (e.g., /dev/mmcblk0p7).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */
#define AB_METADATA_MAGIC       "AB01"
#define AB_METADATA_VERSION     1
#define AB_METADATA_PATH        "/data/ota/ab_metadata.bin"
#define AB_MAX_FAIL_COUNT       3
#define AB_SLOT_COUNT           2
#define AB_FIRMWARE_PART_A      "/dev/mmcblk0p5"
#define AB_FIRMWARE_PART_B      "/dev/mmcblk0p6"

/* ------------------------------------------------------------------ */
/*  Slot identifiers                                                   */
/* ------------------------------------------------------------------ */
typedef enum {
    AB_SLOT_INVALID = -1,
    AB_SLOT_A       = 0,
    AB_SLOT_B       = 1
} ab_slot_t;

static const char *ab_slot_name(ab_slot_t s)
{
    return (s == AB_SLOT_A) ? "A" : (s == AB_SLOT_B) ? "B" : "INVALID";
}

/* ------------------------------------------------------------------ */
/*  Metadata structure (persistent)                                    */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    char     magic[4];            /* "AB01" */
    uint32_t version;             /* metadata version */
    uint32_t active_slot;         /* current active slot (0=A, 1=B) */
    uint32_t boot_count;          /* total boot attempts for active slot */
    uint32_t fail_count;          /* consecutive fail count for active slot */
    uint32_t last_known_good_slot;/* slot that last booted successfully */
    uint64_t last_known_good_timestamp; /* unix timestamp of LKG boot */
    uint32_t boot_ok_count;       /* total successful boots on active slot */
    uint32_t boot_total;          /* total boot attempts ever */
    uint32_t slot_a_boot_count;   /* total boots on slot A */
    uint32_t slot_b_boot_count;   /* total boots on slot B */
    uint32_t slot_a_fail_count;   /* total failures on slot A */
    uint32_t slot_b_fail_count;   /* total failures on slot B */
    uint32_t crc32;               /* metadata integrity check */
    uint8_t  reserved[64];        /* future use */
} ab_metadata_t;

_Static_assert(sizeof(ab_metadata_t) == 128, "ab_metadata_t size mismatch");

/* ------------------------------------------------------------------ */
/*  Global state                                                       */
/* ------------------------------------------------------------------ */
static ab_metadata_t g_meta;
static bool g_initialized = false;
static bool g_meta_dirty = false;

/* ------------------------------------------------------------------ */
/*  CRC32 helper                                                       */
/* ------------------------------------------------------------------ */
static uint32_t ab_crc32(const void *data, size_t len)
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

/* Compute CRC of metadata excluding the crc32 field itself */
static uint32_t ab_meta_compute_crc(const ab_metadata_t *m)
{
    return ab_crc32(m, sizeof(ab_metadata_t) - sizeof(uint32_t));
}

/* ------------------------------------------------------------------ */
/*  Metadata persistence                                               */
/* ------------------------------------------------------------------ */
static int ab_meta_save(void)
{
    g_meta.crc32 = ab_meta_compute_crc(&g_meta);

    FILE *f = fopen(AB_METADATA_PATH, "wb");
    if (!f) return -EIO;

    size_t written = fwrite(&g_meta, 1, sizeof(ab_metadata_t), f);
    fclose(f);
    if (written != sizeof(ab_metadata_t)) return -EIO;

    g_meta_dirty = false;
    return 0;
}

static int ab_meta_load(void)
{
    FILE *f = fopen(AB_METADATA_PATH, "rb");
    if (!f) {
        /* First boot: initialize with defaults */
        memset(&g_meta, 0, sizeof(g_meta));
        memcpy(g_meta.magic, AB_METADATA_MAGIC, 4);
        g_meta.version = AB_METADATA_VERSION;
        g_meta.active_slot = 0;
        g_meta.last_known_good_slot = 0;
        g_meta.crc32 = ab_meta_compute_crc(&g_meta);
        ab_meta_save();
        return 0;
    }

    size_t n = fread(&g_meta, 1, sizeof(ab_metadata_t), f);
    fclose(f);

    if (n != sizeof(ab_metadata_t))
        return -EIO;

    /* Validate magic */
    if (memcmp(g_meta.magic, AB_METADATA_MAGIC, 4) != 0)
        return -EINVAL;

    /* Validate CRC */
    uint32_t expected = g_meta.crc32;
    uint32_t computed = ab_meta_compute_crc(&g_meta);
    if (expected != computed)
        return -EINVAL;

    return 0;
}

/* ------------------------------------------------------------------ */
/*  Error notifier (stub — integrate with cloud reporting)             */
/* ------------------------------------------------------------------ */
static void ab_notify(const char *fmt, ...)
{
    /* Stub: send alert to cloud. */
    (void)fmt;
    fprintf(stderr, "[AB] notification stub\n");
}

/* ------------------------------------------------------------------ */
/*  API                                                                */
/* ------------------------------------------------------------------ */

/**
 * ab_init - load metadata from persistent storage
 */
int ab_init(void)
{
    if (g_initialized) return 0;

    int rc = ab_meta_load();
    if (rc != 0) {
        /* Corrupt metadata: reset to defaults */
        fprintf(stderr, "[AB] metadata corrupt, resetting\n");
        memset(&g_meta, 0, sizeof(g_meta));
        memcpy(g_meta.magic, AB_METADATA_MAGIC, 4);
        g_meta.version = AB_METADATA_VERSION;
        g_meta.active_slot = 0;
        g_meta.last_known_good_slot = 0;
        g_meta.crc32 = ab_meta_compute_crc(&g_meta);
        ab_meta_save();
        ab_notify("AB metadata reset due to corruption");
    }

    g_initialized = true;
    return 0;
}

/**
 * ab_get_active_slot - return the currently active slot
 */
ab_slot_t ab_get_active_slot(void)
{
    if (!g_initialized) ab_init();
    return (ab_slot_t)g_meta.active_slot;
}

/**
 * ab_get_inactive_slot - return the slot that is NOT active
 */
ab_slot_t ab_get_inactive_slot(void)
{
    ab_slot_t active = ab_get_active_slot();
    return (active == AB_SLOT_A) ? AB_SLOT_B : AB_SLOT_A;
}

/**
 * ab_get_partition_for_slot - get device path for a slot
 */
const char *ab_get_partition_for_slot(ab_slot_t slot)
{
    switch (slot) {
    case AB_SLOT_A: return AB_FIRMWARE_PART_A;
    case AB_SLOT_B: return AB_FIRMWARE_PART_B;
    default:        return NULL;
    }
}

/**
 * ab_boot_attempt - record a boot attempt on the current active slot.
 *                   Called early in boot sequence (before application init).
 */
int ab_boot_attempt(void)
{
    if (!g_initialized) ab_init();

    g_meta.boot_count++;
    g_meta.boot_total++;

    if (g_meta.active_slot == AB_SLOT_A)
        g_meta.slot_a_boot_count++;
    else
        g_meta.slot_b_boot_count++;

    g_meta_dirty = true;
    ab_meta_save();
    return 0;
}

/**
 * ab_boot_succeeded - mark current slot as successfully booted.
 *                     Called after application init completes.
 */
int ab_boot_succeeded(void)
{
    if (!g_initialized) ab_init();

    g_meta.fail_count = 0;
    g_meta.last_known_good_slot = g_meta.active_slot;
    g_meta.last_known_good_timestamp = (uint64_t)time(NULL);
    g_meta.boot_ok_count++;
    g_meta_dirty = true;
    ab_meta_save();
    return 0;
}

/**
 * ab_boot_failed - increment fail count for active slot.
 *                  If fail count exceeds threshold, mark slot bad and
 *                  switch to other slot.
 *
 * Returns:
 *   0 if slot still considered healthy
 *   1 if slot was marked bad and switched to other slot
 *  -1 on error
 */
int ab_boot_failed(void)
{
    if (!g_initialized) ab_init();

    g_meta.fail_count++;

    if (g_meta.active_slot == AB_SLOT_A)
        g_meta.slot_a_fail_count++;
    else
        g_meta.slot_b_fail_count++;

    fprintf(stderr, "[AB] boot FAILED on slot %s (fail_count=%u/%u)\n",
            ab_slot_name((ab_slot_t)g_meta.active_slot),
            g_meta.fail_count, AB_MAX_FAIL_COUNT);

    if (g_meta.fail_count >= AB_MAX_FAIL_COUNT) {
        /* Mark slot as bad and switch */
        ab_slot_t bad_slot = (ab_slot_t)g_meta.active_slot;

        /* Switch to the other slot */
        g_meta.active_slot = (g_meta.active_slot == AB_SLOT_A) ? AB_SLOT_B : AB_SLOT_A;
        g_meta.fail_count = 0;
        g_meta.boot_count = 0;

        ab_meta_save();

        ab_notify("Slot %s marked BAD after %u failures, switching to slot %s",
                  ab_slot_name(bad_slot), AB_MAX_FAIL_COUNT,
                  ab_slot_name((ab_slot_t)g_meta.active_slot));

        return 1; /* switched slot */
    }

    ab_meta_save();
    return 0; /* still on same slot */
}

/**
 * ab_switch_slot - manually switch active slot
 */
int ab_switch_slot(void)
{
    if (!g_initialized) ab_init();

    ab_slot_t prev = (ab_slot_t)g_meta.active_slot;
    g_meta.active_slot = (g_meta.active_slot == AB_SLOT_A) ? AB_SLOT_B : AB_SLOT_A;
    g_meta.fail_count = 0;
    g_meta.boot_count = 0;
    g_meta_dirty = true;
    ab_meta_save();

    ab_notify("Manual slot switch: %s -> %s",
              ab_slot_name(prev), ab_slot_name((ab_slot_t)g_meta.active_slot));
    return 0;
}

/**
 * ab_mark_bad - explicitly mark the specified slot as bad
 */
int ab_mark_bad(ab_slot_t slot)
{
    if (!g_initialized) ab_init();
    if (slot < AB_SLOT_A || slot >= AB_SLOT_COUNT)
        return -EINVAL;

    if (slot == g_meta.active_slot) {
        /* Marking active slot as bad — switch to other */
        ab_slot_t new_slot = ab_get_inactive_slot();
        g_meta.active_slot = new_slot;
        g_meta.fail_count = 0;
        g_meta.boot_count = 0;
    }

    ab_meta_save();
    ab_notify("Slot %s explicitly marked BAD", ab_slot_name(slot));
    return 0;
}

/**
 * ab_get_info - retrieve metadata info for diagnostics
 */
int ab_get_info(ab_metadata_t *info)
{
    if (!g_initialized) ab_init();
    if (!info) return -EINVAL;
    memcpy(info, &g_meta, sizeof(ab_metadata_t));
    return 0;
}

/**
 * ab_is_slot_bad - check if a slot is considered bad
 *                  (heuristic: high fail count)
 */
bool ab_is_slot_bad(ab_slot_t slot)
{
    if (!g_initialized) ab_init();
    if (slot < AB_SLOT_A || slot >= AB_SLOT_COUNT)
        return true;

    if (slot == AB_SLOT_A)
        return g_meta.slot_a_fail_count >= AB_MAX_FAIL_COUNT;
    else
        return g_meta.slot_b_fail_count >= AB_MAX_FAIL_COUNT;
}

/**
 * ab_get_fail_count - get consecutive fail count for current slot
 */
uint32_t ab_get_fail_count(void)
{
    if (!g_initialized) ab_init();
    return g_meta.fail_count;
}

/**
 * ab_get_boot_count - get total boot count for current slot
 */
uint32_t ab_get_boot_count(void)
{
    if (!g_initialized) ab_init();
    return g_meta.boot_count;
}

/**
 * ab_get_lkg_slot - get the last known good slot
 */
ab_slot_t ab_get_lkg_slot(void)
{
    if (!g_initialized) ab_init();
    return (ab_slot_t)g_meta.last_known_good_slot;
}

/* ------------------------------------------------------------------ */
/*  Self-test                                                          */
/* ------------------------------------------------------------------ */
#ifdef TEST_AB
int main(void)
{
    ab_init();

    printf("Active slot: %s\n", ab_slot_name(ab_get_active_slot()));
    printf("Inactive slot: %s\n", ab_slot_name(ab_get_inactive_slot()));

    /* Simulate boot sequence */
    ab_boot_attempt();
    printf("Boot count: %u\n", ab_get_boot_count());

    /* Simulate a failed boot */
    ab_boot_failed();
    printf("Fail count after 1 fail: %u\n", ab_get_fail_count());

    /* Simulate enough failures to trigger slot switch */
    ab_boot_failed();
    ab_boot_failed();
    printf("After 3 fails, active slot: %s\n", ab_slot_name(ab_get_active_slot()));
    printf("Fail count after switch: %u\n", ab_get_fail_count());

    /* Boot and succeed */
    ab_boot_attempt();
    ab_boot_succeeded();
    printf("After success, LKG slot: %s\n", ab_slot_name(ab_get_lkg_slot()));

    return 0;
}
#endif /* TEST_AB */
