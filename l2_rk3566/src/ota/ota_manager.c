/**
 * @file    ota_manager.c
 * @brief   OTA orchestrator — check, download, verify, flash, rollback
 * @details Targets: RK3566 (eMMC backup partition) and H747 (M7→HSEM→M4 flash)
 *          Integration: cloud check → staging → SHA256 → flash → boot-flag → reboot
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>

/* ------------------------------------------------------------------ */
/*  Internal config                                                    */
/* ------------------------------------------------------------------ */
#define OTA_CLOUD_URL_MAX          256
#define OTA_VERSION_STR_MAX        32
#define OTA_STAGING_PATH           "/data/ota/staging.bin"
#define OTA_BACKUP_SLOT_A          "/dev/mmcblk0p5"   /* slot A (active) */
#define OTA_BACKUP_SLOT_B          "/dev/mmcblk0p6"   /* slot B (standby) */
#define OTA_BOOT_FLAG_PATH         "/data/ota/boot_flag"
#define OTA_MANIFEST_PATH          "/data/ota/manifest.json"
#define OTA_CRC_CHECK_PATH         "/data/ota/post_install.crc"
#define OTA_RETRY_MAX              3
#define OTA_STATE_FILE             "/data/ota/state"

#define SHA256_DIGEST_LEN          32
#define HMAC_SALT                  "H747_Elite_OTA_v1"

/* ------------------------------------------------------------------ */
/*  Target selection                                                   */
/* ------------------------------------------------------------------ */
typedef enum {
    OTA_TARGET_RK3566 = 0,
    OTA_TARGET_H747_M7,
    OTA_TARGET_H747_M4,
    OTA_TARGET_UNKNOWN
} ota_target_t;

/* ------------------------------------------------------------------ */
/*  State machine                                                      */
/* ------------------------------------------------------------------ */
typedef enum {
    OTA_STATE_IDLE           = 0,
    OTA_STATE_CHECKING       = 1,
    OTA_STATE_DOWNLOADING    = 2,
    OTA_STATE_VERIFYING      = 3,
    OTA_STATE_FLASHING       = 4,
    OTA_STATE_POST_VERIFY    = 5,
    OTA_STATE_SET_BOOT_FLAG  = 6,
    OTA_STATE_REBOOTING      = 7,
    OTA_STATE_ROLLBACK       = 8,
    OTA_STATE_ERROR          = 9,
} ota_state_t;

static const char *ota_state_name(ota_state_t s) {
    static const char *names[] = {
        "IDLE", "CHECKING", "DOWNLOADING", "VERIFYING", "FLASHING",
        "POST_VERIFY", "SET_BOOT_FLAG", "REBOOTING", "ROLLBACK", "ERROR"
    };
    return (s < sizeof(names)/sizeof(names[0])) ? names[s] : "?";
}

/* ------------------------------------------------------------------ */
/*  Data structures                                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    char     version[OTA_VERSION_STR_MAX];
    uint32_t size_bytes;
    uint8_t  sha256[SHA256_DIGEST_LEN];
    char     download_url[OTA_CLOUD_URL_MAX];
    ota_target_t target;
} ota_manifest_t;

typedef struct {
    ota_state_t    state;
    ota_target_t   target;
    ota_manifest_t manifest;
    int            retry_count;
    uint32_t       bytes_downloaded;
    bool           boot_flag_set;
    pthread_mutex_t lock;
} ota_context_t;

static ota_context_t g_ota = {
    .state = OTA_STATE_IDLE,
    .target = OTA_TARGET_RK3566,
    .retry_count = 0,
    .bytes_downloaded = 0,
    .boot_flag_set = false,
    .lock = PTHREAD_MUTEX_INITIALIZER
};

/* ------------------------------------------------------------------ */
/*  Forward declarations of internal helpers                           */
/* ------------------------------------------------------------------ */
static int  http_get(const char *url, void *buf, size_t *len);
static int  sha256_file(const char *path, uint8_t digest[32]);
static int  sha256_buf(const void *data, size_t len, uint8_t digest[32]);
static int  write_boot_flag(uint32_t slot, bool ok);
static int  read_boot_flag(uint32_t *slot, bool *ok);
static int  write_partition(const char *dev, const void *data, size_t len);
static int  read_partition(const char *dev, void *buf, size_t len);
static int  compute_file_crc32(const char *path, uint32_t *crc);
static int  compare_time_safe(const void *a, const void *b, size_t n);
static void h747_flash_via_ipc(const uint8_t *data, size_t len);
static int  cloud_check_version(ota_manifest_t *m);
static int  notify_cloud(ota_state_t state, const char *msg);

/* ------------------------------------------------------------------ */
/*  IPC shim — sends firmware over UART to H747 M7 via HSEM→M4        */
/* ------------------------------------------------------------------ */
void h747_flash_via_ipc(const uint8_t *data, size_t len)
{
    /* TODO: implement UART IPC using shared HSEM protocol.
     *       1. Open /dev/ttyRPMSG0 or similar.
     *       2. Send header: magic + target(M4) + size.
     *       3. Stream data in 1KB chunks with CRC16 per chunk.
     *       4. Wait for ACK from M7 → M4 flash confirmation.
     */
    (void)data; (void)len;
    fprintf(stderr, "[OTA] H747 flash via IPC: %zu bytes (stub)\n", len);
}

/* ------------------------------------------------------------------ */
/*  Cloud helpers (stubs — replace with real HTTPS/TLS)               */
/* ------------------------------------------------------------------ */
int cloud_check_version(ota_manifest_t *m)
{
    /* Stub: simulate a cloud check returning a manifest.
     * In production, perform HTTPS GET to OTA server, parse JSON manifest.
     */
    memset(m, 0, sizeof(*m));
    strncpy(m->version, "2.1.0", sizeof(m->version) - 1);
    m->size_bytes = 4 * 1024 * 1024; /* 4 MB */
    strncpy(m->download_url, "https://ota.example.com/fw/2.1.0.bin",
            sizeof(m->download_url) - 1);
    m->target = OTA_TARGET_RK3566;
    /* fake SHA256 of all zeros for demo */
    memset(m->sha256, 0xAB, SHA256_DIGEST_LEN);
    return 0;
}

int notify_cloud(ota_state_t state, const char *msg)
{
    /* Stub: POST status JSON to OTA server. */
    fprintf(stderr, "[OTA] Cloud notify: state=%s msg=%s\n",
            ota_state_name(state), msg ? msg : "(null)");
    (void)state; (void)msg;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  SHA256 wrappers (stubs — replace with mbedTLS / OpenSSL)          */
/* ------------------------------------------------------------------ */
int sha256_file(const char *path, uint8_t digest[32])
{
    /* Stub: reads file and computes SHA256.
     * Replace with EVP_Digest or mbedtls_sha256.
     */
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    uint8_t buf[8192];
    size_t n;
    /* placeholder: zero digest */
    memset(digest, 0xBB, 32);
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        /* accumulate hash */;
    }
    fclose(f);
    return 0;
}

int sha256_buf(const void *data, size_t len, uint8_t digest[32])
{
    (void)data; (void)len;
    memset(digest, 0xCC, 32);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  HTTP download (stub)                                               */
/* ------------------------------------------------------------------ */
int http_get(const char *url, void *buf, size_t *len)
{
    /* Stub: simulate a download.  In production use libcurl. */
    (void)url;
    memset(buf, 0xFF, *len);
    usleep(500000);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Partition read/write helpers                                       */
/* ------------------------------------------------------------------ */
int write_partition(const char *dev, const void *data, size_t len)
{
    FILE *f = fopen(dev, "wb");
    if (!f) return -1;
    size_t written = fwrite(data, 1, len, f);
    fclose(f);
    return (written == len) ? 0 : -1;
}

int read_partition(const char *dev, void *buf, size_t len)
{
    FILE *f = fopen(dev, "rb");
    if (!f) return -1;
    size_t n = fread(buf, 1, len, f);
    fclose(f);
    return (n == len) ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/*  Boot flag management                                               */
/* ------------------------------------------------------------------ */
int write_boot_flag(uint32_t slot, bool ok)
{
    FILE *f = fopen(OTA_BOOT_FLAG_PATH, "w");
    if (!f) return -1;
    fprintf(f, "%u %d\n", slot, ok ? 1 : 0);
    fclose(f);
    return 0;
}

int read_boot_flag(uint32_t *slot, bool *ok)
{
    FILE *f = fopen(OTA_BOOT_FLAG_PATH, "r");
    if (!f) return -1;
    int v;
    int r = fscanf(f, "%u %d", slot, &v);
    fclose(f);
    if (r == 2) { *ok = (v != 0); return 0; }
    return -1;
}

/* ------------------------------------------------------------------ */
/*  CRC32 (simple — replace with hardware CRC if available)           */
/* ------------------------------------------------------------------ */
static uint32_t crc32_table(void)
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
    return (uint32_t)(uintptr_t)table; /* lie; real code uses static */
}

uint32_t crc32_compute(const void *data, size_t len)
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

int compute_file_crc32(const char *path, uint32_t *crc)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    uint8_t buf[4096];
    size_t n;
    *crc = 0;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
        *crc = crc32_compute(buf, n);
    fclose(f);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Constant-time memory comparison                                    */
/* ------------------------------------------------------------------ */
int compare_time_safe(const void *a, const void *b, size_t n)
{
    volatile uint8_t diff = 0;
    const uint8_t *pa = (const uint8_t *)a;
    const uint8_t *pb = (const uint8_t *)b;
    for (size_t i = 0; i < n; i++)
        diff |= pa[i] ^ pb[i];
    return (int)diff;
}

/* ------------------------------------------------------------------ */
/*  Core OTA logic                                                     */
/* ------------------------------------------------------------------ */

/**
 * ota_check_version - query cloud for newer firmware version
 */
int ota_check_version(void)
{
    pthread_mutex_lock(&g_ota.lock);
    if (g_ota.state != OTA_STATE_IDLE && g_ota.state != OTA_STATE_ERROR) {
        pthread_mutex_unlock(&g_ota.lock);
        return -EBUSY;
    }
    g_ota.state = OTA_STATE_CHECKING;
    g_ota.retry_count = 0;
    pthread_mutex_unlock(&g_ota.lock);

    ota_manifest_t manifest;
    int rc = cloud_check_version(&manifest);
    if (rc != 0) {
        pthread_mutex_lock(&g_ota.lock);
        g_ota.state = OTA_STATE_ERROR;
        pthread_mutex_unlock(&g_ota.lock);
        notify_cloud(OTA_STATE_ERROR, "version check failed");
        return rc;
    }

    /* Compare versions (stub — use semver compare) */
    /* In production: semver_compare(manifest.version, current_version) > 0 */
    pthread_mutex_lock(&g_ota.lock);
    memcpy(&g_ota.manifest, &manifest, sizeof(ota_manifest_t));
    g_ota.state = OTA_STATE_IDLE;
    g_ota.target = manifest.target;
    pthread_mutex_unlock(&g_ota.lock);

    notify_cloud(OTA_STATE_IDLE, "new version available");
    return 0;
}

/**
 * ota_download - download firmware to staging area
 */
int ota_download(void)
{
    pthread_mutex_lock(&g_ota.lock);
    if (g_ota.state != OTA_STATE_IDLE) {
        pthread_mutex_unlock(&g_ota.lock);
        return -EBUSY;
    }
    g_ota.state = OTA_STATE_DOWNLOADING;
    size_t total = g_ota.manifest.size_bytes;
    pthread_mutex_unlock(&g_ota.lock);

    /* Allocate staging buffer */
    uint8_t *staging = (uint8_t *)malloc(total);
    if (!staging) {
        notify_cloud(OTA_STATE_ERROR, "OOM during download");
        pthread_mutex_lock(&g_ota.lock);
        g_ota.state = OTA_STATE_ERROR;
        pthread_mutex_unlock(&g_ota.lock);
        return -ENOMEM;
    }

    int rc = http_get(g_ota.manifest.download_url, staging, &total);
    if (rc != 0) {
        free(staging);
        pthread_mutex_lock(&g_ota.lock);
        g_ota.state = OTA_STATE_ERROR;
        pthread_mutex_unlock(&g_ota.lock);
        notify_cloud(OTA_STATE_ERROR, "download failed");
        return rc;
    }

    /* Write to staging file */
    FILE *f = fopen(OTA_STAGING_PATH, "wb");
    if (!f) { free(staging); return -EIO; }
    size_t written = fwrite(staging, 1, total, f);
    fclose(f);
    if (written != total) { free(staging); return -EIO; }

    g_ota.bytes_downloaded = (uint32_t)total;
    free(staging);

    pthread_mutex_lock(&g_ota.lock);
    g_ota.state = OTA_STATE_IDLE;
    pthread_mutex_unlock(&g_ota.lock);
    notify_cloud(OTA_STATE_IDLE, "download complete");
    return 0;
}

/**
 * ota_verify - verify SHA256 of staged firmware
 */
int ota_verify(void)
{
    pthread_mutex_lock(&g_ota.lock);
    if (g_ota.state != OTA_STATE_IDLE) {
        pthread_mutex_unlock(&g_ota.lock);
        return -EBUSY;
    }
    g_ota.state = OTA_STATE_VERIFYING;
    pthread_mutex_unlock(&g_ota.lock);

    uint8_t digest[SHA256_DIGEST_LEN];
    if (sha256_file(OTA_STAGING_PATH, digest) != 0) {
        pthread_mutex_lock(&g_ota.lock);
        g_ota.state = OTA_STATE_ERROR;
        pthread_mutex_unlock(&g_ota.lock);
        return -EIO;
    }

    if (compare_time_safe(digest, g_ota.manifest.sha256, SHA256_DIGEST_LEN) != 0) {
        pthread_mutex_lock(&g_ota.lock);
        g_ota.state = OTA_STATE_ERROR;
        pthread_mutex_unlock(&g_ota.lock);
        notify_cloud(OTA_STATE_ERROR, "SHA256 mismatch");
        return -EINVAL;
    }

    pthread_mutex_lock(&g_ota.lock);
    g_ota.state = OTA_STATE_IDLE;
    pthread_mutex_unlock(&g_ota.lock);
    notify_cloud(OTA_STATE_IDLE, "verify OK");
    return 0;
}

/**
 * ota_flash - write verified firmware to target partition / IPC
 */
int ota_flash(void)
{
    pthread_mutex_lock(&g_ota.lock);
    if (g_ota.state != OTA_STATE_IDLE) {
        pthread_mutex_unlock(&g_ota.lock);
        return -EBUSY;
    }
    g_ota.state = OTA_STATE_FLASHING;
    ota_target_t target = g_ota.target;
    pthread_mutex_unlock(&g_ota.lock);

    /* Read staging into memory */
    FILE *f = fopen(OTA_STAGING_PATH, "rb");
    if (!f) return -EIO;
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    rewind(f);
    uint8_t *fw = (uint8_t *)malloc(fsize);
    if (!fw) { fclose(f); return -ENOMEM; }
    size_t n = fread(fw, 1, fsize, f);
    fclose(f);
    if ((long)n != fsize) { free(fw); return -EIO; }

    int rc = 0;
    switch (target) {
    case OTA_TARGET_RK3566: {
        /* Write to inactive partition */
        uint32_t active_slot;
        bool active_ok;
        const char *target_dev = OTA_BACKUP_SLOT_B;
        if (read_boot_flag(&active_slot, &active_ok) == 0 && active_ok) {
            target_dev = (active_slot == 0) ? OTA_BACKUP_SLOT_B : OTA_BACKUP_SLOT_A;
        }
        rc = write_partition(target_dev, fw, (size_t)fsize);
        break;
    }
    case OTA_TARGET_H747_M7:
    case OTA_TARGET_H747_M4:
        h747_flash_via_ipc(fw, (size_t)fsize);
        break;
    default:
        rc = -EINVAL;
        break;
    }

    free(fw);
    if (rc != 0) {
        pthread_mutex_lock(&g_ota.lock);
        g_ota.state = OTA_STATE_ERROR;
        pthread_mutex_unlock(&g_ota.lock);
        notify_cloud(OTA_STATE_ERROR, "flash failed");
        return rc;
    }

    pthread_mutex_lock(&g_ota.lock);
    g_ota.state = OTA_STATE_IDLE;
    pthread_mutex_unlock(&g_ota.lock);
    notify_cloud(OTA_STATE_IDLE, "flash complete");
    return 0;
}

/**
 * ota_post_verify - CRC check of flashed image
 */
int ota_post_verify(void)
{
    pthread_mutex_lock(&g_ota.lock);
    if (g_ota.state != OTA_STATE_IDLE) {
        pthread_mutex_unlock(&g_ota.lock);
        return -EBUSY;
    }
    g_ota.state = OTA_STATE_POST_VERIFY;
    pthread_mutex_unlock(&g_ota.lock);

    uint32_t staged_crc, flashed_crc;
    if (compute_file_crc32(OTA_STAGING_PATH, &staged_crc) != 0) {
        pthread_mutex_lock(&g_ota.lock);
        g_ota.state = OTA_STATE_ERROR;
        pthread_mutex_unlock(&g_ota.lock);
        return -EIO;
    }

    /* For RK3566, read the partition we just wrote */
    const char *dev = OTA_BACKUP_SLOT_B; /* same logic as flash */
    uint32_t active_slot;
    bool active_ok;
    if (read_boot_flag(&active_slot, &active_ok) == 0 && active_ok) {
        dev = (active_slot == 0) ? OTA_BACKUP_SLOT_B : OTA_BACKUP_SLOT_A;
    }

    FILE *f = fopen(dev, "rb");
    if (!f) { pthread_mutex_lock(&g_ota.lock); g_ota.state = OTA_STATE_ERROR; pthread_mutex_unlock(&g_ota.lock); return -EIO; }
    uint8_t buf[4096]; size_t r;
    uint32_t crc = 0;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0)
        crc = crc32_compute(buf, r);
    fclose(f);
    flashed_crc = crc;

    if (staged_crc != flashed_crc) {
        pthread_mutex_lock(&g_ota.lock);
        g_ota.state = OTA_STATE_ERROR;
        pthread_mutex_unlock(&g_ota.lock);
        notify_cloud(OTA_STATE_ERROR, "post-install CRC mismatch");
        return -EINVAL;
    }

    /* Persist CRC check record */
    FILE *ck = fopen(OTA_CRC_CHECK_PATH, "w");
    if (ck) { fprintf(ck, "%08x\n", staged_crc); fclose(ck); }

    pthread_mutex_lock(&g_ota.lock);
    g_ota.state = OTA_STATE_IDLE;
    pthread_mutex_unlock(&g_ota.lock);
    notify_cloud(OTA_STATE_IDLE, "post-verify OK");
    return 0;
}

/**
 * ota_set_boot_flag - mark new slot as active
 */
int ota_set_boot_flag(void)
{
    pthread_mutex_lock(&g_ota.lock);
    if (g_ota.state != OTA_STATE_IDLE) {
        pthread_mutex_unlock(&g_ota.lock);
        return -EBUSY;
    }
    g_ota.state = OTA_STATE_SET_BOOT_FLAG;

    uint32_t active_slot;
    bool active_ok;
    uint32_t new_slot = 0;
    if (read_boot_flag(&active_slot, &active_ok) == 0) {
        new_slot = (active_slot == 0) ? 1 : 0;
    }
    write_boot_flag(new_slot, true);
    g_ota.boot_flag_set = true;
    pthread_mutex_unlock(&g_ota.lock);
    notify_cloud(OTA_STATE_IDLE, "boot flag set");
    return 0;
}

/**
 * ota_reboot - trigger system reboot
 */
int ota_reboot(void)
{
    pthread_mutex_lock(&g_ota.lock);
    g_ota.state = OTA_STATE_REBOOTING;
    pthread_mutex_unlock(&g_ota.lock);

    notify_cloud(OTA_STATE_REBOOTING, "rebooting for OTA");
    sync();

    /* In production: use proper reboot syscall or watchdog reset */
    int rc = system("reboot");
    if (rc != 0) {
        /* fallback to direct syscall */
        /* execl("/sbin/reboot", "reboot", "-f", NULL); */
    }
    return rc;
}

/**
 * ota_rollback - revert to previous slot on boot failure
 */
int ota_rollback(void)
{
    pthread_mutex_lock(&g_ota.lock);
    g_ota.state = OTA_STATE_ROLLBACK;
    pthread_mutex_unlock(&g_ota.lock);

    uint32_t active_slot;
    bool active_ok;
    if (read_boot_flag(&active_slot, &active_ok) != 0) {
        active_slot = 0;
    }
    /* Switch to the other slot */
    uint32_t rollback_slot = (active_slot == 0) ? 1 : 0;
    write_boot_flag(rollback_slot, false);

    /* Notify cloud of rollback */
    notify_cloud(OTA_STATE_ROLLBACK, "rolling back due to boot failure");
    g_ota.state = OTA_STATE_IDLE;
    return 0;
}

/**
 * ota_run_pipeline - execute full OTA pipeline
 */
int ota_run_pipeline(void)
{
    int rc;

    rc = ota_check_version();
    if (rc != 0) return rc;

    /* If no update needed, return early */
    /* (stub: always proceeds) */

    rc = ota_download();
    if (rc != 0) return rc;

    rc = ota_verify();
    if (rc != 0) {
        /* If verify fails after retries, roll back */
        ota_rollback();
        return rc;
    }

    rc = ota_flash();
    if (rc != 0) {
        ota_rollback();
        return rc;
    }

    rc = ota_post_verify();
    if (rc != 0) {
        ota_rollback();
        return rc;
    }

    rc = ota_set_boot_flag();
    if (rc != 0) return rc;

    rc = ota_reboot();
    return rc;
}

/**
 * ota_get_state - query current OTA state (thread-safe)
 */
ota_state_t ota_get_state(void)
{
    pthread_mutex_lock(&g_ota.lock);
    ota_state_t s = g_ota.state;
    pthread_mutex_unlock(&g_ota.lock);
    return s;
}

/* ------------------------------------------------------------------ */
/*  Initialization                                                     */
/* ------------------------------------------------------------------ */
int ota_init(void)
{
    /* Ensure staging directory exists */
    system("mkdir -p /data/ota");

    /* On boot: check if previous update failed and roll back */
    uint32_t slot = 0;
    bool ok = false;
    if (read_boot_flag(&slot, &ok) == 0 && !ok) {
        fprintf(stderr, "[OTA] Previous boot failed for slot %u, rolling back\n", slot);
        ota_rollback();
    }

    g_ota.state = OTA_STATE_IDLE;
    pthread_mutex_init(&g_ota.lock, NULL);
    return 0;
}

void ota_shutdown(void)
{
    pthread_mutex_destroy(&g_ota.lock);
}
