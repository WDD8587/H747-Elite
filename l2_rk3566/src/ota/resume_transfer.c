/**
 * @file    resume_transfer.c
 * @brief   Resume interrupted OTA transfer from cloud
 * @details Tracks received blocks using a bitmap (1 bit per 4 KB block).
 *          On reconnection, requests only missing blocks from the server.
 *          Each block has an individual SHA256 for corruption detection.
 *          Maximum resume attempts: 5 before full re-download.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */
#define RESUME_MAGIC            "RS01"
#define RESUME_BLOCK_SIZE       4096         /* 4 KB per block */
#define RESUME_MAX_ATTEMPTS     5
#define RESUME_STATE_PATH       "/data/ota/resume_state.bin"
#define SHA256_DIGEST_LEN       32

/* ------------------------------------------------------------------ */
/*  Resume state (persistent)                                          */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    char     magic[4];            /* "RS01" */
    uint32_t version;             /* state version */
    uint32_t total_blocks;        /* total blocks in firmware */
    uint32_t blocks_received;     /* count of received (complete) blocks */
    uint32_t attempt_count;       /* number of resume attempts so far */
    uint64_t total_size;          /* total firmware size in bytes */
    uint8_t  fw_sha256[32];       /* SHA256 of entire firmware */
    /* bitmap follows immediately (variable length = ceil(total_blocks/8)) */
} resume_header_t;

/* ------------------------------------------------------------------ */
/*  Runtime context                                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    resume_header_t header;
    uint8_t        *bitmap;        /* received bitmap (1 = received) */
    size_t          bitmap_size;   /* bytes in bitmap */
    uint8_t        *block_hashes;  /* SHA256 per block (total_blocks * 32) */
    size_t          block_hashes_size;
} resume_ctx_t;

static resume_ctx_t g_ctx;
static bool g_initialized = false;

/* ------------------------------------------------------------------ */
/*  CRC32 (simple)                                                    */
/* ------------------------------------------------------------------ */
static uint32_t resume_crc32(const void *data, size_t len)
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
/*  Bitmap helpers                                                     */
/* ------------------------------------------------------------------ */

static inline size_t bitmap_bytes(uint32_t total_blocks)
{
    return (total_blocks + 7) / 8;
}

static inline bool bitmap_test(const uint8_t *bm, uint32_t block_idx)
{
    return (bm[block_idx / 8] >> (block_idx % 8)) & 1;
}

static inline void bitmap_set(uint8_t *bm, uint32_t block_idx)
{
    bm[block_idx / 8] |= (uint8_t)(1 << (block_idx % 8));
}

static inline void bitmap_clear(uint8_t *bm, uint32_t block_idx)
{
    bm[block_idx / 8] &= (uint8_t)(~(1 << (block_idx % 8)));
}

/* ------------------------------------------------------------------ */
/*  SHA256 stub — replace with mbedTLS / OpenSSL                       */
/* ------------------------------------------------------------------ */
static int sha256_compute(const void *data, size_t len, uint8_t digest[32])
{
    /* STUB: real SHA256 here */
    (void)data; (void)len;
    memset(digest, 0xDD, 32);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Persistence                                                        */
/* ------------------------------------------------------------------ */
static int resume_save_state(void)
{
    FILE *f = fopen(RESUME_STATE_PATH, "wb");
    if (!f) return -EIO;

    if (fwrite(&g_ctx.header, 1, sizeof(resume_header_t), f) != sizeof(resume_header_t)) {
        fclose(f);
        return -EIO;
    }
    if (fwrite(g_ctx.bitmap, 1, g_ctx.bitmap_size, f) != g_ctx.bitmap_size) {
        fclose(f);
        return -EIO;
    }
    if (g_ctx.block_hashes && g_ctx.block_hashes_size > 0) {
        if (fwrite(g_ctx.block_hashes, 1, g_ctx.block_hashes_size, f) != g_ctx.block_hashes_size) {
            fclose(f);
            return -EIO;
        }
    }

    fclose(f);
    return 0;
}

static int resume_load_state(void)
{
    FILE *f = fopen(RESUME_STATE_PATH, "rb");
    if (!f) return -ENOENT;

    if (fread(&g_ctx.header, 1, sizeof(resume_header_t), f) != sizeof(resume_header_t)) {
        fclose(f);
        return -EIO;
    }

    if (memcmp(g_ctx.header.magic, RESUME_MAGIC, 4) != 0) {
        fclose(f);
        return -EINVAL;
    }

    g_ctx.bitmap_size = bitmap_bytes(g_ctx.header.total_blocks);
    g_ctx.bitmap = (uint8_t *)calloc(g_ctx.bitmap_size, 1);
    if (!g_ctx.bitmap) { fclose(f); return -ENOMEM; }

    if (fread(g_ctx.bitmap, 1, g_ctx.bitmap_size, f) != g_ctx.bitmap_size) {
        fclose(f);
        free(g_ctx.bitmap);
        g_ctx.bitmap = NULL;
        return -EIO;
    }

    g_ctx.block_hashes_size = g_ctx.header.total_blocks * SHA256_DIGEST_LEN;
    g_ctx.block_hashes = (uint8_t *)malloc(g_ctx.block_hashes_size);
    if (!g_ctx.block_hashes) {
        fclose(f);
        free(g_ctx.bitmap);
        g_ctx.bitmap = NULL;
        return -ENOMEM;
    }

    size_t to_read = g_ctx.block_hashes_size;
    if (fread(g_ctx.block_hashes, 1, to_read, f) != to_read) {
        /* Hashes may not exist in old state files; this is OK */
        memset(g_ctx.block_hashes, 0, to_read);
    }

    fclose(f);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * resume_init - initialize resume context for a new transfer
 * @total_size:       total firmware size in bytes
 * @fw_sha256:        SHA256 of the complete firmware (for final verification)
 *
 * Returns 0 on success, negative on error.  If a previous transfer state
 * exists, it is loaded automatically.
 */
int resume_init(uint64_t total_size, const uint8_t fw_sha256[32])
{
    memset(&g_ctx, 0, sizeof(g_ctx));

    /* Try to load existing state */
    int rc = resume_load_state();
    if (rc == 0 && g_ctx.header.total_size == total_size) {
        /* Existing state matches — check attempt count */
        g_ctx.header.attempt_count++;
        if (g_ctx.header.attempt_count > RESUME_MAX_ATTEMPTS) {
            fprintf(stderr, "[RESUME] max attempts (%d) exceeded, starting fresh\n",
                    RESUME_MAX_ATTEMPTS);
            resume_reset();
            return resume_init(total_size, fw_sha256);
        }
        resume_save_state();
        g_initialized = true;
        return 0;
    }

    /* Start fresh */
    memcpy(g_ctx.header.magic, RESUME_MAGIC, 4);
    g_ctx.header.version = 1;
    g_ctx.header.total_blocks = (uint32_t)((total_size + RESUME_BLOCK_SIZE - 1) / RESUME_BLOCK_SIZE);
    g_ctx.header.blocks_received = 0;
    g_ctx.header.attempt_count = 1;
    g_ctx.header.total_size = total_size;
    memcpy(g_ctx.header.fw_sha256, fw_sha256, 32);

    g_ctx.bitmap_size = bitmap_bytes(g_ctx.header.total_blocks);
    g_ctx.bitmap = (uint8_t *)calloc(g_ctx.bitmap_size, 1);
    if (!g_ctx.bitmap) return -ENOMEM;

    g_ctx.block_hashes_size = g_ctx.header.total_blocks * SHA256_DIGEST_LEN;
    g_ctx.block_hashes = (uint8_t *)calloc(g_ctx.block_hashes_size, 1);
    if (!g_ctx.block_hashes) {
        free(g_ctx.bitmap);
        g_ctx.bitmap = NULL;
        return -ENOMEM;
    }

    resume_save_state();
    g_initialized = true;
    return 0;
}

/**
 * resume_reset - clear resume state (start over)
 */
void resume_reset(void)
{
    if (g_ctx.bitmap) {
        free(g_ctx.bitmap);
        g_ctx.bitmap = NULL;
    }
    if (g_ctx.block_hashes) {
        free(g_ctx.block_hashes);
        g_ctx.block_hashes = NULL;
    }
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_initialized = false;

    remove(RESUME_STATE_PATH);
}

/**
 * resume_mark_block_received - record a successfully received block
 * @block_idx:  block index (0-based)
 * @data:       block data
 * @len:        block length (<= RESUME_BLOCK_SIZE)
 * @block_hash: SHA256 of this block (for verification)
 *
 * Returns 0 on success, -EINVAL if hash mismatch.
 */
int resume_mark_block_received(uint32_t block_idx,
                                const uint8_t *data, size_t len,
                                const uint8_t block_hash[32])
{
    if (!g_initialized) return -EPERM;
    if (block_idx >= g_ctx.header.total_blocks) return -EINVAL;

    /* Verify block hash */
    uint8_t computed[SHA256_DIGEST_LEN];
    sha256_compute(data, len, computed);

    volatile int mismatch = 0;
    for (int i = 0; i < SHA256_DIGEST_LEN; i++) {
        mismatch |= (computed[i] ^ block_hash[i]);
    }

    if (mismatch) {
        fprintf(stderr, "[RESUME] block %u hash mismatch, requesting retransmit\n",
                block_idx);
        return -EINVAL;
    }

    /* Store block hash */
    memcpy(g_ctx.block_hashes + block_idx * SHA256_DIGEST_LEN,
           block_hash, SHA256_DIGEST_LEN);

    if (!bitmap_test(g_ctx.bitmap, block_idx)) {
        bitmap_set(g_ctx.bitmap, block_idx);
        g_ctx.header.blocks_received++;
    }

    resume_save_state();
    return 0;
}

/**
 * resume_get_missing_blocks - build list of block indices not yet received
 * @indices:   output buffer (caller allocates, should hold total_blocks entries)
 * @count:     output: number of missing blocks
 *
 * Returns 0 on success.
 */
int resume_get_missing_blocks(uint32_t *indices, uint32_t *count)
{
    if (!g_initialized) return -EPERM;
    if (!indices || !count) return -EINVAL;

    *count = 0;
    for (uint32_t i = 0; i < g_ctx.header.total_blocks; i++) {
        if (!bitmap_test(g_ctx.bitmap, i)) {
            indices[(*count)++] = i;
        }
    }
    return 0;
}

/**
 * resume_is_complete - check if all blocks have been received
 */
bool resume_is_complete(void)
{
    if (!g_initialized) return false;
    return g_ctx.header.blocks_received >= g_ctx.header.total_blocks;
}

/**
 * resume_get_progress - get transfer progress info
 */
int resume_get_progress(uint32_t *received, uint32_t *total, uint32_t *attempts)
{
    if (!g_initialized) return -EPERM;
    if (received) *received = g_ctx.header.blocks_received;
    if (total)    *total    = g_ctx.header.total_blocks;
    if (attempts) *attempts = g_ctx.header.attempt_count;
    return 0;
}

/**
 * resume_build_manifest - create a JSON manifest of missing blocks
 *                         for the server to fulfill.
 * @buf:      output buffer
 * @buf_size: buffer size
 *
 * Returns 0 on success, -ENOSPC if buffer too small.
 */
int resume_build_manifest(char *buf, size_t buf_size)
{
    if (!g_initialized) return -EPERM;
    if (!buf || buf_size < 64) return -EINVAL;

    /* Count missing blocks */
    uint32_t missing = 0;
    for (uint32_t i = 0; i < g_ctx.header.total_blocks; i++) {
        if (!bitmap_test(g_ctx.bitmap, i))
            missing++;
    }

    int n = snprintf(buf, buf_size,
                     "{\"total_blocks\":%u,\"block_size\":%u,"
                     "\"blocks_received\":%u,\"blocks_missing\":%u,"
                     "\"attempt\":%u,\"max_attempts\":%u}",
                     g_ctx.header.total_blocks,
                     RESUME_BLOCK_SIZE,
                     g_ctx.header.blocks_received,
                     missing,
                     g_ctx.header.attempt_count,
                     RESUME_MAX_ATTEMPTS);

    if (n < 0 || (size_t)n >= buf_size)
        return -ENOSPC;

    return 0;
}

/**
 * resume_get_total_size - get total firmware size
 */
uint64_t resume_get_total_size(void)
{
    if (!g_initialized) return 0;
    return g_ctx.header.total_size;
}

/**
 * resume_get_attempt_count - get number of resume attempts
 */
uint32_t resume_get_attempt_count(void)
{
    if (!g_initialized) return 0;
    return g_ctx.header.attempt_count;
}

/**
 * resume_is_max_attempts_reached - check if maximum attempts exceeded
 */
bool resume_is_max_attempts_reached(void)
{
    if (!g_initialized) return false;
    return g_ctx.header.attempt_count >= RESUME_MAX_ATTEMPTS;
}

/* ------------------------------------------------------------------ */
/*  Self-test                                                          */
/* ------------------------------------------------------------------ */
#ifdef TEST_RESUME
int main(void)
{
    uint8_t fw_sha256[32];
    memset(fw_sha256, 0xAA, 32);

    printf("Init transfer for 16 KB firmware (4 blocks of 4 KB)\n");
    resume_init(16384, fw_sha256);

    uint32_t indices[100];
    uint32_t count = 0;
    resume_get_missing_blocks(indices, &count);
    printf("Missing blocks before: %u\n", count);

    /* Simulate receiving blocks 0 and 2 */
    uint8_t dummy[4096];
    uint8_t hash[32];
    memset(dummy, 0x42, 4096);
    sha256_compute(dummy, 4096, hash);

    resume_mark_block_received(0, dummy, 4096, hash);
    resume_mark_block_received(2, dummy, 4096, hash);

    resume_get_missing_blocks(indices, &count);
    printf("Missing blocks after partial: %u\n", count);
    for (uint32_t i = 0; i < count; i++)
        printf("  missing block: %u\n", indices[i]);

    printf("Progress: %u/%u\n", g_ctx.header.blocks_received, g_ctx.header.total_blocks);
    printf("Complete: %d\n", resume_is_complete());

    resume_reset();
    printf("Reset OK\n");
    return 0;
}
#endif /* TEST_RESUME */
