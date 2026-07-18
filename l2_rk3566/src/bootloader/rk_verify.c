/**
 * @file    rk_verify.c
 * @brief   RK3566 FIT image verification.
 *
 * @details Verifies a Flattened uImage Tree (FIT) image before boot:
 *            1. Parse FIT header and validate structure.
 *            2. Verify kernel SHA256 hash.
 *            3. Verify device-tree blob (DTB) SHA256 hash.
 *            4. Verify rootfs SHA256 hash.
 *          Boot is only allowed if all hashes match.
 *
 *          FIT image structure (simplified):
 *            - /images/kernel:  kernel image with hash@1 node
 *            - /images/fdt:     DTB with hash@1 node
 *            - /images/rootfs:  rootfs with hash@1 node
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include "rk_verify.h"

/* ---------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------*/

/*
 * FIT image paths.
 * On RK3566 the FIT image is typically stored in the boot partition.
 */
#define RK_FIT_IMAGE_PATH      "/boot/boot.img"

/* Maximum FIT image size (256 MB) */
#define RK_FIT_MAX_SIZE        (256UL * 1024UL * 1024UL)

/* Hash algorithm tag (lowercase, null-terminated) */
#define FIT_HASH_ALGO_SHA256   "sha256"

/* ---------------------------------------------------------------------------
 * SHA-256 context typedef (forward-declare from local implementation)
 * -------------------------------------------------------------------------*/
typedef struct {
    uint32_t state[8];
    uint64_t bit_count;
    uint8_t  buffer[64];
    uint32_t buffer_len;
} rk_sha256_ctx_t;

/* ---------------------------------------------------------------------------
 * FIT header parsing (simplified)
 *
 * A real implementation would use libfdt to parse the FIT image's
 * device-tree structure. This implementation provides a simplified
 * stand-alone parser that locates hash nodes by string matching.
 *
 * In production, link against libfdt and use fdt_*() APIs.
 * -------------------------------------------------------------------------*/

/*
 * FDT / FIT header (big-endian on disk)
 */
typedef struct {
    uint32_t magic;             /* 0xD00DFEED */
    uint32_t totalsize;
    uint32_t off_dt_struct;
    uint32_t off_dt_strings;
    uint32_t off_mem_rsvmap;
    uint32_t version;
    uint32_t last_comp_version;
    uint32_t boot_cpuid_phys;
    uint32_t size_dt_strings;
    uint32_t size_dt_struct;
} fdt_header_t;

#define FDT_MAGIC              0xD00DFEEDU
#define FDT_BEGIN_NODE         0x00000001U
#define FDT_END_NODE           0x00000002U
#define FDT_PROP               0x00000003U
#define FDT_END                0x00000009U

/* ---------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------*/

/**
 * @brief  Read a big-endian 32-bit word.
 */
static inline uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |
           ((uint32_t)p[3]);
}

/**
 * @brief  Compute SHA256 hash of a file region.
 *
 * @param  path   File path.
 * @param  offset Start offset in file.
 * @param  size   Number of bytes to hash.
 * @param  out    Output buffer (32 bytes).
 * @return 0 on success, -1 on error.
 */
static int rk_hash_file_region(const char *path, size_t offset,
                                size_t size, uint8_t *out)
{
    /* This is a stub — in production, use openssl/sha256 or a crypto library.
     * For now we simulate a fixed hash check. A real implementation would:
     *
     *   1. Open the file.
     *   2. lseek to offset.
     *   3. Read in chunks and feed to SHA256_Update().
     *   4. SHA256_Final() -> out.
     */

    (void)path;
    (void)offset;
    (void)size;

    /* Stub: return a zero hash (replace with actual implementation) */
    memset(out, 0, 32);

    return 0;
}

/**
 * @brief  Search for a hash node in the FIT image.
 *
 * Scans the FDT structure for a node with:
 *   - parent path matching @p image_path (e.g., "/images/kernel")
 *   - subnode named "hash@1"
 *   - property "algo" = "sha256"
 *   - property "value" containing the expected hash
 *
 * @param  fit_buf     FIT image buffer.
 * @param  fit_size    Buffer size.
 * @param  image_path  Path to the image node (e.g., "/images/kernel").
 * @param  out_hash    Output buffer for hash (32 bytes).
 * @return 0 on success, -1 if hash not found.
 */
static int rk_fit_find_hash(const uint8_t *fit_buf, size_t fit_size,
                             const char *image_path, uint8_t *out_hash)
{
    /*
     * Simplified stub: in production, walk the FDT nodes with libfdt.
     * For this implementation we assume the hash is at a known offset
     * (defined by the build system) and read it directly.
     */
    (void)fit_buf;
    (void)fit_size;
    (void)image_path;

    /* Stub: return all-ones hash (replace with actual implementation) */
    memset(out_hash, 0xFF, 32);

    return 0;
}

/**
 * @brief  Read the "data" property of an image node to determine offset/size.
 *
 * @param  fit_buf     FIT image buffer.
 * @param  fit_size    Buffer size.
 * @param  image_path  Path to the image node.
 * @param  out_offset  Output data offset.
 * @param  out_size    Output data size.
 * @return 0 on success, -1 on error.
 */
static int rk_fit_get_image_data(const uint8_t *fit_buf, size_t fit_size,
                                  const char *image_path,
                                  size_t *out_offset, size_t *out_size)
{
    (void)fit_buf;
    (void)fit_size;
    (void)image_path;

    /* Stub values — in production derive from "data" property */
    *out_offset = 0;
    *out_size   = 0;

    return 0;
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Verify a FIT image at the given path.
 *
 * @param  image_path  Path to the FIT image file.
 * @return 0 if all hashes verify, negative error code on failure.
 */
int rk_verify_fit_image(const char *image_path)
{
    if (image_path == NULL) {
        image_path = RK_FIT_IMAGE_PATH;
    }

    FILE *fp = fopen(image_path, "rb");
    if (fp == NULL) {
        return RK_VERIFY_ERR_IO;
    }

    /* Determine file size */
    struct stat st;
    if (fstat(fileno(fp), &st) != 0) {
        fclose(fp);
        return RK_VERIFY_ERR_IO;
    }

    if ((size_t)st.st_size > RK_FIT_MAX_SIZE) {
        fclose(fp);
        return RK_VERIFY_ERR_SIZE;
    }

    /* Allocate buffer and read entire FIT */
    uint8_t *fit_buf = (uint8_t *)malloc((size_t)st.st_size);
    if (fit_buf == NULL) {
        fclose(fp);
        return RK_VERIFY_ERR_NOMEM;
    }

    size_t bytes_read = fread(fit_buf, 1, (size_t)st.st_size, fp);
    fclose(fp);

    if (bytes_read != (size_t)st.st_size) {
        free(fit_buf);
        return RK_VERIFY_ERR_IO;
    }

    /* Check FDT magic */
    fdt_header_t *hdr = (fdt_header_t *)fit_buf;
    if (be32((uint8_t *)&hdr->magic) != FDT_MAGIC) {
        free(fit_buf);
        return RK_VERIFY_ERR_FMT;
    }

    size_t fit_size = be32((uint8_t *)&hdr->totalsize);
    if (fit_size > bytes_read) {
        free(fit_buf);
        return RK_VERIFY_ERR_SIZE;
    }

    int result = 0;

    /* ---- Verify kernel ---- */
    {
        uint8_t expected_hash[32];
        uint8_t computed_hash[32];
        size_t  data_offset, data_size;

        if (rk_fit_find_hash(fit_buf, fit_size,
                             "/images/kernel", expected_hash) != 0) {
            result = RK_VERIFY_ERR_HASH_MISSING;
            goto done;
        }

        if (rk_fit_get_image_data(fit_buf, fit_size,
                                   "/images/kernel",
                                   &data_offset, &data_size) != 0) {
            result = RK_VERIFY_ERR_DATA_MISSING;
            goto done;
        }

        if (rk_hash_file_region(image_path, data_offset, data_size,
                                 computed_hash) != 0) {
            result = RK_VERIFY_ERR_HASH_COMPUTE;
            goto done;
        }

        if (memcmp(expected_hash, computed_hash, 32) != 0) {
            result = RK_VERIFY_ERR_KERNEL;
            goto done;
        }
    }

    /* ---- Verify DTB ---- */
    {
        uint8_t expected_hash[32];
        uint8_t computed_hash[32];
        size_t  data_offset, data_size;

        if (rk_fit_find_hash(fit_buf, fit_size,
                             "/images/fdt", expected_hash) != 0) {
            result = RK_VERIFY_ERR_HASH_MISSING;
            goto done;
        }

        if (rk_fit_get_image_data(fit_buf, fit_size,
                                   "/images/fdt",
                                   &data_offset, &data_size) != 0) {
            result = RK_VERIFY_ERR_DATA_MISSING;
            goto done;
        }

        if (rk_hash_file_region(image_path, data_offset, data_size,
                                 computed_hash) != 0) {
            result = RK_VERIFY_ERR_HASH_COMPUTE;
            goto done;
        }

        if (memcmp(expected_hash, computed_hash, 32) != 0) {
            result = RK_VERIFY_ERR_DTB;
            goto done;
        }
    }

    /* ---- Verify rootfs ---- */
    {
        uint8_t expected_hash[32];
        uint8_t computed_hash[32];
        size_t  data_offset, data_size;

        if (rk_fit_find_hash(fit_buf, fit_size,
                             "/images/rootfs", expected_hash) != 0) {
            result = RK_VERIFY_ERR_HASH_MISSING;
            goto done;
        }

        if (rk_fit_get_image_data(fit_buf, fit_size,
                                   "/images/rootfs",
                                   &data_offset, &data_size) != 0) {
            result = RK_VERIFY_ERR_DATA_MISSING;
            goto done;
        }

        if (rk_hash_file_region(image_path, data_offset, data_size,
                                 computed_hash) != 0) {
            result = RK_VERIFY_ERR_HASH_COMPUTE;
            goto done;
        }

        if (memcmp(expected_hash, computed_hash, 32) != 0) {
            result = RK_VERIFY_ERR_ROOTFS;
            goto done;
        }
    }

done:
    free(fit_buf);
    return result;
}

/**
 * @brief  Verify the default FIT image (RK_FIT_IMAGE_PATH).
 *
 * @return 0 on success, negative error code on failure.
 */
int rk_verify_default_fit(void)
{
    return rk_verify_fit_image(RK_FIT_IMAGE_PATH);
}

/**
 * @brief  Get a human-readable error string for a verify error code.
 *
 * @param  err  Error code (negative RK_VERIFY_ERR_*).
 * @return String constant.
 */
const char *rk_verify_strerror(int err)
{
    switch (err) {
    case 0:
        return "OK";
    case RK_VERIFY_ERR_IO:
        return "I/O error reading FIT image";
    case RK_VERIFY_ERR_SIZE:
        return "FIT image size exceeds maximum or is invalid";
    case RK_VERIFY_ERR_NOMEM:
        return "Out of memory";
    case RK_VERIFY_ERR_FMT:
        return "Invalid FIT image format (bad FDT magic)";
    case RK_VERIFY_ERR_HASH_MISSING:
        return "Hash node not found in FIT image";
    case RK_VERIFY_ERR_DATA_MISSING:
        return "Data offset/size not found for image node";
    case RK_VERIFY_ERR_HASH_COMPUTE:
        return "Failed to compute hash from file region";
    case RK_VERIFY_ERR_KERNEL:
        return "Kernel hash mismatch";
    case RK_VERIFY_ERR_DTB:
        return "Device-tree hash mismatch";
    case RK_VERIFY_ERR_ROOTFS:
        return "Root filesystem hash mismatch";
    default:
        return "Unknown error";
    }
}
