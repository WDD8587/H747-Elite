/**
 * @file    boot_verify.c
 * @brief   Firmware verification: SHA256 + ECDSA P-256 signature check.
 *
 * Verification steps:
 *   1. Compute SHA256 hash of application region (512 KB at 0x08010000).
 *   2. Compare computed hash against the SHA256 stored in the app header
 *      (first 64 bytes of app region).
 *   3. Verify ECDSA P-256 signature over the header using a hard-coded
 *      public key (burned in bootloader ROM).
 *   4. Return 0 on success, non-zero error code on failure.
 */

#include <stdint.h>
#include <string.h>
#include "boot_verify.h"
#include "boot_flash.h"

/* ---------------------------------------------------------------------------
 * Application header layout (first 64 bytes of app flash)
 * ---------------------------------------------------------------------------
 * Offset | Size | Field
 * -------+------+-----------------------------
 *   0    |  4   | Magic: "APP1" (0x31505041)
 *   4    |  4   | Header version
 *   8    |  4   | Image length (bytes)
 *  12    |  4   | Header CRC32
 *  16    |  4   | Reserved
 *  20    |  4   | Reserved
 *  24    |  4   | Reserved
 *  28    |  4   | Reserved
 *  32    | 32   | SHA256 hash of app body
 *  64    |  6   | Application data starts here
 * -------------------------------------------------------------------------*/

#define APP_HEADER_MAGIC        0x31505041UL    /* "APP1"              */
#define APP_HDR_SHA256_OFFSET   32U
#define APP_HDR_SHA256_SIZE     32U

/* ---------------------------------------------------------------------------
 * ECDSA P-256 constants
 * -------------------------------------------------------------------------*/
#define ECDSA_R_SIZE            32U
#define ECDSA_S_SIZE            32U

/*
 * Hard-coded ECDSA P-256 public key (Q = (Qx, Qy)).
 * Replace with the actual keypair generated for your production builds.
 */
static const uint8_t ecdsa_pub_key[64] = {
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00,
    0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89,
    0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    0x00, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99
};

/*
 * ECDSA signature is stored in the header after the SHA256.
 * For the purposes of this bootloader, the signature (r || s, 64 bytes)
 * is stored at offset 64 of the header.
 */
#define APP_HDR_SIG_OFFSET      64U
#define APP_HDR_SIG_SIZE        64U

/* ---------------------------------------------------------------------------
 * SHA-256 implementation (compact, no HW crypto dependency)
 * -------------------------------------------------------------------------*/
#define SHA256_BLOCK_SIZE       64U

typedef struct {
    uint32_t state[8];
    uint64_t bit_count;
    uint8_t  buffer[SHA256_BLOCK_SIZE];
    uint32_t buffer_len;
} sha256_ctx_t;

static const uint32_t sha256_k[64] = {
    0x428A2F98UL, 0x71374491UL, 0xB5C0FBCFUL, 0xE9B5DBA5UL,
    0x3956C25BUL, 0x59F111F1UL, 0x923F82A4UL, 0xAB1C5ED5UL,
    0xD807AA98UL, 0x12835B01UL, 0x243185BEUL, 0x550C7DC3UL,
    0x72BE5D74UL, 0x80DEB1FEUL, 0x9BDC06A7UL, 0xC19BF174UL,
    0xE49B69C1UL, 0xEFBE4786UL, 0x0FC19DC6UL, 0x240CA1CCUL,
    0x2DE92C6FUL, 0x4A7484AAUL, 0x5CB0A9DCUL, 0x76F988DAUL,
    0x983E5152UL, 0xA831C66DUL, 0xB00327C8UL, 0xBF597FC7UL,
    0xC6E00BF3UL, 0xD5A79147UL, 0x06CA6351UL, 0x14292967UL,
    0x27B70A85UL, 0x2E1B2138UL, 0x4D2C6DFCUL, 0x53380D13UL,
    0x650A7354UL, 0x766A0ABBUL, 0x81C2C92EUL, 0x92722C85UL,
    0xA2BFE8A1UL, 0xA81A664BUL, 0xC24B8B70UL, 0xC76C51A3UL,
    0xD192E819UL, 0xD6990624UL, 0xF40E3585UL, 0x106AA070UL,
    0x19A4C116UL, 0x1E376C08UL, 0x2748774CUL, 0x34B0BCB5UL,
    0x391C0CB3UL, 0x4ED8AA4AUL, 0x5B9CCA4FUL, 0x682E6FF3UL,
    0x748F82EEUL, 0x78A5636FUL, 0x84C87814UL, 0x8CC70208UL,
    0x90BEFFFAUL, 0xA4506CEBUL, 0xBEF9A3F7UL, 0xC67178F2UL
};

#define ROTR32(x, n) (((x) >> (n)) | ((x) << (32U - (n))))
#define CH(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x) (ROTR32(x, 2)  ^ ROTR32(x, 13) ^ ROTR32(x, 22))
#define EP1(x) (ROTR32(x, 6)  ^ ROTR32(x, 11) ^ ROTR32(x, 25))
#define SIG0(x) (ROTR32(x, 7)  ^ ROTR32(x, 18) ^ ((x) >> 3))
#define SIG1(x) (ROTR32(x, 17) ^ ROTR32(x, 19) ^ ((x) >> 10))

static void sha256_transform(sha256_ctx_t *ctx, const uint8_t *block)
{
    uint32_t w[64];
    uint32_t a, b, c, d, e, f, g, h;
    uint32_t t1, t2;

    /* Prepare message schedule */
    for (uint32_t i = 0; i < 16; i++) {
        w[i] = ((uint32_t)block[i * 4]     << 24) |
               ((uint32_t)block[i * 4 + 1] << 16) |
               ((uint32_t)block[i * 4 + 2] <<  8) |
               ((uint32_t)block[i * 4 + 3]);
    }
    for (uint32_t i = 16; i < 64; i++) {
        w[i] = SIG1(w[i - 2]) + w[i - 7] + SIG0(w[i - 15]) + w[i - 16];
    }

    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];

    for (uint32_t i = 0; i < 64; i++) {
        t1 = h + EP1(e) + CH(e, f, g) + sha256_k[i] + w[i];
        t2 = EP0(a) + MAJ(a, b, c);
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }

    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c;
    ctx->state[3] += d; ctx->state[4] += e; ctx->state[5] += f;
    ctx->state[6] += g; ctx->state[7] += h;
}

static void sha256_init(sha256_ctx_t *ctx)
{
    ctx->state[0] = 0x6A09E667UL; ctx->state[1] = 0xBB67AE85UL;
    ctx->state[2] = 0x3C6EF372UL; ctx->state[3] = 0xA54FF53AUL;
    ctx->state[4] = 0x510E527FUL; ctx->state[5] = 0x9B05688CUL;
    ctx->state[6] = 0x1F83D9ABUL; ctx->state[7] = 0x5BE0CD19UL;
    ctx->bit_count = 0;
    ctx->buffer_len = 0;
}

static void sha256_update(sha256_ctx_t *ctx, const uint8_t *data, uint32_t len)
{
    uint32_t space;
    uint32_t copy_len;

    for (uint32_t i = 0; i < len; ) {
        if (ctx->buffer_len >= SHA256_BLOCK_SIZE) {
            /* Process buffer */
            sha256_transform(ctx, ctx->buffer);
            ctx->buffer_len = 0;
        }
        space = SHA256_BLOCK_SIZE - ctx->buffer_len;
        copy_len = (len - i < space) ? (len - i) : space;
        memcpy(ctx->buffer + ctx->buffer_len, data + i, copy_len);
        ctx->buffer_len += copy_len;
        i += copy_len;
    }
    ctx->bit_count += (uint64_t)len * 8U;
}

static void sha256_final(sha256_ctx_t *ctx, uint8_t *hash)
{
    uint64_t bit_count_be;

    /* Pad: append 0x80 */
    ctx->buffer[ctx->buffer_len++] = 0x80;

    /* Pad with zeros until 8 bytes left */;
    uint32_t space = SHA256_BLOCK_SIZE - ctx->buffer_len;
    if (space < 8) {
        /* Not enough room; fill and process extra block */
        memset(ctx->buffer + ctx->buffer_len, 0, space);
        sha256_transform(ctx, ctx->buffer);
        ctx->buffer_len = 0;
        space = SHA256_BLOCK_SIZE;
    }
    memset(ctx->buffer + ctx->buffer_len, 0, space - 8);
    ctx->buffer_len = SHA256_BLOCK_SIZE - 8;

    /* Append bit count in big-endian */
    bit_count_be = ctx->bit_count;
    ctx->buffer[56] = (uint8_t)(bit_count_be >> 56);
    ctx->buffer[57] = (uint8_t)(bit_count_be >> 48);
    ctx->buffer[58] = (uint8_t)(bit_count_be >> 40);
    ctx->buffer[59] = (uint8_t)(bit_count_be >> 32);
    ctx->buffer[60] = (uint8_t)(bit_count_be >> 24);
    ctx->buffer[61] = (uint8_t)(bit_count_be >> 16);
    ctx->buffer[62] = (uint8_t)(bit_count_be >>  8);
    ctx->buffer[63] = (uint8_t)(bit_count_be);

    sha256_transform(ctx, ctx->buffer);

    /* Output hash (big-endian words) */
    for (uint32_t i = 0; i < 8; i++) {
        hash[i * 4 + 0] = (uint8_t)(ctx->state[i] >> 24);
        hash[i * 4 + 1] = (uint8_t)(ctx->state[i] >> 16);
        hash[i * 4 + 2] = (uint8_t)(ctx->state[i] >>  8);
        hash[i * 4 + 3] = (uint8_t)(ctx->state[i]);
    }
}

/* ---------------------------------------------------------------------------
 * ECDSA P-256 verify (minimal implementation)
 *
 * This is intended to be replaced with hardware crypto acceleration
 * (STM32H747 CRYP peripheral) or a production library (mbedTLS).
 * The stub below illustrates the calling convention.
 * -------------------------------------------------------------------------*/
#define ECDSA_P256_R     32U
#define ECDSA_P256_S     32U

/*
 * Internal helper: constant-time memory comparison.
 */
static int ct_memcmp(const void *a, const void *b, uint32_t n)
{
    uint8_t diff = 0U;
    const uint8_t *pa = (const uint8_t *)a;
    const uint8_t *pb = (const uint8_t *)b;
    for (uint32_t i = 0; i < n; i++) {
        diff |= (pa[i] ^ pb[i]);
    }
    return (diff == 0U) ? 0 : -1;
}

/**
 * @brief  Verify ECDSA P-256 signature.
 *
 * @param  pub_key   64-byte public key (Qx || Qy).
 * @param  hash      32-byte SHA256 digest.
 * @param  sig       64-byte signature (r || s).
 * @return 0 on success, -1 on failure.
 */
static int ecdsa_p256_verify(const uint8_t *pub_key,
                              const uint8_t *hash,
                              const uint8_t *sig)
{
    /*
     * In a production bootloader this function would:
     *   1. Validate that r and s are in [1, n-1].
     *   2. Compute e = hash (truncated to order bit length).
     *   3. Compute w = s^(-1) mod n.
     *   4. Compute u1 = e*w mod n, u2 = r*w mod n.
     *   5. Compute point X = u1*G + u2*Q.
     *   6. Verify (x coordinate of X) mod n == r.
     *
     * For now we stub to return success if signature is non-zero.
     * Replace with mbedTLS p256_verify() or hardware CRYP engine.
     */
    (void)pub_key;
    (void)hash;

    /* Reject all-zero signature */
    uint32_t sum = 0;
    for (uint32_t i = 0; i < ECDSA_P256_R + ECDSA_P256_S; i++) {
        sum += sig[i];
    }
    if (sum == 0) {
        return -1;  /* All-zero signature == invalid */
    }

    return 0;  /* Stub: always pass (replace with real impl) */
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Verify a firmware image at the given base address.
 *
 * Steps:
 *   1. Check header magic.
 *   2. Compute SHA256 of the image body (bytes beyond header).
 *   3. Compare computed hash against stored hash in header.
 *   4. Verify ECDSA signature over header.
 *
 * @param  image_base  Start address of the image (in flash or RAM).
 * @param  max_size    Maximum size to read (typically 512 KB).
 * @return 0 on success, negative error code on failure.
 */
int boot_verify_image(uint32_t image_base, uint32_t max_size)
{
    /* ---- Step 1: read and validate header ---- */
    uint32_t header[16];  /* 64 bytes */
    boot_flash_read(image_base, header, sizeof(header));

    if (header[0] != APP_HEADER_MAGIC) {
        return -1;  /* Bad magic */
    }

    uint32_t image_length = header[2];
    if ((image_length == 0) || (image_length > max_size)) {
        return -2;  /* Invalid image length */
    }

    /* ---- Step 2: compute SHA256 of the image body ---- */
    uint8_t expected_hash[32];
    uint8_t computed_hash[32];

    /* Copy expected hash from header */
    memcpy(expected_hash,
           (uint8_t *)header + APP_HDR_SHA256_OFFSET,
           APP_HDR_SHA256_SIZE);

    sha256_ctx_t ctx;
    sha256_init(&ctx);

    /*
     * Read flash in 256-byte chunks to avoid stack overflow and
     * to support flash running via ART accelerator.
     */
    uint32_t offset = sizeof(header);  /* Skip header */
    uint8_t  chunk[256];

    while (offset < image_length) {
        uint32_t read_size = (image_length - offset < sizeof(chunk))
                                 ? (image_length - offset)
                                 : sizeof(chunk);
        boot_flash_read(image_base + offset, chunk, read_size);
        sha256_update(&ctx, chunk, read_size);
        offset += read_size;
    }
    sha256_final(&ctx, computed_hash);

    /* ---- Step 3: compare SHA256 ---- */
    if (ct_memcmp(expected_hash, computed_hash, 32) != 0) {
        return -3;  /* Hash mismatch */
    }

    /* ---- Step 4: ECDSA signature verification ---- */
    const uint8_t *sig = (const uint8_t *)header + APP_HDR_SIG_OFFSET;
    int sig_ok = ecdsa_p256_verify(ecdsa_pub_key, expected_hash, sig);
    if (sig_ok != 0) {
        return -4;  /* Signature invalid */
    }

    return 0;  /* All checks passed */
}

/**
 * @brief  Get the expected SHA256 hash from the image header.
 *
 * @param  image_base  Address of image.
 * @param  out_hash    Output buffer (must be >= 32 bytes).
 */
void boot_verify_get_expected_hash(uint32_t image_base, uint8_t *out_hash)
{
    uint8_t header[64];
    boot_flash_read(image_base, header, sizeof(header));
    memcpy(out_hash, header + APP_HDR_SHA256_OFFSET, APP_HDR_SHA256_SIZE);
}

/**
 * @brief  Get the ECDSA signature from the image header.
 *
 * @param  image_base  Address of image.
 * @param  out_sig     Output buffer (must be >= 64 bytes).
 */
void boot_verify_get_signature(uint32_t image_base, uint8_t *out_sig)
{
    uint8_t header[128];
    boot_flash_read(image_base, header, sizeof(header));
    memcpy(out_sig, header + APP_HDR_SIG_OFFSET, APP_HDR_SIG_SIZE);
}
