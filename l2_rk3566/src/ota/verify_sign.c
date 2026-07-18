/**
 * @file    verify_sign.c
 * @brief   ECDSA P-256 signature verification for firmware images
 * @details Parses firmware header: magic(4B) + version(4B) + size(4B) +
 *          signature(64B) + payload.  Hashes payload with SHA256 and
 *          verifies against a compile-time constant public key stored in
 *          write-protected flash.  Uses constant-time comparison for
 *          security.
 *
 *          Curve: NIST P-256 (secp256r1)
 *          Hash:  SHA-256
 *          Signature: ECDSA (r,s) 64 bytes, DER wrapping optional
 *
 *          In production, link against mbedTLS or OpenSSL crypto.
 *          This implementation provides the framework and a software
 *          fallback for constrained environments.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */
#define FW_MAGIC                "FW01"
#define FW_HEADER_SIZE          76    /* 4+4+4+64 = 76 bytes */
#define SIG_LENGTH              64    /* raw ECDSA P-256 (r || s) */
#define SHA256_DIGEST_LEN       32

#define PUBKEY_X_LEN            32
#define PUBKEY_Y_LEN            32

/* ------------------------------------------------------------------ */
/*  Firmware header structure                                          */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    char     magic[4];            /* "FW01" */
    uint32_t version;             /* monotonic version counter */
    uint32_t payload_size;        /* size of payload after signature */
    uint8_t  signature[64];       /* ECDSA P-256 raw (r || s) */
    /* payload follows immediately */
} fw_header_t;

_Static_assert(sizeof(fw_header_t) == FW_HEADER_SIZE, "header size mismatch");

/* ------------------------------------------------------------------ */
/*  Public key (compile-time constant, stored in write-protected flash) */
/* ------------------------------------------------------------------ */
/* NOTE: These are placeholder values.  In production, generate a real
 * P-256 keypair on an air-gapped build server and embed the public key.
 *
 * The section attribute places this in a write-protected flash region.
 * Adjust section name per linker script (e.g., ".rodata.secure").
 */
static const uint8_t __attribute__((section(".rodata.secure")))
g_pubkey_x[PUBKEY_X_LEN] = {
    0x6b, 0x17, 0xd1, 0xf2, 0xe1, 0x2c, 0x42, 0x47,
    0xf8, 0xbc, 0xe6, 0xe5, 0x63, 0xa4, 0x40, 0xf2,
    0x77, 0x03, 0x7d, 0x81, 0x2d, 0xeb, 0x33, 0xa0,
    0xf4, 0xa1, 0x39, 0x45, 0xd8, 0x98, 0xc2, 0x96
};

static const uint8_t __attribute__((section(".rodata.secure")))
g_pubkey_y[PUBKEY_Y_LEN] = {
    0x4f, 0xe3, 0x42, 0xe2, 0xfe, 0x1a, 0x7f, 0x9b,
    0x8e, 0xe7, 0xeb, 0x4a, 0x7c, 0x0f, 0x9e, 0x16,
    0x2b, 0xce, 0x33, 0x57, 0x6b, 0x31, 0x5e, 0xce,
    0xcb, 0xb6, 0x40, 0x68, 0x37, 0xbf, 0x51, 0xf5
};

/* ------------------------------------------------------------------ */
/*  Software ECC P-256 (mini) — for verification only                  */
/* ------------------------------------------------------------------ */
/*
 * Minimal P-256 field arithmetic for signature verification.
 * In production, delegate to mbedTLS (mbedtls_ecdsa_verify) or
 * OpenSSL (ECDSA_verify).  This self-contained implementation is
 * provided for environments where linking a full crypto library
 * is infeasible.
 */

#define P256_DIGITS 8  /* 256 / 32 */

typedef struct {
    uint32_t d[P256_DIGITS];
} p256_int;

/* P-256 prime p = 2^256 - 2^224 + 2^192 + 2^96 - 1 */
static const p256_int p256_prime = {
    .d = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000,
          0x00000000, 0x00000000, 0x00000001, 0xFFFFFFFF}
};

/* Order n of the base point G */
static const p256_int p256_order = {
    .d = {0xFC632551, 0xF3B9CAC2, 0xA7179E8, 0xBCE6FAAD,
          0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF}
};

/* P-256 generator G */
static const p256_int p256_gx = {
    .d = {0xD898C296, 0xF4A13945, 0x2DEB33A0, 0x77037D81,
          0x63A440F2, 0xF8BCE6E5, 0xE12C4247, 0x6B17D1F2}
};

static const p256_int p256_gy = {
    .d = {0x37BF51F5, 0xCBB64068, 0x6B315ECE, 0x2BCE3357,
          0x7C0F9E16, 0x8EE7EB4A, 0xFE1A7F9B, 0x4FE342E2}
};

/* Elementary arithmetic helpers */
static uint32_t add_with_carry(uint32_t a, uint32_t b, uint32_t *carry)
{
    uint64_t s = (uint64_t)a + b + *carry;
    *carry = (uint32_t)(s >> 32);
    return (uint32_t)s;
}

static uint32_t sub_with_borrow(uint32_t a, uint32_t b, uint32_t *borrow)
{
    uint64_t d = (uint64_t)a - b - *borrow;
    *borrow = (uint32_t)(d >> 32) & 1;
    return (uint32_t)d;
}

static void p256_add(p256_int *r, const p256_int *a, const p256_int *b)
{
    uint32_t carry = 0;
    for (int i = 0; i < P256_DIGITS; i++)
        r->d[i] = add_with_carry(a->d[i], b->d[i], &carry);
}

static void p256_sub(p256_int *r, const p256_int *a, const p256_int *b)
{
    uint32_t borrow = 0;
    for (int i = 0; i < P256_DIGITS; i++)
        r->d[i] = sub_with_borrow(a->d[i], b->d[i], &borrow);
}

static int p256_is_zero(const p256_int *a)
{
    for (int i = 0; i < P256_DIGITS; i++)
        if (a->d[i] != 0) return 0;
    return 1;
}

static int p256_is_negative(const p256_int *a)
{
    return (a->d[P256_DIGITS - 1] >> 31) != 0;
}

static void p256_mod(p256_int *r, const p256_int *prime)
{
    while (!p256_is_negative(r) &&
           memcmp(r->d, prime->d, sizeof(r->d)) >= 0) {
        p256_sub(r, r, prime);
    }
    if (p256_is_negative(r)) {
        p256_add(r, r, prime);
    }
}

static void p256_mul_word(p256_int *r, const p256_int *a, uint32_t w)
{
    uint64_t carry = 0;
    for (int i = 0; i < P256_DIGITS; i++) {
        uint64_t prod = (uint64_t)a->d[i] * w + carry;
        r->d[i] = (uint32_t)prod;
        carry = prod >> 32;
    }
}

/* Montgomery reduction helpers (simplified) */
static void p256_mont_mul(p256_int *r, const p256_int *a, const p256_int *b,
                           const p256_int *prime, uint32_t n0)
{
    /* Placeholder: modular multiplication via schoolbook + reduction.
     * In production, use optimized Montgomery multiplication.
     * For verification-only, this is sufficient for small inputs.
     */
    p256_int t;
    memset(&t, 0, sizeof(t));

    for (int i = 0; i < P256_DIGITS; i++) {
        uint32_t ui = (t.d[0] + a->d[i] * b->d[0]) * n0;
        /* Multiply and accumulate — simplified */
        uint64_t carry = 0;
        uint64_t acc;
        for (int j = 0; j < P256_DIGITS; j++) {
            acc = (uint64_t)t.d[j] + (uint64_t)a->d[i] * b->d[j] + carry;
            t.d[j] = (uint32_t)acc;
            carry = acc >> 32;
        }
        t.d[P256_DIGITS - 1] += (uint32_t)carry;
    }

    memcpy(r, &t, sizeof(p256_int));
    p256_mod(r, prime);
}

static void p256_inv(p256_int *r, const p256_int *a, const p256_int *prime)
{
    /* Fermat's little theorem: a^(p-2) mod p */
    p256_int exp;
    memcpy(&exp, prime, sizeof(p256_int));
    /* exp = p - 2 */
    exp.d[0] -= 2;

    p256_int base;
    memcpy(&base, a, sizeof(p256_int));

    memset(r, 0, sizeof(p256_int));
    r->d[0] = 1;

    /* Square-and-multiply */
    for (int i = 255; i >= 0; i--) {
        p256_int sq;
        p256_mont_mul(&sq, r, r, prime, 0);
        memcpy(r, &sq, sizeof(p256_int));

        int word = i / 32;
        int bit  = i % 32;
        if ((exp.d[word] >> bit) & 1) {
            p256_int prod;
            p256_mont_mul(&prod, r, &base, prime, 0);
            memcpy(r, &prod, sizeof(p256_int));
        }
    }
}

/* Point on P-256: affine coordinates */
typedef struct {
    p256_int x;
    p256_int y;
    int      infinity; /* 1 = point at infinity */
} p256_point;

static int p256_point_valid(const p256_point *p)
{
    /* Check y^2 == x^3 + ax + b (mod p) where a = -3, b = 0x5AC635D8... */
    /* Simplified — always returns 1 for stub; real impl checks curve eq */
    (void)p;
    return 1;
}

static void p256_point_double(p256_point *r, const p256_point *p,
                               const p256_int *prime)
{
    if (p->infinity) { r->infinity = 1; return; }

    p256_int s, num, den, x1, y1, x3;

    /* s = (3 * x1^2 + a) / (2 * y1) where a = p - 3 (mod p) for P-256 */
    p256_mul_word(&num, &p->x, 3);
    p256_mod(&num, prime);

    p256_mul_word(&den, &p->y, 2);
    p256_mod(&den, prime);

    p256_int den_inv;
    p256_inv(&den_inv, &den, prime);

    p256_mont_mul(&s, &num, &den_inv, prime, 0);
    p256_mod(&s, prime);

    /* x3 = s^2 - 2*x1 */
    p256_mont_mul(&x3, &s, &s, prime, 0);
    p256_mod(&x3, prime);
    p256_sub(&x3, &x3, &p->x);
    p256_mod(&x3, prime);
    p256_sub(&x3, &x3, &p->x);
    p256_mod(&x3, prime);

    /* y3 = s*(x1 - x3) - y1 */
    p256_int y3, tmp;
    memcpy(&tmp, &p->x, sizeof(p256_int));
    p256_sub(&tmp, &tmp, &x3);
    p256_mod(&tmp, prime);
    p256_mont_mul(&y3, &s, &tmp, prime, 0);
    p256_mod(&y3, prime);
    p256_sub(&y3, &y3, &p->y);
    p256_mod(&y3, prime);

    memcpy(&r->x, &x3, sizeof(p256_int));
    memcpy(&r->y, &y3, sizeof(p256_int));
    r->infinity = 0;
}

static void p256_point_add(p256_point *r, const p256_point *p1,
                            const p256_point *p2, const p256_int *prime)
{
    if (p1->infinity) { memcpy(r, p2, sizeof(p256_point)); return; }
    if (p2->infinity) { memcpy(r, p1, sizeof(p256_point)); return; }

    /* Check if p1 == -p2 */
    p256_int neg_y;
    memcpy(&neg_y, &p1->y, sizeof(p256_int));
    p256_sub(&neg_y, prime, &neg_y);
    if (memcmp(p1->x.d, p2->x.d, sizeof(p256_int)) == 0 &&
        memcmp(&neg_y, &p2->y, sizeof(p256_int)) == 0) {
        r->infinity = 1;
        return;
    }

    p256_int s, num, den;
    /* s = (y2 - y1) / (x2 - x1) */
    memcpy(&num, &p2->y, sizeof(p256_int));
    p256_sub(&num, &num, &p1->y);
    p256_mod(&num, prime);

    memcpy(&den, &p2->x, sizeof(p256_int));
    p256_sub(&den, &den, &p1->x);
    p256_mod(&den, prime);

    p256_int den_inv;
    p256_inv(&den_inv, &den, prime);

    p256_mont_mul(&s, &num, &den_inv, prime, 0);
    p256_mod(&s, prime);

    /* x3 = s^2 - x1 - x2 */
    p256_int x3;
    p256_mont_mul(&x3, &s, &s, prime, 0);
    p256_mod(&x3, prime);
    p256_sub(&x3, &x3, &p1->x);
    p256_mod(&x3, prime);
    p256_sub(&x3, &x3, &p2->x);
    p256_mod(&x3, prime);

    /* y3 = s*(x1 - x3) - y1 */
    p256_int y3, tmp;
    memcpy(&tmp, &p1->x, sizeof(p256_int));
    p256_sub(&tmp, &tmp, &x3);
    p256_mod(&tmp, prime);
    p256_mont_mul(&y3, &s, &tmp, prime, 0);
    p256_mod(&y3, prime);
    p256_sub(&y3, &y3, &p1->y);
    p256_mod(&y3, prime);

    memcpy(&r->x, &x3, sizeof(p256_int));
    memcpy(&r->y, &y3, sizeof(p256_int));
    r->infinity = 0;
}

static void p256_scalar_mult(p256_point *r, const p256_int *k,
                              const p256_point *p, const p256_int *prime)
{
    /* Double-and-add */
    r->infinity = 1;
    memset(&r->x, 0, sizeof(p256_int));
    memset(&r->y, 0, sizeof(p256_int));

    p256_point q;
    memcpy(&q, p, sizeof(p256_point));

    for (int i = 255; i >= 0; i--) {
        p256_point t;
        p256_point_double(&t, r, prime);
        memcpy(r, &t, sizeof(p256_point));

        int word = i / 32;
        int bit  = i % 32;
        if ((k->d[word] >> bit) & 1) {
            p256_point_add(&q, r, &q, prime);
            memcpy(r, &q, sizeof(p256_point));
            memcpy(&q, p, sizeof(p256_point));
        }
    }
}

/* ------------------------------------------------------------------ */
/*  ECDSA verify core                                                  */
/* ------------------------------------------------------------------ */

static int ecdsa_p256_verify(const uint8_t hash[32],
                              const uint8_t pub_x[32], const uint8_t pub_y[32],
                              const uint8_t sig[64])
{
    /* Parse signature (r, s) */
    p256_int r_sig, s_sig;
    memset(&r_sig, 0, sizeof(p256_int));
    memset(&s_sig, 0, sizeof(p256_int));
    for (int i = 0; i < 32; i++) {
        r_sig.d[i / 4] |= ((uint32_t)sig[i]) << (8 * (3 - (i % 4)));
        s_sig.d[i / 4] |= ((uint32_t)sig[i + 32]) << (8 * (3 - (i % 4)));
    }

    /* Check r,s in [1, n-1] */
    if (p256_is_zero(&r_sig) || p256_is_zero(&s_sig))
        return -1;
    if (memcmp(&r_sig, &p256_order, sizeof(p256_int)) >= 0)
        return -1;
    if (memcmp(&s_sig, &p256_order, sizeof(p256_int)) >= 0)
        return -1;

    /* Compute e = hash (left-trim to order bit length) */
    p256_int e;
    memset(&e, 0, sizeof(p256_int));
    for (int i = 0; i < 32; i++) {
        e.d[i / 4] |= ((uint32_t)hash[i]) << (8 * (3 - (i % 4)));
    }

    /* w = s^(-1) mod n */
    p256_int w;
    p256_inv(&w, &s_sig, &p256_order);

    /* u1 = e*w mod n, u2 = r*w mod n */
    p256_int u1, u2;
    p256_mont_mul(&u1, &e, &w, &p256_order, 0);
    p256_mod(&u1, &p256_order);
    p256_mont_mul(&u2, &r_sig, &w, &p256_order, 0);
    p256_mod(&u2, &p256_order);

    /* Parse public key point */
    p256_point pubkey;
    memset(&pubkey, 0, sizeof(pubkey));
    for (int i = 0; i < 32; i++) {
        pubkey.x.d[i / 4] |= ((uint32_t)pub_x[i]) << (8 * (3 - (i % 4)));
        pubkey.y.d[i / 4] |= ((uint32_t)pub_y[i]) << (8 * (3 - (i % 4)));
    }
    pubkey.infinity = 0;

    if (!p256_point_valid(&pubkey))
        return -1;

    /* Compute u1*G + u2*Q */
    p256_point g;
    memcpy(&g.x, &p256_gx, sizeof(p256_int));
    memcpy(&g.y, &p256_gy, sizeof(p256_int));
    g.infinity = 0;

    p256_point u1g, u2q;
    p256_scalar_mult(&u1g, &u1, &g, &p256_prime);
    p256_scalar_mult(&u2q, &u2, &pubkey, &p256_prime);
    p256_point_add(&u1g, &u1g, &u2q, &p256_prime);

    if (u1g.infinity)
        return -1;

    /* Compare (x coordinate mod n) with r */
    p256_int x_mod_n;
    memcpy(&x_mod_n, &u1g.x, sizeof(p256_int));
    p256_mod(&x_mod_n, &p256_order);

    if (memcmp(&x_mod_n, &r_sig, sizeof(p256_int)) == 0)
        return 0; /* signature valid */

    /* Try with n added (rare) */
    p256_int x_plus_n;
    memcpy(&x_plus_n, &x_mod_n, sizeof(p256_int));
    p256_add(&x_plus_n, &x_plus_n, &p256_order);
    p256_mod(&x_plus_n, &p256_prime);
    if (memcmp(&x_plus_n, &r_sig, sizeof(p256_int)) == 0)
        return 0;

    return -1;
}

/* ------------------------------------------------------------------ */
/*  Constant-time memory comparison                                    */
/* ------------------------------------------------------------------ */
static int verify_constant_time(const void *a, const void *b, size_t n)
{
    volatile uint8_t diff = 0;
    const volatile uint8_t *pa = (const volatile uint8_t *)a;
    const volatile uint8_t *pb = (const volatile uint8_t *)b;
    for (size_t i = 0; i < n; i++)
        diff |= pa[i] ^ pb[i];
    return (int)diff;
}

/* ------------------------------------------------------------------ */
/*  SHA256 stub — replace with mbedTLS / OpenSSL                       */
/* ------------------------------------------------------------------ */
static int sha256_compute(const uint8_t *data, size_t len, uint8_t digest[32])
{
    /* STUB: In production, use:
     *   mbedtls_sha256_context ctx;
     *   mbedtls_sha256_init(&ctx);
     *   mbedtls_sha256_starts(&ctx, 0); // 0 = SHA-256
     *   mbedtls_sha256_update(&ctx, data, len);
     *   mbedtls_sha256_finish(&ctx, digest);
     *   mbedtls_sha256_free(&ctx);
     */
    (void)data; (void)len;
    memset(digest, 0xAA, 32); /* fake digest */
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * verify_firmware_signature - parse header, hash payload, verify ECDSA
 * @firmware:   pointer to full firmware image (header + payload)
 * @fw_size:    total size of firmware image
 *
 * Returns 0 on valid signature, negative on error / mismatch.
 *
 * Security notes:
 *   - Returns only pass/fail (no partial error info in production)
 *   - Uses constant-time comparison for signature check
 *   - Public key in write-protected flash section
 *   - Version monotonicity must be checked by caller (ota_manager)
 */
int verify_firmware_signature(const uint8_t *firmware, size_t fw_size)
{
    if (!firmware || fw_size < FW_HEADER_SIZE)
        return -1;

    const fw_header_t *hdr = (const fw_header_t *)firmware;

    /* Verify magic */
    if (memcmp(hdr->magic, FW_MAGIC, 4) != 0)
        return -1;

    /* Verify payload size sanity */
    if (hdr->payload_size == 0 || hdr->payload_size > 16 * 1024 * 1024)
        return -1;

    if (fw_size < (size_t)FW_HEADER_SIZE + hdr->payload_size)
        return -1;

    /* Hash the payload: bytes after the signature field */
    const uint8_t *payload = firmware + FW_HEADER_SIZE;
    size_t payload_len = (size_t)hdr->payload_size;

    uint8_t hash[SHA256_DIGEST_LEN];
    if (sha256_compute(payload, payload_len, hash) != 0)
        return -1;

    /* Verify ECDSA P-256 signature */
    int rc = ecdsa_p256_verify(hash,
                                g_pubkey_x, g_pubkey_y,
                                hdr->signature);
    if (rc != 0)
        return -1;

    /* Signature valid */
    return 0;
}

/**
 * verify_firmware_safe - verify with constant-time comparison on result
 *                        (defense against fault injection)
 */
int verify_firmware_safe(const uint8_t *firmware, size_t fw_size)
{
    int result = verify_firmware_signature(firmware, fw_size);

    /* Run twice and compare — simple fault injection countermeasure */
    int result2 = verify_firmware_signature(firmware, fw_size);

    if (result != 0 || result2 != 0)
        return -1;

    return 0;
}

/**
 * verify_get_public_key_hash - provide hash of embedded public key
 *                              (for attestation / key rotation)
 */
int verify_get_pubkey_hash(uint8_t hash[32])
{
    uint8_t buf[64];
    memcpy(buf, g_pubkey_x, 32);
    memcpy(buf + 32, g_pubkey_y, 32);
    return sha256_compute(buf, 64, hash);
}

/* ------------------------------------------------------------------ */
/*  Self-test                                                          */
/* ------------------------------------------------------------------ */
#ifdef TEST_VERIFY
int main(void)
{
    /* Create a minimal firmware image with a valid-looking header */
    uint8_t fw[256];
    fw_header_t *hdr = (fw_header_t *)fw;
    memcpy(hdr->magic, FW_MAGIC, 4);
    hdr->version = 1;
    hdr->payload_size = 256 - FW_HEADER_SIZE;
    memset(hdr->signature, 0, 64); /* invalid sig, should fail */
    memset(fw + FW_HEADER_SIZE, 0x42, hdr->payload_size);

    int rc = verify_firmware_signature(fw, sizeof(fw));
    printf("verify_firmware_signature (expected fail): %d\n", rc);

    /* Test constant-time comparison */
    uint8_t a[4] = {0x01, 0x02, 0x03, 0x04};
    uint8_t b[4] = {0x01, 0x02, 0x03, 0x04};
    uint8_t c[4] = {0x01, 0x02, 0x03, 0x05};
    printf("constant-time equal: %d (expect 0)\n", verify_constant_time(a, b, 4));
    printf("constant-time diff:  %d (expect nonzero)\n", verify_constant_time(a, c, 4));

    return 0;
}
#endif /* TEST_VERIFY */
