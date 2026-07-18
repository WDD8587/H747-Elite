/**
 * @file    rk_verify.h
 * @brief   RK3566 FIT image verification API.
 */
#ifndef RK_VERIFY_H
#define RK_VERIFY_H

#include <stdint.h>

/* Error codes */
#define RK_VERIFY_ERR_OK           0
#define RK_VERIFY_ERR_IO          -1
#define RK_VERIFY_ERR_SIZE        -2
#define RK_VERIFY_ERR_NOMEM       -3
#define RK_VERIFY_ERR_FMT         -4
#define RK_VERIFY_ERR_HASH_MISSING   -5
#define RK_VERIFY_ERR_DATA_MISSING  -6
#define RK_VERIFY_ERR_HASH_COMPUTE  -7
#define RK_VERIFY_ERR_KERNEL      -8
#define RK_VERIFY_ERR_DTB         -9
#define RK_VERIFY_ERR_ROOTFS      -10

int         rk_verify_fit_image(const char *image_path);
int         rk_verify_default_fit(void);
const char *rk_verify_strerror(int err);

#endif /* RK_VERIFY_H */
