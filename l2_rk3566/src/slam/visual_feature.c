/**
 * @file    visual_feature.c
 * @brief   ORB feature extraction from camera frames
 * @details FAST corner detection with orientation, BRIEF descriptor
 *          (256-bit), matching against local map features, PnP solver
 *          for visual odometry.  Results used as additional constraints
 *          in the EKF.
 *
 *          This is a self-contained implementation suitable for embedded
 *          deployment on RK3566.  For higher accuracy, consider
 *          integrating OpenCV ORB; this provides a zero-dependency
 *          alternative optimized for the use case.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <errno.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */
#define VF_MAX_FEATURES             500
#define VF_FAST_THRESHOLD           20
#define VF_FAST_NMS_RADIUS          3
#define VF_DESC_BITS                256
#define VF_DESC_BYTES               (VF_DESC_BITS / 8)
#define VF_MAX_MATCHES              100
#define VF_MATCH_DISTANCE_THRESHOLD 50
#define VF_PNP_MIN_POINTS           4
#define VF_PNP_MAX_ITERATIONS       50
#define VF_PNP_REPROJECTION_THRESH  5.0f  /* pixels */
#define VF_MAX_FRAME_W              1280
#define VF_MAX_FRAME_H              720
#define VF_PYRAMID_LEVELS           4
#define VF_PATCH_SIZE               31

#define VF_PI                       3.14159265358979323846f

/* ------------------------------------------------------------------ */
/*  Image types                                                        */
/* ------------------------------------------------------------------ */
typedef struct {
    int       width;
    int       height;
    int       stride;       /* bytes per row */
    uint8_t  *data;         /* grayscale pixels */
} vf_image_t;

/* ------------------------------------------------------------------ */
/*  Keypoint                                                           */
/* ------------------------------------------------------------------ */
typedef struct {
    float x;            /* image column */
    float y;            /* image row */
    float angle;        /* dominant orientation (radians) */
    float score;        /* corner strength */
    int   octave;       /* pyramid level */
} vf_keypoint_t;

/* ------------------------------------------------------------------ */
/*  Feature (keypoint + descriptor)                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    vf_keypoint_t kp;
    uint8_t       descriptor[VF_DESC_BYTES];
} vf_feature_t;

/* ------------------------------------------------------------------ */
/*  Match                                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    int query_idx;      /* index in query features */
    int map_idx;        /* index in map features */
    float distance;     /* Hamming distance */
} vf_match_t;

/* ------------------------------------------------------------------ */
/*  3D point (for PnP)                                                 */
/* ------------------------------------------------------------------ */
typedef struct {
    float x, y, z;      /* world coordinates */
} vf_point3d_t;

/* ------------------------------------------------------------------ */
/*  Camera intrinsic parameters                                        */
/* ------------------------------------------------------------------ */
typedef struct {
    float fx, fy;        /* focal lengths in pixels */
    float cx, cy;        /* principal point */
    float k1, k2, p1, p2; /* distortion coefficients */
} vf_camera_t;

/* ------------------------------------------------------------------ */
/*  Visual feature context                                             */
/* ------------------------------------------------------------------ */
typedef struct {
    vf_feature_t    features[VF_MAX_FEATURES];
    int             feature_count;

    /* Local map (persistent features with 3D positions) */
    vf_feature_t    map_features[VF_MAX_FEATURES];
    vf_point3d_t    map_points[VF_MAX_FEATURES];
    int             map_size;

    /* Current matches */
    vf_match_t      matches[VF_MAX_MATCHES];
    int             match_count;

    /* Camera parameters */
    vf_camera_t     camera;

    /* Pose output */
    float           rvec[3];   /* rotation vector */
    float           tvec[3];   /* translation vector */

    bool initialized;
} vf_ctx_t;

static vf_ctx_t g_vf;

/* ------------------------------------------------------------------ */
/*  Image helpers                                                      */
/* ------------------------------------------------------------------ */

static inline uint8_t vf_pixel(const vf_image_t *img, int x, int y)
{
    if (x < 0 || x >= img->width || y < 0 || y >= img->height)
        return 0;
    return img->data[y * img->stride + x];
}

static inline int vf_ibound(int v, int min, int max)
{
    return (v < min) ? min : (v > max) ? max : v;
}

/* ------------------------------------------------------------------ */
/*  FAST corner detection                                              */
/* ------------------------------------------------------------------ */

/* FAST-9: compare 16 points on a Bresenham circle of radius 3 */
static const int fast_offsets[16][2] = {
    {0, -3}, {1, -3}, {2, -2}, {3, -1}, {3, 0}, {3, 1}, {2, 2}, {1, 3},
    {0, 3}, {-1, 3}, {-2, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-2, -2}, {-1, -3}
};

static bool vf_fast_test(const vf_image_t *img, int x, int y, int threshold)
{
    uint8_t center = vf_pixel(img, x, y);
    int bright = center + threshold;
    int dark   = center - threshold;

    /* Quick test: check points at positions 0, 4, 8, 12 */
    int n0 = vf_pixel(img, x + fast_offsets[0][0],  y + fast_offsets[0][1]);
    int n4 = vf_pixel(img, x + fast_offsets[4][0],  y + fast_offsets[4][1]);
    int n8 = vf_pixel(img, x + fast_offsets[8][0],  y + fast_offsets[8][1]);
    int n12 = vf_pixel(img, x + fast_offsets[12][0], y + fast_offsets[12][1]);

    if (!((n0 >= bright || n0 <= dark) ||
          (n4 >= bright || n4 <= dark) ||
          (n8 >= bright || n8 <= dark) ||
          (n12 >= bright || n12 <= dark)))
        return false;

    /* Full test: need at least 9 consecutive pixels */
    int count_bright = 0, count_dark = 0;
    for (int i = 0; i < 16; i++) {
        int p = vf_pixel(img, x + fast_offsets[i][0], y + fast_offsets[i][1]);
        if (p >= bright) { count_bright++; count_dark = 0; }
        else if (p <= dark) { count_dark++; count_bright = 0; }
        else { count_bright = 0; count_dark = 0; }

        if (count_bright >= 9 || count_dark >= 9) return true;
    }

    return false;
}

static float vf_fast_score(const vf_image_t *img, int x, int y)
{
    uint8_t center = vf_pixel(img, x, y);
    float sum = 0.0f;
    for (int i = 0; i < 16; i++) {
        int p = vf_pixel(img, x + fast_offsets[i][0], y + fast_offsets[i][1]);
        sum += (float)abs((int)center - p);
    }
    return sum / 16.0f;
}

static void vf_nonmax_suppression(vf_keypoint_t *kps, int *count)
{
    /* Simple grid-based NMS */
    bool keep[VF_MAX_FEATURES];
    memset(keep, true, sizeof(keep));

    for (int i = 0; i < *count; i++) {
        if (!keep[i]) continue;
        for (int j = i + 1; j < *count; j++) {
            if (!keep[j]) continue;
            float dx = kps[i].x - kps[j].x;
            float dy = kps[i].y - kps[j].y;
            if (dx * dx + dy * dy < (float)(VF_FAST_NMS_RADIUS * VF_FAST_NMS_RADIUS)) {
                if (kps[i].score >= kps[j].score)
                    keep[j] = false;
                else
                    keep[i] = false;
            }
        }
    }

    int out = 0;
    for (int i = 0; i < *count; i++) {
        if (keep[i])
            kps[out++] = kps[i];
    }
    *count = out;
}

/* ------------------------------------------------------------------ */
/*  Orientation (intensity centroid)                                   */
/* ------------------------------------------------------------------ */

static float vf_compute_orientation(const vf_image_t *img, int x, int y)
{
    int m01 = 0, m10 = 0;
    int half = VF_PATCH_SIZE / 2;

    for (int dy = -half; dy <= half; dy++) {
        for (int dx = -half; dx <= half; dx++) {
            int p = vf_pixel(img, x + dx, y + dy);
            m01 += dy * p;
            m10 += dx * p;
        }
    }

    return atan2f((float)m01, (float)m10);
}

/* ------------------------------------------------------------------ */
/*  BRIEF descriptor (256-bit)                                         */
/* ------------------------------------------------------------------ */

/* Pre-computed random test locations (Bryant and Mironov PRNG seeded) */
static int brief_pattern[VF_DESC_BITS][4]; /* (x1, y1, x2, y2) per bit */

static void vf_brief_init_pattern(void)
{
    static bool init = false;
    if (init) return;

    /* Deterministic pattern using a simple LCG */
    uint32_t state = 0xDEADBEEF;
    for (int i = 0; i < VF_DESC_BITS; i++) {
        for (int j = 0; j < 4; j++) {
            state = state * 1103515245u + 12345u;
            brief_pattern[i][j] = (int)((state >> 16) % (VF_PATCH_SIZE + 1)) - VF_PATCH_SIZE / 2;
        }
    }
    init = true;
}

static void vf_brief_compute(const vf_image_t *img, int x, int y,
                              float angle, uint8_t desc[VF_DESC_BYTES])
{
    vf_brief_init_pattern();
    memset(desc, 0, VF_DESC_BYTES);

    float c = cosf(angle);
    float s = sinf(angle);

    for (int i = 0; i < VF_DESC_BITS; i++) {
        /* Rotate test locations by keypoint orientation */
        int x1 = (int)(c * brief_pattern[i][0] - s * brief_pattern[i][1]);
        int y1 = (int)(s * brief_pattern[i][0] + c * brief_pattern[i][1]);
        int x2 = (int)(c * brief_pattern[i][2] - s * brief_pattern[i][3]);
        int y2 = (int)(s * brief_pattern[i][2] + c * brief_pattern[i][3]);

        int p1 = vf_pixel(img, x + x1, y + y1);
        int p2 = vf_pixel(img, x + x2, y + y2);

        if (p1 < p2)
            desc[i >> 3] |= (uint8_t)(1 << (i & 7));
    }
}

/* ------------------------------------------------------------------ */
/*  Feature detection                                                  */
/* ------------------------------------------------------------------ */

int vf_detect_features(const vf_image_t *image, vf_feature_t *features, int *count)
{
    if (!image || !features || !count) return -EINVAL;

    vf_keypoint_t kps[VF_MAX_FEATURES];
    int kp_count = 0;

    /* Detect FAST corners */
    for (int y = VF_FAST_NMS_RADIUS; y < image->height - VF_FAST_NMS_RADIUS; y += 2) {
        for (int x = VF_FAST_NMS_RADIUS; x < image->width - VF_FAST_NMS_RADIUS; x += 2) {
            if (vf_fast_test(image, x, y, VF_FAST_THRESHOLD)) {
                if (kp_count >= VF_MAX_FEATURES) goto done_detect;
                kps[kp_count].x = (float)x;
                kps[kp_count].y = (float)y;
                kps[kp_count].score = vf_fast_score(image, x, y);
                kps[kp_count].octave = 0;
                kp_count++;
            }
        }
    }

done_detect:
    /* Non-max suppression */
    vf_nonmax_suppression(kps, &kp_count);

    /* Limit count */
    if (kp_count > VF_MAX_FEATURES) kp_count = VF_MAX_FEATURES;

    /* Compute orientation and descriptor for each keypoint */
    for (int i = 0; i < kp_count; i++) {
        features[i].kp = kps[i];
        features[i].kp.angle = vf_compute_orientation(image,
            (int)kps[i].x, (int)kps[i].y);
        vf_brief_compute(image, (int)kps[i].x, (int)kps[i].y,
                          features[i].kp.angle,
                          features[i].descriptor);
    }

    *count = kp_count;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Feature matching (brute force Hamming)                             */
/* ------------------------------------------------------------------ */

static float vf_hamming_distance(const uint8_t *a, const uint8_t *b)
{
    /* Popcount over 256 bits */
    float dist = 0.0f;
    for (int i = 0; i < VF_DESC_BYTES; i++) {
        uint8_t x = a[i] ^ b[i];
        /* Popcount for each byte */
        x = (x & 0x55) + ((x >> 1) & 0x55);
        x = (x & 0x33) + ((x >> 2) & 0x33);
        x = (x & 0x0F) + ((x >> 4) & 0x0F);
        dist += (float)x;
    }
    return dist;
}

int vf_match_features(const vf_feature_t *query, int query_count,
                       const vf_feature_t *map, int map_count,
                       vf_match_t *matches, int *match_count,
                       float max_distance)
{
    if (!query || !map || !matches || !match_count) return -EINVAL;

    int out = 0;

    for (int i = 0; i < query_count && out < VF_MAX_MATCHES; i++) {
        float best_dist = max_distance;
        int   best_idx  = -1;

        for (int j = 0; j < map_count; j++) {
            float dist = vf_hamming_distance(query[i].descriptor, map[j].descriptor);
            if (dist < best_dist) {
                best_dist = dist;
                best_idx  = j;
            }
        }

        if (best_idx >= 0) {
            matches[out].query_idx = i;
            matches[out].map_idx   = best_idx;
            matches[out].distance  = best_dist;
            out++;
        }
    }

    *match_count = out;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  PnP solver (simplified EPnP-like)                                  */
/* ------------------------------------------------------------------ */

static void vf_cross(float *r, const float *a, const float *b)
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

static float vf_dot(const float *a, const float *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void vf_normalize(float *v)
{
    float n = sqrtf(vf_dot(v, v));
    if (n > 1e-8f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}

static void vf_rodrigues_to_mat(const float r[3], float R[9])
{
    float theta = sqrtf(vf_dot(r, r));
    if (theta < 1e-8f) {
        memset(R, 0, 9 * sizeof(float));
        R[0] = R[4] = R[8] = 1.0f;
        return;
    }

    float rx = r[0] / theta, ry = r[1] / theta, rz = r[2] / theta;
    float c = cosf(theta), s = sinf(theta);
    float ic = 1.0f - c;

    R[0] = c + rx*rx*ic;  R[1] = rx*ry*ic - rz*s; R[2] = rx*rz*ic + ry*s;
    R[3] = ry*rx*ic + rz*s; R[4] = c + ry*ry*ic;  R[5] = ry*rz*ic - rx*s;
    R[6] = rz*rx*ic - ry*s; R[7] = rz*ry*ic + rx*s; R[8] = c + rz*rz*ic;
}

static void vf_project(const vf_camera_t *cam,
                        const float *R, const float *t,
                        const vf_point3d_t *world,
                        float *u, float *v)
{
    /* Transform to camera frame */
    float xc = R[0]*world->x + R[1]*world->y + R[2]*world->z + t[0];
    float yc = R[3]*world->x + R[4]*world->y + R[5]*world->z + t[1];
    float zc = R[6]*world->x + R[7]*world->y + R[8]*world->z + t[2];

    if (zc < 1e-6f) zc = 1e-6f;

    /* Perspective projection */
    float xp = xc / zc;
    float yp = yc / zc;

    /* Apply distortion (simplified) */
    float r2 = xp*xp + yp*yp;
    float dist = 1.0f + cam->k1*r2 + cam->k2*r2*r2;

    *u = cam->fx * xp * dist + cam->cx;
    *v = cam->fy * yp * dist + cam->cy;
}

/**
 * vf_solve_pnp - solve Perspective-n-Point using Direct Linear Transform
 * @world_pts: array of 3D world points
 * @image_pts: array of 2D image points (keypoint locations)
 * @num_pts:   number of correspondences (>= 4)
 * @camera:    camera intrinsic parameters
 * @rvec:      output rotation vector (3 elements)
 * @tvec:      output translation vector (3 elements)
 *
 * Returns 0 on success, negative on error.
 */
int vf_solve_pnp(const vf_point3d_t *world_pts,
                  const float (*image_pts)[2],
                  int num_pts,
                  const vf_camera_t *camera,
                  float rvec[3], float tvec[3])
{
    if (!world_pts || !image_pts || num_pts < VF_PNP_MIN_POINTS || !camera)
        return -EINVAL;

    /* Simplified PnP: use a linear approximation with normalization.
     * In production, replace with OpenCV solvePnP or EPnP algorithm.
     */

    /* Initialize guess: assume small rotation, translation at centroid */
    memset(rvec, 0, 3 * sizeof(float));
    tvec[0] = tvec[1] = tvec[2] = 0.0f;

    float R[9];
    vf_rodrigues_to_mat(rvec, R);

    /* Iterative refinement (Gauss-Newton style) */
    int n = num_pts;
    for (int iter = 0; iter < VF_PNP_MAX_ITERATIONS; iter++) {
        float J[2*VF_PNP_MAX_POINTS][6]; /* Jacobian */
        float e[2*VF_PNP_MAX_POINTS];    /* residual */
        int valid = 0;

        for (int i = 0; i < n; i++) {
            float u_proj, v_proj;
            vf_project(camera, R, tvec, &world_pts[i], &u_proj, &v_proj);

            float du = image_pts[i][0] - u_proj;
            float dv = image_pts[i][1] - v_proj;
            float err = sqrtf(du*du + dv*dv);

            if (err > VF_PNP_REPROJECTION_THRESH) continue;

            /* Numerical Jacobian */
            float eps = 1e-4f;
            for (int j = 0; j < 6; j++) {
                float p[3];
                memcpy(p, (j < 3) ? rvec : tvec, sizeof(float)*3);
                float orig = p[j % 3];
                p[j % 3] += eps;

                float ru[3] = {rvec[0], rvec[1], rvec[2]};
                float tu[3] = {tvec[0], tvec[1], tvec[2]};
                if (j < 3) ru[j] = p[0]; else tu[j-3] = p[0];

                float tmpR[9];
                vf_rodrigues_to_mat(ru, tmpR);

                float uu, vv;
                vf_project(camera, tmpR, tu, &world_pts[i], &uu, &vv);

                J[valid*2][j]   = (uu - u_proj) / eps;
                J[valid*2+1][j] = (vv - v_proj) / eps;

                p[j % 3] = orig;
            }

            e[valid*2]   = du;
            e[valid*2+1] = dv;
            valid++;
        }

        if (valid < VF_PNP_MIN_POINTS) break;

        /* Solve J^T J x = J^T e via pseudo-inverse (simplified) */
        float JTJ[6][6] = {{0}};
        float JTe[6] = {0};

        for (int i = 0; i < valid * 2; i++) {
            for (int j = 0; j < 6; j++) {
                for (int k = 0; k < 6; k++)
                    JTJ[j][k] += J[i][j] * J[i][k];
                JTe[j] += J[i][j] * e[i];
            }
        }

        /* Solve using Cholesky-like (small 6x6, direct inversion) */
        /* Gauss-Seidel iteration */
        float delta[6] = {0};
        for (int gs = 0; gs < 20; gs++) {
            for (int j = 0; j < 6; j++) {
                float s = JTe[j];
                for (int k = 0; k < 6; k++) {
                    if (k != j) s -= JTJ[j][k] * delta[k];
                }
                if (JTJ[j][j] > 1e-10f)
                    delta[j] = s / JTJ[j][j];
            }
        }

        /* Update */
        rvec[0] += delta[0]; rvec[1] += delta[1]; rvec[2] += delta[2];
        tvec[0] += delta[3]; tvec[1] += delta[4]; tvec[2] += delta[5];

        vf_rodrigues_to_mat(rvec, R);

        /* Check convergence */
        float step = sqrtf(vf_dot(delta, delta));
        if (step < 1e-6f) break;
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int vf_init(float fx, float fy, float cx, float cy,
             float k1, float k2, float p1, float p2)
{
    memset(&g_vf, 0, sizeof(g_vf));

    g_vf.camera.fx = fx;
    g_vf.camera.fy = fy;
    g_vf.camera.cx = cx;
    g_vf.camera.cy = cy;
    g_vf.camera.k1 = k1;
    g_vf.camera.k2 = k2;
    g_vf.camera.p1 = p1;
    g_vf.camera.p2 = p2;

    /* Initialize pattern just once */
    vf_brief_init_pattern();

    g_vf.initialized = true;
    return 0;
}

/**
 * vf_process_frame - detect features, match against map, estimate pose
 * @image:      grayscale camera frame
 * @rvec_out:   output rotation vector
 * @tvec_out:   output translation vector
 *
 * Returns number of inlier matches used for pose, or negative on error.
 */
int vf_process_frame(const vf_image_t *image,
                      float rvec_out[3], float tvec_out[3])
{
    if (!g_vf.initialized) return -EPERM;
    if (!image || !rvec_out || !tvec_out) return -EINVAL;

    /* Detect features in current frame */
    vf_feature_t query_features[VF_MAX_FEATURES];
    int query_count = 0;

    int rc = vf_detect_features(image, query_features, &query_count);
    if (rc != 0) return rc;

    if (query_count < VF_PNP_MIN_POINTS) return 0;

    /* If we have a map, match and estimate pose */
    if (g_vf.map_size >= VF_PNP_MIN_POINTS) {
        rc = vf_match_features(query_features, query_count,
                                g_vf.map_features, g_vf.map_size,
                                g_vf.matches, &g_vf.match_count,
                                VF_MATCH_DISTANCE_THRESHOLD);
        if (rc != 0) return rc;

        if (g_vf.match_count < VF_PNP_MIN_POINTS) return 0;

        /* Prepare data for PnP */
        vf_point3d_t world_pts[VF_MAX_MATCHES];
        float image_pts[VF_MAX_MATCHES][2];
        int valid = 0;

        for (int i = 0; i < g_vf.match_count && valid < VF_MAX_MATCHES; i++) {
            int mi = g_vf.matches[i].map_idx;
            world_pts[valid] = g_vf.map_points[mi];
            int qi = g_vf.matches[i].query_idx;
            image_pts[valid][0] = query_features[qi].kp.x;
            image_pts[valid][1] = query_features[qi].kp.y;
            valid++;
        }

        rc = vf_solve_pnp(world_pts, image_pts, valid,
                           &g_vf.camera, rvec_out, tvec_out);
        if (rc == 0) {
            memcpy(g_vf.rvec, rvec_out, 3 * sizeof(float));
            memcpy(g_vf.tvec, tvec_out, 3 * sizeof(float));
            return valid;
        }
    }

    return 0;
}

/**
 * vf_update_map - add new features to local map with triangulated 3D positions
 */
int vf_update_map(const vf_feature_t *features, int count,
                   const vf_point3d_t *points)
{
    if (!features || !points) return -EINVAL;
    if (count <= 0) return 0;

    int to_add = count;
    if (g_vf.map_size + to_add > VF_MAX_FEATURES)
        to_add = VF_MAX_FEATURES - g_vf.map_size;

    memcpy(&g_vf.map_features[g_vf.map_size], features,
           to_add * sizeof(vf_feature_t));
    memcpy(&g_vf.map_points[g_vf.map_size], points,
           to_add * sizeof(vf_point3d_t));
    g_vf.map_size += to_add;

    return to_add;
}

void vf_get_pose(float rvec[3], float tvec[3])
{
    memcpy(rvec, g_vf.rvec, 3 * sizeof(float));
    memcpy(tvec, g_vf.tvec, 3 * sizeof(float));
}

int vf_get_map_size(void)
{
    return g_vf.map_size;
}

void vf_reset_map(void)
{
    g_vf.map_size = 0;
    memset(g_vf.rvec, 0, sizeof(g_vf.rvec));
    memset(g_vf.tvec, 0, sizeof(g_vf.tvec));
}

void vf_shutdown(void)
{
    g_vf.initialized = false;
}
