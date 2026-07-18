/**
 * @file    loop_closure.c
 * @brief   Loop closure detection via scan matching
 * @details Compares current LIDAR scan with all past scans within a 5m
 *          radius.  Uses correlative scan matching with multi-resolution
 *          (coarse 20cm -> fine 5cm).  Adds loop closure constraints to
 *          pose graph.  Optimizes with sparse Cholesky factorization.
 *
 *          This module provides loop closure detection only; the pose
 *          graph and optimization framework is in the SLAM core.
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
#define LC_MAX_SCANS_TO_SEARCH      500
#define LC_SEARCH_RADIUS_M          5.0
#define LC_COARSE_RESOLUTION_M      0.20
#define LC_FINE_RESOLUTION_M        0.05
#define LC_MIN_MATCH_POINTS         50
#define LC_MIN_MATCH_SCORE          0.75
#define LC_MAX_SCAN_POINTS          4096
#define LC_MAX_RANGE_M              30.0
#define LC_THETA_STEPS              360      /* angular steps for coarse search */

#define LC_PI                        3.14159265358979323846
#define LC_DEG2RAD(d)               ((d) * LC_PI / 180.0)

/* ------------------------------------------------------------------ */
/*  2D point                                                          */
/* ------------------------------------------------------------------ */
typedef struct {
    float x;
    float y;
    float theta;   /* orientation, radians */
} lc_pose2d_t;

typedef struct {
    float x;
    float y;
} lc_point2d_t;

typedef struct {
    lc_point2d_t points[LC_MAX_SCAN_POINTS];
    int          count;
    lc_pose2d_t  origin;   /* sensor pose when scan was taken */
    uint32_t     id;       /* scan ID */
    float        timestamp;
} lc_scan_t;

/* ------------------------------------------------------------------ */
/*  Correlation grid cell                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    float  score;
    int    count;
} lc_grid_cell_t;

/* ------------------------------------------------------------------ */
/*  Correlation grid (multi-resolution)                                */
/* ------------------------------------------------------------------ */
typedef struct {
    float    resolution;      /* cell size in meters */
    int      width;           /* columns */
    int      height;          /* rows */
    float    origin_x;        /* world coords of center */
    float    origin_y;
    lc_grid_cell_t *cells;    /* width x height */
} lc_grid_t;

/* ------------------------------------------------------------------ */
/*  Loop closure candidate                                             */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t   query_scan_id;
    uint32_t   match_scan_id;
    lc_pose2d_t relative_pose; /* transform from query to match */
    float      score;          /* matching score (0..1) */
    bool       accepted;       /* finalized as loop closure */
} lc_candidate_t;

/* ------------------------------------------------------------------ */
/*  Loop closure context                                               */
/* ------------------------------------------------------------------ */
typedef struct {
    lc_scan_t    *scan_history;       /* circular buffer of past scans */
    int           scan_history_max;
    int           scan_history_count;
    int           scan_history_head;
    int           scan_history_tail;
    uint32_t      next_scan_id;
    lc_candidate_t candidates[64];
    int           candidate_count;

    /* Statistics */
    uint32_t      loops_detected;
    uint32_t      loops_accepted;

    bool initialized;
} lc_ctx_t;

static lc_ctx_t g_lc;

/* ------------------------------------------------------------------ */
/*  Math helpers                                                       */
/* ------------------------------------------------------------------ */

static inline float lc_min(float a, float b) { return a < b ? a : b; }
static inline float lc_max(float a, float b) { return a > b ? a : b; }
static inline float lc_abs(float a) { return a < 0 ? -a : a; }
static inline int   lc_imin(int a, int b) { return a < b ? a : b; }

static float lc_normalize_angle(float a)
{
    while (a > LC_PI) a -= 2.0f * (float)LC_PI;
    while (a < -LC_PI) a += 2.0f * (float)LC_PI;
    return a;
}

static void lc_transform_point(lc_point2d_t *out, const lc_point2d_t *in,
                                const lc_pose2d_t *pose)
{
    float c = cosf(pose->theta);
    float s = sinf(pose->theta);
    out->x = c * in->x - s * in->y + pose->x;
    out->y = s * in->x + c * in->y + pose->y;
}

static float lc_distance(const lc_pose2d_t *a, const lc_pose2d_t *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    return sqrtf(dx * dx + dy * dy);
}

/* ------------------------------------------------------------------ */
/*  Correlation grid helpers                                           */
/* ------------------------------------------------------------------ */

static lc_grid_t *lc_grid_create(float resolution, int width, int height,
                                  float origin_x, float origin_y)
{
    lc_grid_t *g = (lc_grid_t *)calloc(1, sizeof(lc_grid_t));
    if (!g) return NULL;

    g->resolution = resolution;
    g->width      = width;
    g->height     = height;
    g->origin_x   = origin_x;
    g->origin_y   = origin_y;
    g->cells      = (lc_grid_cell_t *)calloc(width * height, sizeof(lc_grid_cell_t));

    if (!g->cells) { free(g); return NULL; }
    return g;
}

static void lc_grid_destroy(lc_grid_t *g)
{
    if (g) {
        free(g->cells);
        free(g);
    }
}

static int lc_grid_world_to_cell(const lc_grid_t *g, float wx, float wy,
                                  int *cx, int *cy)
{
    *cx = (int)((wx - g->origin_x) / g->resolution + g->width / 2);
    *cy = (int)((wy - g->origin_y) / g->resolution + g->height / 2);
    return (*cx >= 0 && *cx < g->width && *cy >= 0 && *cy < g->height) ? 0 : -1;
}

static void lc_grid_clear(lc_grid_t *g)
{
    if (g && g->cells)
        memset(g->cells, 0, g->width * g->height * sizeof(lc_grid_cell_t));
}

static void lc_grid_add_scan(lc_grid_t *g, const lc_scan_t *scan,
                              const lc_pose2d_t *pose)
{
    for (int i = 0; i < scan->count; i++) {
        lc_point2d_t world_pt;
        lc_transform_point(&world_pt, &scan->points[i], pose);

        int cx, cy;
        if (lc_grid_world_to_cell(g, world_pt.x, world_pt.y, &cx, &cy) == 0) {
            int idx = cy * g->width + cx;
            g->cells[idx].score += 1.0f;
            g->cells[idx].count++;
        }
    }
}

static float lc_grid_score_scan(const lc_grid_t *g, const lc_scan_t *scan,
                                 const lc_pose2d_t *pose)
{
    float total_score = 0.0f;
    int   hits = 0;

    for (int i = 0; i < scan->count; i++) {
        lc_point2d_t world_pt;
        lc_transform_point(&world_pt, &scan->points[i], pose);

        int cx, cy;
        if (lc_grid_world_to_cell(g, world_pt.x, world_pt.y, &cx, &cy) == 0) {
            int idx = cy * g->width + cx;
            if (g->cells[idx].count > 0) {
                total_score += g->cells[idx].score / (float)g->cells[idx].count;
                hits++;
            }
        }
    }

    if (hits < LC_MIN_MATCH_POINTS) return 0.0f;
    return total_score / (float)hits;
}

/* ------------------------------------------------------------------ */
/*  Correlative scan matching                                          */
/* ------------------------------------------------------------------ */

/**
 * Correlative scan matching at a single resolution.
 * Searches over a 2D translation window around the initial guess.
 */
static float lc_correlate_at_resolution(
    const lc_grid_t *grid, const lc_scan_t *scan,
    const lc_pose2d_t *initial_guess,
    float search_window_m, float resolution,
    lc_pose2d_t *best_pose)
{
    int steps = (int)(search_window_m / resolution);
    if (steps < 1) steps = 1;

    float best_score = 0.0f;
    memcpy(best_pose, initial_guess, sizeof(lc_pose2d_t));

    for (int dx = -steps; dx <= steps; dx++) {
        for (int dy = -steps; dy <= steps; dy++) {
            lc_pose2d_t candidate;
            candidate.x     = initial_guess->x + dx * resolution;
            candidate.y     = initial_guess->y + dy * resolution;
            candidate.theta = initial_guess->theta; /* fixed for this pass */

            float score = lc_grid_score_scan(grid, scan, &candidate);
            if (score > best_score) {
                best_score = score;
                *best_pose = candidate;
            }
        }
    }

    return best_score;
}

/**
 * lc_match_scan - match current scan against a grid built from a past scan
 * @current:   current LIDAR scan
 * @past:      past scan to match against
 * @out_pose:  output: relative pose between scans
 *
 * Returns match score (0..1).  0 = no match.
 */
static float lc_match_scan(const lc_scan_t *current, const lc_scan_t *past,
                            lc_pose2d_t *out_pose)
{
    /* Build correlation grid from past scan at its origin */
    float search_window = 2.0f; /* +/- 2m search */
    int grid_size = (int)(search_window / LC_COARSE_RESOLUTION_M) * 2 + 50;

    lc_grid_t *grid = lc_grid_create(
        LC_COARSE_RESOLUTION_M, grid_size, grid_size,
        past->origin.x, past->origin.y);

    if (!grid) return 0.0f;

    lc_grid_clear(grid);
    lc_grid_add_scan(grid, past, &past->origin);

    /* Initial guess: assume same pose */
    lc_pose2d_t guess = past->origin;
    lc_pose2d_t best_pose;
    float score;

    /* Coarse search */
    score = lc_correlate_at_resolution(
        grid, current, &guess,
        search_window, LC_COARSE_RESOLUTION_M, &best_pose);

    lc_grid_destroy(grid);

    if (score < LC_MIN_MATCH_SCORE * 0.5f) {
        return 0.0f;
    }

    /* Fine search with higher resolution grid */
    grid_size = (int)(search_window / LC_FINE_RESOLUTION_M) * 2 + 50;
    grid = lc_grid_create(
        LC_FINE_RESOLUTION_M, grid_size, grid_size,
        past->origin.x, past->origin.y);

    if (!grid) return score;

    lc_grid_clear(grid);
    lc_grid_add_scan(grid, past, &past->origin);

    /* Angular search at 1 degree increments */
    float best_angular_score = 0.0f;
    lc_pose2d_t angular_best = best_pose;

    for (int theta_step = -5; theta_step <= 5; theta_step++) {
        lc_pose2d_t ang_guess = best_pose;
        ang_guess.theta += (float)theta_step * LC_DEG2RAD(1.0f);

        lc_pose2d_t fine_pose;
        float s = lc_correlate_at_resolution(
            grid, current, &ang_guess,
            search_window * 0.5f, LC_FINE_RESOLUTION_M, &fine_pose);

        if (s > best_angular_score) {
            best_angular_score = s;
            angular_best = fine_pose;
        }
    }

    lc_grid_destroy(grid);

    score = best_angular_score;

    if (score >= LC_MIN_MATCH_SCORE) {
        /* Compute relative pose: transform from current origin to past origin */
        out_pose->x     = angular_best.x - current->origin.x;
        out_pose->y     = angular_best.y - current->origin.y;
        out_pose->theta = lc_normalize_angle(angular_best.theta - current->origin.theta);
    }

    return score;
}

/* ------------------------------------------------------------------ */
/*  Scan history management                                            */
/* ------------------------------------------------------------------ */

static void lc_add_scan_to_history(const lc_scan_t *scan)
{
    lc_scan_t *slot = &g_lc.scan_history[g_lc.scan_history_head];
    memcpy(slot, scan, sizeof(lc_scan_t));

    g_lc.scan_history_head = (g_lc.scan_history_head + 1) % g_lc.scan_history_max;

    if (g_lc.scan_history_count < g_lc.scan_history_max) {
        g_lc.scan_history_count++;
    } else {
        g_lc.scan_history_tail = (g_lc.scan_history_tail + 1) % g_lc.scan_history_max;
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int lc_init(void)
{
    memset(&g_lc, 0, sizeof(g_lc));

    g_lc.scan_history_max = LC_MAX_SCANS_TO_SEARCH;
    g_lc.scan_history = (lc_scan_t *)calloc(LC_MAX_SCANS_TO_SEARCH, sizeof(lc_scan_t));
    if (!g_lc.scan_history) return -ENOMEM;

    g_lc.next_scan_id = 0;
    g_lc.initialized = true;
    return 0;
}

/**
 * lc_add_scan - add a new LIDAR scan and check for loop closures
 * @points:     scan points (range readings converted to cartesian)
 * @num_points: number of valid points
 * @origin:     sensor pose when scan was captured
 *
 * Returns number of new loop closure candidates found (0 if none).
 */
int lc_add_scan(const lc_point2d_t *points, int num_points,
                 const lc_pose2d_t *origin)
{
    if (!g_lc.initialized) return -EPERM;
    if (!points || num_points <= 0 || num_points > LC_MAX_SCAN_POINTS || !origin)
        return -EINVAL;

    /* Build scan object */
    lc_scan_t current_scan;
    current_scan.count = num_points;
    current_scan.origin = *origin;
    current_scan.id = g_lc.next_scan_id++;
    current_scan.timestamp = 0.0f; /* in production, use real timestamp */
    memcpy(current_scan.points, points, num_points * sizeof(lc_point2d_t));

    int new_candidates = 0;

    /* Search through scan history for potential loop closures */
    /* Only check scans within spatial proximity and temporal distance */
    for (int i = 0; i < g_lc.scan_history_count; i++) {
        int idx = (g_lc.scan_history_tail + i) % g_lc.scan_history_max;
        const lc_scan_t *past = &g_lc.scan_history[idx];

        /* Don't match against ourselves */
        if (past->id == current_scan.id) continue;

        /* Skip very recent scans (temporal distance > 50 scans) */
        if (current_scan.id - past->id < 50) continue;

        /* Spatial proximity check */
        float dist = lc_distance(origin, &past->origin);
        if (dist > LC_SEARCH_RADIUS_M) continue;

        /* Attempt scan matching */
        lc_pose2d_t relative;
        float score = lc_match_scan(&current_scan, past, &relative);

        if (score >= LC_MIN_MATCH_SCORE &&
            g_lc.candidate_count < (int)(sizeof(g_lc.candidates) / sizeof(g_lc.candidates[0]))) {

            lc_candidate_t *cand = &g_lc.candidates[g_lc.candidate_count++];
            cand->query_scan_id  = current_scan.id;
            cand->match_scan_id  = past->id;
            cand->relative_pose  = relative;
            cand->score          = score;
            cand->accepted       = false;

            g_lc.loops_detected++;
            new_candidates++;

            fprintf(stderr, "[LC] loop candidate: scan %u -> scan %u, "
                    "score=%.3f, dist=%.2fm\n",
                    current_scan.id, past->id, score, dist);
        }
    }

    /* Add current scan to history */
    lc_add_scan_to_history(&current_scan);

    return new_candidates;
}

/**
 * lc_get_candidates - get list of loop closure candidates
 */
int lc_get_candidates(const lc_candidate_t **candidates, int *count)
{
    if (!g_lc.initialized) return -EPERM;
    if (!candidates || !count) return -EINVAL;

    *candidates = g_lc.candidates;
    *count = g_lc.candidate_count;
    return 0;
}

/**
 * lc_accept_candidate - accept a loop closure (validated by pose graph)
 */
int lc_accept_candidate(int index)
{
    if (index < 0 || index >= g_lc.candidate_count)
        return -EINVAL;

    g_lc.candidates[index].accepted = true;
    g_lc.loops_accepted++;
    return 0;
}

/**
 * lc_reject_candidate - reject a false positive
 */
int lc_reject_candidate(int index)
{
    if (index < 0 || index >= g_lc.candidate_count)
        return -EINVAL;

    /* Remove by swapping with last */
    g_lc.candidates[index] = g_lc.candidates[--g_lc.candidate_count];
    return 0;
}

/**
 * lc_clear_candidates - clear all candidates after graph optimization
 */
void lc_clear_candidates(void)
{
    g_lc.candidate_count = 0;
}

/**
 * lc_get_stats - get loop closure statistics
 */
void lc_get_stats(uint32_t *detected, uint32_t *accepted,
                   int *history_count)
{
    if (detected)  *detected  = g_lc.loops_detected;
    if (accepted)  *accepted  = g_lc.loops_accepted;
    if (history_count) *history_count = g_lc.scan_history_count;
}

void lc_shutdown(void)
{
    free(g_lc.scan_history);
    g_lc.initialized = false;
}
