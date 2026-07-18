/**
 * @file    submap.c
 * @brief   Submap management for large environments
 * @details Creates a new submap every 100 scans or 30m traveled.
 *          - Active submaps: current + previous (for scan matching)
 *          - Finished submaps: frozen, used only for loop closure
 *
 *          Each submap is a probability grid (occupancy grid) with
 *          resolution configurable at creation time.
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
#define SM_MAX_SUBMAPS              256
#define SM_DEFAULT_RESOLUTION_M     0.05   /* 5cm cells */
#define SM_DEFAULT_SIZE_M           30.0   /* 30m x 30m submap */
#define SM_MAX_SCANS_PER_SUBMAP     100
#define SM_MAX_TRAVEL_M             30.0f
#define SM_PROB_HIT                0.85f   /* occupancy probability for a hit */
#define SM_PROB_MISS               0.40f   /* for a miss */
#define SM_PROB_UNKNOWN            0.50f
#define SM_PROB_MIN                0.10f
#define SM_PROB_MAX                0.90f

#define SM_PI                       3.14159265358979323846f

/* ------------------------------------------------------------------ */
/*  2D types                                                          */
/* ------------------------------------------------------------------ */
typedef struct {
    float x;
    float y;
    float theta;
} sm_pose2d_t;

typedef struct {
    float range;
    float angle;   /* relative to sensor */
} sm_ray_t;

/* ------------------------------------------------------------------ */
/*  Occupancy grid for a submap                                        */
/* ------------------------------------------------------------------ */
typedef struct {
    float  resolution;       /* meters per cell */
    int    size_x;           /* cells in x */
    int    size_y;           /* cells in y */
    float  origin_x;         /* world coordinate of center */
    float  origin_y;
    float  *cells;           /* probability values, row-major */
    int    *update_count;    /* how many times each cell was updated */
} sm_grid_t;

/* ------------------------------------------------------------------ */
/*  Submap                                                             */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t    id;
    sm_grid_t   grid;
    sm_pose2d_t pose;               /* global pose of submap origin */
    int         scan_count;         /* scans inserted so far */
    int         max_scans;
    float       max_travel_m;
    bool        finished;           /* true = frozen, only used for loop closure */
    uint32_t    first_scan_id;
    uint32_t    last_scan_id;
    float       creation_time;
} sm_submap_t;

/* ------------------------------------------------------------------ */
/*  Submap manager                                                     */
/* ------------------------------------------------------------------ */
typedef struct {
    sm_submap_t submaps[SM_MAX_SUBMAPS];
    int         submap_count;
    int         active_submap_idx;  /* index of current active submap */
    int         prev_submap_idx;    /* index of previous submap (for matching) */

    /* Pose tracking for travel distance */
    sm_pose2d_t last_pose;
    bool        pose_valid;

    uint32_t    next_submap_id;
    uint32_t    next_scan_id;

    bool initialized;
} sm_manager_t;

static sm_manager_t g_sm;

/* ------------------------------------------------------------------ */
/*  Grid operations                                                    */
/* ------------------------------------------------------------------ */

static int sm_grid_init(sm_grid_t *g, float resolution,
                         int size_x, int size_y,
                         float origin_x, float origin_y)
{
    memset(g, 0, sizeof(sm_grid_t));
    g->resolution = resolution;
    g->size_x     = size_x;
    g->size_y     = size_y;
    g->origin_x   = origin_x;
    g->origin_y   = origin_y;

    g->cells = (float *)calloc(size_x * size_y, sizeof(float));
    if (!g->cells) return -ENOMEM;

    g->update_count = (int *)calloc(size_x * size_y, sizeof(int));
    if (!g->update_count) {
        free(g->cells);
        g->cells = NULL;
        return -ENOMEM;
    }

    /* Initialize to unknown probability */
    for (int i = 0; i < size_x * size_y; i++)
        g->cells[i] = SM_PROB_UNKNOWN;

    return 0;
}

static void sm_grid_destroy(sm_grid_t *g)
{
    if (g->cells)         { free(g->cells);         g->cells = NULL; }
    if (g->update_count)  { free(g->update_count);  g->update_count = NULL; }
}

static int sm_grid_world_to_cell(const sm_grid_t *g,
                                  float wx, float wy,
                                  int *cx, int *cy)
{
    *cx = (int)((wx - g->origin_x) / g->resolution + g->size_x / 2);
    *cy = (int)((wy - g->origin_y) / g->resolution + g->size_y / 2);
    return (*cx >= 0 && *cx < g->size_x && *cy >= 0 && *cy < g->size_y) ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/*  Occupancy update using Bresenham ray casting                       */
/* ------------------------------------------------------------------ */

static void sm_grid_ray_cast(sm_grid_t *g,
                              float sx, float sy,  /* sensor origin */
                              float ex, float ey,  /* endpoint (hit) */
                              float range_max)
{
    int cx0, cy0, cx1, cy1;

    if (sm_grid_world_to_cell(g, sx, sy, &cx0, &cy0) != 0) return;
    if (sm_grid_world_to_cell(g, ex, ey, &cx1, &cy1) != 0) return;

    /* Bresenham line algorithm */
    int dx = abs(cx1 - cx0);
    int dy = -abs(cy1 - cy0);
    int sx_step = (cx0 < cx1) ? 1 : -1;
    int sy_step = (cy0 < cy1) ? 1 : -1;
    int err = dx + dy;

    int x = cx0, y = cy0;

    while (1) {
        int idx = y * g->size_x + x;

        if (x == cx1 && y == cy1) {
            /* Endpoint: occupied (hit) */
            g->cells[idx] = SM_PROB_HIT;
            break;
        }

        /* Free space (miss) */
        g->cells[idx] = SM_PROB_MISS;
        g->update_count[idx]++;

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx_step; }
        if (e2 <= dx) { err += dx; y += sy_step; }

        /* Range check */
        float wx = g->origin_x + (x - g->size_x / 2) * g->resolution;
        float wy = g->origin_y + (y - g->size_y / 2) * g->resolution;
        float dist = sqrtf((wx - sx) * (wx - sx) + (wy - sy) * (wy - sy));
        if (dist > range_max) break;
    }
}

/* ------------------------------------------------------------------ */
/*  Submap operations                                                  */
/* ------------------------------------------------------------------ */

static int sm_create_submap(const sm_pose2d_t *pose)
{
    if (g_sm.submap_count >= SM_MAX_SUBMAPS)
        return -ENOSPC;

    sm_submap_t *sm = &g_sm.submaps[g_sm.submap_count];

    memset(sm, 0, sizeof(sm_submap_t));
    sm->id = g_sm.next_submap_id++;
    sm->pose = *pose;
    sm->scan_count = 0;
    sm->max_scans = SM_MAX_SCANS_PER_SUBMAP;
    sm->max_travel_m = SM_MAX_TRAVEL_M;
    sm->finished = false;
    sm->first_scan_id = g_sm.next_scan_id;
    sm->last_scan_id = 0;
    sm->creation_time = 0.0f;

    int cells_xy = (int)(SM_DEFAULT_SIZE_M / SM_DEFAULT_RESOLUTION_M);
    int rc = sm_grid_init(&sm->grid,
                           SM_DEFAULT_RESOLUTION_M,
                           cells_xy, cells_xy,
                           pose->x, pose->y);
    if (rc != 0) return rc;

    g_sm.submap_count++;
    g_sm.active_submap_idx = g_sm.submap_count - 1;
    g_sm.pose_valid = true;
    g_sm.last_pose = *pose;

    fprintf(stderr, "[SM] created submap %u (total %d)\n",
            sm->id, g_sm.submap_count);

    return 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int sm_init(void)
{
    memset(&g_sm, 0, sizeof(g_sm));
    g_sm.active_submap_idx = -1;
    g_sm.prev_submap_idx = -1;
    g_sm.pose_valid = false;
    g_sm.initialized = true;
    return 0;
}

/**
 * sm_insert_scan - insert a LIDAR scan into the active submap
 * @rays:         array of range-bearing readings
 * @num_rays:     number of rays
 * @sensor_pose:  sensor pose in global frame
 * @range_max:    maximum valid range
 *
 * If the active submap exceeds scan count or travel distance,
 * a new submap is created automatically.
 *
 * Returns submap ID scan was inserted into, or negative on error.
 */
int sm_insert_scan(const sm_ray_t *rays, int num_rays,
                    const sm_pose2d_t *sensor_pose,
                    float range_max)
{
    if (!g_sm.initialized) return -EPERM;
    if (!rays || num_rays <= 0 || !sensor_pose) return -EINVAL;

    /* Check if we need a new submap */
    bool need_new = false;

    if (g_sm.active_submap_idx < 0) {
        need_new = true;
    } else {
        sm_submap_t *active = &g_sm.submaps[g_sm.active_submap_idx];

        /* Check scan count threshold */
        if (active->scan_count >= active->max_scans)
            need_new = true;

        /* Check travel distance threshold */
        if (g_sm.pose_valid) {
            float dx = sensor_pose->x - g_sm.last_pose.x;
            float dy = sensor_pose->y - g_sm.last_pose.y;
            float dist = sqrtf(dx * dx + dy * dy);

            static float total_travel = 0.0f;
            total_travel += dist;
            if (total_travel >= active->max_travel_m) {
                need_new = true;
                total_travel = 0.0f;
            }
        }
    }

    if (need_new) {
        /* Freeze current active as previous for scan matching */
        if (g_sm.active_submap_idx >= 0) {
            g_sm.submaps[g_sm.active_submap_idx].finished = true;
            g_sm.submaps[g_sm.active_submap_idx].last_scan_id = g_sm.next_scan_id - 1;
            g_sm.prev_submap_idx = g_sm.active_submap_idx;
        }

        int rc = sm_create_submap(sensor_pose);
        if (rc != 0) return rc;
    }

    sm_submap_t *active = &g_sm.submaps[g_sm.active_submap_idx];

    /* Insert rays into active submap grid */
    float sx = sensor_pose->x;
    float sy = sensor_pose->y;

    for (int i = 0; i < num_rays; i++) {
        if (rays[i].range <= 0.0f || rays[i].range > range_max)
            continue;

        float ex = sx + rays[i].range * cosf(sensor_pose->theta + rays[i].angle);
        float ey = sy + rays[i].range * sinf(sensor_pose->theta + rays[i].angle);

        sm_grid_ray_cast(&active->grid, sx, sy, ex, ey, range_max);
    }

    active->scan_count++;
    active->last_scan_id = g_sm.next_scan_id++;
    g_sm.last_pose = *sensor_pose;
    g_sm.pose_valid = true;

    return (int)active->id;
}

/**
 * sm_get_active - get pointer to active submap
 */
const sm_submap_t *sm_get_active(int *index)
{
    if (!g_sm.initialized) return NULL;
    if (g_sm.active_submap_idx < 0) return NULL;

    if (index) *index = g_sm.active_submap_idx;
    return &g_sm.submaps[g_sm.active_submap_idx];
}

/**
 * sm_get_previous - get pointer to previous (most recently finished) submap
 */
const sm_submap_t *sm_get_previous(int *index)
{
    if (!g_sm.initialized) return NULL;
    if (g_sm.prev_submap_idx < 0) return NULL;

    if (index) *index = g_sm.prev_submap_idx;
    return &g_sm.submaps[g_sm.prev_submap_idx];
}

/**
 * sm_get_submap - get submap by index
 */
const sm_submap_t *sm_get_submap(int index)
{
    if (!g_sm.initialized) return NULL;
    if (index < 0 || index >= g_sm.submap_count) return NULL;
    return &g_sm.submaps[index];
}

/**
 * sm_get_count - get total number of submaps
 */
int sm_get_count(void)
{
    return g_sm.submap_count;
}

/**
 * sm_get_grid - get occupancy grid data for a submap (for visualization / export)
 */
int sm_get_grid(int submap_index,
                 float **cells, int *size_x, int *size_y,
                 float *resolution, float *origin_x, float *origin_y)
{
    if (submap_index < 0 || submap_index >= g_sm.submap_count)
        return -EINVAL;

    const sm_submap_t *sm = &g_sm.submaps[submap_index];
    if (cells)     *cells     = sm->grid.cells;
    if (size_x)    *size_x    = sm->grid.size_x;
    if (size_y)    *size_y    = sm->grid.size_y;
    if (resolution)*resolution = sm->grid.resolution;
    if (origin_x)  *origin_x  = sm->grid.origin_x;
    if (origin_y)  *origin_y  = sm->grid.origin_y;
    return 0;
}

void sm_shutdown(void)
{
    for (int i = 0; i < g_sm.submap_count; i++)
        sm_grid_destroy(&g_sm.submaps[i].grid);
    g_sm.submap_count = 0;
    g_sm.active_submap_idx = -1;
    g_sm.initialized = false;
}
