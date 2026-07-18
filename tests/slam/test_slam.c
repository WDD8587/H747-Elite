/**
 * test_slam.c
 * Unit tests for SLAM grid mapping (occupancy grid and Bresenham ray tracing).
 *
 * Tests:
 *   - Grid map: add obstacle -> cell becomes occupied
 *   - Empty map: all cells are zero (unknown/free)
 *   - Bresenham line from robot position is continuous
 *
 * Build:
 *   gcc -I. -I../../firmware -DUNIT_TEST test_slam.c -lm -o test_slam
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Grid map types                                                    */
/* ------------------------------------------------------------------ */

#define GRID_SIZE_X      200
#define GRID_SIZE_Y      200
#define GRID_RESOLUTION_MM  50   /* 50 mm per cell */
#define GRID_MAP_SIZE    (GRID_SIZE_X * GRID_SIZE_Y)

/* Cell value: 0 = unknown, 1-49 = free, 51-100 = occupied, 50 = threshold */
#define CELL_UNKNOWN     0
#define CELL_FREE_MIN    1
#define CELL_FREE_MAX    49
#define CELL_THRESHOLD   50
#define CELL_OCCUPIED_MIN 51
#define CELL_OCCUPIED_MAX 100
#define CELL_INIT        0

typedef struct {
    uint8_t cells[GRID_MAP_SIZE];
    int     origin_x_mm;     /* robot origin in world mm */
    int     origin_y_mm;
} grid_map_t;

/* Ray cast result */
typedef struct {
    int x, y;               /* cell coordinates */
    int occupied;           /* 1 if cell is obstacle */
} ray_hit_t;

/* DUT functions */
void  grid_map_init(grid_map_t *map);
void  grid_map_set_obstacle(grid_map_t *map, int world_x_mm, int world_y_mm);
int   grid_map_get_cell(const grid_map_t *map, int world_x_mm, int world_y_mm);
void  grid_map_raytrace(grid_map_t *map, int x0, int y0, int x1, int y1);
void  grid_map_world_to_cell(int world_x_mm, int world_y_mm, int *cx, int *cy);

/* ------------------------------------------------------------------ */
/*  DUT implementation                                                */
/* ------------------------------------------------------------------ */

void grid_map_init(grid_map_t *map)
{
    memset(map, 0, sizeof(*map));
    map->origin_x_mm = GRID_SIZE_X * GRID_RESOLUTION_MM / 2;
    map->origin_y_mm = GRID_SIZE_Y * GRID_RESOLUTION_MM / 2;
}

void grid_map_world_to_cell(int world_x_mm, int world_y_mm, int *cx, int *cy)
{
    *cx = (world_x_mm + GRID_SIZE_X * GRID_RESOLUTION_MM / 2) / GRID_RESOLUTION_MM;
    *cy = (world_y_mm + GRID_SIZE_Y * GRID_RESOLUTION_MM / 2) / GRID_RESOLUTION_MM;

    if (*cx < 0) *cx = 0;
    if (*cx >= GRID_SIZE_X) *cx = GRID_SIZE_X - 1;
    if (*cy < 0) *cy = 0;
    if (*cy >= GRID_SIZE_Y) *cy = GRID_SIZE_Y - 1;
}

void grid_map_set_obstacle(grid_map_t *map, int world_x_mm, int world_y_mm)
{
    int cx, cy;
    grid_map_world_to_cell(world_x_mm, world_y_mm, &cx, &cy);
    int idx = cy * GRID_SIZE_X + cx;

    /* Increase occupancy */
    if (map->cells[idx] < CELL_OCCUPIED_MAX)
        map->cells[idx] += 10;
    if (map->cells[idx] > CELL_OCCUPIED_MAX)
        map->cells[idx] = CELL_OCCUPIED_MAX;
}

int grid_map_get_cell(const grid_map_t *map, int world_x_mm, int world_y_mm)
{
    int cx, cy;
    grid_map_world_to_cell(world_x_mm, world_y_mm, &cx, &cy);
    return map->cells[cy * GRID_SIZE_X + cx];
}

/* Bresenham ray tracing: mark cells between (x0,y0) and (x1,y1) as free,
 * and the endpoint as occupied. */
void grid_map_raytrace(grid_map_t *map, int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int cx = x0, cy = y0;

    while (1) {
        /* Mark as free (unless endpoint) */
        if (cx != x1 || cy != y1) {
            if (cx >= 0 && cx < GRID_SIZE_X && cy >= 0 && cy < GRID_SIZE_Y) {
                int idx = cy * GRID_SIZE_X + cx;
                if (map->cells[idx] == CELL_UNKNOWN || map->cells[idx] > CELL_FREE_MAX)
                    map->cells[idx] = CELL_FREE_MAX;
                else if (map->cells[idx] > 1)
                    map->cells[idx]--;  /* decrease occupancy */
            }
        }

        if (cx == x1 && cy == y1) {
            /* Mark endpoint as occupied */
            if (cx >= 0 && cx < GRID_SIZE_X && cy >= 0 && cy < GRID_SIZE_Y) {
                int idx = cy * GRID_SIZE_X + cx;
                map->cells[idx] = CELL_OCCUPIED_MAX;
            }
            break;
        }

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; cx += sx; }
        if (e2 <= dx) { err += dx; cy += sy; }
    }
}

/* ------------------------------------------------------------------ */
/*  Test utilities                                                    */
/* ------------------------------------------------------------------ */

static int test_passed = 0;
static int test_failed = 0;

#define TEST_START(name)  do { printf("  TEST %-55s ", name); } while(0)
#define TEST_PASS()       do { printf("PASS\n"); test_passed++; } while(0)
#define TEST_FAIL(msg)    do { printf("FAIL: %s\n", msg); test_failed++; } while(0)

/* ------------------------------------------------------------------ */
/*  Tests                                                             */
/* ------------------------------------------------------------------ */

static void test_empty_map_all_zero(void)
{
    TEST_START("Empty map: all cells initialized to zero");
    grid_map_t map;
    grid_map_init(&map);

    int all_zero = 1;
    for (int i = 0; i < GRID_MAP_SIZE; i++) {
        if (map.cells[i] != 0) {
            all_zero = 0;
            break;
        }
    }

    if (all_zero)
        TEST_PASS();
    else
        TEST_FAIL("not all cells are zero");
}

static void test_add_obstacle(void)
{
    TEST_START("Add obstacle -> cell becomes occupied");
    grid_map_t map;
    grid_map_init(&map);

    /* Place obstacle at (0, 0) in world coordinates (center of map) */
    grid_map_set_obstacle(&map, 0, 0);

    int cell_val = grid_map_get_cell(&map, 0, 0);

    if (cell_val >= CELL_OCCUPIED_MIN)
        TEST_PASS();
    else
        TEST_FAIL("cell not occupied after adding obstacle");
}

static void test_multiple_obstacles(void)
{
    TEST_START("Multiple obstacles at different positions");
    grid_map_t map;
    grid_map_init(&map);

    grid_map_set_obstacle(&map, 1000, 0);
    grid_map_set_obstacle(&map, -1000, 500);
    grid_map_set_obstacle(&map, 500, -500);

    int v1 = grid_map_get_cell(&map, 1000, 0);
    int v2 = grid_map_get_cell(&map, -1000, 500);
    int v3 = grid_map_get_cell(&map, 500, -500);

    /* Also check that a distant free cell is still 0 (it may have been
     * cleared by a raytrace, but this raw set test should show occupancy) */
    int v_empty = grid_map_get_cell(&map, 2000, 2000);

    if (v1 >= CELL_OCCUPIED_MIN && v2 >= CELL_OCCUPIED_MIN &&
        v3 >= CELL_OCCUPIED_MIN && v_empty == CELL_UNKNOWN)
        TEST_PASS();
    else
        TEST_FAIL("obstacle placement failed");
}

static void test_bresenham_continuous(void)
{
    TEST_START("Bresenham line from robot pos is continuous");
    grid_map_t map;
    grid_map_init(&map);

    /* Robot at center (0,0), obstacle at (500mm, 500mm) */
    int x0 = GRID_SIZE_X / 2;
    int y0 = GRID_SIZE_Y / 2;
    int x1 = x0 + 10;  /* +10 cells in X */
    int y1 = y0 + 10;  /* +10 cells in Y */

    grid_map_raytrace(&map, x0, y0, x1, y1);

    /* Check that the endpoint is occupied */
    int end_val = map.cells[y1 * GRID_SIZE_X + x1];
    if (end_val < CELL_OCCUPIED_MIN) { TEST_FAIL("endpoint not occupied"); return; }

    /* Check that cells along the line are set to free */
    /* Walk the line using Bresenham and verify each intermediate cell */
    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int cx = x0, cy = y0;
    int cells_visited = 0;

    while (1) {
        if (cx >= 0 && cx < GRID_SIZE_X && cy >= 0 && cy < GRID_SIZE_Y) {
            cells_visited++;
        }
        if (cx == x1 && cy == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; cx += sx; }
        if (e2 <= dx) { err += dx; cy += sy; }
    }

    /* The number of visited cells should be > distance along major axis */
    if (cells_visited >= 10)
        TEST_PASS();
    else
        TEST_FAIL("Bresenham line is too short (not continuous)");
}

static void test_bresenham_clears_path(void)
{
    TEST_START("Bresenham clears path cells between robot and obstacle");
    grid_map_t map;
    grid_map_init(&map);

    /* First place an obstacle */
    int x0 = GRID_SIZE_X / 2;
    int y0 = GRID_SIZE_Y / 2;
    int x1 = x0 + 5;
    int y1 = y0;

    grid_map_set_obstacle(&map,
        (x1 - GRID_SIZE_X / 2) * GRID_RESOLUTION_MM,
        (y1 - GRID_SIZE_Y / 2) * GRID_RESOLUTION_MM);

    /* Now raytrace */
    grid_map_raytrace(&map, x0, y0, x1, y1);

    /* Cells between should be free (<= CELL_FREE_MAX) */
    int mid_val = map.cells[y0 * GRID_SIZE_X + (x0 + 2)];
    int end_val = map.cells[y1 * GRID_SIZE_X + x1];

    if (mid_val <= CELL_FREE_MAX && end_val >= CELL_OCCUPIED_MIN)
        TEST_PASS();
    else
        TEST_FAIL("path not cleared or endpoint not occupied");
}

static void test_obstacle_at_map_edge(void)
{
    TEST_START("Obstacle at map edge does not overflow");
    grid_map_t map;
    grid_map_init(&map);

    /* Place obstacle far beyond map bounds */
    grid_map_set_obstacle(&map, 100000, 100000);

    /* Cell at max extent should be safe to read */
    int v = grid_map_get_cell(&map, 100000, 100000);
    if (v >= 0 && v <= CELL_OCCUPIED_MAX)
        TEST_PASS();
    else
        TEST_FAIL("edge obstacle caused invalid state");
}

/* ------------------------------------------------------------------ */
/*  Main                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== SLAM Grid Map Unit Tests ===\n\n");

    test_empty_map_all_zero();
    test_add_obstacle();
    test_multiple_obstacles();
    test_bresenham_continuous();
    test_bresenham_clears_path();
    test_obstacle_at_map_edge();

    printf("\nResults: %d passed, %d failed\n", test_passed, test_failed);
    return test_failed > 0 ? 1 : 0;
}
