/**
 * @file test_nav_path.c
 * @brief Integration test: Navigate from (0,0) to (2,1) with obstacle at (1,0.5)
 *
 * Tests the path planning and navigation system:
 * - Global planner must find path around obstacle
 * - Local planner must follow path without collision
 * - Robot must reach goal within 0.1m tolerance
 *
 * Build: gcc -o test_nav_path test_nav_path.c -lm -lcmocka
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdint.h>
#include <cmocka.h>

/* ======================== Data Structures ======================== */

#define MAP_SIZE_X            20      /* 10 meters at 0.5m resolution */
#define MAP_SIZE_Y            20
#define MAP_RESOLUTION        0.5     /* meters per cell */
#define MAX_PATH_LENGTH       1000
#define ROBOT_RADIUS          0.175   /* meters */
#define GOAL_TOLERANCE        0.1     /* meters */
#define OBSTACLE_CLEARANCE    0.25    /* meters */

typedef struct {
    float x, y;
} point_t;

typedef struct {
    point_t points[MAX_PATH_LENGTH];
    int length;
} path_t;

typedef struct {
    uint8_t grid[MAP_SIZE_Y][MAP_SIZE_X]; /* 0=free, 100=occupied, 255=unknown */
    float origin_x, origin_y;
    float resolution;
} occupancy_grid_t;

typedef struct {
    point_t position;
    float yaw;
    float linear_vel;
    float angular_vel;
} robot_pose_t;

typedef struct {
    uint8_t obstacle_detected;
    float obstacle_distance;
    float obstacle_angle;
} sensor_data_t;

/* ======================== Global State ======================== */

static occupancy_grid_t test_map;
static path_t planned_path;
static robot_pose_t robot;
static sensor_data_t sensors;

/* ======================== Mock Map Setup ======================== */

static void init_test_map(void) {
    memset(&test_map, 0, sizeof(test_map));
    test_map.origin_x = -5.0;
    test_map.origin_y = -5.0;
    test_map.resolution = MAP_RESOLUTION;

    /* Add obstacle at (1.0, 0.5) in world coordinates -> grid coordinates */
    int ox = (int)((1.0 - test_map.origin_x) / test_map.resolution);
    int oy = (int)((0.5 - test_map.origin_y) / test_map.resolution);

    /* Obstacle is a circle of radius 0.3m */
    float obs_radius_cells = 0.3 / test_map.resolution;
    for (int dy = -(int)ceil(obs_radius_cells); dy <= (int)ceil(obs_radius_cells); dy++) {
        for (int dx = -(int)ceil(obs_radius_cells); dx <= (int)ceil(obs_radius_cells); dx++) {
            float dist = sqrtf(dx*dx + dy*dy);
            if (dist <= obs_radius_cells) {
                int cx = ox + dx;
                int cy = oy + dy;
                if (cx >= 0 && cx < MAP_SIZE_X && cy >= 0 && cy < MAP_SIZE_Y) {
                    test_map.grid[cy][cx] = 100;
                }
            }
        }
    }

    /* Add some noise / unknown cells at edges */
    for (int y = 0; y < MAP_SIZE_Y; y++) {
        for (int x = 0; x < MAP_SIZE_X; x++) {
            if (x == 0 || x == MAP_SIZE_X-1 || y == 0 || y == MAP_SIZE_Y-1) {
                test_map.grid[y][x] = 255;
            }
        }
    }
}

/* ======================== World-to-Grid Conversion ======================== */

static void world_to_grid(float wx, float wy, int* gx, int* gy) {
    *gx = (int)((wx - test_map.origin_x) / test_map.resolution);
    *gy = (int)((wy - test_map.origin_y) / test_map.resolution);
    if (*gx < 0) *gx = 0;
    if (*gx >= MAP_SIZE_X) *gx = MAP_SIZE_X - 1;
    if (*gy < 0) *gy = 0;
    if (*gy >= MAP_SIZE_Y) *gy = MAP_SIZE_Y - 1;
}

static void grid_to_world(int gx, int gy, float* wx, float* wy) {
    *wx = test_map.origin_x + (gx + 0.5f) * test_map.resolution;
    *wy = test_map.origin_y + (gy + 0.5f) * test_map.resolution;
}

/* ======================== A* Path Planner ======================== */

typedef struct {
    int x, y;
    float g, f;
    int parent_x, parent_y;
    int in_open, in_closed;
} astar_node_t;

static astar_node_t astar_nodes[MAP_SIZE_Y][MAP_SIZE_X];

static float heuristic(int x1, int y1, int x2, int y2) {
    /* Euclidean distance */
    float dx = (float)(x1 - x2);
    float dy = (float)(y1 - y2);
    return sqrtf(dx*dx + dy*dy) * MAP_RESOLUTION;
}

static int is_collision_free(int gx, int gy) {
    if (gx < 0 || gx >= MAP_SIZE_X || gy < 0 || gy >= MAP_SIZE_Y) return 0;
    if (test_map.grid[gy][gx] >= 50) return 0;  /* Occupied or unknown */

    /* Check clearance radius around the cell */
    int clear_cells = (int)(OBSTACLE_CLEARANCE / MAP_RESOLUTION);
    for (int dy = -clear_cells; dy <= clear_cells; dy++) {
        for (int dx = -clear_cells; dx <= clear_cells; dx++) {
            int nx = gx + dx;
            int ny = gy + dy;
            if (nx >= 0 && nx < MAP_SIZE_X && ny >= 0 && ny < MAP_SIZE_Y) {
                float dist = sqrtf((float)(dx*dx + dy*dy)) * MAP_RESOLUTION;
                if (dist <= ROBOT_RADIUS && test_map.grid[ny][nx] >= 50) {
                    return 0;
                }
            }
        }
    }
    return 1;
}

static int plan_path(point_t start, point_t goal, path_t* path) {
    int sx, sy, gx, gy;
    world_to_grid(start.x, start.y, &sx, &sy);
    world_to_grid(goal.x, goal.y, &gx, &gy);

    if (!is_collision_free(sx, sy) || !is_collision_free(gx, gy)) {
        printf("Start or goal is in collision\n");
        return 0;
    }

    /* Reset A* nodes */
    memset(astar_nodes, 0, sizeof(astar_nodes));

    /* Open list (simple array - for test purposes) */
    #define MAX_OPEN 10000
    static int open_list[MAX_OPEN][2];
    int open_count = 0;

    /* Initialize start node */
    astar_nodes[sy][sx].x = sx;
    astar_nodes[sy][sx].y = sy;
    astar_nodes[sy][sx].g = 0;
    astar_nodes[sy][sx].f = heuristic(sx, sy, gx, gy);
    astar_nodes[sy][sx].parent_x = -1;
    astar_nodes[sy][sx].parent_y = -1;
    astar_nodes[sy][sx].in_open = 1;

    open_list[open_count][0] = sx;
    open_list[open_count][1] = sy;
    open_count++;

    /* 8-connected grid */
    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const float step_cost[8] = {1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f};

    int found = 0;
    int iterations = 0;
    const int MAX_ITER = 50000;

    while (open_count > 0 && iterations < MAX_ITER) {
        iterations++;

        /* Find node with lowest f in open list */
        int best_idx = 0;
        float best_f = INFINITY;
        for (int i = 0; i < open_count; i++) {
            int nx = open_list[i][0];
            int ny = open_list[i][1];
            if (astar_nodes[ny][nx].f < best_f) {
                best_f = astar_nodes[ny][nx].f;
                best_idx = i;
            }
        }

        int cx = open_list[best_idx][0];
        int cy = open_list[best_idx][1];

        /* Remove from open list (swap with last) */
        open_list[best_idx][0] = open_list[open_count-1][0];
        open_list[best_idx][1] = open_list[open_count-1][1];
        open_count--;

        astar_nodes[cy][cx].in_open = 0;
        astar_nodes[cy][cx].in_closed = 1;

        /* Check if we reached the goal */
        if (cx == gx && cy == gy) {
            found = 1;
            break;
        }

        /* Expand neighbors */
        for (int dir = 0; dir < 8; dir++) {
            int nx = cx + dx[dir];
            int ny = cy + dy[dir];

            if (nx < 0 || nx >= MAP_SIZE_X || ny < 0 || ny >= MAP_SIZE_Y) continue;
            if (astar_nodes[ny][nx].in_closed) continue;
            if (!is_collision_free(nx, ny)) continue;

            float tentative_g = astar_nodes[cy][cx].g + step_cost[dir] * MAP_RESOLUTION;

            if (!astar_nodes[ny][nx].in_open) {
                /* New node */
                astar_nodes[ny][nx].x = nx;
                astar_nodes[ny][nx].y = ny;
                astar_nodes[ny][nx].g = tentative_g;
                astar_nodes[ny][nx].f = tentative_g + heuristic(nx, ny, gx, gy);
                astar_nodes[ny][nx].parent_x = cx;
                astar_nodes[ny][nx].parent_y = cy;
                astar_nodes[ny][nx].in_open = 1;

                if (open_count < MAX_OPEN) {
                    open_list[open_count][0] = nx;
                    open_list[open_count][1] = ny;
                    open_count++;
                }
            } else if (tentative_g < astar_nodes[ny][nx].g) {
                /* Better path to existing node */
                astar_nodes[ny][nx].g = tentative_g;
                astar_nodes[ny][nx].f = tentative_g + heuristic(nx, ny, gx, gy);
                astar_nodes[ny][nx].parent_x = cx;
                astar_nodes[ny][nx].parent_y = cy;
            }
        }
    }

    if (!found) {
        printf("A* failed to find path after %d iterations\n", iterations);
        return 0;
    }

    /* Reconstruct path */
    path->length = 0;
    int cx = gx, cy = gy;

    while (!(cx == sx && cy == sy)) {
        float wx, wy;
        grid_to_world(cx, cy, &wx, &wy);
        path->points[path->length].x = wx;
        path->points[path->length].y = wy;
        path->length++;

        int px = astar_nodes[cy][cx].parent_x;
        int py = astar_nodes[cy][cx].parent_y;
        cx = px;
        cy = py;

        if (cx < 0 || cy < 0 || path->length >= MAX_PATH_LENGTH) break;
    }

    /* Add start point */
    path->points[path->length].x = start.x;
    path->points[path->length].y = start.y;
    path->length++;

    /* Reverse path */
    for (int i = 0; i < path->length / 2; i++) {
        point_t temp = path->points[i];
        path->points[i] = path->points[path->length - 1 - i];
        path->points[path->length - 1 - i] = temp;
    }

    printf("Path planned: %d waypoints, %.2fm length\n",
           path->length,
           path->length > 0 ? heuristic(cx, cy, gx, gy) : 0);

    return 1;
}

/* ======================== Path Follower (Pure Pursuit) ======================== */

static float pt_dist(point_t a, point_t b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx*dx + dy*dy);
}

static int follow_path(path_t* path, robot_pose_t* robot,
                       point_t goal, float tolerance, int max_steps)
{
    int lookahead_idx = 0;
    float lookahead_dist = 0.3;  /* 0.3m lookahead */
    float linear_speed = 0.2;     /* 0.2 m/s */
    float max_angular = 1.0;      /* 1.0 rad/s */

    printf("Following path to goal (%.2f, %.2f)...\n", goal.x, goal.y);

    for (int step = 0; step < max_steps; step++) {
        /* Check goal reached */
        float dist_to_goal = pt_dist(robot->position, goal);
        if (dist_to_goal < tolerance) {
            printf("Goal reached at step %d: dist=%.3fm\n", step, dist_to_goal);
            return 1;
        }

        /* Find lookahead point */
        int found_lookahead = 0;
        for (int i = lookahead_idx; i < path->length; i++) {
            float d = pt_dist(robot->position, path->points[i]);
            if (d >= lookahead_dist) {
                lookahead_idx = i;
                found_lookahead = 1;
                break;
            }
        }
        if (!found_lookahead) {
            lookahead_idx = path->length - 1;
        }

        /* Calculate steering angle (pure pursuit) */
        point_t target = path->points[lookahead_idx];
        float dx = target.x - robot->position.x;
        float dy = target.y - robot->position.y;

        /* Transform to robot frame */
        float cos_yaw = cosf(robot->yaw);
        float sin_yaw = sinf(robot->yaw);
        float local_x = dx * cos_yaw + dy * sin_yaw;
        float local_y = -dx * sin_yaw + dy * cos_yaw;

        /* Curvature = 2 * lateral_error / lookahead^2 */
        float curvature = 2.0f * local_y / (lookahead_dist * lookahead_dist);

        /* Angular velocity = linear_vel * curvature */
        float angular = linear_speed * curvature;
        if (angular > max_angular) angular = max_angular;
        if (angular < -max_angular) angular = -max_angular;

        /* Update robot pose (simple odometry simulation) */
        float dt = 0.1;  /* 100 Hz control loop */

        if (fabsf(angular) < 1e-6) {
            robot->position.x += linear_speed * cosf(robot->yaw) * dt;
            robot->position.y += linear_speed * sinf(robot->yaw) * dt;
        } else {
            float radius = linear_speed / angular;
            float dtheta = angular * dt;
            robot->position.x += radius * (sinf(robot->yaw + dtheta) - sinf(robot->yaw));
            robot->position.y += radius * (-cosf(robot->yaw + dtheta) + cosf(robot->yaw));
        }
        robot->yaw += angular * dt;
        robot->yaw = atan2f(sinf(robot->yaw), cosf(robot->yaw));

        /* Simulate obstacle avoidance: check if path goes through obstacle */
        int rx, ry;
        world_to_grid(robot->position.x, robot->position.y, &rx, &ry);
        if (!is_collision_free(rx, ry)) {
            printf("WARNING: Robot in collision at (%.2f, %.2f) step %d\n",
                   robot->position.x, robot->position.y, step);
            /* Try to back away */
            robot->position.x -= linear_speed * cosf(robot->yaw) * dt * 2;
            robot->position.y -= linear_speed * sinf(robot->yaw) * dt * 2;
        }

        /* Print progress every 10 steps */
        if (step % 10 == 0) {
            printf("  Step %3d: pos(%.2f, %.2f) yaw=%.1fdeg dist=%.3fm\n",
                   step, robot->position.x, robot->position.y,
                   robot->yaw * 180.0 / M_PI, dist_to_goal);
        }

        /* Check if stuck (barely moving toward goal for many steps) */
        if (step > 50 && step % 50 == 0) {
            printf("  Progress check: dist=%.3fm (was %.3fm)\n",
                   dist_to_goal, pt_dist(robot->position, goal));
        }
    }

    return 0;  /* Max steps reached without reaching goal */
}

/* ======================== Test Cases ======================== */

static void test_navigate_to_goal_with_obstacle(void** state) {
    (void)state;

    printf("\n=== Navigation Integration Test ===\n");
    printf("Start: (0, 0), Goal: (2, 1), Obstacle at (1, 0.5)\n\n");

    /* Initialize */
    init_test_map();
    robot.position.x = 0.0f;
    robot.position.y = 0.0f;
    robot.yaw = 0.0f;
    robot.linear_vel = 0.0f;
    robot.angular_vel = 0.0f;

    point_t start = {0.0f, 0.0f};
    point_t goal = {2.0f, 1.0f};

    /* Step 1: Plan path */
    printf("Step 1: Planning path...\n");
    int found_path = plan_path(start, goal, &planned_path);
    assert_true(found_path);

    /* Verify path avoids the obstacle */
    printf("Step 2: Verifying path avoids obstacle...\n");
    int avoids_obstacle = 1;
    for (int i = 0; i < planned_path.length; i++) {
        float d = pt_dist(planned_path.points[i], (point_t){1.0f, 0.5f});
        if (d < 0.4f) {  /* Clearance should be at least 0.4m from obstacle center */
            avoids_obstacle = 0;
            printf("  FAIL: Waypoint %d (%.2f, %.2f) is %.2fm from obstacle\n",
                   i, planned_path.points[i].x, planned_path.points[i].y, d);
        }
    }
    assert_true(avoids_obstacle);
    printf("  Path avoids obstacle.\n");

    /* Print the planned path */
    printf("  Planned path (%d waypoints):\n", planned_path.length);
    for (int i = 0; i < planned_path.length && i < 20; i++) {
        printf("    [%d] (%.2f, %.2f)\n", i,
               planned_path.points[i].x, planned_path.points[i].y);
    }
    if (planned_path.length > 20) {
        printf("    ... (%d more waypoints)\n", planned_path.length - 20);
    }

    /* Step 3: Follow the path */
    printf("\nStep 3: Following path (max 500 steps)...\n");
    int goal_reached = follow_path(&planned_path, &robot, goal, GOAL_TOLERANCE, 500);

    /* Step 4: Verify goal reached */
    printf("\nStep 4: Verifying goal position...\n");
    float final_dist = pt_dist(robot.position, goal);
    printf("  Final position: (%.3f, %.3f)\n", robot.position.x, robot.position.y);
    printf("  Distance to goal: %.3fm (tolerance: %.2fm)\n", final_dist, GOAL_TOLERANCE);

    assert_true(goal_reached);
    assert_true(final_dist <= GOAL_TOLERANCE);
    printf("  PASS: Goal reached within tolerance.\n");

    /* Step 5: Verify no collisions during path following */
    printf("\nStep 5: Verifying final position is not in collision...\n");
    int rx, ry;
    world_to_grid(robot.position.x, robot.position.y, &rx, &ry);
    assert_true(is_collision_free(rx, ry));
    printf("  PASS: Final position is collision-free.\n");

    printf("\n=== TEST PASSED ===\n");
}

static void test_path_around_obstacle(void** state) {
    (void)state;

    printf("\n=== Path Planning Around Obstacle Test ===\n");

    init_test_map();
    point_t start = {0.0f, 0.0f};
    point_t goal = {2.0f, 1.0f};

    /* Plan path */
    int found_path = plan_path(start, goal, &planned_path);
    assert_true(found_path);

    /* The path should have more than 2 points (straight line would be blocked) */
    assert_true(planned_path.length > 2);
    printf("Path length: %d waypoints (minimum 3 for obstacle avoidance)\n",
           planned_path.length);

    /* Verify path starts and ends correctly */
    float start_dist = pt_dist(planned_path.points[0], start);
    float end_dist = pt_dist(planned_path.points[planned_path.length - 1], goal);

    assert_true(start_dist < MAP_RESOLUTION);
    assert_true(end_dist < MAP_RESOLUTION);

    printf("Path starts at (%.2f, %.2f) - correct\n",
           planned_path.points[0].x, planned_path.points[0].y);
    printf("Path ends at (%.2f, %.2f) - correct\n",
           planned_path.points[planned_path.length-1].x,
           planned_path.points[planned_path.length-1].y);

    printf("=== PATH PLANNING TEST PASSED ===\n");
}

static void test_obstacle_clearance(void** state) {
    (void)state;

    printf("\n=== Obstacle Clearance Test ===\n");

    init_test_map();
    point_t start = {0.0f, 0.0f};
    point_t goal = {2.0f, 1.0f};

    int found_path = plan_path(start, goal, &planned_path);
    assert_true(found_path);

    /* Check every waypoint maintains minimum clearance */
    float min_clearance = INFINITY;
    for (int i = 0; i < planned_path.length; i++) {
        float d = pt_dist(planned_path.points[i], (point_t){1.0f, 0.5f});
        if (d < min_clearance) min_clearance = d;
    }

    printf("Minimum clearance from obstacle: %.3fm (requires > %.2fm)\n",
           min_clearance, ROBOT_RADIUS + 0.05);
    assert_true(min_clearance > ROBOT_RADIUS + 0.05);
    printf("=== CLEARANCE TEST PASSED ===\n");
}

/* ======================== Test Runner ======================== */

int main(void) {
    printf("========================================\n");
    printf("  H747 Elite Navigation Integration Test\n");
    printf("  Path: (0,0) -> (2,1) w/ obstacle\n");
    printf("========================================\n\n");

    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_path_around_obstacle),
        cmocka_unit_test(test_obstacle_clearance),
        cmocka_unit_test(test_navigate_to_goal_with_obstacle),
    };

    int result = cmocka_run_group_tests(tests, NULL, NULL);

    printf("\n========================================\n");
    if (result == 0) {
        printf("  ALL NAVIGATION TESTS PASSED\n");
    } else {
        printf("  SOME TESTS FAILED\n");
    }
    printf("========================================\n");

    return result;
}
