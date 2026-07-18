#include <cmath>
#include <cstdlib>

typedef struct { float linear, angular; } TwistVel;
typedef struct {
    float max_v, max_w, dt, predict_time;
    int v_samples, w_samples;
    float w_goal, w_obstacle, w_speed;
} DWAConfig;

typedef struct { float x, y, theta; } Pose2D;

static uint8_t Slam_GetCell(int, int);
static float gGoalX = 2.0f, gGoalY = 0.0f;

TwistVel DWA_Plan(const Pose2D *pose, float goal_x, float goal_y)
{
    DWAConfig cfg = {0.3f, 1.0f, 0.1f, 3.0f, 5, 4, 0.5f, 0.4f, 0.1f};
    TwistVel best = {0, 0}; float best_cost = 1e9f;

    for (int vi = 0; vi < cfg.v_samples; vi++) {
        float v = cfg.max_v * (float)vi / (float)(cfg.v_samples - 1);
        for (int wi = 0; wi < cfg.w_samples; wi++) {
            float w = cfg.max_w * (2.0f * (float)wi / (float)(cfg.w_samples - 1) - 1.0f);

            float px = pose->x, py = pose->y, pt = pose->theta;
            float cost = 0; int collision = 0;
            for (float t = 0; t < cfg.predict_time; t += cfg.dt) {
                pt += w * cfg.dt; px += v * cosf(pt) * cfg.dt; py += v * sinf(pt) * cfg.dt;
                int cx = (int)(px / 0.05f), cy = (int)(py / 0.05f);
                if (Slam_GetCell(cx, cy) > 200) { collision = 1; break; }
            }
            if (collision) continue;

            float dist = sqrtf((px - goal_x) * (px - goal_x) + (py - goal_y) * (py - goal_y));
            cost = cfg.w_goal * dist + cfg.w_speed * (cfg.max_v - v);

            if (cost < best_cost) { best_cost = cost; best.linear = v; best.angular = w; }
        }
    }
    return best;
}
