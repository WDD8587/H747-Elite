#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <mutex>
#include "ipc_proto.h"

static int gWidth = 1000, gHeight = 1000;
static float gRes = 0.05f;
static uint8_t *gMap = NULL;
static float gPoseX, gPoseY, gPoseTheta;
static std::mutex gMutex;

void Slam_Init(int w, int h, float res)
{
    gWidth = w; gHeight = h; gRes = res;
    gMap = (uint8_t *)calloc((size_t)(w * h), 1);
    gPoseX = (float)w * res / 2; gPoseY = (float)h * res / 2; gPoseTheta = 0;
    printf("[SLAM] Grid %dx%d @ %.2fm (%.1fx%.1fm)\n", w, h, res, w*res, h*res);
}

void Slam_UpdatePose(float x, float y, float theta)
{
    std::lock_guard<std::mutex> lk(gMutex);
    gPoseX = x; gPoseY = y; gPoseTheta = theta;
}

void Slam_AddObstacle(float wx, float wy, float weight)
{
    std::lock_guard<std::mutex> lk(gMutex);
    int cx = (int)(wx / gRes), cy = (int)(wy / gRes);
    if (cx >= 0 && cx < gWidth && cy >= 0 && cy < gHeight) {
        int idx = cy * gWidth + cx;
        int val = (int)(weight * 255.0f);
        if (val > (int)gMap[idx]) gMap[idx] = (uint8_t)val;
    }
}

uint8_t Slam_GetCell(int x, int y)
{
    if (!gMap || x < 0 || x >= gWidth || y < 0 || y >= gHeight) return 0;
    return gMap[y * gWidth + x];
}

void Slam_GetPose(float *x, float *y, float *theta)
{
    std::lock_guard<std::mutex> lk(gMutex);
    if (x) *x = gPoseX; if (y) *y = gPoseY; if (theta) *gPoseTheta;
}
