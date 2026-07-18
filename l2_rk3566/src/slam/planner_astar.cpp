#include <vector>
#include <queue>
#include <cmath>

typedef struct { int x, y; } GridCoord;

struct Node {
    int x, y, g, h;
    Node *parent;
    int f() const { return g + h; }
    bool operator<(const Node &o) const { return f() > o.f(); }
};

static int heuristic(int x1, int y1, int x2, int y2) {
    return (int)sqrtf((float)((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))) * 10;
}

static uint8_t Slam_GetCell(int, int);

std::vector<GridCoord> AStar_Plan(int sx, int sy, int gx, int gy, int w, int h)
{
    std::vector<GridCoord> path;
    std::priority_queue<Node> open;
    Node start = {sx, sy, 0, heuristic(sx, sy, gx, gy), NULL};
    open.push(start);

    static const int dx[8] = {-1,0,1,-1,1,-1,0,1};
    static const int dy[8] = {-1,-1,-1,0,0,1,1,1};

    for (int iter = 0; iter < 1000 && !open.empty(); iter++) {
        Node cur = open.top(); open.pop();
        if (cur.x == gx && cur.y == gy) {
            path.push_back({gx, gy});
            return path;
        }
        for (int d = 0; d < 8; d++) {
            int nx = cur.x + dx[d], ny = cur.y + dy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (Slam_GetCell(nx, ny) > 200) continue;
            Node next = {nx, ny, cur.g + ((d < 4) ? 10 : 14), heuristic(nx, ny, gx, gy), NULL};
            open.push(next);
        }
    }
    return path;
}
