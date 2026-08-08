#ifndef GENERICS_HPP
#define GENERICS_HPP

#include <concepts>
#include <vector>
#include <cmath>
#include <limits>

inline constexpr float INF = std::numeric_limits<float>::infinity();

struct Position {
    int x{0}, y{0};
    float direction{0.0f};
};

struct Cell {
    float absolute_elevation{0.0f};
    float craterDelta{0.0f};
    float slope_dx{0.0f};
    float slope_dy{0.0f};

    float roughness{0.0f};
    float humidity{0.0f};

    float wind_speed{0.0f};
    float wind_dir{0.0f};

    bool is_impassable{false};
    bool is_rock{false};

    bool is_visited{false};

    [[nodiscard]] float combinedElevation() const { return absolute_elevation + craterDelta; }
};

struct Region{
    int width{100};
    int height{100};

    std::vector<Cell> cells;

    void resize(int widthNew, int heightNew) {
        width = widthNew;
        height = heightNew;
        cells.assign(static_cast<std::size_t>(width) * height, Cell{});
    }

    Cell& operator[](int x, int y){
        return cells[y * width + x];
    }

    const Cell& operator[](int x, int y) const {
        return cells[y * width + x];
    }
};

// General navigator
// Should be able for a given position calculate path to the goal
template<typename T>
concept Navigator = requires(T nav, Position start, Position goal  ){
    { nav.replan(start, goal) } -> std::convertible_to<std::vector<Position>>;
    { nav.reset() } -> std::same_as<void>;
};

// General heuristic
template<typename F>
concept Heuristic = requires(F f, Position start, Position end) {
    { f(start, end) } -> std::convertible_to<float>;
};

inline float zeroHeuristic(Position, Position) { return 0.0f; }

inline float euclideanHeuristic(Position a, Position b) {
    float dx = static_cast<float>(a.x - b.x);
    float dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

// Function, that for a given cell returns it's cost
template<typename F>
concept CostFunction = requires (const F& f, const Cell& a, const Cell& b, Position pa, Position pb) {
    { f(a, b, pa, pb) } -> std::convertible_to<float>;
};

inline float actualCost(const Cell& from, const Cell& to, Position a, Position b) {
    if (from.is_impassable || to.is_impassable) return INF;

    float dx = static_cast<float>(b.x - a.x);
    float dy = static_cast<float>(b.y - a.y);
    float distance = std::sqrt(dx*dx + dy*dy);
    if (distance < 1e-6f) return 0.0f;

    float elevationGain = to.combinedElevation() - from.combinedElevation();
    float slopeAngle = elevationGain / distance;
    float slopePenalty = slopeAngle > 0.10f
        ? (slopeAngle - 0.10f) * (slopeAngle - 0.10f) * 10.0f
        : 0.0f;

    float avgRoughness = 0.5f * (from.roughness + to.roughness);
    float avgHumidity = 0.5f * (from.humidity + to.humidity);
    float roughnessPenalty = avgRoughness * (1.0f + 3.0f * avgHumidity);

    float windX = std::cos(from.wind_dir);
    float windY = std::sin(from.wind_dir);
    float moveX = dx / distance;
    float moveY = dy / distance;
    float windAlignment = windX*moveX + windY*moveY;
    float windPenalty = from.wind_speed * (1.0f - windAlignment) * 0.5f;

    return distance * (1.0f + slopePenalty + 2.0f * roughnessPenalty + windPenalty);
}

template<typename F>
concept ProbeFunction = requires (const F& f, Position p) {
    { f(p) } -> std::convertible_to<Cell>;
};

template<typename F>
concept SensorFunction = requires (const F& f, Position robot, const Region& state) {
    { f(robot, state) } -> std::convertible_to<std::vector<Position>>;
};

// Default SensorFunction implementation
inline bool lineOfSight(const Region& state, int width, int height,
                         Position from, Position to) {
    int x0 = from.x, y0 = from.y, x1 = to.x, y1 = to.y;
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;

    float totalSq = static_cast<float>((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    if (totalSq == 0.0f) return true;

    float eFrom = state[from.x, from.y].combinedElevation();
    float eTo   = state[to.x, to.y].combinedElevation();

    while (true) {
        if (x == x1 && y == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 <  dx) { err += dx; y += sy; }
        if (x == x1 && y == y1) break;
        if (x < 0 || y < 0 || x >= width || y >= height) continue;
        if (!state[x, y].is_visited) continue;
        float t = ((x - x0) * (x1 - x0) + (y - y0) * (y1 - y0)) / totalSq;
        float lineH = eFrom + t * (eTo - eFrom);
        if (state[x, y].combinedElevation() > lineH) return false;
    }

    return true;
}

// reveals all cells in a disk of randius sensor_range aroung robot
inline auto defaultScan(float sensor_range) {
    return [sensor_range](Position robot, const Region& state) -> std::vector<Position> {
        std::vector<Position> result;
        int r = static_cast<int>(std::ceil(sensor_range));
        float r2 = sensor_range * sensor_range;

        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                int tx = robot.x + dx, ty = robot.y + dy;
                if (tx < 0 || ty < 0 || tx >= state.width || ty >= state.height) continue;
                if (dx * dx + dy * dy > r2) continue;

                Position t{.x = tx, .y = ty, .direction = 0.0f};

                // skip all cell that cannot be seen from current position
                if (!lineOfSight(state, state.width, state.height, robot, t)) continue;
                if (state[tx, ty].is_visited) continue;

                result.push_back(t);
            }
        }
        return result;
    };
}

#endif
