#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include <array>
#include <cmath>
#include <queue>
#include <ranges>
#include <utility>
#include <vector>
#include "generics.hpp"

inline constexpr float kInf = std::numeric_limits<float>::infinity();

inline float defaultCost(const Cell& c) {
    return c.final_cost
         + 0.1f * c.roughness
         + 0.05f * std::abs(c.slope_dx)
         + 0.05f * std::abs(c.slope_dy);
}

inline float zeroHeuristic(Position, Position) { return 0.0f; }

inline float euclideanHeuristic(Position a, Position b) {
    float dx = static_cast<float>(a.x - b.x);
    float dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

template<typename CostFn, typename ObserveFn, typename Heur>
    requires CostFunction<CostFn> && ObserveFunction<ObserveFn> && Heuristics<Heur>
struct DStarLiteNavigator {
    DStarLiteNavigator(int width, int height, float visibility_range, 
                       CostFn cost_func, ObserveFn observe, Heur heuristic = zeroHeuristic)
        : width_(width), height_(height), sensor_range_(visibility_range),
          cost_func_(cost_func), observe_(observe), heuristic_(heuristic) {
            current_state_.resize(width, height);
            g_.assign(static_cast<size_t>(width) * height, kInf);
            rhs_.assign(static_cast<size_t>(width) * height, kInf);
        }

    std::vector<Position> calculatePath(Position start, Position end) {
        if (!initialized_) {
            last_start_ = start;
            initialize(end);
            sense(start);
            computeShortestPath(start);
            initialized_ = true;
        } else {
            km += heuristic_(last_start_, start); 
            sense(start);
            computeShortestPath(start);
            last_start_ = start;
        }
        return reconstructPath(start);
    }

    struct key { float k1, k2; 
        bool operator==(const key&) const = default;
        auto operator<=>(const key&) const = default; };
    struct keyPos { key k; Position pos; };
    struct keyGreater {
        bool operator()(const keyPos& a, const keyPos& b){
            if(a.k.k1 != b.k.k1) return a.k.k1 > b.k.k1;
            return a.k.k2 > b.k.k2;
        }
    };

    int width_, height_;
    float sensor_range_;
    CostFn cost_func_;
    ObserveFn observe_;
    Heur heuristic_;

    Region current_state_;
    std::vector<float> g_, rhs_;
    std::priority_queue<keyPos, std::vector<keyPos>, keyGreater> open_;

    float km{0.0f};
    Position last_start_, goal_;
    bool initialized_{false};

    //  E  W  S  N  SE NE SW NW
    static constexpr std::array<std::pair<int, int>, 8> kNeighb {{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, -1}, {-1, -1}, {-1, 1}, {1, 1}
    }};

    int idx(int x, int y) const { return y * width_ + x; }
    bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }

    auto neighbors(Position s) const {
        return kNeighb
             | std::views::transform([s](auto p) { auto [dx, dy] = p; return Position{.x = s.x + dx, .y = s.y + dy}; })
             | std::views::filter([this](Position p) { return inBounds(p.x, p.y); });
    };

    float edgeCost(Position a, Position b) const {
        const auto& cellA = current_state_[a.x, a.y];
        const auto& cellB = current_state_[b.x, b.y];
        if(cellA.is_impassable || cellB.is_impassable) { return kInf; }
        float d = (a.x == b.x || a.y == b.y) ? 1.0f : std::sqrt(2.0f);
        return 0.5f * (cost_func_(cellA) + cost_func_(cellB)) * d;
    };

    key calculateKey(Position s) const {
        float gVal = g_[idx(s.x, s.y)];
        float rVal = rhs_[idx(s.x, s.y)];
        float m  = std::min(gVal, rVal);
        return {.k1 = m + heuristic_(last_start_, s) + km, .k2 = m};
    }

    void push(Position s) { open_.push({.k = calculateKey(s), .pos = s}); }
    
    void initialize(Position end) {
        goal_ = end;
        std::ranges::fill(g_, kInf);
        std::ranges::fill(rhs_, kInf);
        while (!open_.empty()) open_.pop();
        km = 0.0f;
        rhs_[idx(goal_.x, goal_.y)] = 0.0f;
        push(goal_);
    }

    bool lineOfSight(Position from, Position to) const {
        int x0 = from.x, y0 = from.y, x1 = to.x, y1 = to.y;
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int x = x0, y = y0;

        float totalSq = static_cast<float>((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
        if (totalSq == 0.0f) return true;

        float eFrom = current_state_[from.x, from.y].absolute_elevation;
        float eTo = current_state_[to.x, to.y].absolute_elevation;

        while(true){
            if(x == x1 && y == y1) break;

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
            if (x == x1 && y == y1) break;
            if (!inBounds(x, y)) continue;
            if (!current_state_[x, y].is_visited) continue; 
            float t = ((x - x0) * (x1 - x0) + (y - y0) * (y1 - y0)) / totalSq;
            float lineH = eFrom + t * (eTo - eFrom);
            if (current_state_[x, y].absolute_elevation > lineH) return false;
        }

        return true;
    }

    void sense(Position robot){
        std::vector<Position> dirty;
        int r = static_cast<int>(std::ceil(sensor_range_));
        float r2 = sensor_range_ * sensor_range_;
    
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                int tx = robot.x + dx, ty = robot.y + dy;
                if (!inBounds(tx, ty)) continue;
                if (dx * dx + dy * dy > r2) continue;           // outside sensor disc
    
                Position t{.x = tx, .y = ty, .direction = 0.0f};
                if (!lineOfSight(robot, t)) continue;           // occluded
                if (current_state_[tx, ty].is_visited) continue; // already sensed — cached

                Cell truth = observe_(t);
                Cell& cur  = current_state_[tx, ty];
                bool changed = true;  // first observation is always a change
    
                cur = truth;
                cur.is_visited = true;
                if (changed) dirty.push_back(t);
            }
        }
        for (const Position& d : dirty) {
            updateVertex(d);                                
            for (const Position& n : neighbors(d)) updateVertex(n);
        }
    }

    void computeShortestPath(Position start) {
        while (!open_.empty()) {
            while (!open_.empty()) {
                keyPos top = open_.top();
                if (!(top.pos.x >= 0 && top.pos.x < width_ && top.pos.y >= 0 && top.pos.y < height_)) {
                    open_.pop();
                    continue;
                }
                if (top.k == calculateKey(top.pos)) break;   
                open_.pop();                                 
            }
            if (open_.empty()) break;
    
            // --- termination check ---
            keyPos top = open_.top();
            Position u = top.pos;
            key ku = top.k;
            key kStart = calculateKey(start);
            float gStart   = g_[idx(start.x, start.y)];
            float rhsStart = rhs_[idx(start.x, start.y)];
            bool startConsistent = (gStart == rhsStart);
            if (!(ku < kStart) && startConsistent) break;     
    
            // --- process u ---
            open_.pop();
            float gu = g_[idx(u.x, u.y)];
            float rhsu = rhs_[idx(u.x, u.y)];
    
            if (gu > rhsu) {                                   // overconsistent
                g_[idx(u.x, u.y)] = rhsu;        
                for (const Position& n : neighbors(u)) updateVertex(n);
            } else if (gu < rhsu) {                            // underconsistent
                g_[idx(u.x, u.y)] = kInf;                     
                for (const Position& n : neighbors(u)) updateVertex(n);
                updateVertex(u);                     
            }
        }
    }

    void updateVertex(Position s) {
        if (!(s.x == goal_.x && s.y == goal_.y)) {
            float best = kInf;
            for (const Position& n : neighbors(s)) {
                best = std::min(best, edgeCost(s, n) + g_[idx(n.x, n.y)]);
            }
            rhs_[idx(s.x, s.y)] = best;
        }
        if (g_[idx(s.x, s.y)] != rhs_[idx(s.x, s.y)]) {
            push(s);
        }
    }

    std::vector<Position> reconstructPath(Position start) const {
        std::vector<Position> path;
        if (g_[idx(start.x, start.y)] == kInf) return path;   // unreachable
        
        Position cur = start;
        path.push_back(cur);
        int guard = 0, maxSteps = width_ * height_ + 8;
        
        while (!(cur.x == goal_.x && cur.y == goal_.y)) {
            if (++guard > maxSteps) break;
            Position best = cur;
            float bestVal = kInf;
            for (const Position& n : neighbors(cur)) {
                float v = edgeCost(cur, n) + g_[idx(n.x, n.y)];
                if (v < bestVal) { bestVal = v; best = n; }
            }
            if (bestVal == kInf) break;          // stuck
            cur = best;
            path.push_back(cur);
        }
        
        return path;
    }
    
};

template<typename CostFn, typename ObserveFn, typename Heur>
    requires CostFunction<CostFn> && ObserveFunction<ObserveFn> && Heuristics<Heur>
DStarLiteNavigator(int, int, float, CostFn, ObserveFn, Heur) -> DStarLiteNavigator<CostFn, ObserveFn, Heur>;

#endif
