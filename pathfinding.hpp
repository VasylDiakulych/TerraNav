#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <ranges>
#include <utility>
#include <vector>
#include "generics.hpp"

inline constexpr float kInf = std::numeric_limits<float>::infinity();

template<typename CostFn, typename Probe, typename Heur, typename Scan>
requires CostFunction<CostFn> && ProbeFunction<Probe> && Heuristic<Heur> && SensorFunction<Scan>
struct DStarLiteNavigator {
    DStarLiteNavigator(int width, int height,
                       CostFn cost_func, Probe probe, Scan scan,
                       Heur heuristic = zeroHeuristic)
        : width_(width), height_(height),
          cost_func_(cost_func), probe_(probe), scan_(scan), heuristic_(heuristic) {
            current_state_.resize(width, height);
            auto N = static_cast<size_t>(width) * height;
            g_.assign(N, kInf);
            rhs_.assign(N, kInf);
            in_open_.assign(N, false);
        }

    std::vector<Position> replan(Position start, Position end) {
        if (!initialized_) {
            last_start_ = start;
            initialize(end);
            scan(start);
            computeShortestPath(start);
            initialized_ = true;
        } else {
            km += heuristic_(last_start_, start);
            scan(start);
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
    CostFn cost_func_;
    Probe probe_;
    Scan scan_;
    Heur heuristic_;

    Region current_state_;
    std::vector<float> g_, rhs_;
    std::vector<bool> in_open_;
    std::priority_queue<keyPos, std::vector<keyPos>, keyGreater> open_;

    float km{0.0f};
    Position last_start_, goal_;
    bool initialized_{false};

    static constexpr std::array<std::pair<int, int>, 8> kNeighb {{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, -1}, {-1, -1}, {-1, 1}, {1, 1}
    }};

    inline int idx(int x, int y) const { return y * width_ + x; }
    inline bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }

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

    void push(Position s) {
        open_.push({.k = calculateKey(s), .pos = s});
        in_open_[idx(s.x, s.y)] = true;
    }

    void remove(Position s) {
        in_open_[idx(s.x, s.y)] = false;
    }

    void initialize(Position end) {
        goal_ = end;
        std::ranges::fill(g_, kInf);
        std::ranges::fill(rhs_, kInf);
        std::ranges::fill(in_open_, false);
        while (!open_.empty()) open_.pop();
        km = 0.0f;
        rhs_[idx(goal_.x, goal_.y)] = 0.0f;
        push(goal_);
    }

    void scan(Position robot){
        std::vector<Position> dirty = scan_(robot, current_state_);

        for (const Position& d : dirty) {
            Cell truth = probe_(d);
            Cell& cur  = current_state_[d.x, d.y];
            cur = truth;
            cur.is_visited = true;
        }

        for (const Position& d : dirty) {
            updateVertex(d);
            for (const Position& n : neighbors(d)) 
                updateVertex(n);
        }
    }

    void computeShortestPath(Position start) {
        #ifdef DSTAR_DEBUG
        int popped = 0, stale = 0, staleRepush = 0, staleDrop = 0, processed = 0;
        size_t openSizeBefore = open_.size();
        #endif
        while (!open_.empty()) {
            keyPos top = open_.top();
            open_.pop();
            #ifdef DSTAR_DEBUG
            popped++;
            #endif

            if (!(top.pos.x >= 0 && top.pos.x < width_ 
                && top.pos.y >= 0 && top.pos.y < height_))
                continue;
            
            if (!in_open_[idx(top.pos.x, top.pos.y)]) continue;
            
            key kNew = calculateKey(top.pos);
            if (top.k < kNew) {
                #ifdef DSTAR_DEBUG
                staleRepush++;
                #endif
                push(top.pos);
                continue;
            }
            if (top.k > kNew) {
                #ifdef DSTAR_DEBUG
                staleDrop++;
                #endif
                continue;
            }

            #ifdef DSTAR_DEBUG
            processed++;
            #endif

            key kStart = calculateKey(start);
            float gStart   = g_[idx(start.x, start.y)];
            float rhsStart = rhs_[idx(start.x, start.y)];
            bool startConsistent = (gStart == rhsStart);
            if (!(top.k < kStart) && startConsistent) break;

            Position u = top.pos;
            remove(u);
            
            float gu = g_[idx(u.x, u.y)];
            float rhsu = rhs_[idx(u.x, u.y)];

            if (gu > rhsu) {
                g_[idx(u.x, u.y)] = rhsu;
                for (const Position& n : neighbors(u)) updateVertex(n);
            } else if (gu < rhsu) {
                g_[idx(u.x, u.y)] = kInf;
                for (const Position& n : neighbors(u)) updateVertex(n);
                updateVertex(u);
            }
        }
        #ifdef DSTAR_DEBUG
        if (staleRepush > 0 || staleDrop > 0 || popped > processed + 5) {
            std::cerr << "[DSTAR] popped=" << popped
                      << " repush=" << staleRepush << " drop=" << staleDrop
                      << " processed=" << processed
                      << " openSz=" << openSizeBefore << "\n";
        }
        #endif
    }

    void updateVertex(Position s) {
        int i = idx(s.x, s.y);
        if (!(s.x == goal_.x && s.y == goal_.y)) {
            float best = kInf;
            for (const Position& n : neighbors(s)) {
                best = std::min(best, edgeCost(s, n) + g_[idx(n.x, n.y)]);
            }
            rhs_[i] = best;
        }
        if (in_open_[i]) {
            if (g_[i] == rhs_[i]) {
                remove(s);
            } else {
                push(s);
            }
        } else if (g_[i] != rhs_[i]) {
            push(s);
        }
    }

    std::vector<Position> reconstructPath(Position start) const {
        std::vector<Position> path;
        if (g_[idx(start.x, start.y)] == kInf) return path;

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
            if (bestVal == kInf) break;
            cur = best;
            path.push_back(cur);
        }

        return path;
    }
};

template<typename CostFn, typename Probe, typename Heur, typename Scan>
    requires CostFunction<CostFn> && ProbeFunction<Probe> && Heuristic<Heur> && SensorFunction<Scan>
DStarLiteNavigator(int, int, CostFn, Probe, Scan, Heur)
    -> DStarLiteNavigator<CostFn, Probe, Heur, Scan>;

template<typename CostFn, typename Probe, typename Heur, typename Scan>
requires CostFunction<CostFn> && ProbeFunction<Probe> && Heuristic<Heur> && SensorFunction<Scan>
struct NaiveAStarNavigator {
    NaiveAStarNavigator(int width, int height,
                       CostFn cost_func, Probe probe, Scan scan,
                       Heur heuristic = zeroHeuristic)
        : width_(width), height_(height),
          cost_func_(cost_func), probe_(probe), scan_(scan), heuristic_(heuristic) {
            current_state_.resize(width, height);
            auto N = static_cast<size_t>(width) * height;
            g_.assign(N, kInf);
            closed_.assign(N, false);
            cameFrom_.assign(N, {-1, -1});
        }

    std::vector<Position> replan(Position start, Position end) {
        if (!initialized_) {
            initialize(end);
            scan(start);
            computeShortestPath(start, end);
            initialized_ = true;
        } else {
            scan(start);
            computeShortestPath(start, end);
        }
        return reconstructPath(start);
    }
        
    int width_, height_;
    CostFn cost_func_;
    Probe probe_;
    Scan scan_;
    Heur heuristic_;

    Region current_state_{};
    std::vector<float> g_;
    std::vector<bool> closed_;
    std::vector<Position> cameFrom_;

    Position goal_{};
    bool initialized_{false};

    static constexpr std::array<std::pair<int, int>, 8> kNeighb {{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, -1}, {-1, -1}, {-1, 1}, {1, 1}
    }};

    inline int idx(int x, int y) const { return y * width_ + x; }
    inline bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }

    auto neighbors(Position s) const {
        return kNeighb
             | std::views::transform([s](auto p) { auto [dx, dy] = p; return Position{.x = s.x + dx, .y = s.y + dy}; })
             | std::views::filter([this](Position p) { return inBounds(p.x, p.y); });
    }

    float edgeCost(Position a, Position b) const {
        const auto& cellA = current_state_[a.x, a.y];
        const auto& cellB = current_state_[b.x, b.y];
        if(cellA.is_impassable || cellB.is_impassable) { return kInf; }
        float d = (a.x == b.x || a.y == b.y) ? 1.0f : std::sqrt(2.0f);
        return 0.5f * (cost_func_(cellA) + cost_func_(cellB)) * d;
    }

    void scan(Position robot){
        std::vector<Position> dirty = scan_(robot, current_state_);

        for (const Position& d : dirty) {
            Cell truth = probe_(d);
            Cell& cur  = current_state_[d.x, d.y];
            cur = truth;
            cur.is_visited = true;
        }
    }

    void initialize(Position end) {
        goal_ = end;
    }

    void computeShortestPath(Position start, Position end) {
        std::ranges::fill(g_, kInf);
        std::ranges::fill(closed_, false);

        struct Node { 
            float f; 
            Position pos;
            bool operator<(const Node& o) const { return f > o.f; }
        };

        int startIdx = idx(start.x, start.y);
        g_[startIdx] = 0.0f;
        std::priority_queue<Node> open;
        open.push({heuristic_(start, end), start});

        while (!open.empty()) {
            Node current = open.top();
            open.pop();

            int currentIdx = idx(current.pos.x, current.pos.y);
            if (closed_[currentIdx]) continue;
            closed_[currentIdx] = true;

            if (current.pos.x == end.x && current.pos.y == end.y) break;

            for (const Position& neighb : neighbors(current.pos)) {
                int neighbIdx = idx(neighb.x, neighb.y);
                if (closed_[neighbIdx]) continue;
                float newCost = g_[currentIdx] + edgeCost(current.pos, neighb);
                if (newCost >= g_[neighbIdx]) continue;
                g_[neighbIdx] = newCost;
                cameFrom_[neighbIdx] = current.pos;
                open.push({newCost   + heuristic_(neighb, end), neighb});
            }
        }
    }

    std::vector<Position> reconstructPath(Position start) const {
        std::vector<Position> path;
        int goalIdx = idx(goal_.x, goal_.y);
        if (g_[goalIdx] == kInf) return path;

        Position cur = goal_;
        while (!(cur.x == start.x && cur.y == start.y)) {
            path.push_back(cur);
            cur = cameFrom_[idx(cur.x, cur.y)];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }
};

template<typename CostFn, typename Probe, typename Heur, typename Scan>
requires CostFunction<CostFn> && ProbeFunction<Probe> && Heuristic<Heur> && SensorFunction<Scan>
NaiveAStarNavigator(int, int, CostFn, Probe, Scan, Heur)
    -> NaiveAStarNavigator<CostFn, Probe, Heur, Scan>;

template<typename CostFn, typename Probe, typename Heur, typename Scan>
requires CostFunction<CostFn> && ProbeFunction<Probe> && Heuristic<Heur> && SensorFunction<Scan>
struct MultiPathAdaptiveAStar {
    MultiPathAdaptiveAStar(){
        
    };
};
    
#endif
