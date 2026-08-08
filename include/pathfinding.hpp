#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <vector>
#include "generics.hpp"

struct Neighbors {
    Position data[8];
    int count = 0;

    Position* begin() { return data; }
    Position* end()   { return data + count; }
    const Position* begin() const { return data; }
    const Position* end()   const { return data + count; }
};

static constexpr std::array<std::pair<int, int>, 8> NEIGHB {{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, -1}, {-1, -1}, {-1, 1}, {1, 1}
}};

template<typename CostFn, typename Probe, typename Heur, typename Scan>
requires CostFunction<CostFn> && ProbeFunction<Probe> && Heuristic<Heur> && SensorFunction<Scan>
struct DStarLiteNavigator {
    DStarLiteNavigator(int width, int height,
                       CostFn costFunc, Probe probe, Scan scan,
                       Heur heuristic = zeroHeuristic)
        : width_(width), height_(height),
          costFunc_(costFunc), probe_(probe), scan_(scan), heuristic_(heuristic) {
            currentState_.resize(width_, height_);
            std::ranges::fill(g_, INF);
            std::ranges::fill(rhs_, INF);
            std::ranges::fill(inOpen_, false);
        }

    void reset() {
        initialized_ = false;
        std::ranges::fill(g_, INF);
        std::ranges::fill(rhs_, INF);
        std::ranges::fill(inOpen_, false);
        while (!open_.empty()) open_.pop();
        km = 0.0f;
        lastStart_ = Position{};
        goal_ = Position{};
        currentState_ = Region{};
    }
        
    std::vector<Position> replan(Position start, Position end) {
        if (!initialized_) {
            lastStart_ = start;
            initialize(end);
            scan(start);
            computeShortestPath(start);
            initialized_ = true;
        } else {
            km += heuristic_(lastStart_, start);
            scan(start);
            computeShortestPath(start);
            lastStart_ = start;
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
    CostFn costFunc_;
    Probe probe_;
    Scan scan_;
    Heur heuristic_;

    Region currentState_;
    std::vector<float> g_, rhs_;
    std::vector<bool> inOpen_;
    std::priority_queue<keyPos, std::vector<keyPos>, keyGreater> open_;

    float km{0.0f};
    Position lastStart_, goal_;
    bool initialized_{false};

    inline int idx(int x, int y) const { return y * width_ + x; }
    inline bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }

    Neighbors getNeighbors(Position s) const {
        Neighbors n;
        for (auto [dx, dy] : NEIGHB) {
            int nx = s.x + dx, ny = s.y + dy;
            if (inBounds(nx, ny)) {
                n.data[n.count++] = {.x = nx, .y = ny};
            }
        }
        return n;
    }

    float edgeCost(Position a, Position b) const {
        return costFunc_(currentState_[a.x, a.y], currentState_[b.x, b.y], a, b);
    };

    key calculateKey(Position s) const {
        float gVal = g_[idx(s.x, s.y)];
        float rVal = rhs_[idx(s.x, s.y)];
        float m  = std::min(gVal, rVal);
        return {.k1 = m + heuristic_(lastStart_, s) + km, .k2 = m};
    }

    void push(Position s) {
        open_.push({.k = calculateKey(s), .pos = s});
        inOpen_[idx(s.x, s.y)] = true;
    }

    void remove(Position s) {
        inOpen_[idx(s.x, s.y)] = false;
    }

    void initialize(Position end) {
        goal_ = end;
        std::ranges::fill(g_, INF);
        std::ranges::fill(rhs_, INF);
        std::ranges::fill(inOpen_, false);
        while (!open_.empty()) open_.pop();
        km = 0.0f;
        rhs_[idx(goal_.x, goal_.y)] = 0.0f;
        push(goal_);
    }

    void scan(Position robot){
        std::vector<Position> dirty = scan_(robot, currentState_);

        for (const Position& d : dirty) {
            Cell truth = probe_(d);
            Cell& cur  = currentState_[d.x, d.y];
            cur = truth;
            cur.is_visited = true;
        }

        for (const Position& d : dirty) {
            updateVertex(d);
            for (const auto& n : getNeighbors(d))
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
            
            if (!inOpen_[idx(top.pos.x, top.pos.y)]) continue;
            
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
            bool startConsistent = std::abs(gStart - rhsStart) < 1e-5f;
            if (!(top.k < kStart) && startConsistent) break;

            Position u = top.pos;
            remove(u);
            
            float gu = g_[idx(u.x, u.y)];
            float rhsu = rhs_[idx(u.x, u.y)];

            if (gu > rhsu) {
                g_[idx(u.x, u.y)] = rhsu;
                for (const auto& n : getNeighbors(u)) updateVertex(n);
            } else if (gu < rhsu) {
                g_[idx(u.x, u.y)] = INF;
                for (const auto& n : getNeighbors(u)) updateVertex(n);
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
            float best = INF;
            for (const auto& n : getNeighbors(s)) {
                best = std::min(best, edgeCost(s, n) + g_[idx(n.x, n.y)]);
            }
            rhs_[i] = best;
        }
        if (inOpen_[i]) {
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
        if (g_[idx(start.x, start.y)] == INF) return path;

        Position cur = start;
        path.push_back(cur);
        int guard = 0, maxSteps = width_ * height_ + 8;

        while (!(cur.x == goal_.x && cur.y == goal_.y)) {
            if (++guard > maxSteps) break;
            Position best = cur;
            float bestVal = INF;
            for (const auto& n : getNeighbors(cur)) {
                float v = edgeCost(cur, n) + g_[idx(n.x, n.y)];
                if (v < bestVal) { bestVal = v; best = n; }
            }
            if (bestVal == INF) break;
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
                       CostFn costFunc, Probe probe, Scan scan,
                       Heur heuristic = zeroHeuristic)
        : width_(width), height_(height),
          costFunc_(costFunc), probe_(probe), scan_(scan), heuristic_(heuristic) {
            currentState_.resize(width_, height_);
            auto N = static_cast<size_t>(width_) * height_;
            
            g_.assign(N, INF);
            generated_id_.assign(N, 0);
            closed_id_.assign(N, 0);
            cameFrom_.assign(N, {-1, -1});
            open_.reserve(N);
        }

    void reset() {
        initialized_ = false;
        auto N = static_cast<size_t>(width_) * height_;

        currentSearchId_ = 0;
        g_.assign(N, INF);
        generated_id_.assign(N, 0);
        closed_id_.assign(N, 0);
        cameFrom_.assign(N, {-1, -1});
        open_.clear();

        currentState_ = Region{};
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
    CostFn costFunc_;
    Probe probe_;
    Scan scan_;
    Heur heuristic_;

    Region currentState_{};
    std::vector<float> g_;
    std::vector<bool> closed_;
    std::vector<Position> cameFrom_;

    int currentSearchId_ = 0;
    std::vector<int> generated_id_; 
    std::vector<int> closed_id_;

    Position goal_{};
    bool initialized_{false};

    struct Node { 
        float f;    
        float h;
        Position pos;
    };
    struct NodeGreater {
        bool operator()(const Node& a, const Node& b) const {
            if (std::abs(a.f - b.f) < 1e-5f) return a.h > b.h;
            return a.f > b.f;
        }
    };
    std::vector<Node> open_; 

    inline int idx(int x, int y) const { return y * width_ + x; }
    inline bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }

    Neighbors getNeighbors(Position s) const {
        Neighbors n;
        for (auto [dx, dy] : NEIGHB) {
            int nx = s.x + dx, ny = s.y + dy;
            if (inBounds(nx, ny)) {
                n.data[n.count++] = {.x = nx, .y = ny};
            }
        }
        return n;
    }

    float edgeCost(Position a, Position b) const {
        const auto& cellA = currentState_[a.x, a.y];
        const auto& cellB = currentState_[b.x, b.y];
        return costFunc_(cellA, cellB, a, b);
    }

    void scan(Position robot){
        std::vector<Position> dirty = scan_(robot, currentState_);

        for (const Position& d : dirty) {
            Cell truth = probe_(d);
            Cell& cur  = currentState_[d.x, d.y];
            cur = truth;
            cur.is_visited = true;
        }
    }

    void initialize(Position end) {
        goal_ = end;
    }

    void computeShortestPath(Position start, Position end) {
        currentSearchId_++;

        open_.clear();

        int startIdx = idx(start.x, start.y);
        generated_id_[startIdx] = currentSearchId_;
        g_[startIdx] = 0.0f;
        
        float start_h = heuristic_(start, end);
        open_.push_back(Node{start_h, start_h, start});
        std::push_heap(open_.begin(), open_.end(), NodeGreater{});
    
        while (!open_.empty()) {
            std::pop_heap(open_.begin(), open_.end(), NodeGreater{});
            Node current = open_.back();
            open_.pop_back();
    
            int currentIdx = idx(current.pos.x, current.pos.y);
            
            if (closed_id_[currentIdx] == currentSearchId_) continue;
            closed_id_[currentIdx] = currentSearchId_;
    
            if (current.pos.x == end.x && current.pos.y == end.y) break;
    
            for (const auto& neighb : getNeighbors(current.pos)) {
                int neighbIdx = idx(neighb.x, neighb.y);
                if (closed_id_[neighbIdx] == currentSearchId_) continue;
                
                float currentNeighbG = (generated_id_[neighbIdx] == currentSearchId_) ? g_[neighbIdx] : INF;
                float newCost = g_[currentIdx] + edgeCost(current.pos, neighb);
                
                if (newCost >= currentNeighbG) continue;
                
                generated_id_[neighbIdx] = currentSearchId_;
                g_[neighbIdx] = newCost;
                cameFrom_[neighbIdx] = current.pos;
                
                float h = heuristic_(neighb, end);
                open_.push_back({newCost + h, h, neighb});
                std::push_heap(open_.begin(), open_.end(), NodeGreater{});
            }
        }
    }

    std::vector<Position> reconstructPath(Position start) const {
        std::vector<Position> path;
        int goalIdx = idx(goal_.x, goal_.y);
        if (g_[goalIdx] == INF) return path;

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
struct MPAAStarNavigator {
    MPAAStarNavigator(int width, int height,
                       CostFn costFunc, Probe probe, Scan scan,
                       Heur heuristic = zeroHeuristic)
        : width_(width), height_(height),
          costFunc_(costFunc), probe_(probe), scan_(scan), heuristic_(heuristic) {
            currentState_.resize(width_, height);
            auto N = static_cast<size_t>(width_) * height_;
            g_.assign(N, INF);
            h_.assign(N, 0.0f);
            search_.assign(N, 0);
            closed_.assign(N, false);
            cameFrom_.assign(N, {-1, -1});
            next_.assign(N, {-1, -1});
            open_.reserve(N);
            closedThisRun_.reserve(N / 4);
        }

    void reset(){
        initialized_ = false;
        counter_ = 0;
        auto N = static_cast<size_t>(width_) * height_;
        g_.assign(N, INF);
        h_.assign(N, 0.0f);
        search_.assign(N, 0);
        closed_.assign(N, false);
        cameFrom_.assign(N, {-1, -1});
        next_.assign(N, {-1, -1});
        open_.clear();
        closedThisRun_.clear();

        goal_ = Position{};
        currentState_ = Region{};
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

    struct Node {
        float f;
        Position pos;
    };
    struct NodeGreater {
        bool operator()(const Node& a, const Node& b) const { return a.f > b.f; }
    };

    int width_, height_;
    CostFn costFunc_;
    Probe probe_;
    Scan scan_;
    Heur heuristic_;

    Region currentState_{};
    std::vector<float> g_;
    std::vector<float> h_;   
    std::vector<int> search_;  
    std::vector<bool> closed_;
    std::vector<Position> cameFrom_;
    std::vector<Position> next_; 

    Position goal_{};
    int counter_{0};
    bool initialized_{false};

    std::vector<Node> open_;          
    std::vector<int> closedThisRun_; 

    inline int idx(int x, int y) const { return y * width_ + x; }
    inline bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }

    Neighbors getNeighbors(Position s) const {
        Neighbors n;
        for (auto [dx, dy] : NEIGHB) {
            int nx = s.x + dx, ny = s.y + dy;
            if (inBounds(nx, ny)) {
                n.data[n.count++] = {.x = nx, .y = ny};
            }
        }
        return n;
    }

    float edgeCost(Position a, Position b) const {
        const auto& cellA = currentState_[a.x, a.y];
        const auto& cellB = currentState_[b.x, b.y];
        return costFunc_(cellA, cellB, a, b);
    }

    void scan(Position robot){
        std::vector<Position> dirty = scan_(robot, currentState_);

        for (const Position& d : dirty) {
            Cell old = currentState_[d.x, d.y];
            Cell truth = probe_(d);
            Cell& cur  = currentState_[d.x, d.y];
            cur = truth;
            cur.is_visited = true;

            bool costIncreased = false;
            if (truth.is_impassable && !old.is_impassable) costIncreased = true;

            // Observe(s): if cost of an arc increased, null next pointers for the arc endpoints.
            if (costIncreased) {
                next_[idx(d.x, d.y)] = {-1, -1};
                for (const auto& n : getNeighbors(d))
                    next_[idx(n.x, n.y)] = {-1, -1};
            }
        }
    }

    void initialize(Position end) {
        goal_ = end;
        for (int y = 0; y < height_; ++y)
            for (int x = 0; x < width_; ++x)
                h_[idx(x, y)] = heuristic_({x, y}, end);
        std::ranges::fill(search_, 0);
        std::ranges::fill(next_, Position{-1, -1});
        counter_ = 0;
    }

    void initializeState(Position s) {
        int i = idx(s.x, s.y);
        if (search_[i] != counter_) {
            g_[i] = INF;
            search_[i] = counter_;
        }
    }

    bool goalCondition(Position s) const {
        Position cur = s;
        while (next_[idx(cur.x, cur.y)].x != -1) {
            Position nxt = next_[idx(cur.x, cur.y)];
            float hCur = h_[idx(cur.x, cur.y)];
            float hNxt = h_[idx(nxt.x, nxt.y)];
            float cost = edgeCost(cur, nxt);
            if (std::abs(hCur - (hNxt + cost)) > 1e-5f) return false;
            cur = nxt;
        }
        return cur.x == goal_.x && cur.y == goal_.y;
    }

    void computeShortestPath(Position start, Position end) {
        counter_++;

        open_.clear();
        closedThisRun_.clear();

        int startIdx = idx(start.x, start.y);
        initializeState(start);
        g_[startIdx] = 0.0f;
        cameFrom_[startIdx] = {-1, -1};

        open_.push_back(Node{h_[startIdx], start});
        std::push_heap(open_.begin(), open_.end(), NodeGreater{});

        Position result = {-1, -1};

        while (!open_.empty()) {
            std::pop_heap(open_.begin(), open_.end(), NodeGreater{});
            Node current = open_.back();
            open_.pop_back();

            int curIdx = idx(current.pos.x, current.pos.y);
            if (closed_[curIdx]) continue;
            closed_[curIdx] = true;
            closedThisRun_.push_back(curIdx);

            if (goalCondition(current.pos)) {
                result = current.pos;
                break;
            }

            for (const auto& neighb : getNeighbors(current.pos)) {
                int nIdx = idx(neighb.x, neighb.y);
                if (closed_[nIdx]) continue;

                initializeState(neighb);

                float newCost = g_[curIdx] + edgeCost(current.pos, neighb);
                if (newCost >= g_[nIdx]) continue;

                g_[nIdx] = newCost;
                cameFrom_[nIdx] = current.pos;
                open_.push_back(Node{newCost + h_[nIdx], neighb});
                std::push_heap(open_.begin(), open_.end(), NodeGreater{});
            }
        }

        // Heuristic update: for each state in Closed, h(s0) = g(s) + h(s) - g(s0).
        if (result.x != -1) {
            int resIdx = idx(result.x, result.y);
            float gResult = g_[resIdx];
            float hResult = h_[resIdx];
            for (int currentIdx : closedThisRun_) {
                if (search_[currentIdx] == counter_ && g_[currentIdx] != INF) {
                    h_[currentIdx] = gResult + hResult - g_[currentIdx];
                }
            }

            // BuildPath(result): set next pointers along the newly found prefix.
            Position cur = result;
            while (!(cur.x == start.x && cur.y == start.y)) {
                int curIdx = idx(cur.x, cur.y);
                Position par = cameFrom_[curIdx];
                if (par.x == -1 && par.y == -1) break;
                int parIdx = idx(par.x, par.y);
                next_[parIdx] = cur;
                cur = par;
            }
        }

        for (int ci : closedThisRun_)
            closed_[ci] = false;
    }

    std::vector<Position> reconstructPath(Position start) const {
        std::vector<Position> path;
        int startIdx = idx(start.x, start.y);
        if (next_[startIdx].x == -1 && !(start.x == goal_.x && start.y == goal_.y))
            return path;

        Position cur = start;
        path.push_back(cur);
        int guard = 0, maxSteps = width_ * height_ + 8;
        while (!(cur.x == goal_.x && cur.y == goal_.y)) {
            if (++guard > maxSteps) break;
            int curIdx = idx(cur.x, cur.y);
            if (next_[curIdx].x == -1) break;
            cur = next_[curIdx];
            path.push_back(cur);
        }
        return path;
    }
};

template<typename CostFn, typename Probe, typename Heur, typename Scan>
requires CostFunction<CostFn> && ProbeFunction<Probe> && Heuristic<Heur> && SensorFunction<Scan>
MPAAStarNavigator(int, int, CostFn, Probe, Scan, Heur)
    -> MPAAStarNavigator<CostFn, Probe, Heur, Scan>;

#endif
