#ifndef REGION_PATHFINDING_HPP
#define REGION_PATHFINDING_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

#include "generics.hpp"
#include "terrain_generation.hpp"

enum class GateDirection { Horizontal, Vertical };

struct Gate {
    int regionAX{0}, regionAY{0};
    int regionBX{0}, regionBY{0};
    GateDirection direction{GateDirection::Horizontal};

    // Inclusive range of passable cells along the border
    // Horizontal gate (regions side by side): varies along Y, at x = regionW-1 / x = 0
    // Vertical gate (regions stacked): varies along X, at y = regionH-1 / y = 0
    int startCoord{0}, endCoord{0};

    // Global cell coords of the range endpoints (for rendering)
    int globalStartX{0}, globalStartY{0};
    int globalEndX{0}, globalEndY{0};

    float cost{1.0f};
    bool blocked{false};

    // Returns the cell in the gate range closest to the given local position.
    Position getGoalCell(int localX, int localZ, int regionW, int regionH, int side) const {
        if (direction == GateDirection::Horizontal) {
            int clampedY = std::clamp(localZ, startCoord, endCoord);
            int x = (side == 0) ? regionW - 1 : 0;
            return { .x = x, .y = clampedY };
        } else {
            int clampedX = std::clamp(localX, startCoord, endCoord);
            int y = (side == 0) ? regionH - 1 : 0;
            return { .x = clampedX, .y = y };
        }
    }

    // Look-ahead version: biases the goal cell toward the next gate's coordinate range
    // so the drone enters the next region already positioned well for the exit after.
    // nextGateStart/nextGateEnd = the coordinate range of the next region's exit gate
    // nextIsHorizontal = whether the next gate varies along Y (horizontal) or X (vertical)
    Position getGoalCellLookAhead(int localX, int localZ, int regionW, int regionH, int side,
                                  int nextStart, int nextEnd, bool nextIsHorizontal) const {
        int targetCoord;

        if (direction == GateDirection::Horizontal) {
            // Current gate varies along Y — if next gate is also horizontal, align Y
            // If next gate is vertical, bias Y toward where the next gate's X range maps
            if (nextIsHorizontal) {
                // Next exit is also along Y — aim for the overlap or nearest point
                targetCoord = std::clamp((nextStart + nextEnd) / 2, startCoord, endCoord);
            } else {
                // Next exit is along X — still bias toward center of next gate's X range
                // mapped to our Y range proportionally
                targetCoord = std::clamp((nextStart + nextEnd) / 2, startCoord, endCoord);
            }
            // Blend: 70% look-ahead, 30% closest to drone
            int closest = std::clamp(localZ, startCoord, endCoord);
            targetCoord = static_cast<int>(targetCoord * 0.7f + closest * 0.3f);
            targetCoord = std::clamp(targetCoord, startCoord, endCoord);

            int x = (side == 0) ? regionW - 1 : 0;
            return { .x = x, .y = targetCoord };
        } else {
            if (!nextIsHorizontal) {
                targetCoord = std::clamp((nextStart + nextEnd) / 2, startCoord, endCoord);
            } else {
                targetCoord = std::clamp((nextStart + nextEnd) / 2, startCoord, endCoord);
            }
            int closest = std::clamp(localX, startCoord, endCoord);
            targetCoord = static_cast<int>(targetCoord * 0.7f + closest * 0.3f);
            targetCoord = std::clamp(targetCoord, startCoord, endCoord);

            int y = (side == 0) ? regionH - 1 : 0;
            return { .x = targetCoord, .y = y };
        }
    }
};

struct RegionStep {
    int regionX{0}, regionY{0};
    int gateIndex{-1};
};

struct RegionGraph {
    std::vector<Gate> gates;
    int regionsX{0}, regionsY{0};
    int regionW{0}, regionH{0};

    static bool cellPassable(const Cell& c) {
        return !c.is_impassable && !c.is_rock
            && std::hypot(c.slope_dx, c.slope_dy) < 0.1f;
    }

    void buildFromMap(const Map& map) {
        gates.clear();
        regionsX = static_cast<int>(map.width_);
        regionsY = static_cast<int>(map.height_);
        regionW = static_cast<int>(map.gen.regionWidth_);
        regionH = static_cast<int>(map.gen.regionHeight_);

        for (int ry = 0; ry < regionsY; ++ry)
            for (int rx = 0; rx < regionsX - 1; ++rx)
                detectHorizontalGates_(map, rx, ry, rx + 1, ry);

        for (int ry = 0; ry < regionsY - 1; ++ry)
            for (int rx = 0; rx < regionsX; ++rx)
                detectVerticalGates_(map, rx, ry, rx, ry + 1);
    }

    void detectHorizontalGates_(const Map& map, int ax, int ay, int bx, int by) {
        const Region& regA = map.tiles[ay * regionsX + ax];
        const Region& regB = map.tiles[by * regionsX + bx];

        int runStart = -1;
        for (int y = 0; y <= regionH; ++y) {
            bool passable = (y < regionH)
                && cellPassable(regA[regionW - 1, y])
                && cellPassable(regB[0, y]);

            if (passable && runStart < 0) {
                runStart = y;
            } else if (!passable && runStart >= 0) {
                if (y - runStart >= 2)
                    addHorizontalGate_(map, ax, ay, bx, by, runStart, y - 1);
                runStart = -1;
            }
        }
    }

    void detectVerticalGates_(const Map& map, int ax, int ay, int bx, int by) {
        const Region& regA = map.tiles[ay * regionsX + ax];
        const Region& regB = map.tiles[by * regionsX + bx];

        int runStart = -1;
        for (int x = 0; x <= regionW; ++x) {
            bool passable = (x < regionW)
                && cellPassable(regA[x, regionH - 1])
                && cellPassable(regB[x, 0]);

            if (passable && runStart < 0) {
                runStart = x;
            } else if (!passable && runStart >= 0) {
                if (x - runStart >= 2)
                    addVerticalGate_(map, ax, ay, bx, by, runStart, x - 1);
                runStart = -1;
            }
        }
    }

    void addHorizontalGate_(const Map& map, int ax, int ay, int bx, int by,
                            int yStart, int yEnd) {
        const Region& regA = map.tiles[ay * regionsX + ax];
        const Region& regB = map.tiles[by * regionsX + bx];

        float totalRough = 0.0f;
        int count = yEnd - yStart + 1;
        for (int y = yStart; y <= yEnd; ++y)
            totalRough += regA[regionW - 1, y].roughness + regB[0, y].roughness;
        float avgRough = totalRough / (count * 2);

        gates.push_back({
            .regionAX = ax, .regionAY = ay,
            .regionBX = bx, .regionBY = by,
            .direction = GateDirection::Horizontal,
            .startCoord = yStart, .endCoord = yEnd,
            .globalStartX = ax * regionW + regionW - 1,
            .globalStartY = ay * regionH + yStart,
            .globalEndX = ax * regionW + regionW - 1,
            .globalEndY = ay * regionH + yEnd,
            .cost = 1.0f + avgRough * 3.0f,
            .blocked = false
        });
    }

    void addVerticalGate_(const Map& map, int ax, int ay, int bx, int by,
                          int xStart, int xEnd) {
        const Region& regA = map.tiles[ay * regionsX + ax];
        const Region& regB = map.tiles[by * regionsX + bx];

        float totalRough = 0.0f;
        int count = xEnd - xStart + 1;
        for (int x = xStart; x <= xEnd; ++x)
            totalRough += regA[x, regionH - 1].roughness + regB[x, 0].roughness;
        float avgRough = totalRough / (count * 2);

        gates.push_back({
            .regionAX = ax, .regionAY = ay,
            .regionBX = bx, .regionBY = by,
            .direction = GateDirection::Vertical,
            .startCoord = xStart, .endCoord = xEnd,
            .globalStartX = ax * regionW + xStart,
            .globalStartY = ay * regionH + regionH - 1,
            .globalEndX = ax * regionW + xEnd,
            .globalEndY = ay * regionH + regionH - 1,
            .cost = 1.0f + avgRough * 3.0f,
            .blocked = false
        });
    }

    std::vector<RegionStep> findPath(int startRX, int startRY, int goalRX, int goalRY) {
        int startIdx = startRY * regionsX + startRX;
        int goalIdx = goalRY * regionsX + goalRX;

        if (startIdx == goalIdx)
            return { {startRX, startRY, -1} };

        std::vector<float> gScore(regionsX * regionsY, INF);
        std::vector<int> cameFrom(regionsX * regionsY, -1);
        std::vector<int> cameFromGate(regionsX * regionsY, -1);
        std::vector<bool> closed(regionsX * regionsY, false);

        struct AStarNode { float f; int idx; };
        auto cmp = [](const AStarNode& a, const AStarNode& b) { return a.f > b.f; };
        std::priority_queue<AStarNode, std::vector<AStarNode>, decltype(cmp)> open(cmp);

        gScore[startIdx] = 0.0f;
        open.push({ static_cast<float>(std::abs(startRX - goalRX) + std::abs(startRY - goalRY)), startIdx });

        while (!open.empty()) {
            auto [f, curIdx] = open.top();
            open.pop();

            if (curIdx == goalIdx) break;
            if (closed[curIdx]) continue;
            closed[curIdx] = true;

            int curRX = curIdx % regionsX;
            int curRY = curIdx / regionsX;

            for (int gi = 0; gi < static_cast<int>(gates.size()); ++gi) {
                const Gate& gate = gates[gi];
                if (gate.blocked) continue;

                int neighborIdx = -1;
                if (gate.regionAX == curRX && gate.regionAY == curRY)
                    neighborIdx = gate.regionBY * regionsX + gate.regionBX;
                else if (gate.regionBX == curRX && gate.regionBY == curRY)
                    neighborIdx = gate.regionAY * regionsX + gate.regionAX;

                if (neighborIdx < 0 || closed[neighborIdx]) continue;

                // Penalize regions near map edges
                int nRX = neighborIdx % regionsX;
                int nRY = neighborIdx / regionsX;
                int edgeDist = std::min({nRX, nRY, regionsX - 1 - nRX, regionsY - 1 - nRY});
                float edgePenalty = (edgeDist < 2) ? static_cast<float>(2 - edgeDist) * 2.0f : 0.0f;

                float tentativeG = gScore[curIdx] + gate.cost + edgePenalty;
                if (tentativeG < gScore[neighborIdx]) {
                    gScore[neighborIdx] = tentativeG;
                    cameFrom[neighborIdx] = curIdx;
                    cameFromGate[neighborIdx] = gi;

                    float h = static_cast<float>(std::abs(nRX - goalRX) + std::abs(nRY - goalRY));
                    open.push({ tentativeG + h, neighborIdx });
                }
            }
        }

        if (cameFrom[goalIdx] == -1)
            return {};

        std::vector<int> reverseRegions;
        std::vector<int> reverseGates;
        int cur = goalIdx;
        while (cur != startIdx) {
            reverseRegions.push_back(cur);
            reverseGates.push_back(cameFromGate[cur]);
            cur = cameFrom[cur];
        }
        reverseRegions.push_back(startIdx);

        std::vector<RegionStep> path;
        for (int i = static_cast<int>(reverseRegions.size()) - 1; i >= 0; --i) {
            int rx = reverseRegions[i] % regionsX;
            int ry = reverseRegions[i] / regionsX;
            int gateIdx = (i > 0) ? reverseGates[i - 1] : -1;
            path.push_back({rx, ry, gateIdx});
        }

        return path;
    }

    void markGateBlocked(int gateIndex) {
        if (gateIndex >= 0 && gateIndex < static_cast<int>(gates.size()))
            gates[gateIndex].blocked = true;
    }
};

#endif
