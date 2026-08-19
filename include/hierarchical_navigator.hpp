#ifndef HIERARCHICAL_NAVIGATOR_HPP
#define HIERARCHICAL_NAVIGATOR_HPP

#include <vector>
#include <memory>
#include <functional>

#include "generics.hpp"
#include "terrain_generation.hpp"
#include "region_pathfinding.hpp"
#include "pathfinding.hpp"

struct HierarchicalNavigator {
    Map* groundTruth{nullptr};
    RegionGraph* graph{nullptr};

    std::vector<RegionStep> macroPath;
    int currentStepIndex{0};

    Position droneLocal{0, 0};
    int currentRegionX{0}, currentRegionY{0};

    std::vector<Position> microPath;
    std::vector<Position> visitedGlobal;

    bool running{false};
    bool finished{false};

    using CostFn = std::function<float(const Cell&, const Cell&, Position, Position)>;
    using ProbeFn = std::function<Cell(Position)>;
    using HeurFn = std::function<float(Position, Position)>;
    using ScanFn = std::function<std::vector<Position>(Position, const Region&)>;

    using Nav = DStarLiteNavigator<CostFn, ProbeFn, HeurFn, ScanFn>;

    std::unique_ptr<Nav> nav;
    Region navState;
    float sensorRange{7.0f};

    int goalRegionX{0}, goalRegionY{0};
    Position finalGoal{0, 0};

    void init(Map* map, RegionGraph* g, int startRX, int startRY,
              int goalRX, int goalRY, float sensorRng) {
        groundTruth = map;
        graph = g;
        sensorRange = sensorRng;
        goalRegionX = goalRX;
        goalRegionY = goalRY;
        finished = false;
        running = false;

        macroPath = g->findPath(startRX, startRY, goalRX, goalRY);
        currentStepIndex = 0;

        currentRegionX = startRX;
        currentRegionY = startRY;

        int localX = static_cast<int>(map->gen.regionWidth_) / 2;
        int localZ = static_cast<int>(map->gen.regionHeight_) / 2;
        droneLocal = {localX, localZ};

        finalGoal = {static_cast<int>(map->gen.regionWidth_) / 2,
                     static_cast<int>(map->gen.regionHeight_) / 2};

        visitedGlobal.clear();
        nav.reset();
        navState = Region{};
        microPath.clear();

        startRegionNav();
    }

    void startRegionNav() {
        if (currentStepIndex >= static_cast<int>(macroPath.size())) {
            finished = true;
            return;
        }

        const RegionStep& step = macroPath[currentStepIndex];
        currentRegionX = step.regionX;
        currentRegionY = step.regionY;

        int w = static_cast<int>(groundTruth->gen.regionWidth_);
        int h = static_cast<int>(groundTruth->gen.regionHeight_);

        int originX = currentRegionX * w;
        int originY = currentRegionY * h;

        auto probe = [this, originX, originY](Position p) -> Cell {
            return groundTruth->cellAt(
                static_cast<size_t>(originX + p.x),
                static_cast<size_t>(originY + p.y)
            );
        };

        auto scan = [this, originX, originY](Position robot, const Region& state) -> std::vector<Position> {
            std::vector<Position> result;
            int r = static_cast<int>(std::ceil(sensorRange));
            float r2 = sensorRange * sensorRange;

            auto trueElev = [this, originX, originY](int lx, int ly) -> float {
                const Cell& c = groundTruth->cellAt(
                    static_cast<size_t>(originX + lx),
                    static_cast<size_t>(originY + ly)
                );
                return c.combinedElevation() + (c.is_rock ? 0.15f : 0.0f);
            };

            for (int dy = -r; dy <= r; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    int tx = robot.x + dx, ty = robot.y + dy;
                    if (tx < 0 || ty < 0 || tx >= state.width || ty >= state.height) continue;
                    if (static_cast<float>(dx * dx + dy * dy) > r2) continue;
                    if (state[tx, ty].is_visited) continue;
                    Position target{.x = tx, .y = ty, .direction = 0.0f};
                    if (!lineOfSight(state.width, state.height, robot, target, trueElev, 0.05f)) continue;
                    result.push_back(target);
                }
            }
            return result;
        };

        Position goal;
        if (currentStepIndex < static_cast<int>(macroPath.size()) - 1) {
            int gateIdx = step.gateIndex;
            if (gateIdx >= 0 && gateIdx < static_cast<int>(graph->gates.size())) {
                const Gate& gate = graph->gates[gateIdx];
                int side = (gate.regionAX == currentRegionX && gate.regionAY == currentRegionY) ? 0 : 1;

                // Look ahead: if there's a next gate, bias goal toward it
                if (currentStepIndex + 1 < static_cast<int>(macroPath.size())) {
                    const RegionStep& nextStep = macroPath[currentStepIndex + 1];
                    if (nextStep.gateIndex >= 0 && nextStep.gateIndex < static_cast<int>(graph->gates.size())) {
                        const Gate& nextGate = graph->gates[nextStep.gateIndex];
                        goal = gate.getGoalCellLookAhead(
                            droneLocal.x, droneLocal.y, w, h, side,
                            nextGate.startCoord, nextGate.endCoord,
                            nextGate.direction == GateDirection::Horizontal
                        );
                    } else {
                        goal = gate.getGoalCell(droneLocal.x, droneLocal.y, w, h, side);
                    }
                } else {
                    goal = gate.getGoalCell(droneLocal.x, droneLocal.y, w, h, side);
                }
            } else {
                goal = {w / 2, h / 2};
            }
        } else {
            goal = finalGoal;
        }

        // Determine which edge the goal is on (skip penalty on exit edge)
        int goalEdge = -1;
        if (goal.x == 0) goalEdge = 0;
        else if (goal.x == w - 1) goalEdge = 1;
        else if (goal.y == 0) goalEdge = 2;
        else if (goal.y == h - 1) goalEdge = 3;

        auto costWithEdgePenalty = [w, h, goalEdge](const Cell& from, const Cell& to,
                                                    Position pa, Position pb) -> float {
            float base = actualCost(from, to, pa, pb);
            if (base >= INF) return INF;

            int penalty = 0;
            if (goalEdge != 0 && pb.x < 3) penalty = std::max(penalty, 3 - pb.x);
            if (goalEdge != 1 && (w - 1 - pb.x) < 3) penalty = std::max(penalty, 3 - (w - 1 - pb.x));
            if (goalEdge != 2 && pb.y < 3) penalty = std::max(penalty, 3 - pb.y);
            if (goalEdge != 3 && (h - 1 - pb.y) < 3) penalty = std::max(penalty, 3 - (h - 1 - pb.y));

            base += static_cast<float>(penalty) * 0.5f;
            return base;
        };

        CostFn costFn = costWithEdgePenalty;
        HeurFn heurFn = euclideanHeuristic;

        nav = std::make_unique<Nav>(w, h, costFn, probe, scan, heurFn);
        navState.resize(w, h);

        microPath = nav->replan(droneLocal, goal);
    }

    void tick() {
        if (finished) return;
        if (microPath.size() < 2) {
            if (currentStepIndex < static_cast<int>(macroPath.size()) - 1) {
                RegionStep& step = macroPath[currentStepIndex];
                if (step.gateIndex >= 0) {
                    graph->markGateBlocked(step.gateIndex);
                }
                // Replan macro path but keep current region — find the step
                // in the new path that matches our current region
                auto newPath = graph->findPath(
                    currentRegionX, currentRegionY,
                    goalRegionX, goalRegionY
                );
                if (newPath.empty()) {
                    finished = true;
                    return;
                }
                macroPath = newPath;
                currentStepIndex = 0;

                // Goal changed — D* Lite can't update goals, so create new navigator
                // but preserve discovered cells from the old one
                Region discovered = nav->currentState_;
                int w = static_cast<int>(groundTruth->gen.regionWidth_);
                int h = static_cast<int>(groundTruth->gen.regionHeight_);

                const RegionStep& newStep = macroPath[currentStepIndex];
                Position goal;
                if (newStep.gateIndex >= 0 && newStep.gateIndex < static_cast<int>(graph->gates.size())) {
                    const Gate& gate = graph->gates[newStep.gateIndex];
                    int side = (gate.regionAX == currentRegionX && gate.regionAY == currentRegionY) ? 0 : 1;
                    goal = gate.getGoalCell(droneLocal.x, droneLocal.y, w, h, side);
                } else {
                    goal = finalGoal;
                }

                startRegionNav();
                // Restore discovered cells
                for (int y = 0; y < h; ++y)
                    for (int x = 0; x < w; ++x)
                        if (discovered[x, y].is_visited)
                            nav->currentState_[x, y] = discovered[x, y];
                microPath = nav->replan(droneLocal, goal);
            } else {
                finished = true;
            }
            return;
        }

        droneLocal = microPath[1];

        int w = static_cast<int>(groundTruth->gen.regionWidth_);
        int h = static_cast<int>(groundTruth->gen.regionHeight_);
        int originX = currentRegionX * w;
        int originY = currentRegionY * h;

        visitedGlobal.push_back({
            originX + droneLocal.x,
            originY + droneLocal.y
        });

        if (currentStepIndex < static_cast<int>(macroPath.size()) - 1) {
            const RegionStep& step = macroPath[currentStepIndex];
            if (step.gateIndex >= 0 && step.gateIndex < static_cast<int>(graph->gates.size())) {
                const Gate& gate = graph->gates[step.gateIndex];
                bool atGate = false;
                if (gate.direction == GateDirection::Horizontal) {
                    int targetX = (gate.regionAX == currentRegionX) ? w - 1 : 0;
                    atGate = (droneLocal.x == targetX &&
                              droneLocal.y >= gate.startCoord &&
                              droneLocal.y <= gate.endCoord);
                } else {
                    int targetY = (gate.regionAY == currentRegionY) ? h - 1 : 0;
                    atGate = (droneLocal.y == targetY &&
                              droneLocal.x >= gate.startCoord &&
                              droneLocal.x <= gate.endCoord);
                }

                if (atGate) {
                    bool inA = (gate.regionAX == currentRegionX && gate.regionAY == currentRegionY);
                    if (gate.direction == GateDirection::Horizontal) {
                        // Regions side by side: A=left, B=right
                        droneLocal.x = inA ? 0 : w - 1;
                    } else {
                        // Regions stacked: A=top, B=bottom
                        droneLocal.y = inA ? 0 : h - 1;
                    }
                    currentStepIndex++;
                    startRegionNav();
                    return;
                }
            }
        } else {
            if (droneLocal.x == finalGoal.x && droneLocal.y == finalGoal.y) {
                finished = true;
                return;
            }
        }

        Position goal = microPath.back();
        microPath = nav->replan(droneLocal, goal);
    }

    // Toggle a wall on the current region's cell. D* Lite will replan around it.
    void toggleWall(int localX, int localY) {
        if (!nav || localX < 0 || localY < 0) return;
        int w = nav->width_;
        int h = nav->height_;
        if (localX >= w || localY >= h) return;

        Cell& c = nav->currentState_[localX, localY];
        if (!c.is_visited) return;

        c.is_impassable = !c.is_impassable;

        // Force D* to replan by updating affected vertices
        nav->updateVertex({localX, localY});
        for (const auto& n : nav->getNeighbors({localX, localY}))
            nav->updateVertex(n);

        Position goal = microPath.back();
        microPath = nav->replan(droneLocal, goal);
    }

    Region getExploredRegion() const {
        int w = static_cast<int>(groundTruth->gen.regionWidth_);
        int h = static_cast<int>(groundTruth->gen.regionHeight_);
        int originX = currentRegionX * w;
        int originY = currentRegionY * h;

        Region r;
        if (nav)
            r = nav->currentState_;
        else
            r.resize(w, h);

        // Fill in true elevation for unvisited cells so the mesh has correct geometry
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (!r[x, y].is_visited) {
                    const Cell& truth = groundTruth->cellAt(
                        static_cast<size_t>(originX + x),
                        static_cast<size_t>(originY + y)
                    );
                    r[x, y].absolute_elevation = truth.absolute_elevation;
                    r[x, y].craterDelta = truth.craterDelta;
                    r[x, y].is_rock = truth.is_rock;
                    r[x, y].roughness = truth.roughness;
                }
            }
        }
        return r;
    }

    int getDroneGlobalX() const {
        int w = static_cast<int>(groundTruth->gen.regionWidth_);
        return currentRegionX * w + droneLocal.x;
    }

    int getDroneGlobalZ() const {
        int h = static_cast<int>(groundTruth->gen.regionHeight_);
        return currentRegionY * h + droneLocal.y;
    }

    int getCurrentRegionIndex() const {
        return currentRegionY * graph->regionsX + currentRegionX;
    }
};

#endif
