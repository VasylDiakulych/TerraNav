#include "../../include/pathfinding.hpp"
#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <cstdio>

struct ProfileScope {
    std::string name;
    std::chrono::high_resolution_clock::time_point start;
    ProfileScope(std::string scope_name)
        : name(std::move(scope_name)), start(std::chrono::high_resolution_clock::now()) {}
    ~ProfileScope() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::cout << "[PROFILE] " << name << " took " << duration << " us\n";
    }
};

bool loadMovingAIMap(Region& truth, int& W, int& H, const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filename << "\n";
        return false;
    }
    std::string line;
    bool in_map = false;
    int current_y = 0;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        if (line.find("height") == 0) H = std::stoi(line.substr(7));
        else if (line.find("width") == 0) W = std::stoi(line.substr(6));
        else if (line == "map") { in_map = true; truth.resize(W, H); }
        else if (in_map) {
            if (current_y >= H) break;
            if ((int)line.size() < W) continue;
            for (int x = 0; x < W; ++x) {
                char tile = line[x];
                bool impassable = (tile == '@' || tile == 'O' || tile == 'W' || tile == 'T');
                truth[x, current_y].is_impassable = impassable;
                truth[x, current_y].absolute_elevation = impassable ? 1.0f : 0.0f;
            }
            current_y++;
        }
    }
    return in_map && current_y == H;
}

struct Scenario { Position start, goal; float optimalCost; };

std::vector<Scenario> loadScenarios(const std::string& filename, int W, int H) {
    std::vector<Scenario> result;
    std::ifstream file(filename);
    if (!file) return result;
    std::string line;
    std::getline(file, line); // skip version
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        int bucket, w, h, sx, sy, gx, gy;
        float opt;
        if (sscanf(line.c_str(), "%d\t%*s\t%d\t%d\t%d\t%d\t%d\t%d\t%f",
                   &bucket, &w, &h, &sx, &sy, &gx, &gy, &opt) == 8) {
            if (sx >= 0 && sx < W && sy >= 0 && sy < H && gx >= 0 && gx < W && gy >= 0 && gy < H)
                result.push_back({{sx, sy}, {gx, gy}, opt});
        }
    }
    return result;
}

template<typename NavigatorType>
auto runScenario(NavigatorType&& nav, Region& truth, int W, int H, Position start, Position goal) {
    int steps = 0;
    float pathCost = 0.0f;
    Position prev = start;
    long long totalReplanUs = 0;

    auto path = nav.replan(start, goal);
    Position robot = start;
    for (int step = 1; step <= 2000; ++step) {
        if (path.size() < 2) break;
        robot = path[1];
        pathCost += actualCost(truth[prev.x, prev.y], truth[robot.x, robot.y], prev, robot);
        auto t0 = std::chrono::high_resolution_clock::now();
        path = nav.replan(robot, goal);
        totalReplanUs += std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - t0).count();
        steps++;
        prev = robot;
        if (robot.x == goal.x && robot.y == goal.y) break;
    }
    bool reached = !path.empty() && path.back().x == goal.x && path.back().y == goal.y;
    return std::make_tuple(reached, steps, pathCost, totalReplanUs);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <mapfile.map>\n"
                  << "  Reads <mapfile.scen> from scen-dragon-age/ for scenarios.\n";
        return 1;
    }

    std::string mapFile = argv[1];
    int W = 0, H = 0;
    Region truth;
    if (!loadMovingAIMap(truth, W, H, mapFile)) {
        std::cerr << "Failed to load map.\n";
        return 1;
    }
    std::cout << "Map: " << mapFile << " [" << W << "x" << H << "]\n";

    std::string scenFile = mapFile;
    size_t pos = scenFile.find("map-dragon-age/");
    if (pos != std::string::npos)
        scenFile.replace(pos, 15, "scen-dragon-age/");
    scenFile += ".scen";

    auto scenarios = loadScenarios(scenFile, W, H);
    if (scenarios.empty()) {
        std::cerr << "No scenarios. Running default.\n";
        scenarios.push_back({{1, 1}, {W-2, H-2}, 0});
    }
    std::cout << "Scenarios: " << scenarios.size() << "\n";

    long long astarTotalUs = 0, dstarTotalUs = 0, mpaaTotalUs = 0;
    float astarTotalCost = 0, dstarTotalCost = 0, mpaaTotalCost = 0;
    int astarWins = 0, dstarWins = 0, mpaaWins = 0, ties = 0;

    auto fmtUs = [](long long us) -> std::string {
        float s = static_cast<float>(us) / 1'000'000.0f;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%lld us (%.6f s)", us, s);
        return std::string(buf);
    };

    for (size_t i = 0; i < scenarios.size(); ++i) {
        auto& sc = scenarios[i];

        NaiveAStarNavigator astar(W, H, actualCost,
            [&truth](Position p) -> Cell { return truth[p.x, p.y]; },
            defaultScan(1.5f), euclideanHeuristic);

        DStarLiteNavigator dstar(W, H, actualCost,
            [&truth](Position p) -> Cell { return truth[p.x, p.y]; },
            defaultScan(1.5f), euclideanHeuristic);

        MPAAStarNavigator mpaa(W, H, actualCost,
            [&truth](Position p) -> Cell { return truth[p.x, p.y]; },
            defaultScan(1.5f), euclideanHeuristic);

        auto [aOk, aSteps, aCost, aUs] = runScenario(astar, truth, W, H, sc.start, sc.goal);
        auto [dOk, dSteps, dCost, dUs] = runScenario(dstar, truth, W, H, sc.start, sc.goal);
        auto [mOk, mSteps, mCost, mUs] = runScenario(mpaa, truth, W, H, sc.start, sc.goal);

        astarTotalUs += aUs;
        dstarTotalUs += dUs;
        mpaaTotalUs += mUs;
        astarTotalCost += aCost;
        dstarTotalCost += dCost;
        mpaaTotalCost += mCost;

        long long best = std::min({dUs, mUs});
        int bestCount = (dUs == best) + (mUs == best);
        if (bestCount > 1) {
            ties++;
        } else if (mUs == best) {
            mpaaWins++;
        } else if (aUs == best) {
            astarWins++;
        } else {
            dstarWins++;
        }

        std::cout << "[" << (i+1) << "/" << scenarios.size() << "] "
                  << "A* " << (aOk ? "OK" : "FAIL") << " c=" << aCost << " t=" << aUs << "us"
                  << " | D* " << (dOk ? "OK" : "FAIL") << " c=" << dCost << " t=" << dUs << "us"
                  << " | MPAA* " << (mOk ? "OK" : "FAIL") << " c=" << mCost << " t=" << mUs << "us"
                  << " | opt=" << sc.optimalCost << "\n";
    }

    std::cout << "\n=== Summary ===\n"
              << "A* total: " << fmtUs(astarTotalUs) << "  cost=" << astarTotalCost << "\n"
              << "D* total: " << fmtUs(dstarTotalUs) << "  cost=" << dstarTotalCost << "\n"
              << "MPAA* total: " << fmtUs(mpaaTotalUs) << "  cost=" << mpaaTotalCost << "\n"
              << "  A* wins: " << astarWins
              << "  D* wins: " << dstarWins
              << "  MPAA* wins: " << mpaaWins << "  ties: " << ties << "\n";
}
