#ifndef TERRAIN_GENERATION_HPP
#define TERRAIN_GENERATION_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <random>
#include <vector>

#include "generics.hpp"
#include "../third_party/PerlinNoise.hpp"

// Elevation shaping exponent: >1 flattens valleys while massifs keep height
inline constexpr double kElevationShape{1.3};

struct NoiseParams {
    int octaves{6};
    double persistence{0.35};
    double frequency{2.0};
};

struct CraterParams {
    size_t maxCountPerRegion{1};   // per region-sized unit of area
    double minRadius{4.0};
    double maxRadius{14.0};
    double depthFactor{0.01};
    double rimRatio{0.2};
    double rimWidth{0.5};
};

struct RockParams {
    size_t maxClustersPerRegion{2};
    size_t maxRocksPerCluster{4};
    double clusterSpread{6.0};
    double minCoreRadius{0.5};
    double maxCoreRadius{2.5};
    double haloFactor{2.0};
    double roughness{0.8};
};

struct Rock {
    double x{0};
    double y{0};
    double coreRadius{0};
    double haloRadius{0};
};

struct RegionGenerator {
    size_t seed_{42};
    size_t regionsCountX_;
    size_t regionsCountY_;
    size_t regionWidth_;
    size_t regionHeight_;

    siv::PerlinNoise elevationNoise;
    siv::PerlinNoise humidityNoise;
    siv::PerlinNoise roughnessNoise;

    NoiseParams noise;

    RegionGenerator(long long seed, size_t regionCountX, size_t regionCountY,
                    size_t regionWidth, size_t regionHeight, NoiseParams noiseParams = {})
        : seed_(static_cast<size_t>(seed)),
          regionsCountX_(regionCountX), regionsCountY_(regionCountY),
          regionWidth_(regionWidth), regionHeight_(regionHeight),
          elevationNoise(seed * 17),
          humidityNoise(seed * 113),
          roughnessNoise(seed * 257),
          noise(noiseParams) {}

    double elevation(double nx, double ny) const {
        double sum = 0.0;
        double amplitude = 1.0;
        double norm = 0.0;
        double freq = 1.0;
        for (int o = 0; o < noise.octaves; ++o) {
            double s = elevationNoise.noise2D_01(nx * freq, ny * freq);
            double ridge = 1.0 - std::abs(2.0 * s - 1.0);
            sum += ridge * ridge * amplitude;
            norm += amplitude;
            amplitude *= noise.persistence;
            freq *= 2.0;
        }
        return sum / norm;
    }

    bool validCoordinate(long long x, long long y) const {
        return x >= 0 && static_cast<size_t>(x) < regionsCountX_
            && y >= 0 && static_cast<size_t>(y) < regionsCountY_;
    }

    Region generateRegion(int regionX, int regionY) {
        if (!validCoordinate(regionX, regionY)) {
            std::cerr << "Tried to access invalid coordinate "<< regionX << ' ' << regionY << '\n';
            return Region{};
        }

        Region reg;
        reg.resize(static_cast<int>(regionWidth_), static_cast<int>(regionHeight_));

        size_t totalW = regionsCountX_ * regionWidth_;
        size_t totalH = regionsCountY_ * regionHeight_;
        double fx = noise.frequency / static_cast<double>(totalW);
        double fy = noise.frequency / static_cast<double>(totalH);

        size_t originX = static_cast<size_t>(regionX) * regionWidth_;
        size_t originY = static_cast<size_t>(regionY) * regionHeight_;
 
        for (size_t cy = 0; cy < regionHeight_; ++cy) {
            for (size_t cx = 0; cx < regionWidth_; ++cx) {
                size_t gx = originX + cx;
                size_t gy = originY + cy;
                double nx = gx * fx;
                double ny = gy * fy;

                Cell& c = reg[static_cast<int>(cx), static_cast<int>(cy)];

                double e = elevation(nx, ny);
                c.absolute_elevation = static_cast<float>(std::pow(e, kElevationShape));
                c.humidity = 0.05f;
                c.roughness = std::clamp(
                    static_cast<float>(roughnessNoise.normalizedOctave2D_01(nx + 8.2, ny + 2.5, noise.octaves, noise.persistence)),
                    0.4f, 0.9f);
            }
        }

        computeSlopes(reg);

        return reg;
    }

    void computeSlopes(Region& reg) const {
        int w = reg.width, h = reg.height;
        for (int cy = 0; cy < h; ++cy) {
            for (int cx = 0; cx < w; ++cx) {
                int xm = std::max(cx - 1, 0), xp = std::min(cx + 1, w - 1);
                int ym = std::max(cy - 1, 0), yp = std::min(cy + 1, h - 1);

                Cell& c = reg[cx, cy];
                if (xp > xm)
                    c.slope_dx = (reg[xp, cy].absolute_elevation - reg[xm, cy].absolute_elevation)
                                 / static_cast<float>(xp - xm);
                if (yp > ym)
                    c.slope_dy = (reg[cx, yp].absolute_elevation - reg[cx, ym].absolute_elevation)
                                 / static_cast<float>(yp - ym);
            }
        }
    }

};

struct Map {
    size_t height_;
    size_t width_;
    std::vector<Region> tiles;
    std::vector<Rock> rocks;
    RegionGenerator gen;
    CraterParams craterParams;
    RockParams rockParams;

    Map(long long seed, size_t regionsX, size_t regionsY, size_t regionW, size_t regionH,
        NoiseParams noise = {}, CraterParams craters = {}, RockParams rocksCfg = {})
        : height_(regionsY), width_(regionsX),
          tiles(regionsX * regionsY),
          gen(seed, regionsX, regionsY, regionW, regionH, noise),
          craterParams(craters), rockParams(rocksCfg) {}

    void generate() {
        for (size_t ry = 0; ry < height_; ++ry) {
            for (size_t rx = 0; rx < width_; ++rx) {
                tiles[ry * width_ + rx] = gen.generateRegion(
                    static_cast<int>(rx), static_cast<int>(ry));
            }
        }

        addCraters();
        addRocks();
        recomputeSlopes();
    }

    Cell& cellAt(size_t wx, size_t wy) {
        size_t rx = wx / gen.regionWidth_;
        size_t ry = wy / gen.regionHeight_;
        Region& reg = tiles[ry * width_ + rx];
        return reg[static_cast<int>(wx % gen.regionWidth_),
                   static_cast<int>(wy % gen.regionHeight_)];
    }

    void addCraters() {
        size_t totalW = width_ * gen.regionWidth_;
        size_t totalH = height_ * gen.regionHeight_;

        std::mt19937 rng(gen.seed_);
        std::uniform_int_distribution<size_t> countDist(0, craterParams.maxCountPerRegion * width_ * height_);
        std::uniform_real_distribution<double> xDist(0.0, static_cast<double>(totalW) - 1.0);
        std::uniform_real_distribution<double> yDist(0.0, static_cast<double>(totalH) - 1.0);
        std::uniform_real_distribution<double> radiusDist(craterParams.minRadius, craterParams.maxRadius);

        size_t craterCount = countDist(rng);
        for (size_t i = 0; i < craterCount; ++i) {
            double crx = xDist(rng);
            double cry = yDist(rng);
            double radius = radiusDist(rng);
            double depth = radius * craterParams.depthFactor;
            double rimHeight = depth * craterParams.rimRatio;

            // Rim bump extends ~3 rim-widths past t = 1 (t = dist / radius)
            double extent = radius * (1.0 + 3.0 * craterParams.rimWidth);
            size_t x0 = static_cast<size_t>(std::clamp(std::floor(crx - extent), 0.0, static_cast<double>(totalW - 1)));
            size_t x1 = static_cast<size_t>(std::clamp(std::ceil(crx + extent), 0.0, static_cast<double>(totalW - 1)));
            size_t y0 = static_cast<size_t>(std::clamp(std::floor(cry - extent), 0.0, static_cast<double>(totalH - 1)));
            size_t y1 = static_cast<size_t>(std::clamp(std::ceil(cry + extent), 0.0, static_cast<double>(totalH - 1)));

            for (size_t wy = y0; wy <= y1; ++wy) {
                for (size_t wx = x0; wx <= x1; ++wx) {
                    double t = std::hypot(static_cast<double>(wx) - crx,
                                          static_cast<double>(wy) - cry) / radius;
                    if (t > 1.0 + 3.0 * craterParams.rimWidth) continue;

                    // Parabolic bowl inside, Gaussian bump at t = 1 for the rim
                    double delta = 0.0;
                    if (t < 1.0) delta -= depth * (1.0 - t * t);
                    double rimT = (t - 1.0) / craterParams.rimWidth;
                    delta += rimHeight * std::exp(-rimT * rimT);

                    Cell& c = cellAt(wx, wy);
                    c.absolute_elevation = std::clamp(
                        c.absolute_elevation + static_cast<float>(delta), 0.0f, 1.0f);
                }
            }
        }
    }

    void addRocks() {
        rocks.clear();

        size_t totalW = width_ * gen.regionWidth_;
        size_t totalH = height_ * gen.regionHeight_;

        std::mt19937 rng(gen.seed_ ^ 0x5DEECE66DULL);
        std::uniform_int_distribution<size_t> clusterCountDist(0, rockParams.maxClustersPerRegion * width_ * height_);
        std::uniform_int_distribution<size_t> clusterSizeDist(1, rockParams.maxRocksPerCluster);
        std::uniform_real_distribution<double> xDist(0.0, static_cast<double>(totalW) - 1.0);
        std::uniform_real_distribution<double> yDist(0.0, static_cast<double>(totalH) - 1.0);
        std::uniform_real_distribution<double> spreadDist(0.0, rockParams.clusterSpread);
        std::uniform_real_distribution<double> angleDist(0.0, 2.0 * std::numbers::pi);
        std::uniform_real_distribution<double> coreDist(rockParams.minCoreRadius, rockParams.maxCoreRadius);

        size_t clusterCount = clusterCountDist(rng);
        for (size_t i = 0; i < clusterCount; ++i) {
            double cx = xDist(rng);
            double cy = yDist(rng);

            size_t clusterSize = clusterSizeDist(rng);
            for (size_t j = 0; j < clusterSize; ++j) {
                double angle = angleDist(rng);
                double dist = spreadDist(rng);
                double rockX = std::clamp(cx + std::cos(angle) * dist, 0.0, static_cast<double>(totalW) - 1.0);
                double rockY = std::clamp(cy + std::sin(angle) * dist, 0.0, static_cast<double>(totalH) - 1.0);
                double core = coreDist(rng);
                double halo = core * rockParams.haloFactor;

                rocks.push_back(Rock{.x = rockX, .y = rockY,
                                     .coreRadius = core, .haloRadius = halo});

                size_t x0 = static_cast<size_t>(std::clamp(std::floor(rockX - halo), 0.0, static_cast<double>(totalW) - 1.0));
                size_t x1 = static_cast<size_t>(std::clamp(std::ceil(rockX + halo), 0.0, static_cast<double>(totalW) - 1.0));
                size_t y0 = static_cast<size_t>(std::clamp(std::floor(rockY - halo), 0.0, static_cast<double>(totalH) - 1.0));
                size_t y1 = static_cast<size_t>(std::clamp(std::ceil(rockY + halo), 0.0, static_cast<double>(totalH) - 1.0));

                for (size_t wy = y0; wy <= y1; ++wy) {
                    for (size_t wx = x0; wx <= x1; ++wx) {
                        double d = std::hypot(static_cast<double>(wx) - rockX,
                                              static_cast<double>(wy) - rockY);
                        if (d > halo) continue;

                        Cell& c = cellAt(wx, wy);
                        if (d < core) {
                            c.is_impassable = true;
                            c.is_rock = true;
                        } else {
                            c.roughness = std::clamp(
                                c.roughness + static_cast<float>(rockParams.roughness * (1.0 - d / halo)),
                                0.0f, 1.0f);
                        }
                    }
                }
            }
        }
    }

    void recomputeSlopes() {
        size_t totalW = width_ * gen.regionWidth_;
        size_t totalH = height_ * gen.regionHeight_;

        for (size_t wy = 0; wy < totalH; ++wy) {
            for (size_t wx = 0; wx < totalW; ++wx) {
                size_t xm = wx > 0 ? wx - 1 : 0;
                size_t xp = std::min(wx + 1, totalW - 1);
                size_t ym = wy > 0 ? wy - 1 : 0;
                size_t yp = std::min(wy + 1, totalH - 1);

                Cell& c = cellAt(wx, wy);
                if (xp > xm)
                    c.slope_dx = (cellAt(xp, wy).absolute_elevation - cellAt(xm, wy).absolute_elevation)
                                 / static_cast<float>(xp - xm);
                if (yp > ym)
                    c.slope_dy = (cellAt(wx, yp).absolute_elevation - cellAt(wx, ym).absolute_elevation)
                                 / static_cast<float>(yp - ym);
            }
        }
    }

    Region& operator[](size_t x, size_t y) {
        return tiles[y * width_ + x];
    }

    const Region& operator[](size_t x, size_t y) const {
        return tiles[y * width_ + x];
    }
};

#endif
