#ifndef TERRAIN_GENERATION_HPP
#define TERRAIN_GENERATION_HPP

#include "generics.hpp"
#include "../third_party/PerlinNoise.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <iostream>

struct Map {
    size_t height_;
    size_t width_;
    std::vector<Region> tiles;

    Region& operator[](size_t x, size_t y) {
        return tiles[y * width_ + x];
    }

    const Region& operator[](size_t x, size_t y) const {
        return tiles[y * width_ + x];
    }
};

struct BoundedRegionGenerator {
    size_t seed_{42};
    size_t width_;
    size_t height_;
    
    siv::PerlinNoise elevation_noise{seed_ * 17};
    siv::PerlinNoise humidity_noise{seed_ * 113};
    siv::PerlinNoise roughness_noise{seed_ * 257};
    
    int octaves{4};
    double persistence{0.45};
    double frequency{8.0};

    BoundedRegionGenerator(long long seed, size_t width, size_t height) : 
        seed_(seed), width_(width), height_(height) {}

    bool valid_coordinate(long long x, long long y) {
        return x >= 0 && x < static_cast<size_t>(width_) && y >= 0 && y < static_cast<size_t>(height_);
    }

    Region generate(int region_x, int region_y) {
        if(!valid_coordinate(region_x, region_y)) {
            std::cerr << "Tried to access invalid coordinate " << region_x << ' ' << region_y << '\n';
        }

         return Region{};
    }
};

#endif
