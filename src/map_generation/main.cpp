#include "../../include/terrain_generation.hpp"

#include <cstdio>

struct Rgb {
    int r, g, b;
};

static Rgb colorFor(const Cell& c) {
    if (c.is_rock)            return {170, 170, 180};  // rock core: distinct light gray
    if (c.roughness > 0.92f)  return {100, 100, 110};  // debris halo: dark gray

    float e = c.combinedElevation();
    if (e < 0.30f) return { 40,  80, 170};  // deep water
    if (e < 0.45f) return { 70, 140,  70};  // lowland green
    if (e < 0.60f) return {120, 130,  60};  // dry midland
    if (e < 0.75f) return {130, 100,  60};  // highland rock
    return {230, 230, 235};                  // peaks
}

int main() {
    Map map(42, 4, 4, 64, 64);
    map.generate();

    size_t totalW = map.width_ * map.gen.regionWidth_;
    size_t totalH = map.height_ * map.gen.regionHeight_;

    // half-block trick: fg = upper cell, bg = lower cell
    for (size_t wy = 0; wy + 4 <= totalH; wy += 4) {
        for (size_t wx = 0; wx < totalW; wx += 2) {
            Rgb top = colorFor(map.cellAt(wx, wy));
            Rgb bot = colorFor(map.cellAt(wx, wy + 2));
            std::printf("\x1b[38;2;%d;%d;%dm\x1b[48;2;%d;%d;%dm\xe2\x96\x80",
                        top.r, top.g, top.b, bot.r, bot.g, bot.b);
        }
        std::printf("\x1b[0m\n");
    }

    std::printf("map %zux%zu cells, rocks: %zu\n"
                "legend: \x1b[48;2;170;170;180m  \x1b[0m rock core  "
                "\x1b[48;2;100;100;110m  \x1b[0m rock halo  "
                "\x1b[48;2;40;80;170m  \x1b[0m water  "
                "\x1b[48;2;70;140;70m  \x1b[0m lowland  "
                "\x1b[48;2;120;130;60m  \x1b[0m midland  "
                "\x1b[48;2;130;100;60m  \x1b[0m highland  "
                "\x1b[48;2;230;230;235m  \x1b[0m peaks\n",
                totalW, totalH, map.rocks.size());
    return 0;
}
