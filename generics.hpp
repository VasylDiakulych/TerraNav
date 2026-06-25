#ifndef GENERICS_HPP
#define GENERICS_HPP

#include <concepts>
#include <vector>

struct Position {
    int x{0}, y{0};
    float direction{0.0f};
};

struct Cell {
    float absolute_elevation{0.0f};
    float slope_dx{0.0f};
    float slope_dy{0.0f};

    float roughness{0.0f};
    float humidity{0.0f};

    float wind_speed{0.0f};
    // angle 0 -> 2pi
    float wind_dir{0.0f};

    bool is_impassable{false};
    float final_cost{1.0f};

    bool is_visited{false};
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

template<typename T>
concept Navigator = requires(T nav, Position start, Position end  ){
    { nav.calculatePath(start, end) } -> std::convertible_to<std::vector<Position>>;
};

template<typename F>
concept Heuristics = requires(F f, Position start, Position end) {
    { f(start, end) } -> std::convertible_to<float>;
};

template<typename F>
concept CostFunction = requires (const F& f, const Cell& c) {
    { f(c) } -> std::convertible_to<float>;
};

template<typename F>
concept ObserveFunction = requires (const F& f, Position p) {
    { f(p) } -> std::convertible_to<Cell>;
};

#endif
