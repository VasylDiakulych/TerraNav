#ifndef CHUNK_RENDERER_HPP
#define CHUNK_RENDERER_HPP

#include <vector>
#include <cmath>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include "generics.hpp"
#include "terrain_generation.hpp"

constexpr float CELL_SIZE = 5.0f;
constexpr int CHUNK_SIZE = 64;

inline Color lerpColor(Color a, Color b, float t) {
    if (t < 0.0f) t = 0.0f; 
    else if (t > 1.0f) t = 1.0f;

    // unsigned char is 0-255, arithmetic produces float, cast back after rounding
    return {
        static_cast<unsigned char>(std::round(a.r + (b.r - a.r) * t)),
        static_cast<unsigned char>(std::round(a.g + (b.g - a.g) * t)),
        static_cast<unsigned char>(std::round(a.b + (b.b - a.b) * t)),
        255
    };
}

inline Color elevationColor(const Cell& c) {
    if (c.is_rock)            return {  90,  70,  60, 255 };
    if (c.roughness > 0.92f)  return { 110,  85,  65, 255 };

    float e = c.combinedElevation();
    if (e < 0.0f) e = 0.0f;
    else if (e > 1.0f) e = 1.0f;

    struct Stop { float pos; Color color; };
    constexpr Stop stops[] = {
        { 0.00f, {  80,  50,  35 } },
        { 0.25f, { 130,  70,  45 } },
        { 0.40f, { 170,  95,  55 } },
        { 0.55f, { 200, 140,  95 } },
        { 0.70f, { 220, 180, 140 } },
        { 0.85f, { 245, 230, 210 } },
        { 1.00f, { 245, 230, 210 } },
    };

    for (size_t i = 1; i < std::size(stops); ++i) {
        if (e <= stops[i].pos) {
            // Normalize e from [prev.pos, cur.pos] to [0, 1] 
            float t = (e - stops[i - 1].pos) / (stops[i].pos - stops[i - 1].pos);
            return lerpColor(stops[i - 1].color, stops[i].color, t);
        }
    }
    return stops[std::size(stops) - 1].color;
}

struct TerrainRenderer {
    float heightScale{ 375.0f };
    float craterScale{ 375.0f };

    std::vector<Model> chunkModels_;
    std::vector<bool> dirty_;
    size_t chunkCountX_ = 0;
    size_t chunkCountZ_ = 0;
    
    float offsetX_ = 0.0f;
    float offsetZ_ = 0.0f;

    void allocateMesh_(Mesh& mesh, int vertexCount, int triCount) {
        mesh.vertexCount = vertexCount;
        mesh.triangleCount = triCount;
        mesh.vertices  = static_cast<float*>(RL_MALLOC(vertexCount * 3 * sizeof(float)));
        mesh.texcoords = static_cast<float*>(RL_MALLOC(vertexCount * 2 * sizeof(float)));
        mesh.normals   = static_cast<float*>(RL_MALLOC(vertexCount * 3 * sizeof(float)));
        mesh.colors    = static_cast<unsigned char*>(RL_MALLOC(vertexCount * 4 * sizeof(unsigned char)));
        mesh.indices   = static_cast<unsigned short*>(RL_MALLOC(triCount * 3 * sizeof(unsigned short)));
    }

    void fillVertexData_(Mesh& mesh, const Map& map, int beginX, int beginZ,
                         int chunkCols, int chunkRows, int mapCols, int mapRows) {
        for (int localZ = 0; localZ < chunkRows; ++localZ) {
            int globalZ = beginZ + localZ;
            for (int localX = 0; localX < chunkCols; ++localX) {
                int globalX = beginX + localX;
                int idx = localZ * chunkCols + localX;
                const Cell& c = map.cellAt(globalX, globalZ);
    
                float h = heightAt(map, globalX, globalZ);
    
                // position
                mesh.vertices[idx * 3 + 0] = offsetX_ + static_cast<float>(globalX) * CELL_SIZE;
                mesh.vertices[idx * 3 + 1] = h;
                mesh.vertices[idx * 3 + 2] = offsetZ_ + static_cast<float>(globalZ) * CELL_SIZE;
    
                // texcoords (normalized 0-1, can be used later for textures)
                mesh.texcoords[idx * 2 + 0] = static_cast<float>(globalX) / mapCols;
                mesh.texcoords[idx * 2 + 1] = static_cast<float>(globalZ) / mapRows;
    
                // Normal via finite differences: N = (-dh/dx, 1, -dh/dz), then normalized
                int rightX = std::min(globalX + 1, mapCols - 1);
                int downZ = std::min(globalZ + 1, mapRows - 1);
                float dx = heightAt(map, rightX, globalZ) - h;
                float dz = heightAt(map, globalX, downZ) - h;
                float len = std::sqrt(dx * dx + 1.0f + dz * dz);
                float invLen = 1.0f / len;
                mesh.normals[idx * 3 + 0] = -dx * invLen;
                mesh.normals[idx * 3 + 1] = invLen;
                mesh.normals[idx * 3 + 2] = -dz * invLen;
    
                // color
                Color col = elevationColor(c);
                mesh.colors[idx * 4 + 0] = col.r;
                mesh.colors[idx * 4 + 1] = col.g;
                mesh.colors[idx * 4 + 2] = col.b;
                mesh.colors[idx * 4 + 3] = col.a;
            }
        }
    }

    void fillIndices_(Mesh& mesh, int chunkCols, int chunkRows) {
        int t = 0;
        for (int row = 0; row < chunkRows - 1; ++row) {
            for (int col = 0; col < chunkCols - 1; ++col) {
                unsigned short v00 = static_cast<unsigned short>(row * chunkCols + col);
                unsigned short v10 = static_cast<unsigned short>(row * chunkCols + (col + 1));
                unsigned short v01 = static_cast<unsigned short>((row + 1) * chunkCols + col);
                unsigned short v11 = static_cast<unsigned short>((row + 1) * chunkCols + (col + 1));
    
                mesh.indices[t++] = v00;
                mesh.indices[t++] = v01;
                mesh.indices[t++] = v10;
    
                mesh.indices[t++] = v10;
                mesh.indices[t++] = v01;
                mesh.indices[t++] = v11;
            }
        }
    }
    
    void rebuildChunk_(const Map& map, int cx, int cz) {
        int mapCols = static_cast<int>(map.width_ * map.gen.regionWidth_);
        int mapRows = static_cast<int>(map.height_ * map.gen.regionHeight_);

        int beginX = cx * CHUNK_SIZE;
        int beginZ = cz * CHUNK_SIZE;
        if (cx > 0) beginX -= 1;  
        if (cz > 0) beginZ -= 1;
        int endX = std::min(beginX + CHUNK_SIZE + 1, mapCols);
        int endZ = std::min(beginZ + CHUNK_SIZE + 1, mapRows);
    
        int chunkCols = endX - beginX;
        int chunkRows = endZ - beginZ;
        int vertexCount = chunkCols * chunkRows;
        int triCount = (chunkCols - 1) * (chunkRows - 1) * 2;
    
        // Build mesh
        Mesh mesh{};
        allocateMesh_(mesh, vertexCount, triCount);
        fillVertexData_(mesh, map, beginX, beginZ, chunkCols, chunkRows, mapCols, mapRows);
        fillIndices_(mesh, chunkCols, chunkRows);
    
        // Replace old model if exists, upload new one
        int modelIdx = cz * chunkCountX_ + cx;
        if (modelIdx < static_cast<int>(chunkModels_.size()) && chunkModels_[modelIdx].meshes != nullptr)
            UnloadModel(chunkModels_[modelIdx]);
    
        UploadMesh(&mesh, false);
        chunkModels_[modelIdx] = LoadModelFromMesh(mesh);
    }
    
    void rebuildAll(const Map& map) {
        unload();
    
        int cols = static_cast<int>(map.width_ * map.gen.regionWidth_);
        int rows = static_cast<int>(map.height_ * map.gen.regionHeight_);
        chunkCountX_ = (cols + CHUNK_SIZE - 1) / CHUNK_SIZE;
        chunkCountZ_ = (rows + CHUNK_SIZE - 1) / CHUNK_SIZE;
    
        offsetX_ = -cols * 0.5f * CELL_SIZE;
        offsetZ_ = -rows * 0.5f * CELL_SIZE;
    
        chunkModels_.resize(chunkCountX_ * chunkCountZ_);
        dirty_.assign(chunkCountX_ * chunkCountZ_, true);
    
        rebuildDirty(map);
    }

    void rebuildDirty(const Map& map) {
        for (int cz = 0; cz < chunkCountZ_; ++cz) {
            for (int cx = 0; cx < chunkCountX_; ++cx) {
                if (dirty_[cz * chunkCountX_ + cx]) {
                    rebuildChunk_(map, cx, cz);
                    dirty_[cz * chunkCountX_ + cx] = false;
                }
            }
        }
    }

    void markDirty(int cx, int cz) { dirty_[cz * chunkCountX_ + cx] = true; }

    void markAllDirty() { std::fill(dirty_.begin(), dirty_.end(), true); }
    
    void draw() const {
        for (const Model& m : chunkModels_)
            DrawModel(m, { 0, 0, 0 }, 1.0f, WHITE);
    }

    float gridHeight(const Map& map, int x, int z, int mapCols, int mapRows) const {
        if (x >= mapCols) x = mapCols - 1;
        if (z >= mapRows) z = mapRows - 1;
        return heightAt(map, x, z) + 0.5f;
    }

    float globalX(int cell) const { return offsetX_ + static_cast<float>(cell) * CELL_SIZE; }
    float globalZ(int cell) const { return offsetZ_ + static_cast<float>(cell) * CELL_SIZE; }

    void drawGridLines_(const Map& map, int mapCols, int mapRows, int step, 
                       unsigned char r, unsigned char g, unsigned char b, unsigned char a) const {
    
        rlBegin(RL_LINES);
        rlColor4ub(r, g, b, a);
    
        // Vertical lines (constant X, varying Z)
        for (int x = 0; x <= mapCols; x += step) {
            for (int z = 0; z < mapRows; ++z) {
                rlVertex3f(globalX(x), gridHeight(map, x, z, mapCols, mapRows), globalZ(z));
                rlVertex3f(globalX(x), gridHeight(map, x, z + 1, mapCols, mapRows), globalZ(z + 1));
            }
        }
    
        // Horizontal lines (constant Z, varying X)
        for (int z = 0; z <= mapRows; z += step) {
            for (int x = 0; x < mapCols; ++x) {
                rlVertex3f(globalX(x), gridHeight(map, x, z, mapCols, mapRows), globalZ(z));
                rlVertex3f(globalX(x + 1), gridHeight(map, x + 1, z, mapCols, mapRows), globalZ(z));
            }
        }

        rlEnd();
    }
    
    void drawGrid(const Map& map) const {
        int mapCols = static_cast<int>(map.width_ * map.gen.regionWidth_);
        int mapRows = static_cast<int>(map.height_ * map.gen.regionHeight_);
    
        drawGridLines_(map, mapCols, mapRows, 8, 120, 100, 80, 60);
        drawGridLines_(map, mapCols, mapRows, CHUNK_SIZE, 255, 255, 255, 160);
    }

    void unload() {
        for (auto& m : chunkModels_)
            UnloadModel(m);
        chunkModels_.clear();
        dirty_.clear();
        chunkCountX_ = 0;
        chunkCountZ_ = 0;
    }

    float heightAt(const Map& map, int x, int z) const {
        const Cell& c = map.cellAt(x, z);
        return c.absolute_elevation * heightScale + c.craterDelta * craterScale;
    }

};

#endif
