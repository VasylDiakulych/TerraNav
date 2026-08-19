#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <algorithm>
#include <vector>
#include <cmath>
#include <raylib.h>
#include <rlgl.h>

#include "generics.hpp"
#include "terrain_generation.hpp"
#include "region_pathfinding.hpp"

constexpr float CELL_SIZE = 1.0f;
constexpr int CHUNK_SIZE = 64;
constexpr int SUBDIV = 4;

inline Color lerpColor(Color a, Color b, float t) {
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f) t = 1.0f;

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
    if (e > 1.0f) e = 1.0f;

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
            float t = (e - stops[i - 1].pos) / (stops[i].pos - stops[i - 1].pos);
            return lerpColor(stops[i - 1].color, stops[i].color, t);
        }
    }
    return stops[std::size(stops) - 1].color;
}

inline Color cellColor(const Cell& c, bool fogEnabled) {
    if (fogEnabled && !c.is_visited) return { 35, 30, 28, 255 };
    if (c.is_impassable && !c.is_rock) return { 200, 50, 50, 255 };
    return elevationColor(c);
}


enum class RenderMode {
    Satellite,
    Ground
};

struct Renderer {
    float heightScale{ 75.0f };
    float craterScale{ 75.0f };

    // --- Satellite: full-map chunk system ---
    std::vector<Model> chunkModels_;
    std::vector<bool> chunkDirty_;
    size_t chunkCountX_ = 0;
    size_t chunkCountZ_ = 0;

    // --- Ground: single-region mesh ---
    Model regionModel_{};
    bool regionDirty_ = true;

    // Wall flash effect
    Position flashCell_{-1, -1};
    float flashTimer_ = 0.0f;

    // Rock models (variations)
    std::vector<Model> rockModels_;

    // Offsets: satellite uses full-map, ground uses single-region
    float satOffsetX_ = 0.0f;
    float satOffsetZ_ = 0.0f;
    float offsetX_ = 0.0f;
    float offsetZ_ = 0.0f;

    // --- Mesh building (shared) ---

    void allocateMesh_(Mesh& mesh, int vertexCount, int triCount) {
        mesh.vertexCount = vertexCount;
        mesh.triangleCount = triCount;
        mesh.vertices  = static_cast<float*>(RL_MALLOC(vertexCount * 3 * sizeof(float)));
        mesh.texcoords = static_cast<float*>(RL_MALLOC(vertexCount * 2 * sizeof(float)));
        mesh.normals   = static_cast<float*>(RL_MALLOC(vertexCount * 3 * sizeof(float)));
        mesh.colors    = static_cast<unsigned char*>(RL_MALLOC(vertexCount * 4 * sizeof(unsigned char)));
        mesh.indices   = static_cast<unsigned short*>(RL_MALLOC(triCount * 3 * sizeof(unsigned short)));
    }

    void fillIndices_(Mesh& mesh, int w, int h) {
        int t = 0;
        for (int row = 0; row < h - 1; ++row) {
            for (int col = 0; col < w - 1; ++col) {
                unsigned short v00 = static_cast<unsigned short>(row * w + col);
                unsigned short v10 = static_cast<unsigned short>(row * w + (col + 1));
                unsigned short v01 = static_cast<unsigned short>((row + 1) * w + col);
                unsigned short v11 = static_cast<unsigned short>((row + 1) * w + (col + 1));

                mesh.indices[t++] = v00;
                mesh.indices[t++] = v01;
                mesh.indices[t++] = v10;

                mesh.indices[t++] = v10;
                mesh.indices[t++] = v01;
                mesh.indices[t++] = v11;
            }
        }
    }

    // --- Satellite: chunk mesh from Map (full knowledge, no fog) ---

    void fillChunkVertexData_(Mesh& mesh, const Map& map, int beginX, int beginZ,
                             int chunkCols, int chunkRows, int mapCols, int mapRows) {
        for (int localZ = 0; localZ < chunkRows; ++localZ) {
            int globalZ = beginZ + localZ;
            for (int localX = 0; localX < chunkCols; ++localX) {
                int globalX = beginX + localX;
                int idx = localZ * chunkCols + localX;
                const Cell& c = map.cellAt(globalX, globalZ);

                float h = heightAt(map, globalX, globalZ);

                mesh.vertices[idx * 3 + 0] = satOffsetX_ + static_cast<float>(globalX) * CELL_SIZE;
                mesh.vertices[idx * 3 + 1] = h;
                mesh.vertices[idx * 3 + 2] = satOffsetZ_ + static_cast<float>(globalZ) * CELL_SIZE;

                constexpr float TEX_REPEAT = 1.0f / 64.0f;
                mesh.texcoords[idx * 2 + 0] = static_cast<float>(globalX) * TEX_REPEAT;
                mesh.texcoords[idx * 2 + 1] = static_cast<float>(globalZ) * TEX_REPEAT;

                int radius = 2;
                int leftX  = std::max(globalX - radius, 0);
                int rightX = std::min(globalX + radius, mapCols - 1);
                int upZ    = std::max(globalZ - radius, 0);
                int downZ  = std::min(globalZ + radius, mapRows - 1);
                float dx = (heightAt(map, rightX, globalZ) - heightAt(map, leftX, globalZ)) / static_cast<float>(rightX - leftX);
                float dz = (heightAt(map, globalX, downZ) - heightAt(map, globalX, upZ)) / static_cast<float>(downZ - upZ);
                float len = std::sqrt(dx * dx + 1.0f + dz * dz);
                float invLen = 1.0f / len;
                mesh.normals[idx * 3 + 0] = -dx * invLen;
                mesh.normals[idx * 3 + 1] = invLen;
                mesh.normals[idx * 3 + 2] = -dz * invLen;

                Color col = elevationColor(c);
                mesh.colors[idx * 4 + 0] = col.r;
                mesh.colors[idx * 4 + 1] = col.g;
                mesh.colors[idx * 4 + 2] = col.b;
                mesh.colors[idx * 4 + 3] = col.a;
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

        Mesh mesh{};
        allocateMesh_(mesh, vertexCount, triCount);
        fillChunkVertexData_(mesh, map, beginX, beginZ, chunkCols, chunkRows, mapCols, mapRows);
        fillIndices_(mesh, chunkCols, chunkRows);

        int modelIdx = cz * chunkCountX_ + cx;
        if (modelIdx < static_cast<int>(chunkModels_.size()) && chunkModels_[modelIdx].meshes != nullptr)
            UnloadModel(chunkModels_[modelIdx]);

        UploadMesh(&mesh, false);
        chunkModels_[modelIdx] = LoadModelFromMesh(mesh);
    }

    void rebuildAll(const Map& map) {
        unloadChunks();

        int cols = static_cast<int>(map.width_ * map.gen.regionWidth_);
        int rows = static_cast<int>(map.height_ * map.gen.regionHeight_);
        chunkCountX_ = (cols + CHUNK_SIZE - 1) / CHUNK_SIZE;
        chunkCountZ_ = (rows + CHUNK_SIZE - 1) / CHUNK_SIZE;

        offsetX_ = -cols * 0.5f * CELL_SIZE;
        offsetZ_ = -rows * 0.5f * CELL_SIZE;
        satOffsetX_ = offsetX_;
        satOffsetZ_ = offsetZ_;

        chunkModels_.resize(chunkCountX_ * chunkCountZ_);
        chunkDirty_.assign(chunkCountX_ * chunkCountZ_, true);

        rebuildDirty(map);
    }

    void rebuildDirty(const Map& map) {
        for (int cz = 0; cz < static_cast<int>(chunkCountZ_); ++cz) {
            for (int cx = 0; cx < static_cast<int>(chunkCountX_); ++cx) {
                if (chunkDirty_[cz * chunkCountX_ + cx]) {
                    rebuildChunk_(map, cx, cz);
                    chunkDirty_[cz * chunkCountX_ + cx] = false;
                }
            }
        }
    }

    void markAllDirty() { std::fill(chunkDirty_.begin(), chunkDirty_.end(), true); }

    // --- Ground: single-region mesh with fog of war ---

    void fillRegionVertexData_(Mesh& mesh, const Region& reg, bool fogEnabled) {
        int w = reg.width;
        int h = reg.height;
        int meshW = (w - 1) * SUBDIV + 1;
        int meshH = (h - 1) * SUBDIV + 1;

        auto sampleHeight = [&](float fx, float fz) -> float {
            int x0 = std::clamp(static_cast<int>(fx), 0, w - 1);
            int z0 = std::clamp(static_cast<int>(fz), 0, h - 1);
            int x1 = std::min(x0 + 1, w - 1);
            int z1 = std::min(z0 + 1, h - 1);
            float tx = fx - static_cast<float>(x0);
            float tz = fz - static_cast<float>(z0);

            float h00 = heightAt(reg[x0, z0]);
            float h10 = heightAt(reg[x1, z0]);
            float h01 = heightAt(reg[x0, z1]);
            float h11 = heightAt(reg[x1, z1]);

            return (h00 * (1 - tx) + h10 * tx) * (1 - tz) +
                   (h01 * (1 - tx) + h11 * tx) * tz;
        };

        for (int mz = 0; mz < meshH; ++mz) {
            for (int mx = 0; mx < meshW; ++mx) {
                int idx = mz * meshW + mx;

                float fx = static_cast<float>(mx) / SUBDIV;
                float fz = static_cast<float>(mz) / SUBDIV;

                int cellX = std::clamp(static_cast<int>(fx), 0, w - 1);
                int cellZ = std::clamp(static_cast<int>(fz), 0, h - 1);
                const Cell& c = reg[cellX, cellZ];

                float hVal = sampleHeight(fx, fz);

                mesh.vertices[idx * 3 + 0] = offsetX_ + fx * CELL_SIZE;
                mesh.vertices[idx * 3 + 1] = hVal;
                mesh.vertices[idx * 3 + 2] = offsetZ_ + fz * CELL_SIZE;

                constexpr float TEX_REPEAT = 1.0f / 64.0f;
                mesh.texcoords[idx * 2 + 0] = fx * TEX_REPEAT;
                mesh.texcoords[idx * 2 + 1] = fz * TEX_REPEAT;

                float step = 1.0f / SUBDIV;
                float hxL = sampleHeight(std::max(fx - step, 0.0f), fz);
                float hxR = sampleHeight(std::min(fx + step, static_cast<float>(w - 1)), fz);
                float hzU = sampleHeight(fx, std::max(fz - step, 0.0f));
                float hzD = sampleHeight(fx, std::min(fz + step, static_cast<float>(h - 1)));
                float dx = (hxR - hxL) / (2.0f * step * CELL_SIZE);
                float dz = (hzD - hzU) / (2.0f * step * CELL_SIZE);
                float len = std::sqrt(dx * dx + 1.0f + dz * dz);
                float invLen = 1.0f / len;
                mesh.normals[idx * 3 + 0] = -dx * invLen;
                mesh.normals[idx * 3 + 1] = invLen;
                mesh.normals[idx * 3 + 2] = -dz * invLen;

                Color col = cellColor(c, fogEnabled);
                mesh.colors[idx * 4 + 0] = col.r;
                mesh.colors[idx * 4 + 1] = col.g;
                mesh.colors[idx * 4 + 2] = col.b;
                mesh.colors[idx * 4 + 3] = col.a;
            }
        }
    }

    void rebuildRegion(const Region& reg, bool fogEnabled = true) {
        int w = reg.width;
        int h = reg.height;
        int meshW = (w - 1) * SUBDIV + 1;
        int meshH = (h - 1) * SUBDIV + 1;
        int vertexCount = meshW * meshH;
        int triCount = (meshW - 1) * (meshH - 1) * 2;

        offsetX_ = -static_cast<float>(w) * 0.5f * CELL_SIZE;
        offsetZ_ = -static_cast<float>(h) * 0.5f * CELL_SIZE;

        Mesh mesh{};
        allocateMesh_(mesh, vertexCount, triCount);
        fillRegionVertexData_(mesh, reg, fogEnabled);
        fillIndices_(mesh, meshW, meshH);

        if (regionModel_.meshes != nullptr)
            UnloadModel(regionModel_);

        UploadMesh(&mesh, false);
        regionModel_ = LoadModelFromMesh(mesh);
        regionDirty_ = false;
    }

    // --- Drawing ---

    void drawSatellite(const Map& map, bool showGates, bool showGrid,
                       const std::vector<Gate>& gates,
                       const std::vector<int>& macroPath,
                       int regionsX, int regionsY,
                       int droneCellX, int droneCellZ) {
        float saveOffX = offsetX_;
        float saveOffZ = offsetZ_;
        offsetX_ = satOffsetX_;
        offsetZ_ = satOffsetZ_;

        for (const Model& m : chunkModels_)
            DrawModel(m, { 0, 0, 0 }, 1.0f, WHITE);

        if (showGrid)
            drawRegionGrid_(map);

        if (!macroPath.empty())
            drawMacroPath_(macroPath, regionsX, regionsY, map);

        if (showGates)
            drawGates_(gates, map);

        // Drone marker on full map
        int mapCols = static_cast<int>(map.width_ * map.gen.regionWidth_);
        int mapRows = static_cast<int>(map.height_ * map.gen.regionHeight_);
        float dx = globalX(droneCellX);
        float dz = globalZ(droneCellZ);
        float dy = heightAt(map, std::clamp(droneCellX, 0, mapCols - 1),
                                  std::clamp(droneCellZ, 0, mapRows - 1)) + 3.0f;
        DrawCube({ dx, dy, dz }, 3.0f, 3.0f, 3.0f, { 100, 200, 255, 255 });

        offsetX_ = saveOffX;
        offsetZ_ = saveOffZ;
    }

    void drawRegionGrid_(const Map& map) {
        int mapCols = static_cast<int>(map.width_ * map.gen.regionWidth_);
        int mapRows = static_cast<int>(map.height_ * map.gen.regionHeight_);

        auto gridH = [&](int x, int z) -> float {
            x = std::clamp(x, 0, mapCols - 1);
            z = std::clamp(z, 0, mapRows - 1);
            return heightAt(map, x, z) + 0.5f;
        };

        rlBegin(RL_LINES);
            rlColor4ub(255, 255, 255, 100);
            for (int x = 0; x <= mapCols; x += CHUNK_SIZE) {
                for (int z = 0; z < mapRows; ++z) {
                    rlVertex3f(globalX(x), gridH(x, z), globalZ(z));
                    rlVertex3f(globalX(x), gridH(x, z + 1), globalZ(z + 1));
                }
            }
            for (int z = 0; z <= mapRows; z += CHUNK_SIZE) {
                for (int x = 0; x < mapCols; ++x) {
                    rlVertex3f(globalX(x), gridH(x, z), globalZ(z));
                    rlVertex3f(globalX(x + 1), gridH(x + 1, z), globalZ(z));
                }
            }
        rlEnd();
    }

    void drawMacroPath_(const std::vector<int>& macroPath,
                        int regionsX, int regionsY, const Map& map) {
        if (macroPath.size() < 2) return;

        int regionW = CHUNK_SIZE;
        int regionH = CHUNK_SIZE;

        rlBegin(RL_LINES);
        rlColor4ub(100, 200, 255, 255);
        rlSetLineWidth(3);

        for (size_t i = 0; i + 1 < macroPath.size(); ++i) {
            int rx0 = macroPath[i] % regionsX;
            int ry0 = macroPath[i] / regionsX;
            int rx1 = macroPath[i + 1] % regionsX;
            int ry1 = macroPath[i + 1] / regionsX;

            int cellX0 = rx0 * regionW + regionW / 2;
            int cellZ0 = ry0 * regionH + regionH / 2;
            int cellX1 = rx1 * regionW + regionW / 2;
            int cellZ1 = ry1 * regionH + regionH / 2;

            float cx0 = offsetX_ + static_cast<float>(cellX0) * CELL_SIZE;
            float cz0 = offsetZ_ + static_cast<float>(cellZ0) * CELL_SIZE;
            float cx1 = offsetX_ + static_cast<float>(cellX1) * CELL_SIZE;
            float cz1 = offsetZ_ + static_cast<float>(cellZ1) * CELL_SIZE;

            float y0 = heightAt(map, cellX0, cellZ0) + 5.0f;
            float y1 = heightAt(map, cellX1, cellZ1) + 5.0f;

            rlVertex3f(cx0, y0, cz0);
            rlVertex3f(cx1, y1, cz1);
        }

        rlEnd();
    }

    void drawGates_(const std::vector<Gate>& gates, const Map& map) {
        rlBegin(RL_LINES);
        rlColor4ub(255, 180, 60, 220);
        rlSetLineWidth(3);

        for (const Gate& g : gates) {
            if (g.blocked) continue;

            float x0 = offsetX_ + static_cast<float>(g.globalStartX) * CELL_SIZE;
            float z0 = offsetZ_ + static_cast<float>(g.globalStartY) * CELL_SIZE;
            float x1 = offsetX_ + static_cast<float>(g.globalEndX) * CELL_SIZE;
            float z1 = offsetZ_ + static_cast<float>(g.globalEndY) * CELL_SIZE;

            float y0 = heightAt(map, g.globalStartX, g.globalStartY) + 2.0f;
            float y1 = heightAt(map, g.globalEndX, g.globalEndY) + 2.0f;

            rlVertex3f(x0, y0, z0);
            rlVertex3f(x1, y1, z1);
        }

        rlEnd();
    }

    void generateRockModels_() {
        for (auto& m : rockModels_)
            UnloadModel(m);
        rockModels_.clear();

        // Main rock models (low-poly hemispheres)
        int sides[] = {4, 5, 6};
        for (int s : sides) {
            Mesh m = GenMeshHemiSphere(1.0f, s, s + 1);
            UploadMesh(&m, false);
            Model model = LoadModelFromMesh(m);
            model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = { 85, 68, 55, 255 };
            rockModels_.push_back(model);
        }

        // Small pebble model for halo
        Mesh pebble = GenMeshHemiSphere(0.3f, 3, 3);
        UploadMesh(&pebble, false);
        Model pebbleModel = LoadModelFromMesh(pebble);
        pebbleModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = { 110, 90, 72, 255 };
        rockModels_.push_back(pebbleModel);
    }

    void drawGround(const std::vector<Position>& path, const std::vector<Rock>& rocks,
                    int regionOriginX, int regionOriginY, int regionW, int regionH,
                    const Region& reg) {
        if (regionModel_.meshes != nullptr)
            DrawModel(regionModel_, { 0, 0, 0 }, 1.0f, WHITE);

        drawRocks_(rocks, regionOriginX, regionOriginY, regionW, regionH);
        drawMicroPath_(path);
        drawFlash_(reg);
    }

    void triggerFlash(int cellX, int cellZ) {
        flashCell_ = {cellX, cellZ};
        flashTimer_ = 0.5f;
    }

    void updateFlash(float dt) {
        if (flashTimer_ > 0.0f) flashTimer_ -= dt;
    }

    void drawFlash_(const Region& reg) {
        if (flashTimer_ <= 0.0f || flashCell_.x < 0) return;
        float alpha = std::clamp(flashTimer_ / 0.5f, 0.0f, 1.0f);
        float x = globalX(flashCell_.x);
        float z = globalZ(flashCell_.y);
        float y = heightAt(reg[flashCell_.x, flashCell_.y]);
        DrawCube({x, y + 0.5f, z}, 1.0f, 2.0f, 1.0f, {255, 255, 100, static_cast<unsigned char>(alpha * 255)});
    }

    void drawRocks_(const std::vector<Rock>& rocks, int originX, int originY, int w, int h) {
        if (rockModels_.empty())
            generateRockModels_();

        for (size_t i = 0; i < rocks.size(); ++i) {
            const Rock& rock = rocks[i];
            int localX = static_cast<int>(rock.x) - originX;
            int localZ = static_cast<int>(rock.y) - originY;
            if (localX < 0 || localZ < 0 || localX >= w || localZ >= h) continue;

            // Count nearby rocks for cluster bonus
            int clusterCount = 0;
            for (size_t j = 0; j < rocks.size(); ++j) {
                if (i == j) continue;
                double dx = rocks[j].x - rock.x;
                double dy = rocks[j].y - rock.y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < rock.haloRadius + rocks[j].haloRadius) clusterCount++;
            }

            float rx = offsetX_ + static_cast<float>(rock.x - originX) * CELL_SIZE;
            float rz = offsetZ_ + static_cast<float>(rock.y - originY) * CELL_SIZE;

            // Interpolate terrain height at exact rock position
            float fx = static_cast<float>(rock.x - originX);
            float fz = static_cast<float>(rock.y - originY);
            int x0 = std::clamp(static_cast<int>(fx), 0, w - 1);
            int z0 = std::clamp(static_cast<int>(fz), 0, h - 1);
            int x1 = std::min(x0 + 1, w - 1);
            int z1 = std::min(z0 + 1, h - 1);
            float tx = fx - static_cast<float>(x0);
            float tz = fz - static_cast<float>(z0);
            float h00 = heightAt(regionModel_, x0, z0);
            float h10 = heightAt(regionModel_, x1, z0);
            float h01 = heightAt(regionModel_, x0, z1);
            float h11 = heightAt(regionModel_, x1, z1);
            float ry = (h00 * (1 - tx) + h10 * tx) * (1 - tz) +
                       (h01 * (1 - tx) + h11 * tx) * tz;

            float baseScale = static_cast<float>(rock.coreRadius) * 0.55f;
            if (baseScale < 0.35f) baseScale = 0.35f;
            float scale = baseScale * (1.0f + clusterCount * 0.2f);
            if (scale > baseScale * 1.8f) scale = baseScale * 1.8f;

            size_t idx = static_cast<size_t>(localX * 31 + localZ * 17);
            const Model& rockModel = rockModels_[idx % rockModels_.size()];

            float rotY = static_cast<float>((idx * 53) % 360);

            DrawModelEx(rockModel, { rx, ry, rz },
                        { 0, 1, 0 }, rotY,
                        { scale, scale * 0.95f, scale },
                        WHITE);

            // Scatter pebbles in the halo (walkable rough zone)
            const Model& pebbleModel = rockModels_.back();
            float haloR = static_cast<float>(rock.haloRadius);
            int pebbleCount = std::min(static_cast<int>(rock.haloRadius * 3), 8);
            unsigned int seed = static_cast<unsigned int>(i * 2654435761u);

            for (int p = 0; p < pebbleCount; ++p) {
                seed = seed * 1103515245u + 12345u;
                float angle = static_cast<float>(seed) / 4294967295.0f * 6.28318f;
                seed = seed * 1103515245u + 12345u;
                float dist = baseScale + (static_cast<float>(seed) / 4294967295.0f) *
                             (haloR - baseScale);
                if (dist <= baseScale) continue;

                float px = rx + std::cos(angle) * dist;
                float pz = rz + std::sin(angle) * dist;

                int pcx = std::clamp(static_cast<int>(fx + std::cos(angle) * dist), 0, w - 1);
                int pcz = std::clamp(static_cast<int>(fz + std::sin(angle) * dist), 0, h - 1);
                float py = heightAt(regionModel_, pcx, pcz);

                seed = seed * 1103515245u + 12345u;
                float pscale = 0.2f + (static_cast<float>(seed) / 4294967295.0f) * 0.3f;
                float prot = static_cast<float>((seed >> 8) % 360);

                DrawModelEx(pebbleModel, { px, py, pz },
                            { 0, 1, 0 }, prot,
                            { pscale, pscale * 0.5f, pscale },
                            WHITE);
            }
        }
    }

    void drawMicroPath_(const std::vector<Position>& path) {
        if (path.size() < 2) return;

        rlBegin(RL_LINES);
        rlColor4ub(100, 255, 100, 200);

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            float x0 = globalX(path[i].x);
            float z0 = globalZ(path[i].y);
            float y0 = heightAt(regionModel_, path[i].x, path[i].y) + 0.5f;
            float x1 = globalX(path[i + 1].x);
            float z1 = globalZ(path[i + 1].y);
            float y1 = heightAt(regionModel_, path[i + 1].x, path[i + 1].y) + 0.5f;

            rlVertex3f(x0, y0, z0);
            rlVertex3f(x1, y1, z1);
        }

        rlEnd();
    }

    // --- Helpers ---

    float heightAt(const Cell& c) const {
        return c.absolute_elevation * heightScale + c.craterDelta * craterScale;
    }

    float heightAt(const Map& map, int x, int z) const {
        const Cell& c = map.cellAt(x, z);
        return c.absolute_elevation * heightScale + c.craterDelta * craterScale;
    }

    float heightAt(const Model& model, int cellX, int cellZ) const {
        if (model.meshes == nullptr) return 0.0f;
        Mesh& m = model.meshes[0];
        int meshW = static_cast<int>(std::sqrt(static_cast<float>(m.vertexCount)));
        // Map cell coords to subdivided mesh coords
        int mx = std::clamp(cellX * SUBDIV, 0, meshW - 1);
        int mz = std::clamp(cellZ * SUBDIV, 0, meshW - 1);
        return m.vertices[(mz * meshW + mx) * 3 + 1];
    }

    float globalX(int cell) const { return offsetX_ + static_cast<float>(cell) * CELL_SIZE; }
    float globalZ(int cell) const { return offsetZ_ + static_cast<float>(cell) * CELL_SIZE; }

    void unloadChunks() {
        for (auto& m : chunkModels_)
            UnloadModel(m);
        chunkModels_.clear();
        chunkDirty_.clear();
        chunkCountX_ = 0;
        chunkCountZ_ = 0;
    }

    void unloadRegion() {
        if (regionModel_.meshes != nullptr)
            UnloadModel(regionModel_);
        regionModel_ = {};
        regionDirty_ = true;
    }

    void unload() {
        unloadChunks();
        unloadRegion();
        for (auto& m : rockModels_)
            UnloadModel(m);
        rockModels_.clear();
    }

    ~Renderer() { unload(); }
};

#endif
