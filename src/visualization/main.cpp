#include <raylib.h>
#include <raymath.h>

#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

#define RLIGHTS_IMPLEMENTATION
#include "../../third_party/rlights.h"

#include <memory>
#include <algorithm>
#include <vector>

#include "../../include/terrain_generation.hpp"
#include "../../include/renderer.hpp"
#include "../../include/controls.hpp"

namespace {

void updateFreeCamera(Camera3D& cam, float moveSpeed) {
    Vector3 forward = Vector3Subtract(cam.target, cam.position);
    forward = Vector3Normalize(forward);

    Vector3 right = Vector3CrossProduct(forward, cam.up);
    right = Vector3Normalize(right);

    float dt = GetFrameTime();
    float speed = moveSpeed * dt;

    Vector3 move{};

    if (IsKeyDown(KEY_W)) move = Vector3Add(move, forward);
    if (IsKeyDown(KEY_S)) move = Vector3Subtract(move, forward);
    if (IsKeyDown(KEY_A)) move = Vector3Subtract(move, right);
    if (IsKeyDown(KEY_D)) move = Vector3Add(move, right);
    if (IsKeyDown(KEY_SPACE)) move.y += 1.0f;
    if (IsKeyDown(KEY_LEFT_SHIFT)) move.y -= 1.0f;

    if (Vector3Length(move) > 0.0f) {
        move = Vector3Scale(Vector3Normalize(move), speed);
        cam.position = Vector3Add(cam.position, move);
        cam.target = Vector3Add(cam.target, move);
    }

    Vector2 mouseDelta = GetMouseDelta();
    if (mouseDelta.x != 0.0f || mouseDelta.y != 0.0f) {
        float sensitivity = 0.002f;
        float yaw = -mouseDelta.x * sensitivity;
        float pitch = -mouseDelta.y * sensitivity;

        Vector3 dir = Vector3Subtract(cam.target, cam.position);
        float len = Vector3Length(dir);
        dir = Vector3Normalize(dir);

        Matrix yawMat = MatrixRotate(cam.up, yaw);
        dir = Vector3Transform(dir, yawMat);

        Vector3 rightDir = Vector3Normalize(Vector3CrossProduct(dir, cam.up));
        Matrix pitchMat = MatrixRotate(rightDir, pitch);
        dir = Vector3Transform(dir, pitchMat);

        cam.target = Vector3Add(cam.position, Vector3Scale(dir, len));
    }

    float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
        Vector3 dir = Vector3Subtract(cam.target, cam.position);
        float len = Vector3Length(dir);
        dir = Vector3Scale(dir, 1.0f / len);
        float newLen = std::max(len * (1.0f - wheel * 0.1f), moveSpeed * 0.5f);
        cam.position = Vector3Subtract(cam.target, Vector3Scale(dir, newLen));
    }
}

Region makeExploredRegion(const Map& map, size_t regionIdx,
                          int droneX, int droneZ, float sensorRange) {
    Region reg = map.tiles[regionIdx];

    int r = static_cast<int>(std::ceil(sensorRange)) + 1;
    float edgeWidth = 1.5f;

    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            int x = droneX + dx;
            int y = droneZ + dy;
            if (x < 0 || y < 0 || x >= reg.width || y >= reg.height) continue;

            float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
            if (dist > sensorRange + edgeWidth) continue;

            if (dist > sensorRange) {
                float t = 1.0f - (dist - sensorRange) / edgeWidth;
                reg[x, y].is_visited = true;
                reg[x, y].roughness = reg[x, y].roughness * (1.0f - t * 0.5f);
            } else {
                reg[x, y].is_visited = true;
            }
        }
    }
    return reg;
}

} // namespace

int main(void) {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "TerraNav");
    SetTargetFPS(60);
    SetExitKey(0);

    // --- Shader + lighting ---
    Shader lightingShader = LoadShader("src/visualization/shaders/lighting.vs",
                                        "src/visualization/shaders/lighting.fs");
    lightingShader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(lightingShader, "viewPos");
    int ambientLoc = GetShaderLocation(lightingShader, "ambient");
    float ambient[] = { 0.15f, 0.14f, 0.12f, 1.0f };
    SetShaderValue(lightingShader, ambientLoc, ambient, SHADER_UNIFORM_VEC4);

    float fogColor[] = { 0.78f, 0.67f, 0.49f, 1.0f };
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "fogColor"),
                   fogColor, SHADER_UNIFORM_VEC4);
    float fogDensity = 0.0f;
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "fogDensity"),
                   &fogDensity, SHADER_UNIFORM_FLOAT);

    Light sunLight = CreateLight(LIGHT_DIRECTIONAL,
                                  { 50.0f, 100.0f, 30.0f },
                                  { 0.0f, 0.0f, 0.0f },
                                  { 200, 180, 150, 255 },
                                  lightingShader);

    // --- Map generation ---
    constexpr size_t REGIONS_X = 16;
    constexpr size_t REGIONS_Y = 16;
    constexpr size_t REGION_W = 64;
    constexpr size_t REGION_H = 64;

    CraterParams craterParams {
        .maxCountPerRegion = 1,
        .minRadius = 0.5,
        .maxRadius = 20.0,
        .depthFactor = 0.01,
        .rimRatio = 0.1,
        .rimWidth = 0.4,
    };

    int seedValue = 42;

    auto map = std::make_unique<Map>(
        seedValue,
        REGIONS_X, REGIONS_Y,
        REGION_W, REGION_H,
        NoiseParams{},
        craterParams
    );

    map->generate();

    float mapHalf = static_cast<float>(REGIONS_X * REGION_W) * 0.5f * CELL_SIZE;

    // --- Renderer ---
    Renderer renderer {
        .heightScale = 75.0f,
        .craterScale = 75.0f,
    };

    auto setupMaterial = [&]() {
        for (auto& model : renderer.chunkModels_)
            model.materials[0].shader = lightingShader;
        if (renderer.regionModel_.meshes != nullptr)
            renderer.regionModel_.materials[0].shader = lightingShader;
    };

    // Build full-map chunk meshes for satellite mode
    renderer.rebuildAll(*map);
    setupMaterial();

    // --- Current region for ground view ---
    int currentRegionX = static_cast<int>(REGIONS_X) / 2;
    int currentRegionY = static_cast<int>(REGIONS_Y) / 2;
    size_t currentRegionIdx = static_cast<size_t>(currentRegionY) * map->width_
                            + static_cast<size_t>(currentRegionX);

    int droneCellX = static_cast<int>(REGION_W) / 2;
    int droneCellZ = static_cast<int>(REGION_H) / 2;
    float sensorRange = 7.0f;

    // Global drone position (for satellite marker)
    int droneGlobalX = currentRegionX * static_cast<int>(REGION_W) + droneCellX;
    int droneGlobalZ = currentRegionY * static_cast<int>(REGION_H) + droneCellZ;

    Region exploredReg = makeExploredRegion(*map, currentRegionIdx,
                                            droneCellX, droneCellZ, sensorRange);
    renderer.rebuildRegion(exploredReg, true);
    setupMaterial();

    // --- Drone + markers ---
    Mesh droneMesh = GenMeshSphere(0.5f, 16, 16);
    Model droneModel = LoadModelFromMesh(droneMesh);
    droneModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = { 100, 200, 255, 255 };

    Position startPos{ .x = 5, .y = 5 };
    Position goalPos{ .x = static_cast<int>(REGION_W) - 5,
                     .y = static_cast<int>(REGION_H) - 5 };

    Model startMarker = LoadModelFromMesh(GenMeshCube(0.8f, 1.5f, 0.8f));
    startMarker.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = { 80, 255, 80, 255 };

    Model goalMarker = LoadModelFromMesh(GenMeshCube(0.8f, 1.5f, 0.8f));
    goalMarker.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = { 255, 80, 80, 255 };

    // --- Cameras ---
    Camera3D groundCamera{
        .position = { 55.0f, 80.0f, 55.0f },
        .target = { 0.0f, 0.0f, 0.0f },
        .up = { 0.0f, 1.0f, 0.0f },
        .fovy = 45.0f,
        .projection = CAMERA_PERSPECTIVE,
    };

    Camera3D satelliteCamera{
        .position = { mapHalf * 0.8f, mapHalf * 0.8f, mapHalf * 0.8f },
        .target = { 0.0f, 0.0f, 0.0f },
        .up = { 0.0f, 1.0f, 0.0f },
        .fovy = 45.0f,
        .projection = CAMERA_PERSPECTIVE,
    };

    Camera3D camera = satelliteCamera;

    // --- Controls ---
    Controls controller {
        .seedValue = seedValue,
        .prevHeightScale = renderer.heightScale,
        .prevCraterScale = renderer.craterScale,
    };

    DisableCursor();
    rlSetClipPlanes(0.05, controller.renderDistance);

    RenderMode mode = RenderMode::Satellite;
    bool showGates = true;
    bool showGrid = true;
    bool fogEnabled = true;

    // Empty path/gates — populated by hierarchical navigator later
    std::vector<Position> microPath;
    std::vector<int> macroPath;
    std::vector<GateInfo> gates;

    // --- Main loop ---
    while (!WindowShouldClose()) {
        // TAB: toggle render mode
        if (IsKeyPressed(KEY_TAB)) {
            mode = (mode == RenderMode::Ground) ? RenderMode::Satellite : RenderMode::Ground;
            camera = (mode == RenderMode::Satellite) ? satelliteCamera : groundCamera;
        }

        if (IsKeyPressed(KEY_G)) showGates = !showGates;
        if (IsKeyPressed(KEY_F)) showGrid = !showGrid;
        if (IsKeyPressed(KEY_H)) { fogEnabled = !fogEnabled; renderer.regionDirty_ = true; }

        controller.handleEscape();

        if (IsKeyPressed(KEY_R)) {
            camera = (mode == RenderMode::Satellite) ? satelliteCamera : groundCamera;
        }

        if (!controller.uiMode || controller.autoRotate) {
            if (controller.autoRotate)
                UpdateCamera(&camera, CAMERA_ORBITAL);
            else
                updateFreeCamera(camera, mode == RenderMode::Satellite ? mapHalf * 0.15f : 20.0f);
        }

        Vector3 viewPos = camera.position;
        SetShaderValue(lightingShader, lightingShader.locs[SHADER_LOC_VECTOR_VIEW],
                       &viewPos, SHADER_UNIFORM_VEC3);

        // Rebuild on parameter change
        if (controller.update(renderer.heightScale, renderer.craterScale)) {
            renderer.markAllDirty();
            renderer.rebuildDirty(*map);
            setupMaterial();
            renderer.regionDirty_ = true;
            controller.prevHeightScale = renderer.heightScale;
            controller.prevCraterScale = renderer.craterScale;
        }

        if (renderer.regionDirty_) {
            renderer.rebuildRegion(exploredReg, fogEnabled);
            setupMaterial();
        }

        rlSetClipPlanes(0.05, controller.renderDistance);

        // Regenerate map
        if (controller.consumeRegenerate()) {
            map = std::make_unique<Map>(
                static_cast<long long>(controller.seedValue),
                REGIONS_X, REGIONS_Y,
                REGION_W, REGION_H,
                NoiseParams{}, craterParams
            );
            map->generate();
            renderer.rebuildAll(*map);
            setupMaterial();
            exploredReg = makeExploredRegion(*map, currentRegionIdx,
                                            droneCellX, droneCellZ, sensorRange);
            renderer.regionDirty_ = true;
        }

        // --- Drawing ---
        BeginDrawing();

            ClearBackground({ 199, 170, 125, 255 });

            BeginMode3D(camera);
                if (mode == RenderMode::Satellite) {
                    renderer.drawSatellite(*map, showGates, showGrid, gates, macroPath,
                                           static_cast<int>(REGIONS_X),
                                           static_cast<int>(REGIONS_Y),
                                           droneGlobalX, droneGlobalZ);
                } else {
                    renderer.drawGround(microPath);

                    float dx = renderer.globalX(droneCellX);
                    float dz = renderer.globalZ(droneCellZ);
                    float dy = renderer.heightAt(exploredReg[droneCellX, droneCellZ]) + CELL_SIZE * 0.5f;
                    DrawModel(droneModel, { dx, dy, dz }, 1.0f, WHITE);

                    float sx = renderer.globalX(startPos.x);
                    float sz = renderer.globalZ(startPos.y);
                    float sy = renderer.heightAt(exploredReg[startPos.x, startPos.y]) + 0.4f;
                    DrawModel(startMarker, { sx, sy, sz }, 1.0f, WHITE);

                    float gx = renderer.globalX(goalPos.x);
                    float gz = renderer.globalZ(goalPos.y);
                    float gy = renderer.heightAt(exploredReg[goalPos.x, goalPos.y]) + 0.4f;
                    DrawModel(goalMarker, { gx, gy, gz }, 1.0f, WHITE);

                    DrawCircle3D({ dx, dy, dz }, sensorRange, { 1.0f, 0.0f, 0.0f }, 90.0f,
                                 { 100, 200, 255, 100 });
                }
            EndMode3D();

            int cols = static_cast<int>(REGION_W);
            int rows = static_cast<int>(REGION_H);
            controller.draw(
                &renderer.heightScale, &renderer.craterScale,
                screenWidth, screenHeight,
                renderer.chunkModels_.size(),
                map->rocks.size(),
                cols * rows,
                (cols - 1) * (rows - 1) * 2
            );

            const char* modeText = (mode == RenderMode::Satellite) ? "SATELLITE" : "GROUND";
            DrawText(modeText, screenWidth - 120, 15, 20, { 255, 255, 255, 220 });
            DrawText("TAB: switch", screenWidth - 120, 40, 14, { 200, 200, 200, 180 });
            DrawText("G: gates  F: grid  H: fog", screenWidth - 160, 58, 14, { 200, 200, 200, 180 });

        EndDrawing();
    }

    UnloadModel(droneModel);
    UnloadModel(startMarker);
    UnloadModel(goalMarker);
    UnloadShader(lightingShader);
    renderer.unload();
    CloseWindow();
    return 0;
}
