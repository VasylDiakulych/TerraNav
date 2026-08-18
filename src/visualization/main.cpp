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
#include "../../include/region_pathfinding.hpp"
#include "../../include/hierarchical_navigator.hpp"

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

// Ray-march a mouse ray against the terrain heightmap to find the clicked cell.
// Returns {-1,-1} if no intersection within the region.
Position pickTerrainCell(const Camera3D& camera, const Renderer& renderer,
                         const Region& reg, int regionW, int regionH) {
    Ray ray = GetMouseRay(GetMousePosition(), camera);

    // March from camera along ray in small steps, check when we go below terrain
    Vector3 pos = ray.position;
    Vector3 dir = ray.direction;
    float step = 0.5f;
    int maxSteps = 2000;

    for (int i = 0; i < maxSteps; ++i) {
        pos = Vector3Add(pos, Vector3Scale(dir, step));

        // Convert world position to cell coords
        float cellXF = (pos.x - renderer.offsetX_) / CELL_SIZE;
        float cellZF = (pos.z - renderer.offsetZ_) / CELL_SIZE;

        int cx = static_cast<int>(std::floor(cellXF));
        int cz = static_cast<int>(std::floor(cellZF));

        if (cx < 0 || cz < 0 || cx >= regionW || cz >= regionH) continue;

        float terrainY = renderer.heightAt(reg[cx, cz]);
        if (pos.y <= terrainY) {
            return {cx, cz};
        }
    }
    return {-1, -1};
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

    // --- Region graph (gates + macro A*) ---
    RegionGraph regionGraph;
    regionGraph.buildFromMap(*map);

    int startRegionX = 0, startRegionY = 0;
    int goalRegionX = static_cast<int>(REGIONS_X) - 1, goalRegionY = static_cast<int>(REGIONS_Y) - 1;

    // --- Hierarchical navigator ---
    float sensorRange = 12.0f;
    HierarchicalNavigator hNav;
    hNav.init(map.get(), &regionGraph, startRegionX, startRegionY,
              goalRegionX, goalRegionY, sensorRange);

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

    renderer.rebuildAll(*map);
    setupMaterial();

    Region exploredReg = hNav.getExploredRegion();
    renderer.rebuildRegion(exploredReg, true);
    setupMaterial();

    // --- Drone + markers ---
    Mesh droneMesh = GenMeshSphere(0.5f, 16, 16);
    Model droneModel = LoadModelFromMesh(droneMesh);
    droneModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = { 100, 200, 255, 255 };

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

    float tickTimer = 0.0f;
    float tickInterval = 0.3f;
    bool paused = true;

    // --- Main loop ---
    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_TAB)) {
            mode = (mode == RenderMode::Ground) ? RenderMode::Satellite : RenderMode::Ground;
            if (mode == RenderMode::Satellite) {
                float dx = renderer.satOffsetX_ + static_cast<float>(hNav.getDroneGlobalX()) * CELL_SIZE;
                float dz = renderer.satOffsetZ_ + static_cast<float>(hNav.getDroneGlobalZ()) * CELL_SIZE;
                satelliteCamera.position = { dx + 200.0f, 200.0f, dz + 200.0f };
                satelliteCamera.target = { dx, 0.0f, dz };
                camera = satelliteCamera;
            } else {
                camera = groundCamera;
            }
        }

        if (IsKeyPressed(KEY_G)) showGates = !showGates;
        if (IsKeyPressed(KEY_F)) showGrid = !showGrid;
        if (IsKeyPressed(KEY_H)) { fogEnabled = !fogEnabled; renderer.regionDirty_ = true; }

        // Play/pause
        if (IsKeyPressed(KEY_P)) { paused = !paused; hNav.running = !paused; }
        if (IsKeyPressed(KEY_N) && paused) { hNav.tick(); renderer.regionDirty_ = true; }
        // Reset
        if (IsKeyPressed(KEY_Y)) {
            regionGraph.buildFromMap(*map);
            hNav.init(map.get(), &regionGraph, startRegionX, startRegionY,
                      goalRegionX, goalRegionY, sensorRange);
            renderer.regionDirty_ = true;
        }

        controller.handleEscape();

        // Wall placement: left-click in ground mode + UI mode
        if (mode == RenderMode::Ground && controller.uiMode &&
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Position picked = pickTerrainCell(camera, renderer, exploredReg,
                                              static_cast<int>(REGION_W),
                                              static_cast<int>(REGION_H));
            if (picked.x >= 0) {
                hNav.toggleWall(picked.x, picked.y);
                renderer.regionDirty_ = true;
            }
        }

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

        // Simulation tick
        if (!paused && !hNav.finished) {
            tickTimer += GetFrameTime();
            if (tickTimer >= tickInterval) {
                tickTimer = 0.0f;
                hNav.tick();
                renderer.regionDirty_ = true;
            }
        }

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
            exploredReg = hNav.getExploredRegion();
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
            regionGraph.buildFromMap(*map);
            hNav.init(map.get(), &regionGraph, startRegionX, startRegionY,
                      goalRegionX, goalRegionY, sensorRange);
            renderer.rebuildAll(*map);
            setupMaterial();
            renderer.regionDirty_ = true;
        }

        // --- Drawing ---
        BeginDrawing();

            ClearBackground({ 199, 170, 125, 255 });

            BeginMode3D(camera);
                if (mode == RenderMode::Satellite) {
                    std::vector<int> macroPath;
                    for (const auto& step : hNav.macroPath)
                        macroPath.push_back(step.regionY * regionGraph.regionsX + step.regionX);

                    renderer.drawSatellite(*map, showGates, showGrid, regionGraph.gates, macroPath,
                                           static_cast<int>(REGIONS_X),
                                           static_cast<int>(REGIONS_Y),
                                           hNav.getDroneGlobalX(), hNav.getDroneGlobalZ());
                } else {
                    int regionW = static_cast<int>(REGION_W);
                    int regionH = static_cast<int>(REGION_H);
                    int originX = hNav.currentRegionX * regionW;
                    int originY = hNav.currentRegionY * regionH;
                    renderer.drawGround(hNav.microPath, map->rocks, originX, originY, regionW, regionH);

                    int droneX = hNav.droneLocal.x;
                    int droneZ = hNav.droneLocal.y;
                    float dx = renderer.globalX(droneX);
                    float dz = renderer.globalZ(droneZ);
                    float dy = renderer.heightAt(exploredReg[droneX, droneZ]) + CELL_SIZE * 0.5f;
                    DrawModel(droneModel, { dx, dy, dz }, 1.0f, WHITE);

                    if (!hNav.finished && hNav.currentStepIndex == static_cast<int>(hNav.macroPath.size()) - 1) {
                        float gx = renderer.globalX(hNav.finalGoal.x);
                        float gz = renderer.globalZ(hNav.finalGoal.y);
                        float gy = renderer.heightAt(exploredReg[hNav.finalGoal.x, hNav.finalGoal.y]) + 0.4f;
                        DrawModel(goalMarker, { gx, gy, gz }, 1.0f, WHITE);
                    }

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
            DrawText("P: play/pause  N: step  Y: reset", screenWidth - 280, 58, 14, { 200, 200, 200, 180 });
            DrawText("G: gates  F: grid  H: fog", screenWidth - 160, 76, 14, { 200, 200, 200, 180 });

            const char* stateText = paused ? "PAUSED" : (hNav.finished ? "DONE" : "RUNNING");
            DrawText(stateText, 10, 10, 20, { 255, 255, 255, 220 });
            DrawText(TextFormat("Region: (%d, %d)  Step: %d/%zu",
                                hNav.currentRegionX, hNav.currentRegionY,
                                hNav.currentStepIndex, hNav.macroPath.size()),
                     10, 35, 16, { 255, 255, 255, 200 });

        EndDrawing();
    }

    UnloadModel(droneModel);
    UnloadModel(goalMarker);
    UnloadShader(lightingShader);
    renderer.unload();
    CloseWindow();
    return 0;
}
