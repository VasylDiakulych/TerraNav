#include <raylib.h>
#include <raymath.h>

#define RAYGUI_IMPLEMENTATION
#include "../../third_party/raygui.h"

#include <climits>
#include <cstdio>
#include <memory>

#include "../../include/terrain_generation.hpp"
#include "../../include/chunk_renderer.hpp"

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

} // namespace

int main(void) {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "TerraNav 3D");
    SetTargetFPS(60);
    SetExitKey(0);

    constexpr size_t REGIONS_X = 16;
    constexpr size_t REGIONS_Y = 16;
    constexpr size_t REGION_W = 64;
    constexpr size_t REGION_H = 64;

    CraterParams craterParams;
    craterParams.maxCountPerRegion = 1;
    craterParams.minRadius = 3.0;
    craterParams.maxRadius = 20.0;
    craterParams.depthFactor = 0.04;
    craterParams.rimRatio = 0.25;
    craterParams.rimWidth = 0.4;

    auto map = std::make_unique<Map>(42, REGIONS_X, REGIONS_Y, REGION_W, REGION_H,
                                     NoiseParams{}, craterParams);
    map->generate();

    TerrainRenderer renderer;
    renderer.heightScale = 375.0f;
    renderer.craterScale = 375.0f;

    float terrainHalf = (REGIONS_X * REGION_W) * 0.5f * CELL_SIZE;

    Camera3D camera{};
    camera.position = { terrainHalf * 1.2f, terrainHalf * 0.8f, terrainHalf * 1.2f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    int seedValue = 42;
    bool seedEdit = false;
    float renderDistance = 8000.0f;
    float prevHeightScale = renderer.heightScale;
    float prevCraterScale = renderer.craterScale;
    float rebuildTimer = 0.0f;
    bool uiMode = false;
    bool showGrid = false;
    bool autoRotate = false;

    DisableCursor();
    rlSetClipPlanes(0.05, renderDistance);

    const int panelX = 10;
    const int panelY = 10;
    const int panelW = 230;

    bool firstFrame = true;

    while (!WindowShouldClose()) {
        if (firstFrame) {
            renderer.rebuildAll(*map);
            firstFrame = false;
        }

        if (IsKeyPressed(KEY_ESCAPE)) {
            uiMode = !uiMode;
            if (uiMode) EnableCursor();
            else DisableCursor();
        }
        if (IsKeyPressed(KEY_R)) camera.target = { 0.0f, 0.0f, 0.0f };

        if (!uiMode || autoRotate) {
            if (autoRotate)
                UpdateCamera(&camera, CAMERA_ORBITAL);
            else
                updateFreeCamera(camera, terrainHalf * 0.15f);
        }

        if (rebuildTimer > 0.0f) {
            rebuildTimer -= GetFrameTime();
            if (rebuildTimer <= 0.0f) {
                rebuildTimer = 0.0f;
                renderer.markAllDirty();
                renderer.rebuildDirty(*map);
                prevHeightScale = renderer.heightScale;
                prevCraterScale = renderer.craterScale;
            }
        }

        BeginDrawing();

        ClearBackground({ 199, 170, 125, 255 });

        BeginMode3D(camera);
            renderer.draw();
            if (showGrid)
                renderer.drawGrid(*map);
        EndMode3D();

        float y = panelY + 35;
        GuiPanel({ panelX, panelY, panelW, 370 }, "TerraNav Controls");

        GuiLabel({ panelX + 15, y, 80, 20 }, "Elevation:");
        GuiSlider({ panelX + 100, y, 110, 20 }, NULL, NULL, &renderer.heightScale, 1.0f, 1500.0f);
        if (renderer.heightScale != prevHeightScale) {
            rebuildTimer = 0.5f;
            prevHeightScale = renderer.heightScale;
        }
        y += 30;

        GuiLabel({ panelX + 15, y, 80, 20 }, "Craters:");
        GuiSlider({ panelX + 100, y, 110, 20 }, NULL, NULL, &renderer.craterScale, 0.0f, 1500.0f);
        if (renderer.craterScale != prevCraterScale) {
            rebuildTimer = 0.5f;
            prevCraterScale = renderer.craterScale;
        }
        y += 30;

        GuiLabel({ panelX + 15, y, 80, 20 }, "Render:");
        GuiSlider({ panelX + 100, y, 110, 20 }, NULL, NULL, &renderDistance, 500.0f, 20000.0f);
        rlSetClipPlanes(0.05, renderDistance);
        y += 30;

        GuiCheckBox({ panelX + 15, y, 20, 20 }, "Grid Overlay", &showGrid);
        y += 28;

        GuiCheckBox({ panelX + 15, y, 20, 20 }, "Auto-rotate", &autoRotate);
        y += 28;

        GuiLabel({ panelX + 15, y, 80, 20 }, "Seed:");
        if (GuiSpinner({ panelX + 100, y, 110, 25 }, NULL, &seedValue, 0, INT_MAX, seedEdit))
            seedEdit = !seedEdit;
        y += 35;

        if (GuiButton({ panelX + 15, y, 200, 30 }, "Regenerate")) {
            map = std::make_unique<Map>(static_cast<long long>(seedValue),
                                        REGIONS_X, REGIONS_Y, REGION_W, REGION_H,
                                        NoiseParams{}, craterParams);
            map->generate();
            renderer.rebuildAll(*map);
        }

        int cols = static_cast<int>(map->width_ * map->gen.regionWidth_);
        int rows = static_cast<int>(map->height_ * map->gen.regionHeight_);
        GuiStatusBar({ 0, screenHeight - 25, screenWidth, 25 },
            TextFormat("FPS: %d  |  Verts: %d  |  Tris: %d  |  Rocks: %zu  |  Chunks: %zu  |  %s  |  WASD: move  |  Space/Shift: up/down  |  R: recenter",
                       GetFPS(), cols * rows, (cols - 1) * (rows - 1) * 2,
                       map->rocks.size(), renderer.chunkModels_.size(),
                       uiMode ? "ESC: resume freecam" : "ESC: show UI"));

        EndDrawing();
    }

    renderer.unload();
    CloseWindow();
    return 0;
}
