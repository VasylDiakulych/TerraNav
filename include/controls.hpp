#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include <raylib.h>
#include <raygui.h>

#include <cstddef>
#include <climits>

struct Panel {
    float x, y, w;
    float currentY = 0.0f;

    static constexpr float PADDING = 15.0f;
    static constexpr float ROW_H = 30.0f;
    static constexpr float LABEL_W = 80.0f;
    static constexpr float CONTROL_X = PADDING + LABEL_W + 5.0f;
    static constexpr float CONTROL_W = 110.0f;
    static constexpr float CONTROL_H = 20.0f;
    static constexpr float CHECKBOX_H = 28.0f;
    static constexpr float SPINNER_H = 35.0f;
    static constexpr float BUTTON_H = 35.0f;

    void begin(const char* title, float height) {
        GuiPanel({x, y, w, height}, title);
        currentY = y + 35.0f;
    }

    void slider(const char* label, float* value, float min, float max) {
        GuiLabel({x + PADDING, currentY, LABEL_W, CONTROL_H}, label);
        GuiSlider({x + CONTROL_X, currentY, CONTROL_W, CONTROL_H},
                  nullptr, nullptr, value, min, max);
        currentY += ROW_H;
    }

    void checkBox(const char* label, bool* value) {
        GuiCheckBox({x + PADDING, currentY, 20, 20}, label, value);
        currentY += CHECKBOX_H;
    }

    void spinner(const char* label, int* value, int min, int max, bool* edit) {
        GuiLabel({x + PADDING, currentY, LABEL_W, CONTROL_H}, label);
        if (GuiSpinner({x + CONTROL_X, currentY, CONTROL_W, 25},
                       nullptr, value, min, max, *edit))
            *edit = !*edit;
        currentY += SPINNER_H;
    }

    bool button(const char* text) {
        bool clicked = GuiButton({x + PADDING, currentY, 200, 30}, text);
        currentY += BUTTON_H;
        return clicked;
    }
};

struct Controls {
    int seedValue = 42;
    bool seedEdit = false;
    float renderDistance = 8000.0f;
    float prevHeightScale = 75.0f;
    float prevCraterScale = 75.0f;
    float rebuildTimer = 0.0f;
    bool uiMode = false;
    bool showGrid = false;
    bool autoRotate = false;
    bool regenerateClicked = false;

    void handleEscape() {
        if (IsKeyPressed(KEY_ESCAPE)) {
            uiMode = !uiMode;
            if (uiMode) EnableCursor();
            else DisableCursor();
        }
    }

    bool update(float heightScale, float craterScale) {
        if (heightScale != prevHeightScale) {
            rebuildTimer = 0.5f;
            prevHeightScale = heightScale;
        }
        if (craterScale != prevCraterScale) {
            rebuildTimer = 0.5f;
            prevCraterScale = craterScale;
        }

        if (rebuildTimer > 0.0f) {
            rebuildTimer -= GetFrameTime();
            if (rebuildTimer <= 0.0f) {
                rebuildTimer = 0.0f;
                return true;
            }
        }
        return false;
    }

    void draw(float* heightScale, float* craterScale,
              int screenWidth, int screenHeight,
              size_t chunkCount, size_t rockCount, 
              int verts, int tris
    ) {
        static constexpr const char* STATUS_FORMAT =
            "FPS: %d  |  Verts: %d  |  Tris: %d  |  Rocks: %zu  |  Chunks: %zu  |  %s  |  WASD: move  |  Space/Shift: up/down  |  R: recenter";
        static constexpr const char* UI_MODE_FREECAM = "ESC: resume freecam";
        static constexpr const char* UI_MODE_UI = "ESC: show UI";

        Panel panel{10.0f, 10.0f, 230.0f};
        panel.begin("TerraNav Controls", 370.0f);
        panel.slider("Elevation:", heightScale, 1.0f, 300.0f);
        panel.slider("Craters:", craterScale, 0.0f, 300.0f);
        panel.slider("Render:", &renderDistance, 500.0f, 20000.0f);
        panel.checkBox("Grid Overlay", &showGrid);
        panel.checkBox("Auto-rotate", &autoRotate);
        panel.spinner("Seed:", &seedValue, 0, INT_MAX, &seedEdit);
        if (panel.button("Regenerate"))
            regenerateClicked = true;

        GuiStatusBar({0.0f, static_cast<float>(screenHeight - 25),
                      static_cast<float>(screenWidth), 25.0f},
            TextFormat(STATUS_FORMAT, GetFPS(), verts, tris, rockCount, chunkCount,
                       uiMode ? UI_MODE_FREECAM : UI_MODE_UI));
    }

    bool consumeRegenerate() {
        if (regenerateClicked) {
            regenerateClicked = false;
            return true;
        }
        return false;
    }
};

#endif
