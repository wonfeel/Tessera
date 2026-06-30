// demo/interactive_demo/main.cpp
//
// Интерактивная Game of Life с экспортом GIF прямо из приложения.
// Общая панель (FPS, поколения, пауза, скорость, камера) — от DefaultApplication.
//
//   LMB              — рисовать клетки  (или тянуть рамку в режиме выбора региона)
//   RMB              — стирать клетки
//   MMB drag         — панорамирование
//   Scroll           — зум
//   WASD             — панорамирование клавиатурой
//   Space            — пауза / продолжение
//
// GIF-рекордер (панель слева): жмёшь "Select region", тянешь мышкой рамку от угла
// до угла по полю — выделенный прямоугольник записывается. Кадры рисуются напрямую
// из состояния клеток (одна клетка = блок пикселей, без размытия).

#include "engine/core/DefaultApplication.h"
#include "engine/core/TaskScheduler.h"
#include "engine/utils/GifExport.h"
#include "engine/utils/RleLoader.h"
#include "demo/life/full/LifeMap.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#ifdef FLORA_IMGUI_ENABLED
#  include <imgui.h>
#endif

class InteractiveApp : public DefaultApplication {
public:
    InteractiveApp()
        : DefaultApplication(
            []() -> std::unique_ptr<ChunkedTileMap> {
                auto map = std::make_unique<LifeMap>(512, 512, 2.0f, 64);
                map->randomize(0.3f);
                return map;
            },
            std::chrono::milliseconds(0),
            1280, 720,
            "FieldEngine — Interactive Life",
            false)
    {
        std::strcpy(m_recPath, "field.gif");
        m_patterns = RleLoader::listFiles("patterns");   // *.rle рядом с exe
    }

protected:
    void onUpdate(float dt) override {
        // --- Scene load (gun + eater) — must mutate the map on this thread ---
        if (m_loadGunEater.exchange(false))
            loadGunEaterScene();

        // --- GIF recording runs synchronously on this (update) thread ---
        if (m_recordRequested.exchange(false))
            doRecord();

        // --- Clear / Randomize: тоже мутируют карту, поэтому только здесь,
        //     в update-потоке (иначе гонка с симуляцией -> частичное применение) ---
        if (m_clearRequested.exchange(false)) {
            m_tileMap->clearAll();          // оба буфера + список активных -> 0
            m_generation = 0;
        }
        if (m_randomizeRequested.exchange(false)) {
            m_tileMap->clearAll();          // сброс, чтобы старое поле не накладывалось
            static_cast<LifeMap*>(m_tileMap.get())->randomize(0.3f);
            m_generation = 0;
        }
        if (m_loadPatternRequested.exchange(false)) {
            int idx = m_selectedPattern.load();
            if (idx >= 0 && idx < static_cast<int>(m_patterns.size()))
                loadPatternScene(m_patterns[idx]);
        }

        DefaultApplication::onUpdate(dt);   // pause logic + simulation step

        float mx, my;
        m_input.getMousePosition(mx, my);
        bool lmb = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
        bool rmb = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT);

        // --- Region selection: drag LMB from corner to corner ---
        if (m_selectMode.load()) {
            if (lmb && !m_imguiWantMouse.load()) {
                glm::ivec2 g = m_tileMap->worldToTile(getCamera().screenToWorld(mx, my));
                int gx = g.x, gy = g.y;
                if (!m_dragging) { m_dragStartX = gx; m_dragStartY = gy; m_dragging = true; }
                m_dragCurX = gx; m_dragCurY = gy;
                // live rectangle for the overlay (normalised)
                m_regX0 = std::min(m_dragStartX, m_dragCurX);
                m_regY0 = std::min(m_dragStartY, m_dragCurY);
                m_regX1 = std::max(m_dragStartX, m_dragCurX);
                m_regY1 = std::max(m_dragStartY, m_dragCurY);
            } else if (m_dragging) {
                // released → finalise
                m_dragging = false;
                if (m_regX1 > m_regX0 && m_regY1 > m_regY0) m_haveRegion = true;
                m_selectMode = false;
            }
            return;  // don't draw cells while selecting
        }

        // Панорамирование средней кнопкой мыши теперь делает CameraController
        // (engine), под мьютексом камеры — здесь ничего не нужно.

        // Cell painting — suppressed when ImGui is active
        if ((lmb || rmb) && !m_imguiWantMouse.load()) {
            glm::ivec2 t = m_tileMap->worldToTile(getCamera().screenToWorld(mx, my));
            m_tileMap->paintBrush(t.x, t.y, m_brushSize.load(), lmb ? 255 : 0);
        }
    }

    void onImGuiExtra() override {
#ifdef FLORA_IMGUI_ENABLED
        // --- Pattern picker ---
        ImGui::Spacing();
        ImGui::TextDisabled("Pattern");
        ImGui::Separator();
        if (m_patterns.empty()) {
            ImGui::TextDisabled("No .rle files in patterns/");
        } else {
            auto nameOf = [](const std::string& p) {
                return std::filesystem::path(p).stem().string();
            };
            int sel = m_selectedPattern.load();
            ImGui::SetNextItemWidth(-1);
            if (ImGui::BeginCombo("##pattern", nameOf(m_patterns[sel]).c_str())) {
                for (int i = 0; i < static_cast<int>(m_patterns.size()); ++i) {
                    bool selected = (i == sel);
                    if (ImGui::Selectable(nameOf(m_patterns[i]).c_str(), selected))
                        m_selectedPattern = i;
                    if (selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            if (ImGui::Button("Load pattern", ImVec2(-1, 0)))
                m_loadPatternRequested = true;
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Drawing");
        ImGui::Separator();
        int brush = m_brushSize.load();
        ImGui::SetNextItemWidth(-1);
        if (ImGui::SliderInt("##brush", &brush, 1, 15)) {
            if (brush % 2 == 0) ++brush;
            m_brushSize = brush;
        }
        ImGui::TextDisabled("Brush %dx%d  LMB draw  RMB erase", brush, brush);
        if (ImGui::Button("Clear", ImVec2(120, 0)))
            m_clearRequested = true;
        ImGui::SameLine();
        if (ImGui::Button("Randomize", ImVec2(120, 0)))
            m_randomizeRequested = true;
        // Загрузить сцену "пушка Госпера + пожиратель глайдеров" — удобно записать в GIF.
        if (ImGui::Button("Load gun + eater", ImVec2(-1, 0)))
            m_loadGunEater = true;

        // --- GIF recorder ---
        ImGui::Spacing();
        ImGui::TextDisabled("GIF recorder");
        ImGui::Separator();

        bool selecting = m_selectMode.load();
        if (ImGui::Button(selecting ? "Drag a box on the field..."
                                    : "Select region", ImVec2(-1, 0)))
            m_selectMode = !selecting;

        if (m_haveRegion.load()) {
            ImGui::Text("Region: (%d,%d)-(%d,%d)  %dx%d cells",
                        m_regX0.load(), m_regY0.load(), m_regX1.load(), m_regY1.load(),
                        m_regX1.load() - m_regX0.load(), m_regY1.load() - m_regY0.load());
        } else {
            ImGui::TextDisabled("No region — click Select, then drag");
        }

        ImGui::SetNextItemWidth(110); ImGui::SliderInt("scale px/cell", &m_recScale, 1, 24);
        ImGui::SameLine(); ImGui::TextDisabled("(bigger = sharper)");
        ImGui::SetNextItemWidth(110); ImGui::SliderInt("frames",        &m_recFrames, 10, 300);
        ImGui::SetNextItemWidth(110); ImGui::SliderInt("stride",        &m_recStride, 1, 10);
        ImGui::SetNextItemWidth(110); ImGui::SliderInt("delay ms",      &m_recDelay, 20, 200);
        ImGui::SetNextItemWidth(-1);  ImGui::InputText("##file", m_recPath, sizeof(m_recPath));

        bool can = m_haveRegion.load() && !m_recording.load();
        if (!can) ImGui::BeginDisabled();
        if (ImGui::Button("Record GIF", ImVec2(-1, 0)))
            m_recordRequested = true;
        if (!can) ImGui::EndDisabled();

        if (m_recording.load())      ImGui::TextColored(ImVec4(1,0.8f,0,1), "Recording...");
        else if (m_status[0])        ImGui::TextWrapped("%s", m_status);

        drawRegionOverlay();
#endif
    }

private:
#ifdef FLORA_IMGUI_ENABLED
    // Draw the selected region as a rectangle over the field (screen space).
    void drawRegionOverlay() {
        if (!m_haveRegion.load() && !m_dragging) return;
        const Camera2D cam = getCamera();
        const float ts = m_tileMap->getTileSize();
        auto toScreen = [&](int gx, int gy) {
            glm::vec2 w(gx * ts, gy * ts);
            glm::vec2 s = (w - cam.position) * cam.zoom;
            return ImVec2(s.x, s.y);
        };
        ImVec2 a = toScreen(m_regX0.load(), m_regY0.load());
        ImVec2 b = toScreen(m_regX1.load(), m_regY1.load());
        ImDrawList* dl = ImGui::GetForegroundDrawList();
        dl->AddRect(a, b, IM_COL32(255, 220, 0, 255), 0.0f, 0, 2.0f);
        dl->AddRectFilled(a, b, IM_COL32(255, 220, 0, 40));
    }
#endif

    // Loads "Gosper gun + glider eater" centred on the field. Runs on the
    // update thread (mutates the map). Eater position (gun+(33,21), rot1) is the
    // clean config found empirically: the glider is absorbed every period.
    void loadGunEaterScene() {
        m_tileMap->clearAll();
        const int gx = 200, gy = 200;
        RlePattern gun = RleLoader::load("patterns/gosper_gun.rle");
        if (gun.width > 0)
            m_tileMap->stampPattern(gun, gx, gy);
        // eater1, rotation 1, base at (gx+33, gy+21)
        const int e[6][2] = { {2,0},{3,0},{3,1},{1,1},{1,2},{0,2} };
        for (auto& c : e)
            m_tileMap->setTile(gx + 33 + c[0], gy + 21 + c[1], 255);
        m_tileMap->commitInitialState();
        m_generation = 0;
        // auto-select this region for recording (a tidy box around gun+eater)
        m_regX0 = gx - 4;  m_regY0 = gy - 4;
        m_regX1 = gx + 48; m_regY1 = gy + 40;
        m_haveRegion = true;
    }

    // Загружает выбранный RLE-паттерн по центру поля. Выполняется в update-потоке.
    void loadPatternScene(const std::string& path) {
        RlePattern pat = RleLoader::load(path);
        if (pat.width <= 0 || pat.height <= 0) {
            std::snprintf(m_status, sizeof(m_status), "Failed to load %s", path.c_str());
            return;
        }
        m_tileMap->clearAll();
        int ox = (m_tileMap->getWidth()  - pat.width)  / 2;
        int oy = (m_tileMap->getHeight() - pat.height) / 2;
        m_tileMap->stampPattern(pat, ox, oy);
        m_tileMap->commitInitialState();
        m_generation = 0;
        // авто-регион вокруг паттерна — удобно сразу записать GIF
        m_regX0 = ox - 4;               m_regY0 = oy - 4;
        m_regX1 = ox + pat.width + 4;   m_regY1 = oy + pat.height + 4;
        m_haveRegion = true;
        std::snprintf(m_status, sizeof(m_status), "Loaded %s (%dx%d)",
                      std::filesystem::path(path).stem().string().c_str(),
                      pat.width, pat.height);
    }

    // Runs on the update thread; blocks until the GIF is written.
    void doRecord() {
        if (!m_haveRegion.load()) return;
        GifExportParams p;
        p.x0 = m_regX0.load(); p.y0 = m_regY0.load();
        p.x1 = m_regX1.load(); p.y1 = m_regY1.load();
        p.scale = m_recScale; p.frames = m_recFrames;
        p.stride = m_recStride; p.delayMs = m_recDelay;
        p.path = m_recPath;

        Camera2D camSnap = getCamera();
        m_recording = true;
        bool ok = ExportGif(*m_tileMap, p, [&]() {
            m_tileMap->simulateAndWait();
            m_tileMap->commitReadyChunks(camSnap);
        });
        m_recording = false;
        std::snprintf(m_status, sizeof(m_status), ok ? "Saved %s" : "Failed: %s",
                      p.path.c_str());
        std::fprintf(stderr, "[interactive] GIF %s -> %s\n",
                     ok ? "saved" : "FAILED", p.path.c_str());
    }

    std::atomic<int> m_brushSize{1};

    // Region selection
    std::atomic<bool> m_selectMode{false};
    std::atomic<bool> m_haveRegion{false};
    bool m_dragging = false;
    int  m_dragStartX = 0, m_dragStartY = 0, m_dragCurX = 0, m_dragCurY = 0;
    std::atomic<int> m_regX0{0}, m_regY0{0}, m_regX1{0}, m_regY1{0};

    // Recorder params (touched in render thread UI, snapshotted on record)
    int  m_recScale = 4, m_recFrames = 80, m_recStride = 2, m_recDelay = 50;
    char m_recPath[256] = {};
    std::atomic<bool> m_recordRequested{false};
    std::atomic<bool> m_loadGunEater{false};
    std::atomic<bool> m_clearRequested{false};
    std::atomic<bool> m_randomizeRequested{false};

    // Выбор паттерна. m_patterns заполняется один раз в конструкторе (read-only),
    // выбор/запрос — атомарные, т.к. UI в рендер-потоке, загрузка в update-потоке.
    std::vector<std::string> m_patterns;
    std::atomic<int>  m_selectedPattern{0};
    std::atomic<bool> m_loadPatternRequested{false};

    char m_status[300] = {};
};

int main() {
    TaskScheduler::instance().initialize();
    InteractiveApp app;
    app.run();
    TaskScheduler::instance().shutdown();
    return 0;
}
