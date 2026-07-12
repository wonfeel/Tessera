// demo/light/main.cpp
//
// Свет через скалярное поле: три независимых LightField (R/G/B), каждое -
// скалярное волновое уравнение на фиксированной решётке (почему это честнее
// для света, чем 2D упругая сетка demo/cloth - см. LightField.h). Разная
// waveSpeedSq на R/G/B даёт хроматическую дисперсию; область "призмы"
// (paintMedium/eraseMedium) тормозит каждый цвет по-своему и рисуется
// затемнённой.
//
//   LMB/RMB зависят от выбранного инструмента (см. ToolMode):
//     Pluck - LMB клик: разовый импульс амплитуды во всех трёх полях сразу.
//     Brush - LMB впрыскивает энергию, RMB гасит (во всех трёх полях).
//     Prism - LMB рисует среду (медленнее для более "дисперсионных" цветов),
//             RMB стирает.
//   WASD/scroll/MMB - камера (стандартный CameraController).
//
// Рендер и режим "накопление" (Rendering > Accumulate) взяты из
// Light-Simulation-JS Артёма Онигири - см. demo/light/README.md.

#include "engine/core/Application.h"
#include "engine/core/TaskScheduler.h"
#include "engine/core/ParallelFor.h"
#include "engine/graphics/Shader.h"
#include "demo/light/LightField.h"

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <thread>
#include <vector>

#ifdef TESSERA_IMGUI_ENABLED
#  include <imgui.h>
#endif

namespace {
    constexpr int   kCols = 400, kRows = 400;
    constexpr float kSpacing = 32.0f;
    // Пол в 1px, не 6 - это АБСОЛЮТНЫЙ минимум размера точки независимо от
    // зума (см. Shaders/light_point.vert), не подстройка под удобный размер
    // при приближении - за это отвечает zoomedBase (растёт вместе с zoom).
    // При 6px и поле в 400x400 узлов (16x больше прежнего) на отдалении
    // расстояние между узлами на экране падает ниже 1px, а точки всё равно
    // рисовались минимум 6px - массовое перекрытие на аддитивном блендинге
    // сливалось в сплошной белый цвет с радужной рябью.
    constexpr float kPointBaseSize = 1.0f;
}

// Диагностика зависаний - тот же приём, что в demo/cloth (см. его
// HangWatchdog): lock-free пульсы из update/render-потоков, читаемые без
// удержания мьютекса поля, чтобы диагностика работала даже если step()
// реально завис. lastAvgStretch у LightField нет (нет рёбер) - логируем
// только lastAvgSpeed/lastSubsteps R-поля как представителя всех трёх.
class HangWatchdog {
public:
    void start(const LightField* field) {
        m_field = field;
        m_running = true;
        m_thread = std::thread([this] { run(); });
    }
    void stop() {
        m_running = false;
        if (m_thread.joinable()) m_thread.join();
    }
    void tickUpdate() { m_updateTick.fetch_add(1, std::memory_order_relaxed); }
    void tickRender() { m_renderTick.fetch_add(1, std::memory_order_relaxed); }

private:
    void run() {
        uint64_t lastUpdate = 0, lastRender = 0;
        int stallStreak = 0;
        while (m_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            uint64_t curUpdate = m_updateTick.load(std::memory_order_relaxed);
            uint64_t curRender = m_renderTick.load(std::memory_order_relaxed);
            bool updateStuck = (curUpdate == lastUpdate);
            bool renderStuck = (curRender == lastRender);
            if (updateStuck || renderStuck) {
                if (++stallStreak >= 2)
                    logStall(stallStreak, updateStuck, curUpdate, renderStuck, curRender);
            } else {
                stallStreak = 0;
            }
            lastUpdate = curUpdate;
            lastRender = curRender;
        }
    }

    void logStall(int streak, bool updateStuck, uint64_t curUpdate,
                  bool renderStuck, uint64_t curRender) const {
        FILE* f = std::fopen("light_watchdog.log", "a");
        if (!f) return;
        std::fprintf(f,
            "[watchdog] stall#%d update=%s(%llu) render=%s(%llu) avgSpeed(R)=%.4f substeps(R)=%d\n",
            streak,
            updateStuck ? "STUCK" : "ok", (unsigned long long)curUpdate,
            renderStuck ? "STUCK" : "ok", (unsigned long long)curRender,
            m_field->lastAvgSpeed(), m_field->lastSubsteps());
        std::fclose(f);
    }

    const LightField* m_field = nullptr;
    std::atomic<uint64_t> m_updateTick{0};
    std::atomic<uint64_t> m_renderTick{0};
    std::atomic<bool> m_running{false};
    std::thread m_thread;
};

// Пишутся из рендер-потока (ImGui-виджеты), читаются из update-потока
// (step()) - поэтому atomic, как и в demo/cloth.
struct PhysicsParams {
    std::atomic<float> waveSpeedSq{80.0f};
    std::atomic<float> dampingRate{4.0f};
    std::atomic<float> pluckStrength{40.0f};
    std::atomic<float> timeScale{1.0f};
    // Множители скорости на цвет - база RGB-дисперсии (разные скорости волны).
    std::atomic<float> redSpeedMul{0.92f};
    std::atomic<float> greenSpeedMul{1.0f};
    std::atomic<float> blueSpeedMul{1.10f};
    // Насколько СИЛЬНО именно этот цвет тормозится в области призмы (0..1) -
    // синий тормозится сильнее, как в реальном стекле (bluе bends more).
    std::atomic<float> redDispersion{0.25f};
    std::atomic<float> greenDispersion{0.40f};
    std::atomic<float> blueDispersion{0.55f};
};

struct BrushSettings {
    std::atomic<float> radius{80.0f};
    std::atomic<float> strength{400.0f};
};

struct PrismSettings {
    std::atomic<float> radius{100.0f};
    std::atomic<float> strength{2.0f};   // скорость насыщения маски (в 1/сек)
    std::atomic<float> darken{0.7f};     // насколько темнее рисуется область призмы на экране
};

// Направленный пучок - см. LightField::beam(). aperture шире = у́же и
// направленнее пучок (дифракционный предел), frequency - Гц колебания
// излучателей (выше частота = короче волна при том же waveSpeedSq).
struct BeamSettings {
    std::atomic<float> aperture{250.0f};
    std::atomic<float> frequency{2.0f};
    std::atomic<float> strength{600.0f};
};

enum class ToolMode { Pluck, Brush, Prism, Beam };

// Готовые карты (Maps в ImGui) - форма призмы задана заранее многоугольником
// вместо рисования мышью, см. LightApp::applyMapPreset().
enum class MapPreset { PrismSmall, PrismLarge };

struct RenderSettings {
    std::atomic<bool> showPoints{true};
    std::atomic<bool> showCubes{false};
    // Долгая выдержка вместо затухающего glow - см. LightField::m_accum и
    // demo/light/README.md (идея из Light-Simulation-JS). Монотонно растёт,
    // не гаснет сама, поэтому есть отдельная кнопка сброса.
    std::atomic<bool> accumulate{true};
};

class LightApp : public Application {
public:
    LightApp()
        : Application(1280, 720, "Tessera - Light", false)
        , m_fieldR(std::make_unique<LightField>(kCols, kRows, kSpacing))
        , m_fieldG(std::make_unique<LightField>(kCols, kRows, kSpacing))
        , m_fieldB(std::make_unique<LightField>(kCols, kRows, kSpacing))
    {}

protected:
    void onInit() override {
        TaskScheduler::instance().initialize();
        initShaders();
        initCamera();
        m_watchdog.start(m_fieldR.get());
    }

    void onUpdate(float dt) override {
        m_watchdog.tickUpdate();
        dt = std::min(dt, 0.033f);

        handlePointerInput(dt);
        stepPhysics(dt);
    }

    void onRender(const Camera2D& camera) override {
        m_watchdog.tickRender();

        m_fieldR->snapshot(m_glowR, m_maskR, m_accumR);
        m_fieldG->snapshot(m_glowG, m_maskG, m_accumG);
        m_fieldB->snapshot(m_glowB, m_maskB, m_accumB);

        float dim = m_baseZoom > 0.0f
            ? std::clamp(camera.zoom / m_baseZoom, 0.12f, 1.0f)
            : 1.0f;

        if (dim > 0.2f) {
            if (m_render.showPoints.load())
                renderPointLayer(dim, /*cubeMode=*/false, camera);
            if (m_render.showCubes.load())
                renderPointLayer(dim, /*cubeMode=*/true, camera);
        }

        glBindVertexArray(0);
    }

    void onImGui() override {
#ifdef TESSERA_IMGUI_ENABLED
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Light", nullptr, ImGuiWindowFlags_NoCollapse);

        drawStatus();
        drawMapControls();
        drawTimeControls();
        drawPhysicsControls();
        drawInteractionControls();
        drawRenderingControls();

        ImGui::End();
#endif
    }

    void onDestroy() override {
        m_watchdog.stop();
        m_pointShader.reset();
        if (m_pointVAO) glDeleteVertexArrays(1, &m_pointVAO);
        if (m_pointVBO) glDeleteBuffers(1, &m_pointVBO);
        TaskScheduler::instance().shutdown();
    }

private:
    // ---------- onInit ----------

    void initShaders() {
        m_pointShader = std::make_unique<Shader>("Shaders/light_point.vert", "Shaders/light_point.frag");

        glGenVertexArrays(1, &m_pointVAO);
        glGenBuffers(1, &m_pointVBO);
        glBindVertexArray(m_pointVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_pointVBO);
        // vec2 позиция + vec3 цвет (R/G/B яркость посчитана на CPU, см.
        // renderPointLayer) - в отличие от demo/cloth, где был один скаляр.
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);

        glEnable(GL_PROGRAM_POINT_SIZE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glClearColor(0.03f, 0.03f, 0.05f, 1.0f);
    }

    void initCamera() {
        // Границы поля через LightField::worldPos() - гекс-раскладка имеет
        // сдвиг по x на нечётных строках, поэтому "kCols*kSpacing" (как было
        // в квадратной версии) больше не точная граница; берём фактическую
        // позицию последнего узла.
        glm::vec2 worldMin(0.0f, 0.0f);
        glm::vec2 worldMax = m_fieldR->worldPos(kCols - 1, kRows - 1);
        frameCamera(worldMin, worldMax, kSpacing * 2.0f);
        m_baseZoom = getCamera().zoom;
    }

    // ---------- onUpdate ----------

    void handlePointerInput(float dt) {
        float mx, my;
        m_input.getMousePosition(mx, my);
        glm::vec2 mouseWorld = getCamera().screenToWorld(mx, my);

        bool lmb = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT) && !m_imguiWantMouse.load();
        bool rmb = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT) && !m_imguiWantMouse.load();

        switch (m_tool.load()) {
            case ToolMode::Pluck:
                if (lmb && !m_lmbPrev)
                    pluckAll(mouseWorld, m_physics.pluckStrength.load());
                break;
            case ToolMode::Brush:
                if (lmb) brushAll(mouseWorld, m_brush.radius.load(), m_brush.strength.load(), dt);
                if (rmb) brushDampAll(mouseWorld, m_brush.radius.load(), m_brush.strength.load(), dt);
                break;
            case ToolMode::Prism:
                if (lmb) paintMediumAll(mouseWorld, m_prism.radius.load(), m_prism.strength.load(), dt);
                if (rmb) eraseMediumAll(mouseWorld, m_prism.radius.load(), m_prism.strength.load(), dt);
                break;
            case ToolMode::Beam:
                // LMB press фиксирует origin. Направление фиксируется один раз,
                // как только увод мыши превышает kAimThresholdPx - в ЭКРАННЫХ
                // пикселях, не мировых единицах. Раньше порог был в мировых
                // единицах (20) - после увеличения поля в 4 раза камера
                // зумируется намного дальше, чтобы вместить всё поле, и те же
                // 20 мировых единиц стали меньше одного экранного пикселя:
                // направление фиксировалось буквально на дрожании руки при
                // самом первом движении мыши после нажатия, то есть почти
                // случайно каждый раз - отсюда крест из нескольких лучей в
                // случайных направлениях на скрине.
                if (lmb) {
                    if (!m_beamHeld) {
                        m_beamOrigin = mouseWorld;
                        m_beamOriginScreen = glm::vec2(mx, my);
                        m_beamHeld = true;
                        m_beamDirLocked = false;
                        m_beamTime = 0.0f;
                    }
                    constexpr float kAimThresholdPx = 15.0f;
                    if (!m_beamDirLocked) {
                        glm::vec2 dScreen = glm::vec2(mx, my) - m_beamOriginScreen;
                        if (glm::length(dScreen) > kAimThresholdPx) {
                            m_beamDir = mouseWorld - m_beamOrigin;
                            m_beamDirLocked = true;
                        }
                    }
                    if (m_beamDirLocked) {
                        m_beamTime += dt;
                        beamAll(m_beamOrigin, m_beamDir, m_beam.aperture.load(), m_beam.frequency.load(),
                               m_beam.strength.load(), m_beamTime, dt);
                    }
                } else {
                    m_beamHeld = false;
                }
                break;
        }

        m_lmbPrev = lmb;
    }

    // Щипок/кисть/призма применяются ко всем трём полям ОДНИМ и тем же
    // мировым событием - RGB-волна рождается из одного и того же импульса,
    // а призма - одна и та же физическая область "стекла" (разный ЭФФЕКТ на
    // цвет считается уже внутри LightField::step() через per-color dispersion).
    void pluckAll(glm::vec2 pos, float strength) {
        m_fieldR->pluck(pos, strength);
        m_fieldG->pluck(pos, strength);
        m_fieldB->pluck(pos, strength);
    }
    void brushAll(glm::vec2 pos, float radius, float strength, float dt) {
        m_fieldR->brush(pos, radius, strength, dt);
        m_fieldG->brush(pos, radius, strength, dt);
        m_fieldB->brush(pos, radius, strength, dt);
    }
    void brushDampAll(glm::vec2 pos, float radius, float strength, float dt) {
        m_fieldR->brushDamp(pos, radius, strength, dt);
        m_fieldG->brushDamp(pos, radius, strength, dt);
        m_fieldB->brushDamp(pos, radius, strength, dt);
    }
    void paintMediumAll(glm::vec2 pos, float radius, float strength, float dt) {
        m_fieldR->paintMedium(pos, radius, strength, dt);
        m_fieldG->paintMedium(pos, radius, strength, dt);
        m_fieldB->paintMedium(pos, radius, strength, dt);
    }
    void eraseMediumAll(glm::vec2 pos, float radius, float strength, float dt) {
        m_fieldR->eraseMedium(pos, radius, strength, dt);
        m_fieldG->eraseMedium(pos, radius, strength, dt);
        m_fieldB->eraseMedium(pos, radius, strength, dt);
    }
    void beamAll(glm::vec2 origin, glm::vec2 dir, float aperture, float frequency, float strength, float time, float dt) {
        m_fieldR->beam(origin, dir, aperture, frequency, strength, time, dt);
        m_fieldG->beam(origin, dir, aperture, frequency, strength, time, dt);
        m_fieldB->beam(origin, dir, aperture, frequency, strength, time, dt);
    }

    // Равносторонний треугольник, вершиной вверх (силуэт "призмы") -
    // классическая ориентация для картинки в учебниках по дисперсии.
    static std::vector<glm::vec2> trianglePrism(glm::vec2 center, float halfSize) {
        return {
            {center.x, center.y - halfSize},
            {center.x + halfSize * 0.866f, center.y + halfSize * 0.5f},
            {center.x - halfSize * 0.866f, center.y + halfSize * 0.5f},
        };
    }

    // Сбрасывает волну/среду и заливает поле готовой формой призмы -
    // применяется по кнопке (Maps в ImGui), не рисуется мышью.
    void applyMapPreset(MapPreset preset) {
        m_fieldR->reset();
        m_fieldG->reset();
        m_fieldB->reset();

        glm::vec2 center = m_fieldR->worldPos(kCols / 2, kRows / 2);
        glm::vec2 extent = m_fieldR->worldPos(kCols - 1, kRows - 1);
        float minSpan = std::min(extent.x, extent.y);

        std::vector<glm::vec2> polygon;
        switch (preset) {
            case MapPreset::PrismSmall: polygon = trianglePrism(center, minSpan * 0.12f); break;
            case MapPreset::PrismLarge: polygon = trianglePrism(center, minSpan * 0.28f); break;
        }

        m_fieldR->paintMediumPolygon(polygon, 1.0f);
        m_fieldG->paintMediumPolygon(polygon, 1.0f);
        m_fieldB->paintMediumPolygon(polygon, 1.0f);
    }

    // Троттлинг частоты физики отдельно от кадров - та же причина, что в
    // demo/cloth (см. его stepPhysics()): TaskScheduler не должен захлёбываться
    // тысячами вызовов step() в секунду на маленькой сетке.
    void stepPhysics(float dt) {
        constexpr float kMaxStepHz = 120.0f;
        constexpr float kMinStepInterval = 1.0f / kMaxStepHz;
        constexpr float kMaxStableDt = 1.0f / 50.0f;

        m_timeSinceLastStep += dt;
        if (m_timeSinceLastStep >= kMinStepInterval) {
            float scaledDt = std::min(m_timeSinceLastStep * m_physics.timeScale.load(), kMaxStableDt);
            float base = m_physics.waveSpeedSq.load();
            float damping = m_physics.dampingRate.load();
            m_fieldR->step(scaledDt, base * m_physics.redSpeedMul.load(), damping, m_physics.redDispersion.load());
            m_fieldG->step(scaledDt, base * m_physics.greenSpeedMul.load(), damping, m_physics.greenDispersion.load());
            m_fieldB->step(scaledDt, base * m_physics.blueSpeedMul.load(), damping, m_physics.blueDispersion.load());
            m_timeSinceLastStep = 0.0f;
        }
    }

    // ---------- onRender ----------

    static constexpr float kCullMarginCells = 4.0f;

    void visibleRowRange(const Camera2D& camera, int& rowMin, int& rowMaxEx) const {
        glm::vec2 mn, mx;
        camera.getVisibleAABB(mn, mx);
        float vert = m_fieldR->hexVertSpacing();
        int rows = m_fieldR->rows();
        rowMin = std::clamp(static_cast<int>(std::floor(mn.y / vert - kCullMarginCells)), 0, rows - 1);
        int rowMax = static_cast<int>(std::ceil(mx.y / vert + kCullMarginCells));
        rowMaxEx = std::clamp(rowMax + 1, rowMin + 1, rows);
    }

    void visibleColRange(const Camera2D& camera, int& colMin, int& colMaxEx) const {
        glm::vec2 mn, mx;
        camera.getVisibleAABB(mn, mx);
        float horiz = m_fieldR->hexHorizSpacing();
        int cols = m_fieldR->cols();
        // Строки сдвинуты по x на полклетки (см. LightField::worldPos()) -
        // запас в 1 доп. клетку с каждой стороны (а не только kCullMarginCells)
        // покрывает этот сдвиг, чтобы отбраковка не срезала краевой столбец
        // на нечётных строках.
        colMin = std::clamp(static_cast<int>(std::floor(mn.x / horiz - kCullMarginCells - 1.0f)), 0, cols - 1);
        int colMax = static_cast<int>(std::ceil(mx.x / horiz + kCullMarginCells + 1.0f));
        colMaxEx = std::clamp(colMax + 1, colMin + 1, cols);
    }

    // Один слой точек/кубиков - узлы не двигаются (нет позиции как степени
    // свободы, см. LightField.h), координата узла - LightField::worldPos()
    // (гекс-раскладка, не просто col*spacing/row*spacing).
    void renderPointLayer(float dim, bool cubeMode, const Camera2D& camera) {
        int cols = m_fieldR->cols();
        int rowMin, rowMaxEx, colMin, colMaxEx;
        visibleRowRange(camera, rowMin, rowMaxEx);
        visibleColRange(camera, colMin, colMaxEx);
        int visRows = rowMaxEx - rowMin;
        int visCols = colMaxEx - colMin;
        int visCount = visRows * visCols;
        if (visCount <= 0) return;

        m_pointVertexData.resize(static_cast<size_t>(visCount) * 5);
        const float darken = m_prism.darken.load();
        const bool accumulate = m_render.accumulate.load();

        // Не-накопительный режим - наша собственная добавка поверх
        // референса (Light-Simulation-JS всегда только накапливает, "живого"
        // просмотра у него нет), тут остаётся старая схема: аддитивный
        // блендинг требует постоянного kAmbient (иначе цвет (0,0,0) при
        // glow=0 ничего не добавляет к кадру и поле не видно в покое),
        // затемнение призмы через shade и отдельный аддитивный оттенок.
        constexpr float kAmbientR = 0.18f, kAmbientG = 0.28f, kAmbientB = 0.45f;
        constexpr float kPrismTintR = 0.22f, kPrismTintG = 0.10f, kPrismTintB = 0.02f;

        // Накопительный режим - точно та же формула, что colorValue в
        // logic.js Light-Simulation-JS: color = min(accum,1)^2, и для
        // "стекла" ПРОСТО ПРИБАВЛЯЕТСЯ фиксированный оттенок (их
        // GLASS_COLORS=[50,60,70] из 255), без затемнения/shade - там нет ни
        // ambient, ни darken вообще, кадр перерисовывается с нуля каждый раз
        // (см. demo/light/README.md). kGlassTint масштабируется по mask
        // (0..1), а не бинарно, как их pixelMass<1 - у нас медиум рисуется
        // градиентом, не двоичным флагом.
        constexpr float kGlassTintR = 50.0f / 255.0f, kGlassTintG = 60.0f / 255.0f, kGlassTintB = 70.0f / 255.0f;

        parallelFor(visCount, [&](int begin, int end, int) {
            for (int idx = begin; idx < end; ++idx) {
                int rr = idx / visCols;
                int cc = idx - rr * visCols;
                int col = colMin + cc, row = rowMin + rr;
                size_t i = static_cast<size_t>(row * cols + col);
                float mask = m_maskR[i];
                glm::vec2 p = m_fieldR->worldPos(col, row);
                size_t base = static_cast<size_t>(idx) * 5;
                m_pointVertexData[base + 0] = p.x;
                m_pointVertexData[base + 1] = p.y;
                if (accumulate) {
                    float aR = std::clamp(m_accumR[i], 0.0f, 1.0f);
                    float aG = std::clamp(m_accumG[i], 0.0f, 1.0f);
                    float aB = std::clamp(m_accumB[i], 0.0f, 1.0f);
                    m_pointVertexData[base + 2] = std::min(aR * aR + kGlassTintR * mask, 1.0f);
                    m_pointVertexData[base + 3] = std::min(aG * aG + kGlassTintG * mask, 1.0f);
                    m_pointVertexData[base + 4] = std::min(aB * aB + kGlassTintB * mask, 1.0f);
                } else {
                    // Пол в 0.15 - Maps заливает mask сразу в 1.0 по всему
                    // многоугольнику (не градиентом, как ручная кисть), и на
                    // darken=0.7 полное затемнение читалось как чёрная дыра.
                    float shade = std::max(1.0f - darken * mask, 0.15f);
                    m_pointVertexData[base + 2] = (kAmbientR + m_glowR[i]) * shade + kPrismTintR * mask;
                    m_pointVertexData[base + 3] = (kAmbientG + m_glowG[i]) * shade + kPrismTintG * mask;
                    m_pointVertexData[base + 4] = (kAmbientB + m_glowB[i]) * shade + kPrismTintB * mask;
                }
            }
        });
        glBindBuffer(GL_ARRAY_BUFFER, m_pointVBO);
        glBufferData(GL_ARRAY_BUFFER, m_pointVertexData.size() * sizeof(float),
                     m_pointVertexData.data(), GL_STREAM_DRAW);

        m_pointShader->use();
        m_pointShader->setMat4("uCamera", camera.getViewProjectionMatrix());
        m_pointShader->setFloat("uBaseSize", kPointBaseSize);
        m_pointShader->setFloat("uDim", dim);
        m_pointShader->setInt("uCubeMode", cubeMode ? 1 : 0);
        // +1px нахлёст - см. то же пояснение в demo/cloth/main.cpp.
        // hexHorizSpacing() - фактическое расстояние между соседними узлами
        // (то же самое по всем 6 направлениям на честном гексе), не голый
        // spacing() (радиус гекса, меньше реального шага в sqrt(3) раз).
        m_pointShader->setFloat("uCellSizePx", m_fieldR->hexHorizSpacing() * camera.zoom + 1.0f);
        glBindVertexArray(m_pointVAO);
        glDrawArrays(GL_POINTS, 0, visCount);
    }

    // ---------- onImGui ----------
#ifdef TESSERA_IMGUI_ENABLED
    static void sliderWithInput(const char* label, std::atomic<float>& value,
                                 float lo, float hi, const char* fmt) {
        ImGui::PushID(label);
        float v = value.load();
        ImGui::SetNextItemWidth(120);
        bool changedSlider = ImGui::SliderFloat("##s", &v, lo, hi, fmt);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        bool changedInput = ImGui::InputFloat("##i", &v, 0.0f, 0.0f, fmt);
        ImGui::SameLine();
        ImGui::TextUnformatted(label);
        if (changedSlider) value = std::clamp(v, lo, hi);
        else if (changedInput) value = v;
        ImGui::PopID();
    }

    void drawStatus() {
        ImGui::Text("FPS: %d", m_fps.load());
        switch (m_tool.load()) {
            case ToolMode::Pluck: ImGui::TextWrapped("LMB click - pluck (impulse)."); break;
            case ToolMode::Brush: ImGui::TextWrapped("LMB hold - inject energy. RMB hold - damp it."); break;
            case ToolMode::Prism: ImGui::TextWrapped("LMB hold - paint prism. RMB hold - erase it."); break;
            case ToolMode::Beam: ImGui::TextWrapped("LMB press+drag - aim beam (fires while held)."); break;
        }
    }

    void drawTimeControls() {
        ImGui::Spacing();
        ImGui::TextDisabled("Time");
        ImGui::Separator();
        sliderWithInput("Time scale", m_physics.timeScale, 0.0f, 10.0f, "%.2f");
        if (m_physics.timeScale.load() == 0.0f)
            ImGui::TextDisabled("Paused");
    }

    void drawPhysicsControls() {
        ImGui::Spacing();
        ImGui::TextDisabled("Physics");
        ImGui::Separator();
        sliderWithInput("Wave speed^2", m_physics.waveSpeedSq, 1.0f, 400.0f, "%.0f");
        sliderWithInput("Damping rate", m_physics.dampingRate, 0.0f, 15.0f, "%.1f");
        sliderWithInput("Pluck strength", m_physics.pluckStrength, 0.0f, 200.0f, "%.0f");

        ImGui::Spacing();
        ImGui::TextDisabled("Per-color speed (chromatic base)");
        sliderWithInput("Red mul",   m_physics.redSpeedMul,   0.5f, 1.5f, "%.2f");
        sliderWithInput("Green mul", m_physics.greenSpeedMul, 0.5f, 1.5f, "%.2f");
        sliderWithInput("Blue mul",  m_physics.blueSpeedMul,  0.5f, 1.5f, "%.2f");

        ImGui::Spacing();
        ImGui::TextDisabled("Per-color dispersion in prism");
        sliderWithInput("Red disp",   m_physics.redDispersion,   0.0f, 1.0f, "%.2f");
        sliderWithInput("Green disp", m_physics.greenDispersion, 0.0f, 1.0f, "%.2f");
        sliderWithInput("Blue disp",  m_physics.blueDispersion,  0.0f, 1.0f, "%.2f");
    }

    void drawInteractionControls() {
        ImGui::Spacing();
        ImGui::TextDisabled("Tool");
        ImGui::Separator();

        int tool = static_cast<int>(m_tool.load());
        if (ImGui::RadioButton("Pluck", tool == 0)) m_tool = ToolMode::Pluck;
        ImGui::SameLine();
        if (ImGui::RadioButton("Brush", tool == 1)) m_tool = ToolMode::Brush;
        ImGui::SameLine();
        if (ImGui::RadioButton("Prism", tool == 2)) m_tool = ToolMode::Prism;
        ImGui::SameLine();
        if (ImGui::RadioButton("Beam", tool == 3)) m_tool = ToolMode::Beam;

        ImGui::Spacing();
        ImGui::TextDisabled("Brush");
        sliderWithInput("Brush radius", m_brush.radius, 10.0f, 300.0f, "%.0f");
        sliderWithInput("Brush strength", m_brush.strength, 0.0f, 1000.0f, "%.0f");

        ImGui::Spacing();
        ImGui::TextDisabled("Prism");
        sliderWithInput("Prism radius", m_prism.radius, 10.0f, 300.0f, "%.0f");
        sliderWithInput("Prism strength", m_prism.strength, 0.1f, 10.0f, "%.2f");
        sliderWithInput("Prism darken", m_prism.darken, 0.0f, 1.0f, "%.2f");
        ImGui::TextWrapped("Darken only applies with Accumulate off - the "
                           "accumulate view tints glass like Light-Simulation-JS "
                           "does (flat add, no darkening).");

        ImGui::Spacing();
        ImGui::TextDisabled("Beam");
        sliderWithInput("Beam aperture", m_beam.aperture, 20.0f, 800.0f, "%.0f");
        sliderWithInput("Beam frequency", m_beam.frequency, 0.2f, 10.0f, "%.2f");
        sliderWithInput("Beam strength", m_beam.strength, 0.0f, 2000.0f, "%.0f");
        ImGui::TextWrapped("Wider aperture = narrower, more directional beam "
                           "(diffraction) - it's real physics, not a slider trick.");

        if (ImGui::Button("Reset field", ImVec2(-1, 0))) {
            m_fieldR->reset();
            m_fieldG->reset();
            m_fieldB->reset();
        }
    }

    void drawMapControls() {
        ImGui::Spacing();
        ImGui::TextDisabled("Maps");
        ImGui::Separator();
        if (ImGui::Button("Prism (small)", ImVec2(-1, 0))) applyMapPreset(MapPreset::PrismSmall);
        if (ImGui::Button("Prism (large)", ImVec2(-1, 0))) applyMapPreset(MapPreset::PrismLarge);
    }

    void drawRenderingControls() {
        ImGui::Spacing();
        ImGui::TextDisabled("Rendering");
        ImGui::Separator();

        bool showPoints = m_render.showPoints.load();
        if (ImGui::Checkbox("Points", &showPoints))
            m_render.showPoints = showPoints;

        bool showCubes = m_render.showCubes.load();
        if (ImGui::Checkbox("Cubes", &showCubes))
            m_render.showCubes = showCubes;

        bool accumulate = m_render.accumulate.load();
        if (ImGui::Checkbox("Accumulate (long exposure)", &accumulate))
            m_render.accumulate = accumulate;
        ImGui::TextWrapped("Idea + accumulation renderer from Artem Onigiri's "
                           "Light-Simulation-JS, see demo/light/README.md.");
        if (accumulate && ImGui::Button("Reset accumulation", ImVec2(-1, 0))) {
            m_fieldR->resetAccumulation();
            m_fieldG->resetAccumulation();
            m_fieldB->resetAccumulation();
        }
    }
#endif

    std::unique_ptr<LightField> m_fieldR, m_fieldG, m_fieldB;
    std::unique_ptr<Shader> m_pointShader;
    unsigned int m_pointVAO = 0, m_pointVBO = 0;

    std::vector<float> m_pointVertexData;
    std::vector<float> m_glowR, m_glowG, m_glowB;
    std::vector<float> m_maskR, m_maskG, m_maskB;
    std::vector<float> m_accumR, m_accumG, m_accumB;

    float m_baseZoom = 1.0f;
    float m_timeSinceLastStep = 0.0f;

    PhysicsParams m_physics;
    BrushSettings m_brush;
    PrismSettings m_prism;
    BeamSettings m_beam;
    RenderSettings m_render;
    std::atomic<ToolMode> m_tool{ToolMode::Pluck};

    bool m_lmbPrev = false;
    // Состояние прицеливания лучом - трогается только из update-потока (см.
    // handlePointerInput), atomics не нужны.
    glm::vec2 m_beamOrigin{0.0f, 0.0f};
    glm::vec2 m_beamOriginScreen{0.0f, 0.0f};   // экранные пиксели - см. handlePointerInput
    glm::vec2 m_beamDir{1.0f, 0.0f};
    bool m_beamHeld = false;
    bool m_beamDirLocked = false;
    float m_beamTime = 0.0f;

    HangWatchdog m_watchdog;
};

int main() {
    try {
        LightApp app;
        app.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[light] fatal: %s\n", e.what());
        return 1;
    }
    return 0;
}
