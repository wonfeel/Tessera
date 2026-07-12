// demo/cloth/main.cpp
//
// "Свет через пружины": сетка масс на пружинах (Гук + явное интегрирование),
// возбуждение расходится по сетке затухающей волной. Рендерится не отдельным
// шейдером освещения, а прямо энергией физики — скорость узла/растяжение
// ребра это и есть яркость (аддитивный блендинг поверх тёмного фона).
//
// Time scale масштабирует dt, который передаётся в SpringNetwork::step() —
// замедление времени уменьшает величину смещения за шаг, а не частоту его
// вызова (step() всегда ровно один раз за кадр).
//
// Физика внутри step() параллелится через TaskScheduler (см. SpringNetwork.cpp) —
// на большой сетке (десятки-сотни тысяч узлов, как здесь) это уже не косметика:
// в один поток на 64000 узлов / ~380000 рёбер каждый кадр сам по себе заметен.
// CUDA-бэкенда у этой демки нет — грид (force-per-edge -> scatter в оба конца
// ребра -> gather по узлу) не переиспользует ISimulationBackend клеточного
// автомата, там своя топология; отдельный .cu под force-based spring-solver
// не писали, TaskScheduler уже даёт нужный запас на разумных размерах сетки.
//
// Не использует ChunkedTileMap/DefaultApplication — это другая физика
// (непрерывная, не клеточный автомат), поэтому строится прямо на Application,
// с собственным маленьким GL-рендером (см. docs/new-demo.html, раздел про
// демки без общего тайлмапа).
//
//   LMB drag   — схватить узел и тащить (непрерывно возбуждает сетку)
//   RMB click  — щипок: разовый импульс в ближайший узел
//   Brush mode — альтернатива drag/pluck: LMB впрыскивает энергию, RMB гасит
//   WASD/scroll/MMB — камера (стандартный CameraController)

#include "engine/core/Application.h"
#include "engine/core/TaskScheduler.h"
#include "engine/graphics/Shader.h"
#include "demo/cloth/SpringNetwork.h"
#include "engine/core/ParallelFor.h"

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
    constexpr int   kCols = 100, kRows = 100;
    constexpr float kSpacing = 32.0f;
    constexpr float kPointBaseSize = 6.0f;
}

// Диагностика зависаний (см. историю расследования дедлока в TaskScheduler):
// два atomic-пульса, дёргаемые из update/render-потоков, проверяются раз в
// 500мс; если хоть один не сдвинулся 2 проверки подряд (~1с), пишет строку в
// springs_watchdog.log вместе с lock-free диагностикой физики
// (lastAvgStretch/lastAvgSpeed/lastSubsteps — читаются без m_mutex, поэтому
// доступны, даже если сам SpringNetwork::step() завис, держа лок).
class HangWatchdog {
public:
    void start(const SpringNetwork* network) {
        m_network = network;
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
        FILE* f = std::fopen("springs_watchdog.log", "a");
        if (!f) return;
        std::fprintf(f,
            "[watchdog] stall#%d update=%s(%llu) render=%s(%llu) "
            "avgStretch=%.4f avgSpeed=%.4f substeps=%d\n",
            streak,
            updateStuck ? "STUCK" : "ok", (unsigned long long)curUpdate,
            renderStuck ? "STUCK" : "ok", (unsigned long long)curRender,
            m_network->lastAvgStretch(), m_network->lastAvgSpeed(), m_network->lastSubsteps());
        std::fclose(f);
    }

    const SpringNetwork* m_network = nullptr;
    std::atomic<uint64_t> m_updateTick{0};
    std::atomic<uint64_t> m_renderTick{0};
    std::atomic<bool> m_running{false};
    std::thread m_thread;
};

// Параметры физики из ImGui — пишутся из рендер-потока (виджеты), читаются
// из update-потока (step()), поэтому atomic.
struct PhysicsParams {
    std::atomic<float> stiffness{80.0f};
    std::atomic<float> dampingRate{4.0f};
    std::atomic<float> pluckStrength{60.0f};
    std::atomic<float> timeScale{1.0f};
};

// Кисть — альтернатива drag/pluck, см. SpringLightApp::handleBrushInput().
struct BrushSettings {
    std::atomic<bool> enabled{false};
    std::atomic<float> radius{80.0f};
    std::atomic<float> strength{400.0f};
};

// Что рисовать. Grid/Points/Cubes — три независимых слоя (не режимы одного
// переключателя, любая комбинация допустима, см. onRender()). showStretch —
// общий источник яркости для Points И Cubes (не по отдельности): без Grid
// натяжение раньше нигде не было видно, т.к. они всегда показывали только
// скорость узла.
struct RenderSettings {
    std::atomic<bool> showGrid{true};
    std::atomic<bool> showPoints{true};
    std::atomic<bool> showCubes{false};
    std::atomic<bool> showStretch{false};
};

class SpringLightApp : public Application {
public:
    SpringLightApp()
        : Application(1280, 720, "Tessera — Light through Springs", false)
        , m_network(std::make_unique<SpringNetwork>(kCols, kRows, kSpacing))
    {}

protected:
    void onInit() override {
        TaskScheduler::instance().initialize();
        initShaders();
        initCamera();
        cacheTopology();
        m_watchdog.start(m_network.get());
    }

    void onUpdate(float dt) override {
        m_watchdog.tickUpdate();
        dt = std::min(dt, 0.033f);   // не даём сетке "взорваться" на подвисании кадра

        handlePointerInput(dt);
        stepPhysics(dt);
    }

    void onRender(const Camera2D& camera) override {
        m_watchdog.tickRender();

        std::vector<glm::vec2> pos;
        std::vector<float> nodeGlow, nodeStretchGlow, edgeGlow;
        m_network->snapshot(pos, nodeGlow, nodeStretchGlow, edgeGlow);
        const std::vector<float>& displayGlow = m_render.showStretch.load() ? nodeStretchGlow : nodeGlow;

        // < 1 при отдалении (camera.zoom < m_baseZoom) — гасит яркость,
        // чтобы перекрывающиеся при отдалении узлы/пружины не сливались в
        // засвеченное пятно. Нижний предел не даём уйти в ноль (сетку видно
        // на любом зуме).
        float dim = m_baseZoom > 0.0f
            ? std::clamp(camera.zoom / m_baseZoom, 0.12f, 1.0f)
            : 1.0f;

        if (m_render.showGrid.load())
            renderGrid(pos, edgeGlow, dim, camera);

        // Аддитивный блендинг с overdraw на ~20К точек — реальная нагрузка
        // на GPU, а не бесплатная; при сильном отдалении и так почти не
        // видно, пропускаем целиком.
        if (dim > 0.2f) {
            if (m_render.showPoints.load())
                renderPointLayer(pos, displayGlow, dim, /*cubeMode=*/false, camera);
            if (m_render.showCubes.load())
                renderPointLayer(m_restPos, displayGlow, dim, /*cubeMode=*/true, camera);
        }

        glBindVertexArray(0);
    }

    void onImGui() override {
#ifdef TESSERA_IMGUI_ENABLED
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Light through Springs", nullptr, ImGuiWindowFlags_NoCollapse);

        drawStatus();
        drawTimeControls();
        drawPhysicsControls();
        drawInteractionControls();
        drawRenderingControls();

        ImGui::End();
#endif
    }

    void onDestroy() override {
        m_watchdog.stop();

        m_lineShader.reset();
        m_pointShader.reset();
        if (m_lineVAO) glDeleteVertexArrays(1, &m_lineVAO);
        if (m_lineVBO) glDeleteBuffers(1, &m_lineVBO);
        if (m_pointVAO) glDeleteVertexArrays(1, &m_pointVAO);
        if (m_pointVBO) glDeleteBuffers(1, &m_pointVBO);
        TaskScheduler::instance().shutdown();
    }

private:
    // ---------- onInit ----------

    void initShaders() {
        m_lineShader  = std::make_unique<Shader>("Shaders/cloth_line.vert",  "Shaders/cloth_line.frag");
        m_pointShader = std::make_unique<Shader>("Shaders/cloth_point.vert", "Shaders/cloth_point.frag");

        glGenVertexArrays(1, &m_lineVAO);
        glGenBuffers(1, &m_lineVBO);
        setupVertexLayout(m_lineVAO, m_lineVBO);

        glGenVertexArrays(1, &m_pointVAO);
        glGenBuffers(1, &m_pointVBO);
        setupVertexLayout(m_pointVAO, m_pointVBO);

        glBindVertexArray(0);

        glEnable(GL_PROGRAM_POINT_SIZE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);   // аддитивно — светящиеся пружины/узлы складываются
        glClearColor(0.03f, 0.03f, 0.05f, 1.0f);
    }

    // Линии и точки используют один и тот же layout: vec2 позиция + float яркость.
    static void setupVertexLayout(unsigned int vao, unsigned int vbo) {
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }

    void initCamera() {
        glm::vec2 worldMin(0.0f, 0.0f);
        glm::vec2 worldMax(kCols * kSpacing, kRows * kSpacing);
        frameCamera(worldMin, worldMax, kSpacing * 2.0f);

        // Зум сразу после frameCamera = "вся сетка целиком в кадре". Дальше
        // берём отношение текущего зума к этому как коэффициент затемнения
        // при отдалении (см. onRender()).
        m_baseZoom = getCamera().zoom;
    }

    void cacheTopology() {
        // Топология и позиции покоя не меняются после этого — кэшируем один
        // раз здесь, а не гоняем их каждый кадр вместе со snapshot() (на
        // большой сетке Edge-массив — самый тяжёлый кусок состояния, см.
        // SpringNetwork.h).
        m_network->topology(m_edges, m_renderEdgeCount);
        m_lineVertexData.resize(m_renderEdgeCount * 2 * 3);
        // Для Cube mode (см. renderPointLayer()) — кубик стоит на месте
        // сетки, не там, где узел физически оказался.
        m_network->restPositions(m_restPos);
        // CSR-таблица structural-рёбер по чанкам — для отбраковки вне-экранных
        // рёбер при рендере по чанкам вместо строк (см. renderGrid()). Порядок
        // рёбер ВНУТРИ [0, m_renderEdgeCount) теперь чанковый, не построчный
        // (см. SpringNetwork::reset()/bucketEdgeBlock()).
        m_network->chunkTopology(m_chunkStructOffset);
    }

    // ---------- onUpdate ----------

    void handlePointerInput(float dt) {
        float mx, my;
        m_input.getMousePosition(mx, my);
        glm::vec2 mouseWorld = getCamera().screenToWorld(mx, my);

        bool lmb = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT) && !m_imguiWantMouse.load();
        bool rmb = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT) && !m_imguiWantMouse.load();

        if (m_brush.enabled.load())
            handleBrushInput(mouseWorld, dt, lmb, rmb);
        else
            handleDragPluckInput(mouseWorld, dt, lmb, rmb);

        m_rmbPrev = rmb;
    }

    // LMB впрыскивает энергию, RMB гасит — оба держатся, а не кликаются (в
    // отличие от pluck), поэтому вызываем каждый кадр, пока кнопка зажата.
    void handleBrushInput(glm::vec2 mouseWorld, float dt, bool lmb, bool rmb) {
        if (lmb) m_network->brush(mouseWorld, m_brush.radius.load(), m_brush.strength.load(), dt);
        if (rmb) m_network->brushDamp(mouseWorld, m_brush.radius.load(), m_brush.strength.load(), dt);
        // Если переключили режим прямо с зажатым LMB — не оставляем узел
        // висеть "схваченным" в drag-состоянии навсегда.
        if (m_grabbed >= 0) { m_network->endDrag(m_grabbed); m_grabbed = -1; }
    }

    void handleDragPluckInput(glm::vec2 mouseWorld, float dt, bool lmb, bool rmb) {
        if (lmb) {
            if (m_grabbed < 0) m_grabbed = m_network->beginDrag(mouseWorld);
            if (m_grabbed >= 0) m_network->updateDrag(m_grabbed, mouseWorld, dt);
        } else if (m_grabbed >= 0) {
            m_network->endDrag(m_grabbed);
            m_grabbed = -1;
        }

        if (rmb && !m_rmbPrev)
            m_network->pluck(mouseWorld, m_physics.pluckStrength.load());
    }

    // Update-поток (Application::updateLoop) крутится без ограничения
    // частоты — на 20К узлов step() параллелится и отрабатывает за доли
    // миллисекунды, из-за чего update-цикл звал бы его тысячи раз в
    // секунду, захлёбывая общий TaskScheduler и мешая рендер-потоку
    // получать свои задачи вовремя. Поэтому ограничиваем частоту физики
    // отдельно от кадров: копим реальное время и шагаем не чаще kMaxStepHz.
    void stepPhysics(float dt) {
        constexpr float kMaxStepHz = 120.0f;
        constexpr float kMinStepInterval = 1.0f / kMaxStepHz;
        constexpr float kMaxStableDt = 1.0f / 50.0f;   // устойчивость явного интегратора, не про троттлинг

        m_timeSinceLastStep += dt;
        if (m_timeSinceLastStep >= kMinStepInterval) {
            float scaledDt = std::min(m_timeSinceLastStep * m_physics.timeScale.load(), kMaxStableDt);
            m_network->step(scaledDt, m_physics.stiffness.load(), m_physics.dampingRate.load());
            m_timeSinceLastStep = 0.0f;
        }
    }

    // ---------- onRender ----------

    // Видимые (плюс запас в клетках) строки/столбцы сетки по AABB камеры —
    // на большой сетке (сотни тысяч+ узлов) строить/заливать в GPU данные
    // для того, что всё равно за кадром, бессмысленно: при обычном
    // приближении видна лишь малая доля всего поля. Запас в клетках — узел
    // может отклониться от rest-позиции (Points берёт реальную m_pos, не
    // restPos — см. вызовы renderPointLayer ниже), да и чтобы новые
    // ряды/столбцы не "выпрыгивали" резко на самом краю кадра при панораме.
    // Для Cube mode (renderPos==m_restPos) это не приближение, а точный
    // расчёт — узел физически не двигается.
    static constexpr float kCullMarginCells = 4.0f;

    void visibleRowRange(const Camera2D& camera, int& rowMin, int& rowMaxEx) const {
        glm::vec2 mn, mx;
        camera.getVisibleAABB(mn, mx);
        float spacing = m_network->spacing();
        int rows = m_network->rows();
        rowMin = std::clamp(static_cast<int>(std::floor(mn.y / spacing - kCullMarginCells)), 0, rows - 1);
        int rowMax = static_cast<int>(std::ceil(mx.y / spacing + kCullMarginCells));
        rowMaxEx = std::clamp(rowMax + 1, rowMin + 1, rows);
    }

    void visibleColRange(const Camera2D& camera, int& colMin, int& colMaxEx) const {
        glm::vec2 mn, mx;
        camera.getVisibleAABB(mn, mx);
        float spacing = m_network->spacing();
        int cols = m_network->cols();
        colMin = std::clamp(static_cast<int>(std::floor(mn.x / spacing - kCullMarginCells)), 0, cols - 1);
        int colMax = static_cast<int>(std::ceil(mx.x / spacing + kCullMarginCells));
        colMaxEx = std::clamp(colMax + 1, colMin + 1, cols);
    }

    // Видимый прямоугольник ЧАНКОВ (не строк — см. renderGrid()) по AABB
    // камеры, с запасом kCullMarginCells клеток, переведённым в чанки, ПЛЮС
    // ещё один целый чанк с каждой стороны. Обязательно: рёбра принадлежат
    // чанку своего узла a ("верхне-левого" конца — см. bucketEdgeBlock()), а
    // рендерится только structural-срез ВЛАДЕЮЩЕГО чанка целиком (см.
    // renderGrid()). kCullMarginCells (4 клетки) — запас в МИРОВЫХ
    // координатах, а один чанк — это kChunkSize (64) клеток; без запаса в
    // целый чанк ребро "право"/"вниз", уходящее ИЗ невидимого чанка ролью в
    // видимый, теряется целиком (его владелец — чанк за кадром, не
    // рендерится вовсе), что даёт периodические пробелы в сетке ровно по
    // границам чанков при панораме/зуме.
    void visibleChunkRange(const Camera2D& camera, int& chunkColLo, int& chunkColHi,
                            int& chunkRowLo, int& chunkRowHi) const {
        glm::vec2 mn, mx;
        camera.getVisibleAABB(mn, mx);
        float chunkWorld = static_cast<float>(SpringNetwork::kChunkSize) * m_network->spacing();
        float margin = kCullMarginCells * m_network->spacing();
        int chunksX = m_network->chunksX(), chunksY = m_network->chunksY();
        chunkColLo = std::clamp(static_cast<int>(std::floor((mn.x - margin) / chunkWorld)) - 1, 0, chunksX - 1);
        chunkColHi = std::clamp(static_cast<int>(std::floor((mx.x + margin) / chunkWorld)) + 1, 0, chunksX - 1);
        chunkRowLo = std::clamp(static_cast<int>(std::floor((mn.y - margin) / chunkWorld)) - 1, 0, chunksY - 1);
        chunkRowHi = std::clamp(static_cast<int>(std::floor((mx.y + margin) / chunkWorld)) + 1, 0, chunksY - 1);
    }

    void renderGrid(const std::vector<glm::vec2>& pos, const std::vector<float>& edgeGlow,
                     float dim, const Camera2D& camera) {
        // Рёбра [0, m_renderEdgeCount) идут блоками ПО ЧАНКАМ (см.
        // SpringNetwork::reset()/bucketEdgeBlock()), не по строкам — поэтому
        // отбраковка вне-экранного тоже по чанкам: для каждого видимого
        // чанка его structural-срез — [m_chunkStructOffset[c], [c+1)) —
        // склеиваем эти срезы подряд в m_lineVertexData.
        int chunksX = m_network->chunksX();
        int chunkColLo, chunkColHi, chunkRowLo, chunkRowHi;
        visibleChunkRange(camera, chunkColLo, chunkColHi, chunkRowLo, chunkRowHi);

        size_t cursor = 0;
        for (int cy = chunkRowLo; cy <= chunkRowHi; ++cy) {
            for (int cx = chunkColLo; cx <= chunkColHi; ++cx) {
                int c = cy * chunksX + cx;
                uint32_t eBegin = m_chunkStructOffset[static_cast<size_t>(c)];
                uint32_t eEnd = m_chunkStructOffset[static_cast<size_t>(c) + 1];
                int count = static_cast<int>(eEnd - eBegin);
                if (count <= 0) continue;
                size_t writeBase = cursor;
                parallelFor(count, [&](int begin, int end, int) {
                    for (int e = begin; e < end; ++e) {
                        const auto& edge = m_edges[eBegin + static_cast<uint32_t>(e)];
                        float glow = edgeGlow[eBegin + static_cast<uint32_t>(e)];
                        size_t base = (writeBase + static_cast<size_t>(e)) * 6;
                        m_lineVertexData[base + 0] = pos[static_cast<size_t>(edge.a)].x;
                        m_lineVertexData[base + 1] = pos[static_cast<size_t>(edge.a)].y;
                        m_lineVertexData[base + 2] = glow;
                        m_lineVertexData[base + 3] = pos[static_cast<size_t>(edge.b)].x;
                        m_lineVertexData[base + 4] = pos[static_cast<size_t>(edge.b)].y;
                        m_lineVertexData[base + 5] = glow;
                    }
                });
                cursor += static_cast<size_t>(count);
            }
        }
        int renderEdges = static_cast<int>(cursor);
        if (renderEdges <= 0) return;

        glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
        glBufferData(GL_ARRAY_BUFFER, static_cast<size_t>(renderEdges) * 6 * sizeof(float),
                     m_lineVertexData.data(), GL_STREAM_DRAW);

        m_lineShader->use();
        m_lineShader->setMat4("uCamera", camera.getViewProjectionMatrix());
        m_lineShader->setFloat("uDim", dim);
        glBindVertexArray(m_lineVAO);
        glDrawArrays(GL_LINES, 0, renderEdges * 2);
    }

    // Один слой точек/кубиков: renderPos — откуда брать позицию вершины
    // (реальная физическая для Points, m_restPos для статичных Cubes),
    // cubeMode — форма пятна в шейдере (круглое затухание vs сплошной
    // квадрат фиксированного размера, см. spring_point.vert/.frag).
    void renderPointLayer(const std::vector<glm::vec2>& renderPos, const std::vector<float>& glow,
                           float dim, bool cubeMode, const Camera2D& camera) {
        int cols = m_network->cols();
        int rowMin, rowMaxEx, colMin, colMaxEx;
        visibleRowRange(camera, rowMin, rowMaxEx);
        visibleColRange(camera, colMin, colMaxEx);
        int visRows = rowMaxEx - rowMin;
        int visCols = colMaxEx - colMin;
        int visCount = visRows * visCols;
        if (visCount <= 0) return;

        m_pointVertexData.resize(static_cast<size_t>(visCount) * 3);
        parallelFor(visCount, [&](int begin, int end, int) {
            for (int idx = begin; idx < end; ++idx) {
                int rr = idx / visCols;
                int cc = idx - rr * visCols;
                int i = (rowMin + rr) * cols + (colMin + cc);
                size_t base = static_cast<size_t>(idx) * 3;
                m_pointVertexData[base + 0] = renderPos[i].x;
                m_pointVertexData[base + 1] = renderPos[i].y;
                m_pointVertexData[base + 2] = glow[i];
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
        // gl_PointSize — в экранных пикселях, не зависит от зума сам по
        // себе, поэтому кубики без этого при приближении становятся
        // относительно всё меньше клетки сетки. Переводим мировой spacing в
        // пиксели через camera.zoom (screenToWorld делит на zoom — см.
        // Camera2D.h), чтобы кубик всегда стыковался со следующим, даже
        // вблизи. +1px — GL_POINTS может округлить размер точки вниз на
        // суб-пиксель (implementation-defined), из-за чего между соседними
        // кубиками появлялась щель ровно в один пиксель; небольшой нахлёст
        // дешевле и надёжнее, чем полагаться на точное round-to-even.
        m_pointShader->setFloat("uCellSizePx", m_network->spacing() * camera.zoom + 1.0f);
        glBindVertexArray(m_pointVAO);
        glDrawArrays(GL_POINTS, 0, visCount);
    }

    // ---------- onImGui ----------
#ifdef TESSERA_IMGUI_ENABLED
    // Слайдер (для перетаскивания в привычном диапазоне) + отдельное поле
    // ввода рядом (клик — сразу можно печатать/вставлять, без зажатия и без
    // Ctrl/двойного клика). lo/hi ограничивают только перетаскивание
    // слайдера — в поле ввода можно напечатать значение за пределами этого
    // диапазона, оно не обрезается.
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
        if (changedSlider) value = std::clamp(v, lo, hi);   // drag сам не выходит за диапазон
        else if (changedInput) value = v;                   // напечатанное — без ограничений
        ImGui::PopID();
    }

    void drawStatus() {
        ImGui::Text("FPS: %d", m_fps.load());
        if (m_brush.enabled.load())
            ImGui::TextWrapped("LMB hold - paint energy. RMB hold - erase energy.");
        else
            ImGui::TextWrapped("LMB drag - grab a node. RMB click - pluck it.");
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

        sliderWithInput("Stiffness", m_physics.stiffness, 1.0f, 200.0f, "%.0f");
        sliderWithInput("Damping rate", m_physics.dampingRate, 0.0f, 15.0f, "%.1f");
        sliderWithInput("Pluck strength", m_physics.pluckStrength, 0.0f, 200.0f, "%.0f");
    }

    void drawInteractionControls() {
        ImGui::Spacing();
        ImGui::TextDisabled("Interaction");
        ImGui::Separator();

        bool brushEnabled = m_brush.enabled.load();
        if (ImGui::Checkbox("Brush mode (LMB paint / RMB erase)", &brushEnabled))
            m_brush.enabled = brushEnabled;

        sliderWithInput("Brush radius", m_brush.radius, 10.0f, 300.0f, "%.0f");
        sliderWithInput("Brush strength", m_brush.strength, 0.0f, 1000.0f, "%.0f");

        if (ImGui::Button("Reset grid", ImVec2(-1, 0)))
            m_network->reset();
    }

    void drawRenderingControls() {
        ImGui::Spacing();
        ImGui::TextDisabled("Rendering");
        ImGui::Separator();

        bool showGrid = m_render.showGrid.load();
        if (ImGui::Checkbox("Grid", &showGrid))
            m_render.showGrid = showGrid;

        bool showPoints = m_render.showPoints.load();
        if (ImGui::Checkbox("Points", &showPoints))
            m_render.showPoints = showPoints;

        bool showCubes = m_render.showCubes.load();
        if (ImGui::Checkbox("Cubes", &showCubes))
            m_render.showCubes = showCubes;

        ImGui::TextDisabled("What to display (Points/Cubes)");
        bool showStretch = m_render.showStretch.load();
        if (ImGui::RadioButton("Speed", !showStretch)) m_render.showStretch = false;
        ImGui::SameLine();
        if (ImGui::RadioButton("Stretch", showStretch)) m_render.showStretch = true;
    }
#endif

    std::unique_ptr<SpringNetwork> m_network;
    std::unique_ptr<Shader> m_lineShader, m_pointShader;
    unsigned int m_lineVAO = 0, m_lineVBO = 0;
    unsigned int m_pointVAO = 0, m_pointVBO = 0;

    // Переиспользуемые CPU-буферы для аплоада — чтобы не аллоцировать
    // std::vector<float> заново каждый кадр в рендер-потоке.
    std::vector<float> m_lineVertexData;
    std::vector<float> m_pointVertexData;

    // Топология и позиции покоя — кэш из SpringNetwork, не меняются после
    // onInit(). Рёбра [0, m_renderEdgeCount) — то, что реально рисуется.
    std::vector<SpringNetwork::Edge> m_edges;
    size_t m_renderEdgeCount = 0;
    std::vector<glm::vec2> m_restPos;
    // CSR-таблица structural-рёбер по чанкам (size chunksX*chunksY+1) — для
    // видимости по чанкам в renderGrid(), см. cacheTopology()/visibleChunkRange().
    std::vector<uint32_t> m_chunkStructOffset;

    float m_baseZoom = 1.0f;            // зум камеры сразу после frameCamera() в onInit
    float m_timeSinceLastStep = 0.0f;   // троттлинг частоты step() — см. stepPhysics()

    PhysicsParams m_physics;
    BrushSettings m_brush;
    RenderSettings m_render;

    int  m_grabbed = -1;
    bool m_rmbPrev = false;

    HangWatchdog m_watchdog;
};

int main() {
    // Без try/catch необработанное исключение (например, Shader() не нашёл
    // .vert/.frag — типичная причина "в релизе вылетает", если exe запущен
    // не из своей директории и относительный путь "Shaders/..." не резолвится)
    // улетает из main() как есть, и std::terminate() валит процесс без
    // единого печатного слова — не отличить от честного краша по гонке.
    try {
        SpringLightApp app;
        app.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[springs] fatal: %s\n", e.what());
        return 1;
    }
    return 0;
}
