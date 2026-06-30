// engine/core/DefaultApplication.h
#pragma once
#include "engine/core/Application.h"
#include "engine/chunk/ChunkedTileMap.h"
#include "engine/chunk/ChunkRenderer.h"
#include "engine/core/TaskScheduler.h"
#include <functional>
#include <memory>
#include <chrono>
#include <atomic>

#ifdef TESSERA_IMGUI_ENABLED
#  include <imgui.h>
#endif

class DefaultApplication : public Application {
public:
    using TileMapFactory = std::function<std::unique_ptr<ChunkedTileMap>()>;

    DefaultApplication(TileMapFactory factory,
        std::chrono::milliseconds simInterval = std::chrono::milliseconds(0),
        int windowWidth = 1920, int windowHeight = 1080,
        const std::string& title = "Tessera Demo",
        bool showPerformance = false)
        : Application(windowWidth, windowHeight, title, showPerformance)
        , m_factory(std::move(factory))
        , m_simSpeedMs(static_cast<int>(simInterval.count()))
    {}

protected:
    void onInit() override {
        m_tileMap = m_factory();
        if (!m_tileMap) throw std::runtime_error("TileMap factory returned nullptr");

        float mapW = m_tileMap->getWidth()  * m_tileMap->getTileSize();
        float mapH = m_tileMap->getHeight() * m_tileMap->getTileSize();
        getCamera().position = glm::vec2((mapW - getWidth())  * 0.5f,
                                          (mapH - getHeight()) * 0.5f);

        TaskScheduler::instance().initialize();
        m_lastSimTime = std::chrono::steady_clock::now();
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    }

    void onUpdate(float) override {
        // Space: toggle pause (edge-detect in update thread)
        bool spaceNow = m_input.isKeyPressed(GLFW_KEY_SPACE);
        if (spaceNow && !m_spacePrev)
            m_paused = !m_paused.load();
        m_spacePrev = spaceNow;

        if (m_paused.load()) {
            // На паузе можно сделать ровно один шаг кнопкой "Step".
            if (m_stepRequested.exchange(false)) {
                if (m_tileMap->simulateActiveChunks())
                    m_generation.fetch_add(1, std::memory_order_relaxed);
            }
            return;
        }

        int ms = m_simSpeedMs.load();
        auto now = std::chrono::steady_clock::now();
        if (ms == 0 || now - m_lastSimTime >= std::chrono::milliseconds(ms)) {
            // Считаем поколение только если шаг действительно стартовал: при
            // ms==0 update-цикл крутится быстрее, чем шаги коммитятся, и пустые
            // вызовы (барьер фаз ещё держит предыдущее поколение) не должны
            // накручивать счётчик.
            if (m_tileMap->simulateActiveChunks())
                m_generation.fetch_add(1, std::memory_order_relaxed);
            m_lastSimTime = now;
        }
    }

    void onRender(const Camera2D& camera) override {
        if (m_tileMap) {
            // Во время записи GIF шаги симуляции и commit делает update-поток
            // (синхронно). Чтобы не было двух commit'ов из разных потоков —
            // здесь его пропускаем.
            if (!m_recording.load(std::memory_order_acquire))
                m_tileMap->commitReadyChunks(camera);
            m_tileMap->render(camera);
        }
    }

    // ImGui panel — called by Application's render loop between NewFrame/Render.
    void onImGui() override {
#ifdef TESSERA_IMGUI_ENABLED
        // Фиксированная высота + скролл: иначе при авто-высоте панель растёт за
        // нижний край экрана и нижние кнопки (в т.ч. запись GIF) не видно.
        // Окно можно перетаскивать и ресайзить, позиция/размер только по старту.
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(290, 600), ImGuiCond_FirstUseEver);
        ImGui::Begin("Tessera", nullptr,
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings);

        // --- Performance ---
        ImGui::TextDisabled("Performance");
        ImGui::Separator();
        ImGui::Text("FPS:        %d", m_fps.load());
        ImGui::Text("Generation: %llu",
                    static_cast<unsigned long long>(m_generation.load()));
        if (m_tileMap)
            ImGui::Text("Chunks:     %d active",
                        static_cast<int>(m_tileMap->getActiveChunks().size()));

        // --- Simulation ---
        ImGui::Spacing();
        ImGui::TextDisabled("Simulation");
        ImGui::Separator();

        bool paused = m_paused.load();
        if (ImGui::Button(paused ? " Play [Space] " : " Pause [Space] "))
            m_paused = !paused;
        ImGui::SameLine();
        // Шаг вперёд: ставит на паузу (если ещё не) и просит один шаг.
        if (ImGui::Button(" Step > ")) {
            m_paused = true;
            m_stepRequested = true;
        }

        int speed = m_simSpeedMs.load();
        ImGui::SetNextItemWidth(-1);
        if (ImGui::SliderInt("##speed", &speed, 0, 500))
            m_simSpeedMs = speed;
        ImGui::SameLine(0, 4);
        // label to the left of the slider doesn't work well with -1 width,
        // so show it as a separate text above
        if (speed == 0)
            ImGui::TextDisabled("Speed: max");
        else
            ImGui::TextDisabled("Speed: %d ms/step", speed);

        // --- Camera ---
        ImGui::Spacing();
        ImGui::TextDisabled("Camera");
        ImGui::Separator();
        ImGui::Text("Zoom: %.2fx", getCamera().zoom);
        ImGui::Text("Pos:  %.0f, %.0f",
                    getCamera().position.x, getCamera().position.y);

        // Subclass panels go below
        onImGuiExtra();

        ImGui::End();
#endif
    }

    // Override this in subclasses to add extra ImGui widgets inside the panel.
    virtual void onImGuiExtra() {}

    void onDestroy() override {
        TaskScheduler::instance().shutdown();
        m_tileMap.reset();
        ChunkRenderer::shutdownStatics();
    }

    std::unique_ptr<ChunkedTileMap> m_tileMap;
    std::atomic<bool>     m_paused{false};
    std::atomic<int>      m_simSpeedMs{0};
    std::atomic<uint64_t> m_generation{0};
    std::atomic<bool>     m_recording{false};  // true пока update-поток пишет GIF
    std::atomic<bool>     m_stepRequested{false};  // один шаг на паузе (кнопка Step)

private:
    TileMapFactory m_factory;
    std::chrono::steady_clock::time_point m_lastSimTime;
    bool m_spacePrev = false;
};
