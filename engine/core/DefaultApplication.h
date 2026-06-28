// engine/core/DefaultApplication.h
#pragma once
#include "engine/core/Application.h"
#include "engine/chunk/ChunkedTileMap.h"
#include "engine/chunk/ChunkRenderer.h"
#include "engine/core/TaskScheduler.h"
#include <functional>
#include <memory>
#include <chrono>

class DefaultApplication : public Application {
public:
    using TileMapFactory = std::function<std::unique_ptr<ChunkedTileMap>()>;

    DefaultApplication(TileMapFactory factory,
        std::chrono::milliseconds simInterval = std::chrono::milliseconds(0),
        int windowWidth = 1920, int windowHeight = 1080,
        const std::string& title = "FieldEngine Demo",
        bool showPerformance = false)
        : Application(windowWidth, windowHeight, title, showPerformance)
        , m_factory(std::move(factory))
        , m_simInterval(simInterval)
    {
    }

protected:
    void onInit() override {
        m_tileMap = m_factory();
        if (!m_tileMap) throw std::runtime_error("TileMap factory returned nullptr");

        float mapW = m_tileMap->getWidth() * m_tileMap->getTileSize();
        float mapH = m_tileMap->getHeight() * m_tileMap->getTileSize();
        getCamera().position = glm::vec2((mapW - getWidth()) * 0.5f,
            (mapH - getHeight()) * 0.5f);

        TaskScheduler::instance().initialize();
        m_lastSimTime = std::chrono::steady_clock::now();
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    }

    void onUpdate(float) override {
        if (m_simInterval.count() == 0) {
            m_tileMap->simulateActiveChunks();   // неблокирующий вызов
        }
        else {
            auto now = std::chrono::steady_clock::now();
            if (now - m_lastSimTime >= m_simInterval) {
                m_tileMap->simulateActiveChunks();
                m_lastSimTime = now;
            }
        }
    }

    void onRender(const Camera2D& camera) override {
        if (m_tileMap) {
            m_tileMap->commitReadyChunks(camera);
            m_tileMap->render(camera);
        }
    }

    void onDestroy() override {
        // Останавливаем планировщик, чтобы гарантировать, что все задачи-симуляции завершены
        // и больше не обращаются к ChunkedTileMap.
        TaskScheduler::instance().shutdown();

        m_tileMap.reset();
        ChunkRenderer::shutdownStatics();
    }

private:
    TileMapFactory m_factory;
    std::unique_ptr<ChunkedTileMap> m_tileMap;
    std::chrono::milliseconds m_simInterval;
    std::chrono::steady_clock::time_point m_lastSimTime;
};