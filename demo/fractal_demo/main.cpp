#include "engine/core/Application.h"
#include "engine/core/TaskScheduler.h"
#include "engine/graphics/GridRenderer.h"
#include "engine/graphics/Palette.h"
#include "demo/fractal_demo/FractalTileMap.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>

class FractalApp : public Application {
public:
    FractalApp()
        : Application(1920, 1080, "Mandelbrot Fractal", true)
    {
    }

protected:
    void onInit() override {
        TaskScheduler::instance().initialize();

        m_tileMap = std::make_unique<FractalTileMap>(1920, 1080, 1.0f);
        m_renderer = std::make_unique<GridRenderer>(*m_tileMap);
        // Палитра по умолчанию – можно переключать клавишами 1..5
        m_renderer->setPalette(Palette::DefaultRainbow());
        m_needRecompute = true;
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    }

    void onUpdate(float) override {
        // Переключение палитр по цифровым клавишам (1..5) – только запоминаем,
        // а применяем в onRender()
        if (m_input.isKeyPressed(GLFW_KEY_1)) {
            m_pendingPalette = Palette::DefaultRainbow();
        } else if (m_input.isKeyPressed(GLFW_KEY_2)) {
            m_pendingPalette = Palette::Grayscale();
        } else if (m_input.isKeyPressed(GLFW_KEY_3)) {
            m_pendingPalette = Palette::ANSI();
        } else if (m_input.isKeyPressed(GLFW_KEY_4)) {
            m_pendingPalette = Palette::Heatmap();
        } else if (m_input.isKeyPressed(GLFW_KEY_5)) {
            m_pendingPalette = Palette::Cool();
        }

        if (m_needRecompute) {
            double zoom = 300.0 * getCamera().zoom;
            m_tileMap->computeMandelbrotParallel(0.0, 0.0, zoom);
            m_needRecompute = false;
        }
    }

    void onRender(const Camera2D& camera) override {
        // Применяем отложенную смену палитры в рендерном потоке
        if (!m_pendingPalette.empty()) {
            m_renderer->setPalette(m_pendingPalette);
            m_pendingPalette.clear();
        }
        m_renderer->render(camera);
    }

    void onDestroy() override {
        m_tileMap.reset();
        m_renderer.reset();
        TaskScheduler::instance().shutdown();
    }

private:
    std::unique_ptr<FractalTileMap> m_tileMap;
    std::unique_ptr<GridRenderer> m_renderer;
    bool m_needRecompute = true;
    std::vector<glm::vec3> m_pendingPalette;
};

int main() {
    FractalApp app;
    app.run();
    return 0;
}