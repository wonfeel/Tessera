// demo/light/main.cpp
//
// Наименьшая возможная демка LightField: одно скалярное волновое поле на
// гекс-сетке, клик мышью — разовый импульс (pluck), рендер — обычный
// decaying-glow без призмы/луча/накопления/ImGui-панелей. Полная демка со
// всеми инструментами и хроматической дисперсией R/G/B — в отдельном
// репозитории github.com/wonfeel/WaveLight (тот же класс LightField).
//
//   LMB click — pluck (разовый импульс амплитуды)
//   WASD/scroll/MMB — камера

#include "engine/core/Application.h"
#include "engine/core/TaskScheduler.h"
#include "engine/graphics/Shader.h"
#include "demo/light/LightField.h"

#include <glad/glad.h>
#include <algorithm>
#include <memory>
#include <vector>

namespace {
    constexpr int   kCols = 200, kRows = 200;
    constexpr float kSpacing = 32.0f;
}

class MinimalLightApp : public Application {
public:
    MinimalLightApp()
        : Application(1280, 720, "Minimal Light", false)
        , m_field(std::make_unique<LightField>(kCols, kRows, kSpacing))
    {}

protected:
    void onInit() override {
        TaskScheduler::instance().initialize();

        m_shader = std::make_unique<Shader>("Shaders/hex_point.vert", "Shaders/hex_point.frag");
        glGenVertexArrays(1, &m_vao);
        glGenBuffers(1, &m_vbo);
        glBindVertexArray(m_vao);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);

        glEnable(GL_PROGRAM_POINT_SIZE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glClearColor(0.03f, 0.03f, 0.05f, 1.0f);

        glm::vec2 worldMax = m_field->worldPos(kCols - 1, kRows - 1);
        frameCamera(glm::vec2(0.0f), worldMax, kSpacing * 2.0f);
    }

    void onUpdate(float dt) override {
        dt = std::min(dt, 0.033f);

        float mx, my;
        m_input.getMousePosition(mx, my);
        bool lmb = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT) && !m_imguiWantMouse.load();
        if (lmb && !m_lmbPrev)
            m_field->pluck(getCamera().screenToWorld(mx, my), 1.0f);
        m_lmbPrev = lmb;

        m_field->step(dt, /*waveSpeedSq*/ 4000.0f, /*dampingRate*/ 0.6f, /*dispersion*/ 0.0f);
    }

    void onRender(const Camera2D& camera) override {
        m_field->snapshot(m_glow, m_mask, m_accum);

        int cols = m_field->cols(), rows = m_field->rows();
        m_vertexData.resize(static_cast<size_t>(cols) * rows * 5);
        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                size_t i = static_cast<size_t>(row) * cols + col;
                glm::vec2 p = m_field->worldPos(col, row);
                size_t base = i * 5;
                float c = 0.15f + m_glow[i];
                m_vertexData[base + 0] = p.x;
                m_vertexData[base + 1] = p.y;
                m_vertexData[base + 2] = c;
                m_vertexData[base + 3] = c;
                m_vertexData[base + 4] = c * 1.2f;
            }
        }
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferData(GL_ARRAY_BUFFER, m_vertexData.size() * sizeof(float),
                     m_vertexData.data(), GL_STREAM_DRAW);

        m_shader->use();
        m_shader->setMat4("uCamera", camera.getViewProjectionMatrix());
        m_shader->setFloat("uBaseSize", 1.0f);
        m_shader->setFloat("uCellSizePx", m_field->spacing() * 2.0f * camera.zoom + 1.0f);
        m_shader->setInt("uShapeMode", 0);
        glBindVertexArray(m_vao);
        glDrawArrays(GL_POINTS, 0, cols * rows);
        glBindVertexArray(0);
    }

    void onDestroy() override {
        m_shader.reset();
        if (m_vao) glDeleteVertexArrays(1, &m_vao);
        if (m_vbo) glDeleteBuffers(1, &m_vbo);
        TaskScheduler::instance().shutdown();
    }

private:
    std::unique_ptr<LightField> m_field;
    std::unique_ptr<Shader> m_shader;
    unsigned int m_vao = 0, m_vbo = 0;
    std::vector<float> m_glow, m_mask, m_accum;
    std::vector<float> m_vertexData;
    bool m_lmbPrev = false;
};

int main() {
    MinimalLightApp app;
    app.run();
    return 0;
}
