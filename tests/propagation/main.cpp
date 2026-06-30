// demo/propagation_test/main.cpp
//
// Headless-проверка межчанковой эволюции: одиночный глайдер Конвея запускается у
// границы чанка и должен корректно перейти в соседний чанк, сохранив форму и
// сдвинувшись на (+1,+1) за каждые 4 шага. Ловит баг, когда жизнь "обрезается"
// на границе чанка (сосед не активируется). Exit 0 = OK, 1 = FAIL.
//
// Нужен GL-контекст: Chunk создаёт ChunkRenderer. Открываем скрытое окно.

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "engine/automaton/LifeLikeAutomaton.h"
#include "engine/core/TaskScheduler.h"
#include "engine/graphics/Camera2D.h"
#include "engine/simulation/LifeRule.h"

#include <cstdio>
#include <vector>

namespace {

constexpr int CHUNK = 16;     // маленький чанк, чтобы глайдер быстро дошёл до границы
constexpr int GRID  = 64;
constexpr float TS  = 1.0f;

// Канонический глайдер, движущийся на юго-восток, левый верхний угол в (ox,oy):
//   . # .
//   . . #
//   # # #
void stampGlider(LifeLikeAutomaton& m, int ox, int oy) {
    m.setTile(ox + 1, oy + 0, 255);
    m.setTile(ox + 2, oy + 1, 255);
    m.setTile(ox + 0, oy + 2, 255);
    m.setTile(ox + 1, oy + 2, 255);
    m.setTile(ox + 2, oy + 2, 255);
}

// Проверяет, что живые клетки = ровно глайдер с левым верхним углом (ox,oy).
bool checkGlider(LifeLikeAutomaton& m, int ox, int oy) {
    const int expect[5][2] = {
        {ox + 1, oy + 0}, {ox + 2, oy + 1},
        {ox + 0, oy + 2}, {ox + 1, oy + 2}, {ox + 2, oy + 2}
    };
    for (auto& c : expect)
        if (m.getTileState(c[0], c[1]) == 0) {
            std::printf("  missing cell at (%d,%d)\n", c[0], c[1]);
            return false;
        }
    int live = 0;
    for (int y = 0; y < GRID; ++y)
        for (int x = 0; x < GRID; ++x)
            if (m.getTileState(x, y) != 0) ++live;
    if (live != 5) {
        std::printf("  expected 5 live cells, got %d\n", live);
        return false;
    }
    return true;
}

} // namespace

int main() {
    if (!glfwInit()) { std::fprintf(stderr, "glfwInit failed\n"); return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* win = glfwCreateWindow(64, 64, "prop", nullptr, nullptr);
    if (!win) { std::fprintf(stderr, "window failed\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::fprintf(stderr, "GLAD failed\n"); return 1;
    }
    TaskScheduler::instance().initialize();

    LifeRule rule = MakeConwayRule();
    LifeLikeAutomaton map(GRID, GRID, CHUNK, TS, rule);

    // Глайдер у границы первого чанка (чанк 0 = 0..15). За 8 шагов сдвиг (+2,+2)
    // -> угол (16,16): полностью в чанке (1,1).
    const int ox0 = 14, oy0 = 14;
    stampGlider(map, ox0, oy0);
    map.commitInitialState();

    Camera2D cam(static_cast<float>(GRID), static_cast<float>(GRID));
    cam.zoom = 1.0f;
    cam.position = glm::vec2(0.0f);

    const int STEPS = 16;   // 4 периода глайдера -> сдвиг (+4,+4)
    for (int i = 0; i < STEPS; ++i) {
        map.simulateAndWait();
        map.commitReadyChunks(cam);
    }

    // Ожидаемый угол: (14+4, 14+4) = (18,18) -> глайдер целиком в чанке (1,1).
    bool ok = checkGlider(map, ox0 + 4, oy0 + 4);
    std::printf("[%s] glider crossed chunk boundary (16,16) intact\n",
                ok ? "PASS" : "FAIL");

    TaskScheduler::instance().shutdown();
    glfwMakeContextCurrent(nullptr);
    glfwDestroyWindow(win);
    glfwTerminate();

    std::printf("\n%s\n", ok ? "ALL TESTS PASSED" : "*** FAILED ***");
    return ok ? 0 : 1;
}
