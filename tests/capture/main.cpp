// demo/capture_demo/main.cpp
//
// Headless frame-dump tool for making GIFs of a simulation.
// Renders a fixed grid region into a hidden offscreen window at a given
// resolution, steps the simulation deterministically, and writes PPM frames.
// No ImGui, no mouse cursor, static camera, simulation runs at full speed.
//
// A companion Python script (tools/capture_gif.py) drives this exe with
// parameters and assembles the PPM frames into a GIF.
//
// Args (all positional, with defaults):
//   1  outDir    output directory for frame_####.ppm        (default ".")
//   2  stopStep  last simulation step to capture             (default 300)
//   3  stride    steps between captured frames               (default 2)
//   4  resW      output width  in pixels                     (default 600)
//   5  resH      output height in pixels                     (default 400)
//   6  x0        region left   in grid (tile) coordinates    (default 0)
//   7  y0        region top    in grid coordinates           (default 0)
//   8  x1        region right  in grid coordinates           (default 300)
//   9  y1        region bottom in grid coordinates           (default 200)
//   10 gridW     field width  in tiles                       (default 1024)
//   11 gridH     field height in tiles                       (default 1024)
//   12 gunsX     number of Gosper guns horizontally          (default 1)
//   13 gunsY     number of Gosper guns vertically            (default 1)

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "engine/automaton/LifeLikeAutomaton.h"
#include "engine/chunk/ChunkRenderer.h"
#include "engine/core/TaskScheduler.h"
#include "engine/graphics/Camera2D.h"
#include "engine/simulation/LifeRule.h"
#include "engine/utils/GifExport.h"
#include "engine/utils/RleLoader.h"

#include <string>
#include <functional>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr float TILE_SIZE  = 10.0f;
constexpr int   CHUNK_SIZE = 64;

int argi(int argc, char** argv, int idx, int def) {
    return (argc > idx) ? std::atoi(argv[idx]) : def;
}

// Write an RGB buffer (top-down, row-major) as a binary PPM (P6).
bool writePPM(const std::string& path, const std::vector<unsigned char>& rgb,
              int w, int h) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    std::fwrite(rgb.data(), 1, rgb.size(), f);
    std::fclose(f);
    return true;
}

} // namespace

int main(int argc, char** argv) {
    const std::string outDir = (argc > 1) ? argv[1] : ".";
    const int stopStep = argi(argc, argv, 2, 300);
    const int stride   = std::max(1, argi(argc, argv, 3, 2));
    const int resW     = argi(argc, argv, 4, 600);
    const int resH     = argi(argc, argv, 5, 400);
    const int x0       = argi(argc, argv, 6, 0);
    const int y0       = argi(argc, argv, 7, 0);
    const int x1       = argi(argc, argv, 8, 300);
    const int y1       = argi(argc, argv, 9, 200);
    const int gridW    = argi(argc, argv, 10, 1024);
    const int gridH    = argi(argc, argv, 11, 1024);
    const int gunsX    = std::max(1, argi(argc, argv, 12, 1));
    const int gunsY    = std::max(1, argi(argc, argv, 13, 1));
    const int eaterX   = argi(argc, argv, 14, -1);   // <0 => no eater
    const int eaterY   = argi(argc, argv, 15, -1);
    const int eaterRot = argi(argc, argv, 16, 0);    // 0..3 поворот пожирателя
    const int eaterShape = argi(argc, argv, 17, 0);  // 0 = eater1 (крючок), 1 = блок 2x2
    // scene: 0 = guns (default), 1 = random 64x64, 2 = glider
    const int scene    = argi(argc, argv, 18, 0);

    if (x1 <= x0 || y1 <= y0) {
        std::fprintf(stderr, "[capture] invalid region: (%d,%d)-(%d,%d)\n",
                     x0, y0, x1, y1);
        return 1;
    }

    // --- GLFW hidden offscreen window ---
    if (!glfwInit()) { std::fprintf(stderr, "[capture] glfwInit failed\n"); return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);   // offscreen — no window on screen

    GLFWwindow* win = glfwCreateWindow(resW, resH, "capture", nullptr, nullptr);
    if (!win) { std::fprintf(stderr, "[capture] window creation failed\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::fprintf(stderr, "[capture] GLAD init failed\n"); return 1;
    }

    TaskScheduler::instance().initialize();

    // --- Build scene ---
    LifeRule rule{};
    rule.table[1][2] = 192;
    rule.table[1][3] = 255;
    rule.table[0][3] = 64;   // rainbow life-like rule (matches minimal demo)

    auto map = std::make_unique<LifeLikeAutomaton>(gridW, gridH, CHUNK_SIZE,
                                                   TILE_SIZE, rule);

    if (scene == 1) {
        // Random 64x64 field centered on the view region, seed 42, ~30% density.
        const int ox = x0, oy = y0;
        const int fw = std::min(64, x1 - x0), fh = std::min(64, y1 - y0);
        uint32_t rng = 42;
        auto lcg = [&]() { rng = rng * 1664525u + 1013904223u; return rng; };
        for (int cy = 0; cy < fh; ++cy)
            for (int cx = 0; cx < fw; ++cx)
                if ((lcg() & 0xFF) < 77)   // ~30 %
                    map->setTile(ox + cx, oy + cy, 255);
        std::fprintf(stderr, "[capture] random scene %dx%d at (%d,%d)\n", fw, fh, ox, oy);

    } else if (scene == 2) {
        // Single glider (.O. / ..O / OOO) at (x0+4, y0+4).
        const int gx = x0 + 4, gy = y0 + 4;
        const int cells[5][2] = {{1,0},{2,1},{0,2},{1,2},{2,2}};
        for (auto& c : cells) map->setTile(gx + c[0], gy + c[1], 255);
        std::fprintf(stderr, "[capture] glider at (%d,%d)\n", gx, gy);

    } else {
        // Default: Gosper guns.
        RlePattern gun = RleLoader::load("patterns/gosper_gun.rle");
        if (gun.width > 0) {
            const int stepX = 200, stepY = 200;
            int count = 0;
            for (int gj = 0; gj < gunsY; ++gj)
                for (int gi = 0; gi < gunsX; ++gi) {
                    map->stampPattern(gun, 20 + gi * stepX, 20 + gj * stepY);
                    ++count;
                }
            std::fprintf(stderr, "[capture] stamped %d Gosper gun(s) from RLE\n", count);
        } else {
            std::fprintf(stderr, "[capture] gosper_gun.rle not found\n");
        }

        // Auto-eater for a single gun (gun@(20,20) -> eater@(53,41), rot1).
        int ex = eaterX, ey = eaterY, erot = eaterRot;
        if (eaterX < 0 && eaterY < 0 && gunsX == 1 && gunsY == 1 && gun.width > 0) {
            ex = 53; ey = 41; erot = 1;
        }
        if (ex >= 0 && ey >= 0) {
            if (eaterShape == 1) {
                const int blk[4][2] = { {0,0},{1,0},{0,1},{1,1} };
                for (auto& c : blk) map->setTile(ex + c[0], ey + c[1], 255);
            } else {
                const int base[6][2] = { {0,0},{1,0},{0,1},{1,2},{2,2},{2,3} };
                for (auto& c : base) {
                    int rx = c[0], ry = c[1];
                    for (int r = 0; r < (erot & 3); ++r) { int t = rx; rx = 3 - ry; ry = t; }
                    map->setTile(ex + rx, ey + ry, 255);
                }
            }
            std::fprintf(stderr, "[capture] stamped eater shape %d at (%d,%d) rot %d\n",
                         eaterShape, ex, ey, erot & 3);
        }
    }

    map->commitInitialState();

    // --- Camera framed on the requested grid region ---
    // World region spans [x0..x1]*ts by [y0..y1]*ts. Choose zoom so the whole
    // region fits in resWxresH (letterbox if aspect ratios differ).
    Camera2D cam(static_cast<float>(resW), static_cast<float>(resH));
    const float regionW = (x1 - x0) * TILE_SIZE;
    const float regionH = (y1 - y0) * TILE_SIZE;
    cam.zoom     = std::min(resW / regionW, resH / regionH);
    cam.position = glm::vec2(x0 * TILE_SIZE, y0 * TILE_SIZE);
    cam.width    = static_cast<float>(resW);
    cam.height   = static_cast<float>(resH);

    glViewport(0, 0, resW, resH);
    glDisable(GL_BLEND);

    auto stepFn = [&]() {
        map->simulateAndWait();
        map->commitReadyChunks(cam);
    };

    // Если выход оканчивается на .gif — рисуем напрямую из клеток (без GL,
    // без размытия) через общий ExportGif. Удобно для headless-проверки.
    {
        const std::string out = outDir;
        if (out.size() >= 4 && out.compare(out.size() - 4, 4, ".gif") == 0) {
            GifExportParams gp;
            gp.x0 = x0; gp.y0 = y0; gp.x1 = x1; gp.y1 = y1;
            gp.scale   = std::max(1, resW / std::max(1, x1 - x0));
            gp.stride  = stride;
            gp.frames  = stopStep / stride + 1;
            gp.delayMs = 50;
            gp.path    = out;
            bool ok = ExportGif(*map, gp, stepFn);
            std::fprintf(stderr, "[capture] ExportGif -> %s : %s\n",
                         out.c_str(), ok ? "OK" : "FAILED");
            TaskScheduler::instance().shutdown();
            map.reset();
            ChunkRenderer::shutdownStatics();
            glfwMakeContextCurrent(nullptr);
            glfwDestroyWindow(win);
            glfwTerminate();
            return ok ? 0 : 1;
        }
    }

    std::vector<unsigned char> pixels(static_cast<size_t>(resW) * resH * 3);
    std::vector<unsigned char> flipped(pixels.size());

    int frameIdx = 0;
    for (int step = 0; step <= stopStep; ++step) {
        // Capture the current state every `stride` steps (and the final step).
        if (step % stride == 0 || step == stopStep) {
            glClearColor(0.f, 0.f, 0.f, 1.f);
            glClear(GL_COLOR_BUFFER_BIT);
            map->render(cam);
            glFinish();

            glReadPixels(0, 0, resW, resH, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

            // glReadPixels origin is bottom-left → flip vertically for top-down PPM.
            const int rowBytes = resW * 3;
            for (int row = 0; row < resH; ++row)
                std::copy(&pixels[(resH - 1 - row) * rowBytes],
                          &pixels[(resH - 1 - row) * rowBytes] + rowBytes,
                          &flipped[row * rowBytes]);

            char name[512];
            std::snprintf(name, sizeof(name), "%s/frame_%04d.ppm",
                          outDir.c_str(), frameIdx);
            writePPM(name, flipped, resW, resH);
            ++frameIdx;
        }

        if (step < stopStep) {
            map->simulateAndWait();
            map->commitReadyChunks(cam);
        }
    }

    std::fprintf(stderr, "[capture] wrote %d frames to %s\n", frameIdx, outDir.c_str());

    TaskScheduler::instance().shutdown();
    map.reset();
    ChunkRenderer::shutdownStatics();
    glfwMakeContextCurrent(nullptr);
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
