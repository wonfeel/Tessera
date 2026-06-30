// engine/utils/GifExport.cpp
//
// Прямой экспорт GIF из состояния клеток. gif.h подключается ТОЛЬКО здесь —
// он содержит определения функций в заголовке, поэтому второй #include в другом
// .cpp дал бы дублирующиеся символы при линковке.
#include "engine/utils/GifExport.h"
#include "engine/chunk/ChunkedTileMap.h"
#include "engine/chunk/ChunkRenderer.h"

#include "libs/gif/gif.h"

#include <glm/glm.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace {
inline uint8_t toByte(float v) {
    return static_cast<uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f);
}
} // namespace

bool ExportGif(ChunkedTileMap& map, const GifExportParams& p,
               const std::function<void()>& stepFn) {
    const int rw = p.x1 - p.x0;
    const int rh = p.y1 - p.y0;
    if (rw <= 0 || rh <= 0 || p.scale <= 0 || p.frames <= 0)
        return false;

    const int W = rw * p.scale;
    const int H = rh * p.scale;
    // Защита от случайных гигантских буферов.
    if (static_cast<long long>(W) * H > 4000LL * 4000LL)
        return false;

    const auto& pal = ChunkRenderer::getGlobalPalette();

    // value (0..255) -> упакованный RGB-цвет блока.
    auto colorOf = [&](int v) -> std::array<uint8_t, 3> {
        if (v >= 0 && v < static_cast<int>(pal.size())) {
            const glm::vec3& c = pal[v];
            return { toByte(c.r), toByte(c.g), toByte(c.b) };
        }
        return { 0, 0, 0 };
    };

    std::vector<uint8_t> frame(static_cast<size_t>(W) * H * 4, 0);
    const int delayCs = std::max(2, p.delayMs / 10);   // gif.h задержка в сантисекундах

    GifWriter writer{};
    if (!GifBegin(&writer, p.path.c_str(), static_cast<uint32_t>(W),
                  static_cast<uint32_t>(H), static_cast<uint32_t>(delayCs)))
        return false;

    for (int f = 0; f < p.frames; ++f) {
        // Рисуем текущее состояние: одна клетка -> блок scale×scale.
        for (int cy = 0; cy < rh; ++cy) {
            for (int cx = 0; cx < rw; ++cx) {
                const int v = map.getTileState(p.x0 + cx, p.y0 + cy);
                const auto col = colorOf(v);
                for (int sy = 0; sy < p.scale; ++sy) {
                    const int py = cy * p.scale + sy;
                    uint8_t* row = &frame[(static_cast<size_t>(py) * W
                                           + static_cast<size_t>(cx) * p.scale) * 4];
                    for (int sx = 0; sx < p.scale; ++sx) {
                        row[sx * 4 + 0] = col[0];
                        row[sx * 4 + 1] = col[1];
                        row[sx * 4 + 2] = col[2];
                        row[sx * 4 + 3] = 255;
                    }
                }
            }
        }

        GifWriteFrame(&writer, frame.data(), static_cast<uint32_t>(W),
                      static_cast<uint32_t>(H), static_cast<uint32_t>(delayCs));

        if (f < p.frames - 1)
            for (int s = 0; s < p.stride; ++s)
                stepFn();
    }

    GifEnd(&writer);
    return true;
}
