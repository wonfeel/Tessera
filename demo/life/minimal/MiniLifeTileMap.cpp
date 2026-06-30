#include "MiniLifeTileMap.h"
#include "engine/graphics/Palette.h"
#include <random>

LifeRule MiniLifeTileMap::makeRule() {
    // Эквивалент прежней transition():
    //   живая клетка: 2 соседа -> 192, 3 соседа -> 255, иначе -> 0 (смерть)
    //   мёртвая клетка: 3 соседа -> 64 (рождение), иначе -> 0
    LifeRule r{};
    r.table[1][2] = 192;
    r.table[1][3] = 255;
    r.table[0][3] = 64;
    return r;
}

MiniLifeTileMap::MiniLifeTileMap(int w, int h, float ts, int chunkSize)
    : LifeLikeAutomaton(w, h, chunkSize, ts, makeRule()) {
    // Радужная палитра даёт яркие цвета. Если нужна чёрно-белая — замените на Palette::Grayscale()
    setPalette(Palette::DefaultRainbow());
}

void MiniLifeTileMap::randomize(float density) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    for (int y = 0; y < getHeight(); ++y)
        for (int x = 0; x < getWidth(); ++x)
            if (dist(rng) < density)
                setTile(x, y, 255);   // начальное состояние — 255 (белый в Grayscale, яркий в радуге)
    commitInitialState();
}
