#include "LifeMap.h"
#include "engine/graphics/Palette.h"
#include <random>

LifeRule LifeMap::makeRule() {
    LifeRule r{};
    r.table[1][2] = 192;
    r.table[1][3] = 255;
    r.table[0][3] = 64;
    return r;
}

LifeMap::LifeMap(int w, int h, float ts, int chunkSize)
    : LifeLikeAutomaton(w, h, chunkSize, ts, makeRule()) {
    setPalette(Palette::DefaultRainbow());
}

void LifeMap::randomize(float density) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.f, 1.f);
    for (int y = 0; y < getHeight(); ++y)
        for (int x = 0; x < getWidth(); ++x)
            if (dist(rng) < density)
                setTile(x, y, 255);
    commitInitialState();
}
