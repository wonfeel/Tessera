#pragma once
#include "engine/automaton/LifeLikeAutomaton.h"

// Демо-правило на основе "Жизни": живая клетка выживает при 2 или 3 соседях,
// мёртвая рождается при 3. Разные исходы дают разные оттенки (для красивой
// радужной палитры). Само правило задаётся таблицей, а считается на CPU или GPU
// в зависимости от сборки — см. LifeLikeAutomaton.
class MiniLifeTileMap : public LifeLikeAutomaton {
public:
    MiniLifeTileMap(int w, int h, float ts, int chunkSize = 64);
    void randomize(float density = 0.3f);

private:
    static LifeRule makeRule();
};
