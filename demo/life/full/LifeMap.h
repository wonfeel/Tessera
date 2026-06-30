#pragma once
#include "engine/automaton/LifeLikeAutomaton.h"

class LifeMap : public LifeLikeAutomaton {
public:
    LifeMap(int w, int h, float ts, int chunkSize = 64);
    void randomize(float density = 0.3f);
private:
    static LifeRule makeRule();
};
