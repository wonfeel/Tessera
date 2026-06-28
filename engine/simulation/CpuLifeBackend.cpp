// engine/simulation/CpuLifeBackend.cpp
#include "engine/simulation/CpuLifeBackend.h"

void CpuLifeBackend::simulate(const uint8_t* ext, int extW,
                              uint8_t* out, int S,
                              const LifeRule& rule) {
    // ext имеет границу border=1, поэтому центр клетки (x,y) лежит в ext[(y+1)*extW + (x+1)].
    for (int y = 0; y < S; ++y) {
        for (int x = 0; x < S; ++x) {
            const int cx = x + 1;
            const int cy = y + 1;

            // Считаем живых соседей (8 штук), не включая центр.
            int alive = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (ext[(cy + dy) * extW + (cx + dx)] != 0) ++alive;
                }
            }

            const int center = (ext[cy * extW + cx] != 0) ? 1 : 0;
            out[y * S + x] = rule.table[center][alive];
        }
    }
}
