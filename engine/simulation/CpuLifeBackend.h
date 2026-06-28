// engine/simulation/CpuLifeBackend.h
#pragma once
#include "engine/simulation/ISimulationBackend.h"

// Эталонная реализация шага автомата на CPU. Без состояния, поэтому
// автоматически потокобезопасна.
class CpuLifeBackend : public ISimulationBackend {
public:
    void simulate(const uint8_t* ext, int extW,
                  uint8_t* out, int S,
                  const LifeRule& rule) override;

    const char* name() const override { return "CPU"; }
};
