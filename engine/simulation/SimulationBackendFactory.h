// engine/simulation/SimulationBackendFactory.h
#pragma once
#include "engine/simulation/ISimulationBackend.h"
#include <memory>

// Создаёт оптимальный доступный бэкенд:
//   - если проект собран с CUDA (FE_CUDA_ENABLED) И на машине есть GPU -> CUDA;
//   - иначе -> CPU.
// Никогда не возвращает nullptr.
std::unique_ptr<ISimulationBackend> MakeSimulationBackend();
