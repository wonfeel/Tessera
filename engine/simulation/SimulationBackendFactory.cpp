// engine/simulation/SimulationBackendFactory.cpp
#include "engine/simulation/SimulationBackendFactory.h"
#include "engine/simulation/CpuLifeBackend.h"
#include <cstdio>

// ВОТ ОНА — развилка "есть GPU / нет GPU".
// FE_CUDA_ENABLED определяется в CMake только когда найден CUDA-компилятор.
// Если макрос не задан, CUDA-код вообще не компилируется и не линкуется,
// поэтому проект собирается на машинах без NVIDIA/CUDA.
#ifdef FE_CUDA_ENABLED
#include "engine/simulation/CudaLifeBackend.h"
#endif

std::unique_ptr<ISimulationBackend> MakeSimulationBackend() {
#ifdef FE_CUDA_ENABLED
    if (CudaLifeBackend::isAvailable()) {
        std::fprintf(stderr, "[FieldEngine] Simulation backend: CUDA (GPU)\n");
        return std::make_unique<CudaLifeBackend>();
    }
    std::fprintf(stderr, "[FieldEngine] CUDA compiled, but no GPU found -> CPU fallback\n");
#else
    std::fprintf(stderr, "[FieldEngine] Simulation backend: CPU (built without CUDA)\n");
#endif
    return std::make_unique<CpuLifeBackend>();
}
