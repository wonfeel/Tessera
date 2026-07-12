// demo/cloth/SpringBackendFactory.cpp
#include "demo/cloth/SpringBackendFactory.h"
#include <cstdio>

#ifdef FE_CUDA_ENABLED
#include "engine/simulation/CudaSpringBackend.h"

std::unique_ptr<CudaSpringBackend> MakeSpringGpuBackend() {
    if (CudaSpringBackend::isAvailable()) {
        std::fprintf(stderr, "[Tessera] Springs physics backend: CUDA (GPU)\n");
        return std::make_unique<CudaSpringBackend>();
    }
    std::fprintf(stderr, "[Tessera] CUDA compiled, but no GPU found -> CPU spring backend\n");
    return nullptr;
}
#endif
