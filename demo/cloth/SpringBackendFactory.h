// demo/cloth/SpringBackendFactory.h
#pragma once
#include <memory>

// Развилка "есть GPU / нет GPU" для физики пружин — тот же паттерн, что
// engine/simulation/SimulationBackendFactory.cpp использует для клеточного
// автомата (см. MakeSimulationBackend()), только без общего полиморфного
// интерфейса (ISimulationBackend не подходит модели узел/ребро — см.
// design-план сессии).
//
// Вся сигнатура целиком под #ifdef FE_CUDA_ENABLED (не просто тело функции):
// unique_ptr<CudaSpringBackend> требует ПОЛНОГО типа (с реальным
// ~CudaSpringBackend()) в каждом месте, где объект такого unique_ptr
// создаётся/уничтожается/переприсваивается — а определение деструктора
// живёт только в CudaSpringBackend.cu, которая в CPU-only сборке
// (FE_USE_CUDA=OFF) вообще не компилируется. Поэтому в такой сборке эта
// функция просто не существует — вызывающий (SpringNetwork) сам гейтит
// вызов/член m_gpuBackend тем же макросом, см. SpringNetwork.h/.cpp.
#ifdef FE_CUDA_ENABLED
class CudaSpringBackend;
std::unique_ptr<CudaSpringBackend> MakeSpringGpuBackend();
#endif
