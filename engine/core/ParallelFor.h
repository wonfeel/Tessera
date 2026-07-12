// engine/core/ParallelFor.h
#pragma once
#include "engine/core/TaskScheduler.h"
#include <algorithm>
#include <functional>
#include <mutex>
#include <exception>

// Раскладывает [0,count) по потокам TaskScheduler и ждёт завершения всех (тот
// же Latch-паттерн, что использует SimulationCoordinator для барьера фаз).
// На маленьких count (< 2000) не плодит задачи — раздача по потокам дороже
// последовательного прохода. Общая утилита для физики (cloth SpringNetwork,
// light LightField) и для сборки вершинных буферов под рендер — все гоняют
// одну и ту же форму цикла по независимым диапазонам. Раньше жила в
// demo/cloth/ParallelFor.h — вынесена в engine/, когда появилась вторая
// демка (demo/light), которой нужна та же утилита.
inline void parallelFor(int count, const std::function<void(int begin, int end, int threadIdx)>& body) {
    size_t numThreads = std::max<size_t>(1, TaskScheduler::instance().thread_count());
    if (numThreads <= 1 || count < 2000) { body(0, count, 0); return; }

    int per = (count + static_cast<int>(numThreads) - 1) / static_cast<int>(numThreads);
    TaskScheduler::Latch latch(static_cast<int>(numThreads));
    std::mutex excMutex;
    std::exception_ptr firstException;
    for (size_t t = 0; t < numThreads; ++t) {
        int begin = static_cast<int>(t) * per;
        int end = std::min(count, begin + per);
        if (begin >= end) { latch.count_down(); continue; }
        TaskScheduler::instance().schedule([begin, end, t, &body, &latch, &excMutex, &firstException]() {
            // count_down() обязан выполниться, даже если body() бросит
            // исключение — иначе latch.wait() ниже повиснет навсегда, а
            // поскольку parallelFor вызывается изнутри step() под m_mutex,
            // это зависание блокирует и рендер-поток на snapshot() — это и
            // есть механизм "дедлока в случайный момент". NB: НЕ защита от
            // NaN/Inf в вычислениях — по умолчанию C++ не кидает исключений
            // на NaN/Inf-арифметику (нужен явный аппаратный trap —
            // feenableexcept/_controlfp, здесь не включён), они просто молча
            // текут дальше по буферам. От этого отдельно защищает
            // isfinite-клэмп в step(). Этот catch(...) ловит другие
            // исключения (bad_alloc и т.п.), не FP.
            try {
                body(begin, end, static_cast<int>(t));
            } catch (...) {
                std::lock_guard<std::mutex> lock(excMutex);
                if (!firstException) firstException = std::current_exception();
            }
            latch.count_down();
        });
    }
    latch.wait();
    if (firstException) std::rethrow_exception(firstException);
}
