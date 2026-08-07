// tests/thread_safety/main.cpp
//
// Headless-стресс параллельных структур движка. Смысл цели - не в том, что
// она "проверяет результат" (проверок тут минимум), а в том, что её надо
// гонять под ThreadSanitizer:
//
//     cmake --preset linux-tsan && cmake --build out/build/linux-tsan
//     ./out/build/linux-tsan/Test_thread_safety
//
// TSan печатает WARNING на КАЖДУЮ найденную гонку; TSAN_OPTIONS=halt_on_error=1
// (так он и запускается в CI) превращает первую же в ненулевой код возврата.
// Без санитайзера тест просто прогоняет сценарии и всегда проходит - гонка
// без TSan не наблюдаема, потому и заводилась в этом коде незаметно.
//
// Сценарии подобраны по местам, где гонки тут УЖЕ были и чинились руками:
//
//   1. TaskScheduler::initialize - воркеры стартовали до выставления
//      m_running, и первый же успевший проснуться поток видел false и
//      выходил, а пул молча оказывался меньше заявленного.
//   2. TaskScheduler::shutdown - m_running сбрасывался вне мьютекса, то есть
//      мог измениться ровно между проверкой предиката и засыпанием воркера:
//      классическая потерянная побудка, поток не просыпался никогда.
//   3. ChunkStore::getOrCreate - вставка шла под shared-блокировкой, так что
//      одновременная вставка из двух потоков могла вызвать рехеш
//      unordered_map прямо под идущим параллельно find(): порча памяти, а не
//      просто устаревшее чтение.
//   4. ChunkStore::markActive - m_activeChunks правился вообще без
//      блокировки store (setTile держал мьютекс КОНКРЕТНОГО чанка, который
//      это множество ничем не защищает).
//
// Exit 0 = сценарии отработали и внутренние инварианты сошлись.

#include "engine/core/TaskScheduler.h"
#include "engine/chunk/ChunkStore.h"
#include "engine/chunk/Chunk.h"
#include "engine/chunk/ChunkCoord.h"

#include <atomic>
#include <cstdio>
#include <thread>
#include <vector>

namespace {

constexpr int kChunkSize = 16;
int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("%-46s %s\n", what, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

// ---------------------------------------------------------------------------
// 1+2. Тред-пул: старт, поток задач с нескольких сторон, остановка.
// ---------------------------------------------------------------------------
// Цикл, а не один прогон: и гонка на старте, и потерянная побудка на
// остановке - это узкие окна в несколько инструкций. TSan ловит их не по
// факту "неудачно совпало", а по отсутствию happens-before, но сам код всё
// равно надо ПРОВЕСТИ через эти окна, причём с разными исходами планировщика.
void stressScheduler() {
    constexpr int kRounds = 20;
    constexpr int kProducers = 4;
    constexpr int kTasksPerProducer = 250;

    for (int round = 0; round < kRounds; ++round) {
        auto& sched = TaskScheduler::instance();
        sched.initialize(4);

        std::atomic<int> done{0};
        TaskScheduler::Latch latch(kProducers * kTasksPerProducer);

        // Задачи кладут посторонние потоки, а не главный: schedule() обязан
        // быть безопасен из любого потока, и именно так его зовёт симуляция.
        std::vector<std::thread> producers;
        for (int p = 0; p < kProducers; ++p) {
            producers.emplace_back([&] {
                for (int i = 0; i < kTasksPerProducer; ++i) {
                    sched.schedule([&] {
                        done.fetch_add(1, std::memory_order_relaxed);
                        latch.count_down();
                    });
                }
            });
        }
        for (auto& t : producers) t.join();

        // Ждём именно латчем: он же используется симуляцией как фазовый
        // барьер, так что заодно проверяется и он.
        latch.wait();

        const int seen = done.load(std::memory_order_relaxed);
        if (seen != kProducers * kTasksPerProducer) {
            std::printf("  round %d: executed %d of %d tasks\n",
                        round, seen, kProducers * kTasksPerProducer);
            ++g_failures;
        }

        // shutdown в цикле - именно здесь жила потерянная побудка: если
        // воркер уснул после проверки предиката, но до notify_all, он не
        // просыпался и join() висел навсегда.
        sched.shutdown();
    }
    check(g_failures == 0, "scheduler: start/schedule/latch/shutdown");
}

// ---------------------------------------------------------------------------
// 3+4. ChunkStore: одновременное создание, чтение и пометка активности.
// ---------------------------------------------------------------------------
void stressChunkStore() {
    constexpr int kThreads = 6;
    constexpr int kCoords = 64;
    constexpr int kIterations = 300;

    ChunkStore store;
    // Фабрика создаёт настоящий Chunk. Ни одного GL-вызова тут не
    // происходит: ChunkRenderer откладывает создание VAO/VBO до первого
    // рендера (ensureGLReady), а деструктор ничего не удаляет, если до него
    // не дошло - поэтому чанки живут и умирают headless.
    // Счётчик вызовов фабрики - это и есть проверка double-checked пути.
    // Считать размер store.map() бессмысленно: он по определению не может
    // превысить число различных ключей, сколько бы лишних вставок ни
    // произошло. А вот ЛИШНИЙ вызов фабрики означает, что два потока прошли
    // повторную проверку под unique-блокировкой и один построенный чанк
    // (вместе с записанным в него состоянием) был молча выброшен.
    std::atomic<int> factoryCalls{0};
    ChunkStore::Factory factory = [&](ChunkCoord c) {
        factoryCalls.fetch_add(1, std::memory_order_relaxed);
        return std::make_shared<Chunk>(c, glm::ivec2(c.x() * kChunkSize, c.y() * kChunkSize),
                                       1.0f, nullptr, kChunkSize);
    };

    std::atomic<int> created{0};
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t] {
            for (int i = 0; i < kIterations; ++i) {
                // Координаты потоков ПЕРЕСЕКАЮТСЯ намеренно: смысл теста в
                // том, чтобы два потока целились в один и тот же
                // отсутствующий чанк и оба пошли по пути создания.
                const int idx = (i * 7 + t) % kCoords;
                ChunkCoord coord(idx % 8, idx / 8);

                Chunk* chunk = store.getOrCreate(coord, factory);
                if (chunk) created.fetch_add(1, std::memory_order_relaxed);

                // Читатель, идущий параллельно вставкам - тот самый find(),
                // которому рехеш ломал итераторы.
                if (auto shared = store.getShared(ChunkCoord((idx + 3) % 8, idx / 8)))
                    shared->liveCells.load(std::memory_order_acquire);

                // Пометка активности из потока, который никакой блокировки
                // store не держит - ровно как setTile/paintTile.
                store.markActive(coord);
            }
        });
    }
    for (auto& w : workers) w.join();

    check(created.load() == kThreads * kIterations,
          "chunk store: concurrent getOrCreate never null");

    const std::size_t chunkCount = store.map().size();
    check(chunkCount == static_cast<std::size_t>(kCoords),
          "chunk store: every coordinate got a chunk");
    check(factoryCalls.load() == kCoords,
          "chunk store: factory ran exactly once per coordinate");
    check(store.active().size() == chunkCount,
          "chunk store: active set matches created chunks");
}

}  // namespace

int main() {
    std::printf("=== Tessera thread-safety stress (run under TSan) ===\n");
    stressScheduler();
    stressChunkStore();

    if (g_failures) {
        std::printf("\n%d check(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("\nAll checks passed.\n");
    return 0;
}
