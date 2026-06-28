#include "TaskScheduler.h"
#include <algorithm>

TaskScheduler& TaskScheduler::instance() {
    static TaskScheduler s;
    return s;
}

TaskScheduler::~TaskScheduler() { shutdown(); }

void TaskScheduler::initialize(size_t numThreads) {
    if (m_running) return;
    if (numThreads == 0) numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0) numThreads = 2;

    for (size_t i = 0; i < numThreads; ++i)
        m_threads.emplace_back(&TaskScheduler::workerLoop, this);
    m_running = true;
}

void TaskScheduler::shutdown() {
    if (!m_running) return;
    m_running = false;
    m_cv.notify_all();
    for (auto& t : m_threads) {
        if (t.joinable()) t.join();
    }
    m_threads.clear();
}

void TaskScheduler::schedule(Task task) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(std::move(task));
    }
    m_cv.notify_one();
}

// Улучшенное распределение: при малом количестве задач раздаём циклически,
// чтобы каждый поток получил хотя бы одну задачу.
void TaskScheduler::schedule_bulk(std::vector<Task>& tasks) {
    if (tasks.empty()) return;
    size_t numTasks = tasks.size();
    size_t numWorkers = m_threads.size();
    if (numWorkers == 0) return;

    // Если задач меньше, чем потоков, распределяем по одной циклически
    if (numTasks <= numWorkers) {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (size_t i = 0; i < numTasks; ++i) {
            m_queue.push(std::move(tasks[i]));
        }
        m_cv.notify_all();
        return;
    }

    // Иначе делим на примерно равные порции, но стараемся дать всем
    size_t base = numTasks / numWorkers;
    size_t remainder = numTasks % numWorkers;
    size_t idx = 0;
    for (size_t w = 0; w < numWorkers; ++w) {
        size_t count = base + (w < remainder ? 1 : 0);
        if (count == 0) continue;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            for (size_t i = 0; i < count; ++i) {
                m_queue.push(std::move(tasks[idx++]));
            }
        }
    }
    m_cv.notify_all();
}

void TaskScheduler::workerLoop() {
    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [this] { return !m_queue.empty() || !m_running; });
            if (!m_running && m_queue.empty()) return;
            task = std::move(m_queue.front());
            m_queue.pop();
        }
        task();
    }
}

// ---------- Latch ----------
TaskScheduler::Latch::Latch(int count) : m_count(count) {}
void TaskScheduler::Latch::count_down() {
    if (m_count.fetch_sub(1) == 1) {
        std::lock_guard<std::mutex> lock(m_mtx);
        m_cv.notify_all();
    }
}
void TaskScheduler::Latch::wait() {
    if (m_count.load() == 0) return;
    std::unique_lock<std::mutex> lock(m_mtx);
    m_cv.wait(lock, [this] { return m_count.load() == 0; });
}