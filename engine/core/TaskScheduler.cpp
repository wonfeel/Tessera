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
        try {
            task();
        } catch (...) {
            // Задача не должна убивать рабочий поток — иначе пул навсегда
            // остаётся на один поток меньше, и следующий parallelFor
            // гарантированно повиснет (латч не досчитается до нуля).
            // parallelFor() уже перехватывает исключение из body() и
            // перевыбрасывает его на вызывающем потоке — здесь просто не
            // даём потоку пула умереть, если исключение всё же дошло сюда.
        }
    }
}

// ---------- Latch ----------
// count_down() ОБЯЗАН декрементировать m_count и звать notify_all() под тем
// же m_mtx, что wait() держит при проверке предиката — иначе возможна гонка
// с уничтожением Latch: wait() мог бы проснуться (в т.ч. от spurious
// wakeup — это законно для condition_variable) и увидеть m_count==0 ДО того,
// как поток-нотификатор вообще попытается захватить m_mtx. Latch обычно живёт
// на стеке вызывающего (см. parallelFor() в ParallelFor.h) — как только
// wait() вернулся, этот объект начинает разрушаться (а на его месте на
// стеке тут же может появиться СЛЕДУЮЩИЙ Latch того же вызывающего потока).
// Опоздавший поток-нотификатор тогда лочит/разлочивает уже чужую память —
// возможно, mutex НОВОГО Latch, которым в этот момент владеет другой поток:
// ровно "unlock of mutex not owned by the current thread". Раньше здесь был
// lock-free fetch_sub снаружи лока (и ранний lock-free выход в wait()) —
// именно это открывало окно гонки; чем больше вызовов parallelFor() за
// кадр (после chunk-системы — их стало на порядок больше), тем чаще в него
// попадали. m_count можно было бы сделать обычным int (больше не нужен
// atomic, раз всегда под локом), оставлен atomic<int> для минимальности
// правки.
TaskScheduler::Latch::Latch(int count) : m_count(count) {}
void TaskScheduler::Latch::count_down() {
    std::lock_guard<std::mutex> lock(m_mtx);
    if (m_count.fetch_sub(1) == 1) {
        m_cv.notify_all();
    }
}
void TaskScheduler::Latch::wait() {
    std::unique_lock<std::mutex> lock(m_mtx);
    m_cv.wait(lock, [this] { return m_count.load() == 0; });
}