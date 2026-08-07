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

    // m_running ОБЯЗАН стать true ДО создания потоков. Раньше он ставился
    // после цикла - и воркер, которому ОС успевала отдать квант раньше, чем
    // выполнялась эта строка, видел в предикате wait() (!m_running == true),
    // сразу проходил его, упирался в `!m_running && m_queue.empty()` и молча
    // выходил. Поток умирал, но m_threads его запись сохраняла, поэтому
    // thread_count() продолжал возвращать полное N: parallelFor() создавал N
    // задач, Latch ждал N вызовов count_down(), часть которых уже некому было
    // сделать - вечное зависание. Не стреляло только потому, что создание
    // потока на порядки дороже одной записи в atomic, то есть держалось на
    // удаче планировщика, а не на инварианте.
    m_running = true;
    for (size_t i = 0; i < numThreads; ++i)
        m_threads.emplace_back(&TaskScheduler::workerLoop, this);
}

void TaskScheduler::shutdown() {
    if (!m_running) return;
    {
        // Менять условие, которое проверяет предикат m_cv.wait(), нужно ПОД
        // тем же мьютексом, что держит ожидающий при проверке - иначе classic
        // lost wakeup: воркер уже захватил m_mutex, вычислил предикат как
        // false, но ещё не успел атомарно освободить лок и заснуть; notify_all
        // в этот момент не будит никого (спящих ещё нет), а следом воркер
        // засыпает уже навсегда - join() ниже виснет. То, что m_running сам по
        // себе atomic, от этой гонки не спасает: она не о разрыве чтения, а о
        // порядке относительно засыпания.
        std::lock_guard<std::mutex> lock(m_mutex);
        m_running = false;
    }
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

// schedule_bulk() удалён. Он обещал в комментарии "циклическую раздачу задач
// по потокам", но раздачи не делал и сделать не мог: очередь у пула ОДНА
// общая, и обе его ветки просто пушили в неё все задачи подряд, давая
// одинаковый результат. Отличие было лишь в том, что "оптимизированная"
// ветка захватывала мьютекс numWorkers раз вместо одного, то есть при том же
// поведении работала медленнее. Вызывающих у функции не было ни одного
// (координатор зовёт schedule() в цикле), так что чинить её было незачем.
//
// Если contention на общей очереди когда-нибудь станет измеренной проблемой,
// правильный ответ - отдельные очереди на воркер плюс work stealing, а не
// эта функция.

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