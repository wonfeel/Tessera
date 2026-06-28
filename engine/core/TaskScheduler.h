// engine/core/TaskScheduler.h
#pragma once
#include <functional>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

class TaskScheduler {
public:
    using Task = std::function<void()>;

    static TaskScheduler& instance();
    void initialize(size_t numThreads = 0);
    void shutdown();
    void schedule(Task task);
    void schedule_bulk(std::vector<Task>& tasks);

    size_t thread_count() const { return m_threads.size(); }

    class Latch {
    public:
        Latch(int count);
        void count_down();
        void wait();
    private:
        std::atomic<int> m_count;
        std::mutex m_mtx;
        std::condition_variable m_cv;
    };

private:
    TaskScheduler() = default;
    ~TaskScheduler();
    void workerLoop();

    std::vector<std::thread> m_threads;
    std::queue<Task> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::atomic<bool> m_running{ false };
};