#pragma once
#include <thread>
#include <atomic>
#include <chrono>

class ChunkedTileMap;

class SimulationManager {
public:
    explicit SimulationManager(ChunkedTileMap& map);
    SimulationManager(ChunkedTileMap& map, std::chrono::milliseconds interval);
    ~SimulationManager();

    void start();
    void stop();

protected:
    void run();
    ChunkedTileMap& m_map;
    std::atomic<bool> m_running{ false };
    std::chrono::milliseconds m_interval{ 0 };

private:
    std::thread m_thread;
};