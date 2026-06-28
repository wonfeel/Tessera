#include "SimulationManager.h"
#include "engine/chunk/ChunkedTileMap.h"
#include <thread>

SimulationManager::SimulationManager(ChunkedTileMap& map)
    : m_map(map) {
}

SimulationManager::SimulationManager(ChunkedTileMap& map, std::chrono::milliseconds interval)
    : m_map(map), m_interval(interval) {
}

SimulationManager::~SimulationManager() { stop(); }

void SimulationManager::start() {
    m_running = true;
    m_thread = std::thread(&SimulationManager::run, this);
}

void SimulationManager::stop() {
    m_running = false;
    if (m_thread.joinable()) m_thread.join();
}

void SimulationManager::run() {
    while (m_running) {
        m_map.simulateActiveChunks();
        if (m_interval.count() > 0)
            std::this_thread::sleep_for(m_interval);
    }
}