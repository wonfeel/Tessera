// engine/simulation/WorldOccupancy.h
#pragma once
#include <atomic>
#include <vector>
#include <cstdint>

class WorldOccupancy {
public:
    WorldOccupancy(int totalWidth, int totalHeight)
        : m_width(totalWidth), m_height(totalHeight),
        m_grid(totalWidth* totalHeight) {
    }

    // Попытка захвата. plantID=0 означает свободную клетку.
    bool tryClaim(int x, int y, uint32_t plantID, uint32_t cellIndex) {
        if (x < 0 || x >= m_width || y < 0 || y >= m_height) return false;
        // Используем 64 бита: старшие 32 бита — plantID, младшие 32 — cellIndex
        uint64_t newVal = (static_cast<uint64_t>(plantID) << 32) | (cellIndex & 0xFFFFFFFF);
        uint64_t expected = 0;
        size_t idx = y * m_width + x;
        return m_grid[idx].compare_exchange_strong(expected, newVal,
            std::memory_order_acquire, std::memory_order_relaxed);
    }

    void release(int x, int y) {
        if (x < 0 || x >= m_width || y < 0 || y >= m_height) return;
        size_t idx = y * m_width + x;
        m_grid[idx].store(0, std::memory_order_release);
    }

    bool isFree(int x, int y) const {
        if (x < 0 || x >= m_width || y < 0 || y >= m_height) return false;
        size_t idx = y * m_width + x;
        return m_grid[idx].load(std::memory_order_acquire) == 0;
    }

    // Получить владельца (plantID, cellIndex)
    std::pair<uint32_t, uint32_t> getOwner(int x, int y) const {
        size_t idx = y * m_width + x;
        uint64_t val = m_grid[idx].load(std::memory_order_acquire);
        return { static_cast<uint32_t>(val >> 32), static_cast<uint32_t>(val & 0xFFFFFFFF) };
    }

private:
    int m_width, m_height;
    std::vector<std::atomic<uint64_t>> m_grid;   // 64-битный атомарный вектор
};