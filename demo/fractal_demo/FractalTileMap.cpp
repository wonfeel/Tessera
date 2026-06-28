// demo/fractal_demo/FractalTileMap.cpp
#include "FractalTileMap.h"
#include "engine/core/TaskScheduler.h"
#include <complex>

static void computeRow(int y, int width, int height, double centerX, double centerY, double zoom, int maxIter, uint8_t* row) {
    for (int x = 0; x < width; ++x) {
        double cx = (x - width / 2.0 + 0.5) / zoom + centerX;
        double cy = (y - height / 2.0 + 0.5) / zoom + centerY;

        std::complex<double> z(0, 0);
        int iter = 0;
        while (iter < maxIter && std::norm(z) <= 4.0) {
            z = z * z + std::complex<double>(cx, cy);
            ++iter;
        }

        // Масштабируем количество итераций в диапазон 1..255, 0 – для точек внутри множества
        uint8_t index = (iter == maxIter) ? 0 : static_cast<uint8_t>(1 + (iter * 254) / maxIter);
        row[x] = index;
    }
}

FractalTileMap::FractalTileMap(int width, int height, float tileSize)
    : m_width(width), m_height(height), m_tileSize(tileSize),
    m_indices(width* height, 0)
{
}

int FractalTileMap::getWidth() const { return m_width; }
int FractalTileMap::getHeight() const { return m_height; }
float FractalTileMap::getTileSize() const { return m_tileSize; }
std::string FractalTileMap::getTileInfo(int x, int y) const { return {}; }
void FractalTileMap::setTile(int, int, int) {}
int FractalTileMap::getTileState(int x, int y) const {
    if (x < 0 || x >= m_width || y < 0 || y >= m_height) return 0;
    return m_indices[y * m_width + x];
}

const std::vector<uint8_t>& FractalTileMap::getTileIndices() const {
    return m_indices;
}
bool FractalTileMap::isDirty() const { return m_dirty; }
void FractalTileMap::clearDirty() { m_dirty = false; }

void FractalTileMap::computeMandelbrotParallel(double centerX, double centerY, double zoom) {
    const int maxIter = 100;
    auto& scheduler = TaskScheduler::instance();
    size_t numWorkers = scheduler.thread_count();
    if (numWorkers == 0) {
        for (int y = 0; y < m_height; ++y)
            computeRow(y, m_width, m_height, centerX, centerY, zoom, maxIter, &m_indices[y * m_width]);
        m_dirty = true;
        return;
    }

    TaskScheduler::Latch latch(m_height);
    std::vector<TaskScheduler::Task> tasks;
    tasks.reserve(m_height);
    for (int y = 0; y < m_height; ++y) {
        tasks.push_back([this, y, centerX, centerY, zoom, maxIter, &latch]() {
            computeRow(y, m_width, m_height, centerX, centerY, zoom, maxIter, &m_indices[y * m_width]);
            latch.count_down();
            });
    }
    scheduler.schedule_bulk(tasks);
    latch.wait();
    m_dirty = true;
}