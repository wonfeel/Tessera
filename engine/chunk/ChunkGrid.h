// engine/chunk/ChunkGrid.h
#pragma once
#include "engine/chunk/ChunkCoord.h"
#include <glm/glm.hpp>

// Геометрия чанковой сетки: размеры мира, размер чанка, размер тайла и перевод
// мировых координат клетки в (координата чанка + локальный индекс внутри чанка).
// Вынесено из ChunkedTileMap, где этот расчёт (cx/cy/lx/ly) дублировался в
// setTile/setTileDirect/getTileState/getSimTileState.
struct CellLocation {
    ChunkCoord coord;
    int localX;
    int localY;
    int index(int chunkSize) const { return localY * chunkSize + localX; }
};

class ChunkGrid {
public:
    ChunkGrid(int totalWidth, int totalHeight, int chunkSize, float tileSize)
        : m_totalWidth(totalWidth), m_totalHeight(totalHeight),
          m_chunkSize(chunkSize), m_tileSize(tileSize) {}

    int   width()     const { return m_totalWidth; }
    int   height()    const { return m_totalHeight; }
    int   chunkSize() const { return m_chunkSize; }
    float tileSize()  const { return m_tileSize; }

    bool inBounds(int x, int y) const {
        return x >= 0 && x < m_totalWidth && y >= 0 && y < m_totalHeight;
    }

    // Перевод мировой клетки (x,y) в чанк + локальные координаты. Предполагает
    // inBounds(x,y); вызывающий проверяет границы заранее.
    CellLocation locate(int x, int y) const {
        return CellLocation{ ChunkCoord(x / m_chunkSize, y / m_chunkSize),
                             x % m_chunkSize, y % m_chunkSize };
    }

    ChunkCoord chunkOfWorldOffset(const glm::ivec2& worldOffset) const {
        return ChunkCoord(worldOffset.x / m_chunkSize, worldOffset.y / m_chunkSize);
    }

private:
    int   m_totalWidth, m_totalHeight;
    int   m_chunkSize;
    float m_tileSize;
};
