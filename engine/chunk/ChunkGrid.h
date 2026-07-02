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

    // width()/height() больше не жёсткая граница мира (поле безгранично в обе
    // стороны по каждой оси) — это только номинальный размер "стартовой области",
    // которым пользуются randomize()/центрирование камеры/центрирование паттерна.
    int   width()     const { return m_totalWidth; }
    int   height()    const { return m_totalHeight; }
    int   chunkSize() const { return m_chunkSize; }
    float tileSize()  const { return m_tileSize; }

    // Перевод мировой клетки (x,y), в т.ч. отрицательной, в чанк + локальные
    // координаты. Обычное int-деление/остаток округляют к нулю, а не вниз,
    // поэтому для x<0 даёт неверный чанк/индекс (напр. x=-1, chunkSize=64:
    // -1/64==0, -1%64==-1 — оба неверны). Нужно floor-деление.
    CellLocation locate(int x, int y) const {
        return CellLocation{ ChunkCoord(floorDiv(x, m_chunkSize), floorDiv(y, m_chunkSize)),
                             floorMod(x, m_chunkSize), floorMod(y, m_chunkSize) };
    }

    ChunkCoord chunkOfWorldOffset(const glm::ivec2& worldOffset) const {
        return ChunkCoord(floorDiv(worldOffset.x, m_chunkSize), floorDiv(worldOffset.y, m_chunkSize));
    }

private:
    // chunkSize здесь всегда положительный, поэтому достаточно простой поправки
    // на отрицательный остаток, а не общей формулы floor-деления.
    static int floorDiv(int a, int chunkSize) {
        int q = a / chunkSize;
        if (a % chunkSize != 0 && a < 0) --q;
        return q;
    }
    static int floorMod(int a, int chunkSize) {
        int r = a % chunkSize;
        if (r < 0) r += chunkSize;
        return r;
    }

    int   m_totalWidth, m_totalHeight;
    int   m_chunkSize;
    float m_tileSize;
};
