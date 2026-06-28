#pragma once
#include "engine/tilemap/IndexedTileMap.h"
#include <vector>
#include <glm/glm.hpp>

class FractalTileMap : public IIndexedTileMap {
public:
    FractalTileMap(int width, int height, float tileSize);

    // ITileMap
    int getWidth() const override;
    int getHeight() const override;
    float getTileSize() const override;
    std::string getTileInfo(int x, int y) const override;
    void setTile(int x, int y, int state) override;
    int getTileState(int x, int y) const override;

    // IIndexedTileMap
    const std::vector<uint8_t>& getTileIndices() const override;
    bool isDirty() const override;
    void clearDirty() override;

    void computeMandelbrotParallel(double centerX, double centerY, double zoom);

private:
    int m_width, m_height;
    float m_tileSize;
    std::vector<uint8_t> m_indices;
    mutable bool m_dirty = false;
};