// engine/tilemap/IIndexedTileMap.h
#pragma once
#include "ITileMap.h"
#include <vector>
#include <cstdint>

class IIndexedTileMap : public ITileMap {
public:
    virtual const std::vector<uint8_t>& getTileIndices() const = 0;
    virtual bool isDirty() const = 0;
    virtual void clearDirty() = 0;
};