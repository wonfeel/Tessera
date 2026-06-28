// engine/tilemap/IColorTileMap.h
#pragma once
#include "ITileMap.h"
#include <vector>
#include <glm/glm.hpp>

class IColorTileMap : public ITileMap {
public:
    virtual const std::vector<glm::vec3>& getTileColors() const = 0;
    virtual bool isDirty() const = 0;
    virtual void clearDirty() = 0;
};