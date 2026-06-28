#pragma once
#include "engine/graphics/IRenderable.h"
#include "engine/tilemap/ITileMap.h"

class ITileMapRenderer : public IRenderable {
public:
    virtual void setTileMap(ITileMap* map) = 0;
};