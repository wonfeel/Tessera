// engine/tilemap/ITileMap.h
#pragma once
#include <string>

class ITileMap {
public:
    virtual ~ITileMap() = default;
    virtual int getWidth() const = 0;
    virtual int getHeight() const = 0;
    virtual float getTileSize() const = 0;
    virtual std::string getTileInfo(int x, int y) const = 0;
    virtual void setTile(int x, int y, int state) = 0;
    virtual int getTileState(int x, int y) const = 0;
};