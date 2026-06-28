#pragma once
class Camera2D;
class IRenderable {
public:
    virtual ~IRenderable() = default;
    virtual void render(const Camera2D& camera) = 0;
};