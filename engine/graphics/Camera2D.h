#pragma once
#include <glm/glm.hpp>

class Camera2D {
public:
    Camera2D() : width(0), height(0), zoom(1.0f) {}
    Camera2D(float screenWidth, float screenHeight);

    glm::mat4 getViewProjectionMatrix() const;
    void processKeyboard(float dt, bool w, bool s, bool a, bool d);
    void processScroll(double yoffset, float mouseScreenX, float mouseScreenY);

    glm::vec2 screenToWorld(float screenX, float screenY) const {
        return position + glm::vec2(screenX, screenY) / zoom;
    }

    void getVisibleAABB(glm::vec2& outMin, glm::vec2& outMax) const {
        outMin = screenToWorld(0.0f, 0.0f);
        outMax = screenToWorld(width, height);
    }

    glm::vec2 position = glm::vec2(0.0f);
    float zoom = 1.0f;
    float width, height;
};