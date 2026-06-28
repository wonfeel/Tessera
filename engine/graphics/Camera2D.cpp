#include "engine/graphics/Camera2D.h"
#include <glm/gtc/matrix_transform.hpp>

Camera2D::Camera2D(float screenWidth, float screenHeight)
    : width(screenWidth), height(screenHeight) {
}

glm::mat4 Camera2D::getViewProjectionMatrix() const {
    glm::mat4 view = glm::translate(glm::mat4(1.0f), glm::vec3(-position, 0.0f));
    glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(zoom, zoom, 1.0f));
    glm::mat4 ortho = glm::ortho(0.0f, width, height, 0.0f, -1.0f, 1.0f);
    return ortho * scale * view;
}

void Camera2D::processKeyboard(float dt, bool w, bool s, bool a, bool d) {
    float speed = 800.0f / zoom * dt;
    if (w) position.y -= speed;
    if (s) position.y += speed;
    if (a) position.x -= speed;
    if (d) position.x += speed;
}

void Camera2D::processScroll(double yoffset, float mouseScreenX, float mouseScreenY) {
    glm::vec2 worldBefore = screenToWorld(mouseScreenX, mouseScreenY);
    zoom *= (yoffset > 0) ? 1.1f : 0.9f;
    zoom = glm::clamp(zoom, 0.001f, 1000.0f);
    glm::vec2 worldAfter = screenToWorld(mouseScreenX, mouseScreenY);
    position += (worldBefore - worldAfter);
}