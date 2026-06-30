// engine/graphics/CameraController.cpp
#include "engine/graphics/CameraController.h"
#include "engine/graphics/Camera2D.h"
#include "engine/input/Input.h"
#include <GLFW/glfw3.h>

void CameraController::update(Input& input, Camera2D& camera, float dt) {
    if (!enabled) return;

    bool w = input.isKeyPressed(GLFW_KEY_W);
    bool s = input.isKeyPressed(GLFW_KEY_S);
    bool a = input.isKeyPressed(GLFW_KEY_A);
    bool d = input.isKeyPressed(GLFW_KEY_D);

    float scroll = input.getScrollDelta();
    float mx = 0.0f, my = 0.0f;
    if (scroll != 0.0f)
        input.getMousePosition(mx, my);

    camera.processKeyboard(dt, w, s, a, d);
    if (scroll != 0.0f)
        camera.processScroll(scroll, mx, my);

    // Панорамирование перетаскиванием средней кнопки мыши.
    if (mousePanEnabled && input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_MIDDLE)) {
        float pmx = 0.0f, pmy = 0.0f;
        input.getMousePosition(pmx, pmy);
        if (m_mmbDragging)
            camera.position -= glm::vec2(pmx - m_mmbLastX, pmy - m_mmbLastY) / camera.zoom;
        m_mmbDragging = true;
        m_mmbLastX = pmx;
        m_mmbLastY = pmy;
    } else {
        m_mmbDragging = false;
    }
}
