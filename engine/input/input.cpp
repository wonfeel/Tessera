#include "engine/input/Input.h"

bool Input::isKeyPressed(int key) const {
    if (key >= 0 && key < MAX_KEYS)
        return m_keys[key];
    return false;
}

bool Input::isMouseButtonPressed(int button) const {
    if (button >= 0 && button < 8)
        return m_mouseButtons[button];
    return false;
}

void Input::getMousePosition(float& outX, float& outY) const {
    outX = m_mouseX;
    outY = m_mouseY;
}

float Input::getScrollDelta() {
    float delta = m_scrollDelta;
    m_scrollDelta = 0.0f; // reset after reading
    return delta;
}

void Input::setKeyState(int key, bool pressed) {
    if (key >= 0 && key < MAX_KEYS)
        m_keys[key] = pressed;
}

void Input::setMouseButtonState(int button, bool pressed) {
    if (button >= 0 && button < 8)
        m_mouseButtons[button] = pressed;
}

void Input::setMousePosition(float x, float y) {
    m_mouseX = x;
    m_mouseY = y;
}

void Input::setScrollDelta(float delta) {
    m_scrollDelta = delta;
}