#pragma once

class Input {
public:
    bool isKeyPressed(int key) const;
    bool isMouseButtonPressed(int button) const;
    void getMousePosition(float& outX, float& outY) const;
    float getScrollDelta();

    // Called by Application (via GLFW callbacks)
    void setKeyState(int key, bool pressed);
    void setMouseButtonState(int button, bool pressed);
    void setMousePosition(float x, float y);
    void setScrollDelta(float delta);

private:
    static constexpr int MAX_KEYS = 1024;
    bool m_keys[MAX_KEYS] = { false };
    bool m_mouseButtons[8] = { false };
    float m_mouseX = 0.0f, m_mouseY = 0.0f;
    float m_scrollDelta = 0.0f;
};