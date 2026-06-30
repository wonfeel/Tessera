// engine/graphics/CameraController.h
#pragma once

class Input;
class Camera2D;

// Политика управления камерой с клавиатуры/колеса мыши (WASD — панорамирование,
// scroll — зум к курсору). Вынесено из Application: базовый класс больше не зашивает
// конкретную раскладку управления — он лишь вызывает контроллер, а демки могут
// его отключить (enabled=false) или заменить логику, переопределив onCameraUpdate.
class CameraController {
public:
    // Читает WASD + scroll + перетаскивание средней кнопкой мыши и применяет
    // панорамирование/зум к camera за dt. Потокобезопасность (мьютекс камеры) —
    // забота вызывающего (этот же мьютекс читает рендер-поток).
    void update(Input& input, Camera2D& camera, float dt);

    bool enabled = true;          // всё управление
    bool mousePanEnabled = true;  // панорамирование средней кнопкой мыши

private:
    bool  m_mmbDragging = false;
    float m_mmbLastX = 0.0f, m_mmbLastY = 0.0f;
};
