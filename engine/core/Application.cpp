// engine/core/Application.cpp
#include "Application.h"
#include <iostream>
#include <ctime>
#include <sstream>
#include <iomanip>

Application::Application(int width, int height, const std::string& title,
    bool showPerformance)
    : m_width(width), m_height(height),
    m_camera(static_cast<float>(width), static_cast<float>(height)),
    m_showPerformance(showPerformance)
{
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    m_window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create window");
    }

    glfwMakeContextCurrent(m_window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwDestroyWindow(m_window);
        glfwTerminate();
        throw std::runtime_error("Failed to initialize GLAD");
    }

    glViewport(0, 0, width, height);

    glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);

    glfwSetWindowUserPointer(m_window, this);
    glfwSetKeyCallback(m_window, keyCallback);
    glfwSetMouseButtonCallback(m_window, mouseButtonCallback);
    glfwSetCursorPosCallback(m_window, cursorPosCallback);
    glfwSetScrollCallback(m_window, scrollCallback);
}

Application::~Application() {
    m_running = false;
    if (m_renderThread.joinable())
        m_renderThread.join();
    if (m_window) {
        glfwDestroyWindow(m_window);
        glfwTerminate();
    }
}

void Application::run() {
    if (!m_window) return;

    onInit();   // Все OpenGL-ресурсы создаются здесь, контекст ещё в основном потоке

    // Отвязываем контекст от основного потока, чтобы его мог захватить рендер-поток
    glfwMakeContextCurrent(nullptr);

    m_running = true;
    m_renderThread = std::thread(&Application::renderLoop, this);
    updateLoop();

    m_running = false;
    if (m_renderThread.joinable())
        m_renderThread.join();

    // Теперь контекст свободен, забираем его в главный поток для корректного удаления ресурсов
    glfwMakeContextCurrent(m_window);

    onDestroy();

    // Отвязываем контекст перед тем, как деструктор разрушит окно
    glfwMakeContextCurrent(nullptr);
}

void Application::updateLoop() {
    m_lastFrameTime = glfwGetTime();

    while (m_running && !glfwWindowShouldClose(m_window)) {
        double now = glfwGetTime();
        float dt = static_cast<float>(now - m_lastFrameTime);
        m_lastFrameTime = now;

        glfwPollEvents();

        // Передаём изменение размера в рендер-поток (без вызова glViewport здесь)
        if (m_framebufferSizeChanged) {
            m_framebufferSizeChanged = false;
            std::lock_guard<std::mutex> lock(m_sizeMutex);
            m_newWidth = m_pendingWidth;
            m_newHeight = m_pendingHeight;
            m_sizeChanged = true;
            {
                std::lock_guard<std::mutex> camLock(m_cameraMutex);
                m_camera.width = static_cast<float>(m_newWidth);
                m_camera.height = static_cast<float>(m_newHeight);
            }
            onFramebufferSizeChanged(m_newWidth, m_newHeight);
        }

        onUpdate(dt);
        onCameraUpdate(dt);
    }
}

void Application::renderLoop() {
    glfwMakeContextCurrent(m_window);   // Захватываем контекст

    while (m_running && !glfwWindowShouldClose(m_window)) {
        {
            std::lock_guard<std::mutex> lock(m_sizeMutex);
            if (m_sizeChanged) {
                glViewport(0, 0, m_newWidth, m_newHeight);
                m_width = m_newWidth;
                m_height = m_newHeight;
                m_sizeChanged = false;
            }
        }

        Camera2D cameraCopy;
        {
            std::lock_guard<std::mutex> lock(m_cameraMutex);
            cameraCopy = m_camera;
        }

        glClear(GL_COLOR_BUFFER_BIT);
        onRender(cameraCopy);
        glfwSwapBuffers(m_window);

        // FPS counting in render thread
        if (m_showPerformance) {
            m_renderFrameCount++;
            double now = glfwGetTime();
            if (now - m_lastRenderFpsTime >= 0.5) {
                double elapsed = now - m_lastRenderFpsTime;
                if (elapsed > 0.0) {
                    int fps = static_cast<int>(m_renderFrameCount / elapsed);
                    updatePerformanceDisplay(fps, 0); // CPU не меряем
                }
                m_renderFrameCount = 0;
                m_lastRenderFpsTime = now;
            }
        }
    }

    // Освобождаем контекст перед выходом из потока
    glfwMakeContextCurrent(nullptr);
}

void Application::onFramebufferSizeChanged(int width, int height) {
    // По умолчанию пусто
}

void Application::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (app) {
        app->m_pendingWidth = width;
        app->m_pendingHeight = height;
        app->m_framebufferSizeChanged = true;
    }
}

void Application::onCameraUpdate(float dt) {
    bool w = m_input.isKeyPressed(GLFW_KEY_W);
    bool s = m_input.isKeyPressed(GLFW_KEY_S);
    bool a = m_input.isKeyPressed(GLFW_KEY_A);
    bool d = m_input.isKeyPressed(GLFW_KEY_D);

    float scroll = m_input.getScrollDelta();
    float mx = 0.0f, my = 0.0f;
    if (scroll != 0.0f)
        m_input.getMousePosition(mx, my);

    // Рендер-поток читает m_camera под m_cameraMutex (см. renderLoop),
    // поэтому запись из update-потока тоже должна быть под этим мьютексом,
    // иначе data race / UB.
    std::lock_guard<std::mutex> lock(m_cameraMutex);
    m_camera.processKeyboard(dt, w, s, a, d);
    if (scroll != 0.0f)
        m_camera.processScroll(scroll, mx, my);
}

void Application::updatePerformanceDisplay(int fps, int cpuPercent) {
    if (!m_window) return;
    std::ostringstream title;
    title << "FPS: " << fps << " | CPU: " << cpuPercent << "%";
    glfwSetWindowTitle(m_window, title.str().c_str());
}

void Application::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (app) app->m_input.setKeyState(key, action != GLFW_RELEASE);
}

void Application::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (app) app->m_input.setMouseButtonState(button, action != GLFW_RELEASE);
}

void Application::cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (app) app->m_input.setMousePosition(static_cast<float>(xpos), static_cast<float>(ypos));
}

void Application::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (app) app->m_input.setScrollDelta(static_cast<float>(yoffset));
}