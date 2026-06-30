// engine/core/Application.cpp
#include "Application.h"
#include <iostream>
#include <ctime>
#include <sstream>
#include <iomanip>
#ifdef FLORA_IMGUI_ENABLED
#  include <imgui.h>
#  include <imgui_impl_glfw.h>
#  include <imgui_impl_opengl3.h>
#endif

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

// ImGui helpers — only compiled when ImGui is available.
#ifdef FLORA_IMGUI_ENABLED
static void imguiInit(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;   // don't write imgui.ini next to the exe
    ImGui::StyleColorsDark();
    // install_callbacks=false: we feed input manually to avoid calling
    // glfwSet*Callback from the render thread (GLFW limitation on some platforms).
    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init("#version 460");
}
static void imguiShutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}
#endif

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

    // Render thread has released the GL context — reclaim it in the main thread.
    glfwMakeContextCurrent(m_window);

#ifdef FLORA_IMGUI_ENABLED
    if (m_imguiReady) {
        imguiShutdown();
        m_imguiReady = false;
    }
#endif

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

        // Push mouse state to render thread for ImGui consumption.
        float mx = 0.f, my = 0.f;
        m_input.getMousePosition(mx, my);
        m_imguiMouse.x   = mx;
        m_imguiMouse.y   = my;
        m_imguiMouse.btn0 = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
        m_imguiMouse.btn1 = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT);
        m_imguiMouse.btn2 = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_MIDDLE);
        m_imguiMouse.scroll = m_input.getScrollDelta();
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

#ifdef FLORA_IMGUI_ENABLED
        // Lazy init — GL context is current here in the render thread.
        if (!m_imguiReady) {
            imguiInit(m_window);
            m_imguiReady = true;
        }

        // Feed mouse input from atomics (written by update thread).
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize  = ImVec2(static_cast<float>(m_width),
                                  static_cast<float>(m_height));
        io.MousePos     = ImVec2(m_imguiMouse.x.load(), m_imguiMouse.y.load());
        io.MouseDown[0] = m_imguiMouse.btn0.load();
        io.MouseDown[1] = m_imguiMouse.btn1.load();
        io.MouseDown[2] = m_imguiMouse.btn2.load();
        io.MouseWheel   = m_imguiMouse.scroll.exchange(0.f);
        io.DeltaTime    = 1.f / 60.f;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        onImGui();

        m_imguiWantMouse =
            ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) ||
            ImGui::IsAnyItemActive();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#endif

        glfwSwapBuffers(m_window);

        // FPS counting in render thread (always, not just when showPerformance).
        m_renderFrameCount++;
        double nowFps = glfwGetTime();
        if (nowFps - m_lastRenderFpsTime >= 0.5) {
            double elapsed = nowFps - m_lastRenderFpsTime;
            if (elapsed > 0.0) {
                int fps = static_cast<int>(m_renderFrameCount / elapsed);
                m_fps = fps;
                if (m_showPerformance)
                    updatePerformanceDisplay(fps, 0);
            }
            m_renderFrameCount = 0;
            m_lastRenderFpsTime = nowFps;
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
    // Рендер-поток читает m_camera под m_cameraMutex (см. renderLoop),
    // поэтому запись из update-потока тоже должна быть под этим мьютексом,
    // иначе data race / UB. Сама раскладка управления живёт в CameraController.
    std::lock_guard<std::mutex> lock(m_cameraMutex);
    m_cameraController.update(m_input, m_camera, dt);
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