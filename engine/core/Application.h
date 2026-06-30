#pragma once
#include <string>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "engine/input/Input.h"
#include "engine/graphics/Camera2D.h"
#include "engine/graphics/CameraController.h"

class Application {
public:
    Application(int width, int height, const std::string& title,
        bool showPerformance = false);
    virtual ~Application();

    void run();

    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    GLFWwindow* getWindow() const { return m_window; }

    Camera2D& getCamera() { return m_camera; }
    const Camera2D& getCamera() const { return m_camera; }

protected:
    virtual void onInit() = 0;
    virtual void onUpdate(float dt) = 0;
    virtual void onRender(const Camera2D& camera) = 0;
    virtual void onDestroy() = 0;

    // Override to add ImGui widgets. Called from the render thread every frame
    // after onRender(), between ImGui::NewFrame() and ImGui::Render().
    // The base implementation shows nothing.
    virtual void onImGui() {}

    virtual void onCameraUpdate(float dt);
    void updatePerformanceDisplay(int fps, int cpuPercent);

    Input m_input;
    Camera2D m_camera;
    // Управление камерой с клавиатуры/мыши. Демки могут отключить (enabled=false)
    // или переопределить onCameraUpdate целиком.
    CameraController m_cameraController;
    bool m_showPerformance;

    // True while ImGui is hovering/using the mouse — subclasses should not
    // process mouse clicks for painting/interaction when this is set.
    std::atomic<bool> m_imguiWantMouse{false};

    // Current render FPS, updated every 0.5 s in the render thread.
    std::atomic<int> m_fps{0};

    virtual void onFramebufferSizeChanged(int width, int height);

private:
    // Mouse state written by the update thread, read by the render thread
    // for ImGui input (plain atomics, no mutex needed for float/bool reads).
    struct ImGuiMouseState {
        std::atomic<float> x{0.f}, y{0.f};
        std::atomic<float> scroll{0.f};
        std::atomic<bool>  btn0{false}, btn1{false}, btn2{false};
    } m_imguiMouse;

    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);

    void updateLoop();
    void renderLoop();

    GLFWwindow* m_window = nullptr;
    int m_width, m_height;

    std::thread m_renderThread;
    std::atomic<bool> m_running{ false };

    int    m_renderFrameCount = 0;
    double m_lastRenderFpsTime = 0.0;
    double m_lastFrameTime = 0.0;

    bool m_framebufferSizeChanged = false;
    int  m_pendingWidth = 0, m_pendingHeight = 0;

    std::mutex m_cameraMutex;
    std::mutex m_sizeMutex;
    bool m_sizeChanged = false;
    int m_newWidth = 0, m_newHeight = 0;

    bool m_imguiReady = false;
};
