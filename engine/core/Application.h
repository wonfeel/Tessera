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

    virtual void onCameraUpdate(float dt);
    void updatePerformanceDisplay(int fps, int cpuPercent);

    Input m_input;
    Camera2D m_camera;
    bool m_showPerformance;

    virtual void onFramebufferSizeChanged(int width, int height);

private:
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

    // FPS counting in render thread
    int    m_renderFrameCount = 0;
    double m_lastRenderFpsTime = 0.0;

    double m_lastFrameTime = 0.0;

    bool m_framebufferSizeChanged = false;
    int  m_pendingWidth = 0, m_pendingHeight = 0;

    std::mutex m_cameraMutex;
    std::mutex m_sizeMutex;
    bool m_sizeChanged = false;
    int m_newWidth = 0, m_newHeight = 0;
};