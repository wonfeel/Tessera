// engine/core/Application.cpp
#include "Application.h"
#include <iostream>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <filesystem>
#ifdef TESSERA_IMGUI_ENABLED
#  include <imgui.h>
#  include <imgui_impl_glfw.h>
#  include <imgui_impl_opengl3.h>
// Внутренний хелпер бэкенда (externally-linked, но не объявлен в публичном
// imgui_impl_glfw.h) — чистая функция без вызовов GLFW, поэтому безопасно
// звать с любого потока. Нужен взамен ImGui_ImplGlfw_KeyCallback(), который
// внутри дёргает glfwGetKey()/glfwGetKeyName() — а те, по документации GLFW,
// разрешено звать только с главного потока (см. renderLoop() ниже).
extern ImGuiKey ImGui_ImplGlfw_KeyToImGuiKey(int keycode, int scancode);
#endif

namespace {
// ФИКСИРОВАННЫЙ FPS ЗАПИСИ, независимый от фактического FPS рендера.
// Роликам для README/демонстрации плавность важнее точного соответствия
// частоте кадров движка, а фиксированное число упрощает и ffmpeg-команду
// (-framerate), и расчёт задержки GIF-кадра — обоим бэкендам достаточно
// знать одно и то же целое число, а не мерить реальный FPS на лету.
constexpr int kRecordingFps = 30;

// Имя файла по времени старта записи — без него второй F9 за сессию
// перезаписал бы первый ролик молча. recordings/ создаётся здесь же, если
// её ещё нет: без этого ffmpeg/GifStreamBegin просто не откроют файл в
// несуществующей папке и запись тихо не начнётся.
std::string makeRecordingPathNoExt() {
    std::filesystem::create_directories("recordings");
    const auto now = std::time(nullptr);
    std::tm tmv{};
    localtime_s(&tmv, &now);
    std::ostringstream oss;
    oss << "recordings/capture_" << std::put_time(&tmv, "%Y%m%d_%H%M%S");
    return oss.str();
}
} // namespace

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
    glfwSetCharCallback(m_window, charCallback);
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
#ifdef TESSERA_IMGUI_ENABLED
static void imguiInit(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;   // don't write imgui.ini next to the exe

    // ImGui встроенный шрифт (ProggyClean) - только ASCII, весь кириллический
    // текст в панелях (demo/worm и любая другая демка с русскими подписями)
    // рисуется квадратиками-заглушками без глифов. Segoe UI - системный шрифт
    // Windows, кириллица есть с XP; путь проверяем сами (не полагаясь на
    // AddFontFromFileTTF само по себе) - при отсутствующем файле оно зовёт
    // IM_ASSERT_USER_ERROR, который в этой сборке (ConfigErrorRecoveryEnable-
    // Assert по умолчанию true) реально роняет debug-сборку через assert(), а
    // не просто тихо возвращает nullptr - при отсутствии файла тихо остаёмся
    // на дефолтном шрифте вместо падения. 18px, не дефолтные 13 - Proggy
    // (пиксельный, маленький нативный размер) и пропорциональный TTF-шрифт
    // на одном и том же пиксельном размере визуально не эквивалентны, 18
    // читается на типичном для этого проекта разрешении/зуме демок.
    const char* kCyrillicFontPath = "C:\\Windows\\Fonts\\segoeui.ttf";
    ImFont* font = nullptr;
    if (std::filesystem::exists(kCyrillicFontPath)) {
        font = io.Fonts->AddFontFromFileTTF(kCyrillicFontPath, 18.0f, nullptr, io.Fonts->GetGlyphRangesCyrillic());
    }
    if (!font) io.Fonts->AddFontDefault();

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

#ifdef TESSERA_IMGUI_ENABLED
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
        serviceClipboardRequest();   // см. .h — glfwGet/SetClipboardString только с этого потока

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

        // F9 обрабатывается ЗДЕСЬ, а не в keyCallback: старт/стоп записи
        // трогает GL-состояние опосредованно (через m_screenRecorder,
        // captureFrame которого читает бэкбуфер) и должен происходить на
        // рендер-потоке, том же, что держит GL-контекст текущим - см.
        // класс-комментарий ScreenRecorder про потоковость. exchange(false)
        // одним атомарным чтением снимает и сбрасывает флаг, дребезг клавиши
        // внутри кадра схлопывается в одно переключение.
        if (m_toggleRecordingRequested.exchange(false)) {
            if (m_screenRecorder.isRecording()) {
                m_screenRecorder.stop();
                std::cout << "[recording] stopped -> " << m_screenRecorder.lastOutputPath() << std::endl;
            } else {
                const std::string path = makeRecordingPathNoExt();
                if (m_screenRecorder.start(m_width, m_height, kRecordingFps, path, kFfmpegRelPath)) {
                    std::cout << "[recording] started -> " << m_screenRecorder.lastOutputPath() << std::endl;
                } else {
                    std::cerr << "[recording] failed to start (see " << path << ".ffmpeg.log if ffmpeg backend)" << std::endl;
                }
            }
        }

        glClear(GL_COLOR_BUFFER_BIT);
        onRender(cameraCopy);

#ifdef TESSERA_IMGUI_ENABLED
        // Lazy init — GL context is current here in the render thread.
        if (!m_imguiReady) {
            imguiInit(m_window);
            // ImGui_ImplGlfw_InitForOpenGL() выше уже поставил
            // io.GetClipboardTextFn/SetClipboardTextFn на GLFW-функции
            // напрямую — переопределяем их на потокобезопасные обёртки (см.
            // .h): те звонят glfwGet/SetClipboardString только с главного
            // потока через serviceClipboardRequest() в updateLoop().
            ImGuiIO& clipIo = ImGui::GetIO();
            clipIo.GetClipboardTextFn = &Application::imguiGetClipboardText;
            clipIo.SetClipboardTextFn = &Application::imguiSetClipboardText;
            clipIo.ClipboardUserData = this;
            // Атлас ещё не заблокирован (до первого NewFrame() ниже) - см.
            // Application.h за тем, почему это единственное безопасное место
            // для демок, добавляющих свои шрифты (onImGui() уже опоздал -
            // R6010 "Cannot modify a locked ImFontAtlas", воспроизведено и
            // проверено на этой машине при вызове AddFontFromFileTTF из
            // onImGui() до этого хука).
            onImGuiInit();
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

        // Клавиатура/текст — раньше вообще не доходили до ImGui (только
        // мышь прокидывалась вручную), поэтому Ctrl+клик по слайдеру для
        // ввода точного числа не работал: ImGui не получал ни одного
        // key-события. install_callbacks=false не даёт GLFW самому звать
        // ImGui_ImplGlfw_Key/CharCallback, поэтому зовём их сами — тут, на
        // рендер-потоке, из очереди, накопленной в keyCallback/charCallback
        // на update-потоке. ВАЖНО: не вызываем ImGui_ImplGlfw_KeyCallback()
        // напрямую — она внутри читает модификаторы через glfwGetKey() и
        // (для нераспознанных клавиш) glfwGetKeyName(), а обе эти функции
        // GLFW разрешает звать только с главного потока (которым тут
        // является update-поток, не рендер-поток) — вызов с чужого потока
        // и был настоящей причиной, почему Ctrl не распознавался стабильно.
        // Вместо этого берём mods прямо из события — оно захвачено в
        // keyCallback НА главном потоке, так что валидно и безопасно.
        {
            std::vector<GlfwKeyEvent> keyEvents;
            std::vector<unsigned int> charEvents;
            {
                std::lock_guard<std::mutex> lock(m_imguiInputMutex);
                keyEvents.swap(m_imguiKeyEvents);
                charEvents.swap(m_imguiCharEvents);
            }
            for (const auto& e : keyEvents) {
                if (e.action != GLFW_PRESS && e.action != GLFW_RELEASE)
                    continue;   // GLFW_REPEAT — ImGui сама генерирует повтор по held-состоянию
                io.AddKeyEvent(ImGuiMod_Ctrl,  (e.mods & GLFW_MOD_CONTROL) != 0);
                io.AddKeyEvent(ImGuiMod_Shift, (e.mods & GLFW_MOD_SHIFT) != 0);
                io.AddKeyEvent(ImGuiMod_Alt,   (e.mods & GLFW_MOD_ALT) != 0);
                io.AddKeyEvent(ImGuiMod_Super, (e.mods & GLFW_MOD_SUPER) != 0);
                ImGuiKey imguiKey = ImGui_ImplGlfw_KeyToImGuiKey(e.key, e.scancode);
                io.AddKeyEvent(imguiKey, e.action == GLFW_PRESS);
                io.SetKeyEventNativeData(imguiKey, e.key, e.scancode);
            }
            // CharCallback безопасна с любого потока — просто io.AddInputCharacter(c),
            // без обращений к GLFW.
            for (unsigned int c : charEvents)
                ImGui_ImplGlfw_CharCallback(m_window, c);
        }

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

        // ДО свапа: читаем GL_BACK, пока в нём ещё лежит только что
        // нарисованный (и, если включён ImGui, уже композитный) кадр - см.
        // ScreenRecorder::captureFrame. isRecording() внутри - функция сама
        // ничего не делает, если запись не идёт, лишний atomic-load дешевле
        // дублирования проверки на каждом вызывающем сайте.
        m_screenRecorder.captureFrame();

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
    if (!app) return;
    app->m_input.setKeyState(key, action != GLFW_RELEASE);
    // F9 - переключатель записи экрана (см. Application.h/renderLoop). Только
    // GLFW_PRESS, не GLFW_REPEAT/RELEASE - удержание клавиши не должно
    // перещёлкивать запись туда-сюда несколько раз за секунду.
    if (key == GLFW_KEY_F9 && action == GLFW_PRESS) {
        app->m_toggleRecordingRequested = true;
    }
    std::lock_guard<std::mutex> lock(app->m_imguiInputMutex);
    app->m_imguiKeyEvents.push_back({key, scancode, action, mods});
}

void Application::charCallback(GLFWwindow* window, unsigned int codepoint) {
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (!app) return;
    std::lock_guard<std::mutex> lock(app->m_imguiInputMutex);
    app->m_imguiCharEvents.push_back(codepoint);
}

// См. .h — glfwGet/SetClipboardString обязаны звать с главного потока
// (update-поток), а ImGui дёргает io.GetClipboardTextFn/SetClipboardTextFn
// синхронно из виджетов на рендер-потоке. Кладём запрос и блокируемся,
// пока главный поток не обслужит его в serviceClipboardRequest().
void Application::serviceClipboardRequest() {
    std::unique_lock<std::mutex> lock(m_clipboardMutex);
    if (m_clipboardOp == ClipboardOp::Get) {
        const char* text = glfwGetClipboardString(m_window);
        m_clipboardBuffer = text ? text : "";
    } else if (m_clipboardOp == ClipboardOp::Set) {
        glfwSetClipboardString(m_window, m_clipboardBuffer.c_str());
    } else {
        return;
    }
    m_clipboardOp = ClipboardOp::None;
    m_clipboardDone = true;
    lock.unlock();
    m_clipboardCv.notify_all();
}

const char* Application::imguiGetClipboardText(void* userData) {
    auto* app = static_cast<Application*>(userData);
    std::unique_lock<std::mutex> lock(app->m_clipboardMutex);
    app->m_clipboardDone = false;
    app->m_clipboardOp = ClipboardOp::Get;
    app->m_clipboardCv.wait(lock, [app] { return app->m_clipboardDone; });
    // m_clipboardBuffer остаётся валидным до следующего вызова — ImGui
    // копирует строку сразу после возврата, как и в оригинальном
    // glfwGetClipboardString(), возвращающем указатель на внутренний буфер.
    return app->m_clipboardBuffer.c_str();
}

void Application::imguiSetClipboardText(void* userData, const char* text) {
    auto* app = static_cast<Application*>(userData);
    std::unique_lock<std::mutex> lock(app->m_clipboardMutex);
    app->m_clipboardBuffer = text ? text : "";
    app->m_clipboardDone = false;
    app->m_clipboardOp = ClipboardOp::Set;
    app->m_clipboardCv.wait(lock, [app] { return app->m_clipboardDone; });
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