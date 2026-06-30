// demo/life/capture_ui/main.cpp
//
// A small ImGui tool for configuring and launching headless GIF captures.
// No simulation runs here — it just builds the command for Test_capture and
// executes it in a background thread while showing progress in the UI.
//
// The capture exe is expected next to this binary (same folder).
// Output defaults to %USERPROFILE%\Pictures\Tessera\capture.gif.

#include "engine/core/Application.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <string>
#include <thread>

#ifdef _WIN32
#  define NOMINMAX
#  include <windows.h>
#endif

#ifdef TESSERA_IMGUI_ENABLED
#  include <imgui.h>
#endif

namespace {

std::string defaultOutPath() {
    const char* profile = std::getenv("USERPROFILE");
    std::filesystem::path base = profile ? profile : std::filesystem::temp_directory_path();
    return (base / "Pictures" / "Tessera" / "capture.gif").string();
}

std::string exeDir() {
#ifdef _WIN32
    char buf[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, buf, MAX_PATH);
    return std::filesystem::path(buf).parent_path().string();
#else
    return ".";
#endif
}

// Launch a command in a background thread; when done, store exit code.
void runAsync(const std::string& cmd,
              std::atomic<bool>& running,
              std::atomic<int>&  exitCode) {
    running = true;
    exitCode = -1;
    std::thread([cmd, &running, &exitCode]() {
        exitCode = std::system(cmd.c_str());
        running  = false;
    }).detach();
}

} // namespace

class CaptureUiApp : public Application {
public:
    CaptureUiApp()
        : Application(520, 560, "Tessera — Capture UI", false)
    {
        std::strncpy(m_outPath, defaultOutPath().c_str(), sizeof(m_outPath) - 1);
    }

protected:
    void onInit()    override { glClearColor(0.12f, 0.12f, 0.12f, 1.f); }
    void onUpdate(float) override {}
    void onRender(const Camera2D&) override {}
    void onDestroy() override {}

    void onImGui() override {
#ifdef TESSERA_IMGUI_ENABLED
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
        ImGui::Begin("Capture settings", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove     | ImGuiWindowFlags_NoCollapse);

        // --- Scene ---
        ImGui::TextDisabled("Scene");
        ImGui::Separator();
        static const char* kScenes[] = { "Gosper gun + eater", "Random 64x64 field", "Glider" };
        ImGui::SetNextItemWidth(-1);
        ImGui::Combo("##scene", &m_scene, kScenes, 3);

        if (m_scene == 0) {
            ImGui::SetNextItemWidth(80);
            ImGui::InputInt("guns X", &m_gunsX); ImGui::SameLine();
            ImGui::SetNextItemWidth(80);
            ImGui::InputInt("guns Y", &m_gunsY);
            m_gunsX = std::max(1, m_gunsX);
            m_gunsY = std::max(1, m_gunsY);
        }

        // --- Output ---
        ImGui::Spacing();
        ImGui::TextDisabled("Output");
        ImGui::Separator();
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##out", m_outPath, sizeof(m_outPath));
        if (ImGui::Button("Reset to Pictures", ImVec2(-1, 0)))
            std::strncpy(m_outPath, defaultOutPath().c_str(), sizeof(m_outPath) - 1);

        // --- Capture parameters ---
        ImGui::Spacing();
        ImGui::TextDisabled("Parameters");
        ImGui::Separator();

        ImGui::SetNextItemWidth(120); ImGui::InputInt("Steps",     &m_stop);
        ImGui::SetNextItemWidth(120); ImGui::InputInt("Delay (ms)",&m_delayMs);
        ImGui::SetNextItemWidth(120); ImGui::InputInt("Width px",  &m_resW);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120); ImGui::InputInt("Height px", &m_resH);

        m_stop    = std::max(1,   m_stop);
        m_delayMs = std::max(20,  m_delayMs);
        m_resW    = std::max(64,  m_resW);
        m_resH    = std::max(64,  m_resH);

        ImGui::Spacing();
        ImGui::TextDisabled("Region (tile coords)");
        ImGui::SetNextItemWidth(80); ImGui::InputInt("x0", &m_x0); ImGui::SameLine();
        ImGui::SetNextItemWidth(80); ImGui::InputInt("y0", &m_y0); ImGui::SameLine();
        ImGui::SetNextItemWidth(80); ImGui::InputInt("x1", &m_x1); ImGui::SameLine();
        ImGui::SetNextItemWidth(80); ImGui::InputInt("y1", &m_y1);
        m_x1 = std::max(m_x0 + 1, m_x1);
        m_y1 = std::max(m_y0 + 1, m_y1);

        // Presets
        ImGui::Spacing();
        if (ImGui::Button("Preset: gun+eater")) {
            m_x0=14; m_y0=14; m_x1=80; m_y1=60;
            m_stop=60; m_resW=600; m_resH=360; m_scene=0;
        }
        ImGui::SameLine();
        if (ImGui::Button("Preset: random")) {
            m_x0=0; m_y0=0; m_x1=80; m_y1=80;
            m_stop=60; m_resW=480; m_resH=480; m_scene=1;
        }
        ImGui::SameLine();
        if (ImGui::Button("Preset: glider")) {
            m_x0=0; m_y0=0; m_x1=35; m_y1=35;
            m_stop=60; m_resW=420; m_resH=420; m_scene=2;
        }

        // --- Capture button ---
        ImGui::Spacing();
        ImGui::Separator();
        bool busy = m_running.load();
        if (busy) ImGui::BeginDisabled();
        if (ImGui::Button("Capture!", ImVec2(-1, 40)))
            startCapture();
        if (busy) ImGui::EndDisabled();

        if (busy) {
            ImGui::TextColored(ImVec4(1,0.8f,0,1), "Capturing... (this window will freeze)");
        } else if (m_exitCode.load() == 0) {
            ImGui::TextColored(ImVec4(0.4f,1,0.4f,1), "Done: %s", m_outPath);
        } else if (m_exitCode.load() > 0) {
            ImGui::TextColored(ImVec4(1,0.4f,0.4f,1), "Failed (exit %d)", m_exitCode.load());
        } else {
            ImGui::TextDisabled("Ready");
        }

        ImGui::End();
#endif
    }

private:
    void startCapture() {
        std::string captureExe = (std::filesystem::path(exeDir()) / "Test_capture.exe").string();
        // test_capture positional args:
        //   outOrDir stop stride resW resH x0 y0 x1 y1 gridW gridH gunsX gunsY
        //   eaterX eaterY eaterRot eaterShape scene
        std::ostringstream cmd;
        cmd << '"' << captureExe << '"'
            << " \"" << m_outPath << '"'
            << ' ' << m_stop << " 1"               // stride always 1
            << ' ' << m_resW << ' ' << m_resH
            << ' ' << m_x0 << ' ' << m_y0
            << ' ' << m_x1 << ' ' << m_y1
            << " 512 512"                           // grid size
            << ' ' << m_gunsX << ' ' << m_gunsY
            << " -1 -1 0 0"                        // eater (auto)
            << ' ' << m_scene
            << ' ' << m_delayMs;
        runAsync(cmd.str(), m_running, m_exitCode);
    }

    // scene: 0=guns, 1=random, 2=glider
    int  m_scene   = 0;
    int  m_gunsX   = 1, m_gunsY = 1;
    int  m_stop    = 60;
    int  m_delayMs = 50;
    int  m_resW    = 600, m_resH = 360;
    int  m_x0 = 14, m_y0 = 14, m_x1 = 80, m_y1 = 60;
    char m_outPath[512] = {};

    std::atomic<bool> m_running{false};
    std::atomic<int>  m_exitCode{-1};
};

int main() {
    CaptureUiApp app;
    app.run();
    return 0;
}
