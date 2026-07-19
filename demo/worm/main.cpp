// demo/worm/main.cpp
//
// C. elegans коннектом (connectome/, вендорено из connectome-sim) поверх
// гекс-поля Tessera. Не клеточный автомат (см. WormSim) - как demo/light и
// demo/cloth, наследуется от Application напрямую, свой маленький GL-рендер,
// без ChunkedTileMap/DefaultApplication.
//
// Поле - гекс-решётка (та же геометрия, что и в demo/light, свой шейдер
// Shaders/hex_point.*), тело червя - Shaders/worm_body.* как треугольная
// лента с сужением к голове/хвосту. Гекс-решётка тут не только декорация:
// каждая клетка - это ещё и клетка непрерывного поля еды WormSim (1 гекс = 1
// клетка поля), поэтому подсветка земли ЖИВАЯ - показывает настоящий
// "бактериальный газон", который червь ест и по которому нюхает градиент.
//
// Управление - без ручного выбора нейронов и без автопилота: ЛКМ рисует еду
// по полю (или стирает - см. переключатель инструмента в панели), к ней
// реагируют настоящие хемосенсорные нейроны червя (см. WormSim - там же
// честный клинокинез и шум вместо любого "навести на цель"). Без еды сеть
// держит независимый шум - червь всё равно подёргивается и ищет сам.
// Перемещение тела - решение баланса сил анизотропного трения на форме,
// которую породила сеть (см. connectome::WormBody), не отдельная эвристика.
// WASD/scroll/MMB - камера (стандартный CameraController).
#include "engine/core/Application.h"
#include "engine/graphics/Shader.h"
#include "WormSim.h"

#include <glad/glad.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <exception>
#include <memory>
#include <vector>

#include "engine/core/HexGrid.h"

#ifdef TESSERA_IMGUI_ENABLED
#  include <imgui.h>
#endif

namespace {
    constexpr int kHexCols = 280, kHexRows = 200;
    constexpr float kHexSpacing = 36.0f;
    constexpr float kPointBaseSize = 1.0f;

    // Дешёвый детерминированный хэш (col,row) -> [0,1) - только для лёгкой
    // яркостной "текстуры" земли, не для чего-либо, влияющего на симуляцию.
    float hash01(int a, int b) {
        unsigned int h = static_cast<unsigned int>(a * 374761393 + b * 668265263 + 2166136261u);
        h = (h ^ (h >> 13)) * 1274126177u;
        h ^= (h >> 16);
        return static_cast<float>(h & 0xFFFFFFu) / static_cast<float>(0xFFFFFFu);
    }
}

class WormApp : public Application {
public:
    WormApp()
        : Application(1280, 800, "Tessera - C. elegans worm", false),
          m_wormSim("worm_data/celegans_herm.connectome") {}

protected:
    void onInit() override {
        initHexShader();
        initHexField();
        initBodyShader();

        glm::vec2 worldMax = HexGrid::worldPos(kHexCols - 1, kHexRows - 1, kHexSpacing);
        frameCamera(glm::vec2(0.0f), worldMax, kHexSpacing * 2.0f);
        // fieldCols/fieldRows == kHexCols/kHexRows: 1 гекс = 1 клетка поля
        // еды. WormSim теперь считает boundsMax сам по той же гекс-формуле
        // (HexGrid::worldPos), а не по прямоугольному приближению - иначе
        // покраска/нюх промахивались мимо клетки, которая реально светится.
        m_wormSim.setBounds(glm::vec2(0.0f), kHexCols, kHexRows, kHexSpacing);

        glEnable(GL_PROGRAM_POINT_SIZE);
        // Фон вьюпорта - тон "nocturne" (тихий почти-чёрный с лёгким
        // фиолетовым оттенком) вместо прежнего зеленоватого - см.
        // applyNocturneImGuiTheme(): то же самое для панелей ImGui (вызов
        // не отсюда - ImGui-контекст на момент onInit() ещё не создан,
        // imguiInit() в Application.cpp происходит позже, в renderLoop();
        // тема применяется один раз при первом реальном onImGui()).
        // Цвета САМОЙ симуляции (тело червя, тепловая карта нейронов,
        // подсветка еды) не трогаем - это данные, не декор.
        glClearColor(0.039f, 0.039f, 0.063f, 1.0f);
    }

    void onUpdate(float dt) override {
        dt = std::min(dt, 0.05f);
        handleFoodPaint(dt);

        // step() advances the sim by exactly params.dt each call, regardless
        // of real elapsed time - so calling it once per rendered frame ties
        // perceived playback SPEED to the "dt" slider (drag it to 0.005 and
        // everything looks 10x slower/frozen, even though it's really just
        // integrating in finer steps). Fixed-timestep accumulator: "dt"
        // becomes a pure integration-granularity knob, simulated time per
        // real second stays constant regardless of its value. Guard caps
        // substeps per frame so a tiny dt (or a real hitch) can't spiral.
        m_stepAccumulator += dt * m_wormSim.params.timeScale.load();
        const float simDt = std::max(0.0005f, m_wormSim.params.dt.load());
        int guard = 0;
        while (m_stepAccumulator >= simDt && guard < 200) {
            m_wormSim.step();
            m_stepAccumulator -= simDt;
            ++guard;
        }
    }

    void onRender(const Camera2D& camera) override {
        glClear(GL_COLOR_BUFFER_BIT);

        // hex_point.frag выводит alpha-premultiplied цвет для аддитивного
        // блендинга (см. demo/light) - без glEnable(GL_BLEND) сглаживание
        // кромки гекса не работает, края рублёные. Выключаем перед телом
        // червя: то рисуется непрозрачно (worm_body.frag отдаёт alpha=1),
        // аддитивный blend поверх земли дал бы засвеченный, не сплошной цвет.
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        renderGround(camera);
        glDisable(GL_BLEND);

        renderBody(camera);
        glBindVertexArray(0);
    }

    void onImGui() override {
#ifdef TESSERA_IMGUI_ENABLED
        // Once, on the first frame that actually has an ImGui context
        // (onInit() runs too early - see the comment by glClearColor above).
        if (!m_themeApplied) {
            applyNocturneImGuiTheme();
            m_themeApplied = true;
        }

        WormSim::Snapshot snap;
        m_wormSim.snapshot(snap);

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Worm", nullptr, ImGuiWindowFlags_NoCollapse);

        ImGui::Text("%d nodes, food field %dx%d", snap.nodeCount, m_wormSim.foodFieldCols(), m_wormSim.foodFieldRows());
        if (ImGui::Button("Reset to defaults")) resetParamsToDefaults();
        // Было 0.1-2000.0 линейно - полезный диапазон 1-10x занимал
        // мизерную долю хода слайдера, пиксель драга скакал на десятки x.
        // Предохранитель от неадекватно большого значения - substep guard в
        // onUpdate() (max 200 подшагов/кадр), не диапазон самого слайдера.
        sliderWithInput("Time scale (playback speed)", m_wormSim.params.timeScale, 0.0f, 100.0f, "%.2fx",
                         "0x pauses the simulation.");

        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Network")) {
            sliderWithInput("Chemical synapse gain", m_wormSim.params.chemGain, 0.0f, 0.5f, "%.4f");
            sliderWithInput("Gap junction gain", m_wormSim.params.gapGain, 0.0f, 0.15f, "%.4f",
                             "Too high freezes the worm solid - first thing to check if it goes rigid.");
            sliderWithInput("Body curvature gain", m_wormSim.params.bodyGain, 0.0f, 5.0f, "%.2f");
        }

        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Locomotion (substrate friction)", ImGuiTreeNodeFlags_DefaultOpen)) {
            // Только ОТНОШЕНИЕ c_n/c_t определяет результат: solve_propulsion
            // (body.cpp) решает безынерционный (квази-статический) баланс сил
            // trag*V=drive - при равномерном масштабировании обоих коэффициентов
            // в k раз матрица и правая часть системы масштабируются на тот же k,
            // решение (скорость) не меняется. Поэтому кнопки-пресеты трогают
            // только dragNormal, держа dragTangent=1.0 как точку отсчёта - это
            // не приближение, это то, что реально важно в этой физике.
            // Agar (crawling): анизотропия ~10-40x, замерено напрямую (Fang-Yen
            // et al. 2010, Biophysical J.: Cn~222/Ct~22.1 =~10.05; тот же цикл
            // измерений даёт разброс "as much as an order of magnitude" по
            // условиям агара - 40.0 (дефолт) - верхняя граница этого диапазона).
            // Water (swimming): анизотропия куда слабее, ближе к изотропной -
            // Cn/Ct ~= 1.4-2 по совокупности литературы для плавания в жидкости
            // низкой вязкости (в отличие от ползания по гелю, где поверхность
            // сама даёт анизотропию) - 1.7 взято серединой этого диапазона.
            if (ImGui::Button("Agar (crawling)")) {
                m_wormSim.params.dragTangent = 1.0f;
                m_wormSim.params.dragNormal = 40.0f;
            }
            ImGui::SameLine();
            if (ImGui::Button("Water (swimming)")) {
                m_wormSim.params.dragTangent = 1.0f;
                m_wormSim.params.dragNormal = 1.7f;
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(sets the sliders below)");
            sliderWithInput("Drag - tangent (c_t)", m_wormSim.params.dragTangent, 0.05f, 10.0f, "%.2f");
            sliderWithInput("Drag - normal (c_n)", m_wormSim.params.dragNormal, 0.05f, 40.0f, "%.2f");
            sliderWithInput("Proprioceptive gain", m_wormSim.params.proprioceptiveGain, 0.0f, 8.0f, "%.2f");
        }

        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Environment", ImGuiTreeNodeFlags_DefaultOpen)) {
            sliderWithInput("Gradient gain (ASE)", m_wormSim.params.gradientGain, 0.0f, 20.0f, "%.2f");
            sliderWithInput("Spontaneous noise", m_wormSim.params.spontaneousNoise, 0.0f, 15.0f, "%.2f");
            sliderWithInput("Food deposit radius", m_wormSim.params.foodDepositRadius, 50.0f, 300.0f, "%.0f");
            sliderWithInput("Food deposit rate", m_wormSim.params.foodDepositAmount, 5.0f, 300.0f, "%.0f");
            sliderWithInput("Food consumption rate", m_wormSim.params.foodConsumptionRate, 0.0f, 40.0f, "%.1f");
        }

        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Thermotaxis (AFD)")) {
            ImGui::TextDisabled("Gradient slope=0 by default - no background pull until you raise it");
            sliderWithInput("Gradient slope (deg/unit)", m_wormSim.params.tempGradientSlope, 0.0f, 0.1f, "%.4f");
            sliderWithInput("Gradient angle (rad)", m_wormSim.params.tempGradientAngle, 0.0f, 6.2832f, "%.2f");
            sliderWithInput("Baseline temp", m_wormSim.params.tempBaseline, 0.0f, 40.0f, "%.1f");
            sliderWithInput("Cultivation temp (T_c)", m_wormSim.params.cultivationTemp, 0.0f, 40.0f, "%.1f");
            sliderWithInput("Thermo gain (AFD)", m_wormSim.params.thermoGain, -150000.0f, 0.0f, "%.0f");
        }

        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Advanced")) {
            sliderWithInput("dt (integration step)", m_wormSim.params.dt, 0.005f, 0.2f, "%.3f");
            sliderWithInput("Leak scale", m_wormSim.params.leakScale, 0.1f, 10.0f, "%.2f");
            sliderWithInput("Activation slope", m_wormSim.params.activationSlope, 0.1f, 5.0f, "%.2f");
            sliderWithInput("Intrinsic noise (all neurons)", m_wormSim.params.intrinsicNoise, 0.0f, 5.0f, "%.2f");
            sliderWithInput("Food diffusion rate", m_wormSim.params.foodDiffusionRate, 0.0f, 1.0f, "%.2f");
            sliderWithInput("Proprioceptive reach (segments)", m_wormSim.params.proprioceptiveOffset, 1.0f, 24.0f, "%.1f");
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Food tool");
        ImGui::Separator();
        ImGui::RadioButton("Add food", &m_foodToolMode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Remove food", &m_foodToolMode, 1);
        ImGui::SameLine();
        ImGui::TextDisabled("(hold LMB and drag)");
        ImGui::Text("Food in dish: %.0f", m_wormSim.totalFood());
        ImGui::SameLine();
        if (ImGui::Button("Clear food")) m_wormSim.clearFood();

        ImGui::Spacing();
        if (ImGui::Button("Snapshot neurons")) snapshotNeuronsToFile();
        ImGui::SetItemTooltip("Dumps every neuron's name/state/activation to a timestamped "
                               "CSV in the working directory (see NEURONS.md).");
        if (!m_lastSnapshotPath.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("saved: %s", m_lastSnapshotPath.c_str());
        }

        ImGui::End();

        drawNeuronGraph(snap);
#endif
    }

    void onDestroy() override {
        m_hexShader.reset();
        if (m_hexVAO) glDeleteVertexArrays(1, &m_hexVAO);
        if (m_hexVBO) glDeleteBuffers(1, &m_hexVBO);
        m_bodyShader.reset();
        if (m_bodyVAO) glDeleteVertexArrays(1, &m_bodyVAO);
        if (m_bodyVBO) glDeleteBuffers(1, &m_bodyVBO);
    }

private:
#ifdef TESSERA_IMGUI_ENABLED
    // Тема панелей ImGui в стиле "nocturne" (см. дашборд FleetOS, тот же
    // подход) - тихий почти-чёрный фон, один акцентный оттенок (фиолетовый)
    // вместо стандартного серо-синего Dear ImGui, минимум контраста в
    // состоянии покоя. Вызывается один раз в onInit() - это тема окна
    // именно этой демки (per-process ImGuiContext), другие demo/* она не
    // трогает. Активационная тепловая карта нейронов и цвет тела червя
    // ниже (см. renderBody/onImGui) в этот список НЕ входят - это данные
    // симуляции, не оформление, перекрашивать их в один акцентный оттенок
    // значило бы сделать активацию нейронов нечитаемой.
    void applyNocturneImGuiTheme() {
        ImGuiStyle& style = ImGui::GetStyle();
        ImVec4* colors = style.Colors;
        colors[ImGuiCol_WindowBg]         = ImVec4(0.075f, 0.075f, 0.110f, 0.96f); // panel
        colors[ImGuiCol_TitleBg]          = ImVec4(0.039f, 0.039f, 0.063f, 1.00f); // bg (void)
        colors[ImGuiCol_TitleBgActive]    = ImVec4(0.075f, 0.075f, 0.110f, 1.00f); // panel
        colors[ImGuiCol_Text]             = ImVec4(0.910f, 0.902f, 0.941f, 1.00f); // text
        colors[ImGuiCol_TextDisabled]     = ImVec4(0.659f, 0.643f, 0.737f, 1.00f); // text-dim
        colors[ImGuiCol_Border]           = ImVec4(0.910f, 0.902f, 0.941f, 0.12f); // line
        colors[ImGuiCol_Separator]        = ImVec4(0.910f, 0.902f, 0.941f, 0.12f);
        colors[ImGuiCol_FrameBg]          = ImVec4(0.110f, 0.110f, 0.157f, 1.00f); // panel-2
        colors[ImGuiCol_FrameBgHovered]   = ImVec4(0.157f, 0.141f, 0.220f, 1.00f);
        colors[ImGuiCol_FrameBgActive]    = ImVec4(0.200f, 0.173f, 0.282f, 1.00f);
        colors[ImGuiCol_Button]           = ImVec4(0.110f, 0.110f, 0.157f, 1.00f); // panel-2, quiet at rest
        colors[ImGuiCol_ButtonHovered]    = ImVec4(0.200f, 0.173f, 0.282f, 1.00f);
        colors[ImGuiCol_ButtonActive]     = ImVec4(0.600f, 0.565f, 0.788f, 1.00f); // amber-dim
        colors[ImGuiCol_CheckMark]        = ImVec4(0.722f, 0.651f, 1.000f, 1.00f); // amber (accent)
        colors[ImGuiCol_SliderGrab]       = ImVec4(0.600f, 0.565f, 0.788f, 1.00f); // amber-dim
        colors[ImGuiCol_SliderGrabActive] = ImVec4(0.722f, 0.651f, 1.000f, 1.00f); // amber
        colors[ImGuiCol_Header]           = ImVec4(0.110f, 0.110f, 0.157f, 1.00f);
        colors[ImGuiCol_HeaderHovered]    = ImVec4(0.200f, 0.173f, 0.282f, 1.00f);
        colors[ImGuiCol_HeaderActive]     = ImVec4(0.600f, 0.565f, 0.788f, 0.80f);
    }
#endif

    // Params хранит atomic<float>, поэтому не копируется целиком одним
    // присваиванием - собираем "чистый" временный экземпляр (его
    // конструктор по умолчанию и даёт эталонные значения) и переносим поле
    // за полем. Дешёвый выход из любой захламлённой ползунками комбинации,
    // в частности из перегретого gap junction gain (см. предупреждение ниже).
    void resetParamsToDefaults() {
        WormSim::Params d;
        m_wormSim.params.dt = d.dt.load();
        m_wormSim.params.timeScale = d.timeScale.load();
        m_wormSim.params.chemGain = d.chemGain.load();
        m_wormSim.params.gapGain = d.gapGain.load();
        m_wormSim.params.leakScale = d.leakScale.load();
        m_wormSim.params.activationTheta = d.activationTheta.load();
        m_wormSim.params.activationSlope = d.activationSlope.load();
        m_wormSim.params.bodyGain = d.bodyGain.load();
        m_wormSim.params.dragTangent = d.dragTangent.load();
        m_wormSim.params.dragNormal = d.dragNormal.load();
        m_wormSim.params.gradientGain = d.gradientGain.load();
        m_wormSim.params.spontaneousNoise = d.spontaneousNoise.load();
        m_wormSim.params.intrinsicNoise = d.intrinsicNoise.load();
        m_wormSim.params.proprioceptiveGain = d.proprioceptiveGain.load();
        m_wormSim.params.proprioceptiveOffset = d.proprioceptiveOffset.load();
        m_wormSim.params.foodDepositRadius = d.foodDepositRadius.load();
        m_wormSim.params.foodDepositAmount = d.foodDepositAmount.load();
        m_wormSim.params.foodMaxConcentration = d.foodMaxConcentration.load();
        m_wormSim.params.foodConsumptionRate = d.foodConsumptionRate.load();
        m_wormSim.params.foodDiffusionRate = d.foodDiffusionRate.load();
        m_wormSim.params.tempBaseline = d.tempBaseline.load();
        m_wormSim.params.tempGradientSlope = d.tempGradientSlope.load();
        m_wormSim.params.tempGradientAngle = d.tempGradientAngle.load();
        m_wormSim.params.cultivationTemp = d.cultivationTemp.load();
        m_wormSim.params.thermoGain = d.thermoGain.load();
    }

    // Снимок ВСЕХ узлов сети (имя, сырое состояние V, сигмоид-активация) в
    // CSV - для офлайн-анализа, тот же формат, что использовался в
    // headless-диагностиках этой сессии (см. tests/worm_locomotion).
    void snapshotNeuronsToFile() {
        WormSim::Snapshot snap;
        m_wormSim.snapshot(snap);
        const auto& names = m_wormSim.neuronNames();
        const float theta = m_wormSim.params.activationTheta.load();
        const float slope = m_wormSim.params.activationSlope.load();

        char filename[64];
        std::snprintf(filename, sizeof(filename), "neuron_snapshot_%lld.csv",
                       static_cast<long long>(std::time(nullptr)));

        std::FILE* f = std::fopen(filename, "w");
        if (!f) { m_lastSnapshotPath = "failed to open file"; return; }
        std::fprintf(f, "index,name,state,sigmoid\n");
        for (int i = 0; i < snap.nodeCount; ++i) {
            const float v = (i < static_cast<int>(snap.nodeStates.size())) ? snap.nodeStates[static_cast<std::size_t>(i)] : 0.0f;
            const float sig = 1.0f / (1.0f + std::exp(-(v - theta) / std::max(slope, 1e-6f)));
            const char* name = (i < static_cast<int>(names.size())) ? names[static_cast<std::size_t>(i)].c_str() : "";
            std::fprintf(f, "%d,%s,%.6f,%.6f\n", i, name, v, sig);
        }
        std::fclose(f);
        m_lastSnapshotPath = filename;
    }

    // Кисть добавления/стирания еды - действует, пока зажата ЛКМ (не только
    // на клик), чтобы рисовать/стирать перетаскиванием как настоящей кистью.
    void handleFoodPaint(float dt) {
        float mx, my;
        m_input.getMousePosition(mx, my);
        bool lmb = m_input.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT) && !m_imguiWantMouse.load();
        if (lmb) {
            glm::vec2 world = getCamera().screenToWorld(mx, my);
            if (m_foodToolMode == 0) m_wormSim.depositFood(world, dt);
            else m_wormSim.removeFood(world, dt);
        }
    }

    void initHexShader() {
        m_hexShader = std::make_unique<Shader>("Shaders/hex_point.vert", "Shaders/hex_point.frag");
        glGenVertexArrays(1, &m_hexVAO);
        glGenBuffers(1, &m_hexVBO);
        glBindVertexArray(m_hexVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_hexVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    // Базовая яркостная "текстура" земли (hash01) считается один раз - цвет
    // каждой точки на экране = эта базовая подсветка, подмешанная с текущей
    // концентрацией еды в её клетке (см. renderGround) - в отличие от старой
    // версии, буфер теперь перестраивается каждый кадр (живой газон), а не
    // статично один раз.
    void initHexField() {
        m_groundShade.resize(static_cast<std::size_t>(kHexCols) * kHexRows);
        for (int row = 0; row < kHexRows; ++row)
            for (int col = 0; col < kHexCols; ++col)
                m_groundShade[static_cast<std::size_t>(row) * kHexCols + col] = 0.75f + 0.25f * hash01(col, row);
    }

    void initBodyShader() {
        m_bodyShader = std::make_unique<Shader>("Shaders/worm_body.vert", "Shaders/worm_body.frag");
        glGenVertexArrays(1, &m_bodyVAO);
        glGenBuffers(1, &m_bodyVBO);
        glBindVertexArray(m_bodyVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_bodyVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glBindVertexArray(0);
    }

    // Каждая точка гекс-поля = одна клетка непрерывного поля еды WormSim (1:1,
    // см. setBounds в onInit). Цвет = базовая земляная подсветка, смешанная с
    // "цветом еды" пропорционально текущей концентрации в этой клетке -
    // газон буквально светится там, где он есть, и гаснет по мере поедания/
    // стирания. Перестраивается каждый кадр (концентрация живая), как
    // renderBody уже делает для тела.
    void renderGround(const Camera2D& camera) {
        std::vector<float> field = m_wormSim.foodFieldSnapshot();
        const int cols = m_wormSim.foodFieldCols();
        const int rows = m_wormSim.foodFieldRows();
        const float maxConc = std::max(1.0f, m_wormSim.params.foodMaxConcentration.load());

        std::vector<float> verts;
        verts.reserve(static_cast<std::size_t>(cols) * rows * 5);
        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                glm::vec2 p = HexGrid::worldPos(col, row, kHexSpacing);
                const float shade = m_groundShade[static_cast<std::size_t>(row) * cols + col];
                const float conc = field.empty() ? 0.0f : field[static_cast<std::size_t>(row) * cols + col];
                const float t = std::clamp(conc / maxConc, 0.0f, 1.0f);
                // база: приглушённая земляная зелень; еда: тёплый жёлто-оранжевый.
                const float r = (0.20f * shade) + t * (0.85f - 0.20f * shade);
                const float g = (0.24f * shade) + t * (0.60f - 0.24f * shade);
                const float b = (0.15f * shade) + t * (0.12f - 0.15f * shade);
                verts.push_back(p.x);
                verts.push_back(p.y);
                verts.push_back(r);
                verts.push_back(g);
                verts.push_back(b);
            }
        }

        glBindBuffer(GL_ARRAY_BUFFER, m_hexVBO);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STREAM_DRAW);

        m_hexShader->use();
        m_hexShader->setMat4("uCamera", camera.getViewProjectionMatrix());
        m_hexShader->setFloat("uBaseSize", kPointBaseSize);
        m_hexShader->setFloat("uCellSizePx", kHexSpacing * 2.0f * camera.zoom + 1.0f);
        m_hexShader->setInt("uShapeMode", 0); // честный шестиугольник
        glBindVertexArray(m_hexVAO);
        glDrawArrays(GL_POINTS, 0, static_cast<int>(verts.size() / 5));
    }

    // Строит треугольную ленту вдоль центральной линии тела: для каждой
    // точки — нормаль из соседей вдоль ленты, полуширина сужается к
    // голове/хвосту синусом (не квадратный обрубок на концах). Раскладка
    // left0,right0,left1,right1,... - ровно то, что ждёт GL_TRIANGLE_STRIP.
    // Точки уже в мировых координатах (WormSim учитывает position/heading).
    void renderBody(const Camera2D& camera) {
        WormSim::Snapshot snap;
        m_wormSim.snapshot(snap);
        const int n = static_cast<int>(snap.pointsX.size());
        if (n < 2) return;

        constexpr float kBaseHalfWidth = 6.0f;
        m_bodyVertexData.resize(static_cast<std::size_t>(n) * 2 * 4);
        for (int i = 0; i < n; ++i) {
            glm::vec2 p(snap.pointsX[static_cast<std::size_t>(i)], snap.pointsY[static_cast<std::size_t>(i)]);
            glm::vec2 prev = i > 0 ? glm::vec2(snap.pointsX[static_cast<std::size_t>(i - 1)],
                                                snap.pointsY[static_cast<std::size_t>(i - 1)])
                                    : p;
            glm::vec2 next = i < n - 1 ? glm::vec2(snap.pointsX[static_cast<std::size_t>(i + 1)],
                                                     snap.pointsY[static_cast<std::size_t>(i + 1)])
                                        : p;
            glm::vec2 tangent = next - prev;
            float len = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
            glm::vec2 normal = len > 1e-5f ? glm::vec2(-tangent.y / len, tangent.x / len) : glm::vec2(0.0f, 1.0f);

            float t = static_cast<float>(i) / static_cast<float>(n - 1);
            float halfWidth = kBaseHalfWidth * (0.15f + 0.85f * std::sin(3.14159265f * t));
            float glow = snap.glow[static_cast<std::size_t>(i)];

            std::size_t base = static_cast<std::size_t>(i) * 8;
            m_bodyVertexData[base + 0] = p.x + normal.x * halfWidth;
            m_bodyVertexData[base + 1] = p.y + normal.y * halfWidth;
            m_bodyVertexData[base + 2] = glow;
            m_bodyVertexData[base + 3] = t;
            m_bodyVertexData[base + 4] = p.x - normal.x * halfWidth;
            m_bodyVertexData[base + 5] = p.y - normal.y * halfWidth;
            m_bodyVertexData[base + 6] = glow;
            m_bodyVertexData[base + 7] = t;
        }

        glBindBuffer(GL_ARRAY_BUFFER, m_bodyVBO);
        glBufferData(GL_ARRAY_BUFFER, m_bodyVertexData.size() * sizeof(float), m_bodyVertexData.data(),
                     GL_STREAM_DRAW);

        m_bodyShader->use();
        m_bodyShader->setMat4("uCamera", camera.getViewProjectionMatrix());
        glBindVertexArray(m_bodyVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, n * 2);
    }

#ifdef TESSERA_IMGUI_ENABLED
    // tooltip - необязательное ОДНО короткое предложение для "(?)" маркера,
    // не параграф: длинные объяснения/цитаты живут в комментариях у места
    // вызова, не в самом ImGui - см. коммит, сокративший эту панель.
    static void sliderWithInput(const char* label, std::atomic<float>& value, float lo, float hi,
                                 const char* fmt, const char* tooltip = nullptr) {
        ImGui::PushID(label);
        float v = value.load();
        ImGui::SetNextItemWidth(120);
        // Без ImGuiSliderFlags_AlwaysClamp Ctrl+клик на слайдере переключает
        // его в режим текстового ввода, который НЕ обязан укладываться в
        // [lo,hi] (известная особенность ImGui) - живой пример: Time scale
        // оказался выставлен в 80.00x при заявленном максимуме слайдера 8.0x
        // (10-кратный обгон, конкретно на этом слайдере), из-за чего рендер
        // показывает лишь каждый ~десятки-й шаг сети (визуально - "конвульсии")
        // и, за счёт непропорционально длинного симулированного времени на
        // единицу реального, сеть успевает уползти к своему большому
        // собственному равновесию (см. KNOWN OPEN ISSUE в tests/worm_locomotion) -
        // мышцы (Output, leak=0 по конструкции) утыкаются в него быстрее всех.
        // ImGuiSliderFlags_AlwaysClamp - штатный, а не самодельный способ не
        // дать значению выйти за границы через этот путь; дублирующий
        // std::clamp ниже - подстраховка на случай, если значение попало в
        // атомик как-то ещё, а не через сам этот виджет.
        ImGui::SliderFloat("##s", &v, lo, hi, fmt, ImGuiSliderFlags_AlwaysClamp);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        ImGui::InputFloat("##i", &v, 0.0f, 0.0f, fmt);
        ImGui::SameLine();
        ImGui::TextUnformatted(label);
        if (tooltip) {
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            ImGui::SetItemTooltip("%s", tooltip);
        }
        value = std::clamp(v, lo, hi);
        ImGui::PopID();
    }

    // 401 узел раскрашены по активации (sigmoid(state): синий тормозной ->
    // красный возбуждённый), сгруппированы по типу (см.
    // WormSim::nodeLayoutX/Y) - без рёбер, на этом масштабе они были бы
    // нечитаемым волосяным шаром, сами точки уже показывают, что где горит.
    // "(?)" - легенда столбцов/цвета по наведению; сам скаттер тоже
    // интерактивен - наведение на точку подсвечивает её и называет нейрон
    // (единственный способ опознать конкретный узел без подписей на канве).
    void drawNeuronGraph(const WormSim::Snapshot& snap) {
        ImGui::SetNextWindowPos(ImVec2(360, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(320, 380), ImGuiCond_FirstUseEver);
        ImGui::Begin("Neurons");
        ImGui::TextDisabled("(?)");
        ImGui::SetItemTooltip(
            "Left to right: sensory, sensory+processing, interneurons, "
            "command/motor, muscle (dorsal=upper right, ventral=lower right).\n"
            "Color: blue = low activation, red = high. Hover a dot to name it.");
        ImVec2 canvasPos = ImGui::GetCursorScreenPos();
        ImVec2 canvasSize(280.0f, 300.0f);
        ImVec2 canvasEnd(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(canvasPos, canvasEnd, IM_COL32(10, 10, 16, 255)); // nocturne void
        const auto& lx = m_wormSim.nodeLayoutX();
        const auto& ly = m_wormSim.nodeLayoutY();
        const auto& names = m_wormSim.neuronNames();

        const bool overCanvas = ImGui::IsMouseHoveringRect(canvasPos, canvasEnd);
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        int hovered = -1;
        float bestDistSq = 36.0f; // ~6px подбор под курсор
        for (int i = 0; i < snap.nodeCount; ++i) {
            float a = 1.0f / (1.0f + std::exp(-snap.nodeStates[static_cast<std::size_t>(i)]));
            ImU32 col = IM_COL32(static_cast<int>(a * 255), 70, static_cast<int>((1.0f - a) * 255), 255);
            ImVec2 p(canvasPos.x + lx[static_cast<std::size_t>(i)] * canvasSize.x,
                      canvasPos.y + ly[static_cast<std::size_t>(i)] * canvasSize.y);
            dl->AddCircleFilled(p, 2.0f, col);
            if (!overCanvas) continue;
            float dx = mouse.x - p.x, dy = mouse.y - p.y;
            float d2 = dx * dx + dy * dy;
            if (d2 < bestDistSq) { bestDistSq = d2; hovered = i; }
        }
        if (hovered >= 0) {
            ImVec2 hp(canvasPos.x + lx[static_cast<std::size_t>(hovered)] * canvasSize.x,
                       canvasPos.y + ly[static_cast<std::size_t>(hovered)] * canvasSize.y);
            dl->AddCircle(hp, 5.0f, IM_COL32(255, 255, 255, 255), 12, 1.5f);
            const char* nm = (hovered < static_cast<int>(names.size())) ? names[static_cast<std::size_t>(hovered)].c_str() : "?";
            ImGui::SetTooltip("%s  (state=%.2f)", nm, snap.nodeStates[static_cast<std::size_t>(hovered)]);
        }
        ImGui::Dummy(canvasSize);
        ImGui::End();
    }
#endif

    WormSim m_wormSim;
    int m_foodToolMode = 0; // 0 = add, 1 = remove
    float m_stepAccumulator = 0.0f; // fixed-timestep accumulator, see onUpdate
    bool m_themeApplied = false; // applyNocturneImGuiTheme() one-time-apply guard, see onImGui
    std::string m_lastSnapshotPath; // for UI feedback after "Snapshot neurons", see snapshotNeuronsToFile

    std::unique_ptr<Shader> m_hexShader;
    unsigned int m_hexVAO = 0, m_hexVBO = 0;
    std::vector<float> m_groundShade; // precomputed base earth jitter, kHexCols*kHexRows

    std::unique_ptr<Shader> m_bodyShader;
    unsigned int m_bodyVAO = 0, m_bodyVBO = 0;
    std::vector<float> m_bodyVertexData;
};

int main() {
    try {
        WormApp app;
        app.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[worm] fatal: %s\n", e.what());
        return 1;
    }
    return 0;
}
