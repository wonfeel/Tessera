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
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <memory>
#include <vector>

#include "engine/core/HexGrid.h"

#ifdef TESSERA_IMGUI_ENABLED
#  include <imgui.h>
#  include <imgui_impl_opengl3.h>
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
        // fieldCols/fieldRows == kHexCols/kHexRows: 1 гекс = 1 клетка поля
        // еды. WormSim теперь считает boundsMax сам по той же гекс-формуле
        // (HexGrid::worldPos), а не по прямоугольному приближению - иначе
        // покраска/нюх промахивались мимо клетки, которая реально светится.
        m_wormSim.setBounds(glm::vec2(0.0f), kHexCols, kHexRows, kHexSpacing);

        frameCamera(glm::vec2(0.0f), worldMax, kHexSpacing * 2.0f);

        glEnable(GL_PROGRAM_POINT_SIZE);
        // Фон вьюпорта - тон "nocturne" (тихий почти-чёрный с лёгким
        // фиолетовым оттенком) вместо прежнего зеленоватого - см.
        // applyNocturneImGuiTheme(): то же самое для панелей ImGui (вызов
        // не отсюда - ImGui-контекст на момент onInit() ещё не создан,
        // imguiInit() в Application.cpp происходит позже, в renderLoop();
        // тема применяется через onImGuiInit(), см. переопределение ниже).
        // Цвета САМОЙ симуляции (тело червя, тепловая карта нейронов,
        // подсветка еды) не трогаем - это данные, не декор.
        glClearColor(0.039f, 0.039f, 0.063f, 1.0f);
    }

    // Шрифт/тема грузятся здесь, а не в onImGui() - это единственное место
    // между созданием ImGui-контекста (imguiInit(), Application.cpp) и
    // первым ImGui::NewFrame(), где атлас шрифтов ещё НЕ заблокирован.
    // AddFontFromFileTTF из onImGui() валил приложение сразу на старте:
    // "Debug Error! abort() has been called" / imgui_draw.cpp:2628
    // "Cannot modify a locked ImFontAtlas..." - воспроизведено и проверено
    // на этой машине (onImGui() выполняется МЕЖДУ NewFrame() и Render(),
    // см. Application.h за полным объяснением).
    void onImGuiInit() override {
#ifdef TESSERA_IMGUI_ENABLED
        applyNocturneImGuiTheme();
#endif
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
        m_prevRenderSnap = m_currRenderSnap; // состояние на конец ПРОШЛОГО кадра - см. m_prevRenderSnap за обоснованием
        m_stepAccumulator += dt * m_wormSim.params.timeScale.load();
        const float simDt = std::max(0.0005f, m_wormSim.params.dt.load());
        // Предохранитель поднят 200 -> 20000 вместе со снятием потолка Time
        // scale. Он не задаёт скорость симуляции и никогда для этого не
        // предназначался - он не даёт зависшему кадру уйти в неограниченный
        // цикл догона (накопитель растёт быстрее, чем цикл его вычерпывает, и
        // приложение перестаёт отвечать). При 60 кадрах в секунду и dt=0.05 это
        // примерно 6000-кратное реальное время - заведомо выше любого
        // осмысленного множителя.
        int guard = 0;
        while (m_stepAccumulator >= simDt && guard < 20000) {
            m_wormSim.step();
            m_stepAccumulator -= simDt;
            ++guard;
        }
        m_wormSim.snapshot(m_currRenderSnap); // состояние на конец ЭТОГО кадра
        if (!m_haveRenderSnap) { m_prevRenderSnap = m_currRenderSnap; m_haveRenderSnap = true; } // первый кадр - интерполировать не от чего
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
        // repeat=false - один toggle за нажатие, а не за каждый кадр, пока
        // клавиша зажата (IsKeyPressed без этого параметра следует
        // io.KeyRepeat, как и обычные текстовые поля).
        if (ImGui::IsKeyPressed(ImGuiKey_F10, false)) m_showDebugUI = !m_showDebugUI;
        if (!m_showDebugUI) return;

        if (m_monoFont) ImGui::PushFont(m_monoFont);

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
        // Максимум снижен 100x -> 8x. Причина - жалоба владельца проекта "он в
        // моменте умеет ускоряться в десятки раз (я хз когда)": в самой
        // симуляции выбросов скорости НЕТ (замерено, burstRatio ~3 и ни одного
        // шага выше 5x медианы на 8 базах x 3 сида), значит десятикратное
        // ускорение приходило от этого ползунка. ImGui-слайдер прыгает в точку
        // клика, так что один случайный клик у правого края при диапазоне до
        // 100x давал ровно "в десятки раз быстрее" - и понять, когда это
        // случилось, было невозможно. 8x достаточно для обзора, и случайный
        // клик теперь стоит максимум 8x. Значения выше по-прежнему достижимы
        // через поле ввода рядом.
        // ПОТОЛОК СНЯТ по прямому требованию. Он стоял на 8x после того, как
        // Ctrl+клик по ползунку однажды забросил воспроизведение на 100x, и это
        // читалось как "червь внезапно ускорился в десятки раз". Ограничение
        // было косметическим: сама симуляция при большом множителе не ломается,
        // просто рендер показывает лишь каждый N-й шаг сети, и движение
        // выглядит рваным. Значение осталось на своём месте - убрана только
        // граница, за которую нельзя было зайти мышью.
        sliderWithInput("Time scale (playback speed)", m_wormSim.params.timeScale, 0.0f, 1000.0f, "%.2fx",
                         "0x pauses the simulation. No upper cap: the simulation itself stays correct at any "
                         "multiplier, only the rendering thins out - at high values the frame shows every Nth "
                         "network step, so motion reads as choppy rather than fast. The per-frame substep guard "
                         "(20000) exists solely so a stalled frame cannot spiral into an unbounded catch-up loop; "
                         "at 60fps it allows roughly 6000x real time.");

        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Network")) {
            sliderWithInput("Chemical synapse gain", m_wormSim.params.chemGain, 0.0f, 0.5f, "%.4f");
            sliderWithInput("Gap junction gain", m_wormSim.params.gapGain, 0.0f, 0.15f, "%.4f",
                             "Too high freezes the worm solid - first thing to check if it goes rigid.");
            // Диапазон 0..5 не покрывал даже прежний дефолт (200), не говоря о
            // текущем (1200) - см. sliderWithInput за разбором того, что из
            // этого следовало.
            sliderWithInput("Body curvature gain", m_wormSim.params.bodyGain, 0.0f, 8000.0f, "%.0f",
                            "Scales network curvature output into the body. Co-scaled with pose decay: their RATIO "
                            "sets bend amplitude, the decay rate alone sets tempo. See WORM_V5_SPATIAL_ENVELOPE_"
                            "DIAGNOSIS.md section 14.");
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
            // setMedium (не два отдельных присваивания) - см. WormSim.h за
            // тем, какую гонку между UI- и update-потоком это закрывает
            // (step() мог раньше прочитать наполовину обновлённую пару).
            if (ImGui::Button("Agar (crawling)")) {
                m_wormSim.setMedium(1.0f, 40.0f);
            }
            ImGui::SameLine();
            if (ImGui::Button("Water (swimming)")) {
                m_wormSim.setMedium(1.0f, 1.7f);
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(sets the sliders below)");
            sliderWithInput("Drag - tangent (c_t)", m_wormSim.params.dragTangent, 0.05f, 10.0f, "%.2f");
            sliderWithInput("Drag - normal (c_n)", m_wormSim.params.dragNormal, 0.05f, 40.0f, "%.2f");
            sliderWithInput("Drag settle gain (adhesion memory)", m_wormSim.params.dragSettleGain, 0.0f, 300.0f,
                             "%.1f", "c_n += gain*penetration_depth (Kelvin-Voigt memory, tau=dragSettleTau); replaces v1's three instantaneous adhesion formulas, see WORM_V2_DESIGN.md section 4");
            sliderWithInput("Drag settle tau (s)", m_wormSim.params.dragSettleTau, 0.1f, 10.0f, "%.2f");
            sliderWithInput("Body frame rate limit (Hz)", m_wormSim.params.bodyFrameRateLimitHz, 0.1f, 10.0f, "%.2f",
                             "max angular rate at which a joint's accumulated orientation may chase its target, see WORM_V2_DESIGN.md section 3");
            // Нет отдельного слайдера bodyPoseDecayRate в этом UI (только
            // Params-поле) - размещено здесь, рядом с остальными body-
            // kinematics слайдерами этой же секции, а не буквально "рядом с
            // bodyPoseDecayRate", как в тексте WORM_V3_DESIGN.md раздела 2.4
            // (тот слайдер не существует) - тот же паттерн sliderWithInput,
            // что у всех остальных v2/v3 параметров тела.
            sliderWithInput("Body bend stiffness (spatial, neighbor coupling)", m_wormSim.params.bodyBendStiffness,
                             0.0f, 50.0f, "%.2f",
                             "Discrete-Laplacian resistance to curvature DIFFERENCE between neighboring joints (free-free BCs) - spatial mechanical continuity of the cuticle, see WORM_V3_DESIGN.md section 2. Lives entirely in angle kinematics, never touches the 3x3 force-balance solve.");
            sliderWithInput("Proprioceptive gain", m_wormSim.params.proprioceptiveGain, 0.0f, 8.0f, "%.2f");
            sliderWithInput("Drive equalization", m_wormSim.params.driveEqualizationGain, 0.0f, 2.0f, "%.2f",
                            "Per-position homeostatic scaling of the curvature drive: weak body positions are boosted, "
                            "strong ones attenuated, using a long-time-constant average of each position's own "
                            "amplitude. This is what makes the wave span the WHOLE body - without it the ventral cord "
                            "innervation gap leaves positions 1-5 and 23-24 nearly straight. 0 = off. See "
                            "WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md section 23.");
            sliderWithInput("Water amplitude ratio", m_wormSim.params.mediumAmplitudeWaterRatio, 0.1f, 1.0f, "%.2f",
                            "Bend amplitude in water relative to agar, applied by scaling the joint angle limit (not "
                            "the drive - scaling the drive collapses through a bifurcation instead of reducing "
                            "amplitude smoothly). Real C. elegans bends LESS in water: 45 deg swimming vs 135 deg "
                            "crawling, i.e. a ratio of 1/3. Shipped at the cited 0.333: the earlier 0.45 existed only "
                            "because 0.333 appeared to bring back speed spikes, and those turned out to be the wall "
                            "artifact (see section 29), not the water. Costs 22% of water speed. 1.0 = no medium "
                            "dependence.");
            sliderWithInput("Joint angle rate limit (rad/s)", m_wormSim.params.jointAngleRateLimit, 0.0f, 2.0f, "%.2f",
                            "How fast a joint may change angle. This is the entire ceiling on amplitude x frequency: "
                            "measured f*A = rateLimit/4 to within 5% across five independent points. It was held at "
                            "0.25 because higher values produced 'speed spikes' - which turned out to be the wall, not "
                            "locomotion (containBody pivoted the body about its HEAD, sweeping the centroid sideways at "
                            "1.26 BL/s). With the wall fixed, 0.5 gives +13% speed, +21% frequency and a bend peak "
                            "inside the biological range. Only useful together with a faster network - on its own it "
                            "just starves the tempo. See section 29.");
            sliderWithInput("Joint angle limit (rad)", m_wormSim.params.jointAngleClamp, 0.1f, 1.2f, "%.2f",
                            "Per-joint bend ceiling. Only safe to raise toward the biological 0.49-0.59 range once "
                            "drive equalization is on - before that, this clamp was the only thing making the wave "
                            "body-wide. 0.40 specifically is a bad value (speed spikes in 11/16 bases); 0.55 is clean.");
        }

        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Pirouettes (reversal + omega turn)", ImGuiTreeNodeFlags_DefaultOpen)) {
            // Живой индикатор фазы: реверс и омега кратковременны (вместе ~10%
            // времени), и без него на демке легко решить, что механизм не
            // работает, просто не застав его.
            static const char* kPhaseNames[3] = {"FORWARD", "REVERSE", "OMEGA"};
            const int phase = m_wormSim.debugLocomotionPhase();
            const ImVec4 kPhaseColors[3] = {ImVec4(0.6f, 0.9f, 0.6f, 1.0f), ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                                            ImVec4(1.0f, 0.45f, 0.45f, 1.0f)};
            ImGui::TextColored(kPhaseColors[std::clamp(phase, 0, 2)], "phase: %s",
                               kPhaseNames[std::clamp(phase, 0, 2)]);
            ImGui::SameLine();
            ImGui::TextDisabled("| d(scent)/dt = %+.4f", m_wormSim.debugScentRate());
            checkboxParam("Pirouettes enabled", m_wormSim.params.pirouetteEnabled,
                          "Pierce-Shimomura, Morse & Lockery 1999: the worm runs straight, and the RATE of sharp "
                          "reorientations (reversal, often followed by an omega turn) rises when it moves down the "
                          "chemical gradient and falls when it moves up. This biased random walk is the main "
                          "navigation strategy of the real animal - the steering channel above is the other one. "
                          "The reversal itself is not a kinematic trick: it flips the direction of the "
                          "proprioceptive window, i.e. swaps the active motor class (B-class forward, A-class "
                          "backward), and the bending wave reverses on its own.");
            sliderWithInput("Reversal base rate (1/s)", m_wormSim.params.reversalBaseRate, 0.0f, 0.2f, "%.4f",
                            "Spontaneous reversal rate at zero gradient. 0.025 = one per 40s.");
            sliderWithInput("Reversal gradient gain", m_wormSim.params.reversalGradientGain, 0.0f, 5000.0f, "%.0f",
                            "How strongly d(scent)/dt biases the reversal rate: rate = base * exp(-gain * dC/dt). "
                            "Zero makes the walk unbiased - reversals still happen, but carry no information.");
            sliderWithInput("Omega probability", m_wormSim.params.omegaProbability, 0.0f, 1.0f, "%.2f",
                            "Fraction of reversals that end in an omega turn rather than simply resuming forward.");
            sliderWithInput("Omega duration (s)", m_wormSim.params.omegaDuration, 0.2f, 6.0f, "%.2f");
            sliderWithInput("Omega bend bias (rad)", m_wormSim.params.omegaBendBias, 0.0f, 1.2f, "%.2f",
                            "Depth of the one-sided bend during an omega turn. The reorientation itself comes from "
                            "the same friction physics as ordinary crawling - this only sets the shape.");
            sliderWithInput("Omega clamp scale", m_wormSim.params.omegaClampScale, 1.0f, 3.0f, "%.2f",
                            "The joint ceiling and rate limit are set for NORMAL locomotion. An omega turn is a "
                            "different regime - head reaches the tail - so both are raised for its duration. "
                            "Without this the turn came out at 23 degrees instead of the measured 82.");
            sliderWithInput("Omega rate limit scale", m_wormSim.params.omegaRateLimitScale, 1.0f, 4.0f, "%.2f");
        }

        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Environment", ImGuiTreeNodeFlags_DefaultOpen)) {
            sliderWithInput("Gradient gain (ASE)", m_wormSim.params.gradientGain, 0.0f, 20.0f, "%.2f");
            // Диапазон обязан включать ОТРИЦАТЕЛЬНЫЕ значения: отгруженное -
            // -2.0, и именно отрицательный знак приводит червя К еде
            // (положительный уводит от неё). С прежним 0..4 значение
            // зажималось в 0, то есть демка шла с выключенным хемотаксисом.
            sliderWithInput("Chemo steering gain", m_wormSim.params.chemoSteeringGain, -8.0f, 8.0f, "%.2f",
                            "Weathervaning steering (Iino & Yoshida 2009) - lateral scent gradient sampled left/right "
                            "of the head, biasing the anterior joints. Added AFTER the spatial mean-subtract and the "
                            "temporal baseline, which erase any body-wide curvature bias - that is why the ASE gradient "
                            "channel above cannot steer no matter how it is tuned. See WORM_V5_SPATIAL_ENVELOPE_"
                            "DIAGNOSIS.md sections 11 and 13. 0 = off, bitwise prior behavior.");
            sliderWithInput("Spontaneous noise", m_wormSim.params.spontaneousNoise, 0.0f, 15.0f, "%.2f");
            // Верхние границы расширены до assay-масштаба (WORM_V5_SPATIAL_
            // ENVELOPE_DIAGNOSIS.md разделы 15-16): измеренный хемотаксис
            // получен при радиусе 1500, потолке концентрации 1000 и диффузии
            // 0.9, тогда как прежний максимум ползунка (300) и дефолтный
            // потолок 6 давали градиент шириной ~1 длины тела, по которому
            // рулить нечем - и именно на таком поле хемотаксис в этом проекте
            // годами не измерялся. Дефолты не тронуты; расширены только
            // границы, чтобы проверенный режим был достижим из UI.
            sliderWithInput("Food deposit radius", m_wormSim.params.foodDepositRadius, 50.0f, 2000.0f, "%.0f");
            sliderWithInput("Food max concentration", m_wormSim.params.foodMaxConcentration, 1.0f, 1000.0f, "%.0f",
                            "Per-cell concentration ceiling. Too low turns a large food patch into a flat PLATEAU "
                            "where the left/right scent difference is exactly zero and steering has nothing to work "
                            "with. Measured chemotaxis used 1000.");
            sliderWithInput("Food diffusion rate", m_wormSim.params.foodDiffusionRate, 0.0f, 1.0f, "%.2f",
                            "How fast the patch spreads. Sets how far the gradient reaches: the worm covers ~10 body "
                            "lengths per 250s, so a gradient confined to ~1 body length is transited before it can "
                            "steer. Measured chemotaxis used 0.9.");
            sliderWithInput("Food deposit rate", m_wormSim.params.foodDepositAmount, 5.0f, 300.0f, "%.0f");
            // Диапазон 0..40 не покрывал новый дефолт 100 (поднят вместе с
            // foodMaxConcentration 6 -> 200, чтобы пятно выедалось за прежнее
            // время). Верхняя граница 200, но выше 100 измеренно растут выбросы
            // мгновенной скорости - см. Params::foodConsumptionRate.
            sliderWithInput("Food consumption rate", m_wormSim.params.foodConsumptionRate, 0.0f, 200.0f, "%.1f",
                            "Absolute field units eaten per second. Scaled with foodMaxConcentration - above ~100 the "
                            "eaten hole becomes a sharp enough step in the field to raise instantaneous speed spikes.");
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
        drawBodyAngles();
        if (m_monoFont) ImGui::PopFont();
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
    // Тема панелей ImGui в стиле "nocturne" (research-production edition -
    // см. .claude/skills/nocturne, тот же дашборд FleetOS уже на ней)
    // - тихий почти-чёрный фон, один стандартизованный акцент (индиго
    // #8fa6ff, не прежний фиолетовый), тот же радиус скругления и тот же
    // моноширинный шрифт, что и везде в системе. Вызывается один раз из
    // onImGuiInit() (не onInit() - там ImGui-контекста ещё нет; не
    // onImGui() - там уже поздно, атлас шрифтов заблокирован, см. ниже) -
    // это тема окна именно этой демки (per-process ImGuiContext), другие
    // demo/* она не трогает.
    //
    // Активационная тепловая карта нейронов (см. onImGui ниже) и цвет
    // тела/еды червя (renderBody/renderGround) в этот список НЕ входят -
    // но по разным причинам. Тепловая карта - диагностическое значение
    // (activation), по новому формальному правилу data-encoding в
    // palette.md её нужно рисовать по фиксированной шкале danger-surface2-
    // accent, а не тут перекрашивать саму тему. Цвет тела/еды - не данные
    // приборной панели вообще, а внешний вид самого симулируемого
    // организма/местности, той же природы исключение, что и терминал
    // FleetOS - его red-green-authentic вид не подчиняется теме приложения.
    void applyNocturneImGuiTheme() {
        ImGuiStyle& style = ImGui::GetStyle();
        ImVec4* colors = style.Colors;
        colors[ImGuiCol_WindowBg]         = ImVec4(0.075f, 0.075f, 0.110f, 0.96f); // surface
        colors[ImGuiCol_TitleBg]          = ImVec4(0.039f, 0.039f, 0.063f, 1.00f); // void
        colors[ImGuiCol_TitleBgActive]    = ImVec4(0.075f, 0.075f, 0.110f, 1.00f); // surface
        colors[ImGuiCol_Text]             = ImVec4(0.910f, 0.902f, 0.941f, 1.00f); // text
        colors[ImGuiCol_TextDisabled]     = ImVec4(0.659f, 0.643f, 0.737f, 1.00f); // text-dim
        colors[ImGuiCol_Border]           = ImVec4(0.910f, 0.902f, 0.941f, 0.12f); // border
        colors[ImGuiCol_Separator]        = ImVec4(0.910f, 0.902f, 0.941f, 0.12f);
        colors[ImGuiCol_FrameBg]          = ImVec4(0.110f, 0.110f, 0.157f, 1.00f); // surface-2
        colors[ImGuiCol_FrameBgHovered]   = ImVec4(0.190f, 0.210f, 0.310f, 1.00f);
        colors[ImGuiCol_FrameBgActive]    = ImVec4(0.260f, 0.290f, 0.440f, 1.00f);
        colors[ImGuiCol_Button]           = ImVec4(0.110f, 0.110f, 0.157f, 1.00f); // surface-2, quiet at rest
        colors[ImGuiCol_ButtonHovered]    = ImVec4(0.190f, 0.210f, 0.310f, 1.00f);
        colors[ImGuiCol_ButtonActive]     = ImVec4(0.435f, 0.502f, 0.788f, 1.00f); // accent-dim
        colors[ImGuiCol_CheckMark]        = ImVec4(0.561f, 0.651f, 1.000f, 1.00f); // accent
        colors[ImGuiCol_SliderGrab]       = ImVec4(0.435f, 0.502f, 0.788f, 1.00f); // accent-dim
        colors[ImGuiCol_SliderGrabActive] = ImVec4(0.561f, 0.651f, 1.000f, 1.00f); // accent
        colors[ImGuiCol_Header]           = ImVec4(0.110f, 0.110f, 0.157f, 1.00f);
        colors[ImGuiCol_HeaderHovered]    = ImVec4(0.190f, 0.210f, 0.310f, 1.00f);
        colors[ImGuiCol_HeaderActive]     = ImVec4(0.435f, 0.502f, 0.788f, 0.80f);

        // patterns.md: --noc-radius 8px / --noc-radius-sm 5px.
        style.WindowRounding = style.ChildRounding = style.PopupRounding = 8.0f;
        style.FrameRounding = style.GrabRounding = style.ScrollbarRounding = style.TabRounding = 5.0f;

        // typography.md: one monospace face everywhere. ADDS to the atlas
        // (doesn't replace Application.cpp's Segoe UI, which every other
        // demo/* still needs - see imguiInit()'s Cyrillic-coverage comment)
        // and re-uploads the GPU texture, then onImGui() below PushFont()s
        // it around just this demo's panels. Same exists-check-first
        // pattern as imguiInit() uses for Segoe UI, for the same reason -
        // AddFontFromFileTTF on a missing path hits an assert in this
        // build, not a quiet nullptr.
        const char* kMonoFontPath = "C:\\Windows\\Fonts\\consola.ttf";
        if (std::filesystem::exists(kMonoFontPath)) {
            ImGuiIO& io = ImGui::GetIO();
            m_monoFont = io.Fonts->AddFontFromFileTTF(kMonoFontPath, 18.0f, nullptr, io.Fonts->GetGlyphRangesCyrillic());
            if (m_monoFont) ImGui_ImplOpenGL3_CreateFontsTexture();
        }
    }

    // Neuron activation is a real per-neuron value (0=quiet, 1=firing),
    // not decoration - palette.md's data-encoding section exempts this
    // exact case (a diverging quantity needs two distinguishable ends,
    // one accent hue can't represent "low vs high") and names the fixed
    // scale to use instead of an ad-hoc saturated gradient: accent (low)
    // -> surface-2 (mid) -> danger (high). Used by drawNeuronGraph().
    static ImU32 nocturneActivationColor(float t) {
        struct RGB { float r, g, b; };
        constexpr RGB kAccent{143.0f, 166.0f, 255.0f}, kMid{28.0f, 28.0f, 40.0f}, kDanger{217.0f, 143.0f, 143.0f};
        const RGB& from = (t < 0.5f) ? kAccent : kMid;
        const RGB& to   = (t < 0.5f) ? kMid : kDanger;
        const float u = (t < 0.5f) ? (t * 2.0f) : ((t - 0.5f) * 2.0f);
        return IM_COL32(static_cast<int>(from.r + (to.r - from.r) * u),
                         static_cast<int>(from.g + (to.g - from.g) * u),
                         static_cast<int>(from.b + (to.b - from.b) * u), 255);
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
        m_wormSim.params.bodyBendStiffness = d.bodyBendStiffness.load();
        m_wormSim.setMedium(d.dragTangent.load(), d.dragNormal.load());
        m_wormSim.params.gradientGain = d.gradientGain.load();
        m_wormSim.params.chemoSteeringGain = d.chemoSteeringGain.load();
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
        // Интерполированные точки между прошлым и текущим физическим кадром
        // - см. m_prevRenderSnap за полным обоснованием (устраняет визуальный
        // "снэп" между шагами сети при рендере быстрее физического тика).
        // Размеры точно совпадают (kNumSegments фиксирован), кроме первого
        // кадра - там prev==curr (см. onUpdate), alpha не важен.
        const auto& curr = m_currRenderSnap;
        const auto& prev = m_prevRenderSnap;
        const int n = static_cast<int>(curr.pointsX.size());
        if (n < 2 || static_cast<int>(prev.pointsX.size()) != n) return;

        const float simDt = std::max(0.0005f, m_wormSim.params.dt.load());
        const float alpha = std::clamp(m_stepAccumulator / simDt, 0.0f, 1.0f);
        std::vector<float> ix(static_cast<std::size_t>(n)), iy(static_cast<std::size_t>(n)),
            iglow(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            const std::size_t si = static_cast<std::size_t>(i);
            ix[si] = prev.pointsX[si] + (curr.pointsX[si] - prev.pointsX[si]) * alpha;
            iy[si] = prev.pointsY[si] + (curr.pointsY[si] - prev.pointsY[si]) * alpha;
            iglow[si] = prev.glow[si] + (curr.glow[si] - prev.glow[si]) * alpha;
        }

        constexpr float kBaseHalfWidth = 6.0f;
        m_bodyVertexData.resize(static_cast<std::size_t>(n) * 2 * 4);
        for (int i = 0; i < n; ++i) {
            glm::vec2 p(ix[static_cast<std::size_t>(i)], iy[static_cast<std::size_t>(i)]);
            glm::vec2 prevP = i > 0 ? glm::vec2(ix[static_cast<std::size_t>(i - 1)], iy[static_cast<std::size_t>(i - 1)])
                                    : p;
            glm::vec2 next = i < n - 1 ? glm::vec2(ix[static_cast<std::size_t>(i + 1)], iy[static_cast<std::size_t>(i + 1)])
                                        : p;
            glm::vec2 tangent = next - prevP;
            float len = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
            glm::vec2 normal = len > 1e-5f ? glm::vec2(-tangent.y / len, tangent.x / len) : glm::vec2(0.0f, 1.0f);

            float t = static_cast<float>(i) / static_cast<float>(n - 1);
            float halfWidth = kBaseHalfWidth * (0.15f + 0.85f * std::sin(3.14159265f * t));
            float glow = iglow[static_cast<std::size_t>(i)];

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
    // Тумблер для целочисленного параметра-флага. Тот же принцип "писать только
    // при реальном редактировании", что и у sliderWithInput ниже: безусловная
    // запись каждый кадр уже один раз молча подменила отгруженную конфигурацию.
    static void checkboxParam(const char* label, std::atomic<int>& value, const char* tooltip = nullptr) {
        ImGui::PushID(label);
        bool v = value.load() != 0;
        if (ImGui::Checkbox(label, &v)) value = v ? 1 : 0;
        if (tooltip && ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(460.0f);
            ImGui::TextUnformatted(tooltip);
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
        ImGui::PopID();
    }

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
        //
        // ЗАПИСЬ ТОЛЬКО ПРИ РЕАЛЬНОМ РЕДАКТИРОВАНИИ (WORM_V5_SPATIAL_ENVELOPE_
        // DIAGNOSIS.md раздел 17). Раньше здесь стояло безусловное
        // value = std::clamp(v, lo, hi) на каждом кадре, и это молча затирало
        // любой дефолт, не попадающий в [lo,hi], стоило секции с ползунком
        // оказаться раскрытой. Два реальных случая в отгруженном состоянии:
        //   bodyGain           - дефолт 1200 при диапазоне 0..5   -> 5.0
        //   chemoSteeringGain  - дефолт -2.0 при диапазоне 0..4   -> 0.0
        // Второй означал, что в живой демке хемотаксис был выключен ПОЛНОСТЬЮ
        // (секция Environment раскрыта по умолчанию), то есть демка показывала
        // конфигурацию, которую никто не измерял, - и владелец проекта
        // справедливо увидел, что червь уходит от еды.
        //
        // Диапазоны обоих ползунков исправлены, но чинить надо было не числа:
        // безусловная запись превращает КАЖДЫЙ будущий выход дефолта за
        // границы в молчаливую подмену поведения. Теперь значение уходит в
        // атомик только если виджет вернул true, то есть пользователь его
        // действительно двигал или вводил.
        bool edited = ImGui::SliderFloat("##s", &v, lo, hi, fmt, ImGuiSliderFlags_AlwaysClamp);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        edited |= ImGui::InputFloat("##i", &v, 0.0f, 0.0f, fmt);
        ImGui::SameLine();
        ImGui::TextUnformatted(label);
        // Явный маркер, если текущее значение вне диапазона ползунка: сам
        // ползунок в таком случае показывает упор в край и выглядит
        // нормально, из-за чего расхождение и осталось незамеченным.
        const float current = value.load();
        if (current < lo || current > hi) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "[= %.4g, вне диапазона ползунка]", current);
        }
        if (tooltip) {
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            ImGui::SetItemTooltip("%s", tooltip);
        }
        if (edited) value = std::clamp(v, lo, hi);
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
            "Color: indigo = low activation, rose = high. Hover a dot to name it.");
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
            ImU32 col = nocturneActivationColor(a);
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

    // Живой график per-joint угла изгиба (WormSim::debugBodyAngles(), 24
    // значения) - добавлено по прямому запросу пользователя после того, как
    // на новых (WORM_V5_REAL_AMPLITUDE_CALIBRATION.md) дефолтах в демке стало
    // видно ТОЛЬКО ДВЕ зоны выраженного изгиба вместо ожидаемой бегущей волны
    // по всему телу - это графическое подтверждение того же "стационарная
    // среднетелая полоса суставов упирается в физический кламп" эффекта, что
    // уже найден численно в WORM_V5_JOINT_CLAMP_RESULTS.md (пик на суставе
    // ~11 из 24). Даёт возможность отслеживать это НЕ перезапуская отдельные
    // CLI-харнессы (tests/worm_v2_measurement's trace/dumpPhase) каждый раз.
    void drawBodyAngles() {
        const auto& angles = m_wormSim.debugBodyAngles();
        if (angles.empty()) return;
        const float clamp = m_wormSim.params.jointAngleClamp.load();

        ImGui::SetNextWindowPos(ImVec2(360, 400), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(320, 260), ImGuiCond_FirstUseEver);
        ImGui::Begin("Body angles");
        ImGui::TextDisabled("(?)");
        ImGui::SetItemTooltip(
            "Per-joint bend angle (WormSim::debugBodyAngles()), head (0) to tail. "
            "Dashed lines mark the physical clamp (+/-%.3f rad). "
            "\"Bend zones\" = local |angle| peaks past 30%% of the clamp - "
            "a direct count of how many places the body is visibly bending right now.",
            clamp);

        // Подсчёт "зон изгиба" - локальные максимумы |angle_i| выше 30% клампа,
        // с минимальным разносом в 2 сустава, чтобы шумовое дрожание одного
        // соседнего значения не считалось отдельной зоной.
        const float zoneThreshold = 0.30f * clamp;
        std::vector<int> zoneJoints;
        float peakAbs = 0.0f;
        int peakIdx = 0;
        for (std::size_t i = 0; i < angles.size(); ++i) {
            const float a = std::fabs(angles[i]);
            if (a > peakAbs) { peakAbs = a; peakIdx = static_cast<int>(i); }
            if (a < zoneThreshold) continue;
            const bool leftOk = (i == 0) || a >= std::fabs(angles[i - 1]);
            const bool rightOk = (i + 1 >= angles.size()) || a >= std::fabs(angles[i + 1]);
            if (!leftOk || !rightOk) continue;
            if (!zoneJoints.empty() && static_cast<int>(i) - zoneJoints.back() <= 2) continue; // тот же горб
            zoneJoints.push_back(static_cast<int>(i));
        }

        ImGui::Text("Bend zones now: %d", static_cast<int>(zoneJoints.size()));
        if (!zoneJoints.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("(joints:");
            for (std::size_t k = 0; k < zoneJoints.size(); ++k) {
                ImGui::SameLine();
                ImGui::TextDisabled("%d%s", zoneJoints[k], (k + 1 < zoneJoints.size()) ? "," : ")");
            }
        }
        ImGui::Text("Peak |angle| = %.4f rad at joint %d (%.0f%% of clamp)",
                    peakAbs, peakIdx, clamp > 1e-6f ? 100.0f * peakAbs / clamp : 0.0f);
        float meanAbs = 0.0f;
        for (float a : angles) meanAbs += std::fabs(a);
        meanAbs /= static_cast<float>(angles.size());
        ImGui::Text("Mean |angle| across 24 joints = %.4f rad", meanAbs);

        std::vector<float> plotData(angles.begin(), angles.end());
        const float plotRange = std::max(clamp * 1.15f, 0.01f);
        ImGui::PlotLines("##angles", plotData.data(), static_cast<int>(plotData.size()), 0,
                          "head -> tail", -plotRange, plotRange, ImVec2(280, 100));
        ImGui::End();
    }
#endif

    WormSim m_wormSim;
    int m_foodToolMode = 0; // 0 = add, 1 = remove
    float m_stepAccumulator = 0.0f; // fixed-timestep accumulator, see onUpdate

    // Render-side interpolation between physics frames (Fiedler "Fix Your
    // Timestep!" pattern) - см. onUpdate/renderBody. До этого renderBody
    // брало snapshot() заново каждый вызов рендера - т.е. буквально
    // последний завершённый физический шаг (20Гц при dt=0.05 по умолчанию),
    // без сглаживания. При render FPS выше физического тика (обычно так) и
    // особенно при timeScale>1 (несколько шагов сети "проглатываются" за
    // кадр - см. предупреждение у слайдера timeScale ниже про "визуально
    // конвульсии") это выглядит как рывками, ступенчатое движение - тот же
    // симптом, что жалоба "рывки" по прямому запросу пользователя, но с
    // РЕНДЕРА, не из самой физики/сети (которая к этому моменту уже
    // отдельно расследована и клампом на u_k - см. body.cpp/hpp). curr -
    // состояние на конец этого кадра (после всех подшагов), prev - на конец
    // прошлого кадра; renderBody линейно интерполирует между ними по доле
    // m_stepAccumulator/simDt, оставшейся после подшагов - вносит не больше
    // одного simDt (по умолчанию 50мс) визуальной задержки, стандартная и
    // незаметная цена за отсутствие "снэпа" между кадрами.
    WormSim::Snapshot m_prevRenderSnap;
    WormSim::Snapshot m_currRenderSnap;
    bool m_haveRenderSnap = false;
#ifdef TESSERA_IMGUI_ENABLED
    ImFont* m_monoFont = nullptr; // loaded by applyNocturneImGuiTheme(), pushed around onImGui()'s content
    // F10 - см. onImGui(). Панели ставятся по умолчанию поверх ЦЕНТРА арены
    // (червь стартует в m_position = центр bounds, WormSim.cpp, а панели
    // "Worm"/"Neurons"/"Body angles" в сумме перекрывают левые ~680px из
    // 1280) - без скрытия червя за ними не видно ни на одном скриншоте/
    // записи, воспроизведено и проверено на этой машине.
    bool m_showDebugUI = true;
#endif
    std::string m_lastSnapshotPath; // for UI feedback after "Snapshot neurons", see snapshotNeuronsToFile

    std::unique_ptr<Shader> m_hexShader;
    unsigned int m_hexVAO = 0, m_hexVBO = 0;
    std::vector<float> m_groundShade; // precomputed base earth jitter, kHexCols*kHexRows

    std::unique_ptr<Shader> m_bodyShader;
    unsigned int m_bodyVAO = 0, m_bodyVBO = 0;
    std::vector<float> m_bodyVertexData;
};

int main() {
    // Разовое сидирование C-рандома для живой демки - раньше это (ошибочно)
    // делал сам конструктор WormSim при КАЖДОМ создании (см. WormSim.cpp за
    // полной историей находки бага) - здесь нужно ровно один раз при старте
    // процесса, до конструктора WormApp (который уже создаёт m_wormSim в
    // своём списке инициализации).
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    try {
        WormApp app;
        app.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[worm] fatal: %s\n", e.what());
        return 1;
    }
    return 0;
}
