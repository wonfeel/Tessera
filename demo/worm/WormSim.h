// demo/worm/WormSim.h
//
// Обёртка над connectome/ под потоковую модель Tessera: step() зовётся с
// update-потока, snapshot() - с render-потока (см. Application::run()).
// Один m_mutex защищает и шаг сети, и чтение состояния - как
// LightField/SpringNetwork в соседних демках.
//
// Среда - непрерывное поле еды ("бактериальный газон", см.
// depositFood/removeFood/m_foodField), не дискретные точки по клику: у
// настоящего C. elegans источник запаха - облако/лужайка бактерий, а не набор
// изолированных точек. Червь его ЕСТ (consumeFood - реально истощает газон
// под собой) и НЮХАЕТ (applyFoodDrive сэмплит поле в точке головы). AWA/AWC
// получают абсолютный уровень запаха; ASEL/ASER - его ПРОИЗВОДНУЮ ПО ВРЕМЕНИ
// (растёт/падает), как у настоящих ASE - это и есть механизм, которым червь
// реально ищет еду (клинокинез/смещённое случайное блуждание через модуляцию
// частоты поворотов), а не наведение на координаты цели. Небольшой
// независимый шум на каждом хемосенсорном нейроне каждый шаг - вечно есть, на
// что реагировать, живое подёргивание/самостоятельный поиск без всякой
// искусственной точки-приманки или автопилота.
//
// Перемещение тела - ЧЕСТНАЯ (хоть и упрощённая) физика: анизотропное трение
// о субстрат (см. connectome/body.hpp - resistive force theory), решаемое из
// формы, которую породила сеть. Здесь больше НЕТ отдельного "курс/скорость из
// среднего deviation" - WormBody сам считает, куда и как быстро сдвигает тело
// его собственная бегущая волна изгиба; WormSim только поворачивает и
// накапливает эту скорость в мировых координатах.
#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "connectome/body.hpp"
#include "connectome/loader.hpp"
#include "engine/core/HexGrid.h"

class WormSim {
public:
    explicit WormSim(const std::string& connectomeDataPath);

    struct Params {
        std::atomic<float> dt{0.05f};
        // Скорость воспроизведения (не влияет на WormSim вообще - читается
        // только в WormApp::onUpdate для fixed-timestep аккумулятора).
        // Раньше единственным способом "ускорить" было тащить dt вверх, а
        // это меняет ЗЕРНИСТОСТЬ интегрирования: при большом dt (например
        // 0.2) экспоненциальный интегратор почти мгновенно допрыгивает до
        // цели за один шаг вместо плавной эволюции, и curvature каждый шаг
        // сразу упирается в кламп WormBody - тело схлопывается в идеальный
        // круг (постоянная кривизна на всю длину). timeScale масштабирует,
        // СКОЛЬКО подшагов делается за кадр, а не размер каждого подшага.
        std::atomic<float> timeScale{1.0f};
        std::atomic<float> chemGain{0.02f};
        std::atomic<float> gapGain{0.02f};
        std::atomic<float> leakScale{1.0f};
        std::atomic<float> activationTheta{0.0f};
        std::atomic<float> activationSlope{1.0f};
        std::atomic<float> bodyGain{2.0f}; // подобрано перебором - лучше видимая волна без потери устойчивости

        // Локомоция: анизотропное трение о субстрат (см. connectome::WormBody).
        // Именно РАЗНИЦА этих двух определяет, во что превращается волна
        // изгиба - в чистое перемещение или в дёргание на месте.
        std::atomic<float> dragTangent{1.0f};  // вдоль тела
        // Было 6.0 - пересчитано после находки, что метрика "net displacement",
        // которой предыдущие раунды этой сессии (tests/worm_chemotaxis_
        // calibration's "displacement" mode) мерили эффективность ползания,
        // отслеживает points_[0]/m_position - ОДИН КОНЕЦ цепочки сегментов,
        // а не центроид тела. При слабой анизотропии (c_n/c_t близко к 1) RFT
        // предсказывает, что центроид тела вообще не может сдвинуться -
        // одинаковый скалярный drag на каждом сегменте => нулевая сумма сил
        // требует нулевой скорости центроида в КАЖДЫЙ момент (тот же довод,
        // что сохранение импульса), но конец цепочки всё равно может
        // размашисто качаться вокруг неподвижного центроида просто от смены
        // формы. Это и происходило: при ratio=1 старая метрика показывала
        // ЛУЧШУЮ "эффективность" (0.52 против 0.43 у прежнего дефолта),
        // хотя настоящий центроид сдвигался всего на ~2.6 из 121 "units" —
        // почти весь сигнал был паразитным recoil, не перемещением.
        // Замер по центроиду (среднее всех 25 точек тела) вместо points_[0]
        // (см. scratchpad-харнесс этого расследования) показал: РЕАЛЬНОЕ
        // расстояние ползания растёт монотонно с c_n/c_t от 1 минимум до 100,
        // без всякой цены по здоровью (coiled ratio 0.7555-0.7559, 0 заморозок
        // на любом проверенном ratio, 24-48 seeds, независимые seed base).
        // 10.0 (Fang-Yen et al. 2010, Cn~222/Ct~22.1 => ~10.05) было первым
        // шагом - по прямому запросу "пусть двигается быстрее" поднято до
        // 40.0, верхней границы АНИЗОТРОПИИ, встречающейся в той же
        // литературе (не искусственный максимум - конкретно измеренное
        // отношение, не "выкрутить, пока не сломается"). Перепроверено на
        // центроидной метрике (см. tests/worm_chemotaxis_calibration's
        // displacement mode, теперь принимает dragTangent/dragNormal
        // аргументами): 10.0 -> 40.0 даёт настоящее (центроидное) чистое
        // перемещение 54.2 -> 81.9 units за 800 шагов/40с без еды (+51%,
        // 24 независимых seeds), эффективность чуть ВЫШЕ (0.78 -> 0.80,
        // не ниже), 0/24 заморозок на обоих ratio. tests/worm_locomotion
        // по-прежнему PASS. Дальше (до ~100x, тоже без вреда здоровью по
        // той же серии измерений) продолжает расти, но там кончается
        // прямое биологическое обоснование - 40x выбран как граница,
        // подтверждённая цитатой, а не открытый эксперимент с числом.
        std::atomic<float> dragNormal{40.0f};   // поперёк тела

        // Хемотаксис.
        std::atomic<float> gradientGain{4.0f};      // d(запах)/dt -> ASEL/ASER (клинокинез)
        std::atomic<float> spontaneousNoise{3.0f};  // независимый шум на хемосенсорах каждый шаг
        // Независимый шум на КАЖДОМ не-выходном нейроне (не только
        // сенсорных) - реальные нейроны все имеют собственный канальный шум/
        // спонтанный выброс медиатора, не только хемосенсорные. Без этого при
        // достаточно сильной gap junction связи сеть может синхронизироваться
        // в неподвижную точку, до которой шум всего 4-6 сенсорных клеток не
        // всегда доходит через сеть - сеть "не реагирует ни на что".
        std::atomic<float> intrinsicNoise{0.4f};

        // Проприоцепция (растяжение-рецепторная обратная связь вдоль тела) -
        // реальный механизм, которым C. elegans превращает связность
        // коннектома в согласованную волну (Boyle, Berri & Cohen 2012,
        // PMC3296079): каждый B-типа мотонейрон "интегрирует ток
        // stretch-рецепторов локально и постериально вдоль своего аксона".
        // Моделируем это как усреднение РЕАЛЬНОГО (после физики/клампа
        // WormBody, см. applyProprioception) угла изгиба на своей позиции и
        // proprioceptiveOffset позициях к хвосту от неё, с тем же знаком,
        // что curvature (D минус V) - упрощение их асимметричной GABA-
        // эргической кросс-ингибиции между сторонами, но та же суть. Первая
        // версия кормила обратную связь сырым, ДО тела, нейронным сигналом
        // без физического ограничения - каскадно разгонялась к хвосту.
        // Теперь источник - величина, УЖЕ прошедшая кламп тела, поэтому
        // физически не может разогнаться сама по себе. Направление окна "к
        // хвосту" (не "к голове") - НЕ просто выведено из цитаты выше,
        // проверено эмпирически (80 независимых прогонов) против гипотезы,
        // что оно даёт ход задом наперёд - см. applyProprioception в
        // WormSim.cpp за полной историей и числами.
        std::atomic<float> proprioceptiveGain{4.0f};    // сила обратной связи - 0 = выключено
        std::atomic<float> proprioceptiveOffset{4.0f};  // ширина окна "локально и постериально", в позициях

        // Механосенсация (DVA) - настоящий stretch-рецепторный нейрон
        // C. elegans (Li, Feng & Xu 2006, PMC1500850: "DVA... functions as a
        // mechanosensory neuron... required for the regulation of the
        // amplitude of the locomotory waveform"), физиологически отдельный
        // от проприоцептивного контура B-класса выше - тот пропагирует
        // ВОЛНУ вдоль тела, DVA же ощущает МЕХАНИЧЕСКУЮ НАГРУЗКУ целиком и
        // модулирует амплитуду через премоторные интернейроны (AVA/AVB,
        // NLP-12/глутамат). Вход - честная суммарная сила трения среды,
        // реально посчитанная в WormBody::solve_propulsion на этом шаге (см.
        // applyMechanosensation) - не параметр среды напрямую, а то, с чем
        // тело физически борется прямо сейчас. 0.0 = выключено (см.
        // WormSim.cpp конструктор за тем, откалибровано это уже или нет).
        std::atomic<float> mechanoGain{0.0f};

        // Локальная механосенсация: та же физическая величина, что питает
        // DVA выше (|сила трения| на решённой скорости), но по СЕГМЕНТАМ
        // (WormBody::segment_load()), заведённая в applyProprioception на ТЕ
        // ЖЕ мотонейроны и с тем же окном, что и растяжение-рецепторный
        // сигнал. Обоснование разделения с mechanoGain: DVA - один
        // премоторный интернейрон, физиологически не способный различать,
        // КАКОЙ участок тела нагружен, только общую сумму (см. mechanoGain
        // выше и tests/worm_mechanosensation_calibration - там честно
        // показано, что этого недостаточно, чтобы плавание обогнало
        // ползание: одна глобальная скалярная ручка не может по-разному
        // подействовать на локальную частоту волны). Настоящие B/D-
        // мотонейроны сами являются stretch-рецепторами (Wen et al. 2012
        // Neuron; Yeon, Chen et al. 2018 Cell - "Nervous system-wide
        // functional analysis..." показывает механочувствительность,
        // распределённую по мотонейронам, а не сосредоточенную в одном
        // премоторном интерneurone) - этот канал даёт каждому мотонейрону
        // его СОБСТВЕННОЕ ощущение локального сопротивления, а не одну
        // цифру на весь организм. Нормировка на dragNormal - та же
        // Weber-Fechner-перенормировка, что и у mechanoGain (см. там), по
        // той же причине (агар/вода отличаются на порядок по сырой
        // нагрузке). 0.0 = выключено, не откалибровано - см.
        // tests/worm_mechanosensation_calibration за историей поиска.
        std::atomic<float> localMechanoGain{0.0f};

        // Активный (регенеративный) ток B-класса мотонейронов (DB/VB) -
        // качественно другая ось от всего выше: не масштабирование
        // существующей линейной системы (leak/capacitance/веса) и не
        // дополнительный сенсорный вход, а САМОссылочная положительная
        // обратная связь (Network::set_active_current, network.hpp) -
        // минимальная редукция кальциевого plateau-механизма (UNC-2/CaV2),
        // которым, по Gao, Guan, Fouad et al. 2018 (eLife 7:e29915) и Fouad
        // et al. 2018 (eLife 7:e29913), настоящие B-мотонейроны сами
        // являются автономными осцилляторами, задающими ритм переднего
        // хода. Честная оговорка (см. Wen, Gao & Zhen 2018, Phil. Trans. R.
        // Soc. B 373:20170370): что именно UNC-2 стоит за осцилляцией
        // B-класса - перенос по аналогии с A-класса (для A доказано
        // напрямую электрофизиологией, для B - открытый вопрос в самой
        // литературе). bClassOscillatorTauW - постоянная времени
        // медленной (восстановление/инактивация) переменной гейта - реально
        // задающий частоту параметр, нижняя граница диапазона поиска
        // производна (dt/tau_w должно оставаться << 1 для настоящего
        // разделения быстрой/медленной шкалы, иначе гейт становится
        // алгебраическим и осциллятор вырождается), верхняя - от измеренного
        // Gao et al. 2018 периода изолированного A-мотонейрона (~50-90с) как
        // ориентир, не точное значение для B. 0.0 = выключено тождественно
        // (см. Network::step - умножение на active_gain_=0 даёт 0 для
        // любого нейрона на любом шаге) - см.
        // tests/worm_bclass_oscillator_calibration за статусом калибровки.
        std::atomic<float> bClassOscillatorGain{0.0f};
        std::atomic<float> bClassOscillatorTauW{4.0f};

        // WormBody::pose_decay_rate_ (см. body.hpp::set_pose_decay_rate) -
        // вынесенный наружу хардкод 0.5, ИСКЛЮЧИТЕЛЬНО для совместного
        // поиска с bClassOscillatorGain/TauW (см. tests/worm_bclass_body_
        // joint_calibration) - сама по себе эта константа не про
        // осциллятор, а про то, как быстро изгиб забывается без привода;
        // тюнить её в одиночку без активного тока уже пробовали как часть
        // другого эксперимента (network.cpp комментарий) с плохим исходом.
        // Дефолт 0.5 - тот же самый хардкод, что был раньше, побитово то же
        // поведение, если не трогать.
        std::atomic<float> bodyPoseDecayRate{0.5f};

        // Термотаксис (AFD - настоящий термосенсорный нейрон C. elegans,
        // Mori & Ohshima 1995: первый синапс AFD у обоих найденных вариантов
        // коннектома идёт именно на AIY - AFDL->AIYL вес 25, AFDR->AIYR вес
        // 29 в этом файле, того же порядка, что ASE/AWC->AIY - то есть контур
        // навигации уже "бесплатно" общий с хемотаксисом, новых интернейронов
        // не требуется). Статичный линейный градиент вдоль мировой оси X -
        // тот же дизайн, что классический thermal-gradient assay (Hedgecock &
        // Russell 1975), не дискретное поле, как у еды: температуру не едят и
        // не красят кистью во время игры, это гладкая заданная величина.
        std::atomic<float> tempBaseline{20.0f};       // температура при worldPos == boundsMin, условные градусы
        // 0.0 по умолчанию - как и еда, ничего не делает, пока не включено
        // явно (слайдером/пресетом): иначе это была бы постоянная фоновая
        // тяга к одному углу поля в самом обычном демо, а не открываемое
        // поведение.
        std::atomic<float> tempGradientSlope{0.0f};   // градусов на мировую единицу вдоль направления ниже
        std::atomic<float> tempGradientAngle{0.0f};   // радианы - направление РОСТА температуры (0 = вдоль +X)
        // T_c - "запомненная" (упрощение реального thermal imprinting -
        // AFD/AIY у настоящего червя меняют синаптические свойства при
        // культивации при одной температуре) комфортная температура. AFD в
        // реальности инвертирует знак своего ответа на потепление в
        // зависимости от того, ниже или выше T_c текущая температура (Clark
        // et al. 2006, Kimura et al. 2004) - это и есть механизм, из-за
        // которого настоящий термотаксис вообще сходится к T_c, а не просто
        // "теплее/холоднее = всегда хорошо", как у еды. applyTemperatureDrive
        // моделирует это прямо в самом входе AFD (свойство сенсора, не
        // прикладное решение), тем же принципом, что асимметрия ASEL/ASER.
        std::atomic<float> cultivationTemp{20.0f};
        // ОТКАЛИБРОВАНО (tests/worm_thermotaxis_calibration, distribution
        // mode, 40 независимых seed-баз x 8 сидов = 320 честных парных
        // измерений): population mean effect = 0.2646 +/- 0.0724 (~3.65
        // сигма выше нуля) - здоровый (0 деградаций походки на всём
        // проверенном диапазоне gain, в отличие от mechanoGain), устойчивый
        // по знаку на трёх разных величинах gain (-30000/-50000/-100000, все
        // положительные на тех же 16 базах) - см. тест за полной историей
        // поиска. -50000 - середина проверенного диапазона.
        std::atomic<float> thermoGain{-50000.0f};  // AFD <- знак(T_c-T)*dT/dt*gain

        // Roaming/dwelling (ADF/NSM - оба реально серотонинергические у
        // C. elegans). Flavell et al. 2013, Cell: серотонин способствует
        // dwelling (медленный ход, частые повороты - обычно на еде/рядом),
        // нейропептид PDF-1 - противоположному roaming (быстрый, прямой ход).
        //
        // ПЕРВАЯ ВЕРСИЯ (см. tests/worm_roaming_dwelling_calibration за
        // полной историей) кормила ADF/NSM гладкой EMA scent - реального
        // сытого "тонуса". Нашла реальный, статистически надёжный эффект, но
        // ни один режим не был одновременно здоровым, верно направленным и
        // заметным по величине. ВТОРАЯ ВЕРСИЯ (эта): настоящие ADF/NSM
        // физиологически реагируют не на усреднённую концентрацию, а на
        // ДИСКРЕТНЫЕ акты глотания через фарингеальную помпу - гладкий вход
        // был нечестным упрощением именно там, где, похоже, и была причина
        // нестабильности (см. applySerotoninDrive).
        //
        // Частота откачки помпы - НЕ свободный параметр, а измеренный факт
        // (Avery & Horvitz 1990; Raizen, Lee & Avery 1995): ~250-260 качков/
        // мин на еде - 4.3 Гц, середина этого диапазона.
        std::atomic<float> pharyngealPumpRateHz{4.3f};
        std::atomic<float> serotoninGain{0.0f};  // ADF/NSM <- gain импульсом при каждом качке. 0 = выключено

        // PDF-1/PDFR-1 - другая половина того же контура (см. комментарий
        // выше про Flavell et al. 2013): нейропептид, способствующий
        // ПРОТИВОПОЛОЖНОМУ serotoninGain'у roaming (быстрый, прямой ход,
        // меньше разворотов). В отличие от serotoninGain (внешний вход,
        // вручную добавленный в applySerotoninDrive), здесь ничего не
        // подмешивается извне - связность настоящая (Ripoll-Sánchez,
        // Watteyne et al. 2023, Neuron - предсказанная по ко-экспрессии
        // GPCR/лиганда сеть, mid-range модель), грузится прямо из файла
        // коннектома (см. PEPTIDE_EDGES в loader.cpp) и работает через
        // собственную текущую активность настоящих нейронов-источников
        // (в основном AIY - см. Network::set_peptide_connectivity/
        // set_peptide_gain). 0 = выключено тождественно, как и
        // bClassOscillatorGain. См. tests/worm_pdf1_calibration за статусом
        // калибровки.
        std::atomic<float> pdf1Gain{0.0f};
        std::atomic<float> pdf1ReleaseTau{20.0f};  // Flavell et al. 2013: roaming/dwelling эпизоды длятся "минуты"

        // Среда: непрерывное поле еды ("бактериальный газон").
        std::atomic<float> foodDepositRadius{70.0f};     // радиус кисти, мировые единицы
        std::atomic<float> foodDepositAmount{60.0f};      // концентрация/сек при рисовании
        // Раньше было 100.0 - при полном насыщении поля AWA/AWC (получают
        // scent напрямую, без всякого гейна) улетали до V~70-79, что для
        // sigmoid(V) при theta=0/slope=1 - ПОЛНОЕ насыщение (производная
        // ровно 0.0, не просто малая) - сигнал о еде физически не мог дальше
        // передаться, независимо от любых других гейнов. Подтверждено
        // отдельным диагностическим прогоном (see tests/worm_saturation_probe
        // findings): рабочий/чувствительный диапазон сигмоиды - примерно
        // V в [-4,+4]. 6.0 держит пик AWA/AWC в этом диапазоне, оставляя
        // некоторый запас над амплитудой спонтанного шума (spontaneousNoise
        // по умолчанию 3.0), не сжимая сигнал ниже шума.
        std::atomic<float> foodMaxConcentration{6.0f};  // потолок на клетку поля
        std::atomic<float> foodConsumptionRate{6.0f};     // насколько быстро червь съедает под собой, ед/сек
        std::atomic<float> foodDiffusionRate{0.15f};      // 0..1 - скорость расползания газона
    };
    Params params;

    static constexpr int kNumSegments = 24; // = число мышечных позиций (1..24) вдоль тела

    // Поле еды - НАСТОЯЩАЯ гекс-решётка (1 клетка = 1 гекс поля рендера, та
    // же геометрия, что и HexGrid::worldPos), не приближение прямоугольной
    // сеткой - у гексов через ряд смещены на пол-клетки, и без учёта этого
    // покраска/нюх промахивались мимо клетки, которая реально светится на
    // экране. boundsMax считается сам, по той же формуле, что и рендер.
    void setBounds(glm::vec2 worldMin, int fieldCols, int fieldRows, float hexSpacing);

    // Кисть добавления/стирания еды в мировых координатах; dtSeconds - сколько
    // времени кисть действовала в этом кадре (для покраски перетаскиванием -
    // вызывать каждый кадр, пока зажата кнопка, а не один раз на клик).
    void depositFood(glm::vec2 worldPos, float dtSeconds);
    void removeFood(glm::vec2 worldPos, float dtSeconds);
    void clearFood();
    float totalFood() const; // сумма концентрации по всему полю - для UI
    std::vector<float> foodFieldSnapshot() const; // fieldCols*fieldRows, row-major
    int foodFieldCols() const { return m_fieldCols; }
    int foodFieldRows() const { return m_fieldRows; }

    void step(); // вызывать из update-потока (onUpdate); использует params.dt

    // Диагностика: сырой (со знаком) сигнал per-position, который реально
    // ушёл в тело на последнем шаге (curvature - baseline, см. step()) -
    // Snapshot::glow даёт только |tanh| от этого же сигнала, без знака.
    const std::vector<float>& lastCurvatureDeviation() const { return m_lastCurvature; }

    // Имена узлов сети, индексация совпадает с Snapshot::nodeStates - для
    // дампа/снимка всех нейронов по имени (см. WormApp::onImGui).
    const std::vector<std::string>& neuronNames() const { return m_loaded.names; }

    // Доступ к сети для калибровки по классам нейронов (research/tuning путь,
    // не часть обычного игрового API) - см. connectome::Network::
    // scale_type_params и заключение в tests/worm_locomotion про
    // "per-neuron-class calibration" как один из оставшихся честных путей к
    // направленному хемотаксису. Вызывать сразу после конструктора, до
    // первого step().
    connectome::Network& network() { return m_loaded.network; }

    struct Snapshot {
        std::vector<float> pointsX, pointsY; // центральная линия тела в МИРОВЫХ координатах
        std::vector<float> glow;             // |кривизна| в этой точке, сглажено в [0,1)
        std::vector<float> nodeStates;        // состояние ВСЕХ узлов сети, для графа нейронов
        int nodeCount = 0;
    };
    void snapshot(Snapshot& out) const; // вызывать из render-потока (onRender)

    // Раскладка узлов для отрисовки графа - считается один раз в конструкторе
    // (не меняется), поэтому не часть Snapshot. x/y в [0,1] - канвас
    // масштабирует сам. Индексация совпадает с nodeStates.
    const std::vector<float>& nodeLayoutX() const { return m_nodeLayoutX; }
    const std::vector<float>& nodeLayoutY() const { return m_nodeLayoutY; }

private:
    // Мировая точка -> ближайшая клетка гекс-поля (col,row), НАСТОЯЩАЯ
    // геометрия (HexGrid::worldPos), не прямоугольное приближение - иначе на
    // нечётных рядах промах на пол-клетки (см. setBounds).
    void worldToHexCell(glm::vec2 worldPos, int& outCol, int& outRow) const;
    float sampleFood(glm::vec2 worldPos) const;                     // без лока - вызывающий уже держит m_mutex
    void depositAt(glm::vec2 worldPos, float amount, float radiusWorld); // тоже без лока
    float applyFoodDrive();  // сэмплит поле в точке головы, кормит AWA/AWC/ASE, возвращает scent
    void applyIntrinsicNoise(); // независимый шум на всех остальных не-выходных нейронах (кроме моторных - см. applyProprioception)
    void applyProprioception(const std::vector<float>& curvature); // растяжение-рецепторная обратная связь -> бегущая волна
    void applyMechanosensation(); // DVA <- реальная нагрузка трения от WormBody (см. Params::mechanoGain)
    float applyTemperatureDrive(); // сэмплит температуру в точке головы, кормит AFD, возвращает temp
    float sampleTemperature(glm::vec2 worldPos) const; // статичный линейный градиент, см. Params
    void applySerotoninDrive(); // ADF/NSM <- медленная EMA scent (roaming/dwelling), см. Params::serotoninGain
    void containBody(); // держит ВСЮ длину тела в границах поля, с отскоком курса от стены
    void consumeFood(float dt);
    void diffuseFood(float dt);

    connectome::LoadedConnectome m_loaded;
    std::vector<connectome::NeuronId> m_generalChemoIds; // AWAL/AWAR/AWCL/AWCR - абсолютный уровень запаха
    static constexpr connectome::NeuronId kInvalidId = static_cast<connectome::NeuronId>(-1);
    connectome::NeuronId m_aseL = kInvalidId, m_aseR = kInvalidId; // производная по времени (клинокинез)
    connectome::NeuronId m_dva = kInvalidId; // stretch-рецептор, ощущаемая механическая нагрузка (см. applyMechanosensation)
    connectome::NeuronId m_afdL = kInvalidId, m_afdR = kInvalidId; // термосенсор (см. applyTemperatureDrive)
    float m_prevTemp = 0.0f; // для d(T)/dt
    connectome::NeuronId m_adfL = kInvalidId, m_adfR = kInvalidId; // серотонинергические (см. applySerotoninDrive)
    connectome::NeuronId m_nsmL = kInvalidId, m_nsmR = kInvalidId; // серотонинергические (см. applySerotoninDrive)
    float m_pumpPhase = 0.0f; // фаза фарингеальной помпы в [0,1) - см. applySerotoninDrive
    std::array<std::vector<connectome::NeuronId>, 25> m_dorsalByPos, m_ventralByPos;
    connectome::WormBody m_body;

    // Моторные нейроны вентрального тяжа (DA/DB/DD - дорсальные; VA/VB/VD -
    // вентральные) с позицией вдоль тела, приближённой по индексу внутри
    // класса (класс и его реальное анатомическое число нейронов известны и
    // фиксированы - не подгонка, а факт: DA1-9, DB1-7, DD1-6, VA1-12,
    // VB1-11, VD1-13). Используется только для проприоцептивной обратной
    // связи (applyProprioception), не для чтения кривизны тела - за это
    // по-прежнему отвечают m_dorsalByPos/m_ventralByPos (мышцы).
    struct MotorNeuron { connectome::NeuronId id; int pos; bool dorsal; };
    std::vector<MotorNeuron> m_motorNeurons;
    std::vector<bool> m_isMotorNeuron; // индекс = NeuronId, для быстрого исключения в applyIntrinsicNoise
    std::vector<connectome::NeuronId> m_classBMotorNeurons; // DB1-7/VB1-11 - см. Params::bClassOscillatorGain

    std::vector<float> m_nodeLayoutX, m_nodeLayoutY;

    mutable std::mutex m_mutex;
    std::vector<float> m_lastCurvature;
    // Медленно отслеживаемое "среднее" сырой кривизны по каждому сегменту -
    // на некалиброванных весах сеть почти всегда оседает в ПОСТОЯННОМ (не
    // колеблющемся) перекосе, а не в бегущей волне; без вычитания этой
    // базовой линии тело фиксируется в одном изгибе навсегда, а не дёргается.
    // На тело и на пропульсию идёт только deviation (быстрая, "дёрганая"
    // часть), сама базовая линия - никуда, только для вычитания.
    std::vector<float> m_curvatureBaseline;

    std::vector<float> m_foodField; // fieldCols*fieldRows, row-major, [0, foodMaxConcentration]
    int m_fieldCols = 1, m_fieldRows = 1;
    float m_hexSpacing = 36.0f; // см. HexGrid::worldPos - задаётся в setBounds

    float m_prevScent = 0.0f; // для d(scent)/dt -> ASEL/ASER
    glm::vec2 m_position{0.0f};
    float m_heading = 0.0f;
    glm::vec2 m_boundsMin{0.0f}, m_boundsMax{1000.0f};
};
