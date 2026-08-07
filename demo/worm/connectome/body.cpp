#include "body.hpp"

#include <algorithm>
#include <cmath>

namespace connectome {

WormBody::WormBody(int num_segments, float segment_length, float drag_tangent, float drag_normal)
    : segment_length_(segment_length),
      drag_tangent_(drag_tangent),
      drag_normal_(drag_normal),
      angles_(static_cast<std::size_t>(num_segments), 0.0f),
      frame_heading_(static_cast<std::size_t>(num_segments), 0.0f),
      penetration_depth_(static_cast<std::size_t>(num_segments), 0.0f) {
    rebuild_points(0.0f);
}

void WormBody::step(const std::vector<float>& curvature, float dt) {
    const std::vector<float> old_x = points_x_;
    const std::vector<float> old_y = points_y_;

    // Память проседания в субстрат - см. set_drag_settle. Обновляется ПЕРВОЙ
    // вещью в step(), пока segment_load_ ещё хранит значения ПРОШЛОГО шага
    // (перезаписывается только в конце solve_propulsion ниже) - см.
    // WORM_V2_DESIGN.md раздел 4.4 за тем, почему это сохраняет систему
    // линейной (penetration_depth_ этого шага уже известен к моменту решения
    // баланса сил этого же шага, не зависит от искомых (Vx,Vy,w)).
    update_penetration_depth(dt);

    // ПРОБОВАЛИ И ОТКАТИЛИ (см. tests/worm_speed_calibration): резистить
    // скорость изгиба drag_normal_ (по аналогии с тем, как поступательное
    // движение резистится в solve_propulsion) - и линейным множителем на
    // сам привод, и с тем же множителем на затухание (перерасчёт времени
    // механики целиком). Идея: у настоящего червя рост частоты изгиба в
    // ~3-4 раза при переходе в воду компенсирует более слабую анизотропию и
    // делает плавание быстрее ползания; здесь этого нет, потому что curvature
    // -> angles сейчас чисто кинематический (drag не участвует). ОБА варианта
    // на воде (drag_normal=1.7 против дефолтных 40) не ускоряли ход, а
    // ломали его: freq зануляется (0.000 Гц вместо ~0.008), тело замирает в
    // одной статичной дуге вместо колебаний - то же вырожденное состояние,
    // что и при отключении spatial mean-subtract в WormSim.cpp. Похоже, эта
    // сеть удерживает здоровую колеблющуюся походку только в узком диапазоне
    // соотношения "привод/собственная временная константа сети", и любое
    // прямое изменение механической части шага изгиба выбивает её за этот
    // диапазон - тот же паттерн, что и в других "tried and reverted" по этому
    // проекту (leak/capacitance калибровка, reversal-механизм и т.д.).
    // Честный вывод: связка среда->частота изгиба здесь не тривиальна и
    // требует полноценного калибровочного поиска (как tests/worm_speed_
    // calibration уже делает для leak/capacitance) с проверкой на множестве
    // независимых seed-баз, а не точечной формулы - один заход её не решает.
    //
    // ВОЗВРАЩЕНО И РАБОТАЕТ (см. set_bend_time_scale выше и Params::
    // mediumBendCouplingKappa) - но в другой форме, и прежний провал теперь
    // объясним. Тогда множитель не НОРМИРОВАЛСЯ на трение агара, поэтому агар
    // (drag=40) сам замедлялся в десятки раз - отсюда "freq зануляется, тело
    // замирает в одной статичной дуге". И тогда не было понимания, что
    // отношение привод/затухание задаёт амплитуду, а само затухание - темп
    // (WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md раздел 14), из-за чего множитель
    // навешивался не на все члены угловой динамики сразу. Сейчас нормировка
    // делает агар побитово нетронутым, а множитель применяется ко всем членам,
    // то есть это масштабирование ВРЕМЕНИ подсистемы, а не подкрутка одного
    // слагаемого. Догадка про "узкий диапазон соотношения привод/собственная
    // временная константа" оказалась верной по сути - просто это соотношение
    // теперь измерено и сохраняется по построению.
    // Снимок angles_ СО СТАРТА этого шага - изгибная жёсткость (см.
    // set_bend_stiffness) читает СОСЕДЕЙ i-1/i+1 через этот снимок, а не
    // "по ходу" цикла ниже, иначе связь стала бы несимметричной (зависящей
    // от порядка обхода i=0..n-1, т.к. anglesPrev[i-1] уже был бы обновлён к
    // моменту, когда сосед i его читает) - см. WORM_V3_DESIGN.md раздел 2.2.
    const std::vector<float> anglesPrev = angles_;

    // ВРЕМЯ КИНЕМАТИКИ УГЛОВ отличается от времени остального шага - см.
    // set_bend_time_scale в body.hpp. Множитель применяется КО ВСЕМ членам
    // угловой динамики сразу (привод, изгибная жёсткость, затухание позы,
    // кубическое сопротивление), то есть это честное масштабирование времени
    // для этой подсистемы, а не подкрутка одного слагаемого: отношение
    // привод/затухание сохраняется, значит амплитуда не меняется, меняется
    // только темп. Ровно тот вывод, который в WORM_V5_SPATIAL_ENVELOPE_
    // DIAGNOSIS.md разделе 14 позволил поднять частоту, не потеряв амплитуду.
    //
    // ВАЖНО: rebuild_points и solve_propulsion ниже получают НАСТОЯЩИЙ dt.
    // Скорость изменения формы (u_k в solve_propulsion считается как конечная
    // разность, делённая на dt) от этого растёт автоматически - что физически и
    // должно происходить: быстрее изгиб - больше тяги.
    const float bendDt = dt * bend_time_scale_;

    for (std::size_t i = 0; i < angles_.size(); ++i) {
        const float c = i < curvature.size() ? curvature[i] : 0.0f;
        if (muscle_target_tau_ > 0.0f) {
            // МЫШЦА КАК ИСТОЧНИК МОМЕНТА - см. set_muscle_target_tau в
            // body.hpp за выводом и цитатой (Boyle, Berri & Cohen 2012).
            // Активация задаёт ЦЕЛЕВОЙ угол, сустав идёт к нему с постоянной
            // времени мышцы. Экспоненциальная форма, а не явный Эйлер: тот же
            // урок, что с затуханием позы и кубическим членом - явная схема
            // ломается, как только шаг сравним с постоянной времени, а здесь
            // tau=0.1с при dt=0.05с, то есть ровно тот случай.
            //
            // Затухание позы (pose_decay_rate_) в этом режиме НЕ применяется:
            // его роль - возврат к нейтрали, а здесь эту роль играет сама цель
            // (при нулевой активации цель равна нулю). Два механизма возврата
            // одновременно означали бы двойной учёт.
            const float target = c;
            const float alpha = 1.0f - std::exp(-bendDt / muscle_target_tau_);
            angles_[i] += (target - angles_[i]) * alpha;
        } else {
            angles_[i] += c * bendDt;
        }
        // Изгибная жёсткость (см. set_bend_stiffness) - дискретный лапласиан
        // вдоль цепочки суставов, free-free граничные условия (зеркальный
        // фиктивный узел на концах: ghost[-1]:=anglesPrev[0],
        // ghost[n]:=anglesPrev[n-1] - нулевой градиент кривизны на свободном
        // конце эластичного стержня). Вставлено ДО decay/clamp ниже, чтобы
        // жёсткость участвовала в той же физической стадии, что и приводной
        // сигнал сети, а не была "поверх" уже заклампленного результата.
        {
            const float left = (i > 0) ? anglesPrev[i - 1] : anglesPrev[i];
            const float right = (i + 1 < anglesPrev.size()) ? anglesPrev[i + 1] : anglesPrev[i];
            const float bend = left - 2.0f * anglesPrev[i] + right;
            angles_[i] += bend_stiffness_ * bend * bendDt;
        }
        // Затухание к нейтральной позе (см. set_pose_decay_rate). ТОЧНАЯ
        // экспоненциальная форма вместо прежней (1 - r*dt).
        //
        // Явный Эйлер здесь ломался при масштабировании времени изгиба (см.
        // set_bend_time_scale): в воде bendDt в 3.97 раза больше, и произведение
        // pose_decay_rate_*bendDt = 3*0.198 = 0.595, то есть за один шаг
        // "утекало" 60% угла. Амплитуда в воде схлопывалась до 0.035 рад против
        // 0.25 на агаре - и это была ЧИСТО ошибка дискретизации: у точного
        // решения линейной утечки установившаяся амплитуда равна c/r и от
        // масштаба времени не зависит вообще.
        //
        // exp(-r*dt) устойчив при любом шаге и не требует min(1.0, ...) -
        // прежняя заглушка существовала именно чтобы (1 - r*dt) не стало
        // отрицательным. На агаре разница мала но не нулевая: exp(-0.15)=0.8607
        // против 1-0.15=0.85, то есть эффективное затухание немного слабее -
        // проверено измерением, не предположением (см. WORM_V5_SPATIAL_
        // ENVELOPE_DIAGNOSIS.md раздел 19).
        // В режиме мышцы-как-момента возврат к нейтрали обеспечивает сама цель
        // (см. выше), поэтому отдельное затухание позы не применяется.
        if (muscle_target_tau_ <= 0.0f) angles_[i] *= std::exp(-pose_decay_rate_ * bendDt);
        // WORM_V5_SOFT_JOINT_RESISTANCE_RESULTS.md mechanism 2 (nonlinear
        // stiffening resistance, see set_joint_stiffening_gain in body.hpp) -
        // a Duffing-oscillator-style cubic softening spring, added to the
        // accumulation BEFORE any clamp/saturation below (per that report's
        // mandate: "add a cubic restoring term to the accumulation BEFORE
        // any clamp"). Negligible near zero (cubic), growing rapidly as
        // |angle| grows - the NATURAL equilibrium is wherever this
        // resistance balances the network's own drive, not an externally
        // imposed ceiling. k_soft<=0 (default) leaves this branch untaken -
        // contributes exactly 0, bitwise-identical to not having this term.
        if (joint_stiffening_gain_ > 0.0f) {
            const float a = angles_[i];
            // ПОЛУНЕЯВНАЯ форма вместо явного Эйлера. Явный вариант
            // (a -= k*a^3*dt) неустойчив, когда k*a^2*dt подходит к 1: шаг
            // перелетает через ноль и амплитуда начинает раскачиваться вместо
            // того чтобы ограничиваться. На агаре при k=8 и a=0.54 это
            // произведение ещё 0.12, но в воде bendDt в ~4 раза больше (см.
            // set_bend_time_scale), и при a~0.6-1.0 оно доходит до 0.6-1.6.
            // Измерено прямо: явная форма при k=8 давала в воде survival 10/16
            // и выбросы мгновенной скорости до 20x против 3.5x - то есть
            // разваливалась ровно так, как разваливается явная схема.
            //
            // a/(1 + k*a^2*dt) - тот же член с a^2, взятым на старом шаге:
            // безусловно устойчиво при любом dt, знак сохраняется, |угол|
            // монотонно убывает, а для малых k*a^2*dt совпадает с явной формой
            // с точностью до второго порядка. Тот же по сути урок, что и с
            // затуханием позы выше.
            angles_[i] = a / (1.0f + joint_stiffening_gain_ * a * a * bendDt);
        }
        // Кламп на сегмент: слишком широкий (было ±1.2 рад/~69°) позволял
        // телу закрутиться на несколько полных оборотов при кривизне одного
        // знака на большинстве сегментов сразу (не редкость в gap-связанной
        // сети) - выглядело как крошечный узел вместо червя. Слишком узкий
        // (~0.18) даёт вытянутое тело, но почти убивает поворотливость (был
        // проверен headless-перебором - roam схлопывался почти в одну ось).
        // ±0.25 - компромисс, подобранный тем же перебором: тело остаётся
        // явно вытянутым (bbox-диагональ/длина дуги ~0.5-0.6, не ~0.2-0.3),
        // сохраняя реальную двумерную манёвренность. НАЗВАН "анатомическим
        // пределом" в этом комментарии исторически, но честно: он НИКОГДА
        // не калибровался против реального угла изгиба сустава - только
        // против elongation-ratio/manoeuvrability, и очень вероятно на БОЛЕЕ
        // РАННЕЙ версии этой архитектуры (до текущих muscleLeakScale/
        // muscleCalciumTau/bodyPoseDecayRate) - см. WORM_V5_JOINT_CLAMP_
        // RESULTS.md за прямой проверкой этого факта: raising this hard
        // ceiling does not help - the stationary mid-body band just pins
        // against whatever new ceiling is set (zero variance at the
        // ceiling), never settling into a genuine sub-ceiling equilibrium.
        // Вынесен наружу (set_joint_angle_clamp, дефолт 0.25 - тот же самый
        // хардкод) именно для этой проверки; НЕ часть V2's frame-rate-limit
        // замены (см. rebuild_points ниже).
        //
        // WORM_V5_SOFT_JOINT_RESISTANCE_RESULTS.md: two additive, off-by-
        // default alternatives to the discontinuous hard clamp above, tried
        // as the project owner's own proposal ("resists turning instead of
        // strictly limiting it") - see set_joint_soft_saturation/
        // set_joint_stiffening_gain in body.hpp for the full derivation of
        // each. kJointHardSafetyLimit (1.2 rad) is the EXACT value this same
        // comment already names above as what caused catastrophic "tiny
        // knot" curling when it was the PRIMARY clamp - here it is only ever
        // a numerical backstop, never the mechanism doing the real limiting
        // work, for either alternative below.
        constexpr float kJointHardSafetyLimit = 1.2f;
        if (joint_soft_saturation_ > 0.0f) {
            // Mechanism 1 (smooth saturation): continuous-derivative
            // approach to +/-c (c = joint_angle_clamp_, reinterpreted as a
            // nominal scale, not a hard wall) instead of the discontinuous
            // std::clamp above. c*tanh(x/c) already lies strictly inside
            // (-c, c) for any finite x, so the safety clamp just below is a
            // pure backstop against non-finite values, not an active limiter
            // in ordinary operation.
            const float satScale = std::max(1e-6f, joint_angle_clamp_);
            angles_[i] = satScale * std::tanh(angles_[i] / satScale);
            angles_[i] = std::clamp(angles_[i], -kJointHardSafetyLimit, kJointHardSafetyLimit);
        } else if (joint_stiffening_gain_ > 0.0f) {
            // Mechanism 2 (nonlinear stiffening resistance) already added its
            // cubic restoring term further up (before pose-decay's leaky
            // integrator, see above) - here we intentionally do NOT apply
            // the small joint_angle_clamp_ hard wall: the entire point of
            // this mechanism is that k_soft vs. drive sets the natural
            // equilibrium, not an externally imposed ceiling pinning it. Only
            // the large safety net applies.
            angles_[i] = std::clamp(angles_[i], -kJointHardSafetyLimit, kJointHardSafetyLimit);
        } else {
            // Default (both toggles off): bitwise-identical to the original
            // hard clamp.
            angles_[i] = std::clamp(angles_[i], -joint_angle_clamp_, joint_angle_clamp_);
        }
        // ПРЕДЕЛ СКОРОСТИ ИЗМЕНЕНИЯ УГЛА - см. set_joint_angle_rate_limit.
        // Стоит ПОСЛЕДНИМ, после всех ветвей ограничения: ограничивается
        // реально случившееся за шаг изменение, каким бы механизмом оно ни было
        // получено. anglesPrev[i] - значение на начало шага (тот же снимок, что
        // читает изгибная жёсткость выше).
        //
        // Используется НАСТОЯЩИЙ dt, а не bendDt: это предел скорости
        // сокращения мышцы, физическая величина, и масштабировать её вместе с
        // модельным временем изгиба неправильно.
        if (joint_angle_rate_limit_ > 0.0f && dt > 0.0f) {
            const float maxStep = joint_angle_rate_limit_ * dt;
            const float prev = anglesPrev[i];
            angles_[i] = prev + std::clamp(angles_[i] - prev, -maxStep, maxStep);
        }
    }
    rebuild_points(dt);
    solve_propulsion(old_x, old_y, dt);
}

// Обновляет penetration_depth_ (Kelvin-Voigt-style релаксация к нормированной
// нагрузке ПРОШЛОГО шага) - см. set_drag_settle за полным обоснованием.
// dMax=10.0 - мягкий численный потолок (страховка от разгона при аномально
// большой нагрузке), не физическая константа.
void WormBody::update_penetration_depth(float dt) {
    const std::size_t n = angles_.size();
    if (penetration_depth_.size() != n) penetration_depth_.assign(n, 0.0f);
    if (dt <= 0.0f) return;

    constexpr float kDMax = 10.0f;
    const float tau = std::max(1e-6f, drag_settle_tau_);
    const float alpha = std::clamp(dt / tau, 0.0f, 1.0f);
    const float dragNormalSafe = std::max(1e-6f, drag_normal_);
    const std::size_t loadCount = segment_load_.size();
    for (std::size_t k = 0; k < n; ++k) {
        const float loadPrev = (k < loadCount) ? segment_load_[k] : 0.0f;
        const float dEq = std::clamp(loadPrev / dragNormalSafe, 0.0f, kDMax);
        penetration_depth_[k] += (dEq - penetration_depth_[k]) * alpha;
    }
}

// Строит points_x_/points_y_ из frame_heading_ - ОРИЕНТАЦИИ каждого
// сегмента, физически ограниченной по скорости изменения (см.
// set_max_frame_angular_rate), а НЕ из мгновенной кумулятивной суммы углов
// напрямую (как в v1). target - та же наивная prefix-sum, что строила
// heading в v1, но теперь используется только как ЦЕЛЬ, к которой
// frame_heading_[i] релаксирует со скоростью не выше max_frame_angular_rate_
// - см. WORM_V2_DESIGN.md раздел 3.2 за полным выводом того, почему это
// структурно устраняет геометрическое умножение коррелированного дрейфа
// угла по цепочке (v1's причина редких выбросов |u_k| до ~2*10^5 units/s,
// см. tests/worm_jerkiness_diagnostic), а не просто клампирует его следствие
// постфактум.
void WormBody::rebuild_points(float dt) {
    const std::size_t n = angles_.size();
    if (frame_heading_.size() != n) frame_heading_.assign(n, 0.0f);
    points_x_.assign(n + 1, 0.0f);
    points_y_.assign(n + 1, 0.0f);

    // rate<=0 => нет ограничения - frame_heading_ мгновенно телепортируется к
    // target, побитово поведение "голой" наивной кинематики v1 без клампа
    // (диагностический/сравнительный режим, см. Params::bodyFrameRateLimitHz).
    const bool haveLimit = max_frame_angular_rate_ > 0.0f && dt > 0.0f;
    const float maxStep = haveLimit ? max_frame_angular_rate_ * dt : 0.0f;

    float x = 0.0f;
    float y = 0.0f;
    float target = 0.0f;
    points_x_[0] = x;
    points_y_[0] = y;
    for (std::size_t i = 0; i < n; ++i) {
        target += angles_[i];
        if (haveLimit) {
            const float delta = std::clamp(target - frame_heading_[i], -maxStep, maxStep);
            frame_heading_[i] += delta;
        } else {
            frame_heading_[i] = target;
        }
        x += segment_length_ * std::cos(frame_heading_[i]);
        y += segment_length_ * std::sin(frame_heading_[i]);
        points_x_[i + 1] = x;
        points_y_[i + 1] = y;
    }
}

// Анизотропное вязкое трение о субстрат (resistive force theory): для сегмента
// k с единичной касательной t_k сила трения на его центре
//   F_k(v) = -(c_n * v + (c_t - c_n) * (v . t_k) * t_k)
// (c_t вдоль тела, c_n поперёк - при c_t==c_n анизотропии нет и волна изгиба
// никуда не "толкает"). c_n дополнительно пер-сегментно получает аддитивный
// член от penetration_depth_ (см. set_drag_settle) - память проседания в
// субстрат с постоянной времени, ЗАМЕНА трёх мгновенных adhesion-формул v1
// (все три были функциями ОДНОЙ И ТОЙ ЖЕ величины |u_k| в тот же момент
// времени и математически сходились к одному потолку water/agar~=1.0-1.02,
// см. WORM.md раздел 6 за полной историей). penetration_depth_[k] уже
// известен из ПРОШЛОГО шага (обновлён update_penetration_depth ДО вызова
// этой функции) - зависимость от искомых (Vx,Vy,w) отсутствует, система
// остаётся ЛИНЕЙНОЙ. Тело считается безынерционным (квази-статическое
// равновесие - вязкое трение о субстрат на масштабе и скоростях нематоды на
// порядки превосходит инерционные силы, стандартное допущение для локомоции
// такого рода), поэтому в каждый момент сумма сил и сумма моментов
// относительно points_[0] равны нулю:
//   sum_k F_k(v_k) = 0,     sum_k r_k x F_k(v_k) = 0
// Скорость центра сегмента v_k = v_rigid(Vx,Vy,w) + u_k, где u_k - скорость
// ТОЛЬКО от изменения формы (конечная разность центра между предыдущей и
// новой позой при неподвижной точке отсчёта points_[0]), а v_rigid(Vx,Vy,w) =
// (Vx - w*r_k.y, Vy + w*r_k.x) - вклад искомого жёсткого перемещения/поворота
// всего тела (r_k - центр сегмента относительно points_[0]). Оба уравнения
// линейны по (Vx,Vy,w): получаем систему 3x3, столбцы которой - реакция на
// единичные Vx/Vy/w, а правая часть - минус реакция на "чистое" u_k; решаем
// напрямую по Крамеру (или возвращаем нулевую скорость, если форма не
// меняется - тогда правая часть уже нулевая и это единственное решение).
void WormBody::solve_propulsion(const std::vector<float>& old_x, const std::vector<float>& old_y, float dt) {
    const std::size_t n = angles_.size();
    if (n == 0 || dt <= 0.0f || old_x.size() != points_x_.size()) {
        local_vel_x_ = local_vel_y_ = angular_velocity_ = mechanical_load_ = 0.0f;
        segment_load_.assign(n, 0.0f);
        return;
    }

    std::vector<float> tx(n), ty(n), rx(n), ry(n), ux(n), uy(n);
    for (std::size_t k = 0; k < n; ++k) {
        const float cx = 0.5f * (points_x_[k] + points_x_[k + 1]);
        const float cy = 0.5f * (points_y_[k] + points_y_[k + 1]);
        const float ocx = 0.5f * (old_x[k] + old_x[k + 1]);
        const float ocy = 0.5f * (old_y[k] + old_y[k + 1]);
        rx[k] = cx;
        ry[k] = cy;
        ux[k] = (cx - ocx) / dt;
        uy[k] = (cy - ocy) / dt;

        float dx = points_x_[k + 1] - points_x_[k];
        float dy = points_y_[k + 1] - points_y_[k];
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len > 1e-6f) { dx /= len; dy /= len; } else { dx = 1.0f; dy = 0.0f; }
        tx[k] = dx;
        ty[k] = dy;
    }

    const float ct = drag_tangent_;
    const float cn = drag_normal_;

    // Пер-сегментный c_n с аддитивным членом от penetration_depth_ - см.
    // set_drag_settle. gain=0 - cnk[k]==cn для всех k, побитово чисто
    // линейная RFT-физика.
    std::vector<float> cnk(n, cn);
    if (drag_settle_gain_ != 0.0f) {
        for (std::size_t k = 0; k < n; ++k) {
            const float pen = (k < penetration_depth_.size()) ? penetration_depth_[k] : 0.0f;
            cnk[k] = std::max(0.0f, cn + drag_settle_gain_ * pen);
        }
    }

    // D_k(vx,vy) -> (fx,fy) = cnk[k]*v + (ct-cnk[k])*(v.t)*t
    auto drag = [&](std::size_t k, float vx, float vy, float& fx, float& fy) {
        const float vt = vx * tx[k] + vy * ty[k];
        fx = cnk[k] * vx + (ct - cnk[k]) * vt * tx[k];
        fy = cnk[k] * vy + (ct - cnk[k]) * vt * ty[k];
    };

    float A[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    float b[3] = {0.0f, 0.0f, 0.0f};

    // Columns of A: response to unit Vx, unit Vy, unit w in turn.
    for (int col = 0; col < 3; ++col) {
        float sfx = 0.0f, sfy = 0.0f, stau = 0.0f;
        for (std::size_t k = 0; k < n; ++k) {
            float vx = 0.0f, vy = 0.0f;
            if (col == 0) vx = 1.0f;
            else if (col == 1) vy = 1.0f;
            else { vx = -ry[k]; vy = rx[k]; } // w x r, w=1
            float fx, fy;
            drag(k, vx, vy, fx, fy);
            sfx += fx;
            sfy += fy;
            stau += rx[k] * fy - ry[k] * fx;
        }
        A[0][col] = sfx;
        A[1][col] = sfy;
        A[2][col] = stau;
    }

    // RHS: -sum_k D_k(u_k) (pure shape-change contribution).
    {
        float sfx = 0.0f, sfy = 0.0f, stau = 0.0f;
        for (std::size_t k = 0; k < n; ++k) {
            float fx, fy;
            drag(k, ux[k], uy[k], fx, fy);
            sfx += fx;
            sfy += fy;
            stau += rx[k] * fy - ry[k] * fx;
        }
        b[0] = -sfx;
        b[1] = -sfy;
        b[2] = -stau;
    }
    last_rhs_magnitude_ = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);

    auto det3 = [](const float m[3][3]) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    const float d = det3(A);
    last_determinant_ = d;
    if (std::fabs(d) < 1e-9f) {
        local_vel_x_ = local_vel_y_ = angular_velocity_ = mechanical_load_ = 0.0f;
        segment_load_.assign(n, 0.0f);
        return;
    }
    float X[3];
    for (int col = 0; col < 3; ++col) {
        float M[3][3];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                M[r][c] = (c == col) ? b[r] : A[r][c];
        X[col] = det3(M) / d;
    }

    local_vel_x_ = X[0];
    local_vel_y_ = X[1];
    angular_velocity_ = X[2];

    // mechanical_load_ - сумма |сила трения| по всем сегментам НА УЖЕ
    // РЕШЁННОЙ скорости этого шага (v_rigid(X) + u_k, то же v_k, что и в
    // уравнениях баланса выше) - реальная суммарная реакция среды на то, что
    // тело только что сделало, не гадание по drag_normal_ напрямую. Чисто
    // диагностический побочный продукт уже решённой системы - не влияет ни на
    // X[], ни на что-либо ещё в этом классе. ТАКЖЕ вход для
    // update_penetration_depth() СЛЕДУЮЩЕГО шага (см. segment_load()).
    float totalLoad = 0.0f;
    segment_load_.assign(n, 0.0f);
    for (std::size_t k = 0; k < n; ++k) {
        const float vx = X[0] - X[2] * ry[k] + ux[k];
        const float vy = X[1] + X[2] * rx[k] + uy[k];
        float fx, fy;
        drag(k, vx, vy, fx, fy);
        const float mag = std::sqrt(fx * fx + fy * fy);
        segment_load_[k] = mag;
        totalLoad += mag;
    }
    mechanical_load_ = totalLoad;
}

} // namespace connectome
