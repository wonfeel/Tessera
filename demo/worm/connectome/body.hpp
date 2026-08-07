#pragma once

#include <vector>

namespace connectome {

// Кинематика формы тела (цепочка из N сегментов постоянной длины, кривизна
// каждого задаётся извне - обычно разницей активации дорсальной/вентральной
// мышцы в этой точке тела) ПЛЮС честная физика перемещения: анизотропное
// трение о субстрат (resistive force theory - тот же принцип, которым
// реальные нематоды и другие безногие ползающие животные превращают бегущую
// волну изгиба в поступательное движение; см. Niebur & Erdos 1991 "Theory of
// locomotion of nematodes", Gray & Lissmann 1938 "Studies in Animal
// Locomotion VII: Locomotory Reflexes in the Earthworm" (не их работа 1950
// года - та про змею, а не про дождевого червя), Hatton & Choset для
// змеевидных роботов). Тело не задаёт курс/скорость само -
// они РЕШАЮТСЯ каждый шаг из баланса сил трения на возникшей форме, поэтому
// смещение физически привязано к тому, как именно изгибается тело, а не к
// отдельной эвристике "знак кривизны -> поворот".
//
// Упрощения относительно честной гидродинамики (SPH, как в Sibernetic у
// OpenWorm): трение считается чисто анизотропным вязким (сила ~ скорость, не
// ~scorость^2 или контактная механика), тело - недеформируемая цепочка
// отрезков без массы/инерции (квази-статическое равновесие: на масштабе
// нематоды и её скоростей вязкое трение о субстрат подавляет инерцию на
// порядки, так что это не потеря точности, а стандартное для этого масштаба
// приближение).
//
// V2 (см. H:\workspace\projects\Tessera\demo\worm\WORM_V2_DESIGN.md разделы
// 3-4) заменяет два патча из v1 честно-вычисленными механизмами:
//   - кинематика ориентации сегментов (было: мгновенная кумулятивная сумма
//     углов + постфактум-кламп на |u_k|; стало: per-joint bounded-rate
//     relaxation ДО построения позиций - см. rebuild_points/frame_heading_);
//   - "прилипание" к агару (было: три мгновенные алгебраические поправки к
//     c_n; стало: память проседания с постоянной времени - см.
//     penetration_depth_/set_drag_settle).
class WormBody {
public:
    WormBody(int num_segments, float segment_length, float drag_tangent = 1.0f, float drag_normal = 6.0f);

    // curvature[i] - желаемая угловая скорость сегмента i (рад/с);
    // положительная = изгиб в одну сторону (условно "дорсальную"). Обновляет
    // форму, затем решает баланс сил трения на новой форме, чтобы получить
    // local_velocity()/angular_velocity() - перемещение и поворот ЭТОГО шага
    // в собственной (не повёрнутой) системе координат тела.
    void step(const std::vector<float>& curvature, float dt);

    const std::vector<float>& points_x() const { return points_x_; }
    const std::vector<float>& points_y() const { return points_y_; }
    int num_segments() const { return static_cast<int>(angles_.size()); }

    // Реальный (уже прошедший физику и кламп) угол изгиба каждого сегмента -
    // это и есть механический аналог "растяжения", которое в настоящем
    // C. elegans чувствуют stretch-рецепторы B-типа мотонейронов (Boyle,
    // Berri & Cohen 2012): не то, что сеть "хотела" выдать (curvature/
    // deviation до тела), а то, что тело ФАКТИЧЕСКИ приняло после клампа и
    // затухания. Для честной проприоцептивной обратной связи нужен именно
    // этот сигнал, а не внутренний нейронный. ВАЖНО (V2): это ПО-ПРЕЖНЕМУ
    // angles_[i] (угол СУСТАВА), не новая frame_heading_ (накопленная
    // ОРИЕНТАЦИЯ сегмента, см. ниже) - проприоцепция не завязана на
    // ограничение скорости ориентации, весь его эффект локализован внутри
    // rebuild_points()/solve_propulsion().
    const std::vector<float>& angles() const { return angles_; }

    // ОРИЕНТАЦИЯ сегмента после ограничения скорости (frame_heading_, см.
    // rebuild_points) - в отличие от angles() выше это то, что реально
    // построило points_ и, значит, реально видно на экране и реально
    // участвует в solve_propulsion. Разность соседних элементов -
    // РЕАЛИЗОВАННЫЙ угол сустава; он совпадает с angles()[i] ТОЛЬКО пока
    // ограничитель не связывает. Чисто диагностический доступ (WORM_V5:
    // проверка гипотезы "панель показывает амплитуду, а тело её не
    // отрабатывает"), ни на что в этом классе не влияет.
    const std::vector<float>& frame_heading() const { return frame_heading_; }

    // Скорость поступательного движения точки points_[0] (см. rebuild_points)
    // в СОБСТВЕННОЙ системе координат тела - вызывающий код поворачивает её на
    // текущий глобальный heading и интегрирует позицию сам (тело не знает про
    // мировые координаты).
    float local_velocity_x() const { return local_vel_x_; }
    float local_velocity_y() const { return local_vel_y_; }
    // Угловая скорость (рад/с) - применяется к глобальному heading напрямую,
    // это скаляр, не зависящий от системы отсчёта в 2D.
    float angular_velocity() const { return angular_velocity_; }

    // Суммарная (по всем сегментам) сила трения среды при УЖЕ решённой
    // скорости этого шага - честный физический прокси "с какой мехнагрузкой
    // тело сейчас реально борется", а не гадание по параметру среды напрямую.
    // Чисто диагностическое значение (ни на что в этом классе не влияет) -
    // для проприоцептивной/механосенсорной обратной связи снаружи (DVA).
    float mechanical_load() const { return mechanical_load_; }

    // Диагностика (tests/worm_jerkiness_diagnostic) - det3(A) и |b| с
    // ПОСЛЕДНЕГО решения solve_propulsion, для отличения "матрица A почти
    // вырождена" (d -> 0) от "правая часть b сама по себе огромна" (d
    // нормальный, но X=Cramer(A,b)/d всё равно большой, потому что b
    // большой) - см. traceSpike за расследованием редкого выброса.
    float debug_last_determinant() const { return last_determinant_; }
    float debug_last_rhs_magnitude() const { return last_rhs_magnitude_; }

    // То же самое (|сила трения| на уже решённой скорости), но ПО СЕГМЕНТАМ,
    // не суммой - реальные B/D-мотонейроны C. elegans сами являются
    // stretch-рецепторами и чувствуют ЛОКАЛЬНОЕ растяжение своего участка тела
    // (Wen et al. 2012 Neuron, Yeon et al. 2018 Cell), а не одну глобальную
    // цифру на весь организм (это разделение труда с DVA/mechanical_load() -
    // тот честный прокси ЦЕЛОСТНОЙ нагрузки для премоторных интернейронов,
    // этот - для локальной проприоцептивной петли per-мотонейрон). Индекс i
    // соответствует angles()[i]/сегменту i. ТАКЖЕ вход для penetration_depth_
    // (см. ниже) - каждый новый step() использует значения ПРОШЛОГО шага,
    // ещё не перезаписанные новым solve_propulsion.
    const std::vector<float>& segment_load() const { return segment_load_; }

    // c_t (вдоль тела) и c_n (поперёк) - коэффициенты анизотропного трения.
    // Именно их РАЗНИЦА (не абсолютная величина) определяет, насколько
    // эффективно бегущая волна изгиба превращается в чистое перемещение -
    // при c_t == c_n тело изгибается, но никуда не едет (нет анизотропии,
    // нечего "отталкивать"). Реальное ползание по агару даёт анизотропию
    // порядка нескольких единиц - десятков раз.
    void set_drag(float tangent, float normal) { drag_tangent_ = tangent; drag_normal_ = normal; }

    // Скорость затухания угла сегмента к нейтральной позе (см. WormBody::
    // step) - множитель перед dt в (1 - min(1, rate*dt)). Было зашито как
    // константа 0.5 - вынесено наружу ТОЛЬКО для совместного поиска с
    // активным током B-класса (Network::set_active_current) - см.
    // tests/worm_bclass_body_joint_calibration. Дефолт 0.5 - тот же самый
    // хардкод, что и раньше, побитово то же поведение, если не трогать.
    // ПРЕДУПРЕЖДЕНИЕ (см. network.cpp за полной историей): более быстрое
    // затухание уже пробовали в связке с другим экспериментом по ускорению
    // сети - результат был РЕЗКО хуже (efficiency 0.52 -> 0.06), не лучше -
    // укороченная персистентность изгиба не даёт solve_propulsion ничего,
    // от чего оттолкнуться. Это не повод считать направление заведомо
    // мёртвым (тот эксперимент комбинировал другую, более грубую ось), но
    // повод не удивляться, если и здесь оптимум окажется НЕ в сторону
    // "быстрее".
    void set_pose_decay_rate(float rate) { pose_decay_rate_ = rate; }

    // ПАМЯТЬ проседания в субстрат (Kelvin-Voigt-style релаксация) - ЗАМЕНА
    // трёх мгновенных adhesion-формул v1 (все три - см. git-историю body.hpp
    // и WORM.md раздел 6 - были алгебраическими функциями ОДНОЙ И ТОЙ ЖЕ
    // кинематической величины |u_k| в тот же момент времени и математически
    // ВСЕ сходились к одному и тому же потолку water/agar~=1.0-1.02 при
    // больших коэффициентах - доказанное структурное свойство ВСЕГО класса
    // "мгновенная функция от |u_k|", не совпадение отдельных неудачных
    // чисел). Реальная агаровая вязкоупругость (Rabets, Backholm,
    // Dalnoki-Veress & Ryu 2014, Biophysical J. 107:1980 - secondhand,
    // честно помечено как ниже уверенности; Sauvage et al. 2011,
    // elasto-hydrodynamical model) описывается качественно иначе: червь
    // физически проседает в бороздку геля с конечной временной постоянной -
    // сопротивление в момент t зависит от того, СКОЛЬКО времени тело уже
    // давит на это место, не только с какой скоростью оно давит ПРЯМО
    // СЕЙЧАС. penetration_depth_ - новое персистентное состояние (по
    // сегменту), обновляемое КАЖДЫЙ step() ДО rebuild_points()/
    // solve_propulsion(), из segment_load() ПРОШЛОГО шага (см. .cpp за
    // точными уравнениями). c_n получает один аддитивный член:
    //   cnk[k] = cn + gain * penetration_depth_[k]
    // penetration_depth_[k] уже известен к моменту решения баланса сил этого
    // шага (обновлён из данных прошлого шага ДО solve_propulsion) - система
    // остаётся ЛИНЕЙНОЙ, решается тем же прямым Крамером, без итераций.
    // tau<=0 или gain==0.0 => cnk[k]==cn для всех k, побитово чисто линейная
    // RFT-физика.
    void set_drag_settle(float tau, float gain) { drag_settle_tau_ = tau; drag_settle_gain_ = gain; }

    // Верхняя граница угловой скорости, с которой накопленная ОРИЕНТАЦИЯ
    // каждого сегмента (frame_heading_, см. .cpp/rebuild_points) может
    // догонять свою мгновенную цель (prefix-sum угла сустава, та же
    // кумулятивная сумма, что строила points_x_/points_y_ в v1) - ЗАМЕНА
    // постфактум-клампа v1 на |u_k| (скорость центра сегмента ОТ смены
    // формы). Структурное отличие: v1 клампировал СЛЕДСТВИЕ (уже испорченную
    // кумулятивную сумму углов, ПОСЛЕ того как она вошла в построение
    // points_x_/points_y_ и в 3x3 решение баланса сил); эта схема ограничивает
    // саму ориентацию ДО того, как из неё вообще строятся точки - скачок по
    // многим сегментам сразу физически не может произвести скачок хвоста
    // больше rate*dt НА КАЖДЫЙ СУСТАВ (не суммарно), см. WORM_V2_DESIGN.md
    // раздел 3.2 за полным выводом. rate<=0 => НЕТ ограничения (frame_heading_
    // мгновенно телепортируется к цели, побитово поведение v1's rebuild_points
    // без клампа - для сравнительных/диагностических прогонов).
    void set_max_frame_angular_rate(float rate) { max_frame_angular_rate_ = rate; }

    // Множитель ВРЕМЕНИ для кинематики углов (и только для неё - построение
    // точек и решение баланса сил идут по настоящему dt). Смысл: мышца, изгибая
    // тело, преодолевает сопротивление среды, поэтому в менее вязкой среде та
    // же мышечная команда перекладывает сустав быстрее. Именно этого в модели
    // не было: angles_ += curvature*dt не зависел от трения вообще, из-за чего
    // частота волны получалась ПОБИТОВО одинаковой на агаре и в воде.
    //
    // Политика (чему равен множитель для данной среды) живёт в WormSim, где
    // лежат параметры; тело только применяет готовое число. 1.0 - нейтрально,
    // побитово прежнее поведение. См. Params::mediumBendCouplingKappa.
    void set_bend_time_scale(float scale) { bend_time_scale_ = scale; }

    // Предел скорости изменения угла СУСТАВА, рад/с. 0 - выключено.
    //
    // Диагностика выброса скорости (WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md
    // раздел 26) показала механизм: угол сустава стоит прижатым к клампу
    // ПОСТОЯННО (maxAngle равен клампу на каждом шаге), а привод при этом вдвое
    // превышает то, что кламп позволяет. Когда привод меняет знак, сустав
    // проходит весь свой диапазон за ~3 шага - тело работает как релейный
    // генератор, и резкая смена формы даёт скачок правой части в
    // solve_propulsion, то есть скачок решаемой скорости. Определитель матрицы
    // при этом стабилен - вырождение как причина отвергнуто.
    //
    // Отличается от set_max_frame_angular_rate: тот ограничивает скорость
    // изменения ОРИЕНТАЦИИ сегмента (накопленной суммы углов) и, как измерено,
    // не связывает вообще - потому что ограничивает не ту величину. Здесь
    // ограничивается изменение самого угла сустава за шаг.
    //
    // Физический смысл: скорость сокращения мышцы конечна, сустав не может
    // переложиться из одного крайнего положения в другое мгновенно.
    void set_joint_angle_rate_limit(float rate) { joint_angle_rate_limit_ = rate; }

    // РЕЖИМ МЫШЦЫ КАК ИСТОЧНИКА МОМЕНТА (tau>0 включает; 0 - прежняя чисто
    // кинематическая связь). См. WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md раздел 28.
    //
    // Основание - Boyle, Berri & Cohen 2012 (Front Comput Neurosci 6:10,
    // PMC3296079). Там мышца - пружина, у которой АКТИВАЦИЯ задаёт длину покоя
    // и жёсткость:
    //     f_M = kappa_M(A)*(L_0M(A) - L) + beta_M(A)*v
    //     L_0M(A) = L_0L - F_max*sigma(A)*(L_0L - L_min)
    //     kappa_M(A) = kappa_0M*F_max*sigma(A)
    // То есть активация задаёт ЦЕЛЬ, к которой элемент стремится, а не скорость
    // его изменения. Угловой аналог для сустава:
    //     dtheta/dt = (theta_target(A) - theta) / tau_muscle
    //
    // Почему это принципиально. В прежней связи dtheta/dt = curvature скорость
    // сустава пропорциональна приводу, а установившийся угол равен
    // привод/затухание - поэтому амплитуда и скорость жёстко связаны, и
    // измерение (раздел 27.5) показало сохраняющееся произведение
    // амплитуда*частота ~0.06-0.067 рад*Гц против биологических 0.275. Здесь
    // амплитуда задаётся ЦЕЛЬЮ, а скорость - жёсткостью (1/tau), то есть они
    // независимы: частоту можно поднимать, не трогая амплитуду.
    //
    // Побочно исчезает релейный режим (раздел 26.2): сустав ТОРМОЗИТ при
    // подходе к цели (скорость пропорциональна остатку), а не влетает в потолок
    // на полной скорости.
    //
    // tau по умолчанию 0.1с - постоянная времени мышцы у Boyle et al.
    // (tau_M = 100 мс).
    void set_muscle_target_tau(float tau) { muscle_target_tau_ = tau; }

    // Изгибная жёсткость (bending stiffness) - SPATIAL (не временная) связь
    // между СОСЕДНИМИ суставами, отсутствовавшая в v1/v2: pose_decay_rate_
    // связывает angles_[i] с САМИМ СОБОЙ (0-й порядок, "забывание" к
    // нейтральной позе), но между i и i±1 механической связи не было вообще
    // - каждый угол сустава эволюционировал независимо, единственная связь
    // между позициями тела шла ВЫШЕ по потоку, через саму сеть (gap junction
    // между мышцами). Реальная кутикула - механически непрерывная оболочка:
    // локальный излом одного сегмента, не поддержанный соседями, физически
    // сопротивляется самой геометрией материала (изгибная жёсткость EI,
    // Cohen & Ranner 2017, arXiv:1702.04988; латеральные spring-элементы
    // между соседними сегментами, Boyle, Berri & Cohen 2012). Реализовано как
    // дискретный лапласиан по индексу сегмента (overdamped-предел "curvature
    // diffusion" эластичного стержня в вязкой среде), free-free граничные
    // условия (зеркальный фиктивный узел на концах цепи - нулевой градиент
    // кривизны на свободном конце), см. WormBody::step за точной формулой.
    // ЧИСТО алгебраический, без собственного состояния/постоянной времени
    // (в отличие от penetration_depth_/dragSettleTau) - читает СНИМОК
    // angles_ со старта текущего шага и добавляет мгновенную поправку,
    // зависящую только от пространственного распределения угла вдоль тела
    // в этот момент, не от истории во времени. Живёт ЦЕЛИКОМ внутри
    // кинематического обновления angles_ в WormBody::step(), СТРОГО ДО
    // rebuild_points()/solve_propulsion() - 3x3 Крамер не затрагивается ни на
    // строку (см. WORM_V3_DESIGN.md разделы 2-3 за полным обоснованием,
    // включая явное объяснение того, почему буквальный "упругий член в
    // балансе сил", предложенный тремя из четырёх research-веток, - здесь
    // категориальная ошибка: angles_[i] в ЭТОЙ архитектуре - чисто
    // кинематическая величина, не выход решения баланса сил). Единицы 1/с,
    // той же природы, что и pose_decay_rate_ - коэффициент связи, не
    // физический модуль Юнга напрямую. 0.0 = выключено, побитово прежнее
    // поведение.
    void set_bend_stiffness(float k) { bend_stiffness_ = k; }

    // Per-segment joint-angle clamp (see WormBody::step's comment on the
    // ±0.25 rad literal it used to be) - vынесен наружу для
    // WORM_V5_JOINT_CLAMP_RESULTS.md's investigation into whether that
    // literal is itself another uncalibrated-for-this-architecture constant
    // (same pattern already found and fixed for muscleLeakScale/
    // muscleCalciumTau/bodyPoseDecayRate this session, see WORM_V5_REAL_
    // AMPLITUDE_CALIBRATION.md). Default 0.25 is the EXACT same hardcoded
    // value the clamp always had - bitwise prior behavior for any caller
    // that never calls this setter.
    void set_joint_angle_clamp(float limit) { joint_angle_clamp_ = limit; }

    // WORM_V5_JOINT_CLAMP_RESULTS.md found that raising the hard clamp above
    // 0.25 does not let the stationary mid-body band (peak ~joint 11 of 24)
    // settle into a genuine sub-ceiling equilibrium - it just pins against
    // whatever new ceiling is set (section 6.1: pin frequency decays slowly,
    // not to zero, as the ceiling rises). This toggle (see WORM_V5_SOFT_
    // JOINT_RESISTANCE_RESULTS.md) replaces the DISCONTINUOUS hard clamp's
    // effect with a smooth saturating nonlinearity of the SAME nominal scale
    // (joint_angle_clamp_, reinterpreted here as "c"):
    //   angles_[i] = c * tanh(angles_[i] / c)
    // This has a continuous derivative everywhere (sech^2(x/c), 1 at x=0,
    // decaying smoothly), approaches +/-c asymptotically without ever
    // discontinuously stopping, and is approximately the identity for
    // |angles_[i]| << c - small, healthy bends are essentially unaffected,
    // only the extremes get progressively "softer." A much larger true hard
    // safety ceiling (see WormBody::step, kJointHardSafetyLimit = 1.2 rad -
    // the exact value this project's own history says caused catastrophic
    // "tiny knot" curling, see the joint_angle_clamp_ comment above/body.cpp)
    // is still applied AFTER this, purely as a numerical safety net against
    // runaway values (NaN/Inf), not as the primary limiting mechanism - in
    // exact arithmetic tanh already bounds the result strictly inside
    // (-c, c), so that safety clamp should never bind in practice.
    // enable<=0 (default) = off, bitwise-identical hard clamp at
    // joint_angle_clamp_ (today's shipped behavior). enable>0 = on.
    void set_joint_soft_saturation(float enable) { joint_soft_saturation_ = enable; }

    // WORM_V5_SOFT_JOINT_RESISTANCE_RESULTS.md's second, physically distinct
    // mechanism - closer to the project owner's literal proposal ("resists
    // turning instead of strictly limiting it"): a cubic (Duffing-oscillator
    // -style) softening-spring restoring term, added to the angle
    // accumulation BEFORE any clamp/saturation:
    //   angles_[i] += -k_soft * angles_[i]^3 * dt
    // Negligible near zero (cubic - a healthy small bend is barely affected),
    // growing rapidly as |angle| grows - the NATURAL equilibrium is wherever
    // this resistance balances the network's own drive, not an externally
    // imposed ceiling. When this is active (k_soft > 0) WITHOUT soft
    // saturation also enabled, the small joint_angle_clamp_ hard clamp is
    // bypassed entirely (see WormBody::step) - only the same large safety-net
    // ceiling used by set_joint_soft_saturation applies, since the whole
    // point of this mechanism is to let the equilibrium be set by k_soft vs.
    // drive, not pinned against the old ±0.25 rad literal. k_soft<=0
    // (default) = off, bitwise-identical hard clamp at joint_angle_clamp_.
    void set_joint_stiffening_gain(float k_soft) { joint_stiffening_gain_ = k_soft; }

private:
    float segment_length_;
    float drag_tangent_;
    float drag_normal_;
    float pose_decay_rate_ = 0.5f;
    float drag_settle_tau_ = 2.0f;
    float drag_settle_gain_ = 0.0f;
    float max_frame_angular_rate_ = 0.0f;
    float bend_time_scale_ = 1.0f;         // см. set_bend_time_scale
    float joint_angle_rate_limit_ = 0.0f;  // см. set_joint_angle_rate_limit
    float muscle_target_tau_ = 0.0f;       // см. set_muscle_target_tau
    float bend_stiffness_ = 0.0f;
    float joint_angle_clamp_ = 0.25f;
    // See set_joint_soft_saturation/set_joint_stiffening_gain above
    // (WORM_V5_SOFT_JOINT_RESISTANCE_RESULTS.md). Both default to 0.0f = off,
    // bitwise-identical to the plain hard clamp above for any caller that
    // never calls either setter.
    float joint_soft_saturation_ = 0.0f;
    float joint_stiffening_gain_ = 0.0f;
    std::vector<float> angles_;
    std::vector<float> points_x_;
    std::vector<float> points_y_;

    // Накопленная ОРИЕНТАЦИЯ каждого сегмента (в отличие от angles_[i],
    // угла ЭТОГО сустава относительно предыдущего) - физически ограниченная
    // по скорости версия наивной prefix-sum, см. set_max_frame_angular_rate/
    // rebuild_points. Одна запись на сегмент (не n+1, как points_x_/points_y_
    // - те включают точку отсчёта).
    std::vector<float> frame_heading_;

    // Проседание в субстрат по сегменту, см. set_drag_settle.
    std::vector<float> penetration_depth_;

    float local_vel_x_ = 0.0f;
    float local_vel_y_ = 0.0f;
    float angular_velocity_ = 0.0f;
    float mechanical_load_ = 0.0f;
    std::vector<float> segment_load_;
    float last_determinant_ = 0.0f;
    float last_rhs_magnitude_ = 0.0f;

    void update_penetration_depth(float dt);
    void rebuild_points(float dt);
    void solve_propulsion(const std::vector<float>& old_x, const std::vector<float>& old_y, float dt);
};

} // namespace connectome
