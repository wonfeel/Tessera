#include "body.hpp"

#include <algorithm>
#include <cmath>

namespace connectome {

WormBody::WormBody(int num_segments, float segment_length, float drag_tangent, float drag_normal)
    : segment_length_(segment_length),
      drag_tangent_(drag_tangent),
      drag_normal_(drag_normal),
      angles_(static_cast<std::size_t>(num_segments), 0.0f) {
    rebuild_points();
}

void WormBody::step(const std::vector<float>& curvature, float dt) {
    const std::vector<float> old_x = points_x_;
    const std::vector<float> old_y = points_y_;

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
    for (std::size_t i = 0; i < angles_.size(); ++i) {
        const float c = i < curvature.size() ? curvature[i] : 0.0f;
        angles_[i] += c * dt;
        angles_[i] *= (1.0f - std::min(1.0f, pose_decay_rate_ * dt)); // затухание к нейтральной позе (см. set_pose_decay_rate)
        // Кламп на сегмент: слишком широкий (было ±1.2 рад/~69°) позволял
        // телу закрутиться на несколько полных оборотов при кривизне одного
        // знака на большинстве сегментов сразу (не редкость в gap-связанной
        // сети) - выглядело как крошечный узел вместо червя. Слишком узкий
        // (~0.18) даёт вытянутое тело, но почти убивает поворотливость (был
        // проверен headless-перебором - roam схлопывался почти в одну ось).
        // ±0.25 - компромисс, подобранный тем же перебором: тело остаётся
        // явно вытянутым (bbox-диагональ/длина дуги ~0.5-0.6, не ~0.2-0.3),
        // сохраняя реальную двумерную манёвренность.
        angles_[i] = std::clamp(angles_[i], -0.25f, 0.25f);
    }
    rebuild_points();
    solve_propulsion(old_x, old_y, dt);
}

void WormBody::rebuild_points() {
    const std::size_t n = angles_.size();
    points_x_.assign(n + 1, 0.0f);
    points_y_.assign(n + 1, 0.0f);

    float x = 0.0f;
    float y = 0.0f;
    float heading = 0.0f;
    points_x_[0] = x;
    points_y_[0] = y;
    for (std::size_t i = 0; i < n; ++i) {
        heading += angles_[i];
        x += segment_length_ * std::cos(heading);
        y += segment_length_ * std::sin(heading);
        points_x_[i + 1] = x;
        points_y_[i + 1] = y;
    }
}

// Анизотропное вязкое трение о субстрат (resistive force theory): для сегмента
// k с единичной касательной t_k сила трения на его центре
//   F_k(v) = -(c_n * v + (c_t - c_n) * (v . t_k) * t_k)
// (c_t вдоль тела, c_n поперёк - при c_t==c_n анизотропии нет и волна изгиба
// никуда не "толкает"). Тело считается безынерционным (квази-статическое
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

    // D_k(vx,vy) -> (fx,fy) = cn*v + (ct-cn)*(v.t)*t
    auto drag = [&](std::size_t k, float vx, float vy, float& fx, float& fy) {
        const float vt = vx * tx[k] + vy * ty[k];
        fx = cn * vx + (ct - cn) * vt * tx[k];
        fy = cn * vy + (ct - cn) * vt * ty[k];
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

    auto det3 = [](const float m[3][3]) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    const float d = det3(A);
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
    // X[], ни на что-либо ещё в этом классе.
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
