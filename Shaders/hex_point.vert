// Shaders/hex_point.vert
#version 460 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec3 aColor;   // яркость уже посчитана на CPU вызывающей стороной

uniform mat4 uCamera;
uniform float uBaseSize;
// Полная высота гекса (вершина-вершина = 2*spacing) в текущих экранных
// пикселях (+1px нахлёст) - квадратный габарит GL_POINTS, общий для обеих
// форм (см. uShapeMode и hex_point.frag).
uniform float uCellSizePx;
// 0 = шестиугольник (честное замащивание решётки, фиксированный размер),
// 1 = круг (мягкий, растёт с энергией).
uniform int uShapeMode;

out vec3 vColor;

void main() {
    gl_Position = uCamera * vec4(aPos, 0.0, 1.0);
    float t = clamp(max(aColor.r, max(aColor.g, aColor.b)), 0.0, 1.0);
    if (uShapeMode == 0) {
        // Фиксированный размер - честное замащивание требует, чтобы соседние
        // шестиугольники стыковались РОВНО по границе; раздувать один из них
        // ярче/больше значило бы наползать на соседний. Яркость - только
        // через цвет (vColor), не размер.
        gl_PointSize = max(uBaseSize, uCellSizePx);
    } else {
        // Круг может расти с энергией - его перекрытие с соседями (в отличие
        // от гекса) просто складывается в общую яркость на аддитивном
        // блендинге, не режет глаз хардкодной кромкой.
        gl_PointSize = max(uBaseSize, uCellSizePx * (1.0 + t * 0.5));
    }
    vColor = aColor;
}
