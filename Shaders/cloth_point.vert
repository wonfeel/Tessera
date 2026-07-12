// Shaders/spring_point.vert
#version 460 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in float aEnergy;

uniform mat4 uCamera;
uniform float uBaseSize;
// Мировой spacing сетки в текущих экранных пикселях (spacing*zoom) — размер
// кубика в Cube mode, чтобы кубики стыковались друг с другом на любом зуме
// (в отличие от uBaseSize, который в фиксированных пикселях и "усыхает"
// относительно клетки сетки при приближении).
uniform float uCellSizePx;
// 0 = точки (текущий стиль, размер растёт с энергией), 1 = кубики
// (размер = uCellSizePx, меняется только яркость в spring_point.frag).
uniform int uCubeMode;

out float vEnergy;

void main() {
    gl_Position = uCamera * vec4(aPos, 0.0, 1.0);
    float t = clamp(aEnergy, 0.0, 1.0);
    gl_PointSize = (uCubeMode == 1) ? uCellSizePx : uBaseSize * (1.0 + t * 3.0);
    vEnergy = t;
}
