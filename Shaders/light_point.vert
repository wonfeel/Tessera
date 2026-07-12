// Shaders/light_point.vert
#version 460 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec3 aColor;   // R/G/B яркость уже посчитана на CPU (LightApp::renderPointLayer)

uniform mat4 uCamera;
uniform float uBaseSize;
// Мировой spacing сетки в текущих экранных пикселях (spacing*zoom) - размер
// кубика в Cube mode, чтобы кубики стыковались друг с другом на любом зуме
// (см. тот же приём в Shaders/cloth_point.vert).
uniform float uCellSizePx;
uniform int uCubeMode;

out vec3 vColor;

void main() {
    gl_Position = uCamera * vec4(aPos, 0.0, 1.0);
    float t = clamp(max(aColor.r, max(aColor.g, aColor.b)), 0.0, 1.0);
    // Раньше в Points mode размер был ЧИСТО в экранных пикселях (uBaseSize),
    // не зависел от zoom вообще - при приближении точки оставались тем же
    // числом пикселей, а расстояние между узлами (в пикселях) росло, поэтому
    // относительно сетки они визуально "усыхали". Берём долю от uCellSizePx
    // (тот же зум-зависимый масштаб, что и Cube mode - см. main.cpp) как базу,
    // energy (t) добавляет поверх неё, а не служит единственным источником
    // размера.
    float zoomedBase = uCellSizePx * 0.5;
    gl_PointSize = (uCubeMode == 1) ? uCellSizePx : max(uBaseSize, zoomedBase * (1.0 + t * 1.5));
    vColor = aColor;
}
