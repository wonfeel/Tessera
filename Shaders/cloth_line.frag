// Shaders/spring_line.frag
#version 460 core
in float vGlow;
out vec4 FragColor;

// < 1 при отдалении камеры — гасит яркость, чтобы при аддитивном блендинге
// плотно перекрывающиеся на экране пружины не сливались в засвеченное пятно.
uniform float uDim;

void main() {
    // Тускло-синий базовый тон -> тёплый белый на пике натяжения ребра.
    // Аддитивный блендинг (см. main.cpp) сам даёт эффект свечения там,
    // где пересекается несколько ярких пружин.
    vec3 base = vec3(0.05, 0.08, 0.15);
    vec3 hot  = vec3(1.0, 0.85, 0.55);
    float t = clamp(vGlow, 0.0, 1.0);
    vec3 color = mix(base, hot, t);
    FragColor = vec4(color * (0.15 + t) * uDim, 1.0);
}
