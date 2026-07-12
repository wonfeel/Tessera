// Shaders/spring_point.frag
#version 460 core
in float vEnergy;
out vec4 FragColor;

// < 1 при отдалении камеры — см. spring_line.frag.
uniform float uDim;
// См. spring_point.vert — тот же переключатель, тут решает форму пятна:
// круглое мягкое свечение (0) или сплошной квадрат-кубик (1).
uniform int uCubeMode;

void main() {
    float falloff;
    if (uCubeMode == 1) {
        falloff = 1.0;   // сплошной квадрат на весь point sprite — "кубик"
    } else {
        // Мягкое круглое свечение вместо квадратной точки — расстояние от
        // центра point sprite'а (gl_PointCoord — [0,1]x[0,1] внутри точки).
        vec2 d = gl_PointCoord - vec2(0.5);
        float dist = length(d) * 2.0;
        falloff = smoothstep(1.0, 0.0, dist);
    }

    vec3 base = vec3(0.2, 0.35, 0.6);
    vec3 hot  = vec3(1.0, 0.95, 0.8);
    float t = clamp(vEnergy, 0.0, 1.0);
    vec3 color = mix(base, hot, t);

    float alpha = falloff * (0.3 + t) * uDim;
    FragColor = vec4(color * alpha, alpha);
}
