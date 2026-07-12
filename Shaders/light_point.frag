// Shaders/light_point.frag
#version 460 core
in vec3 vColor;
out vec4 FragColor;

// < 1 при отдалении камеры - см. Shaders/cloth_line.frag.
uniform float uDim;
// См. light_point.vert - тот же переключатель формы пятна.
uniform int uCubeMode;

void main() {
    float falloff;
    if (uCubeMode == 1) {
        falloff = 1.0;
    } else {
        vec2 d = gl_PointCoord - vec2(0.5);
        float dist = length(d) * 2.0;
        falloff = smoothstep(1.0, 0.0, dist);
    }

    // В отличие от cloth (один скаляр -> lerp base/hot), здесь RGB уже
    // посчитан на CPU (три независимых поля R/G/B + затемнение призмой -
    // см. LightApp::renderPointLayer) - фрагментный шейдер просто выводит
    // готовый цвет с аддитивным блендингом.
    float t = clamp(max(vColor.r, max(vColor.g, vColor.b)), 0.0, 1.0);
    float alpha = falloff * (0.3 + t) * uDim;
    FragColor = vec4(vColor * alpha, alpha);
}
