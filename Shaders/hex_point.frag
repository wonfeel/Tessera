// Shaders/hex_point.frag
#version 460 core
in vec3 vColor;
out vec4 FragColor;

// См. hex_point.vert.
uniform int uShapeMode;
// Нужен ЗДЕСЬ (не только в .vert) для адаптивного сглаживания гекса при
// маленьком размере - см. ниже.
uniform float uCellSizePx;

void main() {
    float shapeAlpha;

    if (uShapeMode == 0) {
        // gl_PointCoord [0,1]^2 - квадратный габарит точки, высота которого
        // равна полной высоте гекса (вершина-вершина, см. .vert). n в
        // [-1,1]^2 - тот же квадрат с центром в (0,0), единица по обеим
        // осям соответствует ПОЛОВИНЕ высоты гекса (R = spacing), не
        // половине ширины - ширина у pointy-top гекса меньше высоты
        // (sqrt(3)/2 ≈ 0.866 от неё).
        vec2 n = (gl_PointCoord - vec2(0.5)) * 2.0;

        // Точка внутри правильного шестиугольника (вершиной вверх/вниз,
        // circumradius = 1 в координатах n) тогда и только тогда, когда
        // одновременно: (1) не вылезает за плоские боковые грани
        // (x = ±sqrt(3)/2) и (2) не вылезает за две скошенные грани у
        // вершин - стандартный тест через симметрию (|x|,|y|).
        vec2 q = abs(n);
        const float kHalfWidthOverR = 0.8660254f;   // sqrt(3)/2
        const float kInvSqrt3 = 0.5773503f;         // 1/sqrt(3)
        // edgeDist - "запас" до ближайшей из двух границ; < 0 снаружи,
        // smoothstep сам обнулит альфу там без явного discard.
        float edgeDist = min(kHalfWidthOverR - q.x, 1.0 - (q.x * kInvSqrt3 + q.y));

        // При маленьком uCellSizePx (поле сжалось на экране до нескольких
        // пикселей на гекс) сама решётка становится МЕЛЬЧЕ пикселя экрана -
        // это подвыборка (aliasing) самой структуры, не проблема формы точки:
        // хоть гекс, хоть круг, периодическая решётка тоньше экранного
        // пикселя всегда даёт муар без честного даунсемплинга/LOD. Растягиваем
        // сглаживание НАСКОЛЬКО МОЖЕМ дешёвым способом - при uCellSizePx
        // >= 16px кромка честная и резкая (0.06), при <= 3px размываем почти
        // во всю фигуру (0.9) - смягчает муар, но не убирает его полностью.
        float aaWidth = mix(0.9, 0.06, clamp((uCellSizePx - 3.0) / 13.0, 0.0, 1.0));
        shapeAlpha = smoothstep(0.0, aaWidth, edgeDist);
    } else {
        vec2 d = gl_PointCoord - vec2(0.5);
        float dist = length(d) * 2.0;
        shapeAlpha = smoothstep(1.0, 0.0, dist);
    }

    // RGB уже посчитан на CPU вызывающей стороной - фрагментный шейдер
    // просто выводит готовый цвет с аддитивным блендингом.
    float t = clamp(max(vColor.r, max(vColor.g, vColor.b)), 0.0, 1.0);
    float alpha = shapeAlpha * (0.3 + t);
    FragColor = vec4(vColor * alpha, alpha);
}
