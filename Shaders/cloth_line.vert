// Shaders/spring_line.vert
#version 460 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in float aGlow;

uniform mat4 uCamera;

out float vGlow;

void main() {
    gl_Position = uCamera * vec4(aPos, 0.0, 1.0);
    vGlow = aGlow;
}
